import argparse

import paho.mqtt.client as mqtt

import sqlite3
import json
import os
import time
import argparse
import sys
import random
import signal
from datetime import datetime
from decouple import config
from random import random
import threading
import matplotlib.pyplot as plt


args = None
conn = None

class MQTTClient(mqtt.Client):#extend the paho client class
    run_flag=False #global flag used in multi loop
    def __init__(self,cname,**kwargs):
        super(MQTTClient, self).__init__(cname,**kwargs)
        self.data = {} #used to track subscribed topics and data
        self.topics = [] #used to track subscribed topics and data
        self.connected = False
        self.port = 1883
        self.keepalive = 60

    def subscribe(self, topic, qos=0, options=None, properties=None):
        if topic not in self.topics: self.topics.append(topic)
        return super().subscribe(topic, qos=qos, options=options, properties=properties)

def onDisconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected disconnection.")
        client.connected = False
        try:
            res = client.connect(client.broker, client.port)      #connect to broker
        except:
            print("connection to failed")
            sys.exit(0)
        else:
            print("Connection successfull")
            client.connected = True

def onSubscribe(client, userdata, mid, granted_qos):
    print(mid)
    print("Subscribed to: " + str(client.topics[mid-1]))
  
def onConnect(client, userdata, flags, rc):
    """
    set the bad connection flag for rc >0, Sets onnected_flag if connected ok
    also subscribes to topics
    """
    if rc == 0:
        client.connected = True
        print("successfully connected")
    else:
        client.connected = False 
        print("cannot connect")

def onMessage(client, userdata, msg):
    topic = msg.topic
    m_decode = str(msg.payload.decode("utf-8", "ignore"))
    handleMsg(client, topic, m_decode)

def handleMsg(client, topic, m_decode):
    if args.verbose:
        print("Topic: {}, Msg: {}".format(str(topic), str(m_decode)))
    try:
        powDic = json.loads(m_decode)
    except:
        print("no valid JSON")
    else:
        print(powDic)
        keys = ["power", "current", "energy", "volt"]
        for k in keys:
            if k not in powDic:
                print("missing key " + k)
        if not any(k not in powDic for k in keys):
            ts = int(time.time())
            if args.verbose: 
                print("diff {}s: timePC: {}, vs. Powermeter: {} ".format(abs(ts-powDic["ts"]), ts, powDic["ts"]))
            if "ts" in powDic:
                if abs(ts - powDic["ts"]) < 10:
                    #trust powrmeter time
                    ts = powDic["ts"]
            cmd =  "INSERT INTO POWER (TS,ACTIVE_POWER,ENERGY,VOLTAGE,CURRENT) " 
            cmd += "VALUES ({}, {:.2f}, {:.2f}, {:.2f}, {:.2f})".format(ts,
                            float(powDic["power"]), float(powDic["energy"]), float(powDic["volt"]), float(powDic["current"]))
            print(cmd)
            conn.execute(cmd);
            conn.commit()


def publish(topic, msg):
    if not client.connected: return
    client.publish(topic, msg)

  
def stop():
    client.loop_stop() 



def initParser():
    parser = argparse.ArgumentParser()
    parser = argparse.ArgumentParser(description="Get log of powermeters.\
                                                  use multiple \"-d <mdns or ip>\" to connect to individual devices or\
                                                  if you don't specify a device all devices are searched with mdns.")
    parser.add_argument("--host", type=str, default="127.0.0.1",
                        help="MQTT broker address")
    parser.add_argument("-p", "--port", type=int, default=1883,
                        help="MQTT port")
    parser.add_argument("-u", "--user", type=str, default=None,
                        help="MQTT User name")
    parser.add_argument("-x", "--pwd", type=str, default=None,
                        help="MQTT password")
    parser.add_argument("--initDB", action="store_true")
    parser.add_argument("-v", "--verbose", action="count", default=0,
                        help="Increase output verbosity")
    return parser
    

plt.ion()
class UpdatingPlot():

    def __init__(self, title=""):
        #Set up plot
        self.figure = None
        pass

    def initPlot(self, x, y):
        self.figure, self.ax = plt.subplots()
        self.lines = []
        for i in range(len(x)):
            self.lines.append(self.ax.plot([],[])[0])
        #Autoscale on unknown axis and known lims on the other
        self.ax.set_autoscaley_on(True)
        # self.ax.set_xlim(self.min_x, self.max_x)
        #Other stuff
        self.ax.grid()

    def update(self, xdata, ydata):
        if self.figure is None: self.initPlot(xdata, ydata)
        #Update data (with the new _and_ the old points)
        for l in self.lines:
            l.set_xdata(xdata)
            l.set_ydata(ydata)
        #Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
        plt.draw()
        plt.pause(0.2)


powerPlot = UpdatingPlot(title="power")
def plotIt():
    # threading.Timer(5.0, plotIt).start()
    print("Plotting")
    ts = time.time()
    maxTime = 3600
    cmd = "SELECT TS, ACTIVE_POWER, ENERGY, VOLTAGE, CURRENT from POWER WHERE TS BETWEEN {} AND {} ORDER BY TS".format(int(ts-maxTime), int(ts))
    
    connec = sqlite3.connect('powerData.db')
    rows = connec.execute(cmd)

    data = [r for r in rows]
    connec.close()
    # sort based on ts
    data = sorted(data, key=lambda x: x[0])
    ts = [t[0] for t in data]
    power = [t[1] for t in data]
    powerPlot.update(ts, power)

def mqtt_run():
    global conn
    conn = sqlite3.connect('powerData.db')
    client = MQTTClient("pythonPaho-"+str(random()), clean_session=True)
    print("Connecting...")
    client.on_connect = onConnect 
    client.on_disconnect = onDisconnect
    client.on_subscribe = onSubscribe
    client.on_message = onMessage
    
    try:
        client.connect(args.host, args.port, 60)
    except:
        print("connection to failed")
    else:
        print("Connection successfull")
        client.connected = True
    
    client.subscribe("powermeter/powermeter14/state/sample")
    # client.loop_start() 
    client.loop_forever()


    if args.user is not None and args.pwd is not None:
        client.tls_set()  # <--- even without arguments 
        client.username_pw_set(username=args.user, password=args.pwd)

running = True
# _______________Can be called as main__________________
if __name__ == '__main__':
    parser = initParser()
    args = parser.parse_args()


    myConn = sqlite3.connect('powerData.db')
    if args.initDB:
        
        print("Opened database successfully");

        conn.execute('''CREATE TABLE POWER
                       (TS INT PRIMARY KEY     NOT NULL,
                        ACTIVE_POWER   REAL    NOT NULL,
                        ENERGY         REAL    NOT NULL,
                        VOLTAGE        REAL    NOT NULL,
                        CURRENT        REAL    NOT NULL);''')
        print("Table created successfully");
    # Get external abort
    def aborted(signal, frame):
        global running
        running = False
        print("Closing db");
        conn.close()
        sys.exit(0)

    t = threading.Thread(target = mqtt_run, args = ())
    t.start()
    plt.show()
    signal.signal(signal.SIGINT, aborted)

    lastPlotUpdate = time.time()
    while running:
        if time.time()-lastPlotUpdate > 5:
            plotIt()
            lastPlotUpdate = time.time()
        time.sleep(1)
    # client = mqtt.Client(client_id="", clean_session=True, userdata=None, protocol=mqtt.MQTTv311, transport="tcp")
    # client.on_connect = on_connect
    # client.on_message = on_message
