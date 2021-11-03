"""Main File covering self build measurment system."""
# !/usr/bin/python
import sys
import os

import serial
import socket
import struct
import warnings
import time
import traceback
import json
import numpy as np
import threading
import select
import multiprocessing
from multiprocessing import Process, Value
import platform
# We need to add the path
from queue import Queue, Empty
import json
# Import top level module
try:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
except NameError:
    root = os.path.dirname(os.path.dirname(os.path.abspath(sys.argv[0])))
sys.path.append(root)
from powermeter.smartDevice import *



class SmartMeter(SmartDevice):
    """Class which handles a measurement system."""

    TYPE = "SmartMeter".lower()

    DEFAULT_SR = 8000

    AVAILABLE_MEASURES = [
            {"keys": [VOLTAGE[1], CURRENT[1], VOLTAGE[2], CURRENT[2], VOLTAGE[3], CURRENT[3]], "bytes": 24, "cmdMeasure": None},
            {"keys": [VOLTAGE[0], CURRENT[0]], "bytes": 8, "cmdMeasure": "v,i_L1"},
            {"keys": [VOLTAGE[0], CURRENT[0]], "bytes": 8, "cmdMeasure": "v,i_L2"},
            {"keys": [VOLTAGE[0], CURRENT[0]], "bytes": 8, "cmdMeasure": "v,i_L3"},
            {"keys": [VOLTAGE_RMS[1], VOLTAGE_RMS[2], VOLTAGE_RMS[3], CURRENT_RMS[1], CURRENT_RMS[2], CURRENT_RMS[3]], "bytes": 24, "cmdMeasure": "v,i_RMS"},
            {"keys": [ACTIVE_POWER[1], ACTIVE_POWER[2], ACTIVE_POWER[3], REACTIVE_POWER[1], REACTIVE_POWER[2], REACTIVE_POWER[3]], "bytes": 8, "cmdMeasure": "p,q"},
        ]


    # Init function with default values
    def __init__(self, name=None, deviceName=None, samplingRate=None, phase=None, measures=None,
                       serialPort=None, portName=None, baudrate=2000000,
                       socketObject=None, ip=None, port=54321, stream=False,
                       useUDP=False, portUDP=54323,
                       updateInThread=False, directInit=True,
                       flowControl=False,
                       logLevel=LogLevel.INFO, verbose=0, logger=print):
        """
        Init the measurement module.

        :param serialPort: Sertial port
        :type  serialPort: :class: ComSerial or TCPSerial
        :param frameReadyCallback:
            Callback function, that needs an index as input paramter
        :type  frameReadyCallback: function
        :param windowReadyCallback:
            Callback function, that needs an index as input paramter
        :type  windowReadyCallback: function
        :param tcpSocat:
            Initializes a TCP serailport instead of a COM serialport
        :type  tcpSocat: bool
        """


        # Pass everything required to superclass
        super().__init__(   name=name, deviceName=deviceName, samplingRate=samplingRate, 
                            serialPort=serialPort, portName=portName, baudrate=baudrate,
                            socketObject=socketObject, ip=ip, port=port, stream=stream,
                            useUDP=useUDP, portUDP=portUDP,
                            updateInThread=updateInThread, directInit=directInit,
                            flowControl=flowControl,
                            logLevel=logLevel, verbose=verbose, logger=logger)

        if samplingRate == None: self.samplingRate = self.DEFAULT_SR
       
        self.measurementInfo = self.AVAILABLE_MEASURES[0]
        # Look what measures we need
        self.phase = phase
        if phase is not None:
            if phase >= 1 and phase <=3:
                self.measurementInfo = self.AVAILABLE_MEASURES[phase]
            else:
                sys.exit("Error cannot use phase " + str(phase) + " with smartmeter")

            # Look what measures we need
        if measures is not None:
            self.measurementInfo = None
            for availableMeasureSet in self.AVAILABLE_MEASURES:
                if all(item in availableMeasureSet["keys"] for item in measures): 
                    self.measurementInfo = availableMeasureSet
                    break
            if self.measurementInfo is None: 
                sys.exit("Smartmeter cannot handle mesurements: " + str(measures)) 

        self.MEASUREMENTS = self.measurementInfo["keys"]
        self.MEASUREMENT_BYTES = self.measurementInfo["bytes"]
     
    def defaultSettings(self):
        """
        Set default values to this smartmeter.
        """
        self.samplingRate = self.DEFAULT_SR
        self.phase = None
        self.measurementInfo = self.AVAILABLE_MEASURES[0]
        self.MEASUREMENTS = self.measurementInfo["keys"]
        self.MEASUREMENT_BYTES = self.measurementInfo["bytes"]

    def resetEnergy(self):
        """
        Set dely reset for the device.
        
        :param command:   LoRaWAN AT Command
        :type  command:   str
        """
        if not self.inited: self._notInitedError
        self.sendFunc(str("{\"cmd\":\"resetEnergy\"}"))
        
    def calibrate(self, parameter):
        r"""
        Set calibration coefficients.

        :param parameter: dictionary with calibration parameter as \{'v_l1':0.99,'i_l1':1.01 ... \}
        :type  parameter: dict
        """
        if all([vi in parameter for vi in VOLTAGE[1:]+CURRENT[1:]]):
            calDict = {"cmd":"calibration","values":[]}
            for key in [VOLTAGE[1], CURRENT[1], VOLTAGE[2], CURRENT[2], VOLTAGE[3], CURRENT[3]]: calDict["values"] = parameter[key]
            self.sendFunc(json.dumps(calDict)) 
        else:
            self.msPrint("Cannot use given parameters to calibrate")
    # def convertfromSmartDevice(self):
    #     # Standard measures
    #     # Look what measures we need
    #     self.phase = None
    #     self.measurementInfo = self.AVAILABLE_MEASURES[0]
    #     self.MEASUREMENTS = self.measurementInfo["keys"]
    #     self.MEASUREMENT_BYTES = self.measurementInfo["bytes"]


    # def serialEnqueue(self, out, queueData, queueLine):
    #     buffer = b""
    #     repair = 0
    #     for line in iter(out.readline, b''):
    #         if self.LINE_PREFIX in line:
    #             queueLine.put(line[:-2])
    #         elif len(line) == self.MEASUREMENT_BYTES+2:
    #             queueData.put(line[:-2])
    #         # Case \r\n in data
    #         else:
    #             # if len(buffer) == 0: self.msPrint("[W]Startin buffer repair: " + str(line))
    #             buffer += line
    #             if len(buffer) == self.MEASUREMENT_BYTES+2:
    #                 queueData.put(buffer[:-2])
    #                 buffer = b""
    #                 # self.msPrint("[W]Success repair: " + str(line))
    #             else:
    #                 repair += 1
                    
    #             if repair > 1 and len(buffer) > self.MEASUREMENT_BYTES+2:
    #                 self.msPrint("[E]Could not repair measurment: " + str(len(buffer)) + " :" + str(buffer))
    #                 buffer = b""
    #                 repair = 0
    #     out.close()

    # def __updateSerial(self):
    #     # if not self.sampling: return

    #     # read line without blocking
    #     try:  
    #         line = self.__serialQueueLine.get_nowait() # or q.get(timeout=.1)
    #     except Empty:
    #         pass
    #     else: 
    #         self._handleLine(line)
    #     try:  
    #         data = self.__serialQueueData.get_nowait() # or q.get(timeout=.1)
    #     except Empty:
    #         pass
    #     else: 
    #         self.samplesReceived += 1;
    #         self.__handleData(data)
     





# aliasing baudrates in kernel driver mac since baudrates over 3Mbaud are not supported by default
baudrateMapping = {4000000: 300, 8000000: 600, 12000000: 1200}




def initParser():
    import argparse
    parser = argparse.ArgumentParser(description="Records data from smartmeter.\
                                                  Can plot it or write it to mkv file. Serial or TCP/UDP support.\
                                                  Choose between sampling rates and measures.")
    parser.add_argument("--host", type=str,
                        help="Hostname or IP address of device e.g. smartmeter001.local")
    parser.add_argument("--port", type=int, default=54321,
                        help="Port, default: 54321.")
    parser.add_argument("--phase", type=int, choices=[None, 1, 2, 3], default=None,
                        help="Phase to measure. A Smartmeter is typically connected to L1, L2 and L3")
    parser.add_argument("--measures", type=str, choices=[",".join(measure['keys']) for measure in SmartMeter.AVAILABLE_MEASURES],
                        default=",".join(SmartMeter.AVAILABLE_MEASURES[0]['keys']),
                        help="Measures to use for recording/plotting \
                              (default: " + str(",".join(SmartMeter.AVAILABLE_MEASURES[0]['keys'])) + ")")
    parser.add_argument("--samplingrate", type=int, default=4000,
                        help="Samplingrate of the powermeters, default: 4000")
    parser.add_argument("--serial", type=str,
                        help="SerialPort path. COMX under windows, /dev/ttyXXX under Unix systems.")
    parser.add_argument("--baudrate", type=int, default=2000000,
                        help="Baudrate of serialport")
    parser.add_argument("--udp", action="store_true",
                        help="If UDP should be used to send data")
    parser.add_argument("--plot", action="store_true",
                        help="If data should be plotted")
    parser.add_argument("--ffmpeg", action="store_true",
                        help="If data should be written to mkv file. You can specify filename with --filename")
    parser.add_argument("--filename", type=str,
                        help="If ffmpeg is active, it will be stored under this filename")
    parser.add_argument("-v", "--verbose", action="count", default=0,
                        help="Increase output verbosity")
    return parser


# _______________Can be called as main__________________
if __name__ == '__main__':
    import time
    import signal
    import numpy as np
    from pyqtgraph.Qt import QtGui, QtCore
    import pyqtgraph as pg
    import threading
    import subprocess
    from queue import Queue
    parser = initParser()
    args = parser.parse_args()

    if args.baudrate in baudrateMapping:
        args.baudrate = baudrateMapping[args.baudrate]

    # Check for valid arguments passed here
    validConfig = False
    if args.host is not None and args.port is not None: validConfig = True
    if args.serial is not None and args.baudrate is not None: validConfig = True
    if not validConfig: sys.exit("Either provide ip + port or serial + baudrate")

    # device connection
    ms = SmartMeter(ip=args.host, port=args.port, useUDP=args.udp,
                    portName=args.serial, baudrate=args.baudrate, measures=args.measures.split(','),
                    verbose=args.verbose, updateInThread=False, name="smartmeter",
                    samplingRate=args.samplingrate, phase=args.phase)
    ms.frameSize = int(ms.samplingRate/50)

    # Get system info
    ms.systemInfo()
    now = time.time()
    while time.time()-now < 2 and ms.deviceInfo is None: 
        ms.update()
        time.sleep(0.1)

    ffmpegProc = None
    # Update linked views for plotting
    linkedPlots = []
    def updateViews():
        global linkedPlots
        for leftAxisPlot, rightAxisPlot in linkedPlots:
            rightAxisPlot.setGeometry(leftAxisPlot.getViewBox().sceneBoundingRect())
            rightAxisPlot.linkedViewChanged(leftAxisPlot.getViewBox(), rightAxisPlot.XAxis)

    # Mapping holds plots and data
    mapping = {VOLTAGE[0]: {"active": False, "plot": None, "curve": None, "pen":"r", "data": []},
               VOLTAGE[1]: {"active": False, "plot": None, "curve": None, "pen":"r", "data": []},
               VOLTAGE[2]: {"active": False, "plot": None, "curve": None, "pen":"r", "data": []},
               VOLTAGE[3]: {"active": False, "plot": None, "curve": None, "pen":"r", "data": []},
               CURRENT[0]:  {"active": False, "plot": None, "curve": None, "pen":"b", "data": []},
               CURRENT[1]:  {"active": False, "plot": None, "curve": None, "pen":"b", "data": []},
               CURRENT[2]:  {"active": False, "plot": None, "curve": None, "pen":"b", "data": []},
               CURRENT[3]:  {"active": False, "plot": None, "curve": None, "pen":"b", "data": []},
               VOLTAGE_RMS[0]:  {"active": False, "plot": None, "curve": None, "pen":"r", "data": []},
               VOLTAGE_RMS[1]:  {"active": False, "plot": None, "curve": None, "pen":"r", "data": []},
               VOLTAGE_RMS[2]:  {"active": False, "plot": None, "curve": None, "pen":"r", "data": []},
               VOLTAGE_RMS[3]:  {"active": False, "plot": None, "curve": None, "pen":"r", "data": []},
               CURRENT_RMS[0]:  {"active": False, "plot": None, "curve": None, "pen":"b", "data": []},
               CURRENT_RMS[1]:  {"active": False, "plot": None, "curve": None, "pen":"b", "data": []},
               CURRENT_RMS[2]:  {"active": False, "plot": None, "curve": None, "pen":"b", "data": []},
               CURRENT_RMS[3]:  {"active": False, "plot": None, "curve": None, "pen":"b", "data": []},
               ACTIVE_POWER[0]:  {"active": False, "plot": None, "curve": None, "pen":"r", "data": []},
               ACTIVE_POWER[1]:  {"active": False, "plot": None, "curve": None, "pen":"r", "data": []},
               ACTIVE_POWER[2]:  {"active": False, "plot": None, "curve": None, "pen":"r", "data": []},
               ACTIVE_POWER[3]:  {"active": False, "plot": None, "curve": None, "pen":"r", "data": []},
               REACTIVE_POWER[0]:  {"active": False, "plot": None, "curve": None, "pen":"b", "data": []},
               REACTIVE_POWER[1]:  {"active": False, "plot": None, "curve": None, "pen":"b", "data": []},
               REACTIVE_POWER[2]:  {"active": False, "plot": None, "curve": None, "pen":"b", "data": []},
               REACTIVE_POWER[3]:  {"active": False, "plot": None, "curve": None, "pen":"b", "data": []},}

    # Init plot depending on selected measures
    def initPlot():
        global linkedPlots, mapping
        # Enable antialiasing for prettier plots
        # pg.setConfigOptions(antialias=True)
        pg.setConfigOptions(antialias=False)

        # Get measures to display
        for key in mapping:
            if key in ms.MEASUREMENTS: mapping[key]["active"] = True

        for c in [0,1,2,3]:
            if not (mapping[ACTIVE_POWER[c]]["active"] or mapping[REACTIVE_POWER[c]]["active"]): continue
            p_c_0 = win.addPlot()
            p_c_0.getViewBox().setMouseEnabled(x=False, y=False)
            p_c_0.setLabel('left', "Power [W]")
            p_c_0.setLabel('right', "Power [VAR]")
            mapping[ACTIVE_POWER[c]]["plot"] = p_c_0
            mapping[REACTIVE_POWER[c]]["plot"] = p_c_0
            win.nextRow()

        for c in [0,1,2,3]:
            if not (mapping[VOLTAGE[c]]["active"] or mapping[CURRENT[c]]["active"]): continue
            p_c_1 = win.addPlot()
            p_c_1.getViewBox().setMouseEnabled(x=False, y=False)
            p_c_1.setLabel('left', "Voltage [V]")
            p_c_1.setLabel('right', "Current [mA]")
            mapping[VOLTAGE[c]]["plot"] = p_c_1

            p_c_2 = pg.ViewBox()
            p_c_1.scene().addItem(p_c_2)
            p_c_1.getAxis('right').linkToView(p_c_2)
            p_c_2.setXLink(p_c_1)
            mapping[CURRENT[c]]["plot"] = p_c_2
            linkedPlots.append((p_c_1, p_c_2))
            win.nextRow()

        for c in [0,1,2,3]:
            if not (mapping[VOLTAGE_RMS[c]]["active"] or mapping[CURRENT_RMS[c]]["active"]): continue
            p_c_1 = win.addPlot()
            p_c_1.getViewBox().setMouseEnabled(x=False, y=False)
            p_c_1.setLabel('left', "RMS Voltage [V]")
            p_c_1.setLabel('right', "RMS Current [mA]")
            mapping[VOLTAGE_RMS[c]]["plot"] = p_c_1

            p_c_2 = pg.ViewBox()
            p_c_1.scene().addItem(p_c_2)
            p_c_1.getAxis('right').linkToView(p_c_2)
            p_c_2.setXLink(p_c_1)
            mapping[CURRENT_RMS[c]]["plot"] = p_c_2
            linkedPlots.append((p_c_1, p_c_2))
            win.nextRow()

        updateViews()
        for leftAxisPlot, rightAxisPlot in linkedPlots:
            leftAxisPlot.getViewBox().sigResized.connect(updateViews)

        for leftAxisPlot, rightAxisPlot in linkedPlots:
            if leftAxisPlot is not None:
                leftAxisPlot.showAxis('right')

        # add curves that hold the data
        for key in mapping:
            if mapping[key]["active"]:
                curve = pg.PlotCurveItem(pen=mapping[key]["pen"])
                # curves.append(curve)
                mapping[key]["curve"] = curve
                mapping[key]["plot"].addItem(curve)

    # Update liva plot with seconds counter
    start = 0
    theData = None
    def updatePlot():
        global theData, curves, start, running, plotQueue
        while not plotQueue.empty():
            frame = plotQueue.get()
            plotQueue.task_done()
            newFrameLen = len(frame)
            if theData is None: theData = frame
            else: theData = np.concatenate([theData, frame], axis=0)
            theData = theData[-maxPoints:]
            for key in ms.MEASUREMENTS: mapping[key]["data"] = theData[key]
            dlen = len(theData)
            x = np.linspace(start, start + dlen/ms.samplingRate, dlen)
            start += float(newFrameLen/ms.samplingRate)
            x = x[0:dlen]
            for key in ms.MEASUREMENTS:
                mapping[key]["curve"].setData(x, mapping[key]["data"][0:len(x)])


    #b Construct the ffmpeg call for a single device
    def constructFFMPEGCall(ms, name=None):
        systemCall = "ffmpeg -hide_banner -f f32le -ar " + str(ms.samplingRate) + " -guess_layout_max 0 -ac "
        systemCall += str(len(ms.MEASUREMENTS)) + " -i pipe:0"
        cmdMeasure = next(entry["cmdMeasure"] for i, entry in enumerate(ms.AVAILABLE_MEASURES) if entry["keys"] == ms.MEASUREMENTS)
        streams = {}
        if cmdMeasure == None:
            # [v,i,v,i,v,i] -> [v,i],[v,i],[v,i] 
            systemCall += " -map 0 -map_channel 0.0.0:0.0.0 -map_channel 0.0.1:0.0.1"
            systemCall += " -map 0 -map_channel 0.0.2:0.1.0 -map_channel 0.0.3:0.1.1"
            systemCall += " -map 0 -map_channel 0.0.4:0.2.0 -map_channel 0.0.5:0.2.1"
            for i in range(3):
                streams[i] = {
                        "title": "\"{} L{}\"".format(ms.name, i+1),
                        "CHANNELS": 2,
                        "CHANNEL_TAGS": "\"{},{}\"".format(VOLTAGE[0], CURRENT[0]),
                    }
        elif cmdMeasure in ["v,i_L1", "v,i_L2", "v,i_L3"]:
            # v,i
            c = int(cmdMeasure.split("_")[-1])
            streams[0] = {
                    "title": "\"{} {}\"".format(ms.name, c),
                    "CHANNELS": 2,
                    "CHANNEL_TAGS": "\"{},{}\"".format(VOLTAGE[0], CURRENT[0]),
                }
        elif cmdMeasure == "v,i_RMS":
            # v,v,v,i,i,i
            systemCall += " -map 0 -map_channel 0.0.0:0.0.0 -map_channel 0.0.3:0.0.1"
            systemCall += " -map 0 -map_channel 0.0.1:0.1.0 -map_channel 0.0.4:0.1.1"
            systemCall += " -map 0 -map_channel 0.0.2:0.2.0 -map_channel 0.0.5:0.2.1"
            for i in range(3):
                streams[i] = {
                        "title": "\"{} L{}\"".format(ms.name, i+1),
                        "CHANNELS": 2,
                        "CHANNEL_TAGS": "\"{},{}\"".format(VOLTAGE_RMS[0], CURRENT_RMS[0]),
                    }
        elif cmdMeasure == "p,q":
            # p,p,p,q,q,q
            systemCall += " -map 0 -map_channel 0.0.0:0.0.0 -map_channel 0.0.3:0.0.1"
            systemCall += " -map 0 -map_channel 0.0.1:0.1.0 -map_channel 0.0.4:0.1.1"
            systemCall += " -map 0 -map_channel 0.0.2:0.2.0 -map_channel 0.0.5:0.2.1"
            for i in range(3):
                streams[i] = {
                        "title": "\"{} L{}\"".format(ms.name, i+1),
                        "CHANNELS": 2,
                        "CHANNEL_TAGS": "\"{},{}\"".format(ACTIVE_POWER[0], REACTIVE_POWER[0]),
                    }
        TS = ms.getStartTs()
        if TS is None: TS = time.time()
        for s in streams:
            for key in streams[s]:
                systemCall += " -metadata:s:a:{} {}={}".format(s, key, streams[s][key])
            systemCall += " -metadata:s:a:{} TIMESTAMP={}".format(s, TS)
        systemCall += " -c:a wavpack -y "
        if name is not None: systemCall +=  name.rstrip(".mkv") + ".mkv"
        else: systemCall += ms.name + ".mkv"
        print(systemCall)
        # sys.exit()
        return systemCall

    # Update the measurement system
    def updateMs():
        global running, ms, plotQueue, ffmpegProc
        # Init ffmpeg
        if args.ffmpeg:
            FNULL = open(os.devnull, 'w')
            ffmpegCall = constructFFMPEGCall(ms, name=args.filename)
            ffmpegProc = subprocess.Popen(ffmpegCall, shell=True, stdin=subprocess.PIPE, preexec_fn=os.setsid)

        while running:
            # Update ms
            ms.update()
            # on every new frame
            while len(ms.frames) > 0:
                # Update ffmpeg
                if args.ffmpeg:
                    ffmpegProc.stdin.write(ms.frames[0].transpose().view(np.float32).reshape(ms.frames[0].shape + (-1,)).flatten().tobytes())
                # update plot using Queue
                if plotQueue is not None: plotQueue.put(ms.frames[0])
                # remove frame
                del ms.frames[0]
            time.sleep(0.0001)

    # global plot variables required
    if args.plot:
        # Setup main GUI
        app = QtGui.QApplication([])
        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)

        # basic config
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        win = pg.GraphicsWindow(title="Live Data")
        win.resize(1000, 600)
        # define plot size depending on sr
        maxTime = 0.1
        if ms.samplingRate <= 8000: maxTime = 0.2
        if ms.samplingRate <= 4000: maxTime = 0.5
        if ms.samplingRate <= 2000: maxTime = 1.0
        if ms.samplingRate <= 1000: maxTime = 2.0
        if ms.samplingRate <= 100: maxTime = 10.0
        if ms.samplingRate <= 10: maxTime = 20.0
        maxPoints = int(ms.samplingRate*maxTime)

    # Catch control+c
    running = True
    # Get external abort
    def aborted(signal, frame):
        global running
        running = False
        if args.plot: app.quit()
    signal.signal(signal.SIGINT, aborted)


    plotQueue = None
    if args.plot: plotQueue = Queue(maxsize=0)

    thread = threading.Thread(target=updateMs)
    thread.start()

    ms.start()

    # Plotting stuff
    if args.plot:
        initPlot()
        timer = QtCore.QTimer()
        timer.timeout.connect(updatePlot)
        # Update with 50Hz
        timer.start(20)
        QtGui.QApplication.instance().exec_()
        running = False

    # Waits for update thread to complete
    thread.join()

    ms.stop()

    time.sleep(1)
    ms.kill()

    # on every new frame
    if args.ffmpeg:
        while len(ms.frames) > 0:
            # Update ffmpeg
            ffmpegProc.stdin.write(ms.frames[0].transpose().view(np.float32).reshape(ms.frames[0].shape + (-1,)).flatten().tobytes())
            del ms.frames[0]
        ffmpegProc.stdin.close()
        

    print(ms.samplingInfo())

    print("Bye Bye from " + str(os.path.basename(__file__)))
