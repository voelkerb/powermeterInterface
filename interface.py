import os
import sys
import time
import re
import argparse
from powermeter.powerMeter import PowerMeter, RGBColor
from powermeter.smartMeter import SmartMeter
from powermeter.smartDevice import LogLevel, VOLTAGE, CURRENT
import threading
# from powermeter.getDevices import *
import powermeter.getDevices as devGetter
import signal
import threading

running = True

REMOVE_LISTENER_TIME = 1.5
listener = 0
def removeListener():
    global listener
    if listener > 0: listener -= 1

def startListening():
    global listener
    listener += 1
    threading.Timer(REMOVE_LISTENER_TIME, removeListener).start()


def printRed(string, end="\n"):
    if not isinstance(string, str): string = str(string)
    print('\033[91m' + string + '\033[0m', end=end)

def printYellow(string, end="\n"):
    if not isinstance(string, str):string = str(string)
    print('\033[93m' + string + '\033[0m', end=end)

def printBlue(string, end="\n"):
    if not isinstance(string, str): string = str(string)
    print('\033[94m' + string + '\033[0m', end=end)

def printPink(string, end="\n"):
    if not isinstance(string, str): string = str(string)
    print('\033[95m' + string + '\033[0m', end=end)

def helpme():
    for cmd in cmds:
        cmdName = ""
        le = len(cmd["cmd"])
        for i, cmdi in enumerate(cmd["cmd"]):
            cmdName += cmdi
            if i < le-1: cmdName += ", "
        print("\t- {:<16}: {}".format(cmdName, cmd["info"]))

def abort():
    global running
    printBlue("Aborting...")
    running = False
    sys.exit(0)

def sample():
    printBlue("Type in samplingrate")
    stateK = int(input())
    if stateK >= 1 and stateK <= 8000:
        printBlue("Start Sampling with " + str(stateK) + " Hz")
        for ms in mss: 
            ms.samplingRate = stateK
            ms.start()
    else:
        printRed("Cannot sample with " + str(stateK) + " Hz")

def getEnergy():
    startListening()
    for ms in mss: 
        ms.getEnergy()

def getPower():
    startListening()
    for ms in mss: 
        ms.getPower()

def getVoltage():
    startListening()
    for ms in mss: 
        ms.getVoltage()

def getCurrent():
    startListening()
    for ms in mss: 
        ms.getCurrent()

def stop():
    printBlue("Stopping...")
    for ms in mss: ms.stop()

def switch():
    printBlue("Type in 1 (on) or 0 (off)")
    stateK = input()
    state = True
    if stateK == "0" or stateK == "off": state = False 
    printBlue("Switching " + "on" if state else "off")
    for ms in mss: ms.switch(state)
    
def restart():
    printBlue("Press any key to restart PowerMeter(s). Ctr-c to cancel")
    input()
    printBlue("Restarting...")
    for ms in mss: 
        print("\t" + ms.name)
        # This prevents that all devices log into wifi the same time again
        time.sleep(1.5)
        ms.restart()
        time.sleep(0.1)
        ms.kill()
    time.sleep(1.0)
    abort()

def basicReset():
    printBlue("Press any key to reset PowerMeter(s) settings, NOT the name. Ctr-c to cancel")
    input()
    printBlue("Restarting...")
    for ms in mss: 
        print("\t" + ms.name)
        time.sleep(1.5)
        # This prevents that all devices log into wifi the same time again
        ms.basicReset()
        time.sleep(0.1)
        ms.kill()
    time.sleep(1.0)
    abort()


def factoryReset():
    printBlue("Press any key to factory reset PowerMeter(s). Ctr-c to cancel")
    input()
    printBlue("Restarting...")
    for ms in mss: 
        print("\t" + ms.name)
        time.sleep(1.5)
        # This prevents that all devices log into wifi the same time again
        ms.factoryReset()
        time.sleep(0.1)
        ms.kill()
    time.sleep(1.0)
    abort()



def calibrate():
    printBlue("Set Calibration coefficients")
    
    for ms in mss:
        calibration = {}
        if ms.TYPE == SmartMeter.TYPE:
            for c in VOLTAGE[1:]+CURRENT[1:]: calibration[c] = 1.0
        elif ms.TYPE == PowerMeter.TYPE:
            for c in VOLTAGE[0]+CURRENT[0]: calibration[c] = 1.0
        printPink(ms.name + ":")
        printBlue("Required coefficients - " + str(list(calibration.keys())))
        for c in calibration.keys():
            printBlue(str("{}: ".format(c)), end="")
            d = input()
            while d:
                try:
                    value = float(d)
                except ValueError:
                    printRed("Must be float")
                    printBlue(str("{}: ".format(c)), end="")
                else:
                    if value >= 0.5 and value <= 2.0:
                        calibration[c] = float(value)
                        break
                    else:
                        printRed("Calibration should be > 0.5 < 2.0")
                        printBlue(str("{}: ".format(c)), end="")
        printPink("Set to: " + str(calibration))
        ms.calibrate(calibration)
        print("")
        

def logLevel():
    printBlue("Set Log level to ", end="")
    logLevels = [str(level) for level in list(LogLevel)]
    printBlue([str(i+1)+":"+str(level) for i,level in enumerate(list(logLevels))])
    c = input()
    logLevel = LogLevel.INFO
    while c:
        try:
            index = logLevels.index(c.lower())
            logLevel = list(LogLevel)[index]
            break   
        except ValueError:
            try:
                index = int(c)-1
            except ValueError:
                printRed("either type loglevel or integer")
                c=input()
                continue
            else:
                if index >= 0 and index <= len(list(LogLevel))-1:
                    logLevel = list(LogLevel)[index]
                    break
                else:
                    printRed("integer must be > 1 and < " + str(len(list(LogLevel))))

    printBlue("Set LogLevel to " + str(logLevel))
    for ms in mss: 
        ms.setLogLevel(logLevel)


def info():
    printBlue("Getting info about measurement system")
    for ms in mss: ms.systemInfo(sysInfo)
    
def mdns():
    printBlue("Type in new name + enter")
    c = input()
    printBlue("Setting to: " + str(c))
    for ms in mss: ms.setMDNS(c)

def ntp():    
    printBlue("Syncing NTP...")
    for ms in mss: ms.ntpSync()

def streamServer():
    printBlue("Type in stream server ip + enter")
    server = input()
    printBlue("StreamServer: " + str(server))
    for ms in mss: ms.setStreamServer(server)

def mqttServer():
    printBlue("Type in mqtt server ip + enter")
    server = input()
    printBlue("MqttServer: " + str(server))
    for ms in mss: ms.setMqttServer(server)

def timeServer():
    printBlue("Type in time server ip or dns + enter")
    server = input()
    printBlue("TimeServer: " + str(server))
    for ms in mss: ms.setTimeServer(server)

def addWifi():
    printBlue("Type in new ssid + enter")
    ssid = input()
    printBlue("Type in pwd of " + str(ssid) + " + enter")
    pwd = input()
    printBlue("SSID: " + str(ssid) + ", PWD: " + str(pwd))
    for ms in mss: ms.addWifi(ssid, pwd)
    
def delWifi():
    printBlue("Type in ssid to delete + enter")
    ssid = input()
    printBlue("Do you really want to remove AP SSID " + str(ssid) + " any key to continue, ctr-c to exit")
    input()
    for ms in mss: ms.delWifi(ssid)

def log():
    printBlue("Getting Log..")
    for ms in mss: ms.getLog(printLog)
    time.sleep(2)
    printBlue("Type yes to delete the log file(s). Enter to continue")
    delete = input()
    if delete == "yes" or delete == "Yes": 
        for ms in mss: ms.clearLog()

def getColor():
    while True:
        c=input()
        try:
            splits = c.split(",")
            if len(splits) != 3: 
                printRed("Invalid Color R,G,B (e.g. 255,0,0)")
                continue
            r = int(splits[0])
            g = int(splits[1])
            b = int(splits[2])
            color = RGBColor(r,g,b)
            return color
        except ValueError:
            printRed("Invalid Color R,G,B (e.g. 255,0,0)")
            continue
    return None

def getInputInteger(lowerBound=None, upperBound=None):
    c = input()
    index = -1
    while c:
        try:
            index = int(c)
            break   
        except ValueError:
            printRed("Type in valid integer")
            c=input()
            continue
        else:
            valid = True
            if lowerBound is not None and index < lowerBound: 
                printRed("integer must be > " + str(lowerBound))
                valid = False
            if upperBound is not None and index > upperBound: 
                printRed("integer must be < " + str(upperBound))
                valid = False
            if valid: break
    return index

def getInputFloat(lowerBound=None, upperBound=None):
    c = input()
    index = -1
    while c:
        try:
            index = float(c)
            break   
        except ValueError:
            printRed("Type in valid float")
            c=input()
            continue
        else:
            valid = True
            if lowerBound is not None and index < lowerBound: 
                printRed("float must be > " + str(lowerBound))
                valid = False
            if upperBound is not None and index > upperBound: 
                printRed("float must be < " + str(upperBound))
                valid = False
            if valid: break
    return index

def dailyReset():
    printBlue("Type the hour of day the reset should happen (24h format and -1 to disable)")
    hour = getInputInteger(-1, 23)
    printBlue("Type the minute of day the hour (-1 to disable)")
    minute = getInputInteger(-1, 60)
    for ms in mss: ms.setDailyReset(hour, minute)

def resetEnergy():
    printBlue("Are you sure you want to reset the Energy (yes/no)")
    yes = input()
    printBlue("Type value to reset to: ")
    value = getInputFloat(lowerBound=0.0)
    if yes.lower() == "yes": 
        for ms in mss: 
            if ms.TYPE == PowerMeter.TYPE:
                ms.resetEnergy(value=value)
            else:
                ms.resetEnergy()

def lora():
    printBlue("Type command to send to LoRa Device")
    cmd = input()
    for ms in mss: 
        if ms.TYPE == PowerMeter.TYPE:
            ms.loRaCommand(cmd)


def pir():
    startListening()
    for ms in mss: 
        if ms.TYPE == PowerMeter.TYPE and ms.hasSensorBoard():
            ms.getPIR()
def temp():
    startListening()
    for ms in mss: 
        if ms.TYPE == PowerMeter.TYPE and ms.hasSensorBoard():
            ms.getTemp()
def hum():
    startListening()
    for ms in mss: 
        if ms.TYPE == PowerMeter.TYPE and ms.hasSensorBoard():
            ms.getHum()
def light():
    startListening()
    for ms in mss: 
        if ms.TYPE == PowerMeter.TYPE and ms.hasSensorBoard():
            ms.getLight()
def sensors():
    startListening()
    for ms in mss: 
        if ms.TYPE == PowerMeter.TYPE and ms.hasSensorBoard():
            ms.getSensors()
def sensorsInfo():
    startListening()
    for ms in mss: 
        if ms.TYPE == PowerMeter.TYPE and ms.hasSensorBoard():
            ms.getSensorBoardInfo()
def calTemp():
    printBlue("Type temp offset in °C [-10.0,10.0]")
    offset = getInputFloat(-10,10)
    for ms in mss: 
        if ms.TYPE == PowerMeter.TYPE and ms.hasSensorBoard():
            ms.calibrateTempSensor(offset)
def calHum():
    printBlue("Type humidity offset in %% between [-30.0,30.0]")
    offset = getInputFloat(-30,30)
    for ms in mss: 
        if ms.TYPE == PowerMeter.TYPE and ms.hasSensorBoard():
            ms.calibrateHumSensor(offset)
def calLight():
    printBlue("Type light multiplier [0.0,10.0]")
    value = getInputFloat(0.0,10.0)
    for ms in mss: 
        if ms.TYPE == PowerMeter.TYPE and ms.hasSensorBoard():
            ms.calibrateLightSensor(value)
def powerInd():
    printBlue("Type lower power bound in Watt")
    minV = getInputFloat(0.0,10000.0)
    printBlue("Type upper power bound in Watt")
    maxV = getInputFloat(0.0,10000.0)
    for ms in mss: 
        if ms.TYPE == PowerMeter.TYPE and ms.hasSensorBoard():
            ms.powerIndication(minV,maxV)
def ledBright():
    startListening()
    printBlue("Type LED brightness in %% from 0.0 -> 100.0")
    brightness = getInputFloat(0.0,1.0)
    for ms in mss: 
        if ms.TYPE == PowerMeter.TYPE and ms.hasSensorBoard():
            ms.setLEDbrightness(brightness)
def setLEDs():
    printBlue("Type pattern [0,4]")
    pattern = getInputInteger(0,4)
    printBlue("Type duration in ms (-1 for infty)")
    duration = getInputInteger(lowerBound=-1)
    printBlue("Type foreground RGB Color as \"R,G,B\"")
    fgColor = getColor()
    if fgColor is None: return
    printBlue("Type background RGB Color as \"R,G,B\"")
    bgColor = getColor()
    if bgColor is None: return
    for ms in mss: 
        if ms.TYPE == PowerMeter.TYPE and ms.hasSensorBoard():
            ms.setLEDs(pattern,duration,fgColor,bgColor)

# Has been removed
# def verbosi():
#     printBlue("Changing verbosity")
#     for ms in mss: ms.INFO_VERBOSE = not ms.INFO_VERBOSE


# NOTE: do not change lines for documentation 
cmds = [
        {"func":helpme,       "cmd":["help", "h"],             "info":"Display this help message."},
        {"func":abort,        "cmd":["c"],                     "info":"Abort, stop this program."},
        {"func":sample,       "cmd":["sample"],                "info":"Start sampling at a given samplingrate."},
        {"func":getEnergy,    "cmd":["energy", "e"],           "info":"Get energy since last reset."},
        {"func":getPower,     "cmd":["power", "p"],            "info":"Get active, reactive and apparent power."},
        {"func":getVoltage,   "cmd":["voltage", "volt"],       "info":"Get RMS voltage in V."},
        {"func":getCurrent,   "cmd":["current", "cur"],        "info":"Get RMS current in mA."},
        {"func":stop,         "cmd":["stop", "s"],             "info":"Stop sampling."},
        {"func":switch,       "cmd":["switch"],                "info":"Switch relay on or off."},
        {"func":restart,      "cmd":["restart", "r"],          "info":"Restart the device(s)."},
        {"func":factoryReset, "cmd":["factoryReset", "r!"],    "info":"Reset the device to factory settings."},
        {"func":basicReset,   "cmd":["basicReset", "b!"],      "info":"Reset settings og device, not the name."},
        {"func":info,         "cmd":["info", "?", "i"],        "info":"Give info about device(s)."},
        {"func":mdns,         "cmd":["mdns", "m"],             "info":"Set new MDNS name."},
        {"func":ntp,          "cmd":["ntp", "n"],              "info":"Start NTP synchronisation."},
        {"func":streamServer, "cmd":["streamServer", "ss"],    "info":"Add a stream server."},
        {"func":mqttServer,   "cmd":["mqttServer", "mqtt"],    "info":"Add a mqtt server address."},
        {"func":timeServer,   "cmd":["timeServer", "t"],       "info":"Add a time server."},
        {"func":addWifi,      "cmd":["addWifi", "aw"],         "info":"Add a wifi AP."},
        {"func":delWifi,      "cmd":["delWifi", "dw"],         "info":"Delete a wifi AP."},
        {"func":log,          "cmd":["log", "l"],              "info":"Display and delete Log messages."},
        {"func":logLevel,     "cmd":["logLevel", "ll"],        "info":"Change device log level."},
        {"func":calibrate,    "cmd":["calibrate", "cal"],      "info":"Set calibration coefficients."},
        {"func":dailyReset,   "cmd":["dailyRestart", "dr"],    "info":"Set daily restart time."},
        {"func":resetEnergy,  "cmd":["resetEnergy", "e"],      "info":"Reset accumulated energy (kWh)"},
        {"func":lora,         "cmd":["lora", "LoRaWAN"],       "info":"Communicate with LoRaWAN module."},
        {"func":pir,          "cmd":["PIR", "getPIR"],         "info":"Read PIR sensor."},
        {"func":temp,         "cmd":["temp", "getTemp"],       "info":"Read temperature."},
        {"func":hum,          "cmd":["hum", "getHum"],         "info":"Read humidity level."},
        {"func":light,        "cmd":["light", "getLight"],     "info":"Read light sensor."},
        {"func":sensors,      "cmd":["sensors"],               "info":"Read all sensor."},
        {"func":sensorsInfo,  "cmd":["sensorsInfo","si"],      "info":"Get sensor board info."},
        {"func":calTemp,      "cmd":["calTemp","ct"],          "info":"Calibrate temperature sensor."},
        {"func":calHum,       "cmd":["calHum","ch"],           "info":"Calibrate humidity sensor."},
        {"func":calLight,     "cmd":["calLight","cl"],         "info":"Calibrate light sensor."},
        {"func":powerInd,     "cmd":["powerIndication","pi"],  "info":"Set power indication."},
        {"func":ledBright,    "cmd":["LEDbrightness","LEDb"],  "info":"Set LED brightness."},
        {"func":setLEDs,      "cmd":["setLED","LED"],          "info":"Set LEDs."},
        # {"func":verbosi,      "cmd":["verbose", "v"],          "info":"Change verbose output."}
    ]

def checkInput():
    """Check console input for stuff todo."""
    global running
    # press 'a' to abort the program
    # More convinient than using Ctrl-C
    print("Type 'help' to get list of available commands")
    while(running):
        # Wait for key press
        key = input()
        found = False
        for cmd in cmds:
            if any(x == key for x in cmd["cmd"]):
                cmd["func"]()
                found = True
                break
        if not found and key != "":
            print("Unknown command")

def sysInfo(device, dic):
    print("\n")
    printPink(device.name + ":")
    for key in dic:
        print("{0:<15}".format(str(" " + key + ": ")[:15]) + str(dic[key]))
    print("\n")

def printDict(device, dic):
    if listener == 0: return
    print("\n")
    printPink(device.name + ":")
    for key in dic:
        print("{0:<15}".format(str(" " + key + ": ")[:15]) + str(dic[key]))
    print("\n")

def printLog(device, loglist):
    printPink("\n" + device.name + ":")
    for msg in loglist:
        try:
            i = next(i for i, s in enumerate(device.LOGG_STUFF) if s["s"] in msg)
        except StopIteration:
            i = -1
        prefix, endFix = "",""
        if i != -1:
            prefix = device.LOGG_STUFF[i]["c"]
            msg = msg.replace(device.LOGG_STUFF[i]["s"], "").lstrip(" ")
        if prefix != "": endFix = "\033[0m"
        print(prefix + str(msg) + endFix)
    # for ms in mss:
    #     ms.start()

VERBOSE = False
WAITTIME = 30

def initParser():
    parser = argparse.ArgumentParser()
    parser = argparse.ArgumentParser(description="Get log of powermeters. Use multiple \"-d <mdns or ip>\" to connect to individual devices or \
                                                  if you don't specify a device all devices are searched with mdns.")
    devGetter.addDeviceCommandLineArgs(parser, deviceType=True, verify=True)
    parser.add_argument("-v", "--verbose", action="count", default=0,
                        help="Increase output verbosity")
    return parser
    
# _______________Can be called as main__________________
if __name__ == '__main__':
    parser = initParser() 
    args = parser.parse_args()

    
    deviceList = devGetter.getDevices(devArguments=args.deviceList, deviceTypes=args.type, verbose=args.verbose-2, verify=args.verifyType, searchMore=args.more)

    # Catch control+c
    # Get external abort
    def aborted(signal, frame):
        print("Bye Bye from " + str(os.path.basename(__file__)))
        sys.exit(0)
    signal.signal(signal.SIGINT, aborted)
    
    mss = [PowerMeter(updateInThread=False,
                      ip=dev["ip"], port=dev["port"], useUDP=False, portUDP=5323+i, #  port=54322, stream=True,
                      samplingRate=dev["sr"], measures="v,i".split(','),
                      name=dev["name"],
                      verbose=args.verbose>1)
                      for i, dev in enumerate(deviceList)]  
    for ms in mss:
        ms.setLogLevel(LogLevel.WARNING)
        ms.cmdHandler = printDict
    numDevices = len(mss)

    
    t1 = threading.Thread(target=checkInput, daemon=True)
    t1.start()

    ttime = time.time()
    while running:
        for ms in mss: ms.update()
        time.sleep(0.001)
    
    print("Bye Bye from " + str(os.path.basename(__file__)))
