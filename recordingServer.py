
import os
import sys
import time

import argparse
import signal
from powermeter.powerMeter import PowerMeter
# from measurement.networkMeter import NetworkMeter
from powermeter.smartMeter import SmartMeter
from powermeter.smartDevice import LogLevel, SmartDevice
from network.network import NetServer
import schedule
import powermeter.getDevices as devGetter
import os
import numpy as np
import threading
import signal
import json
import subprocess
import threading
from datetime import datetime
import logging
# For writing emails and stuff
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
# To copy over files
import shutil


class bcolors:
    """Enumeration class for escape characters in different colors"""
    HEADER = '\033[95m'
    OK = '\033[94m'
    OK2 = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def time_format_ymdhms(dt):
    """
    Return time format as y.m.d h:m:s.

    :param dt: The timestamp or date to convert to string
    :type  dt: datetime object or timestamp

    :return: Timeformat as \"y.m.d h:m:s\"
    :rtype: str
    """
    if dt is None:
        return "UUPs its (None)"
    import datetime
    if (isinstance(dt, datetime.datetime) is False
            and isinstance(dt, datetime.timedelta) is False):
        dt = datetime.datetime.fromtimestamp(dt)
    return "%s.%s" % (
        dt.strftime('%Y.%m.%d %H:%M:%S'),
        str("%03i" % (int(dt.microsecond/1000)))
    )

# Standard values
# TODO: export this into a file not in git?
smtpDict = {"address":"","user":"","pwd":"","smtp_server":"","smtp_port":587}

STORE_TIME = {
        "1_min": {"sec":   60},
        "5_min": {"sec": 5*60},
        "10_min":{"sec":10*60},
        "20_min":{"sec":20*60},
        "30_min":{"sec":30*60},
        "1_hour":{"sec":60*60},
        "2_hour":{"sec":60*60},
    }
STORE_TIME_FORMAT = '%Y_%m_%d__%H_%M_%S'

#  Construct all ffmpeg calls
FNULL = open(os.devnull, 'w')

running = True

withTS = False
deviceDict = {}
newMss = []
logger = None

REGARD_AS_DEAD_TIME = 100


def setupLogger(name, toConsole=False, filePath=None, clear=False, timeFormat='%Y-%m-%d %H:%M:%S'):
    formatter = logging.Formatter(fmt='%(asctime)s: %(message)s', datefmt=timeFormat)
    logger = logging.getLogger(name)
    if filePath is not None:
        if clear:
            handler = logging.FileHandler(filePath, mode='w')
        else:
            handler = logging.FileHandler(filePath, mode='a')
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    if toConsole:
        # Would also log to std out
        screen_handler = logging.StreamHandler(stream=sys.stdout)
        screen_handler.setFormatter(formatter)
        logger.addHandler(screen_handler)
    logger.setLevel(logging.DEBUG)
    return logger

emails = []
def scheduleEmail(msg, subject="SmartEnergy"):
    """
    Write an email if a target address has been set.

    :param msg: Message body to send 
    :type  msg: str
    :param subject: Message subject to send 
    :type  subject: str, default="SmartEnergy"

    :return: ``True`` if message was sent, ``False`` if not.
    :rtype : bool
    """
    global emails
    emails.append({"msg":msg, "subject":subject})

def emailCollectThread():
    global emails

    while running:
        time.sleep(100)
        if len(emails) > 0:
            myemails = emails
            emails = []
            subject = myemails[0]["subject"]
            msg = ""
            for email in myemails:
                msg += email["msg"] + "\n"
            sendEmail(args.warningEmail, msg, subject=subject)


def sendEmail(receiver, message, subject=""):
    """
    Send an email to an address.
    
    Constants :attr:`SMTP_ADDRESS<recording.recordingServer.SMTP_ADDRESS>`, 
    :attr:`SMTP_PORT<recording.recordingServer.SMTP_PORT>`, 
    :attr:`MAIL_ADDRESS<recording.recordingServer.MAIL_ADDRESS>` 
    and :attr:`MAIL_PASSWORD<recording.recordingServer.MAIL_PASSWORD>` 
    need to be set beforehand

    :param receiver: receiver email address 
    :type  receiver: str
    :param msg: Message body to send 
    :type  msg: str
    :param subject: Message subject to send 
    :type  subject: str, default=""
    """
    try:
        if "ssl" in smtpDict and smtpDict["ssl"]:
            s = smtplib.SMTP_SSL(host=smtpDict["smtp_server"])
        else:
            s = smtplib.SMTP(host=smtpDict["smtp_server"], port=smtpDict["smtp_port"])
            s.starttls()
        s.login(smtpDict["user"], smtpDict["pwd"])
        msg = MIMEMultipart()

        # setup the parameters of the message
        msg['From'] = smtpDict["address"]
        msg['To'] = receiver
        msg['Subject'] = subject
        
        # add in the message body
        msg.attach(MIMEText(message, 'plain'))
        # send the message via the server set up earlier.
        s.send_message(msg)
        s.quit()
        del msg
    except:
        logError("Cannot send mail")
        pass

#b Construct the ffmpeg call for a single device
def constructFFMPEGCall(ms, name=None, ts=None, path=None, filename=None, subfolder=None):
    """
    Create a call to ffmpeg.

    :param        ms: Powermeter or Smartmeter
    :type         ms: SmartDevice
    :param      name: Name to be used for metadata stream title. if ``None``, ``ms.name`` is used
    :type       name: str, default: ``None``
    :param        ts: Time that should be added to filename, default: no time added
    :type         ts: timestamp, default: ``None``
    :param      path: Path where file should be stored. If ``None`` currentpath is used.
    :type       path: str, default: ``None``
    :param  filename: Filename to use. Timestamp may be added. If ``None`` ms.deviceName or ms.name is used.
    :type   filename: str, default: ``None``
    :param subfolder: Path might be extended by a subfolder which is created if not existant. If ``None`` no subfolder is used.
    :type  subfolder: str, default: ``None``
    """
    meta = " -metadata:s:a:0"
    systemCall = "ffmpeg -hide_banner -f f32le -ar <samplingrate> -guess_layout_max 0 -ac <#channels> -i pipe:0 -c:a wavpack " 
    systemCall += meta + " CHANNELS=<#channels>" + meta + " CHANNEL_TAGS=\"<measures>\""
    systemCall += meta + " title=\"<name>\"" + meta + " timestamp=\"<timestamp>\""
    systemCall += " -y <path><sep><subfolder><sep><filename>_<timestampstring>.mkv"

    systemCall = systemCall.replace("<#channels>", str(len(ms.MEASUREMENTS)))
    systemCall = systemCall.replace("<measures>", ",".join(ms.MEASUREMENTS))
    systemCall = systemCall.replace("<samplingrate>", str(int(ms.samplingRate)))


    if path is not None: systemCall = systemCall.replace("<path>", path.rstrip(os.path.sep))
    else: systemCall = systemCall.replace("<path><sep>", "")

    if subfolder is not None: 
        systemCall = systemCall.replace("<subfolder>", subfolder.rstrip(os.path.sep))
        # Test if subfolder exists, if not create it
        subdirname = os.path.join(path, subfolder)
        if not os.path.exists(subdirname):
            try:
                os.mkdir(subdirname)
            except OSError:
                handleException(OSError, "Cannot create directory: " + str(subdirname))
            
    else: 
        systemCall = systemCall.replace("<subfolder><sep>", "")

    if name is not None: 
        systemCall = systemCall.replace("<name>", name)
    else: 
        systemCall = systemCall.replace("<name>", ms.name)

    if filename is not None: 
        systemCall = systemCall.replace("<filename>", filename.rstrip(".mkv"))
    else: 
        if "unknown" in ms.deviceName:
            print(bcolors.WARNING + "Found unknown device, will use name as filename, lets hope names are unique" + bcolors.ENDC)
            systemCall = systemCall.replace("<filename>", ms.name)
        else:
            systemCall = systemCall.replace("<filename>", ms.deviceName)

    if ts is not None: 
        dt = datetime.fromtimestamp(ts)
        timestampstring = dt.strftime(STORE_TIME_FORMAT)
        # If you have selected milliseconds
        if "%f" in STORE_TIME_FORMAT:
            timestampstring = timestampstring[:-3]
        systemCall = systemCall.replace("<timestampstring>", timestampstring)
        systemCall = systemCall.replace("<timestamp>", str(float(ts)))
    else: 
        systemCall = systemCall.replace("_<timestampstring>","")
        systemCall = systemCall.replace("<timestamp>", str(float(ms.getStartTs())))

    systemCall = systemCall.replace("<sep>", os.path.sep)

    # Return system call and filename
    return systemCall, systemCall.split("-y ")[1]


def performInParallel(functions):
    """
    Helper function to perform multiple functions in parallel using threads.

    Will wait for execution to be finished

    :param functions: List of function to call. No parameters allowed.
    :type  functions: list
    """
    # Stop and write all remaining frames to file
    threads = [threading.Thread(daemon=True, target=func) for func in functions]
    for thread in threads: 
        thread.start()

    for thread in threads:
        thread.join()

def getNextStoreTime(lastTime, timeInterval=None, storeTime=None):
    """
    Return the next store time based on:

    * last time
    * current time
    * chosen time interval *or* chosen store time

    :param lastTime: Time when it was stored the last time
    :type  lastTime: timestamp
    :param timeInterval: Number of seconds between stores. e.g. each 60seconds
    :type  timeInterval: int or float, default: None
    :param storeTime: Key of :attr:`STORE_TIME<recording.recordingServer.STORE_TIME>` array.
    :type  storeTime: str, default: None

    :return: Timestamp when next store should / can be performed.
    :rtype : timestamp
    """
    nextStoreTime = None
    if timeInterval is not None:
        nextStoreTime = lastTime+timeInterval
    
    if storeTime is not None:
        nsec = STORE_TIME[storeTime]["sec"]
        nextStoreTime = int((int(lastTime)+nsec)/nsec)*nsec

    return nextStoreTime

def handleBrokenConnection(ms, reason):
    """
    Handles broken connection of SmartDevices.
    See :attr:`connectionResetCB<powermeter.smartDevice.SmartDevice.connectionResetCB>`

    :param ms: Powermeter or Smartmeter which has a broken connection
    :type  ms: SmartDevice
    :param reason: Reason for broken connection. \"Killed\" if killed by us.
    :type  reason: str
    """
    if reason.lower() not in ["killed","restarted"]:
        msg = "Device " + str(ms.deviceName) + " removed, Reason: " + str(reason)
        logWarning(msg)
        if args.warningEmail is not None:
            scheduleEmail(msg, subject="Smartenergy Broken Connection")
    deviceDict[ms.deviceName]["dead"] = True
    deviceDict[ms.deviceName]["running"] = False

def handleDead(ms, ttime):
    """
    Handler if long time no msg or data has been received.
    See :attr:`deadConnectionCB<powermeter.smartDevice.SmartDevice.deadConnectionCB>`

    Will call :func:`handleBrokenConnection<recording.recordingServer.handleBrokenConnection>`
    if more than :attr:`REGARD_AS_DEAD_TIME<recording.recordingServer.REGARD_AS_DEAD_TIME>` seconds
    have passed.

    :param ms: Powermeter or Smartmeter which might be dead
    :type  ms: SmartDevice
    :param ttime: timestamp of last message received
    :type  ttime: timestamp
    """
    deadTime = time.time()-ttime
    if deadTime > REGARD_AS_DEAD_TIME:
        logWarning("Device " + str(ms.deviceName) + " dead for " + str(round(deadTime,3)) + "s, regarding as DEAD")
        handleBrokenConnection(ms, "No msg for " + str(round(deadTime,3)) + "s")

def clientConnected(netClient):
    """
    Called if a new TCP client connected to our server.

    :class:`NetClient<network.network.NetClient>` will be converted to
    :class:`PowerMeter<powermeter.powermeter.PowerMeter> and added to 
    :attr:`newMss<recording.recordingServer.newMss>` list.
    Loglevel is set.
    This list is handled in function :func:`handleNewDevices<recording.recordingServer.handleNewDevices>` 

    :param netClient: NetClient object that connected to our server.
    :type  netClient: NetClient
    """
    global deviceDict
    socket = netClient.sock
    verb = args.verbose>1
    ms = PowerMeter(updateInThread=True, directInit=True,
                    socketObject=socket,
                    verbose=verb, logger=logger)
    ms.setLogLevel(devLogLevel)
    # NTP request with minimum confidence required is now handled in start function
    # ms.ntpSync()
    # ms.start()
    logger("New device with addr " + str(ms.ip) + " connected")
    
    cnt = 0
    typeFound = False
    while not typeFound and running:
        time.sleep(1)
        if ms.brokenConnection:
            ms.kill()
            ms = None
            return
        if ms.deviceName is SmartDevice.UNKNOWN:
            cnt = cnt + 1
            if cnt > 5:
                logWarning("regard device with ip: " + str(ms.ip) + " as broken")
                ms.restart()
                ms.kill()
                ms = None
                return
            ms.systemInfo()
            # Try again next round
            logWarning("devicename for ip: " + str(ms.ip) + " not known yet")
        else:
            typeFound = True
    if typeFound:
        # This is a recover
        if ms.deviceName in deviceDict:
            # This should stop the exisitng thread
            deviceDict[ms.deviceName]["running"] = False
            deviceDict[ms.deviceName]["thread"].join()
            deviceDict[ms.deviceName]["thread"] = None
            deviceDict[ms.deviceName]["ms"] = None

            # Increase recovery cnt
            if not deviceDict[ms.deviceName]["restarted"]:
                deviceDict[ms.deviceName]["recover"] += 1
                msg = "Recovered device " + ms.name + ", recovery-cnt:" + str(deviceDict[ms.deviceName]["recover"])
                logWarning(msg)
                logmsg = time_format_ymdhms(time.time()) + ": " + msg
                scheduleEmail(logmsg, subject="Device recovered")
            newDict = deviceDict[ms.deviceName]
        # This is a brand new device
        else:
            newDict = {"recover":0, 'files':[], "thread":None}
            logger("Added new " + ms.TYPE + ": " + ms.name)
        
        newDict["ms"] = ms
        newDict["running"] = True
        newDict["dead"] = False
        newDict["restarted"] = False
            
        deviceDict[ms.deviceName] = newDict
        
        if (deviceDict[ms.deviceName]["thread"] is None):
            thread = threading.Thread(target=handleDevice,args=(deviceDict[ms.deviceName], ms.deviceName), daemon=True)
            deviceDict[ms.deviceName]["thread"] = thread
            thread.start()
                    


def handleException(exception=None, additionalMsg=""):
    """
    Called if an error occured or a caught eception was raised.
    Will send an email and pass it to logger.
    
    :param exception: Exception which might have happend
    :type  exception: Exception, default: ``None``
    :param additionalMsg: An additional msg which can be appended
    :type  additionalMsg: str, default: \"\"
    """
    msg = "Error: " + str(os.path.basename(__file__))

    if exception is not None:
        msg += "\nAn Exception was raised: " + str(exception)

    if additionalMsg != "":
        msg += "\n" + str(additionalMsg)

    logger(msg)
    if args.warningEmail is not None:
        logmsg = time_format_ymdhms(time.time()) + ": " + msg
        scheduleEmail(logmsg, subject="Smartenergy Exception")
    

def backupFile(filePath):
    """
    Backup the given file to backup directory.
    
    :param filePath: path of file to backup
    :type  filePath: str
    """
    backupPath = filePath.replace(args.outputFolder, args.backupFolder)
    # Create dirname if not existant
    dirname = os.path.dirname(backupPath)
    try:
        os.makedirs(dirname, exist_ok=True)
    except OSError:
        handleException(OSError, "Cannot create directory: " + str(dirname))
    # Copy over old file
    try:
        shutil.copyfile(filePath, backupPath)
    except Exception as e:
        handleException(e, "Error copying " + str(filePath) + " to backup directory: " + str(backupPath))


def handleDevice(device, name):
    """
    Handler for a single device.
    Is called in a thread, will set correct device type and 
    start sampling. Will also handle file storing
    
    :param device: Dictionary entry of dict :attr:`devices<recording.recordingServer.devices>`
    :type  device: dict
    :param name: Device Name of the device connected
    :type  name: str
    """
    ms = device["ms"]

    # We have to create a smartmeter out of the powermeter
    # Set class and sampling arguments
    # Start the sampling
    if ms.TYPE == SmartMeter.TYPE:
        ms.__class__ = SmartMeter
        ms.defaultSettings()
        ms.setMeasures(args.measures_smartmeter.split(","))
        ms.samplingRate = args.srSmartmeter
        ms.frameSize = int(ms.samplingRate/10)
        ms.start(ntpConfidenceMs=args.ntpConfidence)
    elif ms.TYPE == PowerMeter.TYPE:
        ms.__class__ = PowerMeter
        ms.defaultSettings()
        ms.setMeasures(args.measures_powermeter.split(","))
        ms.samplingRate = args.srPowermeter
        ms.frameSize = int(ms.samplingRate/10)
        ms.start(switchOn=args.switchOn, ntpConfidenceMs=args.ntpConfidence)
    
    # Quasi starttime, but we get better estimate from device
    startTS = time.time()
    # S.t. start takes effect and startTS is set correctly
    time.sleep(2.5)
    # Generate call
    tStartTS = ms.getStartTs()
    if tStartTS is not None: 
        startTS = tStartTS
    else:
        logWarning("Device " + str(name) + " maybe not started...")

    ms.deadConnectionCB = handleDead
    ms.connectionResetCB = handleBrokenConnection

    calls = constructFFMPEGCall(ms, path=args.outputFolder, 
                                ts=startTS if withTS else None, 
                                subfolder=ms.deviceName if args.subfolder else None)
    ffmpegP = subprocess.Popen(calls[0], shell=True, stdin=subprocess.PIPE, stdout=FNULL,  stderr=FNULL, preexec_fn=os.setsid)
    device["files"].append(calls[1])
    device["proc"] = ffmpegP
    device["samples"] = 0
    device["start"] = startTS
    device["storeTime"] = False
    if withTS:
        device["storeTime"] = True
        nextStoreTime = getNextStoreTime(startTS, timeInterval=args.timeInterval, storeTime=args.storeTime)
        device["nextStart"] = nextStoreTime
        samples = int((float(nextStoreTime)-float(startTS))*ms.samplingRate)
        device["samplesNext"] = samples
    
    while device["running"] and ms.sampling and not ms.brokenConnection:
        # prevent 100% CPU
        time.sleep(0.1)
        # shown = False
        while len(ms.frames) > 0:
            # Enough samples for next file
            if device["storeTime"] and device["samples"] + len(ms.frames[0]) >= device["samplesNext"]:
                # Write remainign samples to file
                remaining = device["samplesNext"]-device["samples"]
                remFrame = ms.frames[0][0:remaining]
                ffmpegP.stdin.write(remFrame.transpose().view(np.float32).reshape(remFrame.shape + (-1,)).flatten().tobytes())
                if remaining == len(ms.frames[0]):
                    del ms.frames[0]
                else:
                    ms.frames[0] = ms.frames[0][remaining:]
                ffmpegP.stdin.close()
            
                # Calculate next startts and #samples
                startTs = device["nextStart"]
                nextStoreTime = getNextStoreTime(startTs, timeInterval=args.timeInterval, storeTime=args.storeTime)
                offTime = float(time.time())-float(startTs)
                logger("{:<15} off time: {}s".format(str(name) + ":", round(offTime,3)))
                # Reset device if off too much
                if offTime > REGARD_AS_DEAD_TIME:
                    handleBrokenConnection(ms, "off time " + str(round(offTime,1)))
                    ms.restart()

                # Wait for process to complete before copying to backup location
                ffmpegP.wait()
                # If a backup location is specified, copy the last file to the backup location
                if args.backupFolder is not None:
                    backupFile(device["files"][-1])
                    

                device["start"] = startTs
                device["nextStart"] = nextStoreTime
                samples = int((float(nextStoreTime)-float(startTs))*ms.samplingRate)
                device["samplesNext"] = samples

                callAndFileName = constructFFMPEGCall(  ms, path=args.outputFolder, 
                                                        ts=startTs if withTS else None, 
                                                        subfolder=ms.deviceName if args.subfolder else None)
                ffmpegP = subprocess.Popen(callAndFileName[0], shell=True, stdin=subprocess.PIPE, stdout=FNULL,  stderr=FNULL, preexec_fn=os.setsid)
                device["proc"] = ffmpegP
                device["files"].append(callAndFileName[1])
                device["samples"] = 0
                
            else:
                device["samples"] += len(ms.frames[0])
                # recarray to nd array and flat out
                ffmpegP.stdin.write(ms.frames[0].transpose().view(np.float32).reshape(ms.frames[0].shape + (-1,)).flatten().tobytes())
                del ms.frames[0]
    
    ms.restart()
    # now kill ms and proc
    ms.kill()
    ffmpegP.stdin.close()
    time.sleep(0.01)
    ffmpegP.kill()

    # If a backup location is specified, copy the last file to the backup location
    if args.backupFolder is not None:
        backupFile(device["files"][-1])
    device["running"] = False



def logWarning(msg):
    """
    Log a warning message.

    :param msg: mesage to log
    :type  msg: str
    """
    logger(bcolors.WARNING + msg + bcolors.ENDC)

def logError(msg):
    """
    Log an error message.

    :param msg: mesage to log
    :type  msg: str
    """
    logger(bcolors.ERROR + msg + bcolors.ENDC)


# def handleNewDevices():
#     """
#     Handles new devices which are added to 
#     :attr:`newMss<recording.recordingServer.newMss>`.
#     Will retrieve information and if it is a SmartDevice,
#     it will call :func:`handleDevice<recording.recordingServer.handleDevice>` in a new thread.
#     """
#     global newMss, deviceDict
#     while running:
#         time.sleep(1)
#         temp = []
#         cnt = {}
#         # TODO: 
#         # There is some bug here actually
#         # CNT is not increasing, systemInfo is sometimes not sent
#         for nMs in newMss:
#             if nMs.deviceName is SmartDevice.UNKNOWN:
#                 if not nMs.brokenConnection:
#                     if nMs.ip not in cnt:
#                         cnt[nMs.ip] = 0
#                     cnt[nMs.ip] = cnt[nMs.ip] + 1
#                     if cnt[nMs.ip] > 5:
#                         logWarning("regard device with ip: " + str(nMs.ip) + " as broken")
#                         cnt[nMs.ip] = 0
#                         nMs.kill()
#                         nMs = None
#                     else:
#                         nMs.systemInfo()
#                         # Try again next round
#                         logWarning("devicename for ip: " + str(nMs.ip) + " not known yet")
#                         temp.append(nMs) 
#                 else:
#                     logWarning("device with ip: " + str(nMs.ip) + " broken")
#                     nMs.kill()
#                     nMs = None
#             else:
#                 # This is a recover
#                 if nMs.deviceName in deviceDict:
#                     # This should stop the exisitng thread
#                     deviceDict[nMs.deviceName]["running"] = False
#                     deviceDict[nMs.deviceName]["thread"].join()
#                     deviceDict[nMs.deviceName]["thread"] = None
#                     deviceDict[nMs.deviceName]["ms"] = None

#                     # Increase recovery cnt
#                     if not deviceDict[nMs.deviceName]["restarted"]:
#                         deviceDict[nMs.deviceName]["recover"] += 1
#                         logWarning("Recovered device " + nMs.name + ", recovery-cnt:" + str(deviceDict[nMs.deviceName]["recover"]))
#                     newDict = deviceDict[nMs.deviceName]
#                 # This is a brand new device
#                 else:
#                     newDict = {"recover":0, 'files':[], "thread":None}
#                     logger("Added new " + nMs.TYPE + ": " + nMs.name)
                
#                 newDict["ms"] = nMs
#                 newDict["running"] = True
#                 newDict["dead"] = False
#                 newDict["restarted"] = False
                    
#                 deviceDict[nMs.deviceName] = newDict
                
#                 if (deviceDict[nMs.deviceName]["thread"] is None):
#                     thread = threading.Thread(target=handleDevice,args=(deviceDict[nMs.deviceName], nMs.deviceName), daemon=True)
#                     deviceDict[nMs.deviceName]["thread"] = thread
#                     thread.start()
#             newMss = temp

def checkDiscUsage():
    pathsToCheck = [args.outputFolder]
    if args.backupFolder is not None:
        pathsToCheck.append(args.backupFolder)
    for path in pathsToCheck:
        total, used, free = shutil.disk_usage(path)
        logger("Disc Space on path " + str(path))
        logger("Total: %d GiB" % (total // (2**30)))
        logger("Used: %d GiB" % (used // (2**30)))
        logger("Free: %d GiB" % (free // (2**30)))
        GiBfree = (free // (2**30))
        if GiBfree < 10:
            msg = "Disc space below 10GB"
            logWarning(msg)
            logmsg = time_format_ymdhms(time.time()) + ": " + msg
            sendEmail(args.warningEmail, logmsg, subject="Disc space low on recording")


def tsFromFile(tfile):
    """
    Extract timestamp from file.
    Timestampformat must match
    :attr:`STORE_TIME_FORMAT<recording.recordingServer.STORE_TIME_FORMAT>`.

    :param tfile: Filepath/name from which timestamp is extracted.
    :type  tfile: str

    :return: Timestamp of file
    :rtype : timestamp
    """
    index = tfile.find("_")
    index2 = tfile.find(".")
    tsS = tfile[index+1:index2]
    ts = datetime.strptime(tsS, STORE_TIME_FORMAT).timestamp()
    return ts

def helpme():
    """Print help message for available commands"""
    for cmd in cmds:
        cmdName = ""
        le = len(cmd["cmd"])
        for i, cmdi in enumerate(cmd["cmd"]):
            cmdName += cmdi
            if i < le-1: cmdName += ", "
        print("\t- {:<16}: {}".format(cmdName, cmd["info"]))

def selectDevices(active=True):
    """
    Select device

    :param active: If only active devices should be returned or all.
    :type  active: bool, default: True

    :return: List of keys of dictionary :attr:`deviceDict<recording.recordingServer.deviceDict>`.
    :rtype : list
    """
    if active:
        deviceList = devGetter.terminalSelectionProcess([{"name":deviceDict[dev]["ms"].name, 
                                                        "device": deviceDict[dev]["ms"].deviceName, 
                                                        "ip": deviceDict[dev]["ms"].ip, 
                                                        "type": deviceDict[dev]["ms"].TYPE} for dev in deviceDict.keys()
                                                            if deviceDict[dev]["running"] and not deviceDict[dev]["dead"]])
    else:
        deviceList = devGetter.terminalSelectionProcess([{"name":deviceDict[dev]["ms"].name, 
                                                        "device": deviceDict[dev]["ms"].deviceName, 
                                                        "ip": deviceDict[dev]["ms"].ip, 
                                                        "type": deviceDict[dev]["ms"].TYPE} for dev in deviceDict.keys()])
    devNames = [dev["device"] for dev in deviceList]
    if len(devNames) == 0:
        print(bcolors.FAIL + "No devices" + bcolors.ENDC)
    return [dev for dev in deviceDict.keys() if dev in devNames]
    # mss = [deviceDict[dev]["ms"] for dev in deviceDict.keys() if deviceDict[dev]["ms"].ip in ips]
    # return mss

def stop():
    """Stop this recording."""
    global running
    print(bcolors.OK + "Stopping..." + bcolors.ENDC)
    running = False

def logLevel():
    """Set loglevel of device(s)"""
    print(bcolors.OK + "Set Log level" + bcolors.ENDC)
    devNames = selectDevices()  
    print(bcolors.OK + "Set Log level to ", end="" + bcolors.ENDC)
    logLevels = [str(level) for level in list(LogLevel)]
    print(bcolors.OK + [str(i+1)+":"+str(level) for i,level in enumerate(list(logLevels))] + bcolors.ENDC)
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
                print(bcolors.FAIL + "either type loglevel or integer" + bcolors.ENDC)
                c=input()
                continue
            else:
                if index >= 0 and index <= len(list(LogLevel))-1:
                    logLevel = list(LogLevel)[index]
                    break
                else:
                    print(bcolors.FAIL + "integer must be > 1 and < " + str(len(list(LogLevel))) + bcolors.ENDC)

    print(bcolors.OK + "Set LogLevel to " + str(logLevel) + bcolors.ENDC)
    # mss = [deviceDict[dev]["ms"] for dev in deviceDict.keys()]  
    for dev in devNames: 
        deviceDict[dev]["ms"].setLogLevel(logLevel)
       
def sysInfo(device, dic):
    """
    Callback passed to SmartDevice function 
    :func:`systemInfo<powermeter.smartDevice.SmartDevice.systemInfo>`
    """
    
    print("\n")
    print(bcolors.HEADER + device.name + ":" + bcolors.ENDC)
    for key in dic:
        print("{0:<15}".format(str(" " + key + ": ")[:15]) + str(dic[key]))
    print("\n")


def info():
    """Get info of device(s)"""
    print(bcolors.OK + "Select device to get info:" + bcolors.ENDC)
    # mss = selectDevices()
    devNames = selectDevices()  
    print(bcolors.OK + "Getting info about ms..." + bcolors.ENDC)
    for dev in devNames: 
        deviceDict[dev]["ms"].systemInfo(sysInfo)

def sampleInfo():
    """Get sampling info of device(s)"""
    print(bcolors.OK + "Select device to get sampling info:" + bcolors.ENDC)
    # mss = selectDevices()
    devNames = selectDevices(active=False)  
    print(bcolors.OK + "Getting info about sampling..." + bcolors.ENDC)
    for dev in devNames: 
        print(deviceDict[dev]["ms"].samplingInfo())
        if deviceDict[dev]["recover"] > 0:
            print(bcolors.WARNING + "\t#Recovers: " + str(deviceDict[dev]["recover"]) + bcolors.ENDC)
    

def listDevices():
    """List device and sampling info"""
    names = [deviceDict[dev]["ms"].deviceName for dev in deviceDict.keys()]  
    names.sort()
    print("\033[95m{:<14}{:<10}{:<16}{:<11}{:<25}\033[0m".format("Name:", "Recovers:", "IP:", "Avg [Hz]:", "Start:"))
    numPM = 0
    numSM = 0
    avgSM = 0
    avgPM = 0
    for name in names:
        start = time_format_ymdhms(deviceDict[name]["ms"].getStartTs())
        ip = str(deviceDict[name]["ms"].ip)
        avg = deviceDict[name]["ms"].getAvgRate()
        print("\033[95m{:<14}\033[0m{:<10}{:<16}{:<11}{:<25}".format(name[:14], deviceDict[name]["recover"], ip, str(avg), start))

        if deviceDict[name]["ms"].TYPE == SmartMeter.TYPE:
            numSM += 1
            avgSM += avg
        elif deviceDict[name]["ms"].TYPE == PowerMeter.TYPE:
            numPM += 1
            avgPM += avg
    if numPM > 0: 
        avgPM = round(avgPM/float(numPM), 3)
        print("\n\033[95m{} {} with avg. rate {} Hz\033[0m".format(str(numPM), PowerMeter.TYPE, avgPM))
    if numSM > 0: 
        avgSM = round(avgSM/float(numSM), 3)
        print("\033[95m{} {} with avg. rate {} Hz\033[0m".format(str(numSM), SmartMeter.TYPE, avgSM))
    
def restart():
    """Restart device(s)"""
    print(bcolors.OK + "Select device to restart" + bcolors.ENDC)
    # mss = selectDevices()
    devNames = selectDevices()  
    print(bcolors.OK + "Restarting..." + bcolors.ENDC)
    for dev in devNames: 
        deviceDict[dev]["ms"].restart()

def removeDevice():
    """Remove device(s)"""
    print(bcolors.OK + "Select device to stop recording and remove" + bcolors.ENDC)
    # mss = selectDevices()
    devNames = selectDevices()  
    print(bcolors.OK + "Stopping..." + bcolors.ENDC)
    for dev in devNames: 
        deviceDict[dev]["ms"].stop()
        # Kill handled from thread...
        # handleBrokenConnection(ms, reason)

def ntp():    
    """Sync NTP"""
    print(bcolors.OK + "Syncing NTP command" + bcolors.ENDC)
    devNames = selectDevices()  
    print(bcolors.OK + "Syncing NTP..." + bcolors.ENDC)
    for dev in devNames: 
        deviceDict[dev]["ms"].ntpSync(performInBg=True)


cmds = [
    {"func":helpme,       "cmd":["help", "h"],             "info":"Display this help message."},
    {"func":stop,         "cmd":["stop", "s"],             "info":"Stop sampling and the server."},
    {"func":info,         "cmd":["info", "i"],             "info":"Give info about device(s)."},
    {"func":sampleInfo,   "cmd":["?"],                     "info":"Give info about sampling."},
    {"func":listDevices,  "cmd":["l", "list"],             "info":"List connected devices alphabetically."},
    {"func":ntp,          "cmd":["ntp", "n"],              "info":"Start NTP synchronisation."},
    {"func":removeDevice, "cmd":["remove", "r"],           "info":"Remove device(s)."},
    {"func":restart,      "cmd":["restart"],               "info":"Restart device(s)."},
    {"func":logLevel,     "cmd":["logLevel", "ll"],        "info":"Change device(s) log level."}
]

def checkInput():
    """Check console input for stuff todo."""
    global running
    # press 'a' to abort the program
    # More convinient than using Ctrl-C
    print("Type 'help' to get list of available commands")
    while running:
        # Wait for key press
        try:
            key = input()
        except EOFError:
            continue
        found = False
        for cmd in cmds:
            if any(x == key for x in cmd["cmd"]):
                cmd["func"]()
                found = True
                break
        if not found and key != "":
            print("Unknown command")


def initParser():
    parser = argparse.ArgumentParser(description="Records data and stores it to file frequently.\
                                                  The powermeter need to connect to this streamserver.\
                                                  You can specify the streamserver e.g. using \"interface.py\".\
                                                  Pass the Destination Folder + file name things are stored to\
                                                  and the time interval it is stored.\
                                                  If a time interval is chosen, the time is added to the filename.")
    parser.add_argument('outputFolder', type=str, 
                        help="Folder to store MKV data.")
    parser.add_argument('--backup_folder', type=str, default=None,
                        dest="backupFolder",
                        help="Folder to keep a backup of the data e.g. an external drive.")
    parser.add_argument("-s", "--store_time", type=str, 
                        choices=list(STORE_TIME.keys()),
                        default=None,
                        dest="storeTime",
                        help="On each full X Minutes/hour of an hour, the data is stored into a file with \
                              corresponding suffix. e.g. 10_min -> start @ 09:08 -> storing @ 09:10, 09:20, etc.\
                              (default: None)")
    parser.add_argument("-t", "--time_interval", type=int, 
                        default=None, 
                        dest="timeInterval",
                        help="Time interval the data is written onto \
                              disk in SECONDS, default is None. If <timeInterval> is \
                              not None, date will be added to filename \
                              (default: None)")
    parser.add_argument("--samplingrate_powermeter", type=int, 
                        default=PowerMeter.DEFAULT_SR,
                        dest="srPowermeter",
                        help="Global samplingrate setting for powermeters \
                              (default: " + str(PowerMeter.DEFAULT_SR) + ")")
    parser.add_argument("--samplingrate_smartmeter", type=int, 
                        default=SmartMeter.DEFAULT_SR,
                        dest="srSmartmeter",
                        help="Global samplingrate setting for Smartmeters \
                              (default: " + str(SmartMeter.DEFAULT_SR) + ")")
    parser.add_argument("--measures_powermeter", type=str, choices=[",".join(measure['keys']) for measure in PowerMeter.AVAILABLE_MEASURES],
                        default=",".join(PowerMeter.AVAILABLE_MEASURES[0]['keys']),
                        help="Measures to use for powermeters \
                              (default: " + str(",".join(PowerMeter.AVAILABLE_MEASURES[0]['keys'])) + ")")
    parser.add_argument("--measures_smartmeter", type=str, choices=[",".join(measure['keys']) for measure in SmartMeter.AVAILABLE_MEASURES],
                        default=",".join(SmartMeter.AVAILABLE_MEASURES[0]['keys']),
                        help="Measures to use for smartmeters \
                              (default: " + str(",".join(PowerMeter.AVAILABLE_MEASURES[0]['keys'])) + ")")
    parser.add_argument("-p", "--port", type=int, 
                        default=54322,
                        dest="port",
                        help="Port on which to host the streamserver (default: 54322)")
    parser.add_argument("--switch_on", action="store_true", 
                        dest="switchOn",
                        help="Switch all Devices on, (default: False)")
    parser.add_argument("--time_slots", action="store_true", 
                        dest="timeSlots",
                        help="Use Time slots in which devices have to send the data. If you have e.g. 10 devices, a device sends data each 10s for 1s. You have to make sure that this is enough to send all data.")
    parser.add_argument("--subfolder", action="store_true", 
                        dest="subfolder",
                        help="Data is stored into subfolders with the name of the devices.")
    parser.add_argument("-l", "--log_level", type=str, choices=["all","info","debug","warning","error"], default="info",
                        help="Set the loglevel of the powermeters/smartmeters directly")
    parser.add_argument("--log_file", type=str,
                        dest="logFile",
                        help="Specify a logfile name/path where all msgs shown on console are stored")
    parser.add_argument("--restart_time", type=str, default=None,
                        dest="restartTime",
                        help="Specify a time of day when recording is stopped and devices are restarted. Sampling should restart thereafter Format: \"<hour>:<min>[:<sec>]\"(default: no restart)")
    parser.add_argument("--log_append", action="store_true",
                        dest="logAppend",
                        help="Do not delete the existing logfile entries, default: deleted")
    parser.add_argument("--warning_email", type=str, default=None,
                        dest="warningEmail",
                        help="On e.g. low disk space or recovering, an email is written to the specified address.")
    parser.add_argument("--smtp_settings", type=str, default=None,
                        dest="smtpSettings",
                        help="Pass source email address SMTP server settings as : \"<email>:<password>:<smtpSever>:<port>\".")
    parser.add_argument("--smtp_settings_file", type=str, default="email.conf",
                        dest="smtpSettingsFile",
                        help="Pass a config file in JSON format with \{\"address\":<mail>,\"user\":<user>,\"pwd\":<pwd>,\"smtp_server\":<server>,\"smtp_port\": <port>\}.")
    parser.add_argument("--ntp_confidence", type=int, 
                        default=30,
                        dest="ntpConfidence",
                        help="NTP confidence in milliseconds required for start time. The smaller the value the harder to achieve but the more accurate is the start time, default: 30ms")
    parser.add_argument("--store_time_format", type=str, default='%Y_%m_%d__%H_%M_%S',
                        dest="storeTimeFormat",
                        help="Format to use when appending time to the generated files. This ia not allowed to contain certain characters.\
                              You can use milliseconds with \"%%f\" but it MUST be the last format specifier and nothing is allowed behind it.\
                              Default: \"%%Y_%%m_%%d__%%H_%%M_%%S\"")
    parser.add_argument("-v", "--verbose", action="count", default=0,
                        help="Increase output verbosity")
    return parser
    
# _______________Can be called as main__________________
if __name__ == '__main__':
    parser = initParser()    
    args = parser.parse_args()

    # Look if store time format is valid
    STORE_TIME_FORMAT = args.storeTimeFormat
    if 1 in [c in STORE_TIME_FORMAT for c in set('/<>:\"\\|?*')]:
        sys.exit("Store time format not allowed")
    # Look if smtp server for email is correctly parsed
    if args.smtpSettings is not None:
        splitted = args.smtpSettings.split(":")
        if len(splitted) != 4:
            sys.exit("smtpSettings not set correctly. Format is: \"<email>:<password>:<smtpSever>:<port>\".")
        smtpDict["address"] = splitted[0]
        smtpDict["user"] = splitted[0]
        smtpDict["pwd"] = splitted[1]
        smtpDict["smtp_server"] = splitted[2]
        smtpDict["smtp_port"] = splitted[3]
    elif args.warningEmail:
        with open(args.smtpSettingsFile, "r") as tfile:
            mySmtpDict = json.loads(tfile.read())
            for key in list(smtpDict.keys()):
                if key not in mySmtpDict:
                    sys.exit("Missing smtp entry: " + str(key))
            print("Smtp settings successfully loaded from file")
            smtpDict = mySmtpDict


    # Init logger
    loggerObj = setupLogger('logger', toConsole=True, filePath=args.logFile, clear=not args.logAppend)
    logger = loggerObj.info
    # Get log level
    devLogLevel = LogLevel.fromString(args.log_level)
    # Look if output folder and backup folder exists
    if not os.path.exists(args.outputFolder):
        sys.exit("Folder \"" + str(args.outputFolder) + "\" does not exist")
    if args.backupFolder is not None and not os.path.exists(args.backupFolder):
        sys.exit("Backup Folder \"" + str(args.backupFolder) + "\" does not exist")


    # Store all in measurement system's list

    # for ms in mss:
    #     if devLogLevel != ms.logLevelDevice:
    #         ms.setLogLevel(devLogLevel)
    serverLogger = None
    if args.verbose > 1: serverLogger = print
    server = NetServer(port=args.port, address="0.0.0.0", connectedCallback=clientConnected, updateInThread=True, logger=serverLogger)
    server.start()

    ctrCCnt = 0
    # Get external abort
    def aborted(signal, frame):
        global running, ctrCCnt
        running = False
        ctrCCnt += 1
        if ctrCCnt > 2: 
            sys.exit()
    signal.signal(signal.SIGINT, aborted)


    print(bcolors.OK + "Starting Recording Server... stop with ctr-c" + bcolors.ENDC)

    if args.storeTime is not None or args.timeInterval is not None: withTS = True
    
    emailThread = None
    if args.warningEmail is not None:
        emailThread = threading.Thread(target=emailCollectThread, daemon=True)
        emailThread.start()

    inputThread = threading.Thread(target=checkInput, daemon=True)
    inputThread.start()


    def restartJob():
        print("Restarting @ " + str(time_format_ymdhms(time.time())))
        for dev in deviceDict.keys():
            deviceDict[dev]["ms"].stop()
            deviceDict[dev]["running"] = False
            deviceDict[dev]["restarted"] = True

    if args.restartTime is not None:
        schedule.every().day.at(args.restartTime).do(restartJob)
    
    checkDiscUsage()
    schedule.every().hour.do(checkDiscUsage)
    while running:
        time.sleep(1.0)
        schedule.run_pending()
        
    
    server.stop()

    # ffmpegProcesses = [deviceDict[dev]["proc"] for dev in deviceDict.keys()]  
    mss = [deviceDict[dev]["ms"] for dev in deviceDict.keys()]  
    # Stop sampling
    performInParallel([ms.stop for ms in mss])
    
    # Stop all threads
    for dev in deviceDict.keys():
        deviceDict[dev]["running"] = False
        if deviceDict[dev]["thread"] is not None: deviceDict[dev]["thread"].join()

    
    hasRecovers = False
    # Print infos about ms
    for deviceName in deviceDict.keys():
        print(deviceDict[deviceName]["ms"].samplingInfo())
        if deviceDict[deviceName]["recover"] > 0:
            hasRecovers = True
            print(bcolors.WARNING + "\t#Recovers: " + str(deviceDict[deviceName]["recover"]) + bcolors.ENDC)
                 

    print("Bye Bye from " + str(os.path.basename(__file__)))
