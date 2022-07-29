"""Main File covering self build measurment system."""
# !/usr/bin/python
from imghdr import tests
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


class SmartBlueline(SmartDevice):
    """Class which handles a Blueline module."""

    TYPE = "Blueline".lower()

    DEFAULT_SR = 1000
    # BL5080_4T_LSB_TO_uVolt = 62.5
    # BL5080_4T_GAIN = 67.66
    
    AVAILABLE_MEASURES = [
            {"keys": [f"CH{i}" for i in range(4)], "bytes": 24, "dtype":np.float32, "cmdMeasure": None, "units":["NA"]*4}
        ]
    UNITS = {"T":"°C", "P":"bar"}


    # Init function with default values
    def __init__(self, name=None, deviceName=None, samplingRate=None, phase=None, measures=None,
                       serialPort=None, portName=None, baudrate=2000000,
                       socketObject=None, ip=None, port=54321, stream=False,
                       useUDP=False, portUDP=54323, rawValues=False,
                       updateInThread=False, directInit=True,
                       flowControl=False, cycleBased=False, simulation=-1,
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
        self.cycleBased = cycleBased
            # Look what measures we need
        if measures is not None:
            # make list if it is just a string with , spearated channelnames
            singleMeasures = measures[:]
            if not isinstance(measures, list): singleMeasures = measures.split(",")
            # remove leading trailing spaces
            singleMeasures = [s.rstrip(" ").lstrip(" ") for s in singleMeasures]
            # Sort, otherwise would be very difficult 
            singleMeasures.sort()
            allowedChannels = self.AVAILABLE_MEASURES[0]["keys"]
            self.measurementInfo.update({"keys": [], "cmdMeasure": []})
            # {"keys": [f"CH{i}" for i in range(4)], "bytes": 24, "cmdMeasure": None}
            for s in singleMeasures:
                if s not in allowedChannels:
                    sys.exit(f"Channel {s} is not a valid channel")
                self.measurementInfo["keys"].append(s)
                self.measurementInfo["cmdMeasure"].append(s)
            self.measurementInfo["bytes"] = len(self.measurementInfo["keys"])*4
            self.measurementInfo["units"] = ["NA"]*len(self.measurementInfo["keys"])
            if self.measurementInfo is None: 
                sys.exit("Blueline cannot handle mesurements: " + str(measures)) 
            else:
                if verbose: self.msPrint("Using mesures: " + str(self.measurementInfo))
        self.rawValues = rawValues
        if rawValues:
            if verbose: self.msPrint("Raw sampling using 16 bit integer values")
            self.measurementInfo["dtype"] = np.uint16
            self.measurementInfo["bytes"] = len(self.measurementInfo["keys"])*2
        self.offsets = {}
        self.scaling = {}
        self._multiply = {}
        self.refTemp = 0
        self.simulation = simulation


        self.MEASUREMENTS = self.measurementInfo["keys"]
        self.MEASUREMENT_BYTES = self.measurementInfo["bytes"]
    
    def _handleCommand(self, cmd, di):
        super()._handleCommand(cmd, di)
        if cmd == "info":
            # Store unit (up until now not send from device)
            if "deviceType" in di:
                if "T" in di["deviceType"]: self.measurementInfo["units"] = [self.UNITS["T"] for _ in range(len(self.AVAILABLE_MEASURES[0]["keys"]))]
                if "P" in di["deviceType"]: self.measurementInfo["units"] = [self.UNITS["P"] for _ in range(len(self.AVAILABLE_MEASURES[0]["keys"]))]
        # Sampling command to store raw data conversion factors 
        elif cmd == "sample":
            #  Set start ts if in dictionary
            if "offset" in di: self.offsets = {v:d for v,d in zip(self.MEASUREMENTS, di["offset"])}
            if "scaling" in di: self.scalings = {v:d for v,d in zip(self.MEASUREMENTS, di["scaling"])}
            if "scaling" in di and "trans" in di: self._multiply = {v:di["trans"]/self.scalings[s] for v,s in zip(self.MEASUREMENTS, self.scalings)}
            if self.verbose: self.msPrint("New * factor: " + str(self._multiply))
            if "refTemp" in di: self.refTemp = di["refTemp"]

    def defaultSettings(self):
        """
        Set default values to this Blueline.
        """
        self.samplingRate = self.DEFAULT_SR
        self.measurementInfo = self.AVAILABLE_MEASURES[0]
        self.MEASUREMENTS = self.measurementInfo["keys"]
        self.MEASUREMENT_BYTES = self.measurementInfo["bytes"]

    
    def getFrame(self, i):
        """Get a frame, max convert from raw to float"""
        cycleIdx = -1
        frame = None
        if len(self.frames) > i:
            frame = self.frames[i][:]
            if self.rawValues:
                if self.cycleBased:
                    x = np.where(frame[self.MEASUREMENTS[0]] == 1)[0]
                    if len(x) != 0:
                        cycleIdx = x[0]
                        # Check others
                        for _c,v in zip(self.MEASUREMENTS,[1,2,3,4]):
                            if frame[_c][cycleIdx] != v:
                                cycleIdx = -1
                                break
                dt = {'names':frame.dtype.names, 'formats':[np.float32]*len(self.MEASUREMENTS)}
                X = np.empty((len(self.frames[i]),), dtype=dt)
                for m in self.MEASUREMENTS: X[m] = frame[m]
                for m in self.MEASUREMENTS: X[m] = (X[m]-self.offsets[m])*self._multiply[m]+self.refTemp
                frame = X
            else:
                if self.cycleBased:
                    x = np.where(frame[self.MEASUREMENTS[0]] == -100.0)[0]
                    if len(x) != 0:
                        cycleIdx = x[0]
                        # Check others
                        for _c,v in zip(self.MEASUREMENTS,[-100.0,-101.0,-102.0,-103.0]):
                            if frame[_c][cycleIdx] != v:
                                cycleIdx = -1
                                break
        return frame, cycleIdx

    __tmpMsg = ""
    def __start(self, msg):
        self.__tmpMsg = msg

    def start(self, **args):
        temp = self.sendFunc            
        self.sendFunc = self.__start
        super().start(**args)
        # Add stuff specific for this module trigger 
        dic = json.loads(self.__tmpMsg)
        if self.cycleBased:
            dic["payload"]["indicateTrig"] = True
        if self.simulation > 0:
            dic["payload"]["simu"] = self.simulation

        # Restore send func
        self.sendFunc = temp
        self.sendFunc(json.dumps(dic))

    def setDeviceType(self, devType):
        r"""
        Set calibration coefficients.

        :param parameter: dictionary with scaling parameter as list
        :type  parameter: list
        """
        if devType not in ["4T", "4P"]:
            self.msPrint("Error:Unsupported device type: " + str(devType))
            return

        self.sendFunc(json.dumps({
            "cmd":"type",
            "type":devType
        })) 

    def setScaling(self, parameter):
        r"""
        Set calibration coefficients.

        :param parameter: dictionary with scaling parameter as list
        :type  parameter: list
        """
        self.sendFunc(json.dumps({
            "cmd":"scaling",
            "cal":parameter
        })) 
    
    def setOffset(self, parameter):
        r"""
        Set calibration coefficients.

        :param parameter: dictionary with offset parameter as list
        :type  parameter: list
        """
        self.sendFunc(json.dumps({
            "cmd":"offset",
            "cal":parameter
        })) 
    


def initParser():
    import argparse
    parser = argparse.ArgumentParser(description="Records data from Blueline.\
                                                  Can plot it or write it to mkv file. Serial or TCP/UDP support.\
                                                  Choose between sampling rates and measures.")
    parser.add_argument("--host", type=str,
                        help="Hostname or IP address of device e.g. blueline001.local")
    parser.add_argument("--port", type=int, default=54321,
                        help="Port, default: 54321.")
    # parser.add_argument("--measures", type=str, choices=[",".join(measure['keys']) for measure in SmartBlueline.AVAILABLE_MEASURES],
    parser.add_argument("--measures", type=str,
                        default=",".join(SmartBlueline.AVAILABLE_MEASURES[0]['keys']),
                        help="Measures to use for recording/plotting \
                              (default: " + str(",".join(SmartBlueline.AVAILABLE_MEASURES[0]['keys'])) + ")")
    parser.add_argument("--samplingrate", type=int, default=4000,
                        help="Samplingrate of the blueline, default: 250")
    parser.add_argument("--serial", type=str,
                        help="SerialPort path. COMX under windows, /dev/ttyXXX under Unix systems.")
    parser.add_argument("--baudrate", type=int, default=2000000,
                        help="Baudrate of serialport")
    parser.add_argument("--udp", action="store_true",
                        help="If UDP should be used to send data")
    parser.add_argument("--raw", action="store_true",
                        help="If data should be sent as 16 bit integers over channel, effectively devides data througput by 2")
    parser.add_argument("--plot", action="store_true",
                        help="If data should be plotted")
    parser.add_argument("--cycleBased", action="store_true",
                        help="If data should be stored cycle based")
    parser.add_argument("--simulation", type=int, default=-1,
                        help="Simulation mode, int represents milliseconds of cycle, -1 to disable")
    parser.add_argument("--ffmpeg", action="store_true",
                        help="If data should be written to mkv file. You can specify filename with --filename")
    parser.add_argument("--csv", action="store_true",
                        help="If data should be written to csv file. You can specify filename with --filename")
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
    import csv
    from queue import Queue
    parser = initParser()
    args = parser.parse_args()

    # Check for valid arguments passed here
    validConfig = False
    if args.host is not None and args.port is not None: validConfig = True
    if args.serial is not None and args.baudrate is not None: validConfig = True
    if not validConfig: sys.exit("Either provide ip + port or serial + baudrate")

    # device connection
    ms = SmartBlueline(ip=args.host, port=args.port, useUDP=args.udp,
                  portName=args.serial, baudrate=args.baudrate, measures=args.measures.split(','),
                  verbose=args.verbose, updateInThread=False, name="blueline", rawValues=args.raw,
                  samplingRate=args.samplingrate, cycleBased=args.cycleBased, simulation=args.simulation, directInit=True)
    ms.frameSize = max(1,int(ms.samplingRate/50))
    if args.serial is not None: 
        time.sleep(1)
        ms.getType() # flushes serial
    # Catch control+c
    app = None
    running = True
    # Get external abort
    def aborted(signal, frame=None):
        global running
        running = False
        if app: app.quit()
        
        if args.serial: sys.exit()
        if sys.platform == 'win32':
            return True

    if sys.platform == 'win32':
        import win32api
        win32api.SetConsoleCtrlHandler(aborted, True)
    else:
        signal.signal(signal.SIGINT, aborted)

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
    mapping = {
        "CH0": {"active": False, "plot": None, "curve": None, "pen":"r", "data": []},
        "CH1": {"active": False, "plot": None, "curve": None, "pen":"b", "data": []},
        "CH2": {"active": False, "plot": None, "curve": None, "pen":"g", "data": []},
        "CH3": {"active": False, "plot": None, "curve": None, "pen":[0,0,0,128], "data": []},
        }

    # Init plot depending on selected measures
    def initPlot():
        global linkedPlots, mapping
        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)

        # Get measures to display
        for key in mapping:
            if key in ms.MEASUREMENTS: mapping[key]["active"] = True

        # add new plot for power (same axis scale)
        p_c_0 = win.addPlot()
        p_c_0.getViewBox().setMouseEnabled(x=False, y=False)
        p_c_0.setLabel('left', "Value")
        for ch in ms.MEASUREMENTS:
            mapping[ch]["plot"] = p_c_0

        updateViews()
        if len(linkedPlots) > 0:
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

    
    # Update live plot with seconds counter
    start = 0
    end = 0
    theData = None
    def updatePlot():
        global theData, curves, start, running, plotQueue, end
        while not plotQueue.empty():
            frame = plotQueue.get()
            plotQueue.task_done()
            if frame is None:
                # Indication of new cycle remove all existing data
                theData = None
                start = 0
                end = 0
            else:
                newFrameLen = len(frame)
                end += newFrameLen/ms.samplingRate
                if theData is None: theData = frame
                else: theData = np.concatenate([theData, frame], axis=0)
                theData = theData[-maxPoints:]
                for key in ms.MEASUREMENTS: mapping[key]["data"] = theData[key]
                dlen = len(theData)
                myStart = end-(dlen/ms.samplingRate)
                x = np.linspace(myStart, end, dlen)
                start += float(newFrameLen/ms.samplingRate)
                x = x[0:dlen]
                for key in ms.MEASUREMENTS:
                    mapping[key]["curve"].setData(x, mapping[key]["data"][0:len(x)])

    #b Construct the ffmpeg call for a single device
    def constructFFMPEGCall(ms, path=None, cycle=-1):
        systemCall = "ffmpeg -hide_banner -f f32le -ar " + str(ms.samplingRate) + " -guess_layout_max 0 -ac "
        systemCall += str(len(ms.MEASUREMENTS)) + " -i pipe:0 -c:a wavpack "
        meta = " -metadata:s:a:0"
        systemCall += meta + " CHANNELS=" + str(len(ms.MEASUREMENTS)) + meta + " CHANNEL_TAGS=\""
        systemCall += ",".join(ms.MEASUREMENTS) + "\""
        systemCall += meta + " title=" + "\"" + str(ms.name) + "\""
        systemCall += " -y "

        thePath = ""
        if path is not None: 
            thePath = path
            if os.path.isdir(path):
                thePath = os.path.join(path, ms.name + ".mkv")
            else:
                thePath = path.rstrip(".mkv") + ".mkv"
        else:
            ts = ms.getStartTs()
            if ts is None: ts = time.time()
            thePath = ms.name + "_" + time_format_ymdhms(ts).replace(".","_").replace(":","_").replace("/","_").replace(" ","__") + ".mkv"
        if cycle != -1: thePath = thePath.split(".mkv")[0] + "_" + str(ms.samplingRate) + "_cycle_" + str(cycle) + ".mkv"
        systemCall += thePath
        return systemCall

    def constructCSVWriter(ms, path=None, cycle=-1):
        # Embed timestamp in outputfilename
        ts = ms.getStartTs()
        if ts is None: ts = time.time()
        filename = ms.deviceName + "_" + str(ms.samplingRate) + "HZ" + "_" + time_format_ymdhms(ts).replace(".","_").replace(":","_").replace("/","_").replace(" ","__") + ".csv"
        # given filename
        if args.filename is not None: filename = args.filename
        # Make sure csv format
        if not filename.find(".csv"): filename.split(".")[0] + ".csv"
        if cycle != -1: filename = filename.split(".csv")[0] + "_" + str(ms.samplingRate) + "_cycle_" + str(cycle) + ".csv"
        if args.verbose: print("Storing CSV data to: " + filename)
        csvFile = open(filename, 'w')
        csvWriter = csv.writer(csvFile, lineterminator='\n')
        return csvWriter, csvFile

    cycles = -1
    csvFile = None
    cycleCnt = 0
    # Update the measurement system
    def updateMs():
        global cycleCnt, ffmpegProc, running, ms, plotQueue, csvFile, cycles
        # Init ffmpeg
        if args.cycleBased: cycles = 0
        if args.ffmpeg:
            ffmpegCall = constructFFMPEGCall(ms, path=args.filename, cycle=cycles)
            ffmpegProc = subprocess.Popen(ffmpegCall, shell=True, stdin=subprocess.PIPE)
        if args.csv:
            csvWriter, csvFile = constructCSVWriter(ms, path=args.filename, cycle=cycles)
            csvWriter.writerow([f"{m} [{u}]" for m, u in zip(ms.MEASUREMENTS, ms.measurementInfo["units"])])
        cycleCnt = 0

        while running or len(ms.frames):
            # Update ms
            if running: ms.update()
            # on every new frame
            while len(ms.frames) > 0:
                # frame = ms.frames[0]
                frame, cycleIndex = ms.getFrame(0)
                
                cycleSplit = [frame]
                if cycleIndex != -1:#
                    
                    print(f"Cycle: {cycleCnt}")
                    cycleCnt += 1
                    cycleSplit = [frame[:cycleIndex], frame[cycleIndex+1:]]

                for i, frame in enumerate(cycleSplit):
                    
                    # New cycle here
                    if i > 0:
                        cycles += 1
                        # Construct new ffmpeg file
                        if args.ffmpeg:
                            # Close current process
                            ffmpegProc.stdin.close()
                            # Start new one with new filename
                            ffmpegCall = constructFFMPEGCall(ms, path=args.filename, cycle=cycles)
                            ffmpegProc = subprocess.Popen(ffmpegCall, shell=True, stdin=subprocess.PIPE)
                        # Construct new csv file
                        if args.csv:
                            csvFile.close()
                            csvWriter, csvFile = constructCSVWriter(ms, path=args.filename, cycle=cycles)
                            csvWriter.writerow(ms.MEASUREMENTS)
                            csvWriter.writerow([f"{m} [{u}]" for m, u in zip(ms.MEASUREMENTS, ms.measurementInfo["units"])])
                        # Add none to queue to indicate cycle to plot thread
                        if plotQueue is not None: plotQueue.put(None)

                    # Update ffmpeg
                    if args.ffmpeg:
                        ffmpegProc.stdin.write(frame.transpose().view(np.float32).reshape(frame.shape + (-1,)).flatten().tobytes())
                    if args.csv:
                        csvWriter.writerows(frame)

                    # update plot using Queue
                    if plotQueue is not None: plotQueue.put(frame)

                del ms.frames[0]
            if args.serial:
                time.sleep(0.00001)
            else:
                time.sleep(0.001)
            
    # global plot variables required
    if args.plot and running:
        # Setup main GUI
        app = QtGui.QApplication([])
        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)

        # basic config
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        # pg.setConfigOption('foreground', 'w')
        # pg.setConfigOption('background', 'k')
        win = pg.GraphicsWindow(title="Live Data")
        win.resize(1000, 600)
        # define plot size depending on sr
        if args.cycleBased:
            maxTime = 100
        else:
            maxTime = 0.1
            if ms.samplingRate <= 10: maxTime = 20.0
            elif ms.samplingRate <= 100: maxTime = 10.0
            elif ms.samplingRate <= 1000: maxTime = 2.0
            elif ms.samplingRate <= 2000: maxTime = 1.0
            elif ms.samplingRate <= 4000: maxTime = 0.5
            elif ms.samplingRate <= 8000: maxTime = 0.2
        maxPoints = int(ms.samplingRate*maxTime)

    # Queue to hold new frames for plotting
    plotQueue = None
    if args.plot: plotQueue = Queue(maxsize=10)
    # Start sampling data
    ms.start()
    # Thread to update the MS
    thread = threading.Thread(target=updateMs)
    thread.start()

    # Start QT application gui. use app.quit() to cancel this in some event
    if args.plot and running:
        initPlot()
        timer = QtCore.QTimer()
        timer.timeout.connect(updatePlot)
        # Update with 50Hz
        # Make sure that framesize is never below, otherwise
        # queue is not emptied quick enough 
        timer.start(20)
        QtGui.QApplication.instance().exec_()
        running = False

    # Waits for update thread to complete
    thread.join()
    # stop sampling process
    ms.stop()
    # Wait for devices to really stop before killing them
    time.sleep(1)
    ms.kill()
    # Close remaining open files
    if args.ffmpeg:
        ffmpegProc.stdin.close()
    if args.csv: 
        csvFile.close()
    # Print sampling info
    print(ms.samplingInfo())

    print("Bye Bye from " + str(os.path.basename(__file__)))
