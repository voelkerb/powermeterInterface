"""Main File covering self build measurment system."""
# !/usr/bin/python
import sys
import os
import time
import traceback
import threading
# We need to add the path
from smartDevice import *
import json



class PowerMeter(SmartDevice):
    """Class which handles a measurement system."""
    
    TYPE = "PowerMeter".lower()
    DEFAULT_SR = 4000
    AVAILABLE_MEASURES = [
                {"keys": [VOLTAGE[0], CURRENT[0]],                                           "bytes": 8,  "cmdMeasure": "v,i"    },
                {"keys": [ACTIVE_POWER[0], REACTIVE_POWER[0]],                               "bytes": 8,  "cmdMeasure": "p,q"    },
                {"keys": [VOLTAGE[0], CURRENT[0], ACTIVE_POWER[0], REACTIVE_POWER[0]], "bytes": 16, "cmdMeasure": "v,i,p,q"},
                {"keys": [VOLTAGE_RMS[0], CURRENT_RMS[0]],                                   "bytes": 8,  "cmdMeasure": "v,i_RMS"},
            ]

    # Init function with default values
    def __init__(self, name=None, deviceName=None, samplingRate=None, measures=None,
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
        # Standard measures
        self.measurementInfo = self.AVAILABLE_MEASURES[0]
        
        # Look what measures we need
        if measures is not None:
            self.measurementInfo = None
            for availableMeasureSet in self.AVAILABLE_MEASURES:
                if all(item in availableMeasureSet["keys"] for item in measures): 
                    self.measurementInfo = availableMeasureSet
                    break
            if self.measurementInfo is None: 
                sys.exit("Powermeter cannot handle mesurements: " + str(measures)) 
        
        self.MEASUREMENTS = self.measurementInfo["keys"]
        self.MEASUREMENT_BYTES = self.measurementInfo["bytes"]

    def defaultSettings(self):
        self.samplingRate = self.DEFAULT_SR
        self.measurementInfo = self.AVAILABLE_MEASURES[0]
        self.MEASUREMENTS = self.measurementInfo["keys"]
        self.MEASUREMENT_BYTES = self.measurementInfo["bytes"]

    
    def start(self, ts=None, switchOn=True, sendSlot=None, ntpConfidenceMs=None):
        """Overloading start function to switch it on if needed."""
        if switchOn:
            self.switch(True)
        super().start(ts=ts, sendSlot=sendSlot, ntpConfidenceMs=ntpConfidenceMs)

    def switch(self, on):
        """Switch socket of the powermeter."""
        msg = "{\"cmd\":\"switch\",\"payload\":{\"value\":"
        if on: msg += str(1)
        else: msg += str(0)
        msg += "}}"
        self.sendFunc(msg)

    def calibrate(self, parameter):
        r"""
        Set calibration coefficients.

        :param parameter: dictionary with calibration parameter as \{'v':0.99,'i':1.01 \}
        :type  parameter: dict
        """
        if VOLTAGE[0] in parameter and CURRENT[0] in parameter:
            calDict = {"cmd":"calibration","calV":parameter["v"],"calI":parameter["i"]}
            self.sendFunc(json.dumps(calDict)) 
        else:
            self.msPrint("Cannot use given parameters to calibrate")

def initParser():
    import argparse
    parser = argparse.ArgumentParser(description="Records data from powermeter.\
                                                  Can plot it or write it to mkv file. Serial or TCP/UDP support.\
                                                  Choose between sampling rates and measures.")
    parser.add_argument("--host", type=str,
                        help="Hostname or IP address of device: e.g. powermeter01.local")
    parser.add_argument("--port", type=int, default=54321,
                        help="Port, default: 54321.")
    parser.add_argument("--measures",type=str, choices=[",".join(measure['keys']) for measure in PowerMeter.AVAILABLE_MEASURES],
                        default=",".join(PowerMeter.AVAILABLE_MEASURES[0]['keys']),
                        help="Measures to use for recording/plotting")
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

    # Check for valid arguments passed here
    validConfig = False
    if args.host is not None and args.port is not None: validConfig = True
    if args.serial is not None and args.baudrate is not None: validConfig = True
    if not validConfig: sys.exit("Either provide ip + port or serial + baudrate")

    name = "powermeter"
    if args.host is not None and "powermeter" in args.host:
        name = args.host.rstrip(".local")
    # device connection
    ms = PowerMeter(ip=args.host, port=args.port, useUDP=args.udp,
                    portName=args.serial, baudrate=args.baudrate,
                    verbose=args.verbose, updateInThread=False, name=name,
                    flowControl=False,
                    samplingRate=args.samplingrate, measures=args.measures.split(','))
    ms.frameSize = int(args.samplingrate/50.0)
    
    ffmpegProc = None
    # Update linked views for plotting
    linkedPlots = []
    def updateViews():
        global linkedPlots
        for leftAxisPlot, rightAxisPlot in linkedPlots:
            rightAxisPlot.setGeometry(leftAxisPlot.getViewBox().sceneBoundingRect())
            rightAxisPlot.linkedViewChanged(leftAxisPlot.getViewBox(), rightAxisPlot.XAxis)

    # Mapping holds plots and data
    mapping = {ACTIVE_POWER[0]: {"active": False, "plot": None, "curve": None, "pen":"r", "data": []},
               REACTIVE_POWER[0]: {"active": False, "plot": None, "curve": None, "pen":"b", "data": []},
               VOLTAGE[0]: {"active": False, "plot": None, "curve": None, "pen":"r", "data": []},
               CURRENT[0]:  {"active": False, "plot": None, "curve": None, "pen":"b", "data": []},}

    # Init plot depending on selected measures
    def initPlot():
        global linkedPlots, mapping
        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)

        # Get measures to display
        for key in mapping:
            if key in ms.MEASUREMENTS: mapping[key]["active"] = True

        # add new plot for power (same axis scale)
        if ACTIVE_POWER[0] in ms.MEASUREMENTS or REACTIVE_POWER[0] in ms.MEASUREMENTS:
            p_c_0 = win.addPlot()
            p_c_0.getViewBox().setMouseEnabled(x=False, y=False)
            p_c_0.setLabel('left', "Power [W]")
            p_c_0.setLabel('right', "Power [VAR]")
            mapping[ACTIVE_POWER[0]]["plot"] = p_c_0
            mapping[REACTIVE_POWER[0]]["plot"] = p_c_0
            win.nextRow()

        # add plot for current and voltage different axis
        if VOLTAGE[0] in ms.MEASUREMENTS or CURRENT[0] in ms.MEASUREMENTS:
            p_c_1 = win.addPlot()
            p_c_1.getViewBox().setMouseEnabled(x=False, y=False)
            p_c_1.setLabel('left', "Voltage [V]")
            p_c_1.setLabel('right', "Current [mA]")
            mapping[VOLTAGE[0]]["plot"] = p_c_1

            p_c_2 = pg.ViewBox()
            p_c_1.scene().addItem(p_c_2)
            p_c_1.getAxis('right').linkToView(p_c_2)
            p_c_2.setXLink(p_c_1)
            mapping[CURRENT[0]]["plot"] = p_c_2
            linkedPlots.append((p_c_1, p_c_2))

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
    def constructFFMPEGCall(ms, path=None):
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
            thePath = ms.name + ".mkv"
        systemCall += thePath
        return systemCall

    # Update the measurement system
    def updateMs():
        global ffmpegProc, running, ms, plotQueue
        # Init ffmpeg
        if args.ffmpeg:
            FNULL = open(os.devnull, 'w')
            ffmpegCall = constructFFMPEGCall(ms, path=args.filename)
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
            time.sleep(0.001)

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

    ms.start()

    # EVERY_X_SECONDS = 20
    # while running:
    #     if int(time.time()%EVERY_X_SECONDS) == 0:
    #         print("receiving...")
    #         ms.setClearToReceive(True)
    #         while int(time.time()%EVERY_X_SECONDS) == 0:
    #             ms.update()
    #             # time.sleep(0.0001)
    #         ms.setClearToReceive(False)
    #         ms.update()
    #         print("fin")
    #     else:
    #         time.sleep(0.1)
    #         ms.update()
        


    thread = threading.Thread(target=updateMs)
    thread.start()


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
