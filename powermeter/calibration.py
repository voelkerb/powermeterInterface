"""Main File covering self build measurment system."""
# !/usr/bin/python
import sys
import os
# Import top level module
try:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
except NameError:
    root = os.path.dirname(os.path.dirname(os.path.abspath(sys.argv[0])))
sys.path.append(root)

import time
import traceback
import threading
# We need to add the path
from powermeter.smartDevice import SmartDevice, LogLevel
from powermeter.powerMeter import PowerMeter
from powermeter.smartMeter import SmartMeter
import measurement.valuesAndUnits as vu
import math

SFOS = 200


def calculateMeasures(frame):
    sfos = SFOS
    if vu.VOLTAGE[1] in frame.dtype.names:
        voltage = frame[vu.VOLTAGE[1]]
        current = frame[vu.CURRENT[1]]
    else:
        voltage = frame[vu.VOLTAGE[0]]
        current = frame[vu.CURRENT[0]]
    v_mean = np.mean(voltage)
    i_mean = np.mean(current)
    voltage = voltage-v_mean
    current = current-i_mean

    numPoints = len(voltage)
    reshaping = int(math.floor(numPoints/sfos))
    end = reshaping*sfos

    invalidVoltage = np.where(voltage > 500)[0]
    invalidCurrent = np.where(current > 50000)[0]
    if len(invalidCurrent) > 0:
        print("invalid current: ")
        print(current[invalidCurrent])
    if len(invalidVoltage) > 0:
        print("invalid current: ")
        print(voltage[invalidVoltage])
        
    voltage[voltage > 500] = 0
    current[current > 50000] = 0

    momentary = 0.001*np.array(voltage[:end]*current[:end])
    momentary = momentary.reshape((-1, sfos))
    p = np.mean(momentary, axis=1)

    v = voltage[:end].reshape((-1, sfos))
    i = current[:end].reshape((-1, sfos))
    # quicker way to do this
    vrms = np.sqrt(np.einsum('ij,ij->i', v, v)/sfos)
    irms = np.sqrt(np.einsum('ij,ij->i', i, i)/sfos)

    # Because unit of current is in mA
    s = 0.001*vrms*irms
    q = np.sqrt(np.abs(s*s - p*p))
    
    return np.mean(vrms), np.mean(irms), np.mean(p), np.mean(q), np.mean(s), v_mean, i_mean

def initParser():
    import argparse
    parser = argparse.ArgumentParser(description="Records data from powermeter.\
                                                  Can plot it or write it to mkv file. Serial or TCP/UDP support.\
                                                  Choose between sampling rates and measures.")
    parser.add_argument("--host", type=str,
                        help="Hostname or IP address of device: e.g. powermeter01.local")
    parser.add_argument("--port", type=int, default=54321,
                        help="Port, default: 54321.")
    parser.add_argument("--samplingrate", type=int, default=4000,
                        help="Samplingrate of the powermeters, default: 4000")
    parser.add_argument("--serial", type=str,
                        help="SerialPort path. COMX under windows, /dev/ttyXXX under Unix systems.")
    parser.add_argument("--baudrate", type=int, default=2000000,
                        help="Baudrate of serialport")
    parser.add_argument("--udp", action="store_true",
                        help="If UDP should be used to send data")
    parser.add_argument("-v", "--verbose", action="count", default=0,
                        help="Increase output verbosity")

    return parser
    
# _______________Can be called as main__________________
if __name__ == '__main__':
    import time
    import signal
    import numpy as np
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
    if "powermeter" in name:
        # device connection
        ms = PowerMeter(ip=args.host, port=args.port, useUDP=args.udp,
                        portName=args.serial, baudrate=args.baudrate,
                        verbose=args.verbose, updateInThread=False, name=name,
                        flowControl=False,
                        samplingRate=args.samplingrate)
    elif "smartmeter" in name:
        # device connection
        # device connection
        ms = SmartMeter(ip=args.host, port=args.port, useUDP=args.udp,
                        portName=args.serial, baudrate=args.baudrate,
                        verbose=args.verbose, updateInThread=False, name="smartmeter",
                        samplingRate=args.samplingrate, phase=1)

    # Framesize will be 1second
    ms.frameSize = int(args.samplingrate)
    SFOS = int(ms.samplingRate/50.0)
    ms.setLogLevel(LogLevel.WARNING)
    ms.start()

    while True:
        ms.update()
        # on every new frame
        while len(ms.frames) > 0:
            vrms, irms, p, q, s, vmean, imean = calculateMeasures(ms.frames[0])
            print("V_Mean: {:.02f}\tI_Mean: {:.02f}\tV_RMS: {:.02f}\tI_RMS: {:.02f}\tP: {:.02f}\tQ: {:.02f}\tS: {:.02f}".format(vmean, imean, vrms, irms, p, q, s))
            # remove frame
            del ms.frames[0]

    ms.stop()
        
    print(ms.samplingInfo())

    print("Bye Bye from " + str(os.path.basename(__file__)))
