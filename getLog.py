
import os
import sys
import time
import re
import argparse
from powermeter.powerMeter import PowerMeter
# from powermeter.getDevices import *
import powermeter.getDevices as devGetter
import signal

VERBOSE = False
WAITTIME = 30

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

def initParser():
    parser = argparse.ArgumentParser()
    parser = argparse.ArgumentParser(description="Get log of powermeters.\
                                                  use multiple \"-d <mdns or ip>\" to connect to individual devices or\
                                                  if you don't specify a device all devices are searched with mdns.")
    devGetter.addDeviceCommandLineArgs(parser, deviceType=True, verify=True)
    parser.add_argument("-l", "--lines", type=int, default=-1,
                        help="Last number of lines of the log to show")
    parser.add_argument("-c", "--clear", action="store_true",
                        help="Clear the log file")
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
    
    mss = [PowerMeter(updateInThread=True,
                      ip=dev["ip"], port=dev["port"], useUDP=False, portUDP=5323+i, #  port=54322, stream=True,
                      samplingRate=dev["sr"], measures="v,i".split(','),
                      name=dev["name"],
                      verbose=args.verbose>1)
                      for i, dev in enumerate(deviceList)]  

    numDevices = len(mss)
    seenDevices = 0

    start = 0
    if args.lines != -1:
        start = -1*args.lines 
    def printLog(device, loglist):
        global seenDevices
        seenDevices += 1
        printPink("\n" + device.name + ":")
        for msg in loglist[start:]:
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
    for ms in mss:
        ms.getLog(printLog)
    
    ttime = time.time()
    while seenDevices < numDevices and time.time() - ttime < 3.0:
        for ms in mss:
            ms.update()
        time.sleep(0.001)
    
    if args.clear:
        printBlue("Press enter to delete the log file(s). Ctr-c to cancel")
        c = input()
        printBlue("Clearing ...")
        for ms in mss:
            ms.clearLog()

        time.sleep(1)
        for ms in mss:
            ms.update()
    
    print("Bye Bye from " + str(os.path.basename(__file__)))
