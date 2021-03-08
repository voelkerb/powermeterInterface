import os
import sys
import time
import re
import argparse
import powermeter
from powermeter.powerMeter import PowerMeter
# from powermeter.getDevices import *
import powermeter.getDevices as devGetter
import signal

VERBOSE = False
WAITTIME = 30

# _______________Can be called as main__________________
def initParser():
    parser = argparse.ArgumentParser(description="Get info of all powermeters.")
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

    
    mss = [PowerMeter(updateInThread=True,
                      ip=dev["ip"], port=dev["port"], useUDP=False, portUDP=5323+i, #  port=54322, stream=True,
                      samplingRate=dev["sr"], measures="v,i".split(','),
                      name=dev["name"],
                      verbose=args.verbose>1)
                      for i, dev in enumerate(deviceList)]   

    numDevices = len(mss)
    seenDevices = 0
    def sysInfo(device, dic):
        global seenDevices
        seenDevices += 1
        print("\n")
        print(device.name + ":")
        for key in dic:
            print("{0:<15}".format(str(" " + key + ": ")[:15]) + str(dic[key]))
        print("\n")

        # for ms in mss:
        #     ms.start()
    for ms in mss:
        ms.systemInfo(sysInfo)
    while seenDevices < numDevices:
        for ms in mss:
            ms.update()
        time.sleep(0.1)
            
    print("Bye Bye from " + str(os.path.basename(__file__)))
