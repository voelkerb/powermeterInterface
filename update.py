
import os
import sys
import time
# this is required for arduino plugin since it seems to be not included if not called from bash
sys.path.append('/usr/local/lib/python3.7/site-packages')
import argparse
from network.network.mDNS import mDNS, Basher
import signal
from powermeter.powerMeter import PowerMeter
import powermeter.ota.modifiedOTA as ota
import random
from blessings import Terminal
from progressbar import ProgressBar
import threading
import cursor
import multiprocessing
import powermeter.getDevices as devGetter
        
def printYellow(string, end="\n"):
    if not isinstance(string, str):string = str(string)
    print('\033[93m' + string + '\033[0m', end=end)

def printBlue(string, end="\n"):
    if not isinstance(string, str): string = str(string)
    print('\033[94m' + string + '\033[0m', end=end)

class Writer(object):
    """Create an object with a write method that writes to a
    specific place on the screen, defined at instantiation.

    This is the glue between blessings and progressbar.
    """
    def __init__(self, location):
        """
        Input: location - tuple of ints (x, y), the position
                        of the bar in the terminal
        """
        self.location = location

    def write(self, string, clearLine=False):
        if clearLine:
            with term.location(*self.location):
                print(" "*50, end="", flush=True)
        with term.location(*self.location):
            print(string, end="", flush=True)
    def flush(self):
        pass
        


# ['/Users/voelkerb/Documents/smartenergy/powermeter', '/Library/Developer/CommandLineTools/Library/Frameworks/Python3.framework/Versions/3.7/lib/python37.zip', '/Library/Developer/CommandLineTools/Library/Frameworks/Python3.framework/Versions/3.7/lib/python3.7', '/Library/Developer/CommandLineTools/Library/Frameworks/Python3.framework/Versions/3.7/lib/python3.7/lib-dynload',  '/Library/Developer/CommandLineTools/Library/Frameworks/Python3.framework/Versions/3.7/lib/python3.7/site-packages', ]
# '/usr/local/Cellar/python/3.7.4_1/Frameworks/Python.framework/Versions/3.7/lib/python37.zip', '/usr/local/Cellar/python/3.7.4_1/Frameworks/Python.framework/Versions/3.7/lib/python3.7', '/usr/local/Cellar/python/3.7.4_1/Frameworks/Python.framework/Versions/3.7/lib/python3.7/lib-dynload', '/usr/local/lib/python3.7/site-packages', '/usr/local/lib/python3.7/site-packages/av-6.2.0-py3.7-macosx-10.14-x86_64.egg',]

WAITTIME = 100

def initParser():
    parser = argparse.ArgumentParser(description="Update all powermeters. Powermeters are searched using mdns. Update command for one device must be passed as argument.")
    parser.add_argument("type", type=str, choices=devGetter.possibleTypes,     # store in 'list'.
                        help="Device type to update: PowerMeter or SmartMeter")
    parser.add_argument("firmware", type=str,
                        help="Path to firmware.bin")
    devGetter.addDeviceCommandLineArgs(parser, deviceType=False, verify=False)
    parser.add_argument("--password", type=str, default="energy",
                        help="OTA Password to use.")
    parser.add_argument("--port", type=int, default=3232,
                        help="Port to use for OTA, standard=3232.")
    parser.add_argument("--force", action="store_true", 
                        help="Force an update independent of the device type. USE WITH CAUTION!")
    parser.add_argument("--no_interactive_shell", action="store_true", 
                        help="If shell does not allow interaction")
    parser.add_argument("--max_parallel", type=int, default=20,
                        help="Maximum parallel uploads at a time")
    parser.add_argument("-v", "--verbose", action="count", default=0,
                        help="Increase output verbosity")
    return parser
    
# _______________Can be called as main__________________
if __name__ == '__main__':
    parser = initParser() 
    args = parser.parse_args()
        
    # We need the bin file not the elf
    args.firmware = args.firmware.rstrip(".bin").rstrip(".elf") + ".bin"

    types = args.type
    if args.force:
        types = devGetter.possibleTypes
    deviceList = devGetter.getDevices(devArguments=args.deviceList, deviceTypes=types, verbose=args.verbose-2, verify=not args.force, selectionProcess=not args.no_interactive_shell, searchMore=not args.no_interactive_shell and args.more)

    
    # Catch control+c
    # Get external abort
    def aborted(signal, frame):
        print("Bye Bye from " + str(os.path.basename(__file__)))
        cursor.show()
        sys.exit(0)
    signal.signal(signal.SIGINT, aborted)

    if len(deviceList) <= 0:
        sys.exit()
    else:
        if args.force:
            printYellow("YOU ARE IN FORCE UPDATE MODE, TYPE HAS NOT BEEN CHECKED. USE WITH CAUTION!!")
        if not args.no_interactive_shell:
            printBlue("Press enter to update these devices. Ctr-c to cancel")
            c = input()

    # devices = [PowerMeter(updateInThread=True, a
    #                   ip=dev["ip"], port=dev["port"], useUDP=False, portUDP=5323+i, #  port=54322, stream=True,
    #                   samplingRate=dev["sr"], measures="v,i".split(','),
    #                   name=dev["name"],
    #                   verbose=args.verbose>1)
    #                   for i, dev in enumerate(deviceList)]  

    term = Terminal()
    if not args.no_interactive_shell:
        cursor.hide()
        
    currentThreads = 0
    progressDict = {}
    host_ip = "0.0.0.0"
    host_port = random.randint(10000, 50000)
    for dev in deviceList:
        progressDict[dev["ip"]] = {}
        progressDict[dev["ip"]]["state"] = "idle"
        progressDict[dev["ip"]]["name"] = dev["name"]
        progressDict[dev["ip"]]["lastProg"] = 0
    # for dev in devices:
    #     progressDict[dev.ip] = {}
    #     progressDict[dev.ip]["state"] = "idle"
    #     progressDict[dev.ip]["name"] = dev.name
    #     progressDict[dev.ip]["lastProg"] = 0

    if args.no_interactive_shell:
        def onProgress2(ip, prog):
            global currentThreads
            if progressDict[ip]["lastProg"] != int(prog*100): 
                print(".", end="", flush=True)
            progressDict[ip]["lastProg"] = int(prog*100)
            if prog >= 1.0: 
                progressDict[ip]["state"] = "success"
                print("\n$" + str(progressDict[ip]["name"]) + ": Finished")
                currentThreads -= 1

        def onError2(ip, errorMsg):
            global currentThreads
            if "Error" in errorMsg: 
                print("$" + str(progressDict[ip]["name"]) + ": " + errorMsg)
                progressDict[ip]["state"] = "error" 
                currentThreads -= 1
            # print(line)
            pass 
        while True:
            time.sleep(0.2)
            for dev in deviceList:
                while currentThreads >= args.max_parallel: 
                    time.sleep(0.1)
                # Start every untried or failed one
                if progressDict[dev["ip"]]["state"] in ["idle","error"]:
                    thread = threading.Thread(target=ota.serveWithCB, args=(dev["ip"], host_ip, args.port, host_port, args.password, args.firmware, ota.FLASH, onProgress2, None, onError2))
                    thread.deamon = True
                    progressDict[dev["ip"]]["state"] = "running"
                    progressDict[dev["ip"]]["thread"] = thread
                    progressDict[dev["ip"]]["thread"].start()
                    time.sleep(0.2)
                    currentThreads += 1
                    host_port += 10
            if all(v["state"] == "success" for k, v in progressDict.items()): break
            

        for dev in deviceList:
            progressDict[dev["ip"]]["thread"].join()
    # Interactive shell
    else:

        def onProgress(ip, prog):
            global currentThreads
            progressDict[ip]["pbar"].update(int(prog*100))
            if prog >= 1.0: 
                # progressDict[ip]["pbar"].finish()
                progressDict[ip]["state"] = "success"
                currentThreads -= 1

        def onError(ip, errorMsg):
            global currentThreads
            progressDict[ip]["writerMsg"].write("Msg: " + errorMsg, clearLine=True)
            if "Error" in errorMsg: 
                progressDict[ip]["state"] = "error" 
                currentThreads -= 1
            # print(line)
            pass

        startYPosition = term.height
        startXPosition = 20
        startX2Position = 100
        offestBottom = 3

        for dev in deviceList:
            printYellow(dev["name"][0:startXPosition-2] + ":")
            # print("MSG:")
            # print("Progr:")
        print("\n"*offestBottom)

        # Make enough room for progress bars
        # for dev in devices: 

        for i, dev in enumerate(deviceList):
            startPosition = term.height-len(deviceList)-1 + i - offestBottom -1
            writerMsg = Writer((startX2Position, startPosition))
            writerProg = Writer((startXPosition, startPosition))
            pbar = ProgressBar(fd=writerProg)
            progressDict[dev["ip"]]["writerMsg"] = writerMsg
            writerMsg.write("Waiting")
            progressDict[dev["ip"]]["writerProg"] = writerProg
            progressDict[dev["ip"]]["pbar"] = pbar
            pbar.start()

            progressDict[dev["ip"]]["state"] = "idle"
            host_port += 10
            startPosition += 1
        endWriter = Writer((0, term.height-len(deviceList) - offestBottom+1))
        
        while True:
            time.sleep(0.2)
            for dev in deviceList:
                while currentThreads >= args.max_parallel: 
                    time.sleep(0.1)
                # Start every untried or failed one
                if progressDict[dev["ip"]]["state"] in ["idle","error"]:
                    thread = threading.Thread(daemon=True, target=ota.serveWithCB, args=(dev["ip"], host_ip, args.port, host_port, args.password, args.firmware, ota.FLASH, onProgress, onError, onError))
                    progressDict[dev["ip"]]["pbar"].start()
                    progressDict[dev["ip"]]["pbar"].update(0)
                    progressDict[dev["ip"]]["state"] = "running"
                    progressDict[dev["ip"]]["thread"] = thread
                    progressDict[dev["ip"]]["thread"].start()
                    currentThreads += 1
                    host_port += 10
                    time.sleep(0.2)
            if all(v["state"] == "success" for k, v in progressDict.items()): break
            

        for dev in deviceList:
            progressDict[dev["ip"]]["thread"].join()
        time.sleep(1)
        endWriter.write("\r\n")
        endWriter.flush()
        
        print("")
        cursor.show()
    print("Bye Bye from " + str(os.path.basename(__file__)))
