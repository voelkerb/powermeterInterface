
import os
import sys
import time

import argparse
import signal
import struct

from powermeter.smartBlueline import SmartBlueline
from .powerMeter import PowerMeter
from .smartMeter import SmartMeter
from .smartDevice import SmartDevice
import os
import threading
import signal
import re
# Import top level
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from network.network.mDNS import mDNS

possibleTypes = [PowerMeter.TYPE, SmartMeter.TYPE, SmartDevice.TYPE, SmartBlueline.TYPE]

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

def atoi(text):
    return int(text) if text.isdigit() else text

def natural_keys(mtext):
    '''
    alist.sort(key=natural_keys) sorts in human order
    http://nedbatchelder.com/blog/200712/human_sorting.html
    (See Toothy's implementation in the comments)
    '''
    text = mtext
    if isinstance(text, list) or isinstance(text, tuple):
        text = mtext[0]
    print( atoi(c) for c in re.split(r'(\d+)', text) )
    return [ atoi(c) for c in re.split(r'(\d+)', text) ]

def getDeviceNames(verbose=0):
    
    # Get all available mdns devices
    mdns = mDNS()
    devices = []
    # TODO:
    # devices = mdns.searchForService("_elec", "_tcp", waittime=0.5, reachable=True, verbose=verbose>0)
    devices = mdns.searchForService("_arduino", "_tcp", waittime=0.5, reachable=True, verbose=verbose>0)
    devices = list(set(devices))
    devices.sort(key=lambda x: x[0])
    #  All found devices are reachable
    devices = [{"name":cleanName(x[0]), "ip":x[1], "device":cleanDeviceName(x[0]), "reachable":True} for x in devices]
    return devices

def typeByMDNSName(device, types):
    if "name" in device and device["name"] is not None:
        for ttype in types:
            if ttype.upper() in device["name"].upper():
                return ttype
    if "ip" in device and device["ip"] is not None:
        for ttype in types:
            if ttype.upper() in device["ip"].upper():
                return ttype
    if "device" in device and device["device"] is not None:
        for ttype in types:
            if ttype.upper() in device["device"].upper():
                return ttype
    return SmartDevice.TYPE # unknown type

def typeByMDNSNames(devices, types):
    for dev in devices:
        dev["type"] = typeByMDNSName(dev, types)
    return devices

def verifyTypes(devices, types, verbose=0):
    mss = [PowerMeter(updateInThread=False,
                      ip=dev["ip"], port=54321, useUDP=False, portUDP=5323+i, #  port=54322, stream=True,
                      measures="v,i".split(','),
                      name=dev["name"].rstrip(".local"),
                      verbose=verbose>1, logger=None if verbose < 1 else print)
                      for i, dev in enumerate(devices)]
    # is dictionary of "ip":type
    verifiedDevices = verifyDeviceTypes(mss, types, verbose=verbose)

    for ms in mss:
        # ms.stop()
        ms.kill()
    for dev in devices: 
        dev["type"] = verifiedDevices[dev["ip"]]["type"]
        dev["device"] = verifiedDevices[dev["ip"]]["device"]
     # Sort again
    devices.sort(key=lambda x: x["name"])
    return devices
    

seenDevices = 0
devicesSeen = []
def verifyDeviceTypes(mss, types, verbose=0):
    global seenDevices, devicesSeen
    deviceTypes = {}

    numDevices = len(mss)
    seenDevices = 0
    def sysType(device, info):
        global seenDevices, devicesSeen
        # print(device.name + ": " + info["type"])
        theDevice = "unknowndevice"
        if "name" in info: theDevice= info["name"]
        if "type" in info: theType = info["type"]
        else: return
        if theDevice not in devicesSeen or theDevice == "unknowndevice":
            seenDevices += 1
            devicesSeen.append(theDevice)
        if verbose>2:
            printYellow(device.name + ": ", end="")
            print(theType)
        foundType = theType
        for ttype in types:
            if ttype.upper() in theType.upper():
                foundType = ttype
                break
        deviceTypes[device.ip] = {"type":foundType, "device":theDevice}
            
    for ms in mss:
        ms.getType(sysType)
    start = time.time()
    while seenDevices < numDevices and time.time() - start < 5:
        for ms in mss:
            ms.update()
        time.sleep(0.001)
    for ms in mss:
        if ms.ip not in deviceTypes: deviceTypes[ms.ip] = {"type":SmartDevice.TYPE, "device":"unknowndevice"}  # unknown type
    return deviceTypes

mdns = mDNS()
def addReachability(deviceList):
    for dev in deviceList:
        if "reachable" not in dev:
            reachable, _ = mdns.pingResolve(dev["ip"])
            dev["reachable"] = reachable
    return deviceList


def testReachability(deviceList):
    addReachability(deviceList)
    if not all([dev["reachable"] for dev in deviceList]):
        for dev in deviceList:
            if not dev["reachable"]:
                printRed("Device " + dev["name"] + " @ IP: " + dev["ip"] + " not reachable")
        return False
    return True

def sortByType(deviceList):
    deviceList.sort(key=lambda k: (k['type']), reverse=True)
    # deviceList.sort(key=lambda k: (k['name']))
    return deviceList

def terminalSelectionProcess(deviceList, findAgainPossible=False):
    if not len(deviceList) > 0:
        if findAgainPossible: return [], False
        else: return []
    text = ("Press ENTER to continue with all devices.\r\n"
            "Deselect specific devices e.g.: -2,-4,-7\r\n"
            "Select specific devices e.g.: 1,3,5,6\r\n")
    if findAgainPossible:
        text += "Search again a/A\n"
    text += ("Press r/R to reset\n"
             "e/E to cancel Or press CTR-C to exit program")
 
    # Show devices
    printPink("\nAvailable Devices:")
    printDevices(deviceList, ttype=True, device=True)
    printBlue(text)
    oldList = deviceList
    c = input()
    while c:
        if c in ["r","R"]:
            deviceList = oldList
        elif c in ["e","E"]:
            printPink("\nExit\n")
            if findAgainPossible: return [], False
            else: return []
        elif findAgainPossible and c in ["a","A"]:
            return deviceList, True
        else:
            try:
                indices = [int(i) for i in c.split(",")]
            except ValueError:
                printRed("Format error: " + c)
                c = input()
                continue
            if not (all([i >= 0 for i in indices]) or all([i <= 0 for i in indices])):
                printRed("You cannot mix negative and positive indices")
                c = input()
                continue
            try:
                removeFirst = False
                if "-0" in c:
                    removeFirst = True
                    if 0 in indices: indices.remove(0)
                if not removeFirst and len(indices) > 0 and all([i >= 0 for i in indices]):
                    deviceList = [deviceList[i] for i in indices]
                elif len(indices) > 0 and all([i <= 0 for i in indices]):
                    indices = [abs(i) for i in indices]
                    deviceList = [deviceList[i] for i in range(len(deviceList)) if i not in indices]
                if removeFirst:
                    deviceList = deviceList[1:]
            except IndexError:
                printRed("Index error")
                c = input()
                continue
        printPink("\nSelected Devices:")
        printDevices(deviceList, ttype=True, device=True)
        printBlue(text)
        c = input()
    if findAgainPossible:
        return deviceList, False
    else:
        return deviceList


def printDevices(deviceList, index=0, name="Name:", device=False, ttype=False, ip=True, port=False, sr=False, reachable=False, headlinePrint=printPink, devicePrint=print):
    formatter = "{:<3}{:<25}"
    headline = ["#", name]
    keys = ["name"]
    types = []
    afterType = False
    if ttype and "type" in deviceList[-1]: 
        afterType = True
        deviceList = sortByType(deviceList)
        types = list(set([dev["type"] for dev in deviceList]))
        types.sort(reverse=True)
    if device and "device" in deviceList[-1]: 
        formatter += "{:<25}"
        headline.append("Device:")
        keys.append("device")
    if ip and "ip" in deviceList[-1]: 
        formatter += "{:<25}"
        headline.append("IP:")
        keys.append("ip")
    if port and "port" in deviceList[-1]: 
        formatter += "{:<10}"
        headline.append("Port:")
        keys.append("port")
    if sr and "sr" in deviceList[-1]:  
        formatter += "{:<12}"
        headline.append("Samplingrate:")
        keys.append("sr")
    if reachable and "reachable" in deviceList[-1]:  
        formatter += "{:<10}"
        headline.append("Reachable:")
        keys.append("reachable")
    if (afterType):
        for theType in types:
            headline[1] = theType + ":"
            headlinePrint(formatter.format(*headline))
            for dev in [dev for dev in deviceList if dev["type"] == theType]:
                devicePrint(formatter.format(*[index] + [dev[key] for key in keys]))
                index += 1
    else:
        headlinePrint(formatter.format(*headline))
        for dev in deviceList:
            devicePrint(formatter.format(*[index] + [dev[key] for key in keys]))
            index += 1

def setDefaultArg(deviceList):
    # Add "device", "name", "ip", "sr", "port"
    for dev in deviceList:
        if "type" not in dev: dev["type"] = SmartDevice.TYPE # unknwon type
        if "name" not in dev:
            if "ip" not in dev: dev["name"] = "unknown"
            else: dev["name"] = dev["ip"].rstrip(".local")
        if "ip" not in dev: 
            if "name" not in dev: dev["ip"] = None
            else: dev["ip"] = dev["name"].rstrip(".local") + ".local"
        if "port" not in dev: dev["port"] = None
        if "sr" not in dev: dev["sr"] = None
        if "device" not in dev or dev["device"] is None:
            if ".local" in dev["name"]: dev["device"] = dev["name"].lower()
            elif ".local" in dev["ip"]: dev["device"] = dev["ip"]
            else: dev["device"] = "UnknownDevice".lower()
    return deviceList

def cleanName(name):
    nname = name
    nname = nname.lstrip(" ").rstrip(" ").rstrip(".local")
    return nname

def cleanDeviceName(name):
    nname = cleanName(name)
    nname = nname.lower()
    return nname

def getDeviceNamesHost(devicetypes, testNumber=30, verbose=0):
    _devicetypes = devicetypes
    if not isinstance(devicetypes, list): _devicetypes = list(devicetypes)
    names = []
    for devicetype in _devicetypes:
        #  This is the unknown type, we can skip
        if devicetype == SmartDevice.TYPE: continue
        # Smartmeters have name smartmeter<XXX> XXX is a number with leading 0s
        elif devicetype == SmartMeter.TYPE: 
            names.extend([devicetype.lower() + str(i).zfill(3) for i in range(testNumber)])
        # Powermeters have name powermeter<XX> XX is a number with leading 0s
        elif devicetype == PowerMeter.TYPE: 
            names.extend([devicetype.lower() + str(i).zfill(2) for i in range(testNumber)])
        else: 
            names.extend([devicetype.lower() + str(i).zfill(2) for i in range(testNumber)])
    reachAndIPs = mdns.parallelPingResolve(names, waittime=0.5, verbose=verbose)

    devices = [{"name":cleanName(name), "ip":reachAndIP[1], "device":cleanDeviceName(name), "reachable":reachAndIP[0]} for reachAndIP, name in zip(reachAndIPs, names) if reachAndIP[0]]
    return devices

def splitDevArg(rawName):
    name = rawName.split("=")[0].split(":")[0].split("@")[0]
    name = cleanName(name)
    if len(rawName.split("=")) > 1:
        ipName = rawName.split("=")[1].split(":")[0].split("@")[0]
    else:
        ipName = rawName.split("=")[0].split(":")[0].split("@")[0]
    reachable, ip = mdns.pingResolve(ipName, waittime=0.5)
    # print(name)
    # print(ipName)
    # print(reachable)
    # print(ip)
    device = None
    if ".local" in ipName or any([ttype.upper() in ipName.upper() for ttype in possibleTypes]):
        device = cleanDeviceName(ipName)
    # Default port would be used
    port = None 
    if ":" in rawName: port = int(rawName.split(":")[-1].split("@")[0])
    # Default sr would be used
    sr = None
    if "@" in rawName: sr = int(rawName.split("@")[-1])
    return {"name":name, "ip":ip, "port":port, "sr":sr, "reachable":reachable, "device":device}

def addDeviceCommandLineArgs(parser, verify=True, deviceType=True, searchMore=True):
    parser.add_argument("-d", "--device", type=str, 
                        dest='deviceList',     # store in 'list'.
                        default=[],
                        action='append',
                        help="For each device specify path and name as <name>=<path>[:<port>][@<samplingrate>] (e.g. -d \"Coffe Machine\"=192.168.0.2")
    if searchMore:
        parser.add_argument("--more", action="store_true",     # store in 'list'.
                            help="Search for more (not only devicelist)")
    if deviceType:
        parser.add_argument("--type", type=str, choices=possibleTypes,     # store in 'list'.
                            default=possibleTypes,
                            help="Device type: PowerMeter or SmartMeter")
    if verify:
        parser.add_argument("--verify_type", action="store_true",
                            dest="verifyType",
                            help="Send info request and check for correct type, otherwise only name resolving is used")

def getDevices(deviceTypes=possibleTypes, devArguments=[], verbose=0, verify=False, selectionProcess=True, searchMore=False):
    # Catch control+c
    # Get external abort
    def aborted(signal, frame):
        print("Aborted during selection")
        sys.exit(0)
    signal.signal(signal.SIGINT, aborted)

    # If type is selected it is a string, we need a list here
    if not isinstance(deviceTypes, list): deviceTypes = [deviceTypes] 

    # Split args if devices are given directly
    deviceList = [splitDevArg(dev) for dev in devArguments]
    # No selection process if given dirctly
    alreadySelected = True
        
    deviceList = setDefaultArg(deviceList) 
    # As long as this is true we search again for devices
    while True:
        findAgain = False
        # If we do not specify anything, search for devices
        if len(deviceList) == 0 or searchMore:
            # Original names so far
            origNameAndDev = [(v['name'], v['device']) for v in deviceList]

            # Trying to get device names by router host resolve
            deviceNamesAndIp1 = getDeviceNamesHost(deviceTypes)
            # Gets device name and ip by mdns resolve
            deviceNamesAndIp2 = getDeviceNames(verbose=verbose)
            # Sets default arguments for samplignrate etc
            deviceList.extend(setDefaultArg(deviceNamesAndIp1 + deviceNamesAndIp2))
            
            # Everything lowercase
            # deviceList = list({v['device'].lower():v for v in deviceList}.values())
            # Everything lowercase
            # deviceList = list({v['name'].lower():v for v in deviceList}.values())
            # Go back to originalnames
            if len(deviceList):
                #  Use original possible uppercasename
                for device in deviceList:
                    for name, dev in origNameAndDev:
                        if dev == device["device"]:
                            device["name"] = name

            # Selection process is enabled
            alreadySelected = False

        # Devices should be unique except unknown
        newDeviceList = []
        devices = []
        for dev in deviceList:
            if "unknown" in dev["device"]:
                newDeviceList.append(dev)
            elif dev["device"] not in devices: 
                devices.append(dev["device"])
                newDeviceList.append(dev)
        deviceList = newDeviceList

        deviceList = typeByMDNSNames(deviceList, deviceTypes)
        if not testReachability(deviceList):
            sys.exit()
        if not verify:
            deviceList = [dev for dev in deviceList if dev["type"] in deviceTypes]
        # IF should be verified
        if verify:
            # [print(device) for device in deviceList]
            if verbose > 0: print("Verifying... ")
            deviceList = verifyTypes(deviceList, deviceTypes, verbose=verbose)
            # [print(device) for device in deviceList]
            deviceList = [dev for dev in deviceList if dev["type"] in deviceTypes]
            time.sleep(0.1) # Whyever this is required
            if verbose > 0: print("Verifying... Done ")
        # print()deviceList
        deviceList = sorted(deviceList, key=lambda k: k['device']) 
        # Enable selection
        if selectionProcess and not alreadySelected:
            deviceList, findAgain = terminalSelectionProcess(deviceList, findAgainPossible=True)
        # If we do not swant to find devices again, we finish
        if not findAgain: break
        else: deviceList = []
        
    if not len(deviceList) > 0:
        printPink("No devices found")
    elif alreadySelected:
        printPink("Available Devices:")
        printDevices(deviceList, ttype=True, device=True)
    return deviceList
    

def initParser():
    parser = argparse.ArgumentParser(description="See what the getDevices file can do for you.")
    parser.add_argument("-t", "--type", type=str, choices=["PowerMeter", "SmartMeter"], dest='typeList',     # store in 'list'.
                        default=["PowerMeter", "SmartMeter"],
                        action='append',
                        help="The device type you want to update: PowerMeter or SmartMeter")
    parser.add_argument("--verifyType", action="store_true",
                        help="Send info request and check for correct type, otherwise only name resolving is used for type verification")
    parser.add_argument("-v", "--verbose", action="count", default=0,
                        help="Increase output verbosity")
    return parser
    
# _______________Can be called as main__________________
if __name__ == '__main__':
    parser = initParser()
    args = parser.parse_args()

    # Catch control+c
    # Get external abort
    def aborted(signal, frame):
        print("")
        sys.exit(0)
    signal.signal(signal.SIGINT, aborted)

    # Get device name and ip
    deviceNames = getDeviceNames(verbose=args.verbose)
    devices = setDefaultArg(deviceNames)

    printPink("\nAvailable Devices:")
    printDevices(devices)    

    devices = typeByMDNSNames(devices, args.typeList)

    printPink("\nType according to MDNS name:")
    printDevices(devices, ttype=True)  

    if args.verifyType:
        devices = verifyTypes(devices, args.typeList)
        printPink("\nType with Type Verification:")
        printDevices(devices, ttype=True)  

    devices, _ = terminalSelectionProcess(devices, findAgainPossible=True)

    print("Bye Bye from " + str(os.path.basename(__file__)))
