"""Main File covering measurment system."""
# !/usr/bin/python
import sys
import traceback
import os
from abc import ABCMeta, abstractmethod
import time
import json
import numpy as np
import threading
import serial
import select
import socket
import struct
import platform
import json
import multiprocessing
from multiprocessing import Process, Value
from queue import Queue, Empty
from enum import Enum

# IDENTIFIERS
#: Identifier for voltage 
VOLTAGE = ['v', 'v_l1', 'v_l2', 'v_l3']
#: Identifier for RMS voltage 
VOLTAGE_RMS = ['v_rms', 'v_rms_l1', 'v_rms_l2', 'v_rms_l3']
#: Shortform of voltage 
VOLT = 'v'

#: Identifier for current 
CURRENT = ['i', 'i_l1', 'i_l2', 'i_l3']
#: Identifier for RMS current 
CURRENT_RMS = ['i_rms', 'i_rms_l1', 'i_rms_l2', 'i_rms_l3']
#: Shortform of current 
CUR = 'i'

#: Identifier for line frequency 
FREQUENCY = ['f', 'f_l1', 'f_l2', 'f_l3']
#: Shortform of current 
FREQ = 'f'

#: Identifier for line phase shift 
PHI = ['phi_n', 'phi_l1', 'phi_l2', 'phi_l3']

#: Identifier for active power 
ACTIVE_POWER = ['p', 'p_l1', 'p_l2', 'p_l3']
#: Shortform of active power 
ACTIVE_POW = 'p'

#: Identifier for reactive power 
REACTIVE_POWER = ['q', 'q_l1', 'q_l2', 'q_l3']
#: Shortform of reactive power 
REACTIVE_POW = 'q'

#: Identifier for apparent power 
APPARENT_POWER = ['s', 's_l1', 's_l2', 's_l3']
#: Shortform of apparent power 
APPARENT_POW = 's'


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

class LogLevel(Enum):
    """
    Log Level Enumeration Type.
    
    This enumeration is based on a basic integer enumeration. Higher integer numbers
    relate to a higher log level. If you e.g. set the log level to the highest type:
    ERROR, only errors will be shown.
    """
    #: Show all log msges
    ALL = 1
    #: Show messages used for debugging. Maybe you need to recompile the Firmware with DEBUG set to true
    DEBUG = 2
    #: Show info messages. Most of the msgs are info msges. These show state changes of the device, or sampling information.
    #: Or basically all things going on in the device.
    INFO = 3
    #: Warning msgs are yet uncricital msg but they indicate that something is going wrong. 
    #: Like drifts during sampling, or unanswered NTP requests
    WARNING = 4
    #: Error msged show critical errors from which the devices typically cannot recover without a restart.
    ERROR = 5

    def fromString(string):
        """
        Convert a string to a loglevel enumeration type.

        :param string:     log level in string representation. (e.g. error/Error or ERROR)
        :type  string:     str

        :return: LogLevel type based on string representation
        :rtype:  LogLevel
        """
        if string.lower() == str(LogLevel.ALL).lower():
            return LogLevel.ALL
        elif string.lower() == str(LogLevel.DEBUG).lower():
            return LogLevel.DEBUG
        elif string.lower() == str(LogLevel.INFO).lower():
            return LogLevel.INFO
        elif string.lower() == str(LogLevel.WARNING).lower():
            return LogLevel.WARNING
        elif string.lower() == str(LogLevel.ERROR).lower():
            return LogLevel.ERROR


    def __str__(self):
        """
        Overloaded to String method

        :return: String representation of a LogLevel type
        :rtype:  str
        """
        return self.name.lower()
    
    def choices():
        return [l.lower() for l in list(LogLevel)]


class SmartDevice(metaclass=ABCMeta):
    r"""
    Metaclass of a device that samples data.
    
    Class which handles a smart device.
    You can create a child class to sample a certain quantity by using
    the same transmission scheme as used by e.g. 
    `PowerMeters <https://projects.informatik.uni-freiburg.de/projects/smartenergy/wiki/PowerMeter>`_

    The child class need to set TYPE, DEFAULT_SR and the AVAILABLE_MEASURES array as:

    .. code-block:: python3


        AVAILABLE_MEASURES = [
                    {"keys": [<Key1>,<Key2>,...], "bytes": <bytesPerSample>, "cmdMeasure": <measures> },
                    ...
                ]
    """

    #: Type of measurement system. Must be set from chilf class.
    TYPE = "Unknown"

    #: Type for general devices which type is not known yet.
    UNKNOWN = "Unknown"

    #: Standard name.
    STANDARD_NAME = "SmartDevice"

    #: Over TCP a keepalive msg (\"!\") is sent in this interval (seconds), -1 to disable
    KEEPALIVE_INTV = 30

    #: Bytes for single value (all encoded as 32 bit floats)
    SINGLE_VALUE_BYTES = 4

    #: Show samplingrate as a lifeness feature
    LIFENESS_SR = False

    #: Need to be implemented by subclass
    AVAILABLE_MEASURES = []
    DEFAULT_SR = -1

    #: Line separator for single measurement
    SERIAL_LINE_SEP = b'\x0d\x0a'

    #: If something string/command/info is received from the
    #: measurement system which should not be treated as data
    #: it has to be send with this prefix
    LINE_PREFIX = b'Info:'
    DATA_PREFIX = b'Data:'

    def __init__(self, name=None, deviceName=None, samplingRate=None, 
                    serialPort=None, portName=None, baudrate=2000000,
                    socketObject=None, ip=None, port=54321, stream=False,
                    useUDP=False, portUDP=54323,
                    updateInThread=False, directInit=True,
                    flowControl=False,
                    logLevel=LogLevel.INFO, verbose=0, logger=print):
        r"""
        Return Smart Device.

        :param name:            Name you give to this device for identification. 
        :type  name:            str, default: None
        :param deviceName:      Devicename that you can give. If you do not provide, it can be set automatically if info is received.
        :type  deviceName:      str, default: None
        :param samplingRate:    Samplingrate used for sampling, if None or nothing is passed, default samplingrate is used (4000).
        :type  samplingRate:    int, default: None
        :param serialPort:      Serialport 
        :type  serialPort:      Serial, default: None
        :param portName:        Path to serialport to use: e.g. ``dev/tty.usbserialXXX`` under unix or ``COMX`` under windows
        :type  portName:        str, default: None
        :param baudrate:        Baudrate of the serialport
        :type  baudrate:        int, default: 2000000
        :param socketObject:    Python socket which is used for TCP connection.
        :type  socketObject:    socket, default: None
        :param ip:              Remote IP address  used for socket communication.
        :type  ip:              str, default: None
        :param port:            Remote port used for TCP communication.
        :type  port:            int, default: 54321
        :param stream:          If ffmpeg streaming should be used directly.
        :type  stream:          bool, default: False
        :param useUDP:          If UDP streaming should be used. Use togeter with portUDP parameter.
        :type  useUDP:          bool, default: False
        :param portUDP:         Remote port for UDP communication. Need to set :attr:`useUDP<powermeter.smartDevice.SmartDevice.useUDP>` to ``True``
        :type  portUDP:         int, default: 54323
        :param updateInThread:  If update function should be called automatically in thread owned by SmartDevice 
        :type  updateInThread:  bool, default: False
        :param directInit:      If connection to device should be inited directly. If set to false, you need to call: :func:`init<powermeter.smartDevice.SmartDevice.init>`
        :type  directInit:      bool, default: True
        :param logLevel:        LogLevel the device is set to directly after init.
        :type  logLevel:        LogLevel, default: :attr:`INFO<powermeter.smartDevice.LogLevel.INFO>`
        :param verbose:         Increase output verbosity.
        :type  verbose:         int, default: 0
        :param logger:          All messages printed with :func:`msPrint<powermeter.smartDevice.SmartDevice.msPrint>` are passed to the logger function.
                                You can basically pass any function that takes a ``str`` as argument.
        :type  logger:          function(str), default: :samp:`print({msg})`
        
        :return:    SmartDevice object.
        :rtype:     SmartDevice
        """
        #: | default: :attr:`STANDARD_NAME<powermeter.smartDevice.SmartDevice.STANDARD_NAME>` 
        #: | The name of the SmartDevice instance given by user: e.g. <Fridge> 
        self.name = self.STANDARD_NAME
        #: | default: :attr:`UNKNOWN<powermeter.smartDevice.SmartDevice.UNKNOWN>` 
        #: | The name of the sampling device 
        self.deviceName = self.UNKNOWN
        self.MEASUREMENT_BYTES = 2*self.SINGLE_VALUE_BYTES
        #: | default: ``[]``
        #: | Array holding frames of sampling data (recarrays). Set framesize to specify how much samples are in each frame
        self.frames = []
        self._currentFrame = None
        #: | default: :attr:`INFO<powermeter.smartDevice.LogLevel.INFO>` 
        #: | Log level of the device, set using function setLogLevel 
        self.logLevelDevice = logLevel

        # Standard values
        #: | default: ``False`` 
        #: | Indicate that the device is currently sampling
        self.sampling = False
        # TODO: Determine what is the difference here
        #: | default: ``0``  
        #: | Number of samples  
        self.samples = 0
        self.totalSamples = 0
        #: | default: ``{}``
        #: | Dictionary holding AVAILABLE_MEASURES entry used for sampling
        self.measurementInfo = {}

        #: | default: ``0``
        #: | Number of samples received.
        #: | Possible connection problem might happen so samples received hold real # samples
        self.samplesReceived = 0
        #: | default: ``0``
        #: | Total number of frames filles, as they might be popped by external module
        self.framesFilled = 0
        #: | default: ``None``
        #: | Actual start time of sampling sent from device.
        #: | This might be None, therefore use getStartTs instead
        self.startTs = None
        #: | default: ``None``
        #: | Time of first received sample
        self.caStartTs = None
        #: | default: ``None``
        #: | Actual stop time of sampling sent from device.
        #: | This might be None, therefore use getStopTs instead
        self.stopTs = None
        #: | default: ``None``
        #: | Time when stop is called
        self.caStopTs = None
        #: | default: ``0``
        #: | Time when last message was received 
        self.lastMessageReceived = 0
        #: | default: ``None``
        #: | Dictionary with information about the sampling process (startTS/StopTS et.c.)
        self.deviceSamplingInfo = None
        #: | default: ``None``
        #: | Dictionary holding info about measurement system 
        self.deviceInfo = None
        # Last time a keepalive msg is sent
        self._lastKeepAlive = time.time()
        #: | default: ``0``
        #: | If you use slot sending, the total number of slots used
        self.totalSlots = 0
        #: | default: ``0``
        #: | The explicit slot used for this device
        self.slot = 0

        # Private members required
        self._sendFunc = None
        self._sysInfoCB = None
        self._logInfoCB = None
        self._lastDeadConnectionInfo = time.time()+10
        self._lastDeadConnectionCB = time.time()+10

        #: | default: ``None``
        #: | CB if sampling process has started succesfully
        self.samplingStartedCB = None
        #: | default: ``None``
        #: | CB if connection has be reset. Passed parameter (SmartDevice, ResetReason)
        self.connectionResetCB = None
        #: | default: ``None``
        #: | CB if more than XX seconds no data received. Passed parameter (SmartDevice, NumberSeconds)
        self.deadConnectionCB = None
        #: | default: ``False``
        #: | Bool to hold if connection is broken
        self.brokenConnection = False
        
        #: | default: ``print``
        #: | logging module, must be a print function, e.g. print() or logger.info()
        self.logger = logger
        #: | default: ``0``
        #: | Increase output verbosity
        self.verbose = verbose
        
        self.name = self.name.rstrip(".local")
        if deviceName is not None:
            self.deviceName = deviceName

        #: | default: ``4000``
        #: | Sampling rate of the device
        self.samplingRate = 4000
        #  Standard samplingrate and overloading
        if samplingRate is not None: 
            self.samplingRate = samplingRate
        #: frameSize 
        self.frameSize = self.samplingRate

        #:| default: ``None`` 
        #:| IP address of device if TCP or UDP sampling is used 
        self.ip = ip

        #:| default: ``54321`` 
        #:| Port of device for TCP connection 
        self.port = 54321

        if port is not None: self.port = port

        #: | default: ``None``
        #: | Name of serialport
        self.portName = portName

        #: | default: ``2000000``
        #: | Baudrate of serialport
        self.baudrate = baudrate

        #: | default: ``False``
        #: | If UDP should be used to send data
        self.useUDP = useUDP

        #: | default: ``54323``
        #: | Port used for UDP connection
        self.portUDP = portUDP

        #: | default: ``None``
        #: | Serialport object. Either passed to init, or inited in module
        self.serial = serialPort

        #: | default: ``None``
        #: | Socket object. Either passed to init, or inited in module
        self.socket = socketObject
        
        if self.socket is not None:
            self.ip = self.socket.getpeername()[0]
            self.port = self.socket.getsockname()[1]
        

        if self.portName is None and self.ip is None:
            sys.exit("Either pass serial or socket to SmartDevice")

        # Required private members
        self._serialProc = None
        self._lastPacket = -1
        self._thread = None

        #: | default: ``False``
        #: | If flow control should be used for data.
        #: | If enabled, this module has to send CTS: <true/false> to enable/disable data flow
        self.flowCtr = flowControl

        self._udpthread = None
        self._threadRunning = False

        #: | default: ``"serial"``
        #: | Connection type (str enumeration type)
        self.conType = "serial"
        if self.ip is not None:
            if self.useUDP: self.conType = "udp"
            elif stream: self.conType = "stream"
            else: self.conType = "tcp"


        if self.conType == "serial":
            self.sendFunc = self._sendSerialMsg
            self._updateFunc = self._updateSerial

        elif self.conType == "udp":
            self.sendFunc = self._sendSocketMsg
            self._updateFunc = self._updateUDP

        elif self.conType == "tcp":
            self.sendFunc = self._sendSocketMsg
            self._updateFunc = self._updateTCP

        elif self.conType == "stream":
            self.sendFunc = self._sendSocketMsg
            self._updateFunc = self._updateStream
        
        #: | default: ``False``
        #: | Indicate that device is inited.
        self.inited = False
        if directInit:
            self.inited = self.init()

         # Update in thread
        if updateInThread:
            self._thread = threading.Thread(target=self._updateThread, daemon=True)
            self._threadRunning = True
            self._thread.start()
        
    def init(self):
        """
        Init the connection to the measurement device.

        Depending on used connection type, either the 
        serialport, udp- or tcp socket is opened.

        :return: ``True`` if successfull, ``False`` if not
        :rtype: bool
        """
        if self.conType == "serial":
             self.inited = self._initSerial()

        elif self.conType == "udp":
            self.inited = self._initSocket()
            self.inited = self.inited and self._initUDP()

        elif self.conType == "tcp":
            self.inited = self._initSocket()

        elif self.conType == "stream":
            self.inited = True

        # if self.inited:
        #     self.setLogLevel(self.logLevelDevice)
        self.brokenConnection = not self.inited
        return self.inited

    def sendMsg(self, msg):
        """
        Send a raw message to the device.

        :param msg: message to send to the device. NOTE: newline will be added!
        :type  msg: str
        """
        if not self.inited: self._notInitedError
        self.sendFunc(msg)

    def systemInfo(self, sysInfoCB=None):
        """
        Return information about the measurement module.

        :param sysInfoCB: Callback to call if info has been received and parsed. Parameter: (SmartDevice, InfoDict)
        :type  sysInfoCB: function(SmartDevice, dict)
        
        :return:    If info has been received alread, dict is passed directly.
        :rtype:     dict, default: None
        """
        if not self.inited: self._notInitedError
        # Use exisitng info if we already have it
        if self.deviceInfo is not None:
            if sysInfoCB is not None:
                sysInfoCB(self, self.deviceInfo)
            return self.deviceInfo
        # Otherwise ask for info
        self._sysInfoCB = sysInfoCB
        self.sendFunc("{\"cmd\":\"info\"}")
        return {}

    def setMeasures(self, measures):
        """
        Set the measures to use for sampling.

        :param measures: Callback to call if info has been received and parsed. Parameter: (SmartDevice, InfoDict)
        :type  measures: function(SmartDevice, dict)
        """
        measurementI = None
        for availableMeasureSet in self.AVAILABLE_MEASURES:
            if all(item in availableMeasureSet["keys"] for item in measures): 
                measurementI = availableMeasureSet
                break
        if measurementI is None: 
            self.logger("SmartDevice cannot handle mesurements: " + str(measures)) 
        else:
            self.measurementInfo = measurementI

    def setLogLevel(self, newLevel):
        """
        Set the log level of the device.
        
        :param newLevel: Log level (enumeration type) to apply
        :type  newLevel: LogLevel
        """
        if not self.inited: self._notInitedError
        self.logLevelDevice = newLevel
        self.sendFunc("{\"cmd\":\"log\",\"level\":\"" + str(self.logLevelDevice) + "\"}")

    def restart(self):
        """Restarts the measurement module."""
        if not self.inited: self._notInitedError
        self.sendFunc("{\"cmd\":\"restart\"}")
        self.onConnectionReset(reason="restarted")

    def ntpSync(self, performInBg=False):
        """Starts an NTP time sync.

        :param performInBg: If device should wait for result of NTP request and report back.
        :type  performInBg: bool, default: False
        """
        if not self.inited: self._notInitedError
        cmd = "{\"cmd\":\"ntp\""
        if performInBg:
            cmd += ",\"payload\":{\"bg\":" + str(performInBg).lower() + "}"
        cmd += "}"
        self.sendFunc(cmd)

    def factoryReset(self):
        """Put all settings back to default."""
        if not self.inited: self._notInitedError
        self.sendFunc("{\"cmd\":\"factoryReset\"}")
        self.onConnectionReset(reason="factory reset")

    def basicReset(self):
        """Put all settings back to default, except the it's device name."""
        if not self.inited: self._notInitedError
        self.sendFunc("{\"cmd\":\"basicReset\"}")
        self.onConnectionReset(reason="basic reset")
        
    def setMDNS(self, name):
        """
        Set name and MDNS name of the measurement module.
        
        :param name: name to be set
        :type  name: str
        """
        if not self.inited: self._notInitedError
        self.sendFunc(str("{\"cmd\":\"mdns\", \"payload\":{\"name\":\"" + name + "\"}}"))
        
    def addWifi(self, ssid, pwd):
        """
        Add a wifi AP the device can connect to.
        
        :param ssid: Station Name of Access Point to be added
        :type  ssid: str

        :param pwd: Password of Access Point to be added
        :type  pwd: str
        """
        if not self.inited: self._notInitedError
        self.sendFunc(str("{\"cmd\":\"addWifi\", \"payload\":{\"ssid\":\"" + ssid + "\",\"pwd\":\"" + pwd + "\"}}"))

    def delWifi(self, ssid):
        """
        Remove a wifi AP.

        :param ssid: Station Name of Access Point to be removed.
        :type  ssid: str
        """
        if not self.inited: self._notInitedError
        self.sendFunc(str("{\"cmd\":\"delWifi\", \"payload\":{\"ssid\":\"" + ssid + "\"}}"))

    def clearLog(self):
        """Clears log of the measurement module."""
        if not self.inited: self._notInitedError
        self.sendFunc("{\"cmd\":\"clearLog\"}")

    def getLog(self, logInfoCB=None):
        r"""
        Get log (file) from the measurement module.
        
        :param sysInfoCB: Dictionary with log information under key \"msg\".
        :type  sysInfoCB: function(SmartDevice, dict), default: None
        """
        if not self.inited: self._notInitedError
        self._logInfoCB = logInfoCB
        self.sendFunc("{\"cmd\":\"getLog\"}")

    def getType(self, sysInfoCB=None):
        """
        Return type of the measurement module.
        If this has not been passed by the device yet, a request is sent.
        You can pass CB to receive this request.
        
        :param sysInfoCB: Dictionary with type information under key \"type\".
        :type  sysInfoCB: function(SmartDevice, dict), default: None
        
        :return:    Device type or standard type
        :rtype:     str
        """
        if not self.inited: self._notInitedError
        if self.deviceInfo is not None and "type" in self.deviceInfo:
            if sysInfoCB is not None:
                sysInfoCB(self, self.deviceInfo)
            return self.systemInfo("type")
        self._sysInfoCB = sysInfoCB
        self.sendFunc("{\"cmd\":\"info\"}")
        return self.TYPE

    def setMqttServer(self, server):
        """
        Set a MQTT server for measurement module.
           
        :param server: IP address of MQTT broker. Use '-' to disable mqtt.
        :type  server: str
        """
        if not self.inited: self._notInitedError
        self.sendFunc(str("{\"cmd\":\"mqttServer\", \"payload\":{\"server\":\"" + server + "\"}}"))

    def setStreamServer(self, server):
        """
        Set a Stream server for the device.
        
        :param server: IP address of stream server. Use '-' to disable feature.
        :type  server: str
        """
        if not self.inited: self._notInitedError
        self.sendFunc(str("{\"cmd\":\"streamServer\", \"payload\":{\"server\":\"" + server + "\"}}"))

    def setTimeServer(self, server):
        """
        Set a Time server for the device.
        
        :param server: DNS entry for time server. e.g. time.google.com
        :type  server: str
        """
        if not self.inited: self._notInitedError
        self.sendFunc(str("{\"cmd\":\"timeServer\", \"payload\":{\"server\":\"" + server + "\"}}"))


    def samplingInfo(self):
        """
        Return sampling info from current or last sampling.

        :return:    Printable info about sampling
        :rtype:     str
        """
        if not self.inited: self._notInitedError
        info = ""

        info += '\033[95m' + self.name + ":" + '\033[0m'
        start = self.getStartTs()
        stop = self.getStopTs()
        if start is not None:
            info += "\n\t" + str(time_format_ymdhms(start)) + " -> "
            if stop is not None:
                info += str(time_format_ymdhms(stop))
            elif self.sampling:
                info += "now"
            else:
                info += "unknown"
        
        if start is not None:
            info += "\n\t-> avg rate: "
            if stop is not None:
                info += str(round(self.totalSamples/(stop-start),3)) + "Hz"
            elif self.sampling:
                info += str(round(self.totalSamples/(time.time()-start),3)) + "Hz"
            else:
                info += "unknown"

        
        info += "\n\tFrames: " + str(self.framesFilled) + " frames"
        info += "\n\tTotal: " + str(self.totalSamples) + " samples" 
        info += "\n\tReceived: " + str(self.samplesReceived) + " samples"
        lost = self.totalSamples - self.samplesReceived
        if lost > 0:
            info += "\n\t" + '\033[93m' + "Lost: " + str(lost) + " samples, or " + str(round(lost/self.totalSamples*100.0,3)) + "%" + '\033[0m'
        # Make sure to stop here if info not set
        if self.deviceSamplingInfo is None: return info
        if 'samples' in self.deviceSamplingInfo:
            info += "\n\t\033[94m'According to Device:'\033[0m" 
            info += "\n\tTotal: " +  str(self.deviceSamplingInfo['samples']) + " samples"
        if 'sent_samples' in self.deviceSamplingInfo:
            info += "\n\tSent: " +  str(self.deviceSamplingInfo['sent_samples']) + " samples"
        if 'sample_duration' in self.deviceSamplingInfo:
            info += "\n\tDuration: " + str(round(self.deviceSamplingInfo['sample_duration']/1000.0,3)) + " s"
        if 'avg_rate' in self.deviceSamplingInfo:
            info += "\n\t-> avg rate: " + str(round(float(self.deviceSamplingInfo['avg_rate']),3)) + "Hz"
        if self.startTs is not None:
            info += "\n\t" + str(time_format_ymdhms(self.startTs))
            if self.stopTs is not None:
                info += " -> " + str(time_format_ymdhms(self.stopTs))
            elif 'sample_duration' in self.deviceSamplingInfo:
                info += " -> " + str(time_format_ymdhms(float(self.deviceSamplingInfo['start_ts'])+float(self.deviceSamplingInfo['sample_duration']))) 
            if self.startTs is not None and self.stopTs is not None and 'samples' in self.deviceSamplingInfo:
                info += "\n\t-> avg rate: " + str(round(self.deviceSamplingInfo['samples']/(self.stopTs-self.startTs),3)) + "Hz"
        return info

    def getAvgRate(self):
        """
        Return avg sampling rate based on start timestamp/stop timestamp or now 
        and sent number of samples.

        :return:    samplingrate rounded with 3 decimals
        :rtype:     float
        """
        start = self.getStartTs()
        if start is None or self.totalSamples == 0: return -1
        stop = self.getStopTs()
        if stop is None: stop = time.time()
        return round(self.totalSamples/(stop-start),3)

    def getStopTs(self):
        """
        Get the stop timestamp.

        :return:    timestamp when sampling stopped or None, if not stopped yet
        :rtype:     timestamp or None
        """
        #  Prefere device time info
        if self.stopTs is not None: return self.stopTs
        elif self.caStopTs is not None: return self.caStopTs
        else: return None

    def getStartTs(self):
        """
        Get the start timestamp.

        :return:    timestamp when sampling started or None, if not started yet.
        :rtype:     timestamp or None
        """
        #  Prefere device time info
        if self.startTs is not None: return self.startTs
        elif self.caStartTs is not None: return self.caStartTs
        else: return None
        
    def start(self, ts=None, sendSlot=None, ntpConfidenceMs=None):
        r"""
        Start the sampling.
        
        :param ts:              Timestamp at which sampling should be started.
                                Is not allowed to be in the past, and must be <= 20s in the future.
        :type  ts:              timestamp, default: None
        :param sendSlot:        List with slot information [slot, totalSlots]
                                The device will only sample ``if now.seconds%totalSlots == slot``
        :type  sendSlot:        list, default: None
        :param ntpConfidenceMs: Time in milliseconds which is used as minimum NTP confidence.
                                Sampling is started if the ntp request took smaller that this time.
        :type  ntpConfidenceMs: int, default: None
        """
        if not self.inited: self._notInitedError

        self._lastDeadConnectionInfo = time.time()
        self._lastDeadConnectionCB = time.time()
        self.lastMessageReceived = time.time()

        self.sampling = True

        #  On serial stop sampling beforehand
        if self.conType == "serial":
            self.sendFunc("{\"cmd\":\"stop\"}")
            time.sleep(0.1)
            # TODO: Flush serial
        
        # if self._thread is not None:
        #     self._threadRunning = True
        #     self._thread.start()

        if self.conType == "stream": 
            if self._connectSocket() is False:
                sys.exit("Error Socket " + str(self.ip) + " cannot be opened")
            return

        
        startCMD = "{\"cmd\":\"sample\",\"payload\":{\"type\":\"<type>\",\"rate\":<rate>,\"flowCtr\":<flow>"

        if self.conType == "serial": 
            startCMD = startCMD.replace("<type>", "Serial")
        elif self.conType == "tcp": 
            startCMD = startCMD.replace("<type>", "TCP")
        elif self.conType == "udp": 
            startCMD += ",\"port\":" + str(self.portUDP)
            startCMD = startCMD.replace("<type>", "UDP")
            # "Data:" + 4 bytes packet # + 2 bytes chunk size 
            self._udpPacketSize = int(512/(len(self.MEASUREMENTS)*self.SINGLE_VALUE_BYTES))*(len(self.MEASUREMENTS)*self.SINGLE_VALUE_BYTES)+6+len(self.DATA_PREFIX)


        startCMD = startCMD.replace("<rate>", str(self.samplingRate))
        # TODO: remove if every powermeter is on newest software
        if not self.flowCtr:
            startCMD = startCMD.replace(",\"flowCtr\":<flow>", "")
        else:
            startCMD = startCMD.replace("<flow>", str(self.flowCtr).lower())

    
        if sendSlot is not None:
            secondStart = sendSlot[0]
            secondTotal = sendSlot[1]
            self.slot = secondStart
            self.totalSlots = secondTotal
            if self.totalSlots > 0:
                startCMD += ",\"slot\":[" + str(int(secondStart)) + "," + str(int(secondTotal)) + "]"

        if self.measurementInfo["cmdMeasure"] is not None:
            startCMD += ",\"measures\":\"" + self.measurementInfo["cmdMeasure"] + "\""

        if ts is not None:
            startCMD += ",\"time\":" + str(ts)
        
        if ntpConfidenceMs is not None:
            startCMD += ",\"ntpConf\":" + str(int(ntpConfidenceMs))


        startCMD += "}}"
        
        # print(startCMD)

        if self.verbose > 1: self.msPrint(startCMD)
        self.sendFunc(startCMD)
        # self._requestChunk(1024)
        # self.sendFunc(startCMD)
        # Why required?
        # if self.flowCtr:
        #     time.sleep(1.0)
    
    def _notInitedError(self):
        """Print a not inited error."""
        self.msPrint("\033[91mError: You must init the ms first. Call ms.init()\033[0m")

    def stop(self, force=False):
        """
        Stop measurement module.
        
        :param force:   If False, we send stop and wait for stop acknowledge and remaining data.
                        If True, we send stop but immidiately stop receiving data
        :type  force:   bool, default: False
        """
        if not self.inited: self._notInitedError

        # This is set by handlecommand function
        if self.verbose:
            self.msPrint("Stopping measurement system")
        self.sendFunc("{\"cmd\":\"stop\"}")


        # Stop auto detected, if "stop" send as line
        then = time.time()
        
        while not force and self.sampling:
            if self.sampling and time.time()-self.lastMessageReceived > max(5.0, 2*self.totalSlots):
                force = True
            if self._threadRunning:
                time.sleep(0.1)
            else:
                self._updateFunc()

        self._threadRunning = False
        if self._thread is not None:
            self._thread.join()
        if self._udpthread is not None:
            self._udpthread.join()
        if self.conType == "stream":
            self.socket.close()

        # If this has not already been set
        self.sampling = False
        if self.caStopTs is None:
            self.caStopTs = time.time()

        if self._serialProc is not None:
            self._serialProc.terminate()

        # Append last frame
        if self._currentFrame is not None and len(self._currentFrame) > 0:
            self.frames.append(self._currentFrame)

    def kill(self):
        """Kill all socket and serial connections"""
        if not self.inited: self._notInitedError

        killed = False
        if self.caStopTs is None: 
            self.caStopTs = time.time()
        self.sampling = False

        self._threadRunning = False
        if self._thread is not None:
            self._thread.join()
        if self._udpthread is not None:
            self._udpthread.join()

        if self.conType == "serial":
            if self._serialProc is not None:
                self._serialProc.terminate()
            if self._serialThread is not None:
                self._serialThread.terminate()
                killed = True
            self._serialThread = None
        elif self.conType == "udp":
            if self._udpsock is not None:
                self._udpsock.close()
                killed = True
            self._udpsock = None
        elif self.conType == "tcp":
            if self.socket is not None:
                self.socket.close()
                killed = True
            self.socket = None
        elif self.conType == "stream":
            if self.socket is not None:
                self.socket.close()
                killed = True
            self.socket = None
        
        if killed:
            self.onConnectionReset(reason="Killed")
        # Otherwise was killed before

    def _sendKeepAlive(self):
        """Send a keepalive Message"""
        self.sendFunc("!")

    def setClearToReceive(self, clear):
        """
        Enable or disable receiving with flowcontrol.
        
        :param clear:   True to receive, False to stop receiving
        :type  clear:   bool
        """
        self.sendFunc(str("{\"cmd\":\"cts\",\"value\":" + str(clear).lower() + "}"))

    def _requestSamples(self, samples):
        """
        Request for XXX number of samples.

        :param samples: Number of samples we want to receive. 
                        Number of bytes = clear*MEASUREMENT_BYTES
        :type  samples: int
        """
        self.sendFunc(str("{\"cmd\":\"reqSamples\",\"samples\":" + str(int(samples)) + "}"))

    def _handleData(self, data):
        """
        Handles a chunk of data. Data is converted to recarray with fieldnames.
        Samples and SamplesReceived is updated. Frames are appended.

        :param data: Data to be added to frames array.
        :type  data: bytes
        """
        # Wee need at least 2 floats of data
        if data is None or len(data) < self.MEASUREMENT_BYTES: return
        encodedData = np.frombuffer(data, dtype=np.float32)
        encodedData.reshape((len(self.MEASUREMENTS),-1))
        encodedData.dtype = [(m, np.float32) for m in self.MEASUREMENTS]

        if self._currentFrame is None:
            self._currentFrame = encodedData
        else:
            self._currentFrame = np.concatenate((self._currentFrame, encodedData), axis=0)
        while len(self._currentFrame) >= self.frameSize:
            # print(len(self._currentFrame))
            self.frames.append(self._currentFrame[:self.frameSize])
            self._currentFrame = self._currentFrame[self.frameSize:]
            self.framesFilled += 1
            # print(str(len(self._currentFrame)) + "\n")
        self.totalSamples += len(encodedData)
        self.samples += len(encodedData)

        #  Try to calculate the first timestamp
        if self.caStartTs is None:
            # Just a rough guess of what startTs really is
            self.caStartTs = time.time() - (self.totalSamples/self.samplingRate)
            if self.verbose: self.msPrint("Sampling started: " + str(time_format_ymdhms(self.caStartTs)))

        now = time.time()
        if now-self.lastMessageReceived > 1.0:
            if self.LIFENESS_SR:
                self.msPrint(str("@" + str(round(self.totalSamples/(now-self.caStartTs),2)) + "Hz"))
            self.lastMessageReceived = now
   
    # INIT METHODS depending on connection type _____

    def _initSerial(self):
        """
        Init the serial interface.
        
        :return:    True: Successfull inited or False: failed
        :rtype:     bool
        """
        if self.serial is None and self.portName is None: sys.exit("Either provide Serialport or portname")
        if self.serial is None:
            self.serial = serial.Serial()
            self.serial.port = self.portName
            self.serial.baudrate = self.baudrate
        else:
            self.portName = self.serial.port
            self.baudrate = self.serial.baudrate
        # TODO: Seems to be not correct and to test
        # Info lines are stored in this queue
        self._lineQueue = multiprocessing.Queue()
        # planar data frames (planar lists) are stored in this queue
        self._dataQueue = multiprocessing.Queue()
        self._samplesReceivedM = Value('i', 0)
        # We need a real new process (a real thread) to handle incoming serial data since serial rx buffer
        # only holds data for max 7ms at 8000Hz
        self._procSleepTime = 0.05
        # On mac systems the CP21XX kernel driver only holds 1020 bytes so increase polling rate
        if platform.system() == 'Darwin': self._procSleepTime = 0.001

        self.serial.timeout = 0.0002
        if not self.serial.isOpen() and self.serial.open() is False:
            self.msPrint("Error Serial Port " + str(self.serial.port) + " was not opened correctly")
            return False
        return True

    
    def _initSocket(self):
        """
        Init the socket interface, will exit on connection timeout

        :return:    True: Successfull inited or False: failed
        :rtype:     bool
        """
        if self.socket is not None:
            self.socket.setblocking(False)
            self.socket.settimeout(0.1)
            return True
        # Init TCP socket for communication
        self.socket = socket.socket()
        # try connect socket
        try:
            # socket timeout for connect
            self.socket.settimeout(3)
            self.socket.connect((self.ip, self.port))
        except socket.error as msg:
            self.msPrint("Connection failed. Error: %s" % str(msg))
            self.msPrint("Error Socket " + str(self.ip) + ":" + str(self.port) + " cannot be opened")
            return False
        # socket timeout for sending msges
        # What if this timeout is too small?
        # self.socket.settimeout(1)
        # self.socket.settimeout(0.1)
        # self.socket.setblocking(0)
        self.socket.setblocking(False)
        # self.socket.settimeout(None)
        self.socket.settimeout(0.1)
        # self.socket.settimeout(0.0001)
        return True

        
    def _initUDP(self):
        """
        Init the udp socket interface, will exit on connection timeout.

        :return:    True: Successfull inited or False: failed
        :rtype:     bool
        """
        self._udpPacketSize = 523
        # Init UDP socket for sending data
        self._udpsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Try setup udp server socket (this should not fail)
        try:
            self._udpsock.settimeout(3)
            self._udpsock.bind(("0.0.0.0", self.portUDP))
        except socket.error as msg:
            self.msPrint("Connection failed. Error: %s" % str(msg))
            self.msPrint("Error UDP Socket cannot be opened")
            return False
        # socket timeout for sending msges
        # self._udpsock.settimeout(0.00001)
        self._udpsock.settimeout(0.1)
        return True


    # SEND METHODS depending on connection type _____

    # def _sendSocketMsg2(self, msg):
    #     message = (msg + '\r\n').encode('utf-8')
    #     totalsent = 0
    #     while totalsent < len(message):
    #         sent = self.socket.send(message[totalsent:])
    #         if sent == 0:
    #             raise RuntimeError("socket connection broken")
    #         totalsent = totalsent + sent

    def _sendSocketMsg(self, msg):
        """
        Send a message to the socket device.

        The message must be a string. A newline is appended.

        :param msg: Message
        :type  msg: str
        """
        message = msg + '\r\n'
        try:
            self.socket.sendall(message.encode('utf-8'))
        except socket.error as e:
            self.msPrint("Error " + str(e) + " sending over socket: " + str(msg))
            self.onConnectionReset("SendingError")

    def _sendSerialMsg(self, msg):
        """
        Send a message to the serial device.

        The message must be a string. A newline is appended.

        :param msg: Message
        :type  msg: str
        """
        message = msg + '\r\n'
        try:
            self.serial.write(message.encode('utf-8'))
        except:
            self.msPrint("Error sending over serial")

    # General update method
    def _updateThread(self):
        """"Update in thread."""
        while self._threadRunning:
            if not self.inited: 
                time.sleep(0.1)
                continue
            self.update()
            # time.sleep(0.001)
            # time.sleep(0.001)
            # time.sleep(0.0001)
            # time.sleep(0.00001)

    # Update METHODS depending on connection type _____

    def _updateUDP(self):
        """Update the udp socket."""
        # if not self.sampling: return
        try:
            self._receiveUDP()
        # No data yet
        except socket.timeout:
            time.sleep(0.01)
        # Catch other errors
        except socket.error as err:
            # Remote peer disconnected
            self.msPrint(str(err))
            self.msPrint("Detected remote disconnect")
            # Do autoReconnect stuff
            self.sampling = False
            self.onConnectionReset(str(err))

    def _updateTCP(self):
        """Update the tcp socket."""
        try:
            self._receiveSocket()
        # No data yet
        except socket.timeout:
            pass
        # Catch other errors
        except socket.error as err:
            # Remote peer disconnected
            self.msPrint(str(err))
            self.msPrint("Detected remote disconnect")
            # Do autoReconnect stuff
            self.sampling = False
            self.onConnectionReset(str(err))

    def _updateStream(self):
        """Update the stream socket."""
        if not self.sampling: return
        try:
            self._receiveStream()
        # No data yet
        except socket.timeout:
            time.sleep(0.01)
        # Catch other errors
        except socket.error as err:
            # Remote peer disconnected
            self.msPrint(str(err))
            self.msPrint("Detected remote disconnect")
            # Do autoReconnect stuff
            self.sampling = False
            self.onConnectionReset(str(err))

    def _updateSerial(self):
        """Update the serialport."""
        # read line without blocking
        try:  
            line = self._serialQueueLine.get_nowait() # or q.get(timeout=.1)
        except Empty:
            pass
        else: 
            self._handleLine(line)
        try:  
            data = self._serialQueueData.get_nowait() # or q.get(timeout=.1)
        except Empty:
            pass
        else: 
            self.samplesReceived += 1;
            self._handleData(data)

    def update(self):
        """"
        Update function to handle incoming data.
        Use either updateInThread=True in module init or regularly (< 0.001s) call
        this update function. Of course the minimum time between updates depend on the samplingrate
        set. Make sure this is enough for your setting.
        """
        if not self.inited: return
        
        now = time.time()
        if self.sampling and now-self.lastMessageReceived > max(4.0, 2*self.totalSlots):
            if now-self._lastDeadConnectionCB > max(1.0, self.totalSlots):
                self._lastDeadConnectionCB = now
                if self.deadConnectionCB is not None: self.deadConnectionCB(self, self.lastMessageReceived)
            if now-self._lastDeadConnectionInfo > max(4.0, 2*self.totalSlots):
                self._lastDeadConnectionInfo = now
                if self.verbose: self.msPrint("\033[93mDead Connection?\033[0m")
                
        if self.flowCtr and self.sampling:            # time.sleep(1.0)
            # self._updateFunc()
            self._requestSamples(max(int(self.samplingRate/10), 1))
            # time.sleep(0.01)
            pass
        
        if self.KEEPALIVE_INTV != -1 and now-self._lastKeepAlive > self.KEEPALIVE_INTV:
            self._lastKeepAlive = now
            self._sendKeepAlive()
            

        self._updateFunc()
        
   
    
    def _receiveSerial(self):
        """"
        Update function to handle incoming socketdata.
        This is performed in a thread to make sure it is called frequently.
        """
        buff = b''
        while True:
            # Get the number of available bytes
            try:
                if self.serial.inWaiting() > 0 or len(buff) != 0:
                    # print(serialPort.inWaiting())
                    buff += self.serial.read(self.serial.inWaiting())
                    linePre = buff.find(self.LINE_PREFIX)
                    dataPre = buff.find(self.DATA_PREFIX)
                    NL = buff.find(self.SERIAL_LINE_SEP)
                    if linePre != -1 and dataPre == -1 or linePre != -1 and dataPre != -1 and linePre < dataPre:
                        if NL != -1:
                            line = buff[linePre:NL]
                            buff = buff[NL+2:]
                            self.lineQueue.put(line)
                        else:
                            self.msPrint("NL not found, should not happen")
                    elif dataPre != -1 and linePre == -1 or linePre != -1 and dataPre != -1 and dataPre < linePre:
                        buff = buff[dataPre:]
                        if len(buff) < 12: continue
                        lengthbuf = buff[5:7]
                        length, = struct.unpack('<H', lengthbuf)
                        if len(buff) < 11+length: continue
                        packet = buff[7:11]
                        packet, = struct.unpack('<I', packet)
                        if self._lastPacket+1 != packet:
                            self.msPrint("\033[93mMissed: " + str(packet-(self._lastPacket+1)) + " packets, fill with 0s.." + "\033[0m")
                            if packet-(self._lastPacket+1) < 1000:
                                self.dataQueue.put(b"\0" * int((packet-(self._lastPacket+1))*length))
                        if len(buff) < 11+length: continue
                        self._lastPacket = packet
                        data = buff[11:11+length]
                        self.samplesReceivedM.value += int(length/self.SINGLE_BYTES)
                        self.dataQueue.put(data)
                        buff = buff[11+length:]
                else:
                    time.sleep(self.procSleepTime)
                # self.msPrint(numbytes)
            except serial.SerialException as e:
                pass
                #There is no new data from serial port
            except TypeError as e:
                #Disconnect of USB->UART occured
                self.msPrint("Critical Serial error")
                self.errorDetected = True
                return
            except OSError as e:
                #Disconnect of USB->UART occured
                self.msPrint("OS error")
                self.errorDetected = True
                return
        time.sleep(0.001)

    def _receiveUDP(self):
        """Receive chunk of UDP data"""
        # NOTE: Never change the UDP size on the client, otherwise, this will not work
        # restLen = 523-len(self.udpBuff)
        newData, address = self._udpsock.recvfrom(self._udpPacketSize)
        # self.udpBuff += new
        # if len(self.udpBuff) != 523: return
        # newData = self.udpBuff
        # self.udpBuff = b""
        # newData, address = self._udpsock.recvfrom(523)
        if newData is None or len(newData) < 5: return
        prefix = newData[0:len(self.DATA_PREFIX)]
        if prefix.find(b'Data:') == 0:
            newData = newData[len(self.DATA_PREFIX):]
            length, = struct.unpack('<H', newData[0:2])
            newData = newData[2:]
            packet, = struct.unpack('<I', newData[0:4])
            data = newData[4:]
            if len(data) != length:
                self.msPrint("\033[93mCorrupt UDP packet! " + str(length) + " vs: " + str(len(data)) + "\033[0m")
                return
            if packet < self._lastPacket+1:
                self.msPrint("\033[93mReceived old packet, discarding\033[0m")
                return
            if self._lastPacket+1 != packet:
                self.msPrint("\033[93mMissed: " + str(packet-(self._lastPacket+1)) + " packets, fill with 0s..\033[0m")
                self._handleData(b"\0" * int((packet-(self._lastPacket+1))*length))
            self._lastPacket = packet
            self._handleData(data)
            self.samplesReceived += int(length/self.MEASUREMENT_BYTES)
        else:
            self.msPrint("\033[93mStrange UDP: " + str(newData) + "\033[0m")
        # CPU intensive task
        time.sleep(0.001)

    def _receiveStream(self):
        """Receive stream data"""
        try:
            data = self._recvSocket(1024)
            self._handleData(data)
            # No way to get track of lost packets
            self.samplesReceived = self.totalSamples
        # No data yet
        except socket.timeout:
            # self.msPrint("timeout: ")
            time.sleep(0.01)
        # Catch other errors
        except socket.error as err:
            # Remote peer disconnected
            self.msPrint(str(err))
            self.msPrint("Detected remote disconnect")
            # Do autoReconnect stuff
            self.sampling = False
            self.onConnectionReset(str(err))

    # TCP Socket connection using state machine and stuff
    length = 0
    packet = 0
    prefix = None
    buffer = b''
    recBuffer = b''

    def _processPrefix(self):
        """Process the prefix send over tcp. Either \"Info:\" or \"Data:\""""
        # Look if long enough to contain prefix
        if len(self.recBuffer) >= min(len(self.LINE_PREFIX), len(self.DATA_PREFIX)):
            indexL = self.recBuffer.find(self.LINE_PREFIX)
            indexD = self.recBuffer.find(self.DATA_PREFIX)
            # If we find both in the data, remove the one which is behind the other
            if indexL != -1 and indexD != -1:
                if indexL > indexD: indexL = -1
                else: indexD = -1
            # If we found a line prefix
            if indexL != -1:
                self.recBuffer = self.recBuffer[indexL+len(self.LINE_PREFIX):]
                self.prefix = self.LINE_PREFIX
            # If we found a data prefix
            elif indexD != -1:
                self.recBuffer = self.recBuffer[indexD+len(self.DATA_PREFIX):]
                self.prefix = self.DATA_PREFIX

    def _processLine(self):
        """Process a line from the device. it is terminated with \"\n\r\"."""
        # try to find newline character
        indexNL = self.recBuffer.find(b'\n')
        if indexNL >= 0:
            # Extract line
            line = self.recBuffer[:indexNL]
            self._handleLine(line)
            # Reset Buffer and prefix
            self.recBuffer = self.recBuffer[indexNL+1:]
            self.prefix = None

    def _processData(self):
        """Process a data chunk from the device. Length of chunk is part of msg."""
        # Min header size is 6
        minSize = 6
        if len(self.recBuffer) >= minSize:
            lengthAndPacketBuf = self.recBuffer[:6]
            length, = struct.unpack('<H', lengthAndPacketBuf[:2])
            packet, = struct.unpack('<I', lengthAndPacketBuf[2:])
            # Length of data should be header + length in header
            if len(self.recBuffer) >= minSize+length:
                # Check if packet has been lost
                # This is an old packet
                if packet < self._lastPacket+1:
                    self.msPrint("\033[93mReceived old packet, discarding\033[0m")
                # This is a packet from the future, we nee to fill with 0s
                elif self._lastPacket+1 != packet:
                    self.msPrint("\033[93mMissed: " + str(packet-(self._lastPacket+1)) + " packets, fill with 0s.." + "\033[0m")
                    self._handleData(b"\0" * int((packet-(self._lastPacket+1))*length))
                # This is regular packet + future packet \TODO: test
                if packet >= self._lastPacket+1:
                    self._lastPacket = packet
                    # Extract data and handle data
                    data = self.recBuffer[minSize:minSize+length]
                    self._handleData(data)
                    self.samplesReceived += int(length/self.MEASUREMENT_BYTES)
                # Reset Buffer and prefix
                self.recBuffer = self.recBuffer[minSize+length:]
                self.prefix = None

    def _readSocket(self):
        """Read chunk of data."""
        try:
            # Should be ready to read
            data = self.socket.recv(2048)
        except BlockingIOError:
            # Resource temporarily unavailable (errno EWOULDBLOCK)
            pass
        # ECONRESET?
        except ConnectionResetError:
            self.onConnectionReset("ConnectionResetError while reading")
        else:
            if data: 
                self.recBuffer += data
            # ECONRESET?
            else:
                self.onConnectionReset("_readSocket DataNone")

    def _receiveSocket(self):
        """
        Handle TCP receiving of data.
        TODO: Test timeout with different type of sampling settings and
        ``updateInThread`` vs ``device.update()``.
        """
        # Drastically reduces CPU intensity when timeout is used here
        ready_to_read, ready_to_write, in_error = select.select([self.socket], [], [self.socket], 0.1)
        if len(ready_to_read) > 0:
            self._readSocket()
            # Handle all incoming data
            while len(self.recBuffer) > 0:
                # If prefis has not been found
                if self.prefix is None:
                    self._processPrefix()
                    if self.prefix is None: break # we need data

                # If line prefix has been found, we need to find \n
                if self.prefix is self.LINE_PREFIX:
                    self._processLine()
                # If data prefix has been found, length is in header
                elif self.prefix is self.DATA_PREFIX:
                    self._processData()

                if self.prefix is not None: break # we need data
            

    # def _receiveSocket2(self):
    #     #  Only continue if sth ready to read is there
    #     ready_to_read, ready_to_write, in_error = select.select([self.socket], [], [self.socket], 0)
    #     if len(ready_to_read) > 0:
    #         #  Try to read data
    #         self._readSocket()
    #     if len(self.recBuffer) > 0:
    #         # If prefis has not been found
    #         if self.prefix is None:
    #             self._processPrefix()
    #         # If line prefix has been found, we need to find \n
    #         if self.prefix is self.LINE_PREFIX:
    #             self._processLine()
    #         # If data prefix has been found, length is in header
    #         elif self.prefix is self.DATA_PREFIX:
    #             self._processData()

    # state = 0
    # STATE_IDLE = 0
    # STATE_LINE = 1
    # STATE_DATA = 2
    # STATE_DATA_D = 3
    # def _receiveSocket3(self):
    #     chunk = self._recvSocket(5)
    #     if chunk is None: return
    #     self.recBuffer += chunk
    #     # Look for line prefix
    #     index = self.recBuffer.find(self.LINE_PREFIX)
    #     while index >= 0:
    #         self.recBuffer = self.recBuffer[index+len(self.LINE_PREFIX):]
    #         indexNL = self.recBuffer.find(b'\x0a')
    #         if indexNL < 0:
    #             self.recBuffer += self._recvSocketUntil(b'\x0a')
    #             indexNL = self.recBuffer.find(b'\x0a')
    #         line = self.recBuffer[:indexNL]
    #         self.recBuffer = self.recBuffer[indexNL+1:]
    #         self._handleLine(line)
    #         index = self.recBuffer.find(self.LINE_PREFIX)
        
    #     # Look for data prefix
    #     index = self.recBuffer.find(self.DATA_PREFIX)
    #     while index >= 0:
    #         self.recBuffer = self.recBuffer[index+len(self.DATA_PREFIX):]
    #         bufLen = len(self.recBuffer)
    #         if bufLen < 6:
    #             self.recBuffer += self._recvSocket(6)
    #         lengthAndPacketBuf = self.recBuffer[:6]
    #         self.recBuffer = self.recBuffer[6:]
    #         if lengthAndPacketBuf is None: return
    #         length, = struct.unpack('<H', lengthAndPacketBuf[:2])
    #         packet, = struct.unpack('<I', lengthAndPacketBuf[2:])
    #         if self._lastPacket+1 != packet:
    #             self.msPrint("\033[93mMissed: " + str(packet-(self._lastPacket+1)) + " packets, fill with 0s.." + "\033[0m")
    #             self._handleData(b"\0" * int((packet-(self._lastPacket+1))*length))
    #         self._lastPacket = packet
    #         bufLen = len(self.recBuffer)
    #         if bufLen < length:
    #             chunk = None
    #             while chunk is None:
    #                 chunk = self._recvSocket(length-bufLen)
    #             self.recBuffer += chunk
    #         data = self.recBuffer[:length]
    #         self.recBuffer = self.recBuffer[length:]
    #         self._handleData(data)
    #         self.samplesReceived += int(length/self.MEASUREMENT_BYTES)

    #         index = self.recBuffer.find(self.DATA_PREFIX)



    # def _receiveSocket2(self):
    #     """This is a nonblicking receive approach."""
    #     if self.state == self.STATE_IDLE:
    #         # prefix = self._recvSocket(5)
    #         prefix = self._recvSocketUntil(b':')
    #         if prefix is None: return
    #         if self.LINE_PREFIX in prefix:
    #             self.state = self.STATE_LINE
    #         elif self.DATA_PREFIX in prefix:
    #             self.state = self.STATE_DATA
    #         else:
    #             self.msPrint("\033[93mstrange TCP: " + str(prefix) + "\033[0m")
    #     if self.state == self.STATE_LINE:
    #         line = self._recvSocketUntil(b'\x0a')
    #         if line is None: return
    #         self._handleLine(line)
    #         self.state = self.STATE_IDLE
    #     elif self.state == self.STATE_DATA:
    #         lengthAndPacketBuf = self._recvSocket(6)
    #         if lengthAndPacketBuf is None: return
    #         self.length, = struct.unpack('<H', lengthAndPacketBuf[:2])
    #         self.packet, = struct.unpack('<I', lengthAndPacketBuf[2:])
    #         if self._lastPacket+1 != self.packet:
    #             self.msPrint("\033[93mMissed: " + str(self.packet-(self._lastPacket+1)) + " packets, fill with 0s.." + "\033[0m")
    #             self._handleData(b"\0" * int((self.packet-(self._lastPacket+1))*self.length))
    #         self._lastPacket = self.packet
    #         self.state = self.STATE_DATA_D
    #     if self.state == self.STATE_DATA_D:
    #         try:
    #             data = self._recvSocket(self.length)
    #         except:
    #             self.msPrint("\033[93mMissed: something\033[0m")
    #             self.state = self.STATE_IDLE
    #             return

    #         if data is None: return
    #         self._handleData(data)
    #         self.samplesReceived += int(self.length/self.MEASUREMENT_BYTES)
    #         self.state = self.STATE_IDLE


    # def _recvSocket2(self, count):
    #     chunks = []
    #     bytes_recd = 0
    #     while bytes_recd < count:
    #         chunk = self.socket.recv(min(count - bytes_recd, 2048))
    #         if chunk == b'':
    #             raise RuntimeError("socket connection broken")
    #             self.onConnectionReset()
    #             return None
    #         chunks.append(chunk)
    #         bytes_recd = bytes_recd + len(chunk)
    #     return b''.join(chunks)

    # def _recvSocket(self, count):
    #     ready_to_read, ready_to_write, in_error = select.select([self.socket], [], [self.socket], 0)
    #     if len(ready_to_read) > 0:
    #         try:
    #             self.buffer += self.socket.recv(count-len(self.buffer))
    #         except ConnectionResetError:
    #             self.onConnectionReset()
    #             return None
    #         if len(self.buffer) == count:
    #             buf = self.buffer
    #             self.buffer = b''
    #             return buf
    #     return None

    # def _recvSocketUntilNewLine2(self):
    #     while True:
    #         try:
    #             c = self.socket.recv(1)
    #         except ConnectionResetError:
    #             self.onConnectionReset()
    #             return None
    #         self.buffer += c
    #         if c == b'\x0a':
    #             buf = self.buffer
    #             self.buffer = b''
    #             return buf
        
    # def _recvSocketUntil(self, char):
    #     ready_to_read, ready_to_write, in_error = select.select([self.socket], [], [self.socket], 0)
    #     while len(ready_to_read) > 0:
    #         try:
    #             c = self.socket.recv(1)
    #         except ConnectionResetError:
    #             self.onConnectionReset()
    #             return None
    #         self.buffer += c
    #         if c == char:
    #             buf = self.buffer
    #             self.buffer = b''
    #             return buf
    #         ready_to_read, ready_to_write, in_error = select.select([self.socket], [], [self.socket], 0)
    #     return None

    def onConnectionReset(self, reason="unknown"):
        """
        Is called on a connection reset. 

        :param  reason: Reason for the reset. Might also be intentional by calling kill.
        :type   reason: str, default: unknown
        """
        if self.brokenConnection: return
        self.inited = False
        self.brokenConnection = True
        if self.connectionResetCB is not None:
            self.connectionResetCB(self, str(reason))
        else:
            self.msPrint("[E]Connection broken...Reason: " + str(reason)) 

    LOGG_STUFF = [{"s":"[W]", "c":"\033[93m"},
                  {"s":"[E]", "c":"\033[91m"},
                  {"s":"[D]", "c":"\033[94m"},
                  {"s":"[I]", "c":""}]
    def msPrint(self, msg):
        """
        Print a message. 

        :param  msg:    Message to print. The msg will be printed in color if it is a
                        DEBUG,WARNING or ERROR message. See :class:`LogLevel<powermeter.smartDevice.LogLevel>` enum.
        :type   msg:    str, default: unknown
        """
        prefix = ""
        endFix = ""
        try:
            i = next(i for i, s in enumerate(self.LOGG_STUFF) if s["s"] in msg)
        except StopIteration:
            i = -1
        if i != -1:
            prefix = self.LOGG_STUFF[i]["c"]
            msg = msg.replace(self.LOGG_STUFF[i]["s"], "").lstrip(" ")
        if prefix != "": endFix = "\033[0m"
        if self.logger: self.logger(prefix + "{0:<15}".format(str("$" + self.name + ": ")[:15]) + str(msg) + endFix)

    def _samplingStarted(self):
        """Called if sampling is started. Will call callbacks."""
        self.sampling = True
        startTime = time.time()
        if self.startTs is not None:
            startTime = self.startTs
        if self.samplingStartedCB is not None:
            self.samplingStartedCB(startTime)

    def _handleCommand(self, cmd, di):
        """
        Called if sampling is started. Will call callbacks.
        
        :param cmd: The command to handle
        :type  cmd: str
        :param di:  The dictionary with further command info. 
        :type  di:  dict
        """
        # No matter what, set name and type if not set yet
        if "type" in di:
            self.TYPE = di["type"]
        if "name" in di:
            self.deviceName = di["name"]
            # If no name set, set it to standard name
            if self.name == self.STANDARD_NAME:
                self.name = self.deviceName
    
        # Result of sample cmd
        if cmd == "sample":
            self.deviceSamplingInfo = di
            #  Set start ts if in dictionary
            if "start_ts" in di:
                self.startTs = float(di["start_ts"])
            if "samplingrate" in di:
                self.samplingRate = int(di["samplingrate"])
            self._samplingStarted()
        # Result of stop cmd
        elif cmd == "stop":
            self.sampling = False
            self.caStopTs = time.time()
            #  Set stop ts if in dictionary
            if "stop_ts" in di:
                self.stopTs = float(di["stop_ts"])
            self.deviceSamplingInfo = di
        #  Result of info cmd
        elif cmd == "info":
            di.pop("cmd", None)
            self.deviceInfo = di
            if self._sysInfoCB is not None: self._sysInfoCB(self, di)
        # Result of log cmd
        elif cmd == "log":
            if self._logInfoCB is not None: self._logInfoCB(self, di["msg"].replace("//n", "\n").split("\n"))

    def _handleJson(self, di):
        """
        Handles a JSON msg from the device.
        
        :param di:  The decoded JSON dictionary. 
        :type  di:  dict
        """
        if self.verbose: self.msPrint(json.dumps(di, indent=4, sort_keys=True))
        # Look if previous cmd failed or succeeded
        error = False;
        if "error" in list(di.keys()) and di["error"] == True: error = True

        if "cmd" in list(di.keys()):
            self._handleCommand(di["cmd"], di)
        for key in di:
            # Prevent log message from beeing printed out if we have a callback
            if self.logLevelDevice.value > LogLevel.INFO.value and not self.verbose and self._sysInfoCB is not None: return
            # Print message, red on error
            if key == "msg":
                pre = ""
                if error: pre += '\033[91m'+" Error: " + '\033[0m'
                lines = di[key].split("//n")
                for line in lines: 
                    self.msPrint(str(pre + line))
            else:
                if self.verbose:
                    # Simply print key value pair
                    self.msPrint("\t" + str(key) + ": " + str(di[key]))

    def _handleLine(self, line):
        """
        Handles a info line sent from the device.
        This might be a serialized JSON dictionary or just basic information
        encoded in utf-8.
        
        :param line:  The line still iun bytes representation. 
        :type  line:  bytes
        """
        self.lastMessageReceived = time.time()   
        try: 
            string = line.decode("utf-8").rstrip("\n").rstrip("\r").lstrip(self.LINE_PREFIX.decode("utf-8"))
        except UnicodeDecodeError:
            string = str(line)
        try:
            encodeJson = json.loads(string)
            self._handleJson(encodeJson)
            return
        except json.decoder.JSONDecodeError:
            pass
        if "KeepAlive" in string: return
        # if self.verbose:
        self.msPrint(string)
