# Command Line Interface for PowerMeters and SmartMeters

SmartMeters and PowerMeters are custom IP based energy monitoring solutions.
See the following repos:
* [PowerMeter](https://github.com/voelkerb/powermeter)
* [SmartMeter](https://github.com/voelkerb/smartmeter)

Use the files in this repo to 
* distribute firmware updates in a fast and easy way.
* interface with the systems (e.g. plot their data).
* change their settings.

# How to use?

Each script in the top level folder (```getInfo```, ```getLog```, ```interface``` and ```update.py```) give the possibility to select one or multiple power- or smartmeter devices in your network.

E.g. if you call: 
```bash
python3 getInfo.py
```
You will see a list of all devices found in the network, which broadcast the service ```_elec``` over mDNS.
```bash
Available Devices:
#  smartmeter:              Device:                  IP:                      
0  smartmeter001            smartmeter001            192.168.0.36             
#  powermeter:              Device:                  IP:                      
1  powermeter15             powermeter15             192.168.0.113            
2  powermeter20             powermeter20             192.168.0.111            
3  powermeter21             powermeter21             192.168.0.118            
4  powermeter24             powermeter24             192.168.0.115            
5  powermeter27             powermeter27             192.168.0.138            
6  powermeter28             powermeter28             192.168.0.114                  
Press ENTER to continue with all devices.
Deselect specific devices e.g.: -2,-4,-7
Select specific devices e.g.: 1,3,5,6
Search again a/A
Press r/R to reset
e/E to cancel Or press CTR-C to exit program
```

You can now select one or multiple of these devices to include them for the rest of the script. 

## Getting info

Call: 
```bash
python3 getInfo.py
```
Select one or multiple powermeter and confirm.
You will receive general info of all selected devices, e.g.:
```bash
powermeter15:
 type:         powermeter
 version:      2.1
 compiled:     Feb 25 2021 09:47:40
 sys_time:     03/08/2021 15:16:51.149
 name:         powermeter15
 ip:           192.168.0.113
 mqtt_server:  192.168.0.13
 stream_server:192.168.1.101
 time_server:  time.google.com
 sampling_rate:100
 buffer_size:  3670016
 psram:        True
 rtc:          True
 calV:         1
 calI:         1
 state:        idle
 relay:        1
 calibration:  [1, 1]
 ssids:        [esewifi, energywifi, FallObst]
 ssid:         FallObst
 rssi:         -78
 bssid:        <XXXXXXXXX>
```

## Plotting data
Plotting live high frequency data from a PowerMeter: 
```bash
python3 powermeter/powermeter.py --host <powermeter_mDNS_name or IP> --samplingrate <sr> --plot --measures <measures>
```
* ```<powermeter_mDNS_name or IP>```: is either the powermeters IP address or the mDNS name. You can get the name using ```python3 getInfo``` 
* ```<sr>```: Samplingrate as interger. Value between 1 and 8000, default: 4000
* ```measures>```: either "v,i", "p,q", "v,i,p,q" or "v_RMS,i_RMS", default: "v,i"
Example output for: 
```bash
python3 powermeter/powermeter.py --host powermeter20.local --samplingrate 2000 --plot --measures "v,i,p,q"
```
  <img src="/docu/figures/powerMeterPlot2.jpg">

Plotting live high frequency data from a SmartMeter: 
```bash
python3 powermeter/smartmeter.py --host <smartmeter_mDNS_name or IP> --samplingrate <sr> --plot --measures <measures> --phase <phase>
```
* ```<powermeter_mDNS_name or IP>```: is either the powermeters IP address or the mDNS name. You can get the name using ```python3 getInfo```. Instead, you can also connect it over USB. See ```python3 powermeter/smartmeter.py -h``` for more info. 
* ```<sr>```: Samplingrate as interger. Value between 1 and 32000, default: 8000
* ```measures>```: either "v_l1,i_l1,v_l2,i_l2,v_l3,i_l3", "v,i", "v_rms_l1,v_rms_l2,v_rms_l3,i_rms_l1,i_rms_l2,i_rms_l3" or "p_l1,p_l2,p_l3,q_l1,q_l2,q_l3", default: "v_l1,i_l1,v_l2,i_l2,v_l3,i_l3"
* ```phase```: either 1,2,3 or None, default: None (all phases)
Example output for: 
```bash
python3 powermeter/smartmeter.py --host smartmeter001.local --samplingrate 2000 --plot
```
  <img src="/docu/figures/smartMeterPlot.jpg">


## Storing data
You want to store the data as ```wavPack``` encoded audio data inside a ```mkv``` container?
Use the same commands as for plotting and append the options ```--ffmpeg``` and ```--filename <FILENAME>```. E.g.:
```bash
python3 powermeter/smartmeter.py --host smartmeter001.local --samplingrate 2000 --ffmpeg --filename test.mkv
```
Stop the recording using ```ctr-c```.

```
$:powermeterInterface voelkerb$ ffprobe test.mkv 
Input #0, matroska,webm, from 'test.mkv':
  Metadata:
    ENCODER         : Lavf58.45.100
  Duration: 00:00:06.68, start: 0.000000, bitrate: 292 kb/s
    Stream #0:0: Audio: wavpack, 2000 Hz, 6 channels, fltp (default)
    Metadata:
      title           : SmartDevice
      CHANNELS        : 6
      CHANNEL_TAGS    : v_l1,i_l1,v_l2,i_l2,v_l3,i_l3
      ENCODER         : Lavc58.91.100 wavpack
      DURATION        : 00:00:06.683000000
```


## Change settings
```bash
python3 interface.py
```
Select one or multiple powermeter and confirm. You will se something like:
```bash
Type 'help' to get list of available commands
$powermeter15: Log Level set to: warning
$powermeter21: Log Level set to: warning
$powermeter20: Log Level set to: warning
```
enter ```help``` which shows you all available commands.
```bash
help
        - help, h         : Display this help message.
        - c               : Abort, stop this program.
        - sample          : Start sampling at a given samplingrate.
        - stop, s         : Stop sampling.
        - switch          : Switch relay on or off.
        - restart, r      : Restart the device(s).
        - factoryReset, r!: Reset the device to factory settings.
        - basicReset, b!  : Reset settings og device, not the name.
        - info, ?, i      : Give info about device(s).
        - mdns, m         : Set new MDNS name.
        - ntp, n          : Start NTP synchronisation.
        - streamServer, ss: Add a stream server.
        - mqttServer, mqtt: Add a mqtt server address.
        - timeServer, t   : Add a time server.
        - addWifi, aw     : Add a wifi AP.
        - delWifi, dw     : Delete a wifi AP.
        - log, l          : Display and delete Log messages.
        - logLevel, ll    : Change device log level.
        - calibrate, cal  : Set calibration coefficients.
        - verbose, v      : Change verbose output.
```
Lets now e.g. set a new NTP time server for all powermeters.
Therefor, type in ```timeServer``` or simply ```t```.
After typing in the new time server and confirming with enter, you will get the response from all powermeters.
```bash
timeServer
Type in time server ip or dns + enter
time.google.com
TimeServer: time.google.com
$powermeter21: 03/08 15:22:11: RTC updated old: 03/08/2021 15:22:11.000
$powermeter15: 03/08 15:22:11: RTC updated old: 03/08/2021 15:22:11.000
$powermeter20: 03/08 15:22:11: RTC updated old: 03/08/2021 15:22:11.000
$powermeter21: Set TimeServer address to: time.google.com
$powermeter15: Set TimeServer address to: time.google.com
$powermeter20: Set TimeServer address to: time.google.com
```

## Getting logs
```bash
python3 getLog.py
```
Select one or multiple powermeter and confirm. 
```bash
powermeter15:
*** LOGFile *** 
03/08 15:33:18: client 192.168.0.17 disconnected
03/08 16:33:35: Reboot - inited SPIFFS Logger
03/08 16:33:35: Resetted, reasons CPU 0: SW_CPU_RESET, 1: SW_CPU_RESET
*** LOGFile ***

powermeter20:
*** LOGFile *** 
*** LOGFile *** 
```

## Updating the firmware

  <img src="https://github.com/voelkerb/powermeter/blob/master/docu/figures/upload.gif">

* Compile the firmware see [instructions](https://github.com/voelkerb/powermeter/blob/master/docu/README_Firmware_2_compile.md) for [powermeter](https://github.com/voelkerb/powermeter) or [instructions](https://github.com/voelkerb/smartmeter/blob/master/docu/README_Firmware_2_compile.md) for [smartmeter](https://github.com/voelkerb/smartmeter) or 
* Copy the path of the compiled binary _elf_ or _bin_ (the _bin_ will be used either way)
* Use the upload script
  ```bash
  python3 upload.py <smartmeter/powermeter> <pathToElfOrBin> 
  ```
    The script will only show you devices which are compatible with your firmware file.
    Select one or multiple powermeter or smartmeter from the list and confirm twice. 
    Press enter and watch the magic


# Reference

Please cite our publications if you compare to or use this system:
* Benjamin Völker, Philipp M. Scholl, and Bernd Becker. 2019. Semi-Automatic Generation and Labeling of Training Data for Non-intrusive Load Monitoring. In Proceedings of the Tenth ACM International Conference on Future Energy Systems (e-Energy '19). Association for Computing Machinery, New York, NY, USA, 17–23. DOI:https://doi.org/10.1145/3307772.3328295
 
* Benjamin Völker, Marc Pfeifer, Philipp M. Scholl, and Bernd Becker. 2020. FIRED: A Fully-labeled hIgh-fRequency Electricity Disaggregation Dataset. In Proceedings of the 7th ACM International Conference on Systems for Energy-Efficient Buildings, Cities, and Transportation (BuildSys '20). Association for Computing Machinery, New York, NY, USA, 294–297. DOI:https://doi.org/10.1145/3408308.3427623

* Völker, B.; Pfeifer, M.; Scholl, P.M.; Becker, B. A Framework to Generate and Label Datasets for Non-Intrusive Load Monitoring. Energies 2021, 14, 75. https://doi.org/10.3390/en14010075
