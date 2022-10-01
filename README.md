# SDM630-MCT-Modbus v2
Test update'u
SDM630-MCT-Modbus v2 is a 3-phase power meter with RS485 Port modbus RTU.
This is a plugin for domoticz to get all the data from the meter directly into Domoticz. 
To make it work usually you need a USB-Modbus dongle plugged into raspberry and cables connected to A and B in the meter. 
This is a Modbus RTU communication, does not work with TCP-IP Modbus.

Original code by MFxMF for the SDM630-M power meter https://github.com/MFxMF/SDM630-Modbus.
Further edited by bbossink to work with SDM72D-M v1: https://github.com/bbossink/SDM72D-Modbus-Domoticz-plugin,
and by philips to work with SDM72D-M v2 and for SDM630-MCT-Modbus v2

More info can be found in the modbus manual of this energy meter: https://stromzähler.eu/media/pdf/cf/8f/92/SDM630-Modbus-V2-manual-incl-protocoll.pdf

## Prerequisites
You need a working Domoticz instance with working python plugin service (see logs in domoticz)<br>
This plugin requires python modules: <br>
- pyserial -> https://pythonhosted.org/pyserial/ <br>
- minimalmodbus -> http://minimalmodbus.readthedocs.io<br>
To install those above :
```
sudo apt-get update
sudo apt-get install python3.7 libpython3.7 python3.7-dev -y
sudo apt-get install python-pip python3-pip -y
pip install pyserial
pip install minimalmodbus
sudo pip3 install -U pymodbus
sudo reboot
```
## Installation of the plugin
1. Clone repository into your domoticz plugins folder
```
cd ~/domoticz/plugins
git clone https://github.com/philip-s/SDM630-MCT-Modbus-v2-Domoticz-plugin.git
```
2. Restart domoticz:
```
sudo systemctl restart domoticz.service 
```
## Configuration
3. Refresh Domoticz website (F5).<br>
4. Select "Eastron SDM630-MCT-Modbus v2" in Hardware configuration screen.<br>
If needed modify some parameters (defaults will do) and click add.<br>
Hint: Set reading interval to 0 if you want updates per "heartbeat" of the system (aprox 10s in my case).<br>
Hint: The default interval of one minute is usually enough precise for most of the cases.<br>
5. Go to devices tab, there you will find all of grid parameters as devices. Add do domoticz the one you need using red arrow (usually not all of them are necessary). By default the main ones are already set to be visible.
## Updating
```
cd ~/domoticz/plugins/SDM630-MCT-Modbus-v2-Domoticz-plugin/
git pull
```
Tested on domoticz 2021.1 (should work on also on 2020.2)
<br><br><br>


