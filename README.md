# SDM630-MCT-Modbus v2
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

NOTE: The above modules stopped working with Python 3.9 in Raspberry Rasbian bullseye (november 2022). Keep Python 3.7. !

For Bullseye and newer raspbians: start from installing python 3.7 instead of 3.9: https://www.linuxcapable.com/how-to-install-python-3-7-on-debian-11-bullseye/ <br>
Please install both python 3.7 and python PIP 3.7 from the link above. <br>
Next, do this:
```
sudo apt-get update                           # Update repository
sudo apt-get upgrade                          # Update your system
sudo apt-get install python3.7 libpython3.7 python3.7-dev -y           # Archive, works only with old Pythons
sudo apt-get install python-pip -y            # Archive, works only with old Pythons
sudo apt-get install python3-pip -y           # Just in case
pip install pyserial
pip install minimalmodbus
sudo pip3 install -U pymodbus

## Just in case you didn't install pip from the link above with python 3.7 installation:
wget https://bootstrap.pypa.io/get-pip.py     
python3.7 get-pip.py                          

## Using python 3.7 install modules responsible for MODBUS functionality:
pip3.7 install pyserial
pip3.7 install minimalmodbus
sudo pip3.7 install -U pymodbus
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
5. Select your modbus USB dongle from list, should be something like /dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0<br>
6. Set baudrate to 4800 and Minutes between update interval to 2 (might not work if set to 1 minute, propadbly beacuse there is alot of data to acuire) <br>
7. Go to devices tab, there you will find all of grid parameters as devices. Add do domoticz the one you need using red arrow (usually not all of them are necessary). By default the main ones are already set to be visible.
## Updating
```
cd ~/domoticz/plugins/SDM630-MCT-Modbus-v2-Domoticz-plugin/
git pull
sudo systemctl restart domoticz.service
```
Tested on domoticz 2021.1 (should work on also on 2020.2)
<br><br><br>


