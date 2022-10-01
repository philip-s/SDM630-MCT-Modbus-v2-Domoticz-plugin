#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Eastron SDM630 MCT Modbus v2 is 3-phase Energy Meter Python Plugin for Domoticz
#
# Author: Filip Sobstel adapted original Filip Demaertelaere plugin for SDM120 to Sinotimer Meter
#
# The most common devices are created as "used"; the others are also created, but as "unused".
# To use the other devices, go to Setup-Devices to activate them.
#
# The standard communication protocol of the SDM630 MCT Modbus v2 is 9600 baud, 1 stopbit, 8 bit and no parity.
# To change the protocol, set the SDM630 in setup-mode (hold the button 3 second) and send the required
# commands to change (not implemented in this plugin).
#
"""
<plugin key="SDM630-MCT_M_V2" name="Eastron SDM630 MCT Modbus V2 Energy Meter" author="Filip Demaertelaere and adapted by Filip Sobstel" version="1.0.0">
    <params>
        <param field="SerialPort" label="Serial Port" width="120px" required="true"/>
        <param field="Mode1" label="Slave Unit ID" width="120px" required="true" default="1"/>
        <param field="Mode2" label="Baudrate" width="120px" required="true">
            <options>
                <option label="1200" value="1200"/>
                <option label="2400" value="2400"/>
                <option label="4800" value="4800"/>
                <option label="9600" value="9600" default="true"/>
                <option label="19200" value="19200"/>
            </options>
        </param>
        <param field="Mode3" label="Port settings" width="260px" required="true">
            <options>
                <option label="StopBits 1 / ByteSize 8 / Parity: None" value="S1B8PN" default="true"/>
                <option label="StopBits 1 / ByteSize 8 / Parity: Even" value="S1B8PE"/>
                <option label="StopBits 1 / ByteSize 8 / Parity: Odd" value="S1B8PO"/>
                <option label="StopBits 2 / ByteSize 8 / Parity: None" value="S2B8PN"/>
                <option label="StopBits 2 / ByteSize 8 / Parity: Even" value="S2B8PE"/>
                <option label="StopBits 2 / ByteSize 8 / Parity: Odd" value="S2B8PO"/>
            </options>
        </param>
        <param field="Mode4" label="Offset Total Active Energy (kWh)" width="120px" required="true" default="0"/>
        <param field="Mode5" label="Minutes between update" width="120px" required="true" default="1"/>
        <param field="Mode6" label="Debug" width="120px">
            <options>
                <option label="True" value="Debug"/>
                <option label="False" value="Normal" default="true"/>
            </options>
        </param>
    </params>
</plugin>
"""

#IMPORTS
import Domoticz
import subprocess
import sys
import pymodbus
sys.path.append('/usr/local/lib/python3.7/dist-packages')
from pymodbus.client.sync import ModbusSerialClient 	# RTU
from pymodbus.payload import BinaryPayloadDecoder	
from pymodbus.constants import Endian

#DEVICES TO CREATE
_UNIT_VOLTAGE_L1 = 1
_UNIT_VOLTAGE_L2 = 2
_UNIT_VOLTAGE_L3 = 3

_UNIT_CURRENT_L1 = 4
_UNIT_CURRENT_L2 = 5
_UNIT_CURRENT_L3 = 6

_UNIT_TOTALSYSTEMPOWER = 7
_UNIT_ACTIVEPOWER_L1 = 8
_UNIT_ACTIVEPOWER_L2 = 9
_UNIT_ACTIVEPOWER_L3 = 10

_UNIT_TOTALREACTIVEPOWER = 11
_UNIT_REACTIVEPOWER_L1 = 12
_UNIT_REACTIVEPOWER_L2 = 13
_UNIT_REACTIVEPOWER_L3 = 14

#_UNIT_POWERFACTOR_L1 = 15
#_UNIT_POWERFACTOR_L2 = 16
#_UNIT_POWERFACTOR_L3 = 17

_UNIT_FREQUENCY = 18

_UNIT_TOTALACTIVEENERGY = 19
_UNIT_TOTALREACTIVEENERGY = 20

_UNIT_TOTALIMPORTACTIVEENERGY = 21
_UNIT_TOTALEXPORTACTIVEENERGY = 22

_UNIT_NET_KWH = 23

_UNIT_IMPORTENERGY = 24
_UNIT_EXPORTENERGY = 25

#_UNIT_APPARENTPOWER_L1 = 26
#_UNIT_APPARENTPOWER_L2 = 27
#_UNIT_APPARENTPOWER_L3 = 28

#_UNIT_AVERAGELINETONEUTRALVOLTS = 29
#_UNIT_AVERAGELINECURRENT = 30
#_UNIT_SUMOFLINECURRENTS = 31

#_UNIT_TOTALSYSTEMVOLTAMPS = 32
#_UNIT_TOTALSYSTEMPOWERFACTOR = 33

#_UNIT_L1TOL2VOLTS = 34
#_UNIT_L2TOL3VOLTS = 35
#_UNIT_L3TOL1VOLTS = 36
#_UNIT_AVERAGELINETOLINEVOLTS = 37

#_UNIT_NEUTRALCURRENT = 38

#_UNIT_RESETTABLETOTALACTIVEENERGY = 39
#_UNIT_RESETTABLEIMPORTACTIVEENERGY = 40
#_UNIT_RESETTABLEEXPORTACTIVEENERGY = 41

#DEFAULT IMAGE
_NO_IMAGE_UPDATE = -1
_IMAGE = "SDM120"

#THE HAERTBEAT IS EVERY 10s
_MINUTE = 6

#VALUE TO INDICATE THAT THE DEVICE TIMED-OUT
_TIMEDOUT = 1

#DEBUG
_DEBUG_OFF = 0
_DEBUG_ON = 1


################################################################################
# Start Plugin
################################################################################

class BasePlugin:

    def __init__(self):
        self.debug = _DEBUG_OFF
        self.runAgain = 0
        self.StopBits = 1
        self.ByteSize = 8
        self.Parity = "N"
        self.Offset = 0
        return

    def onStart(self):
        Domoticz.Debug("onStart called")

        # Debugging On/Off
        if Parameters["Mode6"] == "Debug":
            self.debug = _DEBUG_ON
        else:
            self.debug = _DEBUG_OFF
        Domoticz.Debugging(self.debug)

        # Serial settings
        if (Parameters["Mode3"] == "S1B8PN"): self.StopBits, self.ByteSize, self.Parity = 1, 8, "N"
        if (Parameters["Mode3"] == "S1B8PE"): self.StopBits, self.ByteSize, self.Parity = 1, 8, "E"
        if (Parameters["Mode3"] == "S1B8PO"): self.StopBits, self.ByteSize, self.Parity = 1, 8, "O"
        if (Parameters["Mode3"] == "S2B8PN"): self.StopBits, self.ByteSize, self.Parity = 2, 8, "N"
        if (Parameters["Mode3"] == "S2B8PE"): self.StopBits, self.ByteSize, self.Parity = 2, 8, "E"
        if (Parameters["Mode3"] == "S2B8PO"): self.StopBits, self.ByteSize, self.Parity = 2, 8, "O"

        # Get Offset (kWh already consumed)
        self.Offset = float(Parameters["Mode4"])

        # Check if images are in database
        if _IMAGE not in Images:
            Domoticz.Image("SDM630MCT_v2.zip").Create()
        Domoticz.Debug("Images created.")

        # Create devices (USED BY DEFAULT)
        CreateDevicesUsed()

        # Create devices (NOT USED BY DEFAULT)
        CreateDevicesNotUsed()

        # Set all devices as timed out
        TimeoutDevice(All=True)

        # Global settings
        DumpConfigToLog()

    def onStop(self):
        Domoticz.Debug("onStop called")

    def onConnect(self, Connection, Status, Description):
        Domoticz.Debug("onConnect called")

    def onMessage(self, Connection, Data):
        Domoticz.Debug("onMessage called")

    def onCommand(self, Unit, Command, Level, Hue):
        Domoticz.Debug("onCommand called for Unit " + str(Unit) + ": Parameter '" + str(Command) + "', Level: " + str(Level))

    def onNotification(self, Name, Subject, Text, Status, Priority, Sound, ImageFile):
        Domoticz.Debug("Notification: " + Name + "," + Subject + "," + Text + "," + Status + "," + str(Priority) + "," + Sound + "," + ImageFile)

    def onDisconnect(self, Connection):
        Domoticz.Debug("onDisconnect called")

    def onHeartbeat(self):
        Domoticz.Debug("onHeartbeat called")
        self.runAgain -= 1
        if self.runAgain <= 0:

            # Get Offset (kWh already consumed)
            self.Offset = float(Parameters["Mode4"])

            # Open the ModBus interface
            try:
                client = ModbusSerialClient(method='rtu', port=Parameters["SerialPort"], stopbits=self.StopBits, bytesize=self.ByteSize, parity=self.Parity, baudrate=int(Parameters["Mode2"]), timeout=1, retries=2)
                Domoticz.Debug("Serial interface RTU opened successfully!")
            except:
                Domoticz.Log("Error opening Serial interface on " + Parameters["SerialPort"])
                TimeoutDevice(All=True)

            # Read the Sinotimer_3F energy information from the ModBus slave
            ReadModbus(client, "Voltage_L1",              0x0000, _UNIT_VOLTAGE_L1)                            #Volts
            ReadModbus(client, "Voltage_L2",              0x0002, _UNIT_VOLTAGE_L2)                            #Volts
            ReadModbus(client, "Voltage_L3",              0x0004, _UNIT_VOLTAGE_L3)                            #Volts
            
            ReadModbus(client, "Current_L1",              0x0006, _UNIT_CURRENT_L1)                            #Amps
            ReadModbus(client, "Current_L2",              0x0008, _UNIT_CURRENT_L2)                            #Amps
            ReadModbus(client, "Current_L3",              0x000A, _UNIT_CURRENT_L3)                            #Amps
            
            ReadModbus(client, "Total_System_Power",      0x0034, _UNIT_TOTALSYSTEMPOWER)                      #Watts
            
            ReadModbus(client, "Active_Power_L1",      	  0x000C, _UNIT_ACTIVEPOWER_L1)                        #Watts
            ReadModbus(client, "Active_Power_L2",      	  0x000E, _UNIT_ACTIVEPOWER_L2)                        #Watts
            ReadModbus(client, "Active_Power_L2",      	  0x0010, _UNIT_ACTIVEPOWER_L3)                        #Watts
            
            #ReadModbus(client, "Apparent_Power_L1",       0x0012, _UNIT_APPARENTPOWER_L1)                      #VA
            #ReadModbus(client, "Apparent_Power_L2",       0x0014, _UNIT_APPARENTPOWER_L2)                      #VA
            #ReadModbus(client, "Apparent_Power_L3",       0x0016, _UNIT_APPARENTPOWER_L3)                      #VA
            
            ReadModbus(client, "Total_Reactive_Power",    0x003C, _UNIT_TOTALREACTIVEPOWER)                    #KVAr
            
            ReadModbus(client, "Reactive_Power_L1",       0x0018, _UNIT_REACTIVEPOWER_L1)                      #VAr
            ReadModbus(client, "Reactive_Power_L2",       0x001A, _UNIT_REACTIVEPOWER_L2)                      #VAr
            ReadModbus(client, "Reactive_Power_L3",       0x001C, _UNIT_REACTIVEPOWER_L3)                      #VAr
            
            #ReadModbus(client, "Power_Factor_L1",         0x001E, _UNIT_POWERFACTOR_L1)                        #-
            #ReadModbus(client, "Power_Factor_L2",         0x0020, _UNIT_POWERFACTOR_L2)                        #-
            #ReadModbus(client, "Power_Factor_L3",         0x0022, _UNIT_POWERFACTOR_L3)                        #-

            ReadModbus(client, "Frequency",            	  0x0046, _UNIT_FREQUENCY)                             #Hz

            ReadModbus(client, "TotalActiveEnergy",    0x0156, _UNIT_TOTALACTIVEENERGY, self.Offset)     	    #kWh
            ReadModbus(client, "TotalReactiveEnergy",  0x0158, _UNIT_TOTALREACTIVEENERGY)                	    #kvarh
            ReadModbus(client, "TotalImportActiveEnergy",  0x0500, _UNIT_TOTALIMPORTACTIVEENERGY)               #Watts
            ReadModbus(client, "TotalExportActiveEnergy",  0x0502, _UNIT_TOTALEXPORTACTIVEENERGY)               #Watts
            ReadModbus(client, "NetkWh", 0x018C, _UNIT_NET_KWH)                                                 #kWh
            ReadModbus(client, "ImportEnergy", 0x0048, _UNIT_IMPORTENERGY)                                      #kWh
            ReadModbus(client, "ExportEnergy", 0x004A, _UNIT_EXPORTENERGY)                                      #kWh
            
            #ReadModbus(client, "Average line to neutral volts", 0x002A, _UNIT_AVERAGELINETONEUTRALVOLTS)        #V
            #ReadModbus(client, "Average line current", 0x002E, _UNIT_AVERAGELINECURRENT)                        #A
            #ReadModbus(client, "Sum of line currents", 0x0030, _UNIT_SUMOFLINECURRENTS)                         #A
            
            #ReadModbus(client, "Total_system_Volt_Amps", 0x0038, _UNIT_TOTALSYSTEMVOLTAMPS)                     #VA
            #ReadModbus(client, "Total_system_Power_Factor", 0x003E, _UNIT_TOTALSYSTEMPOWERFACTOR)               #None
            
            #ReadModbus(client, "L1_to_L2_Voltage", 0x00C8, _UNIT_L1TOL2VOLTS)                                   #V
            #ReadModbus(client, "L2_to_L3_Voltage", 0x00CA, _UNIT_L2TOL3VOLTS)                                   #V
            #ReadModbus(client, "L3_to_L1_Voltage", 0x00CC, _UNIT_L3TOL1VOLTS)                                   #V
            #ReadModbus(client, "Average_line_to_Line_Voltage", 0x00CE, _UNIT_AVERAGELINETOLINEVOLTS)            #V
            
            #ReadModbus(client, "Neutral_Current", 0x00E0, _UNIT_NEUTRALCURRENT)                                 #A
            
            #ReadModbus(client, "Ressetable_Total_Active_Energy", 0x0180, _UNIT_RESETTABLETOTALACTIVEENERGY)     #kWh
            #ReadModbus(client, "Ressetable_Import_Active_Energy", 0x0184, _UNIT_RESETTABLEIMPORTACTIVEENERGY)   #kWh
            #ReadModbus(client, "Ressetable_Export_Active_Energy", 0x0186, _UNIT_RESETTABLEEXPORTACTIVEENERGY)   #kWh
            
            # Run again following the period in the settings
            self.runAgain = _MINUTE*int(Parameters["Mode5"])
        else:
            Domoticz.Debug("onHeartbeat called, run again in "+str(self.runAgain)+" heartbeats.")

global _plugin
_plugin = BasePlugin()

def onStart():
    global _plugin
    _plugin.onStart()

def onStop():
    global _plugin
    _plugin.onStop()

def onConnect(Connection, Status, Description):
    global _plugin
    _plugin.onConnect(Connection, Status, Description)

def onMessage(Connection, Data):
    global _plugin
    _plugin.onMessage(Connection, Data)

def onCommand(Unit, Command, Level, Hue):
    global _plugin
    _plugin.onCommand(Unit, Command, Level, Hue)

def onNotification(Name, Subject, Text, Status, Priority, Sound, ImageFile):
    global _plugin
    _plugin.onNotification(Name, Subject, Text, Status, Priority, Sound, ImageFile)

def onDisconnect(Connection):
    global _plugin
    _plugin.onDisconnect(Connection)

def onHeartbeat():
    global _plugin
    _plugin.onHeartbeat()

################################################################################
# Generic helper functions
################################################################################

#DUMP THE PARAMETER
def DumpConfigToLog():
    for x in Parameters:
        if Parameters[x] != "":
            Domoticz.Debug("'" + x + "':'" + str(Parameters[x]) + "'")
    Domoticz.Debug("Device count: " + str(len(Devices)))
    for x in Devices:
        Domoticz.Debug("Device:           " + str(x) + " - " + str(Devices[x]))
        Domoticz.Debug("Device ID:       '" + str(Devices[x].ID) + "'")
        Domoticz.Debug("Device Name:     '" + Devices[x].Name + "'")
        Domoticz.Debug("Device nValue:    " + str(Devices[x].nValue))
        Domoticz.Debug("Device sValue:   '" + Devices[x].sValue + "'")
        Domoticz.Debug("Device LastLevel: " + str(Devices[x].LastLevel))

#UPDATE THE DEVICE
def UpdateDevice(Unit, nValue, sValue, Image=_NO_IMAGE_UPDATE, TimedOut=0, AlwaysUpdate=False):
    if Unit in Devices:
        if Devices[Unit].nValue != int(nValue) or Devices[Unit].sValue != str(sValue) or Devices[Unit].TimedOut != TimedOut or AlwaysUpdate:
            if Image != _NO_IMAGE_UPDATE:
                Devices[Unit].Update(nValue=int(nValue), sValue=str(sValue), Image=Image, TimedOut=TimedOut)
            else:
                Devices[Unit].Update(nValue=int(nValue), sValue=str(sValue), TimedOut=TimedOut)
            Domoticz.Debug("Update " + Devices[Unit].Name + ": " + str(nValue) + " - '" + str(sValue) + "'")

#SET DEVICE ON TIMED-OUT (OR ALL DEVICES)
def TimeoutDevice(All, Unit=0):
    if All:
        for x in Devices:
            UpdateDevice(x, Devices[x].nValue, Devices[x].sValue, TimedOut=_TIMEDOUT)
    else:
        UpdateDevice(Unit, Devices[Unit].nValue, Devices[Unit].sValue, TimedOut=_TIMEDOUT)

#CREATE ALL THE DEVICES (USED)
def CreateDevicesUsed():
    if (_UNIT_VOLTAGE_L1 not in Devices):
        Domoticz.Device(Name="Voltage L1", Unit=_UNIT_VOLTAGE_L1, Type=0xF3,Subtype=0x8,Options={"Custom": "0;V"},Used=1).Create()
    if (_UNIT_VOLTAGE_L2 not in Devices):
        Domoticz.Device(Name="Voltage L2", Unit=_UNIT_VOLTAGE_L2, Type=0xF3,Subtype=0x8,Options={"Custom": "0;V"},Used=1).Create()
    if (_UNIT_VOLTAGE_L3 not in Devices):
        Domoticz.Device(Name="Voltage L3", Unit=_UNIT_VOLTAGE_L3, Type=0xF3,Subtype=0x8,Options={"Custom": "0;V"},Used=1).Create()

    if (_UNIT_CURRENT_L1 not in Devices):
        Domoticz.Device(Name="Current L1", Unit=_UNIT_CURRENT_L1, TypeName="Custom", Options={"Custom": "0;A"}, Image=Images[_IMAGE].ID, Used=1).Create()
    if (_UNIT_CURRENT_L2 not in Devices):
        Domoticz.Device(Name="Current L2", Unit=_UNIT_CURRENT_L2, TypeName="Custom", Options={"Custom": "0;A"}, Image=Images[_IMAGE].ID, Used=1).Create()
    if (_UNIT_CURRENT_L3 not in Devices):
        Domoticz.Device(Name="Current L3", Unit=_UNIT_CURRENT_L3, TypeName="Custom", Options={"Custom": "0;A"}, Image=Images[_IMAGE].ID, Used=1).Create()

    if (_UNIT_TOTALSYSTEMPOWER not in Devices):
        Domoticz.Device(Name="Total System Power", Unit=_UNIT_TOTALSYSTEMPOWER, TypeName="Custom", Options={"Custom": "0;W"}, Image=Images[_IMAGE].ID, Used=1).Create()
    if (_UNIT_ACTIVEPOWER_L1 not in Devices):
        Domoticz.Device(Name="Active Power L1", Unit=_UNIT_ACTIVEPOWER_L1, TypeName="Custom", Options={"Custom": "0;W"}, Image=Images[_IMAGE].ID, Used=1).Create()
    if (_UNIT_ACTIVEPOWER_L2 not in Devices):
        Domoticz.Device(Name="Active Power L2", Unit=_UNIT_ACTIVEPOWER_L2, TypeName="Custom", Options={"Custom": "0;W"}, Image=Images[_IMAGE].ID, Used=1).Create()
    if (_UNIT_ACTIVEPOWER_L3 not in Devices):
        Domoticz.Device(Name="Active Power L3", Unit=_UNIT_ACTIVEPOWER_L3, TypeName="Custom", Options={"Custom": "0;W"}, Image=Images[_IMAGE].ID, Used=1).Create()

    if (_UNIT_FREQUENCY not in Devices):
        Domoticz.Device(Name="Frequency", Unit=_UNIT_FREQUENCY, TypeName="Custom", Options={"Custom": "0;Hz"}, Image=Images[_IMAGE].ID, Used=1).Create()

    if (_UNIT_TOTALACTIVEENERGY not in Devices):
        Domoticz.Device(Name="Total Active Energy", Unit=_UNIT_TOTALACTIVEENERGY, TypeName="Custom", Options={"Custom": "0;kWh"}, Image=Images[_IMAGE].ID, Used=1).Create()
        
    if (_UNIT_TOTALIMPORTACTIVEENERGY not in Devices):
        Domoticz.Device(Name="Total Import Active Energy", Unit=_UNIT_TOTALIMPORTACTIVEENERGY, TypeName="Custom", Options={"Custom": "0;W"}, Image=Images[_IMAGE].ID, Used=1).Create()

    if (_UNIT_TOTALEXPORTACTIVEENERGY not in Devices):
        Domoticz.Device(Name="Total Export Active Energy", Unit=_UNIT_TOTALEXPORTACTIVEENERGY, TypeName="Custom", Options={"Custom": "0;W"}, Image=Images[_IMAGE].ID, Used=1).Create()
    
    if (_UNIT_NET_KWH not in Devices):
        Domoticz.Device(Name="Net Kwh Import-Export", Unit=_UNIT_NET_KWH, TypeName="Custom", Options={"Custom": "0;kWh"}, Image=Images[_IMAGE].ID, Used=0).Create()
        
    if (_UNIT_IMPORTENERGY not in Devices):
        Domoticz.Device(Name="Import Energy", Unit=_UNIT_IMPORTENERGY, TypeName="Custom", Options={"Custom": "0;kWh"}, Image=Images[_IMAGE].ID, Used=1).Create()
    if (_UNIT_EXPORTENERGY not in Devices):
        Domoticz.Device(Name="Export Energy", Unit=_UNIT_EXPORTENERGY, TypeName="Custom", Options={"Custom": "0;kWh"}, Image=Images[_IMAGE].ID, Used=1).Create()

#CREATE ALL THE DEVICES (NOT USED)
def CreateDevicesNotUsed():
    #if (_UNIT_APPARENTPOWER_L1 not in Devices):
    #    Domoticz.Device(Name="Apparent Power L1", Unit=_UNIT_APPARENTPOWER_L1, TypeName="Custom", Options={"Custom": "0;VA"}, Image=Images[_IMAGE].ID, Used=0).Create()
    #if (_UNIT_APPARENTPOWER_L2 not in Devices):
    #    Domoticz.Device(Name="Apparent Power L2", Unit=_UNIT_APPARENTPOWER_L2, TypeName="Custom", Options={"Custom": "0;VA"}, Image=Images[_IMAGE].ID, Used=0).Create()
    #if (_UNIT_APPARENTPOWER_L3 not in Devices):
    #    Domoticz.Device(Name="Apparent Power L3", Unit=_UNIT_APPARENTPOWER_L3, TypeName="Custom", Options={"Custom": "0;VA"}, Image=Images[_IMAGE].ID, Used=0).Create()

    if (_UNIT_TOTALREACTIVEPOWER not in Devices):
        Domoticz.Device(Name="Total Reactive Power", Unit=_UNIT_TOTALREACTIVEPOWER, TypeName="Custom", Options={"Custom": "0;KVar"}, Image=Images[_IMAGE].ID, Used=1).Create()

    if (_UNIT_REACTIVEPOWER_L1 not in Devices):
        Domoticz.Device(Name="Reactive Power L1", Unit=_UNIT_REACTIVEPOWER_L1, TypeName="Custom", Options={"Custom": "0;Var"}, Image=Images[_IMAGE].ID, Used=0).Create()
    if (_UNIT_REACTIVEPOWER_L2 not in Devices):
        Domoticz.Device(Name="Reactive Power L2", Unit=_UNIT_REACTIVEPOWER_L2, TypeName="Custom", Options={"Custom": "0;Var"}, Image=Images[_IMAGE].ID, Used=0).Create()
    if (_UNIT_REACTIVEPOWER_L3 not in Devices):
        Domoticz.Device(Name="Reactive Power L3", Unit=_UNIT_REACTIVEPOWER_L3, TypeName="Custom", Options={"Custom": "0;Var"}, Image=Images[_IMAGE].ID, Used=0).Create()

    if (_UNIT_TOTALREACTIVEENERGY not in Devices):
        Domoticz.Device(Name="Total Reactive Energy", Unit=_UNIT_TOTALREACTIVEENERGY, TypeName="Custom", Options={"Custom": "0; kVArh"}, Image=Images[_IMAGE].ID, Used=0).Create()

    #if (_UNIT_POWERFACTOR_L1 not in Devices):
    #    Domoticz.Device(Name="Power factor L1", Unit=_UNIT_POWERFACTOR_L1, TypeName="Custom", Options={"Custom": "0;"}, Image=Images[_IMAGE].ID, Used=0).Create()
    #if (_UNIT_POWERFACTOR_L2 not in Devices):
    #    Domoticz.Device(Name="Power factor L2", Unit=_UNIT_POWERFACTOR_L2, TypeName="Custom", Options={"Custom": "0;"}, Image=Images[_IMAGE].ID, Used=0).Create()
    #if (_UNIT_POWERFACTOR_L3 not in Devices):
    #    Domoticz.Device(Name="Power factor L3", Unit=_UNIT_POWERFACTOR_L3, TypeName="Custom", Options={"Custom": "0;"}, Image=Images[_IMAGE].ID, Used=0).Create()
        
    #if (_UNIT_AVERAGELINETONEUTRALVOLTS not in Devices):
    #    Domoticz.Device(Name="Average Line to neutral Volts", Unit=_UNIT_AVERAGELINETONEUTRALVOLTS, Type=0xF3,Subtype=0x8,Options={"Custom": "0;V"},Used=0).Create()
    #if (_UNIT_AVERAGELINECURRENT not in Devices):
    #    Domoticz.Device(Name="Average Line Current", Unit=_UNIT_AVERAGELINECURRENT, TypeName="Custom", Options={"Custom": "0;A"}, Image=Images[_IMAGE].ID, Used=0).Create()
    #if (_UNIT_SUMOFLINECURRENTS not in Devices):
    #    Domoticz.Device(Name="Sum of line currents", Unit=_UNIT_SUMOFLINECURRENTS, TypeName="Custom", Options={"Custom": "0;A"}, Image=Images[_IMAGE].ID, Used=0).Create()
        
    #if (_UNIT_TOTALSYSTEMVOLTAMPS not in Devices):
    #    Domoticz.Device(Name="Total system Volt Amps", Unit=_UNIT_TOTALSYSTEMVOLTAMPS, TypeName="Custom", Options={"Custom": "0;VA"}, Image=Images[_IMAGE].ID, Used=0).Create()
    #if (_UNIT_TOTALSYSTEMPOWERFACTOR not in Devices):
    #    Domoticz.Device(Name="Total system Power Factor", Unit=_UNIT_TOTALSYSTEMPOWERFACTOR, TypeName="Custom", Options={"Custom": "0;"}, Image=Images[_IMAGE].ID, Used=0).Create()
        
    #if (_UNIT_L1TOL2VOLTS not in Devices):
    #    Domoticz.Device(Name="L1 to L2 Voltage", Unit=_UNIT_L1TOL2VOLTS, Type=0xF3,Subtype=0x8,Options={"Custom": "0;V"},Used=0).Create()
    #if (_UNIT_L2TOL3VOLTS not in Devices):
    #    Domoticz.Device(Name="L2 to L3 Voltage", Unit=_UNIT_L2TOL3VOLTS, Type=0xF3,Subtype=0x8,Options={"Custom": "0;V"},Used=0).Create()
    #if (_UNIT_L3TOL1VOLTS not in Devices):
    #    Domoticz.Device(Name="L3 to L1 Voltage", Unit=_UNIT_L3TOL1VOLTS, Type=0xF3,Subtype=0x8,Options={"Custom": "0;V"},Used=0).Create()
    #if (_UNIT_AVERAGELINETOLINEVOLTS not in Devices):
    #    Domoticz.Device(Name="Average line to Line Voltage", Unit=_UNIT_AVERAGELINETOLINEVOLTS, Type=0xF3,Subtype=0x8,Options={"Custom": "0;V"},Used=0).Create()
        
    #if (_UNIT_NEUTRALCURRENT not in Devices):
    #    Domoticz.Device(Name="Neutral Current", Unit=_UNIT_NEUTRALCURRENT, TypeName="Custom", Options={"Custom": "0;A"}, Image=Images[_IMAGE].ID, Used=0).Create()
        
    #if (_UNIT_RESETTABLETOTALACTIVEENERGY not in Devices):
    #    Domoticz.Device(Name="Ressetable Total Active Energy", Unit=_UNIT_RESETTABLETOTALACTIVEENERGY, TypeName="Custom", Options={"Custom": "0;kWh"}, Image=Images[_IMAGE].ID, Used=0).Create()
    #if (_UNIT_RESETTABLEIMPORTACTIVEENERGY not in Devices):
    #    Domoticz.Device(Name="Ressetable Import Active Energy", Unit=_UNIT_RESETTABLEIMPORTACTIVEENERGY, TypeName="Custom", Options={"Custom": "0;kWh"}, Image=Images[_IMAGE].ID, Used=0).Create()
    #if (_UNIT_RESETTABLEEXPORTACTIVEENERGY not in Devices):
    #    Domoticz.Device(Name="Ressetable Export Active Energy", Unit=_UNIT_RESETTABLEEXPORTACTIVEENERGY, TypeName="Custom", Options={"Custom": "0;kWh"}, Image=Images[_IMAGE].ID, Used=0).Create()
        
        
#READ THE MODBUS INFORMATION
def ReadModbus(client, StrData, Address, Unit, Offset=0): 
    try:
        data = client.read_input_registers(address=Address, count=2, unit=int(Parameters["Mode1"]))
        if data.isError():
            data = client.read_input_registers(address=Address, count=2, unit=int(Parameters["Mode1"]))
        decoder = BinaryPayloadDecoder.fromRegisters(data.registers, byteorder=Endian.Big, wordorder=Endian.Big)
        value = round(decoder.decode_32bit_float(), 4) 
        Domoticz.Debug('%s: %.4f' % (StrData, value))
        UpdateDevice(Unit, nValue=value+Offset, sValue='%.4f'%(value+Offset), AlwaysUpdate=False)
    except:
        Domoticz.Error("Error reading data (%s)." % (StrData))
        TimeoutDevice(All=False, Unit=Unit)

