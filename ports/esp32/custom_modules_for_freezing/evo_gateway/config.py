# -*- coding: utf-8 -*-
#!/usr/bin/python
# evohome Listener/Sender
# Copyright (c) 2020 SMAR info@smar.co.uk
#
# Tested with Python 3.6.8 Requires:
# - pyserial (python -m pip install pyserial)
# - paho (pip install paho-mqtt)
#
# Simple evohome 'listener' and 'sender', for listening in and sending messages between evohome devices using an arudino + CC1101 radio receiver
# (other hardware options also possible - see credits below).
# Messages are interpreted and then posted to an mqtt broker if an MQTT broker is defined in the configuration. Similary, sending commands over the
# radio network are initiated via an mqtt 'send' topic, and 'send' status updates posted back to an mqtt topic.
#
# CREDITS:
# Code here is significntly based on the Domitcz source, specifically the EvohomeRadio.cpp file, by
# fulltalgoRythm - https://github.com/domoticz/domoticz/blob/development/hardware/EvohomeRadio.cpp
# Also see http://www.automatedhome.co.uk/vbulletin/showthread.php?5085-My-HGI80-equivalent-Domoticz-setup-without-HGI80
# for info and discussions on homebrew hardware options.
#
# Details on the evohome protocol can be found here: https://github.com/Evsdd/The-Evohome-Protocol/wiki
#
# The arduino nano I am using is running a firmware modded by ghoti57 available
# from https://github.com/ghoti57/evofw2, who had forked it from
# codeaholics, https://github.com/Evsdd, who in turn had forked it  from
# fulltalgoRythm's orignal firmware, https://github.com/fullTalgoRythm/EvohomeWirelessFW.
#
# OpenTherm protocol decoding taken from https://github.com/Evsdd/The-Evohome-Protocol/wiki/3220:-OpenTherm-Message
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


import os,sys,uos
import configparser
import re

import time#, datetime
import json
import re
from collections import namedtuple, deque
import os.path


#---------------------------------------------------------------------------------------------------
VERSION         = "2.1.0"
CONFIG_FILE     = "evogateway.cfg"

# --- Configs/Default
def getConfig(config,section,name,default):
    if config.has_option(section,name):
        return config.get(section,name)
    else:
        return default


# Get any configuration overrides that may be defined in  CONFIG_FILE
# If override not specified, then use the defaults here

config = configparser.RawConfigParser()
config.read(CONFIG_FILE)

# Use json config for multiple com ports if available
COM_PORTS         = getConfig(config,"Serial Port", "COM_PORTS", None)
if COM_PORTS and type(COM_PORTS) == str:
  COM_PORTS = json.loads(COM_PORTS.replace("'", "\""))

# otherwise default to single port
if COM_PORTS is None:
  COM_PORT          = getConfig(config,"Serial Port","COM_PORT","/dev/ttyUSB0")
  COM_BAUD          = int(getConfig(config,"Serial Port","COM_BAUD",115200))
  COM_RETRY_LIMIT   = int(getConfig(config,"Serial Port","COM_RETRY_LIMIT",10))
  COM_TX_PIN = int(getConfig(config, "Serial Port", "COM_TX_PIN", 17))
  COM_RX_PIN = int(getConfig(config, "Serial Port", "COM_RX_PIN", 16))
  COM_PORTS = {COM_PORT: {"baud" : COM_BAUD, "retry_limit": COM_RETRY_LIMIT, "is_send_port": True, "tx_pin": COM_TX_PIN, "rx_pin": COM_RX_PIN}}

EVENTS_FILE       = getConfig(config,"Files", "EVENTS_FILE", "events.log")
LOG_FILE          = getConfig(config,"Files", "LOG_FILE", "evogateway.log")
DEVICES_FILE      = getConfig(config,"Files", "DEVICES_FILE", "devices.json")
NEW_DEVICES_FILE  = getConfig(config,"Files", "NEW_DEVICES_FILE", "devices_new.json")
MAX_FILE_SIZE  = getConfig(config,"Files", "MAX_FILE_SIZE", 15000)

LOG_DROPPED_PACKETS = getConfig(config,"Other", "LOG_DROPPED_PACKETS", "False").lower() == "true"
DROP_DUPLICATE_MESSAGES = getConfig(config,"Other", "DROP_DUPLICATE_MESSAGES", "True").lower() == "true"

MQTT_SERVER       = getConfig(config,"MQTT", "MQTT_SERVER", "")                  # Leave blank to disable MQTT publishing. Messages will still be saved in the various files
MQTT_SUB_TOPIC    = getConfig(config,"MQTT", "MQTT_SUB_TOPIC", "")               # Note to exclude any trailing '/'
MQTT_PUB_TOPIC    = getConfig(config,"MQTT", "MQTT_PUB_TOPIC", "")
MQTT_USER         = getConfig(config,"MQTT", "MQTT_USER", "")
MQTT_PW           = getConfig(config,"MQTT", "MQTT_PW", "")
MQTT_CLIENTID     = getConfig(config,"MQTT", "MQTT_SERVER", "evoGateway")

CONTROLLER_ID     = getConfig(config,"SENDER", "CONTROLLER_ID", "01:139901")
THIS_GATEWAY_ID   = getConfig(config,"SENDER", "THIS_GATEWAY_ID","18:318170")
THIS_GATEWAY_NAME = getConfig(config,"SENDER", "THIS_GATEWAY_NAME","EvoGateway")
THIS_GATEWAY_TYPE_ID = THIS_GATEWAY_ID.split(":")[0]

COMMAND_RESEND_TIMEOUT_SECS = float(getConfig(config,"SENDER", "COMMAND_RESEND_TIMEOUT_SECS", 60.0))
COMMAND_RESEND_ATTEMPTS= int(getConfig(config,"SENDER", "COMMAND_RESEND_ATTEMPTS", 3))    # A value of zero also disables waiting for sent command acknowledgements
AUTO_RESET_PORTS_ON_FAILURE = getConfig(config,"SENDER", "AUTO_RESET_PORTS_ON_FAILURE", "False").lower() == "true"

MAX_LOG_HISTORY   = int(getConfig(config,"SENDER", "MAX_LOG_HISTORY", 3))
MIN_ROW_LENGTH    = int(getConfig(config,"MISC", "MIN_ROW_LENGTH", 160))

MAX_HISTORY_STACK_LENGTH = 10

EMPTY_DEVICE_ID   = "--:------"

SYS_CONFIG_COMMAND = "sys_config"
RESET_COM_PORTS   = "reset_com_ports"
CANCEL_SEND_COMMANDS ="cancel_commands"

SYSTEM_MSG_TAG = "*"
#----------------------------------------