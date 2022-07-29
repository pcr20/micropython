# -*- coding: utf-8 -*-
# !/usr/bin/python
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


import os, sys, uos

import re

import time  # , datetime
import io
import json
import re
from collections import namedtuple, deque

from evo_gateway.config import THIS_GATEWAY_TYPE_ID
from evo_gateway.config import THIS_GATEWAY_NAME
from evo_gateway.config import EMPTY_DEVICE_ID
from evo_gateway.config import CONTROLLER_ID
from evo_gateway.config import NEW_DEVICES_FILE

from evo_gateway.config import MQTT_PUB_TOPIC

from evo_gateway.general import display_and_log
from evo_gateway.general import log
from evo_gateway.general import to_snake

import evo_gateway.globalcfg as gcfg
from evo_gateway.mqtt import mqtt_publish
from evo_gateway.app_helper import update_zone_details
from evo_gateway.app_helper import error_log

def get_dtm_from_packed_hex(dtm_hex):
    dtm_mins = int(dtm_hex[0:2], 16)
    dtm_hours = int(dtm_hex[2:4], 16)
    dtm_day = int(dtm_hex[4:6], 16)
    dtm_month = int(dtm_hex[6:8], 16)
    dtm_year = int(dtm_hex[8:12], 16)
    # return datetime.datetime(year=dtm_year,month=dtm_month, day=dtm_day,hour=dtm_hours,minute=dtm_mins)
    return (dtm_year, dtm_month, dtm_day, dtm_hours, dtm_mins, 0)


def display_data_row(msg, display_text, ref_zone=-1, suffix_text=""):
    destination = "BROADCAST MESSAGE" if msg.is_broadcast() else msg.destination_name
    if ref_zone > -1:
        zone_name = "@ {:<20}".format(gcfg.zones[ref_zone]) if ref_zone in gcfg.zones else "                      "
        display_row = "{:<2}| {:<22} -> {:<22} | {:>5} {:<25} [Zone {:<3}] {}".format(
            msg.msg_type, msg.source_name, destination, display_text, zone_name, ref_zone, suffix_text)
    else:
        display_row = "{:<2}| {:<22} -> {:<22} | {:>5} {}".format(msg.msg_type, msg.source_name, destination,
                                                                  display_text, suffix_text)
    display_and_log(msg.command_name, display_row, msg.port, msg.rssi)


def parity(x):
    shiftamount = 1
    while x >> shiftamount:
        x ^= x >> shiftamount
        shiftamount <<= 1
    return x & 1


def convert_from_twos_comp(hex_val, divisor=100):
    """ Converts hex string of 2's complement """
    try:
        val = float(int(hex_val[0:2], 16) << 8 | int(hex_val[2:4], 16)) / divisor
        return val
    except Exception as e:
        fio = io.StringIO()
        sys.print_exception(e, fio)
        fio.seek(0)
        errmsg="Two's complement error {}. hex_val argument: {} {}".format(e, hex_val,fio.read())
        display_and_log("ERROR",errmsg)
        error_log(display_message=errmsg)


class TwoWayDict(dict):
    def __len__(self):
        return super().__len__() / 2

    def __setitem__(self, key, value):
        super().__setitem__(key, value)
        super().__setitem__(value, key)


DEVICE_TYPE = TwoWayDict()
DEVICE_TYPE["01"] = "CTL"  # Main evohome touchscreen controller
DEVICE_TYPE["02"] = "UFH"  # Underfloor controller, HCC80R or HCE80
DEVICE_TYPE["03"] = "STAT"  # Wireless thermostat -  HCW82
DEVICE_TYPE["04"] = "TRV"  # Radiator TRVs, e.g. HR92
DEVICE_TYPE["07"] = "DHW"  # Hotwater wireless Sender
DEVICE_TYPE["10"] = "OTB"  # OpenTherm Bridge
DEVICE_TYPE["18"] = "CUL"  # This fake HGI80
DEVICE_TYPE["19"] = "CUL"  # Also fake HGI80 - used by evofw2?
DEVICE_TYPE["13"] = "BDR"  # BDR relays
DEVICE_TYPE["30"] = "GWAY"  # Mobile Gateway such as RGS100
DEVICE_TYPE["34"] = "STAT"  # Wireless round thermostats T87RF2033 or part of Y87RF2024

OPENTHERM_MSG_TYPES = {
    0: "Read-Data",  # 0b.000....
    16: "Write-Data",  # 0b.001....
    32: "Invalid-Data",  # 0b.010....
    48: "-reserved-",  # 0b.011....
    64: "Read-Ack",  # 0b.100....
    80: "Write-Ack",  # 0b.101....
    96: "Data-Invalid",  # 0b.110....
    112: "Unknown-DataId",  # 0b.111....
}

CONTROLLER_MODES = {0: "Auto", 1: "Heating Off", 2: "Eco-Auto", 3: "Away", 4: "Day Off",
                    7: "Custom"}  # 0=auto, 1= off, 2=eco, 4 = day off, 7 = custom


# --- Classes
class Message():
    ''' Object to hold details of interpreted (received) message. '''

    def __init__(self, rawmsg, has_rssi=False):
        offset = 4 if has_rssi else 0  # new ghoti57 fifo_hang branch has rssi

        self.rawmsg = rawmsg.strip()

        self.rssi = rawmsg[4:7] if has_rssi else None

        self.source_id = rawmsg[11 + offset: 20 + offset]
        self.msg_type = rawmsg[4 + offset:  6 + offset].strip()
        self.source = rawmsg[
                      11 + offset: 20 + offset]  # device 1 - This looks as if this is always the source; Note this is overwritten with name of +device
        self.source_type = rawmsg[
                           11 + offset: 13 + offset]  # the first 2 digits seem to be identifier for type of +device
        self.source_name = self.source
        self.device2 = rawmsg[
                       21 + offset: 30 + offset]  # device 2 - Requests (RQ), Responses (RP) and Write (W) seem to be sent to device 2 +only
        self.device2_type = rawmsg[21 + offset: 23 + offset]  # device 2 +type
        self.device2_name = self.device2
        self.device3 = rawmsg[
                       31 + offset: 40 + offset]  # device 3 - Information (I) is to device 3. Broadcast messages have device 1 and 3 are the +same
        self.device3_type = rawmsg[31 + offset: 33 + offset]  # device 3 +type
        self.device3_name = self.device3

        if self.device2 == EMPTY_DEVICE_ID:
            self.destination = self.device3
            self.destination_type = self.device3_type
        else:
            self.destination = self.device2
            self.destination_type = self.device2_type
        self.destination_name = self.destination

        # There seem to be some OpenTherm messages without a source/device2; just device3.
        # Could be firmware issue with source/dest swapped? For now, let's swap over and see...
        if self.source == EMPTY_DEVICE_ID and self.device3 != EMPTY_DEVICE_ID:
            self.source = self.device3
            self.source_type = self.device3_type
            self.device3 = rawmsg[11:20]
            self.devic3_type = rawmsg[11:13]

        self._initialise_device_names()

        self.command_code = rawmsg[41 + offset: 45 + offset].upper()  # command code +hex
        self.command_name = self.command_code  # needs to be assigned outside, as we are doing all the processing outside of this class/struct
        try:
            self.payload_length = int(rawmsg[46 + offset: 49 + offset])  # Note this is not +HEX...
        except Exception as e:
            fio = io.StringIO()
            sys.print_exception(e, fio)
            fio.seek(0)
            errmsg="Error instantiating Message class: {} {}. Raw msg: '{}'. length = {}".format(str(e),fio.read(), rawmsg, +len(rawmsg))
            print(errmsg)
            error_log(display_message=errmsg)
            self.payload_length = 0

        self.payload = rawmsg[50 + offset:]
        self.port = None
        self.failed_decrypt = "_ENC" in rawmsg or "_BAD" in rawmsg or "BAD_" in rawmsg or "ERR" in rawmsg

    def _initialise_device_names(self):
        ''' Substitute device IDs for names if we have them or identify broadcast '''
        try:
            if self.source_type == DEVICE_TYPE['CTL'] and self.is_broadcast():
                self.destination_name = "CONTROLLER"
                self.source_name = "CONTROLLER"
            elif DEVICE_TYPE[self.source_type] and self.source in gcfg.devices and gcfg.devices[self.source]['name']:
                self.source_name = "{} {}".format(DEVICE_TYPE[self.source_type], gcfg.devices[self.source][
                    'name'])  # Get the device's actual name if we have it
            else:
                print("Could not find device type '{}' or name for device '{}'".format(self.source_type, self.source))
            if self.destination_name != "CONTROLLER" and self.destination in gcfg.devices and \
                    gcfg.devices[self.destination][
                        'name']:
                if self.destination_type == DEVICE_TYPE['CTL']:
                    self.destination_name = "CONTROLLER"
                else:
                    device_name = gcfg.devices[self.destination][
                        'name'] if self.destination in gcfg.devices else self.destination
                    self.destination_name = "{} {}".format(DEVICE_TYPE[self.destination_type],
                                                           gcfg.devices[self.destination][
                                                               'name'])  # Get the device's actual name if we have it
        except Exception as e:
            fio = io.StringIO()
            sys.print_exception(e, fio)
            fio.seek(0)
            errmsg="Error initalising device names in Message class instantiation, '{}': {}. Raw msg: '{}'. length = {}".format(
                    str(e),fio.read(), self.rawmsg, len(self.rawmsg))
            print(errmsg)
            error_log(display_message=errmsg)

    def is_broadcast(self):
        return self.source == self.destination

    def get_raw_msg_with_ts(self, strip_rssi=True):
        t = time.gmtime()
        if self.rssi:  # remove the rssi before saving to stack - different controllers may receive the same message but with different signal strengths
            raw = "{}-{}-{} {}:{}:{}: {}".format(t[0], t[1], t[2], t[3], t[4], t[5], "--- {}".format(self.rawmsg[8:]))
            # raw = "{}: {}".format(datetime.datetime.now().strftime("%Y-%m-%d %X"), "--- {}".format(self.rawmsg[8:]))
        else:
            raw = "{}-{}-{} {}:{}:{}: {}".format(t[0], t[1], t[2], t[3], t[4], t[5], self.rawmsg)
            # raw = "{}: {}".format(datetime.datetime.now().strftime("%Y-%m-%d %X"), self.rawmsg)
        return raw


class Command():
    ''' Object to hold details of command sent to evohome controller.'''

    def __init__(self, command_code=None, command_name=None, destination=None, args=None, serial_port=-1, send_mode="I",
                 instruction=None):
        self.command_code = command_code
        self.command_name = command_name
        self.destination = destination
        self.args = args
        self.arg_desc = "[]"
        self.serial_port = serial_port
        self.send_mode = send_mode
        self.send_string = None
        self.send_dtm = None
        self.retry_dtm = None
        self.retries = 0
        self.send_failed = False
        self.wait_for_ack = False
        self.reset_ports_on_fail = True
        self.send_acknowledged = False
        self.send_acknowledged_dtm = None
        self.dev1 = None
        self.dev2 = None
        self.dev3 = None
        self.payload = ""
        self.command_instruction = instruction

    def payload_length(self):
        if self.payload:
            return int(len(self.payload) / 2)
        else:
            return 0


def init_homie():
    # WIP....

    # This server is the Gateway device - i.e. the homie device
    # All other devices on the evohome network, including controllers etc, are treated as nodes of this gateway device

    device_type_properties = {
        "CTL": ["temperature", "setpoint", "until", "heat-demand"],
        "UFH": [],
        "TRV": ["temperature", "setpoint", "until", "heat-demand", "window"],
        "DHW": ["state", "temperature", "dhw-mode", "until"],
        "BDR": ["temperature", "heat-demand"],
        "GWAY": [],
        "STAT": ["temperature", "setpoint", "until"]
    }

    msgs = []
    topic_base = "homie/evohome-gateway-{}".format(THIS_GATEWAY_ID.replace(":", "-"))
    msgs.append(("{}/$homie".format(topic_base), 3.0, 0, True))
    msgs.append(("{}/$name".format(topic_base), "evohome Listener/Sender Gateway {}".format(THIS_GATEWAY_ID), 0, True))
    msgs.append(("{}/$state".format(topic_base), "ready", 0, True))
    msgs.append(("{}/$extensions".format(topic_base), "", 0, True))

    nodes_list = []
    node_msgs = []
    nodes_topic = "{}/$nodes".format(topic_base)

    for device_id, gcfg.device in gcfg.devices.items():
        device_type = device_id.split(":")[0]
        device_name = to_snake(device["name"]).lower().replace("_", "-")

        zone_id = device["zoneId"]
        zone_name = gcfg.zones[zone_id]  # if 1 <= zone_id <= 12 else str(zone_id)

        node_name = "{}-{}-{}".format(DEVICE_TYPE[device_type].lower(), device_name, device_id.replace(":", "-"))
        nodes_list.append(node_name)

        node_topic = "{}/{}".format(topic_base, node_name)

        # Add to device object, for reuse later
        device['topic'] = node_topic
        device['node_name'] = node_name

        node_msgs.append(
            ("{}/$name".format(node_topic), "[{:<4}] {}".format(DEVICE_TYPE[device_type], device["name"]), 0, True))
        node_msgs.append(("{}/$type".format(node_topic), "[{:<4}]".format(DEVICE_TYPE[device_type]), 0, True))

        node_properties = ",".join(device_type_properties[DEVICE_TYPE[device_type]])
        node_msgs.append(("{}/$properties".format(node_topic), node_properties, 0, True))

        for device_property in device_type_properties[DEVICE_TYPE[device_type]]:
            node_msgs += get_homie_node_topics(node_topic, device_property)

    msgs.append(("{}/$nodes".format(topic_base), ",".join(nodes_list), 0, True))
    msgs += node_msgs
    return msgs


def get_homie_node_topics(node_topic, node_property):
    property_params = namedtuple("property_params", "property_name unit datatype format settable default")

    if node_property in "temperature":
        params = property_params("Temperature", "°C", "float", "25:100", "false", 5.0)
    elif node_property in "setpoint":
        params = property_params("Setpoint", "°C", "float", "25:100", "false", 5.0)
    elif node_property in "until":
        params = property_params("Temporary Until", "°C", "string", "", "false", "")
    elif node_property in "heat-demand":
        params = property_params("Heat Demand", "%", "float", "0:100", "false", 0.0)
    elif node_property in "window":
        params = property_params("Room Window Status", "", "enum", "OPEN,CLOSED", "false", "CLOSED")
    elif node_property in "actuator-status":
        params = property_params("Actuator Status", "", "enum", "ON:OFF", "false", "OFF")
    elif node_property in "dhw-mode":
        params = property_params("DHW Mode", "", "enum", "AUTO,ON,OFF,TIMED", "false", "AUTO")
    elif node_property in "state":
        params = property_params("State", "", "enum", "ON,OFF", "false", "OFF")

    topic_base = "{}/{}".format(node_topic, node_property)
    msgs = []
    msgs.append((topic_base, params.default, 0, True))
    msgs.append(("{}/$name".format(topic_base), params.property_name, 0, True))
    msgs.append(("{}/$unit".format(topic_base), params.unit, 0, True))
    msgs.append(("{}/$datatype".format(topic_base), params.datatype, 0, True))
    msgs.append(("{}/$format".format(topic_base), params.format, 0, True))
    msgs.append(("{}/$settable".format(topic_base), params.settable, 0, True))
    return msgs


data_pattern = re.compile(
    "^--- ( I| W|RQ|RP) --- (--:------ |\d\d:\d\d\d\d\d\d )(--:------ |\d\d:\d\d\d\d\d\d )(--:------ |\d\d:\d\d\d\d\d\d )[0-9a-fA-F][0-9a-fA-F][0-9a-fA-F][0-9a-fA-F] \d\d\d")
data_pattern_with_rssi = re.compile(
    "^--- \d\d\d ( I| W|RQ|RP) --- (--:------ |\d\d:\d\d\d\d\d\d )(--:------ |\d\d:\d\d\d\d\d\d )(--:------ |\d\d:\d\d\d\d\d\d )[0-9a-fA-F][0-9a-fA-F][0-9a-fA-F][0-9a-fA-F] \d\d\d")


# --- evohome received message command processing functions
def get_message_from_data(data, port_tag=None):
    ''' Convert the received raw data into a Message object  '''
    if not (
            "Invalid Manchester" in data or "Collision" in data or "Truncated" in data or "_ENC" in data or "_BAD" in data or "BAD_" in data or "ERR" in data) and len(
        data) > 40:  # Make sure no obvious errors in getting the data....
        if not data.startswith("---"):
            # Some echos of commands sent by us seem to come back without the --- prefix. Noticed on the fifo firmware that sometimes the request type prefix seems to be messed up. Workaround for this...
            if data.strip().startswith("W---"):
                data = data[1:]
            else:
                data = "---  {}".format(data) if len(data.split(" ", 1)[0]) < 2 else "--- {}".format(data)

        old_fw = data_pattern.match(data)  # older firmware did not include rssi in data
        newer_fw = data_pattern_with_rssi.match(data)  # newer fw (including evofw3) has rssi

        if old_fw or newer_fw:
            msg = Message(data, newer_fw)
            msg.port = port_tag
            return msg
        elif not re.search("^--- ([a-z0-9][a-z0-9]\.)*",
                           data):  # Pattern for some sort of debug msg from the new evofw3. If so, ignore these...
            display_and_log("ERROR", "Pattern match failed on received data: '{}'".format(data), port_tag)

    return None


def process_received_message(msg):
    ''' Process the Message object '''
    if not msg:
        display_and_log("ERROR", "Cannot process message as Message object is None")
        return None

    # Check if device is known...
    if not msg.source in gcfg.devices:
        display_and_log("NEW DEVICE FOUND", msg.source)
        gcfg.devices.update({msg.source: {"name": msg.source, "zoneId": -1, "zoneMaster": False}})
        with open(NEW_DEVICES_FILE, 'w') as fp:
            # fp.write(json.dumps(gcfg.devices, sort_keys=True, indent=4))
            json.dump(gcfg.devices,fp)
        fp.close()

    if msg.command_code in COMMANDS:
        try:
            msg.command_name = COMMANDS[
                msg.command_code].__name__.upper()  # Get the name of the command from our list of commands
            COMMANDS[msg.command_code](msg)
            log('{: <18} {}'.format(msg.command_name, msg.rawmsg), msg.port)
        except Exception as e:
            fio = io.StringIO()
            sys.print_exception(e, fio)
            fio.seek(0)
            errmsg="'{}' on line {} [Command {}, data: '{}', port: {}]".format(str(e),fio.read(),
                                                                                                 msg.command_name,
                                                                                                 msg.rawmsg, msg.port)
            display_and_log("ERROR",errmsg)
            error_log(display_message=errmsg)                                                                 
            # print(traceback.format_exc())
            return None
        return msg
    else:
        msg.command_name = "UNKNOWN COMMAND"
        display_data_row(msg, "Command code: {}, Payload: {}".format(msg.command_code, msg.payload))
        return None


def get_zone_details(payload, source_type=None):
    zone_id = int(payload[0:2], 16)
    if zone_id < 12:
        zone_id += 1
        zone_name = gcfg.zones[zone_id] if zone_id in gcfg.zones else "Zone {}".format(zone_id)
        topic = zone_name
    else:
        # if zone_id == DEVICE_TYPE['UFH']:
        #   topic="UFH Controller"
        if zone_id == 0xfa:  # Depends on whether it is main controller sending message, or UFH controller
            if source_type and str(source_type) == DEVICE_TYPE["UFH"]:
                zone_name = "UFH Controller"
            else:
                zone_name = "BDR DHW Relay"
        elif zone_id == 0xfc:  # Boiler relay or possibly broadcast
            zone_name = "OTB OpenTherm Bridge" if [k for k in gcfg.devices if "10:" in k] else "BDR Boiler Relay"
        elif zone_id == 0xf9:  # Radiator circuit zone valve relay
            zone_name = "BDR Radiators Relay"
        elif zone_id == 0xc:  # Electric underfloor relay
            zone_name = "UFH Electric Relay"
        else:
            device_type = str(hex(msg.source_type))
            zone_name = "RLY {}".format(device_type)
        topic = "Relays/{}".format(zone_name)
    return zone_id, zone_name, topic


def bind(msg):
    display_data_row(msg, "Payload: {}".format(msg.payload), -1, "Payload length: {}".format(msg.payload_length))


def sync(msg):
    # https://www.domoticaforum.eu/viewtopic.php?f=7&t=5806&start=120#p73918
    # Basically the controller sends a broadcast write 1f09 with f8 in the first byte
    # and the last 2 bytes giving the time in 10ths of a second to the next broadcast message.
    # If the TRVs don't hear the controller for a while they start sending a 1f09 request presumably looking for a 1f09 reply
    # giving the time to the next broadcast

    if msg.payload[0:2] == "FF":
        timeout = int(msg.payload[2:6], 16) / 10
        # timeout_time = (datetime.datetime.now() + datetime.timedelta(seconds = timeout)).strftime("%H:%M:%S")
        timeout_t = time.gmtime(time.time() + int(timeout))
        timeout_time = "{:02d}:{:02d}:{:02d}".format(timeout_t[3], timeout_t[4], timeout_t[5])
        display_data_row(msg, "Next sync at {} (in {} secs)".format(timeout_time, timeout))
    else:
        display_data_row(msg, "Payload: {}".format(msg.payload))


def schedule_sync(msg):
    display_data_row(msg, "Payload: {}".format(msg.payload), -1, "Payload length: {}".format(msg.payload_length))
    # The 0x0006 command is a schedule sync message sent between the gateway to controller to check whether there have been any changes since the last exchange of schedule information
    # https://www.automatedhome.co.uk/vbulletin/showthread.php?5085-My-HGI80-equivalent-Domoticz-setup-without-HGI80/page13


def zone_name(msg):
    display_data_row(msg, "Payload: {}".format(msg.payload), -1, "Payload length: {}".format(msg.payload_length))


def setpoint_ufh(msg):
    # Not 100% sure what this command is. First 2 digits seem to be ufh controller zone, followed by 4 digits which appear to be for the
    # zone's setpoint, then 0A2801.
    # Pattern is repeated with any other zones that the ufh controller may have.

    i = 0
    while (i < (msg.payload_length * 2)):
        zone_id, zone_name, topic = get_zone_details(msg.payload[0 + i:2 + i])
        # We add 900 to identify UFH zone numbers
        zone_setpoint = float(int(msg.payload[2 + i:6 + i], 16)) / 100
        # zone_name =""
        for d in gcfg.devices:
            if gcfg.devices[d].get('ufh_zoneId') and zone_id == gcfg.devices[d]['ufh_zoneId']:
                zone_id = gcfg.devices[d]["zoneId"]
                zone_name = gcfg.devices[d]["name"]
                break
        if zone_name == "":
            display_and_log("DEBUG", "UFH Setpoint Zone '{}' name not found".format(zone_id), msg.port)
            zone_name = "UFH Zone {}".format(zone_id)
        display_data_row(msg, "{:5.2f}°C".format(zone_setpoint), zone_id)
        # display_and_log(msg.command_name,'{0: <22}{:>5}  [Zone UFH {}]".format(zone_name, zone_setpoint, zone_id))
        mqtt_publish(zone_name, "setpoint", zone_setpoint)

        i += 12


def setpoint(msg):
    if msg.payload_length % 3 != 0:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be multiple of 3). Raw msg: {}".format(msg.payload_length,
                                                                                                     msg.rawmsg))
    else:
        command_name_suffix = ""
        if msg.payload_length > 3:
            command_name_suffix = "_CTL"
            msg.command_name = "{}_CTL".format(msg.command_name)
        i = 0
        payload_string_length = msg.payload_length * 2
        while (i < payload_string_length):
            zone_data = msg.payload[i:i + 6]
            zone_id = int(zone_data[0:2], 16) + 1  # Zone number
            zone_name = gcfg.zones[zone_id] if zone_id in gcfg.zones else "Zone {}".format(zone_id)

            #if not msg.source in gcfg.zones_list:
            #    gcfg.zones_list[msg.source] = {}
            #gcfg.zones_list[msg.source]["zone_id"] = zone_id
            #gcfg.zones_list[msg.source]["zone_name"] = zone_name

            update_zone_details(zone_data, msg, zones_info_inout=gcfg.zones_list, devices_inout=gcfg.devices)

            # display_and_log("DEBUG","Setpoint: zone not found for zone_id " + str(zone_id) + ", MSG: " + msg.rawmsg)
            zone_setpoint = convert_from_twos_comp(
                zone_data[2:6])  # float(int(zone_data[2:4],16) << 8 | int(zone_data [4:6],16))/100
            if (
                    zone_setpoint >= 300.0):  # Setpoint of 325 seems to be the number sent when TRV manually switched to OFF. Use >300 to avoid rounding errors etc
                zone_setpoint = 0
                flag = " *(Heating is OFF)"
            else:
                flag = ""

            display_data_row(msg, "{:5.2f}°C{}".format(zone_setpoint, flag), zone_id)
            mqtt_publish(zone_name, "setpoint" + command_name_suffix, "{:5.2f}".format(zone_setpoint))
            mqtt_publish(zone_name, "zone_id", "{}".format(zone_id))
            i += 6


def setpoint_override(msg):
    if msg.payload_length != 7 and msg.payload_length != 13:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be 7 or 13). Raw msg: {}".format(msg.payload_length,
                                                                                               msg.rawmsg))
    else:
        zone_id, zone_name, topic = get_zone_details(msg.payload[0:2])
        new_setpoint = convert_from_twos_comp(
            msg.payload[2:6])  # float(int(msg.payload[2:4],16) << 8 | int(msg.payload [4:6],16))/100

        # !!TODO!! Trap for 0x7FFF - this means setpoint not set
        if msg.payload_length == 13:  # We have an 'until' date
            dtm_hex = msg.payload[14:]
            dtm = get_dtm_from_packed_hex(dtm_hex)
            until = " - Until " + str(dtm)
            mqtt_publish(topic, "mode", "Temporary")
            mqtt_publish(topic, "mode_until",
                         "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z".format(dtm[0], dtm[1], dtm[2], dtm[3], dtm[4],
                                                                             dtm[5]))
        else:
            until = ""
            mqtt_publish(topic, "mode", "Scheduled")
            mqtt_publish(topic, "mode_until", "")
        display_data_row(msg, "{:5.2f}°C".format(new_setpoint), zone_id, until)
        mqtt_publish(topic, "setpointOverride", "{:5.2f}".format(new_setpoint))


def zone_temperature(msg):
    """
        zone_temperature info sent singly by individual devices and also by the controller for multiple zones in one message.
        Controller only seems to be sending zone_temperature data for single room zones.
        For multi-room zones, we have to rely  the sending device for its temperature value
    """

    if msg.payload_length == 1:
        display_data_row(msg, "Zone temperature requested")
    elif msg.payload_length % 3 != 0:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be 1 or mod 3). Raw msg: {}".format(msg.payload_length,
                                                                                                  msg.rawmsg))
        return

    i = 0
    payload_string_length = msg.payload_length * 2
    while (i < payload_string_length):
        zone_data = msg.payload[i:i + 6]

        # If msg from controller, then get the zone_id from the data block. Otherwise use the zone_id of the msg sender
        if msg.source_type == "01":
        #if msg.source_id == CONTROLLER_ID:
            zone_id = int(zone_data[:2], 16) + 1
        elif gcfg.devices.get(msg.source_id):
            zone_id = gcfg.devices[msg.source_id]["zoneId"]
        else:
            zone_id = 0
            display_and_log("DEBUG", "Device not found for source ID {}".format(msg.source_id))

        # temperature = float(int(msg.payload[i+2:i+6],16))/100
        temperature = convert_from_twos_comp(zone_data[2:6])
        if zone_id != 0:
            zoneDesc = " [Zone " + str(zone_id) + "]"
        else:
            zoneDesc = ""

        if msg.msg_type != "RQ":
            display_data_row(msg, "{:5.2f}°C".format(temperature), zone_id)
            mqtt_publish("{}/{}".format(gcfg.zones[zone_id] if zone_id in gcfg.zones else zone_id, msg.source_name),
                         "temperature", "{:5.2f}".format(temperature))
        else:
            display_data_row(msg, "msg_type: {}".format(msg.msg_type), zone_id)

        i += 6


def window_status(msg):
    if msg.payload_length < 3:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be less than 3). Raw msg: {}".format(msg.payload_length,
                                                                                                   msg.rawmsg))
    else:
        # Zone is first 2, then window status. 3rd pair seems to be always zero apparently
        zone_id, zone_name, _ = get_zone_details(msg.payload[0:2])
        statusId = int(msg.payload[2:4], 16)
        misc = int(msg.payload[4:6], 16)

        #if not msg.source in gcfg.zones_list:
        #    gcfg.zones_list[msg.source] = {}
        #gcfg.zones_list[msg.source]["zone_id"] = zone_id
        #gcfg.zones_list[msg.source]["zone_name"] = zone_name

        update_zone_details(msg.payload[0:2], msg, zones_info_inout=gcfg.zones_list, devices_inout=gcfg.devices)
        if statusId == 0:
            status = "CLOSED"
        elif statusId == 0xC8:
            status = "OPEN"
        else:
            status = "Unknown (" + str(statusId) + ")"

        if misc > 0:
            miscDesc = " (Misc: " + str(misc) + ")"
        else:
            miscDesc = ""
        display_data_row(msg, "{:>7}".format(status), zone_id)
        mqtt_publish("{}/{}".format(gcfg.zones[zone_id] if zone_id in gcfg.zones else zone_id, msg.source_name),
                     "window_status", status)


def other_command(msg):
    display_and_log(msg.command_name, msg.rawmsg)


def date_request(msg):
    display_data_row(msg, "Ping/Datetime Sync")


def relay_heat_demand(msg):
    # Heat demand sent by the controller for CH / DHW / Boiler  (F9/FA/FC)
    if msg.payload_length != 2:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be 2). Raw msg: {}".format(msg.payload_length,
                                                                                         msg.rawmsg))
    else:
        type_id = int(msg.payload[0:2], 16)
        demand = int(msg.payload[2:4], 16)
        if type_id < 12:
            type_id += 1
        zone_id, zone_name, topic = get_zone_details(msg.payload, msg.source_type)

        demand_percentage = float(demand) / 200 * 100
        display_data_row(msg, "{:>6.1f}% @ {}".format(demand_percentage, zone_name, "(type id: {})".format(type_id)))
        mqtt_publish(topic, "heat_demand", "{:>6.1f}".format(demand_percentage))


def zone_heat_demand(msg):
    if msg.payload_length % 2 != 0:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be mod 2). Raw msg: {}".format(msg.payload_length,
                                                                                             msg.rawmsg))
    else:
        topic = ""
        i = 0
        while (i < (msg.payload_length * 2)):
            # try:
            zone_id, zone_name, topic = get_zone_details(msg.payload[i:2 + i])
            demand = int(msg.payload[2 + i:4 + i], 16)

            #if not msg.source in gcfg.zones_list:
            #    gcfg.zones_list[msg.source] = {}
            #gcfg.zones_list[msg.source]["zone_id"] = zone_id
            #gcfg.zones_list[msg.source]["zone_name"] = zone_name
            update_zone_details(msg.payload[i:2 + i], msg, zones_info_inout=gcfg.zones_list, devices_inout=gcfg.devices)

            # We use zone combined with device name for topic, as demand can be from individual trv
            if zone_id <= 12:
                topic = "{}/{}".format(topic, msg.source_name)

            if msg.source_type == DEVICE_TYPE[
                'UFH'] and zone_id <= 8:  # UFH zone controller only supports 5 (+3 with optional card) zones
                # if destination device is the main touch controller, then then zone Id is that of the matched zone in the main touch controller itself
                # Otherwise, if the destination device is the ufh controller (i.e. message is to itself/broadcast etc), then the zone Id is the ufh controller zone id (i.e. 1 to 8)
                # zone_id must therefore be the ufh controller zones, and valued 0 to 7.
                ufh_zone_id = zone_id - 1  # 1 was added in the get_zone_details fn above as ufh subzones zero based
                if msg.destination_type == DEVICE_TYPE['CTL']:
                    device_type = "UFH {}".format(zone_name.split(' ', 1)[1] if " " in zone_name else zone_name)
                    topic = zone_name
                elif msg.is_broadcast():  # the zone_id refers to the UFH controller zone, and not the main evohome controller zone
                    # zone_id in this refers to the ufh zone id, and so zone_name etc need to be corrected
                    zone_name = "UFH Sub-Zone Id {}".format(ufh_zone_id)
                    for d in gcfg.devices:
                        if 'ufh_zoneId' in gcfg.devices[d] and gcfg.devices[d]['ufh_zoneId'] == ufh_zone_id:
                            # display_and_log("DEBUG","UFH Zone matched to " + devices[d]["name"])
                            zone_id = gcfg.devices[d]["zoneId"]  #
                            zone_name = gcfg.devices[d]["name"]
                            zone_name_parts = zone_name.split(' ', 1)
                            device_type = "UFH {}".format(zone_name_parts[1]) if len(
                                zone_name_parts) > 1 else "UFH {}".format(zone_name)
                            topic = "{}/{}".format(zone_name, msg.source_name)
                            break
                else:
                    display_and_log("ERROR", "UFH message received, but destination is neither main controller nor UFH controller. \
                  destination = {}, destination_type = {}. msg: {} ".format(msg.destination, msg.destination_type,
                                                                            msg.rawmsg))

            demand_percentage = float(demand) / 200 * 100
            display_data_row(msg, "{:6.1f}%".format(demand_percentage), zone_id)
            if len(topic) > 0:
                mqtt_publish(topic, "heat_demand", "{:6.1f}%".format(demand_percentage))
            else:
                display_and_log("DEBUG", "ERROR: Could not post to MQTT as topic undefined")
            i += 4


def dhw_settings(msg):
    #  <1:DevNo><2(uint16_t):SetPoint><1:Overrun?><2:Differential>
    if msg.payload_length != 6:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be 6). Raw msg: {}".format(msg.payload_length,
                                                                                         msg.rawmsg))

    device_number = int(msg.payload[0:2], 16)
    setpoint = float(int(msg.payload[2:6], 16)) / 100
    overrun = int(msg.payload[6:8], 16)
    differential = float(int(msg.payload[8:12], 16)) / 100
    reheat_trigger = setpoint - differential

    display_data_row(msg, "DHW Setpoint: {}°C; Re-heat triggered at {}°C. (Overrun state: {})".format(setpoint,
                                                                                                      reheat_trigger,
                                                                                                      overrun), -1,
                     "(Device: {})".format(device_number))


def actuator_check_req(msg):
    # this is used to synchronise time periods for each relay bound to a controller
    # i.e. all relays get this message and use it to determine when to start each cycle (demand is just a % of the cycle length)
    # https://www.domoticaforum.eu/viewtopic.php?f=7&t=5806&start=105#p73681
    if msg.payload_length != 2:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be 2). Raw msg: {}".format(msg.payload_length,
                                                                                         msg.rawmsg))
    else:
        device_number = int(msg.payload[0:2], 16)
        demand = int(msg.payload[2:4], 16)
        # if device_number == 0xfc: # 252 - apparently some sort of broadcast?
        #   device_type = ": Status Update Request"
        # else:
        #   device_type =""
        status = "Actuator time period sync request: {}".format(device_number)

        display_data_row(msg, status)


def actuator_state(msg):
    if msg.payload_length == 1 and msg.msg_type == "RQ":
        display_data_row(msg, "Request actuator state update")
    elif msg.payload_length % 3 != 0:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be mod 3). Raw msg: {}".format(msg.payload_length,
                                                                                             msg.rawmsg))
    else:
        device_number = int(msg.payload[0:2], 16)  # Apparently this is always 0 and so invalid
        demand = int(msg.payload[2:4], 16)  # (0 for off or 0xc8 i.e. 100% for on)

        if msg.source_type == DEVICE_TYPE["OTB"] and msg.payload_length == 6:  # OpenTherm
            rel_modulation = float(int(msg.payload[2:4], 16))
            flame = "ON" if int(msg.payload[6:8], 16) == 0x0A else "OFF"
            status = "{:5.0f}%  : Relative modulation (Flame: {})".format(rel_modulation, flame)
            mqtt_publish("relays/{}/actuator_state".format(msg.source_name), "relative_modulation", rel_modulation)
            mqtt_publish("relays/{}/actuator_state".format(msg.source_name), "flame", flame)
            display_data_row(msg, "{:>7}".format(status))
        else:  # Normal relays
            if demand == 0xc8:
                status = "ON"
            elif demand == 0:
                status = "OFF"
            else:
                status = "Unknown: " + str(demand)
            display_data_row(msg, "{:>7}".format(status))
            mqtt_publish("relays/{}".format(msg.source_name), "actuator_status", status)


def dhw_state(msg):
    if msg.payload_length == 1 and (msg.msg_type == "RQ" or msg.msg_type == "I"):
        display_and_log(msg.command_name, "Request sent: {}".format(msg.payload))
        return

    if msg.payload_length != 6 and msg.payload_length != 12:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be multiple 6 or 12). Raw msg: {}".format(
                            msg.payload_length, msg.rawmsg))
    else:
        zone_id, _, _ = get_zone_details(msg.payload[0:2])
        stateId = int(msg.payload[2:4], 16)  # 0 or 1 for DHW on/off, or 0xFF if not installed
        modeId = int(msg.payload[4:6], 16)  # 04 = timed??

        if stateId == 0xFF:
            state = "DHW not installed"
        elif stateId == 1:
            state = "On"
        elif stateId == 0:
            state = "Off"
        else:
            state = "Unknown state: {}".format(stateId)

        if modeId == 0:
            mode = "Auto"
        elif modeId == 4:
            mode = "Timed"
        else:
            mode = str(modeId)

        if msg.payload_length == 12:
            dtm_hex = msg.payload[12:]
            dtm = get_dtm_from_packed_hex(dtm_hex)
            until = " - Until {}".format(dtm)
        else:
            until = ""

        if stateId == 0xFF:
            display_and_log(msg.command_name, "{}: DHW not installed".format(msg.source))
        else:
            display_data_row(msg, "State: {}, mode: {}".format(state, mode), -1, until)
            mqtt_publish("DHW", "state", stateId)
            mqtt_publish("DHW", "mode", mode)
            if until > "":
                mqtt_publish("DHW", "mode_until",
                             "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z".format(dtm[0], dtm[1], dtm[2], dtm[3], dtm[4],
                                                                                 dtm[5]))
            else:
                mqtt_publish("DHW", "mode_until", "")


def dhw_temperature(msg):
    if msg.payload_length == 1:
        # This is most likely an outbound request
        return

    if msg.payload_length != 3:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be 3). Raw msg: {}".format(msg.payload_length,
                                                                                         msg.rawmsg))
        return

    temperature = float(int(msg.payload[2:6], 16)) / 100
    display_data_row(msg, "{:5.2f}°C".format(temperature))
    mqtt_publish("DHW", "temperature", "{:5.2f}".format(temperature))


def zone_info(msg):
    if msg.payload_length % 6 != 0:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be mod 6). Raw msg: {}".format(msg.payload_length,
                                                                                             msg.rawmsg))
        return

    i = 0
    payload_string_length = msg.payload_length * 2
    while (i < payload_string_length):
        zone_data = msg.payload[i:i + 12]
        zone_id, zone_name, topic = get_zone_details(zone_data)

        zone_flags = int(zone_data[2:4], 16)
        min_temperature = float(int(zone_data[4:8], 16) / 100)
        max_temperature = float(int(zone_data[8:12], 16) / 100)

        display_data_row(msg, "Temp. range: {}°C to {}°C".format(min_temperature, max_temperature), zone_id,
                         "(Flags: {})".format(zone_flags))
        # mqtt_publish(zone_name, "setpoint" + command_name_suffix,zone_setpoint)
        i += 12


# def zone_schedule(msg):
# """ Schedule data for zone. Data is received in chunks, which need to be combined before decompressing """
# if msg.payload_length == 7: # Outbound command
#     return
# elif  msg.payload_length < 63:
#     display_and_log(msg.command_name, "Invalid payload length of {} (should be greater than 62 for inbound). Raw msg: {}".format(msg.payload_length, msg.rawmsg))
#     return

# display_and_log(msg.command_name, msg.payload)
# pass


def language(msg):
    """ Language localisation setting (iso-639 format)
        https://github.com/Evsdd/The-Evohome-Protocol/wiki/0100:-Localisation-(language)
        Supported lanauges: English - en, Deutsch - nl, Italiano - it, Francais - fr, Nederlands, Espanol - es, 
                            Polski - pl, Cesky - cs, Magyar - hu, Romana - ro, Slovencina - sk & Dansk - da.
    """
    if msg.payload_length != 5:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be 5). Raw msg: {}".format(msg.payload_length,
                                                                                         msg.rawmsg))
        return
    try:
        assert msg.payload[:2] == "00", "Invalid payload '{}' (must start with '00')".format(msg.payload)
        assert msg.payload[8:] == "FF", "Invalid payload '{}' (must end with 'FF')".format(msg.payload)
        iso_code_ascii = msg.payload[2:6] if msg.payload[4:6] != "FF" else msg.payload[2:4]
        b = bytearray([int(iso_code_ascii[n:n + 2], 16) for n in range(0, len(iso_code_ascii), 2)])
        iso_code = b.decode("utf-8").replace("\x00", "").replace("\xff", "")

        display_data_row(msg, "{} ({})".format(iso_code, iso_code_ascii))
    except Exception as e:
        fio = io.StringIO()
        sys.print_exception(e, fio)
        fio.seek(0)
        errmsg="'{}' on line {} [Command {}, payload: '{}', port: {}]".format(str(e),fio.read(),
                                                                               msg.command_name,
                                                                               msg.payload, msg.port)
        display_and_log("ERROR",errmsg)
        error_log(display_message=errmsg)
        # print(traceback.format_exc())
        # sys.print_exception(e)


def fault_log(msg):
    """ Fault log entry from Controller for given log entry number (zero based) """

    if msg.payload_length == 3:  # When requesting information, 3rd byte of payload is the fault message number (zero based) as shown on controller screen
        display_and_log(msg.command_name, "Sytem fault log entry '{}' requested".format(msg.payload))
        return
    if msg.payload_length != 22:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be mod 22). Raw msg: {}".format(msg.payload_length,
                                                                                              msg.rawmsg))
        return

    dtm_hex = int(msg.payload[20:22], 16) << 32 | int(msg.payload[22:24], 16) << 24 | int(msg.payload[24:26],
                                                                                          16) << 16 | int(
        msg.payload[26:28], 16) << 8
    year = ((dtm_hex & int("1111111", 2) << 24) >> 24) + 2000
    month = (dtm_hex & int("1111", 2) << 36) >> 36
    day = (dtm_hex & int("11111", 2) << 31) >> 31

    hour = (dtm_hex & int("11111", 2) << 19) >> 19
    minute = (dtm_hex & int("111111", 2) << 13) >> 13
    second = (dtm_hex & int("111111", 2) << 7) >> 7

    # dtm = datetime.datetime(year, month, day, hour, minute, second)
    dtm = (year, month, day, hour, minute, second)

    dev_id_int = int(msg.payload[38:40], 16) << 16 | int(msg.payload[40:42], 16) << 8 | int(msg.payload[42:44], 16)
    dev_id = "{:02}:{:06}".format((dev_id_int >> 18) & 0X3F, dev_id_int & 0x3FFFF)
    if dev_id in gcfg.devices:
        device_name = gcfg.devices[dev_id]["name"]
    else:
        device_name = dev_id

    fault_type_id = int(msg.payload[2:4], 16)
    fault_code = int(msg.payload[8:10], 16)

    log_entry_number = int(msg.payload[4:6], 16)
    dev_num = int(msg.payload[10:12], 16)
    device_type_id = int(msg.payload[12:14], 16)

    if fault_type_id == 0x00 or fault_type_id == 0xc0:
        fault_type = "Fault"
    elif fault_type_id == 0x40:
        fault_type = "Restore"
    else:
        fault_type = "Unknown info type '{}'".format(fault_type_id)

    if fault_code == 0x04:
        fault = "Battery Low"
    elif fault_code == 0x06:
        fault = "Comms Fault"
    elif fault_code == 0x0a:
        fault = "Sensor Error"
    else:
        fault = "Unknown fault code '{}'".format(fault_code)

    if device_type_id == 0x04:
        device_type = "TRV"
    elif device_type_id == 0x01:
        device_type = "SENSOR"
    elif device_type_id == 0x05:
        device_type = "DHW"
    elif device_type_id == 0x00:
        device_type = "CONTROLLER"
    else:
        device_type = "Unknown device type '{}'".format(device_type_id)

    # display_data_row(msg, "{}: {:%Y-%m-%d %H:%M:%S} [{} {}] {}: '{}' (Device ID: {})".format(
    #  log_entry_number, dtm_str, device_type, device_name, fault_type, fault, dev_id))

    # msg = {"device_type": device_type, "device_id": dev_id, "device_name" : device_name, "device_num" : dev_num,
    #  "fault_type": fault_type, "fault": fault, "event_ts": "{:%Y-%m-%dT%H:%M:%S}".format(dtm), "index": log_entry_number}

    dtm_str = ":{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(dtm[0], dtm[1], dtm[2], dtm[3], dtm[4], dtm[5])
    display_data_row(msg, "{}: {} [{} {}] {}: '{}' (Device ID: {})".format(
        log_entry_number, dtm_str, device_type, device_name, fault_type, fault, dev_id))
    dtm_str_i = ":{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}".format(dtm[0], dtm[1], dtm[2], dtm[3], dtm[4], dtm[5])
    msg = {"device_type": device_type, "device_id": dev_id, "device_name": device_name, "device_num": dev_num,
           "fault_type": fault_type, "fault": fault, "event_ts": dtm_str_i, "index": log_entry_number}
    mqtt_publish("_faults", str(log_entry_number), json.dumps(msg))


def battery_info(msg):
    if msg.payload_length != 3:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be 3). Raw msg: {}".format(msg.payload_length,
                                                                                         msg.rawmsg))
    else:
        device_id = int(msg.payload[0:2], 16)
        battery = int(msg.payload[2:4], 16)
        lowBattery = int(msg.payload[4:5], 16)
        zone_id = gcfg.devices[msg.source]["zoneId"]

        if battery == 0xFF:
            battery = 100  # recode full battery (0xFF) to 100 for consistency across device types
        else:
            battery = battery / 2  # recode battery level values to 0-100 from original 0-200 values

        if (lowBattery != 0):  # TODO... Need to check this to understand how it is used.
            suffix = "- LOW BATTERY WARNING (device ID {})".format(device_id)
        else:
            suffix = "(device ID {})".format(device_id)

        display_data_row(msg, "{:.1f}%".format(battery), zone_id, suffix)
        topic = "dhw" if zone_id == 250 else gcfg.zones[zone_id]
        mqtt_publish("{}/{}".format(topic, msg.source_name), "battery", battery)


def opentherm_msg(msg):
    if msg.payload_length != 5:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be 5). Raw msg: {}".format(msg.payload_length,
                                                                                         msg.rawmsg))
        return

    # [0:2] are unused - always 00
    msg_type_id = int(msg.payload[2:4], 16) & 0x70  # OT message type
    msg_type = OPENTHERM_MSG_TYPES[msg_type_id] if OPENTHERM_MSG_TYPES[msg_type_id] else msg_type_id
    data_id = int(msg.payload[4:6], 16)  # OT command ID
    data_value = msg.payload[6:10]  # Command response value
    zone_id = gcfg.devices[msg.source]["zoneId"]

    if not int(msg.payload[2:4], 16) // 0x80 == parity(int(msg.payload[2:], 16) & 0x7FFFFFFF):
        display_data_row(msg,
                         "Parity error. Msg type_id: {} ({}) id: {}, value: {}".format(msg_type_id, msg_type, data_id,
                                                                                       data_value))
    elif not int(msg.payload[2:4], 16) & 0x0F == 0:  # valid for v2.2 of the protocol
        display_data_row(msg, "Protocol error. Msg type_id: {} ({}), id: {}, value: {}".format(msg_type_id, msg_type,
                                                                                               data_id, data_value))
    else:
        # The OT command response is in byte 4 and 5
        value_int = convert_from_twos_comp(msg.payload[6:10],
                                           1)  # int(msg.payload[6:8],16) << 8 | int(msg.payload[8:10],16)
        value_float = float(value_int) / 256.0

        #  OT commands are as per the OT Specification
        status = ""
        data = {}  # Use dict in case of multiple data/value pairs for single OT ID (e.g. ID 5)
        if data_id == 5:
            # 05 (ID.05) = Fault Code
            data["app_specific_flags"] = msg.payload[6:8]
            data["oem_fault_code"] = msg.payload[8:10]
            status = "{:2}/{:2}   : Application Specific Flags/OEM Fault Code".format(data["app_specific_flags"], data[
                "oem_fault_code"]) if msg_type_id > 0 else "Request App specific flags/OEM fault code"
        elif data_id == 17:
            # 11 (ID.17) = Relative modulation level
            data["relative_modulation"] = value_float
            status = "{:5.1f}%  : Relative modulation".format(
                value_float) if msg_type_id > 0 else "Request Relative Modulation value"

        elif data_id == 18:
            # 12 (ID.18) = CH water pressure
            status = "{:5.1f}   : CH Water Pressure (bar)".format(
                value_float) if msg_type_id > 0 else "Request CH Water Pressure"
            data["ch_water_pressure"] = value_float
        elif data_id == 19:
            # 13 (ID.19) = DHW flow rate
            status = "{:5.1f}   : DHW flow rate (l/min)".format(
                value_float) if msg_type_id > 0 else "Request DHW Flow Rate"
            data["dhw_flow_rate"] = value_float
        elif data_id == 25:
            # 19 (ID.25) = Boiler Water Temperature
            status = "{:5.1f}°C : Flow Water Temperature".format(
                value_float) if msg_type_id > 0 else "Request Boiler Flow Water Temperature"
            data["boiler_temperature"] = value_float
        elif data_id == 26:
            # 1A (ID.26) = DHW Temperature
            status = "{:5.1f}°C : DHW Temperature".format(value_float) if msg_type_id > 0 else "Request DHW Temperature"
            data["dhw_temperature"] = value_float
        elif data_id == 28:
            # 1C (ID.28) = Return Water Temperature
            status = "{:5.1f}°C : Return Water Temperature".format(
                value_float) if msg_type_id > 0 else "Request Boiler Return Water Temperature"
            data["return_water_temperature"] = value_float
        elif data_id == 115:
            # 73 (ID.115) = OEM diagnostic code
            status = "{:5.1f}   : OEM diagnostic code".format(
                value_int) if msg_type_id > 0 else "Request OEM Diagnostic code"
            data["oem_diagnostic_code"] = value_int

        if status:
            display_data_row(msg, status)
            if msg_type_id > 0:
                for key, value in data.items():
                    mqtt_publish("relays/{}".format(msg.source_name), key, value)
        else:
            display_data_row(msg,
                             "Message Data ID not recognised. Type: {} ({}), Data ID: {}, value: {}".format(msg_type,
                                                                                                            msg_type_id,
                                                                                                            data_id,
                                                                                                            data_value))


def opentherm_ticker(msg):
    pass


def boiler_setpoint(msg):
    if msg.payload_length == 1 and msg.msg_type == "RQ":
        display_data_row(msg, "Setpoint update request")
        return
    elif msg.payload_length != 3:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be 3). Raw msg: {}".format(msg.payload_length,
                                                                                         msg.rawmsg))
        return

    setpoint = float(int(msg.payload[2:6], 16)) / 100
    display_data_row(msg, "Boiler setpoint: {}".format(setpoint))
    mqtt_publish("Relays/{}".format(msg.source_name), "boiler_setpoint", "{:5.2f}".format(setpoint))


def controller_mode(msg):
    if msg.payload_length != 8:
        display_and_log(msg.command_name,
                        "Invalid payload length of {} (should be 8). Raw msg: {}".format(msg.payload_length,
                                                                                         msg.rawmsg))
    else:
        modeId = int(msg.payload[0:2], 16)  # controller mode
        try:
            mode = CONTROLLER_MODES[modeId]
        except:
            fio=io.StringIO()
            sys.print_exception(e,fio)
            fio.seek(0)
            mode = "Unknown (" + str(modeId) + ")"+fio.read()
        durationCode = int(msg.payload[14:16], 16)  # 0 = Permanent, 1 = temporary

        if durationCode == 1:
            dtm_hex = msg.payload[2:14]
            dtm = get_dtm_from_packed_hex(dtm_hex)
            until = " [Until {}]".format(dtm)
        else:
            if modeId != 0:
                until = " - PERMANENT"
            else:
                until = ""
        display_data_row(msg, "{} mode".format(mode), -1, until)
        mqtt_publish(msg.source_name, "mode", mode)


def heartbeat(msg):
    display_and_log(msg.command_name, msg.rawmsg, msg.port)


def external_sensor(msg):
    display_and_log(msg.command_name, msg.rawmsg, msg.port)


def unknown_command(msg):
    display_and_log(msg.command_name, msg.rawmsg, msg.port)


def get_reset_serialports_command():
    return Command(SYS_CONFIG_COMMAND, RESET_COM_PORTS)


# --- evohome send command functions
def process_send_command(command):
    ''' Process system configuration command or send command to evohome gateway '''
    if not command.command_code and not command.command_name:
        display_and_log("ERROR",
                        "Cannot process command without valid command_code ({}) or command_name ({}) [args: '{}']".format(
                            command.command_code, command.command_name, args))
        return

    # Check and do system config commands first
    if command.command_code == SYS_CONFIG_COMMAND:
        if command.command_name == RESET_COM_PORTS:
            reset_com_ports()
        else:
            display_and_log(SYSTEM_MSG_TAG,
                            "System configuration command '{}' not recognised".format(command.command_name))
        return  # Either way, we return. Rest of the fn is processing actual evohome commands

    # Command must be an evohome one. Process and send.
    if not command.command_code:  # command_name takes priority over command_code
        command.command_code = COMMAND_CODES[command.command_name] if command.command_name in COMMAND_CODES else "0000"
    else:
        if command.command_code in COMMANDS:
            command.command_name = COMMANDS[command.command_code].__name__.upper()
        else:
            display_and_log("DEBUG", "Command name not found for code '{}'".format(command.command_code))

    if command.command_code == "0000":
        display_and_log("ERROR", "Unrecognised command.command_name '{}'".format(command.command_name))
        return

    send_string = ""
    # print (command.command_name)
    if "payload" not in command.args:
        if (command.command_name and command.command_name == "dhw_state") or (
                command.command_code and command.command_code == "1F41"):
            # 1F41: Change dhw state
            state_id = command.args["state_id"]
            until = command.args["until"] if "until" in command.args else None
            mode_id = command.args["mode_id"] if "mode_id" in command.args else -1
            command.payload = get_dhw_state_payload(state_id, until, mode_id)
            if command.send_mode is None:
                command.send_mode = "W"
            if until:
                command.arg_desc = "[{} until {}]".format("ON" if state_id == 1 else "OFF", until)
            else:
                command.arg_desc = "[{}]".format("ON" if state_id == 1 else "OFF")

        elif (command.command_name and command.command_name in "date_request ping") or (
                command.command_code and command.command_code == "313F"):
            # 0x313F: Send a datetime update request, i.e. like a ping
            command.payload = "00"
            if command.send_mode is None:
                command.send_mode = "RQ"

        elif command.command_name and command.command_name in "fault_log":
            # Default to getting last log entry
            command.payload = "000000"
            command.send_mode = "RQ"

        elif (command.command_name and command.command_name == "controller_mode") or (
                command.command_code and command.command_code == "2E04"):
            # 0x2E04: Set controller mode
            mode = command.args["mode"]
            until = command.args["until"] if "until" in command.args else None
            command.payload = get_controller_mode_payload(mode, until)

            # Send mode needs to be 'W' to set the controller to the new controller mode
            if command.send_mode is None:
                command.send_mode = "W"
            if until:
                command.arg_desc = "[{} until {}]".format(mode, until)
            else:
                command.arg_desc = mode

        elif (command.command_name and command.command_name == "setpoint_override") or (
                command.command_code and command.command_code == "2349"):
            # 0x2349: Setpoint override
            zone_id = command.args["zone_id"]
            setpoint = command.args["setpoint"]
            until = command.args["until"] if "until" in command.args else None
            mode = command.args["mode"] if "mode" in command.args else None
            command.payload = get_setpoint_override_payload(zone_id, setpoint, until, mode)
            if command.send_mode is None:
                command.send_mode = "W"
            if until:
                command.arg_desc = "['{}': {} degC until {}]".format(
                    gcfg.zones[zone_id] if zone_id in gcfg.zones else zone_id, setpoint, until)
            else:
                command.arg_desc = command.arg_desc = "['{}': {} deg C]".format(
                    gcfg.zones[zone_id] if zone_id in gcfg.zones else zone_id, setpoint)
        else:
            if not command.send_mode:  # default to RQ
                command.send_mode = "RQ"
    else:
        command.payload = command.args["payload"]
        if command.send_mode is None:
            command.send_mode = "I"

    command.dev1 = command.args["dev1"] if "dev1" in command.args else THIS_GATEWAY_ID
    command.dev2 = command.args["dev2"] if "dev2" in command.args else CONTROLLER_ID
    command.dev3 = command.args["dev3"] if "dev3" in command.args else EMPTY_DEVICE_ID

    command.destination = command.dev2

    # if command.payload_length() > -1 and command.payload:
    sent_command = send_command_to_evohome(command)
    return sent_command

    # else:
    #     display_and_log("ERROR","Invalid command.payload = '{}' or command.payload length = {}".format(command.payload, command.payload_length()))
    #     return None


def get_controller_mode_payload(mode_id, until_string=None):
    if until_string == None:
        duration_code = 0x0
        until = "FFFFFFFFFFFF"
    else:
        duration_code = 0x1
        until = dtm_string_to_payload(until_string)

    payload = "{:02X}{}{:02X}".format(mode_id, until, duration_code)
    return payload


def get_dhw_state_payload(state_id, until_string=None, mode_id=-1):
    # state_id is 0 or 1 for DHW on/off
    if until_string == None:
        until = ""
        if mode_id == -1:
            mode_id = 0  # Revert to auto
    else:
        until = dtm_string_to_payload(until_string)
        mode_id = 4  # if we have an 'until', mode must be temporary
    zone_id = 0
    payload = "{:02X}{:02x}{:02X}FFFFFF{}".format(zone_id, state_id, mode_id, until)
    return payload


def get_setpoint_override_payload(zone_id, setpoint, until_string="", setpoint_is_permanenent=True):
    """
        modes:  [Auto, Temporary, Permanent, -1, Scheduled] (zero based)
        If setpoint_is_permament is False, the setpoint will revert at the next scheduled setpoint change
    """

    if until_string:
        until = dtm_string_to_payload(until_string)
        mode = 4
    elif setpoint > 0:
        mode = 2 if setpoint_is_permanenent else 1
        until = ""
    else:
        # If setpoint is 0, we revert back to auto
        mode = 0
        until = ""

    payload = "{:02X}{:04X}{:02X}FFFFFF{}".format(zone_id - 1, int(setpoint * 100), mode, until)

    return payload


def dtm_string_to_payload(dtm_string):
    # dtm = datetime.datetime.strptime(dtm_string, "%Y-%m-%dT%H:%M:%SZ")
    year = int(t[0:4])
    month = int(t[5:7])
    day = int(t[8:10])
    hour = int(t[11:13])
    minute = int(t[14:16])
    sec = int(t[17:19])
    payload = "{:02X}{:02X}{:02X}{:02X}{:04X}".format(minute, hour, day, month, year)
    return payload


def send_command_to_evohome(command):
    if not command and notcommand.command_code:
        display_and_log("ERROR", "Send to evohome failed as invalid 'command' argument: {}".format(command))
        return

    if not command.payload:
        command.payload = ""

    # Build outbound string for radio message
    command.send_string = "{} --- {} {} {} {:<4} {:03d} {}".format(command.send_mode, command.dev1,
                                                                   command.dev2, command.dev3, command.command_code,
                                                                   command.payload_length(), command.payload)
    log_row = "{}: Sending '{}'".format(command.command_name.upper() if command.command_name is not None else "-None-",
                                        command.send_string)
    log('{: <18} {}'.format("COMMAND_OUT", log_row), command.serial_port.tag)
    # display_and_log("COMMAND_OUT","{}: Sending '{}'".format(command.command_name.upper() if command.command_name is not None else "-None-", command.send_string))

    # Convert outbound string to bytearray and write via serial port
    byte_command = bytearray('{}\r\n'.format(command.send_string), "utf-8")
    response = serial_port.write(byte_command)

    display_and_log("COMMAND_OUT", "{} {} Command SENT".format(
        command.command_name.upper() if command.command_name is not None else command.command_code,
        command.arg_desc if command.arg_desc != "[]" else ":"), command.serial_port.tag)

    if gcfg.mqtt_client and gcfg.mqtt_client.is_connected:
        # timestamp = datetime.datetime.utcnow().strftime("%Y-%m-%dT%XZ") #2022-03-11T21:30:00Z
        t = time.gmtime()
        timestamp = "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z".format(t[0], t[1], t[2], t[3], t[4], t[5])
        gcfg.mqtt_client.publish("{}/failed".format(SENT_COMMAND_TOPIC), False, 0,
                                 True)  # Reset this before the others, to avoid incorrect interpretation of status by 3rd party apps
        # mqtt_client.publish("{}/failed_ts".format(SENT_COMMAND_TOPIC), "", 0, True)
        gcfg.mqtt_client.publish("{}/retries".format(SENT_COMMAND_TOPIC), command.retries, 0, True)
        mqtt_client.publish("{}/retry_ts".format(SENT_COMMAND_TOPIC), "", 0, True)
        gcfg.mqtt_client.publish("{}/ack".format(SENT_COMMAND_TOPIC), False, 0, True)
        # mqtt_client.publish("{}/ack_ts".format(SENT_COMMAND_TOPIC),"", 0, True)

        gcfg.mqtt_client.publish("{}/command".format(SENT_COMMAND_TOPIC),
                                 "{} {}".format(command.command_name, command.args), 0, True)
        gcfg.mqtt_client.publish("{}/evo_msg".format(SENT_COMMAND_TOPIC), command.send_string, 0, True)

        gcfg.mqtt_client.publish("{}/org_instruction".format(SENT_COMMAND_TOPIC), command.command_instruction, 0, True)
        gcfg.mqtt_client.publish(MQTT_SUB_TOPIC, "", 0, True)
        if command.retries == 0:
            gcfg.mqtt_client.publish("{}/initial_sent_ts".format(SENT_COMMAND_TOPIC), timestamp, 0, True)
        else:
            gcfg.mqtt_client.publish("{}/last_retry_ts".format(SENT_COMMAND_TOPIC), timestamp, 0, True)
    else:
        display_and_log(SYSTEM_MSG_TAG, "[WARN] Client not connected to MQTT broker. No command status messages posted")

    if command.retries == 0:
        # command.send_dtm = datetime.datetime.now()
        command.send_dtm = time.time()
    # command.retry_dtm = datetime.datetime.now()
    command.retry_dtm = time.time()

    command.retries += 1

    return command


def check_previous_command_sent(previous_command):
    ''' Resend previous command if ack not received in reasonable time '''
    if not previous_command or previous_command.send_acknowledged:
        return

    # seconds_since_sent = (datetime.datetime.now() - previous_command.retry_dtm).total_seconds()
    seconds_since_sent = time.time() - previous_command.retry_dtm
    if seconds_since_sent > COMMAND_RESEND_TIMEOUT_SECS:
        if previous_command.retries <= COMMAND_RESEND_ATTEMPTS and not previous_command.send_failed:
            if previous_command.retries == COMMAND_RESEND_ATTEMPTS and AUTO_RESET_PORTS_ON_FAILURE:
                reset_com_ports()  # Reset serial ports before last attempt

            display_and_log("COMMAND_OUT", "{} {} Command NOT acknowledged. Resending attempt {} of {}...".format(
                previous_command.command_name.upper() if previous_command.command_name else previous_command.command_code,
                previous_command.arg_desc if previous_command.arg_desc != "[]" else ":", previous_command.retries,
                COMMAND_RESEND_ATTEMPTS))
            previous_command = send_command_to_evohome(previous_command)
        elif not previous_command.send_failed:
            previous_command.send_failed = True
            mqtt_publish("", "command_sent_failed", True, "{}/failed".format(SENT_COMMAND_TOPIC))
            display_and_log("COMMAND",
                            "ERROR: Possible failure in sending command '{}'. No ack received from controller".format(
                                previous_command.command_name))

            # if AUTO_RESET_PORTS_ON_FAILURE:
            #   # command_code, command_name, args, send_mode = get_reset_serialports_command()
            #   gcfg.send_queue.append(get_reset_serialports_command())


# --- evohome Commands Dict
COMMANDS = {
    '0002': external_sensor,
    '0004': zone_name,
    '0006': schedule_sync,
    '0008': relay_heat_demand,
    '000A': zone_info,
    '0100': language,
    # '0404': zone_schedule,
    '0418': fault_log,
    '1060': battery_info,
    '10A0': dhw_settings,
    '10E0': heartbeat,
    '1260': dhw_temperature,
    '12B0': window_status,
    '1F09': sync,
    '1F41': dhw_state,
    '1FC9': bind,
    '1FD4': opentherm_ticker,
    '22C9': setpoint_ufh,
    '22D9': boiler_setpoint,
    '2309': setpoint,
    '2349': setpoint_override,
    '2E04': controller_mode,
    '30C9': zone_temperature,
    '313F': date_request,
    '3150': zone_heat_demand,
    '3220': opentherm_msg,
    '3B00': actuator_check_req,
    '3EF0': actuator_state
}
# 10A0: DHW settings sent between controller and DHW sensor can also be requested by the gateway

COMMAND_CODES = {
    "actuator_check_req": "3B00",
    "actuator_state": "3EF0",
    "battery_info": "1060",
    "bind": "1FC9",
    "boiler_setpoint": "22D9",
    "controller_mode": "2E04",
    "date_request": "313F",
    "fault_log": "0418",
    "dhw_state": "1F41",
    "dhw_temperature": "1260",
    "external_sensor": "0002",
    "heartbeat": "10E0",
    "language": "0100",
    "opentherm_msg": "3220",
    "opentherm_ticker": "1fd4",
    "other_command": "0100",
    "ping": "313F",
    "relay_heat_demand": "0008",
    "setpoint": "2309",
    "setpoint_override": "2349",
    "setpoint_ufh": "22C9",
    "sync": "1F09",
    "window_status": "12B0",
    "zone_heat_demand": "3150",
    "zone_info": "000A",
    "zone_name": "0004",
    # "zone_schedule" : "0404",
    "zone_temperature": "30C9"
}

THIS_GATEWAY_TYPE = DEVICE_TYPE[THIS_GATEWAY_TYPE_ID]

SENT_COMMAND_SUBTOPIC = "sent_command"
SENT_COMMAND_TOPIC_BASE = to_snake("{}/{}_{}".format(MQTT_PUB_TOPIC, THIS_GATEWAY_TYPE, THIS_GATEWAY_NAME))
SENT_COMMAND_TOPIC = "{}/{}".format(SENT_COMMAND_TOPIC_BASE, SENT_COMMAND_SUBTOPIC)
