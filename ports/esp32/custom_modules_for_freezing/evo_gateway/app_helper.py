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

import sys
import time
from evo_gateway.mqtt import mqtt_publish


def update_zone_details(payload, msg,zones_info_inout={}, devices_inout={}):
    try:
        zone_id = int(payload[0:2], 16)
        assert (zone_id < 12),"ERROR in update_zone_details zone_id >12"
        zone_id += 1
    except ValueError as e:
        sys.print_exception(e)
    if msg.source_type != "01":
        if not msg.source in zones_info_inout:
            zones_info_inout[msg.source] = {}
        zones_info_inout[msg.source]["zone_id"] = zone_id
        zones_info_inout[msg.source]["zone_name"] = "Zone {}".format(zones_info_inout[msg.source]["zone_id"])
        if msg.source in devices_inout:
            devices_inout[msg.source]["zoneId"] = zone_id


def error_log( display_message="",mqtt_post=True):
    t = time.gmtime()
    row = "{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z | {}".format(t[0], t[1], t[2], t[3], t[4], t[5],
                                                                                display_message)
    if mqtt_post:
        mqtt_publish("", "", row,topic="ERROR",auto_ts=False)

        