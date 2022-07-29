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

import time

import json
import re
from collections import namedtuple, deque

import os.path

import evo_gateway.globalcfg as gcfg  # for eventfile and logfile

from evo_gateway.config import EVENTS_FILE
from evo_gateway.config import LOG_FILE
from evo_gateway.config import MAX_LOG_HISTORY
from evo_gateway.config import MIN_ROW_LENGTH
from evo_gateway.config import COM_PORTS
from evo_gateway.config import SYSTEM_MSG_TAG
from evo_gateway.config import MAX_FILE_SIZE



# --- General Functions
def sig_handler(signum, frame):  # Trap Ctl C
    t = time.gmtime()
    tstr = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(t[0], t[1], t[2], t[3], t[4], t[5])
    print("{} Tidying up and exiting...".format(tstr))
    # display_and_log("Tidying up and exiting...")
    gcfg.logfile.close()
    for port_id, port in serial_ports.items():
        if port["connection"].is_open:
            print("Closing port '{}'".format(port["connection"].port))
            port["connection"].close()


def rotate_files(base_file_name):
    if os.path.isfile("{}.{}".format(base_file_name, MAX_LOG_HISTORY)):
        uos.remove(base_file_name + "." + str(MAX_LOG_HISTORY))

    i = MAX_LOG_HISTORY - 1
    while i >= 0:
        if i > 1:
            org_file_ext = "." + str(i)
        else:
            org_file_ext = ""
        if os.path.isfile(base_file_name + org_file_ext):
            uos.rename(base_file_name + org_file_ext, base_file_name + "." + str(i + 1))
        i -= 1


def to_snake(name):
    # simplying the implementation to just replacing spaces with underscores and making all lower case
    name = name.strip().replace("'", "").replace(" ", "_")
    # s1 = _first_cap_re.sub(r'\1_\2', name)
    # s2 = _all_cap_re.sub(r'\1_\2', s1).lower()
    # return s2.replace("__","_")
    return name.replace("__", "_").lower()


def to_camel_case(s):
    # simpliflying implementation to do nothing
    # return re.sub(r'(?!^) ([a-zA-Z])', lambda m: m.group(1).upper(), s)
    return s


def display_and_log(source="-", display_message="", port_tag=None, rssi=None):
    try:
        if os.path.getsize(EVENTS_FILE) > MAX_FILE_SIZE:
            gcfg.eventfile.close()
            rotate_files(EVENTS_FILE)
            gcfg.eventfile = open(EVENTS_FILE, "a")

        port_rssi = "{}/{:3s}".format(port_tag, rssi if rssi else " - ") if port_tag else ""
        # row = "{} |{:<5}| {:<20}| {}".format(datetime.datetime.now().strftime("%Y-%m-%d %X"), port_rssi, source, display_message)
        t = time.gmtime()
        row = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d} |{:<5}| {:<20}| {}".format(t[0], t[1], t[2], t[3], t[4], t[5],
                                                                                    port_rssi, source, display_message)

        row = "{:<{min_row_width}}".format(row, min_row_width=MIN_ROW_LENGTH)
        print(row)
        # print   (datetime.datetime.now().strftime("%Y-%m-%d %X") + ": " + "{:<20}".format(str(source)) + ": " + str(display_message))
        gcfg.eventfile.write(row + "\n")
        gcfg.eventfile.flush()
    except Exception as e:
        sys.print_exception(e)


def log(logentry, port_tag="-"):
    if os.path.getsize(LOG_FILE) > MAX_FILE_SIZE:
        gcfg.logfile.close()
        rotate_files(LOG_FILE)
        gcfg.logfile = open(LOG_FILE, "a")
    t = time.gmtime()
    tstr = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(t[0], t[1], t[2], t[3], t[4], t[5])

    # gcfg.logfile.write("{} |{}| {}\n".format(datetime.datetime.now().strftime("%Y-%m-%d %X"), port_tag, logentry.rstrip()))
    gcfg.logfile.write("{} |{}| {}\n".format(tstr, port_tag, logentry.rstrip()))

    gcfg.logfile.flush()


class FakeSerial():
    def __init__(self):
        pass

    def any(self):
        return 0


# Init com ports
def init_com_ports():
    serial_ports = {}
    if len(COM_PORTS) > 0:
        count = 1
        for port, params in COM_PORTS.items():
            limit = params["retry_limit"] if "retry_limit" in params else 3
            serial_port = None
            while (limit > 0) and serial_port is None:
                try:
                    if sys.platform == 'esp32':
                        assert ('uname' in dir(uos) and uos.uname()[0] == 'esp32'), "Not ESP32, no serial port support"
                        from machine import UART
                        baudrate = params["baud"] if "baud" in params else 115200
                        # set timeout to make .read() and .readline() block
                        serial_port = UART(int(port), baudrate=baudrate, tx=params["tx_pin"], rx=params["rx_pin"],
                                           timeout=1000)
                        break
                    else:
                        serial_port = FakeSerial()

                except Exception as e:
                    fio = io.StringIO()
                    sys.print_exception(e, fio)
                    fio.seek(0)
                    if limit > 1:
                        errmsg=repr(e) + ". Retrying in 5 seconds" + fio.read()
                        display_and_log("COM_PORT ERROR", errmsg)
                        time.sleep(5)
                        limit -= 1
                    else:
                        errmsg="Error connecting to COM port {}. Giving up... {}".format(params["com_port"], fio.read())
                        display_and_log("COM_PORT ERROR",errmsg)

            if serial_port is not None:
                # serial_port.tag = count
                serial_ports[port] = {"connection": serial_port, "parameters": params, "tag": count}
                display_and_log(SYSTEM_MSG_TAG, "{}: Connected to serial port {}".format(count, port))
                count += 1
    return serial_ports


def reset_com_ports():
    if len(serial_ports) > 1:
        display_and_log(SYSTEM_MSG_TAG, "Resetting serial port connections")
        display_and_log(SYSTEM_MSG_TAG, "NOT IMPLEMENTED")
    # if port is changed for a given serial_port, the serial_port is closed/reopened as per pySerial docs
    # TODO
    display_and_log(SYSTEM_MSG_TAG, "Serial ports have been reset")
