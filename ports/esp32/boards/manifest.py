freeze("$(PORT_DIR)/modules")
module("upip.py", base_path="$(MPY_DIR)/tools", opt=3)
module("upip_utarfile.py", base_path="$(MPY_DIR)/tools", opt=3)
include("$(MPY_DIR)/extmod/uasyncio")

# Require some micropython-lib modules.
require("dht")
require("ds18x20")
require("neopixel")
require("ntptime")
require("onewire")
require("umqtt.robust")
require("umqtt.simple")
require("upysh")
require("urequests")
require("webrepl")

include("$(PORT_DIR)/custom_modules_for_freezing/lib/bluetooth/aioble/manifest.py")
freeze("$(PORT_DIR)/custom_modules_for_freezing", ("evo_gateway/__init__.py", "evo_gateway/app.py", "evo_gateway/app_helper.py","evo_gateway/config.py","evo_gateway/general.py","evo_gateway/globalcfg.py","evo_gateway/mqtt.py"))
freeze("$(PORT_DIR)/custom_modules_for_freezing/lib", ("configparser.py"))
freeze("$(PORT_DIR)/custom_modules_for_freezing/lib", ("uping.py"))
freeze("$(PORT_DIR)/custom_modules_for_freezing/lib", ("update_firmware.py"))
freeze("$(PORT_DIR)/custom_modules_for_freezing/lib", ("os/path.py"))
freeze("$(PORT_DIR)/custom_modules_for_freezing/lib", ("collections/__init__.py","collections/defaultdict.py","collections/deque.py"))
