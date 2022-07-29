freeze("$(PORT_DIR)/modules")
freeze("$(MPY_DIR)/tools", ("upip.py", "upip_utarfile.py"))
freeze("$(MPY_DIR)/ports/esp8266/modules", "ntptime.py")
freeze("$(MPY_DIR)/drivers/dht", "dht.py")
freeze("$(MPY_DIR)/drivers/onewire")
include("$(MPY_DIR)/extmod/uasyncio/manifest.py")
include("$(MPY_DIR)/extmod/webrepl/manifest.py")
include("$(MPY_DIR)/drivers/neopixel/manifest.py")
include("$(PORT_DIR)/custom_modules_for_freezing/lib/bluetooth/aioble/manifest.py")

freeze("$(PORT_DIR)/custom_modules_for_freezing", ("evo_gateway/__init__.py", "evo_gateway/app.py", "evo_gateway/app_helper.py","evo_gateway/config.py","evo_gateway/general.py","evo_gateway/globalcfg.py","evo_gateway/mqtt.py"))
freeze("$(PORT_DIR)/custom_modules_for_freezing/lib", ("configparser.py"))
freeze("$(PORT_DIR)/custom_modules_for_freezing/lib", ("urequests.py"))
freeze("$(PORT_DIR)/custom_modules_for_freezing/lib", ("update_firmware.py"))
freeze("$(PORT_DIR)/custom_modules_for_freezing/lib", ("umqtt/simple.py","umqtt/robust.py"))
freeze("$(PORT_DIR)/custom_modules_for_freezing/lib", ("os/path.py"))
freeze("$(PORT_DIR)/custom_modules_for_freezing/lib", ("collections/__init__.py","collections/defaultdict.py","collections/deque.py"))