import gc
import uos
from flashbdev import bdev

try:
    if bdev:
        uos.mount(bdev, "/")
except OSError:
    import inisetup

    vfs = inisetup.setup()

    
import webrepl
import network
import machine
import time
import ubinascii
from umqtt.robust import MQTTClient
from machine import WDT
from esp32 import Partition

SLEEPTIME_S=1800
#SLEEPTIME_S=20
wdt = WDT(timeout=(SLEEPTIME_S+100)*1000)
Partition(Partition.RUNNING).mark_app_valid_cancel_rollback() #it booted, so cancel roll-back
WEBREPL_PASS=""

def do_connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        available=[i[0] for i in wlan.scan()]
        if b'hurstpark' in available:
            wlan.connect('hurstpark', 'impasse-cambridge-gestas')
        elif b'papeterie' in available:
            wlan.connect('papeterie', 'impasse-cambridge-gestas')
        else:
            assert False
        while not wlan.isconnected():
            pass
    print('network config:', wlan.ifconfig())
    return wlan.ifconfig()[0]

ip=do_connect()

def mqtt_sub_cb(topic, msg):
    global WEBREPL_PASS
    WEBREPL_PASS=msg
    print((topic,msg))

c = MQTTClient("umqtt_client", "10.9.8.1")
c.set_callback(mqtt_sub_cb)
c.connect(clean_session=True)
mac = ubinascii.hexlify(network.WLAN().config('mac'),':').decode()
c.subscribe("esp32/{}/enablewebrepl".format(mac))
time.sleep(1)
c.check_msg()
c.disconnect()



c.connect()    
if WEBREPL_PASS:
    c.publish("esp32/{}/enablewebreplstatus".format(mac),"starting webrepl: {} {}".format(WEBREPL_PASS,ip))
    webrepl.start(password=WEBREPL_PASS)
else:
    c.publish("esp32/{}/enablewebreplstatus".format(mac),"not starting webrepl: {}".format(ip))
c.disconnect()

    
gc.collect()
