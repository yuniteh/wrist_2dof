# boot.py -- run on boot-up
# can run arbitrary Python, but best to keep it minimal

import machine
import pyb
import os
# import network
# import webrepl

# webrepl.start()
# ap = network.WLAN(network.AP_IF)
# ap.active(True)
pyb.country('US') # ISO 3166-1 Alpha-2 code, eg US, GB, DE, AU
pyb.freq(168000000, 168000000, 54000000, 108000000)
pyb.main('hargrove_pr.py') # main script to run after this one
sd = pyb.SDCard()
os.mount(sd, '/sd')
pyb.usb_mode('VCP+MSC',msc=(pyb.Flash(), pyb.SDCard())) # act as a serial and a storage device
#pyb.usb_mode('VCP+HID') # act as a serial device and a mouse
#pyb.usb_mode('VCP+MSC', msc=(pyb.SDCard(),))