import RPi.GPIO as GPIO
import subprocess
import os
import time                        #used to get sleep and timestamp for filename
from datetime import datetime      #used to get microsecond timestamps
import csv
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

x = False

# File Operation Class 
class FileOp:
	def __init__(self):
		self.usb_mounted = False
		self.mnt = subprocess.call("mountpoint -q /media/usb1", shell=True)
		if self.mnt == 0:
			self.usb_mounted = True

	def open_file(self):
		if self.usb_mounted == True:
			self.fname = time.strftime("%Y%m%d_%H%M%S", time.localtime())
			self.file = open("/media/usb1/" + self.fname + ".csv", "a+", newline='')
			self.writer = csv.writer(self.file,delimiter='\t')

	def write(self, val):
		if not (self.file.closed):
			self.writer.writerows(val)

	def close_file(self):
		self.file.close()


fo = FileOp()


# GPIO Record Button Handler
def rec_btn_callback(channel):
	global x
	global fo
	if x == False:
		print("Starting to write file")
		fo.open_file()
		GPIO.output(17,GPIO.HIGH)              #Turn on recording indicator LED
		x = True
	else:
		print("Stopping / Closing file")
		fo.close_file()
		GPIO.output(17,GPIO.LOW)               #Turn off recording indicator LED
		x = False

def shutdown_btn_callback(chnl):
	os.system("shutdown now -h")                   #Turn off the Rasp

# GPIO Access
GPIO.setwarnings(False)                                #Ignore Warnings
GPIO.setmode(GPIO.BCM)                                 #Use BCM Pin Numbering
GPIO.setup(17, GPIO.OUT)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)    # Set (GPIO18) for input and pulled low
GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)    # Set (Pin 18 / GPIO24) for triggering a shutdown

GPIO.add_event_detect(18,GPIO.RISING, callback=rec_btn_callback, bouncetime=300)         # Record button
GPIO.add_event_detect(24,GPIO.RISING, callback=shutdown_btn_callback, bouncetime=300)    # Shutdown button 

# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create the ADC object using the I2C bus
ads1 = ADS.ADS1015(i2c, address=0x48)
ads2 = ADS.ADS1015(i2c, address=0x49)
ads3 = ADS.ADS1015(i2c, address=0x4A)
ads4 = ADS.ADS1015(i2c, address=0x4B)

# Create input channels
# ADC1 (operator side)
chan01 = AnalogIn(ads1, ADS.P0)
chan02 = AnalogIn(ads1, ADS.P1)
chan03 = AnalogIn(ads1, ADS.P2)
chan04 = AnalogIn(ads1, ADS.P3)
# ADC2
chan05 = AnalogIn(ads2, ADS.P0)
chan06 = AnalogIn(ads2, ADS.P1)
chan07 = AnalogIn(ads2, ADS.P2)
chan08 = AnalogIn(ads2, ADS.P3)
# ADC3
chan09 = AnalogIn(ads3, ADS.P0)
chan10 = AnalogIn(ads3, ADS.P1)
chan11 = AnalogIn(ads3, ADS.P2)
chan12 = AnalogIn(ads3, ADS.P3)
# ACD4 (machine side)
chan13 = AnalogIn(ads4, ADS.P0)
chan14 = AnalogIn(ads4, ADS.P1)
chan15 = AnalogIn(ads4, ADS.P2)


#print("{:>5}\t{:>5}".format('raw', 'v'))

chan_value = [chan01, chan02, chan03, chan04, chan05, chan06, chan07, chan08, chan09, chan10, chan11, chan12, chan13, chan14, chan15]

# Currently Adjustment values are not used - use this approach if adjustment values are needed
#adj_value = [ 1.04438451030322, 0.977557025707482, 1.08750201194372, 1.03005328039686, 1.10133799416433,
#  1.09780935440337, 1.03782247846704, 0.920201153101695, 1.01052194979014, 1.02927945588788,
#  1.01120291535804, 0.981952348918503, 0.909305704015221, 0.775000722236208,  0.98606909530629 ]

# Main Loop
while True:
	if x == True:
#		lst_output = [[a.value*b for a,b in zip(chan_value,adj_value)]]       #Multiply sensor vals by adjustment vals
		lst_output = [[a.value for a in chan_value]]
		lst_output[0].insert(0, datetime.now().strftime("%Y%m%d_%H%M%S_%f"))

		fo.write(lst_output)

		time.sleep(0.3)
	else:
		time.sleep(1)


#try:
#    GPIO.wait_for_edge(18,GPIO.RISING, bouncetime=300)
#    print("button pushed")
#except KeyboardInterrupt:
#    GPIO.cleanup()

