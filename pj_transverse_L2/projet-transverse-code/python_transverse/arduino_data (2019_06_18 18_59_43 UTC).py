"""
script python pour l'analyse et l'utilisation des donnees recu par bluetooth

"""
import serial
import bluetooth
import sys
import time
import json
import numpy as np
from matplotlib import pyplot
import autopy
"""
ser = serial.Serial('COM1', 9600, timeout = 1)
f = open('data.txt','w+')
try:
	while 1:
		line = ser.readline()
		if line: f.write(line)
except KeyboardInterrupt:
	f.close()
	ser.close()
"""
data_json = {"y":[],"x":[],"button_R":[],"button_L":[]}

############ serial ###########

###############################

print "Searching for devices..."
print ""
devices = bluetooth.discover_devices()

num = 0
print "Select your device by entering its coresponding number..."
for i in devices:
	num+=1
	print num , ": " , bluetooth.lookup_name( i )

selection = input("> ") -1
print "You have selected", bluetooth.lookup_name(devices[selection])
bd_addr = devices[selection]
print "--",bd_addr

port = 1

sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
sock.connect((bd_addr, port))
print 'Connected'
sock.send(" ");
sock.settimeout(5.0)

data = "a"
data_collect = ""
stock_data_event = ""
bis_data = ""
bis2 = ""
count=0

while (data and count<500) :
	data = sock.recv(8)
	bis_data = data
	print str(data).rstrip()
	data_collect += data
	count+=1
	#tous recoder tester prendre le dernier et de la liste+=data puis on split
	bis2 += bis_data
	stock_data_event = bis2.split("/")
	if len(stock_data_event)>=5 :
		for i in range(4):
			if stock_data_event[i] == "":
				stock_data_event[i] = 0
		print "stock",stock_data_event
		y = float(stock_data_event[0])
		x = -float(stock_data_event[1])
		R = float(stock_data_event[2])
		L = float(stock_data_event[3])
		print "x : ",x," y : ",y," R : ",R," L : ",L  
		#autopy.mouse.move(27,60)
		if L>10 or R>10:
			RClick()
		stock_data_event = stock_data_event[4:]

	data = data_collect.split("/")

print "data",data
start = 0
for event in data_json:
	data_json[event]=data[start::4]
	start+=1
	
print data_json
with open('Documents/projet-transverse-code/python_transverse/data.json', 'w') as fichier:
    fichier.write(json.dumps(data_json, indent=4))



sock.close()

def map( x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def LClick():
    autopy.mouse.click(LEFT_BUTTON)
    time.sleep(.1)

def RClick():
    autopy.mouse.click("RIGHT")
    time.sleep(.1)
