import threading
import serial
from serial.serialutil import PARITY_EVEN
from serial.threaded import ReaderThread, LineReader
import sys
import time
from threading import Thread,Event
from enum import IntEnum
import numpy as np
import multiprocessing as mp

statusByte = {
	0x00: "Success",
	0x01: "Failed",
	0x02: "Module busy",
	0x03: "Error occurred",
	0x04: "Key not found",
	0xF5: "Supply voltage too low",
	0xFF: "Operation not permitted"
	}
	
simpleCNF = {
	0x40: "Reset request",
	0x42: "Sleep request",
	0x44: "Send data",
	0x49: "Set channel",
	0x51: "User setting set",
	0x5B: "Restart in transparent mode",
	0x5C: "Factory reset",
	0x5D: "Set radio test",
	0x5F: "Restart in bootloader mode",
	0x69: "Remote GPIO config set",
	0x6A: "Remote GPIO config get",
	0x6B: "Remote GPIO write",
	0x6C: "Remote GPIO read"
	}

class sendRequest(IntEnum):
	 CMD_RESET = 0x00 #request a soft reset
	 CMD_GETSTATE = 0x01 #request the current state
	 CMD_SLEEP = 0x02 #send board to sleep
	 CMD_UNICAST_DATA = 0x04 #send data to default address
	 CMD_MULTICAST_DATA = 0x05 #send data to default multi group id
	 CMD_BROADCAST_DATA = 0x06 #broadcast data to everyone
	 CMD_UNICAST_DATA_EX = 0x07 #send data to address in payload
	 CMD_MULTICAST_DATA_EX = 0x08 #send data to goup id in payload
	 CMD_SETCHANNEL = 0x09 #set the channel
	 CMD_GET = 0x10 #get setting
	 CMD_SET = 0x11 #set setting
	 CMD_FACTORY_RESET = 0x1C #do a factory reset
	 CMD_DTM_START = 0x1D #restart in radio test mode
	 CMD_DTM = 0x1E #start radio test

class settings(IntEnum):
	SERIAL_NUMBER = 1
	FW_VERSION = 2
	UART_CONFIG = 4
	UART_MODE = 5
	UART_TRANSPARENT_TIMEOUT = 6
	RF_CHANNEL = 7
	RF_PROFILE = 9
	RF_NUM_RETRIES = 10
	RF_TX_POWER = 11
	MAC_SOURCE_ADDRESS = 16
	MAC_DEST_ADDRESS = 17
	MAC_GROUP_ID = 18
	MODULE_MODE = 32

packetCnt = 0
byteCnt = 0
reqCnfEvent = Event()
reqRspEvent = Event()

msgEvent = Event()
output = True
startTime = 0
measurements = []

## Helper functions

def measStart():
	"""Starts a time measurement on the global startTime variable
	"""
	global startTime
	startTime = time.time_ns()

def measStop():
	"""Stops the time measurement and appends it to a measurements array
	"""
	global startTime, measurements
	assert startTime != 0, "ERROR: measStart not called"
	measurements.append(time.time_ns()-startTime)
	startTime = 0

def twos(val):
	"""Returns the twos complement of a value

	Args:
		val: the integer to transform

	Returns:
		An integer that represents the value in twos complement
	"""
	b = val.to_bytes(1, byteorder=sys.byteorder, signed=False)
	return int.from_bytes(b, byteorder=sys.byteorder, signed=True)

def getPayloadLength(data):
	"""Gets the length of a ThyoneI payload message

	Args:
		data: a bytearray() Object with the message

	Returns:
		length of the message
	"""
	return data[2] + (data[3] << 8)
	
def getPacketCS(data):
	"""Calculates the checksum of a message

	Args:
		data: a bytearray() Object with the message

	Returns:
		the calculated checksum
	"""
	cs = 0
	for i in data:
		cs = cs^i
	return cs

## Reader Process Functions

def read(comPort, rQue, tQue):
	"""Process loop for the IO part. It opens a com port and reads values into rQue and writes values from tQue

	Args:
		comPort: the COM port (as string. e.g. "COM3") to open
		rQue: Queue object to put read data in
		tQue: Queue object to write data from
	"""
	ser = serial.serial_for_url(comPort, baudrate=1000000, timeout=1, xonxoff=False, parity = PARITY_EVEN, rtscts = True) #open com port
	while(True):
		if(ser.in_waiting != 0): #data available
			que.put(ser.read(ser.in_waiting)) #read everything in buffer
		try:
			ser.write(tQue.get(False)) #write if something is in transmit queue but dont block
		except: #if transmit queue is empty, it will throw an exception
			pass
# Parser Process Functions
#class Parser(): #To make things actually work with processes we need to go back to using classes (AGAIN) :/ 
#	def __init__(parseQueue, cnfQueue, indQueue):
#		self.buff = bytearray()
#		self.expectedCnt = 5
#		self.parseQueue = parseQueue
#		self.cnfQueue = cnfQueue
#		self.indQueue = indQueue
		


def parse(que):
	"""Process function for the parser process that reads from the read queue and parses the raw bytes into messages

	Args:
		que: the read queue that the reader process writes to
	"""
	buff = bytearray() #empty bytes array
	expectedCnt = 5 #We expect a payload of 0 bytes which results into a lenght of 5 bytes (start, cmd, lenL, lenH, cs) 
	while(True):
		buffLen = len(buff) #get how many bytes we already have
		data = que.get() #get data from our queue
		#print("Data: "+str(data)+" length: "+str(len(data)))
		buff.extend(data) #write data to buffer
		#print("Buffer: "+str(buff)+" length: "+str(buffLen) +" expected: "+str(expectedCnt))
		if buffLen <= 3 and buffLen + len(data) >= 4: # byte 3 and 4 have payload length
			if(buff[0] != 0x02): #but only if we dont have trash in buffer (happens when reset is pressed and 0x00 is sent by resetting module)
				try:
					trim = buff.index(0x02) #search for 0x02 which marks the start of a message
					expectedCnt += trim #fix our expected count
					buff = buff[trim:] #remove trash bytes
					if len(buff) < 3: #not enough bytes to get length
						continue
				except ValueError: #all trash. wipe everything (no 0x02 found in buffer)
					buff = bytearray()
					expectedCnt = 5
					continue

			length = buff[2] + (buff[3] << 8) #our buffer has the payload lenght in it
			expectedCnt +=  length # we now know the payload lenght and adjust our expected count
			#print("Got lenght in packet. Length: "+ str(length)+" Expected: "+str(expectedCnt))
		expectedCnt -= len(data) #we got data
		while(expectedCnt <= 0):
			if(expectedCnt == 0): #exactly one message in buffer
				handle_packet(buff) #send the packet to get handled
				buff = bytearray() #reset the buffer
				expectedCnt = 5 #reset the cnt
			else: #we received more than one message (-expectedLen too many bytes)
				handle_packet(buff[:expectedCnt]) #because expectedCnt is negative, this will count from behind and give us the first package
				buff = buff[expectedCnt:] #new buffer with the packet thats left
				if(len(buff)>=4): #we got the length byte already
						expectedCnt = (5 + buff[2] + (buff[3] << 8)) - len(buff) #remove the message 
				else:
					expectedCnt = 5 - len(buff)

def handle_packet(data):
	"""Handles a full message

	Args:
		data: a bytearray() Object with the message
	"""
	if(getPacketCS(data[:-1]) != data[-1]): #check if cs is correct
		print("Checksum wrong. Packet broken!")
		return
	if output: #only print if output is true
		print("<<<", end=" ")
	if(0x40 <= data[1] <= 0x6C): #CNF commands
		handle_CNF(data)
	elif(0x73 <= data[1] <= 0xEC):#IND commands
		handle_IND(data)
		#if(0xC4 <= data[1] <= 0xEC): #its a response and needs to set the response flag TODO: Adjust for processes
			#reqRspEvent.set()
	else:
		print("Something is wrong") #unknown command

def handle_CNF(data):
	"""Handles a CNF message and prints it to the terminal

	Args:
		data: a bytearray() Object with the message
	"""
	cmd = data[1]
	if (getPayloadLength(data) == 1): #one of the simple cnf that only have a status byte
		if output:
			print(simpleCNF[cmd]+" received with status: "+statusByte[data[4]])
	else: #CNF that has a payload
		if(cmd == 0x41):
			print("Mode request received with status: " +statusByte[data[4]]+" and mode: "+{0x01: "Application", 0x05: "Test"}[data[5]])
		elif(cmd == 0x50):
			print("User setting received with status: " + statusByte[data[4]]+" and parameter: "+data[4:-1].hex())
		elif(cmd == 0x5E):
			tmp = "Set radio test received with status " + statusByte[data[4]]
			if getPayloadLength(data) > 1:
				tmp += " and result: "
				res = (data[5] << 8) + data[6]
				if res < 0x8000:
					tmp += "Test " + ["successful", "failed"][res]
				else:
					tmp += "Test successful and received "+ str(res -0x8000)+" packets"
			print(tmp)
		#TODO: GPIO CNF
def handle_IND(data):
	"""Handles a IND message and prints it to the terminal

	Args:
		data: a bytearray() Object with the message
	"""
	cmd = data[1]
	if(cmd == 0x73):
		print("Thyone module (re-)started from " + {0x01: "Power on", 0x02: "Pin reset", 0x04: "Soft reset", 0x06: "Wake up from sleep"}[data[5]] +" in " +{0x01: "Application", 0x05: "Test"}[data[6]] + " mode")
	elif(cmd == 0x84):
		src = data[4:8]
		src.reverse()
		rssi = twos(data[8])
		if output:
			if(getPayloadLength(data) - 5 > 15): #dont print payload if its bigger that 15 bytes
					print(str(packetCnt)+": Received "+str(getPayloadLength(data) - 5) + " Bytes from " + src.hex()+ " with "+ str(rssi)+"dBm")
			else:
				payload = data[9:-1]
				decoded = ""
				try:
					decoded = payload.decode("utf-8")
				except UnicodeDecodeError:
					pass
				print(str(packetCnt)+": Received "+str(getPayloadLength(data) - 5) + " Bytes from " + src.hex()+ " with "+ str(rssi)+"dBm : "+payload.hex() + " | "+ decoded)
	elif(cmd == 0xC4):
		if output:
			print("Data transmitted with status: " + statusByte[data[4]])
	elif(cmd == 0xA2):
		if(data[4] == 0x01):
			print("Error indicated:UART communication error")
		else:
			print("Error indicated: Unknown")


### sender Process TODO: migrate from threads to processes

def safeWrite(data):
	"""A write function that writes to the com port and waits until ThyoneI returns a CNF (and if needed a RSP)

	Args:
		data: the bytearray Object to write
	"""
	global reqEvent
	reqCnfEvent.clear()
	ser.write(data)
	measStart()
	reqCnfEvent.wait()
	reqRspEvent.wait()

def createPacket(cmd, pld):
	"""Builds a ThyoneI packet with a command and a payload

	Args:
		cmd: The command
		pld: The payload

	Returns:
		A bytearray object that can be written to ThyoneI
	"""
	pkt = bytearray([0x02,int(cmd), len(pld) & 0xFF, (len(pld) >> 8)&0xFF, *pld])
	pkt.append(getPacketCS(pkt))
	return pkt

def list2hex(list):
	"""Creates a hex string of a list of integers

	Args:
		list: list of integers

	Returns:
		String containing the integers as hex for pretty printing
	"""
	return "".join(['{:02x}'.format(x) for x in list])

def str2ascii(str):
	"""chreates a list of integers that represent a string in ascii notation

	Args:
		str: The string

	Returns:
		list object with the chars of the string to ascii
	"""
	return list(map(ord,list(str)))

def ascii2str(list):
	"""Converts a list of integers to their corresponding chars and joins them to a string

	Args:
		list: list of integers

	Returns:
		The string thats represented by the list
	"""
	return "".join(chr(c) for c in list)

def reset():
	"""Resets the ThyoneI
	"""
	print(">>> Sending reset request")
	safeWrite(createPacket(sendRequest.CMD_RESET, []))

def factoryReset():
	"""Factory resets the ThyoneI
	"""
	print(">>> Sending factory reset request")
	safeWrite(createPacket(sendRequest.CMD_FACTORY_RESET, []))
		
def getState():
	"""Gets the ThyoneI state
	"""
	print(">>> Sending get state request")
	safeWrite(createPacket(sendRequest.CMD_GETSTATE, []))

def sendData(cmd, pld):
	"""Sends a send data command to ThyoneI to send data to a preconfigured address 

	Args:
		cmd: The type of send command (Broadcast, Multicast, Singlecast)
		pld: The payload to send
	"""
	assert 0x04 <= cmd <= 0x06, "sendData called with wrong command" #make sure its a send data command
	assert len(pld) <= 224, "payload too big for one packet" #check if payload is the right size
	global reqRspEvent
	if output:
		decodedPld = ascii2str(pld)		
		print(">>> Sending "+cmd.name+" with payload: "+list2hex(pld)+" | "+decodedPld)
	reqRspEvent.clear() #clear the response event so that the safeWrite function waits for it
	safeWrite(createPacket(cmd, pld))

def sendDataEx(cmd,addr,pld):
	"""Sends the send data explicit command to ThyoneI to send data to an address in the payload 

	Args:
		cmd ([type]): The type of command (Multicast, Singlecast) 
		addr ([type]): The address to send to 
		pld ([type]): The payload to send
	"""
	assert 0x07<= cmd <= 0x08, "sendDataEx called with wrong command" #make sure its a send data ex command
	assert (cmd == 0x08 and len(addr) == 1 and len(pld) < 223) or (cmd == 0x07 and len(addr) == 3 and len(pld) < 220), "address or payload size wrong" #check if correct address and correct payload sizes
	global reqRspEvent
	
	decodedPld = ascii2str(pld)
	print(">>> Sending "+cmd.name+" to address "+list2hex(addr)+" with payload: "+list2hex(pld)+" | "+decodedPld)
	reqRspEvent.clear()
	safeWrite(createPacket(cmd, [addr,*pld]))
	
def setChannel(channel):
	"""Sets the ThyoneI frequency channel

	Args:
		channel: The channel number (refer to datasheet for frequencies. I am lazy)
	"""
	assert 0 <= channel <= 38, "Invalid channel selected"

	print(">>> Sending channel switch request to channel "+str(channel))
	safeWrite(createPacket(sendRequest.CMD_SETCHANNEL,[channel]))
	
def getSetting(setting):
	"""Request a setting from the ThyoneI. This will trigger a CNF response with the setting

	Args:
		setting: The setting to get
	"""
	assert 0 <= setting <= 32, "Invalid setting requested"
	print(">>> Requesting setting "+setting.name)
	safeWrite(createPacket(sendRequest.CMD_GET,[setting]))

def setSetting(setting, value):
	"""Set a setting on the ThyoneI

	Args:
		setting: The setting
		value: The value to set
	"""
	assert 0 <= setting <= 32, "Invalid setting requested"
	print(">>> Setting "+setting.name+ " to: "+ list2hex(value))
	safeWrite(createPacket(sendRequest.CMD_SET,[setting, *value]))

def sender(self):
	"""The sender Thread TODO: fix for process
	"""
	global byteCnt
	time.sleep(0.5)
	reset()
	#getState()
	#getSetting(settings.FW_VERSION)
	#getSetting(settings.UART_CONFIG)
	#getSetting(settings.UART_MODE)
	#getSetting(settings.RF_CHANNEL)
	#getSetting(settings.RF_PROFILE)
	#getSetting(settings.RF_TX_POWER)
	time.sleep(0.5)

	cnt = 0
	startTim = time.time_ns()
	while(True):
		msgEvent.clear()
		#sendData(sendRequest.CMD_BROADCAST_DATA,str2ascii("Hello")) #"U"*220))
		#msgEvent.wait()
		byteCnt = time.time_ns() - startTim
		startTim = time.time_ns()
		#time.sleep(0.1)


if __name__ == "__main__":
	mp.set_start_method('spawn') #windows compatibility
	comPort = "COM3" #thyone com port
	readQueue = mp.Queue() 
	transmitQueue = mp.Queue()
	readProcess = mp.Process(target=read, args=(comPort, readQueue, transmitQueue)) #configure read process
	parseProcess = mp.Process(target=parse, args=(readQueue, )) #configure parse process
	#masterProcess = 
	readProcess.start()
	parseProcess.start()
	while(True):
		try:
			time.sleep(5)
		except KeyboardInterrupt: #ctrl c handler
			readProcess.terminate()
			parseProcess.terminate()
			exit()
	# Dead code which was used for threads
	reader = readerThread(ser)
	sender = senderThread(ser)
	reader.daemon = True
	sender.daemon = True
	reader.start()
	sender.start()
	start = time.time_ns()
	time.sleep(5)
	print(np.mean(measurements)/1e6)
	print(min(measurements)/1e6)
	print(max(measurements)/1e6)
	exit()
	while(True):
		if(time.time_ns()-start >= 1e9):
			#print("Speed: "+str(byteCnt/1e6)+"bits/s")
			byteCnt = 0
			start = time.time_ns()
		time.sleep(0.2)