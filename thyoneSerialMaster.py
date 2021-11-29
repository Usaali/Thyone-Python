import threading
import serial
from serial.serialutil import PARITY_EVEN
from serial.threaded import ReaderThread, LineReader
import sys
import time
from threading import Thread,Event
from enum import IntEnum

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
output = False
	
def twos(val):
	b = val.to_bytes(1, byteorder=sys.byteorder, signed=False)
	return int.from_bytes(b, byteorder=sys.byteorder, signed=True)

def getPayloadLength(data):
	return data[2] + (data[3] << 8)
	
def getPacketCS(data):
	cs = 0
	for i in data:
		cs = cs^i
	return cs
	
class readerThread(Thread):
	buff = None
	expectedCnt = 5

	def __init__(self, ser):
		Thread.__init__(self)
		self.buff = bytearray()
		self.ser = ser

	def data_received(self,data):
		buffLen = len(self.buff)
		#print("Data: "+str(data)+" length: "+str(len(data)))
		self.buff.extend(data)
		#print("Buffer: "+str(self.buff)+" length: "+str(buffLen) +" expected: "+str(self.expectedCnt))
		if buffLen <= 3 and buffLen + len(data) >= 4:		# byte 3 and 4 have payload length
			self.expectedCnt += self.buff[2] + (self.buff[3] << 8) 
			#print("Got lenght in packet. Expected: "+str(self.expectedCnt))
		self.expectedCnt -= len(data)
		while(self.expectedCnt <= 0):
			if(self.expectedCnt == 0):
				self.handle_packet(self.buff)
				self.buff = bytearray()
				self.expectedCnt = 5
			else: #we received more than one message (-expectedLen too many)
				self.handle_packet(self.buff[:self.expectedCnt]) #because expected Cnt negative, this will count from behind and give us the first package
				self.buff = self.buff[self.expectedCnt:] #new buffer with the packet thats left
				if(len(self.buff)>=4): #we got the length byte already
						self.expectedCnt = (5 + self.buff[2] + (self.buff[3] << 8)) - len(self.buff)
				else:
					self.expectedCnt = 5 - len(self.buff)

	def handle_packet(self, data):
		global packetCnt, reqCnfEvent, reqRspEvent
		if(getPacketCS(data[:-1]) != data[-1]):
			print("Checksum wrong. Packet broken!")
			return
		if(0x40 <= data[1] <= 0x6C):
			if output:
				print("<<<", end=" ")
			self.handle_CNF(data)
			reqCnfEvent.set()
		elif(0x73 <= data[1] <= 0xEC):
			if output:
				print("<<<", end=" ")
			self.handle_IND(data)
			if(0xC4 <= data[1] <= 0xEC): #its a response and needs to set the response flag
				reqRspEvent.set()
		else:
			print("Something is wrong")
	
	def handle_CNF(self,data):
		cmd = data[1]
		if (getPayloadLength(data) == 1): #one of the simple cnf that only have a status byte
			if output:
				print(simpleCNF[cmd]+" received with status: "+statusByte[data[4]])
		else:
			if(cmd == 0x41):
				print("Mode request received with status: " +statusByte[data[4]]+" and mode: "+["Application", "Test"][data[5]])
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
			#missing: GPIO CNF
	def handle_IND(self,data):
		global packetCnt, msgEvent
		cmd = data[1]
		#print(data.hex())
		if(cmd == 0x73):
			print("Thyone module (re-)started from " + {0x01: "Power on", 0x02: "Pin reset", 0x04: "Soft reset", 0x06: "Wake up from sleep"}[data[5]] +" in " +{0x00: "Application", 0x01: "Test"}[data[6]] + " mode")
		elif(cmd == 0x84):
			packetCnt+=1
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
			msgEvent.set()
		elif(cmd == 0xC4):
			if output:
				print("Data transmitted with status: " + statusByte[data[4]])
		elif(cmd == 0xA2):
			if(data[4] == 0x01):
				print("Error indicated:UART communication error")
			else:
				print("Error indicated: Unknown")
	
	def run(self):
		while(True):
			if(self.ser.in_waiting != 0):
				self.data_received(self.ser.read(self.ser.in_waiting))


class senderThread(Thread):
	def __init__(self, ser):
		Thread.__init__(self)
		global reqRspEvent
		self.ser = ser
		reqRspEvent.set()

	def safeWrite(self, data):
		global reqEvent
		reqCnfEvent.clear()
		self.ser.write(data)
		reqCnfEvent.wait()
		reqRspEvent.wait()

	def createPacket(self, cmd, pld):
		pkt = bytearray([0x02,int(cmd), len(pld) & 0xFF, (len(pld) >> 8)&0xFF, *pld])
		pkt.append(getPacketCS(pkt))
		return pkt
	
	def list2hex(self, list):
		return "".join(['{:02x}'.format(x) for x in list])
	
	def str2ascii(self,str):
		return list(map(ord,list(str)))
	
	def ascii2str(self,list):
		return "".join(chr(c) for c in list)

	def reset(self):
		print(">>> Sending reset request")
		self.safeWrite(self.createPacket(sendRequest.CMD_RESET, []))

	def factoryReset(self):
		print(">>> Sending factory reset request")
		self.safeWrite(self.createPacket(sendRequest.CMD_FACTORY_RESET, []))
		
	def getState(self):
		print(">>> Sending get state request")
		self.safeWrite(self.createPacket(sendRequest.CMD_GETSTATE, []))

	def sendData(self, cmd, pld):
		assert 0x04 <= cmd <= 0x06, "sendData called with wrong command" #make sure its a send data command
		assert len(pld) <= 224, "payload too big for one packet" #check if payload is the right size
		global reqRspEvent
		if output:
			decodedPld = self.ascii2str(pld)		
			print(">>> Sending "+cmd.name+" with payload: "+self.list2hex(pld)+" | "+decodedPld)
		reqRspEvent.clear()
		self.safeWrite(self.createPacket(cmd, pld))

	def sendDataEx(self,cmd,addr,pld):
		assert 0x07<= cmd <= 0x08, "sendDataEx called with wrong command" #make sure its a send data ex command
		assert (cmd == 0x08 and len(addr) == 1 and len(pld) < 223) or (cmd == 0x07 and len(addr) == 3 and len(pld) < 220), "address or payload size wrong" #check if correct address and correct payload sizes
		global reqRspEvent

		decodedPld = self.ascii2str(pld)
		print(">>> Sending "+cmd.name+" to address "+self.list2hex(addr)+" with payload: "+self.list2hex(pld)+" | "+decodedPld)
		reqRspEvent.clear()
		self.safeWrite(self.createPacket(cmd, [addr,*pld]))
	
	def setChannel(self,channel):
		assert 0 <= channel <= 38, "Invalid channel selected"

		print(">>> Sending channel switch request to channel "+str(channel))
		self.safeWrite(self.createPacket(sendRequest.CMD_SETCHANNEL,[channel]))
	
	def getSetting(self,setting):
		assert 0 <= setting <= 32, "Invalid setting requested"
		print(">>> Requesting setting "+setting.name)
		self.safeWrite(self.createPacket(sendRequest.CMD_GET,[setting]))

	def setSetting(self,setting, value):
		assert 0 <= setting <= 32, "Invalid setting requested"
		print(">>> Setting "+setting.name+ " to: "+ self.list2hex(value))
		self.safeWrite(self.createPacket(sendRequest.CMD_SET,[setting, *value]))

	def run(self):
		global byteCnt
		time.sleep(0.5)
		self.getState()
		self.getSetting(settings.FW_VERSION)
		#self.getSetting(settings.UART_CONFIG)
		#self.getSetting(settings.UART_MODE)
		#self.getSetting(settings.RF_CHANNEL)
		#self.getSetting(settings.RF_PROFILE)
		#self.getSetting(settings.RF_TX_POWER)
		time.sleep(0.5)

		cnt = 0
		startTim = time.time_ns()
		while(True):
			#msgEvent.clear()
			self.sendData(sendRequest.CMD_BROADCAST_DATA,self.str2ascii("U"*220))
			#msgEvent.wait()
			byteCnt = time.time_ns() - startTim
			startTim = time.time_ns()
			#time.sleep(1)


ser = serial.serial_for_url('COM6', baudrate=1000000, timeout=1, xonxoff=False, parity = PARITY_EVEN, rtscts = True)
reader = readerThread(ser)
sender = senderThread(ser)
reader.daemon = True
sender.daemon = True
reader.start()
sender.start()
start = time.time_ns()
while(True):
	if(time.time_ns()-start >= 1e9):
		print("Speed: "+str(byteCnt/1e6)+"bits/s")
		byteCnt = 0
		start = time.time_ns()
	time.sleep(0.2)
		