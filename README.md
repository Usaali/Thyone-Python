# Thyone-Python
This repository contains code to communicate with the WE Thyone I RF chip
## ThyoneSerialMaster/Slave
This is the first implementation using threading in python. It works but because of the GIL in python that wont let threads run parallel, it is extremely slow and can only be used to send around 7Kbit/s of data
## ThyoneSerialProcess
This is the adaptation of the threading based code an uses processes to speed things up.
