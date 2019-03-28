# -*- coding: UTF-8 -*-
import getch
import serial
import time
from serial import Serial

SSPEED = 115200

ArduinoSerial = serial.Serial('/dev/ttyUSB0',SSPEED)
time.sleep(2)
print ArduinoSerial.readline()
ArduinoSerial.write('1')

def u_entrada():
	start_time = time.time()
	expires_in = 0.5 #in seconds
	saida = ''

	while (time.time() - start_time < expires_in): #keep looping if the time limit has not expired
		if saida:
			return saida
		"""else:
			break
		"""
		print"Esperando entrada"
		saida = getch.getch()
		print"Entrada obtida"
	print("Nah your time has expired I'm out anyways.")
	return 'n'
"""
	try:
		print 'You have 5 seconds to type in your stuff...'
		foo = raw_input()
		return foo
	except:
		# timeout
		return
	"""
"""
while True:
	test = raw_input() #get input from user
	print "you entered",test
	ArduinoSerial.write(test) #send 1
	if (test == 'l'): #if the value is 1
		print ("LED turned ON")
	if (test == 'o'): #if the value is 0
		print ("LED turned OFF")
	time.sleep(1)
"""
"""
	while agora - inicio < 1.00:
		entrada = getch.getch()
		print "A entrada foi:",entrada
		agora   = time.time()
"""

print "[P] para sair:"
while True:
	#entrada = getch.getch()
	entrada = 'n'
	entrada = u_entrada()
	"""
	inicio  = time.time()
	agora   = time.time()
	while True:
		agora   = time.time()

		if agora - inicio < 1.00:
			entrada = getch.getch()
			print "A entrada foi:",entrada
			agora   = time.time()
			break
		#if agora - inicio >= 1.00:
		else:
			entrada = getch.getch()
			print "A entrada foi:",entrada
			agora   = time.time()
			break
		
	"""
	if entrada == 'p':
		break
	
	ArduinoSerial.write(entrada)
	print "A entrada foi:",entrada
	"""
	if (getch.getch() != True):
		entrada = ' '
		print "A entrada foi alterada"
	"""
	
	if (entrada == 'l'): 
		print ("LED turned ON")
	if (entrada == 'o'): 
		print ("LED turned OFF")
	
