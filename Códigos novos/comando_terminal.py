# -*- coding: UTF-8 -*-
import getch
import serial
from serial import Serial

SSPEED = 115200

porta = input("Porta conectada: ")	# é necessário que o usuário forneça a porta p/ conexão serial
									# exemplo: '/dev/ttyS0' (é necessário o uso de aspas simples)
ArduinoSerial = serial.Serial(porta,SSPEED)
#time.sleep(2)
#print ArduinoSerial.readline()
ArduinoSerial.write('n')	

print ("Precione [p] para sair:")
while True:
	#entrada = getch.getch()
	entrada = 'n'				# defino o dado como n p/ funcionar semelhante a um nop e não pegar lixo de memória
	entrada = getch.getch();	# pega chars como entrada sem a necesidade de teclar enter

	ArduinoSerial.write(entrada)
	print ("A entrada foi:"),entrada

	if entrada == 'p' or entrada == 'P':		# critério p/ sair do loop
		break
