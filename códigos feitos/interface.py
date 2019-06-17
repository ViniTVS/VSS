# -*- coding: UTF-8 -*-
from getch import getch
import serial
# from serial import Serial

SSPEED = 115200
print ("Escreva a porta conforme comentado no código")
porta = input("Porta conectada: ")	# é necessário que o usuário forneça a porta p/ conexão serial
									# exemplo: '/dev/ttyUSB0' (é necessário o uso de aspas simples)
ArduinoSerial = serial.Serial(porta,SSPEED)
#time.sleep(2)
#print ArduinoSerial.readline()
#ArduinoSerial.write('n')	
string = "quero que fe"

print ("Digite [p] para sair:")
# string = raw_input("digite algo:")	

while True:
    #entrada = 'n'				# defino o dado como n p/ funcionar semelhante a um nop e não pegar lixo de memória
	# pega chars como entrada sem a necesidade de teclar enter
	string = raw_input("Comando:")
	if string[0] == 'p' or string[0] == 'P':
		break
	ArduinoSerial.write(string)
