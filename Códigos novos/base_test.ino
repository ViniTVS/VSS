#include <Motorino.h>
#include <avr/pgmspace.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 0;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
/**********************************************************/

byte addresses[][6] = {"1Node", "2Node"};

// da base:

#define SSPEED        115200   // Velocidade da interface serial
#define SERIAL_ECHO        0   // Eco da msg recebida (Prod: 0 | Debug: 1) 

#define TAM_BUFFER        16   // Buffer de SW para o rádio
#define BASE_ADDRESS      00   // Base tem o endereço 7 (em octal)
#define netw_channel     100   // Canal padrão de operação do rádio

// Pinos de controle do módulo de rádio (independentes da SPI)

#define RADIO_CE          A0   // Pino CE do módulo de rádio
#define RADIO_CS          A1   // Pino CS do módulo do rádio
#define RADIO_PWR         A2   // Liga/Desl. módulo de rádio

// LEDs de sinalização.
// Obs: Os LEDs vermelhos estão conectados a pinos com PWM.

#define L1G                4   // Led 1 verde
#define L1R                5   // Led 1 vermelho
#define L2G                7   // Led 2 verde
#define L2R                6   // Led 2 vermelho
#define L3G                8   // Led 3 verde
#define L3R                9   // Led 3 vermelho


RF24 radio(RADIO_CE, RADIO_CS);  // Instância do rádio
char data;

void setup() {
  Serial.begin(SSPEED);
  pinMode(L1G, OUTPUT);
  digitalWrite (L1G, LOW);
  Serial.println("Hi!, I am Arduino");
  //Serial.println("THIS IS THE TRANSMITTER CODE - YOU NEED THE OTHER ARDIUNO TO SEND BACK A RESPONSE");
  pinMode(RADIO_PWR, OUTPUT);
  digitalWrite(RADIO_PWR, HIGH);
  // Initiate the radio object
  radio.begin();

  // Set the transmit power to lowest available to prevent power supply related issues
  radio.setPALevel(RF24_PA_MIN);

  // Set the speed of the transmission
  radio.setDataRate(RF24_2MBPS);

  radio.setChannel(netw_channel);

  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);

}

void loop() {
  while (Serial.available()){
    unsigned long started_waiting_at = millis();
    if (millis() - started_waiting_at > 100 ) {
      //Serial.println("No response received - timeout!");
      data = 'n';
      break;
    }
    else{
    
    data = Serial.read();
    
    if (data == 'l')
      digitalWrite (L1G, HIGH);

    else if (data == 'o')
      digitalWrite (L1G, LOW);
    }
  }

  //char data = 'w';

  // Ensure we have stopped listening 
  radio.stopListening();

  if (!radio.write( &data, sizeof(char) )) {
    Serial.println("No acknowledgement of transmission - receiving radio device connected?");
  }
  
  //radio.startListening();
  //unsigned long started_waiting_at = millis();

  // Loop here until we get indication that some data is ready for us to read (or we time out)
  /*
  while ( ! radio.available() ) {
    if (millis() - started_waiting_at > 500 ) {
      Serial.println("No response received - timeout!");
      return;
    }
  }
   */
  /*
  char dataRx;
  radio.read( &dataRx, sizeof(char) );
  */
  // Show user what we sent and what we got back
  Serial.print("Sent: ");
  Serial.println(data);
  /*Serial.print(", received: ");
  Serial.println(dataRx);
  */
  // Try again 0,1s later
  delay(100);
  //data = ' ';
}
