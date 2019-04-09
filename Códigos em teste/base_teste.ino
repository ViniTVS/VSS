//#include <Motorino.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

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


// dados p/ o programa em si:
char data, data_saida;
char data_ant = 'n';
const byte canais[2] = {0xF0, 0xFF}; 

void setup() {
  Serial.begin(SSPEED);
  Serial.println("Arduino transmissor ligado");

  pinMode(L1G, OUTPUT);
  digitalWrite (L1G, LOW);
  
  pinMode(RADIO_PWR, OUTPUT);
  digitalWrite(RADIO_PWR, HIGH);

  radio.begin();
  radio.setPALevel(RF24_PA_MAX);        // deixar o alcance o máx. possível por... motivos
  radio.setDataRate(RF24_2MBPS);        // vel. de transmissão
  radio.setChannel(netw_channel);       // canal de transmissão
  // Abertura do canal de comunicação:
  radio.openWritingPipe(canais[1]);     // transmito no canal 1 
  radio.openReadingPipe(1, canais[0]);  // recebo do canal 0

}

void loop() {
  unsigned long started_waiting_at = millis();
  unsigned long final_time;
  if (Serial.available()/* && (millis() - started_waiting_at < 100 )*/){
      data = Serial.read();
      final_time = millis();
  }
  if (final_time - started_waiting_at > 3000 )      // se o usuário demorar p/ mandar um dado, decido deixar o robô parado
    data = 'n';

  if (data == 'l')
    digitalWrite (L1G, HIGH);    // este é um teste simples: ligo o LED caso a entrada seja l
  
  /*
  if (data == 'o')
    digitalWrite (L1G, LOW);
  */

  //radio.stopListening();
  /*                                  // teste de "solução" do prolema de transmição de dados***
  if (data == 'n' && data_ant == 'n')
    data_saida = 'n';
  else
    data_saida = data_ant;
  data_ant = data;
  */

  if (!radio.write( &data, sizeof(char))) {
    Serial.print("Transmissão ");
    Serial.print(data);
    Serial.println(" não enviada - conexão perdida");
    //Serial.println (final_time);
    
    // caso não consiga enviar os dados, desligos os LEDs verdes e ligo os vermelhos
    digitalWrite(L1R, HIGH);
    digitalWrite(L2R, HIGH);
    digitalWrite(L3R, HIGH);
    digitalWrite(L1G, LOW);
    digitalWrite(L2G, LOW);
    digitalWrite(L3G, LOW);
  }
  //if(radio.write( &data_saida, sizeof(char))){
  else{ 
    Serial.print("Sent: ");     // aviso qual dado foi enviado
    Serial.println(data);

    // ligo os LEDs verdes 2 e 3 e desligo o resto
    digitalWrite(L1G, LOW);
    digitalWrite(L1R, LOW);
    digitalWrite(L2R, LOW);
    digitalWrite(L3R, LOW);
    digitalWrite(L2G, HIGH);
    digitalWrite(L3G, HIGH);
    //Serial.print (final_time);
  }
  delay(5);
}
