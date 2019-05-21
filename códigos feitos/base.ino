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

// ------------------------- declaração de typedefs -----------------------------

typedef union {
    struct {
        uint8_t pwm_A;        // bits menos significativos
        uint8_t pwm_B;
        uint16_t ponte_A : 2,  
                 ponte_B : 2,
                 angulo  : 9,                
                 not_used: 2,
         ponte   : 1; // bits mais significativos
    };
    uint32_t hexa = 0x00000000;   // sempre que declarar uma variável controle, ela iniciará com seus valores zerados
} mensagem;

//  ----------------------- declaração de variáveis globais -----------------------------------

char data_input;
mensagem data, data_exit;
// char data_ant = 'n';
const byte canais[2] = {0x00, 0xFF}; 
unsigned long final_time;

// --------------------------- declaração de funções e afins --------------------------------

void translate_input(char input){
    input = toupper(input);
    switch (input){
    case 'W':
        data.ponte = 1;
        data.ponte_A = 0b01;
        data.ponte_B = 0b01;  
        break;
    
    case 'A':
        data.ponte = 1;
        data.ponte_A = 0b00;
        data.ponte_B = 0b01;
        data.angulo = 90;
        break;
    
    case 'S':
        data.ponte = 1;
        data.ponte_A = 0b11;
        data.ponte_B = 0b11;
    
        break;
    
    case 'D':
        data.ponte = 1;
        data.ponte_A = 0b01;
        data.ponte_B = 0b00; 
        data.angulo = 90;
        break;
    
    case 'M':
        data.pwm_A = data.pwm_A + 10;
        data.pwm_B = data.pwm_B + 10;
        break;
    
    case 'N':
        data.pwm_A = data.pwm_A - 10;
        data.pwm_B = data.pwm_B - 10;    
        break;
    
    default:
        data.ponte = 0;
        break;
    }
}

void send_message(){
    if (!radio.write( &data.hexa, sizeof(mensagem))) {
        Serial.print("Transmissão ");
        Serial.print(data.hexa, HEX);
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
        Serial.println(data.hexa);

        // ligo os LEDs verdes 2 e 3 e desligo o resto
        digitalWrite(L1G, LOW);
        digitalWrite(L1R, LOW);
        digitalWrite(L2R, LOW);
        digitalWrite(L3R, LOW);
        digitalWrite(L2G, HIGH);
        digitalWrite(L3G, HIGH);
        //Serial.print (final_time);
    }
}

// --------------------------- setup e loop do arduino ----------------------------------------

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
    // radio.openReadingPipe(1, canais[0]);  // recebo do canal 0

    data.pwm_A = 50;    data.pwm_B = 50;
}

void loop() {
    unsigned long started_waiting_at = millis();
    if (Serial.available()/* && (millis() - started_waiting_at < 100 )*/){
        data_input = Serial.read();
        final_time = millis();
    }
    if (final_time - started_waiting_at > 3000 )      // se o usuário demorar p/ mandar um dado, decido deixar o robô parado
        data_input = 'x';
    translate_input(data_input);
    Serial.println(data.hexa,HEX);
    send_message();
    // if (data == 'l')
    //     digitalWrite (L1G, HIGH);    // este é um teste simples: ligo o LED caso a entrada seja l

    


    delay(500);
}
