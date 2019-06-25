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
        uint8_t  pwm_motor_B;        // (bits 0-7)   8 bits: Valor do PWM do Motor B
        uint8_t  pwm_motor_A;        // (bits 8-15)  8 bits: Valor do PWM do Motor A
        uint8_t  dir_motor_B : 2,    // (bits 16-17) 2 bits: BIN1 e BIN2  da ponte H
                 dir_motor_A : 2,    // (bits 18-19) 2 bits: AIN1 e AIN2  da ponte H
                 ign1_4b     : 4;    // (bits 20-23) 4 bits não utilizados (padding)
        uint8_t  ign2_8b;            // (bits 24-31) 8 bits não utilizados (padding)
    } config;
    uint32_t status = 0;           // Leitura/Escrita simuntânea do conjunto de variáveis.
} TMotCtrl;

typedef union {
    struct {
          uint16_t  data1 :8,
                    data2 :8; 
    };
    int16_t data;
} Data;


typedef union  {
    struct {
        char      chr;      // bits menos significativos
        uint8_t   id    :4,
                  pad   :4;
        Data      data;     // bits mais significativos
    } conf;
    uint32_t  stats = 0x00000000;
} TRadioMsg;



//  ----------------------- declaração de variáveis globais -----------------------------------

char data_input;
TMotCtrl data, data_exit;
// char data_ant = 'n';
const byte canais[2] = {0x00, 0xFF}; 
unsigned long final_time;
TRadioMsg msg;

// --------------------------- declaração de funções e afins --------------------------------
/*
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
*/
void send_message(){
    if (!radio.write( &msg.stats, sizeof(TRadioMsg))) {
        Serial.print("Transmissão ");
        Serial.print(msg.stats, HEX);
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
        Serial.println(msg.stats, HEX);

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

//    data.pwm_A = 50;    data.pwm_B = 50;
}
/*
void traduz_str(String ent){
    String aux;
    char res[2];
    char chr;
    long t1;
    uint16_t t2;
    int i = 0, j = 0;
    String *teste;

    chr = ent[0];
//    Serial.print("Char: ");
//    Serial.println(chr);
    while (true){
        if (ent[i] == ' ' || ent[i] == '!'){
            Serial.println(" 'qreba'");
            break;
        }
        else{
            res[i] = ent[i];
            Serial.print(ent[i]);
            i++;
        }
    }
    Serial.println(aux);
    t1 = atoi(res);
    Serial.print("Id: ");
    Serial.println(t1);
}
*/

//void
void trata_msg(char subs[5][6]){
    msg.conf.chr = toupper(msg.conf.chr);
//    Serial.println(input);
//    Serial.println(subs[2]);
    msg.conf.pad = 0;
    switch (msg.conf.chr){
        case 'D':
    //        msg.conf. 
              // "tradução" da 3a string p/ os bits da pad
              for (int i= 0; i < 4; i++){
                  if (subs[2][i] == '0')
                      msg.conf.pad = msg.conf.pad | 0b0;
                  else
                      msg.conf.pad = msg.conf.pad | 0b1;
//                  Serial.print("msg.conf.pad: ");
//                  Serial.println(msg.conf.pad, BIN);
                  if (i < 3)
                      msg.conf.pad = msg.conf.pad << 1;
             }
//             Serial.print("msg.conf.pad: ");
//             Serial.println(msg.conf.pad, BIN);
             
             msg.conf.data.data1 = atoi(subs[3]);
             msg.conf.data.data2 = atoi(subs[4]);
//             Serial.println(msg.conf.data.data1);
//             Serial.println(msg.conf.data.data2);
             
            break;
        
        case 'M':
             msg.conf.data.data1 = atoi(subs[2]);
             msg.conf.data.data2 = atoi(subs[3]);
//             Serial.print("data1: ");
//             Serial.println(msg.conf.data.data1);
//             Serial.print("data2: ");
//             Serial.println(msg.conf.data.data1);
    
            break;
            
        case 'S':
             msg.conf.data.data = atoi(subs[2]);
//             Serial.print("data: ");
//             Serial.println(msg.conf.data.data);
             
            break;
        case 'R':
             // se os graus forem negativos, o pad recebe 1111 e o número deixa de ser negativo p/ evitar overflow(ou sei lá)
             Serial.print("Mensagem tipo R. subs [2][0]:");
             Serial.println(subs[2][0]);
             if (subs[2][0] == '-'){
                Serial.println("negativo");
                msg.conf.pad = 0b1111;
                Serial.println(msg.conf.pad);
                subs[2][0] = '0';
//                for (int t = 0; t < 3; t++)
//                  Serial.print(subs[2][t]);
                
             }
             else
                msg.conf.pad = 0b0000;

             msg.conf.data.data = atoi(subs[2]);
//             Serial.print("atoi: ");
//             Serial.println(msg.conf.data.data);

//             Serial.print("data: ");
//             Serial.println(msg.conf.data.data);
    
            break;
        default:
    //       msg.conf.data.data = atoi(subs[2]);
        
            break;
    }
}   


void traduz_str(String ent){
    char subs[5][6];

    int j, i = 0, k = 0;
  
    while(ent[i] != '\0') {
        j = 0;
        while ((ent[i] != ' ') && (ent[i] != '\0')) {
            subs[k][j] = ent[i];
            i++;
            j++;
        }
        subs[k][j] = '\0';
        Serial.println(subs[k]);
        i++;
        k++;
    }
//    int t1;

    msg.conf.id   = atoi(subs[0]);
    msg.conf.chr  = subs[1][0];
//    Serial.print("msg.id: ");
//    Serial.println(msg.conf.id);
//    Serial.print("chr: ");
//    Serial.println(msg.conf.chr);
    trata_msg (subs);
//    byte t2;
//    string.getBytes(buf, len)
}


void loop() {
//
//    unsigned long started_waiting_at = millis();
//    if (Serial.available()/* && (millis() - started_waiting_at < 100 )*/){
//        data_input = Serial.read();
//        final_time = millis();
//    }
//    if (final_time - started_waiting_at > 3000 )      // se o usuário demorar p/ mandar um dado, decido deixar o robô parado
//        data_input = 'x';
//    translate_input(data_input);
//    Serial.println(data.hexa,HEX);
//    send_message();
//    // if (data == 'l')
//    //     digitalWrite (L1G, HIGH);    // este é um teste simples: ligo o LED caso a entrada seja l
    String v;
    if (Serial.available()){
        v = Serial.readString();
//      Serial.println(v);  // eco de mensagem recebida
//      traduz_str(v);    
        traduz_str(v);
        send_message();
    }
    else
      v = ' ';
//        Serial.println("Serial indisponível");
    // delay(50);
}