//#include <Motorino.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

/* ****************************************************************** */
/* *** Definições diversas ****************************************** */

// Descomente para inverter a rotação (somente) do Motor A
// => Necessário com os motores montados simetricamente
#define MTR_REVERSE

#define SSPEED        115200   // Velocidade da interface serial

#define TAM_BUFFER        16   // Buffer de SW para o rádio
#define BASE_ADDRESS      00   // Base tem o endereço 0 (em octal)
#define netw_channel     100   // Canal padrão de operação do rádio

#define MOTOR_CW           2   // Sentido horário:      10b
#define MOTOR_CCW          1   // Sentido anti-horário: 01b
#define MOTOR_BRK_L        0   // Freio elétrico:       00b
#define MOTOR_BRK_H        3   // Freio elétrico:       11b

#define MOTOR_A            0   // Referência ao motor A
#define MOTOR_B            1   // Referência ao motor B

#define WHEEL_TICKS       48   // Número de furos por roda
#define WHEEL_DIAM        60   // Diâmetro da cada roda, em mm
#define WHEELS_SPC        60   // Espaçamento entre rodas, em mm

#define FULL_BAT        8000   // Valor em mV para bat. completamente carregada
#define DEAD_BAT        6000   // Valor em mV para bat. esgotada ( recarregar )

/* ****************************************************************** */
/* Limitações de PWM e velocidade para uso no controlador PID ******* */

#define PWM_MIN         0x0F   // PWM mín. p/ garantir movimento das duas rodas
#define PWM_MAX         0x9F   // PWM máx. para que os motores tenham aprox. 5V
#define SPD_MIN           50   // Vel. mín. em mm/s ( Condição: PWM > PWM_MIN )
#define SPD_MAX          500   // Vel. máx. em mm/s ( Condição: PWM < PWM_MAX )

/* ****************************************************************** */
/* Estas são as conexões de hardware mapeadas aos pinos do Arduino ** */

#define LED         4      // Led conectado a uma saída digital

#define RADIO_CE    7      // Pino CE do módulo de rádio
#define RADIO_CS    8      // Pino CS do módulo do rádio
#define RADIO_A0   A4      // Bit 0 do end. do rádio (LOW = ligado)
#define RADIO_A1   A5      // Bit 1 do end. do rádio (LOW = ligado)

#define IRQ_ENC_A   2      // Pino de interrupção do Encoder A
#define IRQ_ENC_B   3      // Pino de interrupção do Encoder B
#define IRQ_RADIO   5      // Pino de interrupção do Rádio

#define HBRID_EN    6      // Habilita a ponte H (High)
#define MTR_AIN1   A3      // Bit 0 - Controle da ponte H do Motor A
#define MTR_AIN2   A2      // Bit 1 - Controle da ponte H do Motor A
#define MTR_BIN1   A1      // Bit 0 - Controle da ponte H do Motor B
#define MTR_BIN2   A0      // Bit 1 - Controle da ponte H do Motor B
#define MTR_PWMA    9      // Sinal de PWM para controle  do Motor A
#define MTR_PWMB   10      // Sinal de PWM para controle  do Motor B

#define VOLT_BAT   A7      // Tensão da bateria -> Vcc/10

RF24 radio(RADIO_CE,RADIO_CS);

// dados p/ o programa em si:
char data = 'n';
const byte canais[2] = {0xF0, 0xFF}; 
int enc_a, enc_b;
int v_enc_a = 0;
int vel = 100;
//bool move = false;


//  teste p/ que o robo ande de acordo com quantos graus sejam desejados
void teste_enc (int enc, int graus){    // entrada: encoder que será usado e o tanto de graus a serem "andados" 
  // andar todos os 24 furos é o mesmo que virar em 90º, então:
  int andar = WHEEL_TICKS*graus/90;
  
  int anterior = 0;
  int atual;
  int andado = 0;
  int erro = furos / 20;
  while(true){
    atual = digitalRead(enc);
    if (atual == LOW && anterior == HIGH)   // vejo se a roda realmente andou ou se 
      andado++;

    else{
      Serial.print(andado);
      Serial.print(" de ");
      Serial.println(furos);
    }
    anterior = atual;

    if (andado == andar - erro)
      break;
  }
  Serial.print(andado);
}

void direcao_robo (int PWM, char direcao){    

    direcao = toupper(direcao); // padronizo a entrada em char maiúsculo
    analogWrite(MTR_PWMB, PWM); // coloco vel. nas rodas
    analogWrite(MTR_PWMA, PWM);

  // comandos W,A,S,D p/ direção (os padrões de jogos de computador)
    switch (direcao){
      // todos seguem um padrão:
      case 'W':                   
        digitalWrite(LED, HIGH);      // ligo o LED do robô caso seja um movimento W,A,S,D
        digitalWrite(MTR_BIN1, LOW ); // e arrumo as conexões das pontes H 
        digitalWrite(MTR_BIN2, HIGH);
        digitalWrite(MTR_AIN1, LOW );
        digitalWrite(MTR_AIN2, HIGH);

        //delay(50);
        //return true;
        break;
      case 'S':
        digitalWrite(LED, HIGH);
        digitalWrite(MTR_BIN1, HIGH);
        digitalWrite(MTR_BIN2, LOW );
        digitalWrite(MTR_AIN1, HIGH);
        digitalWrite(MTR_AIN2, LOW );

        break;
        
      case 'D':
        digitalWrite(LED, HIGH);
        digitalWrite(MTR_BIN1, LOW);
        digitalWrite(MTR_BIN2, HIGH);
        digitalWrite(MTR_AIN1, LOW );
        digitalWrite(MTR_AIN2, LOW );
        teste_enc (IRQ_ENC_B, 48);
        //return true;
        break;
      case 'A':
        digitalWrite(LED, HIGH);
        digitalWrite(MTR_BIN1, LOW );
        digitalWrite(MTR_BIN2, LOW );
        digitalWrite(MTR_AIN1, LOW );
        digitalWrite(MTR_AIN2, HIGH);
        //teste_enc (IRQ_ENC_A, 24);
        //delay(10);
        //return true;
        break;
      case 'Q':
        digitalWrite(LED, HIGH);
        digitalWrite(MTR_BIN1, LOW );
        digitalWrite(MTR_BIN2, LOW );
        digitalWrite(MTR_AIN1, LOW );
        digitalWrite(MTR_AIN2, HIGH);
        
        teste_enc (IRQ_ENC_A, 24);
        
        break; 
      case 'E':
        digitalWrite(LED, HIGH);
        digitalWrite(MTR_BIN1, LOW);
        digitalWrite(MTR_BIN2, HIGH);
        digitalWrite(MTR_AIN1, LOW );
        digitalWrite(MTR_AIN2, LOW );

        teste_enc (IRQ_ENC_B, 24);

        break;
      /*        ainda sendo feito
      case 'N':
        digitalWrite(MTR_BIN1, LOW);
        digitalWrite(MTR_BIN2, LOW);
        digitalWrite(MTR_AIN1, LOW);
        digitalWrite(MTR_AIN2, LOW);

        digitalWrite(MOTOR_BRK_H, LOW);
        digitalWrite(MOTOR_BRK_L, LOW);
          
        break;
      */     
      default:                      // freio elétrico
        digitalWrite(LED, LOW);     // LED fica desligado
        digitalWrite(MTR_BIN1, HIGH);
        digitalWrite(MTR_BIN2, HIGH);
        digitalWrite(MTR_AIN1, HIGH);
        digitalWrite(MTR_AIN2, HIGH);

        digitalWrite(MOTOR_BRK_H, HIGH);
        digitalWrite(MOTOR_BRK_L, LOW);

        //return false;
        break;          
      }
      delay(50); // tempo p/ que o robô se mova como pedido
      //return true;
}

void setup (){

  pinMode(HBRID_EN, OUTPUT);          // Habilita ponte H
  digitalWrite (HBRID_EN,HIGH);       //
  digitalWrite (LED,LOW);             //


  pinMode(MTR_AIN1, OUTPUT);          // Bit 0 - Controle da ponte H do Motor A
  pinMode(MTR_AIN2, OUTPUT);          // Bit 1 - Controle da ponte H do Motor A
  pinMode(MTR_BIN1, OUTPUT);          // Bit 0 - Controle da ponte H do Motor B
  pinMode(MTR_BIN2, OUTPUT);          // Bit 1 - Controle da ponte H do Motor B
  pinMode(MTR_PWMA, OUTPUT);          // Sinal de PWM para controle  do Motor A
  pinMode(MTR_PWMB, OUTPUT);          // Sinal de PWM para controle  do Motor B
  pinMode(LED     , OUTPUT);          // defino LED como output 
  digitalWrite(LED, LOW);             // e o deixo apagado


  Serial.begin(SSPEED);

  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(netw_channel);
  radio.openWritingPipe(canais[0]);
  radio.openReadingPipe(1, canais[1]);
  radio.startListening();           // espero os dados

  pinMode(IRQ_ENC_A, INPUT_PULLUP); // Declaro os encoders como pullup para evitar
  pinMode(IRQ_ENC_B, INPUT_PULLUP); // problemas com leituras HIGH e LOW 

}

void loop() {
  
  enc_a = digitalRead(IRQ_ENC_A);  // Quando um dos enc tem seu LED acesso (e seu infravermelho NÃO é interrompido)
  enc_b = digitalRead(IRQ_ENC_B);  // lê-se o valor LOW, caso contrário lê-se HIGH
/*  
  if (enc_a == LOW && move == true){       // teste simples p/ ver a leitura do encoder
     digitalWrite(LED, HIGH);
     v_enc_a++;
     Serial.println(v_enc_a);
     //delay(500);
  }
  */
//while (radio.available()) {
  if ( radio.available()) {
    radio.read( &data, sizeof(char));
    //Serial.print("Comando recebido: ");
    //Serial.println(&data);
  
  }

  direcao_robo (vel, data);

}
