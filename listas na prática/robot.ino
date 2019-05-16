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
RF24 radio(RADIO_CE,RADIO_CS);

uint32_t  blinker = 0;
uint32_t  timer = 0;
//byte   endereco;
mensagem comando;
bool locked;
int enc_a_val = 0, enc_b_val = 0;


// --------------------------- declaração de funções e afins --------------------------------

byte end_radio (){  // cálculo do enderço do robô dos pinos físicos para o endereço na variável
  byte addr = 0x00;
  addr = addr | !digitalRead(RADIO_A1);
  addr = addr << 1;
  addr = addr | !digitalRead(RADIO_A0); 

  Serial.print("Endereço do robô: ");
  Serial.println(addr);

  return addr;
}

void tasks(int tempo){    // função de teste p/ fazer um escalonador
  // faço a checagem p/ ver se executo este escalonador
  if( (millis() - (timer - 1)) > tempo){
    // caso sim, atualizo o valor da var. timer
    timer = millis();
    blinker++;
    // escrevo o bit menos significativo de blinke no LED
    // se o nº for par, seu último bit é zero, caso ímpar, o bit é 1
    // então, sempre que blinker for par, o LED se apaga 
    digitalWrite(LED, bitRead(blinker, 0));
    Serial.print(timer);
    Serial.print(" de ");  
    Serial.print(blinker); 

    if (bitRead(blinker, 0) == 0)
      Serial.println(" - LED desligado ");
    else
      Serial.println(" - LED ligado ");

    //acelero mais o robô para testar diferentes velocidades
    // comando.hexa= comando.hexa + 10;
  }
}
//como interrupts não deixa passar parâmetros, ambos enc_check fazem o mesmo
void enc_check_a (){  // cada vez que tenho uma interrupção de borda descendo do enc, 
            // aumento o valor de suas variáveis
  enc_a_val++;
  Serial.print("\nValor do enc A: ");
  Serial.println(enc_a_val);
}

void enc_check_b (){
  enc_b_val++;
  Serial.print("\nValor do enc B: ");
  Serial.println(enc_b_val);
}

void set_motor_status (mensagem entrada){ // defino o direcionamento do robô e sua vel.
  // arrumo as pontes H com os dados da entrada
  digitalWrite(MTR_BIN1, bitRead(entrada.ponte_B, 0));
  digitalWrite(MTR_BIN2, bitRead(entrada.ponte_B, 1));

  digitalWrite(MTR_AIN1, bitRead(entrada.ponte_A, 0));
  digitalWrite(MTR_AIN2, bitRead(entrada.ponte_A, 1));
  
  // ajusto as velocidades caso sejam acima ou abaixo do esperado      
  if (entrada.pwm_A < PWM_MIN)
      entrada.pwm_A = PWM_MIN;
  if (entrada.pwm_A > PWM_MAX)
      entrada.pwm_A = PWM_MAX;
  analogWrite(MTR_PWMA, entrada.pwm_A);

  if (entrada.pwm_B < PWM_MIN)
      entrada.pwm_B = PWM_MIN;
  if (entrada.pwm_B > PWM_MAX)
      entrada.pwm_B = PWM_MAX;
  analogWrite(MTR_PWMB, entrada.pwm_B);

  // para calcular o número de furos necessários para andar,
  // faço uma "regra de 3" e arredondo o valor 

  // como andar todos os 24 furos é o mesmo que virar em 90º, então:
  float furos;
  if (entrada.pwm_B != entrada.pwm_A){
    furos = (WHEEL_TICKS * entrada.angulo)/90.0;
    furos = round (furos);
  }
  else
    furos = 5;
  
  // zero os contadores dos encoders
  enc_a_val = 0; enc_b_val = 0;

  // habilito as pontes H
  digitalWrite (HBRID_EN,entrada.ponte);

  while (!is_motor_locked(entrada)){  // se o motor não estiver travado...
    // faço a contagem nas var. dos encoders, alterando o valor na borda de descida dos encoders
    // (quando "passo" o pino, o contabilizo)
      attachInterrupt(digitalPinToInterrupt(IRQ_ENC_A), enc_check_a, FALLING);    
    attachInterrupt(digitalPinToInterrupt(IRQ_ENC_B), enc_check_b, FALLING);

    if ((enc_a_val == furos) || (enc_b_val == furos)){  // quando andados os furos necessários,desligo a ponte H
//      digitalWrite (HBRID_EN,LOW);
      delay(50);
      break;
    }
  }
}

bool is_motor_locked(mensagem entrada){   // retorna 1 se o motor estiver travado
  bool testeA, testeB, testeF;

  testeA = bitRead(entrada.ponte_A, 0) ^ bitRead(entrada.ponte_A, 1); // faço o xor dos bits de ambas as pontes
  testeB = bitRead(entrada.ponte_B, 0) ^ bitRead(entrada.ponte_B, 1); // já que 00 e 11 são freio elétrico
  testeF = testeA | testeB;
//  testeF = testeF & digitalRead(HBRID_EN);  // se qualquer ponte estiver ligada e se o enable da ponte está ligado
  
  // no final, testeF será 1 caso esteja andando
  if (testeF == 0)
    Serial.println("AVISO: MOTOR TRAVADO (freio elétrico)");

  return !testeF; // como quero saber se o robo está parado, retorno o contrário
}

// uint32_t get_motor_status( void );

void get_motor_status (mensagem entrada){
  if (digitalRead(HBRID_EN) == 0){  // ponte H desativada
    Serial.println("O robô está na banguela");  
    return;
  }

  if (is_motor_locked(entrada))
    return;
  else{
    Serial.println("O robô está em movimento");
//    delay(500);
    return;
  }
}


// --------------------------- setup e loop do arduino ----------------------------------------

byte endereco = end_radio ();      // carrego o valor do end. robo
const byte canais[2] = {0x00, 0xFF}; 

void setup (){

    Serial.begin(SSPEED);

    pinMode(LED     , OUTPUT);          // defino LED como output 
    digitalWrite(LED, LOW);             // e o deixo apagado    

    pinMode(IRQ_ENC_A, INPUT_PULLUP); 
    pinMode(IRQ_ENC_B, INPUT_PULLUP);
    pinMode(RADIO_A0 , INPUT_PULLUP);    // Endereço deste nó: bit 0
    pinMode(RADIO_A1 , INPUT_PULLUP);    // Endereço deste nó: bit 1

    // Habilito as saidas de controle da ponte H
    pinMode(HBRID_EN, OUTPUT);          // Habilita o pino da ponte H
    digitalWrite (HBRID_EN,LOW);    // e a deixo desativada
    pinMode(MTR_AIN1, OUTPUT);          // Bit 0 - Controle da ponte H do Motor A
    pinMode(MTR_AIN2, OUTPUT);          // Bit 1 - Controle da ponte H do Motor A
    pinMode(MTR_BIN1, OUTPUT);          // Bit 0 - Controle da ponte H do Motor B
    pinMode(MTR_BIN2, OUTPUT);          // Bit 1 - Controle da ponte H do Motor B
    pinMode(MTR_PWMA, OUTPUT);          // Sinal de PWM para controle  do Motor A
    pinMode(MTR_PWMB, OUTPUT);          // Sinal de PWM para controle  do Motor B

    //inicialização do rádio e
    radio.begin();
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_2MBPS);
    radio.setChannel(netw_channel);
    radio.openWritingPipe(canais[0]);
    radio.openReadingPipe(1, canais[1]);
    radio.startListening(); 

    // tudo teste:
    // comando.hexa = 0x167A3232;
    // comando.ponte = 1;
    Serial.println("Valor de pwm_A:");
    Serial.println(comando.pwm_A); 
    Serial.println("Valor de pwm_B:");
    Serial.println(comando.pwm_B);
    Serial.println("Valor da ponte:");
    Serial.println(comando.ponte);
    Serial.println("Valor de ponte_B:");
    Serial.println(comando.ponte_B);
    Serial.println("Valor de ponte_A:");
    Serial.println(comando.ponte_A);
    Serial.println("Valor de angulo:");
    Serial.println(comando.angulo);
    Serial.println("Valor de not_used:");
    Serial.println(comando.not_used);

    // Serial.println("Motor locked:");
    // locked = is_motor_locked(comando);
    // Serial.println(locked);

    // task_dir(0,mensagem);

}

void loop() {
    // tasks(1000);                // Tarefas executadas a cada 1s
    if ( radio.available()) {
        radio.read( &comando.hexa, sizeof(mensagem));
        Serial.print("Mensagem recebida: ");
        Serial.println(comando.hexa, BIN);
        digitalWrite(LED, 1);
        Serial.println("Valor de pwm_A:");
        Serial.println(comando.pwm_A); 
        Serial.println("Valor de pwm_B:");
        Serial.println(comando.pwm_B);
        Serial.println("Valor da ponte:");
        Serial.println(comando.ponte);
        Serial.println("Valor de ponte_B:");
        Serial.println(comando.ponte_B);
        Serial.println("Valor de ponte_A:");
        Serial.println(comando.ponte_A);
        Serial.println("Valor de angulo:");
        Serial.println(comando.angulo);
        Serial.println("Valor de not_used:");
        Serial.println(comando.not_used);
        //Serial.print("Comando recebido: ");
            set_motor_status(comando);    

        //Serial.println(data);
    }
//    else
        digitalWrite(LED, 0);

    // int sei_la = 20;
    // attachInterrupt(digitalPinToInterrupt(IRQ_ENC_A), enc_check_a, FALLING);  //altera o valor quando  
    // attachInterrupt(digitalPinToInterrupt(IRQ_ENC_B), enc_check_b, FALLING); //altera o valor quando
    // comando.angulo = 39;
    set_motor_status(comando);    
}
