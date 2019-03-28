#include <Motorino.h>
#include <avr/pgmspace.h>
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
#define NETW_CHANNEL     100   // Canal padrão de operação do rádio

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
#define MTR_AIN1   A3      // Bit 0 - Controle da ponte H do Motor A  <-- mudança com relação
#define MTR_AIN2   A2      // Bit 1 - Controle da ponte H do Motor A  <-- ao código original
#define MTR_BIN1   A1      // Bit 0 - Controle da ponte H do Motor B
#define MTR_BIN2   A0      // Bit 1 - Controle da ponte H do Motor B
#define MTR_PWMA    9      // Sinal de PWM para controle  do Motor A
#define MTR_PWMB   10      // Sinal de PWM para controle  do Motor B

#define VOLT_BAT   A7      // Tensão da bateria -> Vcc/10

/* ******************************************************************* */
/* Definições de estruturas de dados ( funcionais, status e controle ) */

void direcao_robo (int PWM, char direcao){
    direcao = toupper(direcao);
    analogWrite(MTR_PWMB, PWM);
    analogWrite(MTR_PWMA, PWM);

    
    switch (direcao){
        case 'W':
            digitalWrite(MTR_BIN1, LOW );
            digitalWrite(MTR_BIN2, HIGH);
            digitalWrite(MTR_AIN1, LOW );
            digitalWrite(MTR_AIN2, HIGH);
            
            break;
        case 'S':
            digitalWrite(MTR_BIN1, HIGH);
            digitalWrite(MTR_BIN2, LOW );
            digitalWrite(MTR_AIN1, HIGH);
            digitalWrite(MTR_AIN2, LOW );
            
            break;
        case 'D':
            digitalWrite(MTR_BIN1, HIGH);
            digitalWrite(MTR_BIN2, LOW );
            digitalWrite(MTR_AIN1, LOW );
            digitalWrite(MTR_AIN2, LOW );
            
            break;
        case 'A':
            digitalWrite(MTR_BIN1, LOW );
            digitalWrite(MTR_BIN2, LOW );
            digitalWrite(MTR_AIN1, LOW );
            digitalWrite(MTR_AIN2, HIGH);

            break;            
        default:
            digitalWrite(MTR_BIN1, HIGH);
            digitalWrite(MTR_BIN2, HIGH);
            digitalWrite(MTR_AIN1, HIGH);
            digitalWrite(MTR_AIN2, HIGH);
            
            digitalWrite(MOTOR_BRK_H, HIGH);
            digitalWrite(MOTOR_BRK_L, LOW);
            
            break;
       }// o resto é comando via rádio 
}

void setup (){
    //Serial.begin(9600);
    
    pinMode(HBRID_EN, OUTPUT);          // Habilita ponte H
    digitalWrite (HBRID_EN,HIGH);       // 
    digitalWrite (LED,HIGH);       // 

    
    pinMode(MTR_AIN1, OUTPUT);          // Bit 0 - Controle da ponte H do Motor A
    pinMode(MTR_AIN2, OUTPUT);          // Bit 1 - Controle da ponte H do Motor A
    pinMode(MTR_BIN1, OUTPUT);          // Bit 0 - Controle da ponte H do Motor B
    pinMode(MTR_BIN2, OUTPUT);          // Bit 1 - Controle da ponte H do Motor B
    pinMode(MTR_PWMA, OUTPUT);          // Sinal de PWM para controle  do Motor A
    pinMode(MTR_PWMB, OUTPUT);          // Sinal de PWM para controle  do Motor B
}

void loop (){
     direcao_robo (50, 'w');
     delay(1000);
     direcao_robo (50, 'a');
     delay(300);
     direcao_robo (50, 's');
     delay(1000);
     direcao_robo (50, 'd');
     delay(300);
     direcao_robo (50, 't');
     delay(1000);
     

}
