
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
#define MTR_AIN1   A2      // Bit 0 - Controle da ponte H do Motor A
#define MTR_AIN2   A3      // Bit 1 - Controle da ponte H do Motor A
#define MTR_BIN1   A1      // Bit 0 - Controle da ponte H do Motor B
#define MTR_BIN2   A0      // Bit 1 - Controle da ponte H do Motor B
#define MTR_PWMA    9      // Sinal de PWM para controle  do Motor A
#define MTR_PWMB   10      // Sinal de PWM para controle  do Motor B

#define VOLT_BAT   A7      // Tensão da bateria -> Vcc/10

/* ******************************************************************* */
/* Definições de estruturas de dados ( funcionais, status e controle ) */

typedef struct {
    uint8_t  type;              // 1 byte  - Tipo da mensagem
    uint32_t data;              // 4 bytes - Dado da mensagem
} TRadioMsg;


typedef struct {
    uint8_t   head;              // Índice do primeiro elemento da fila
    uint8_t   tail;              // Índice do último elemento da fila
    uint8_t   tam;               // Número de mensagens na fila
    uint32_t  last_t;            // Última transmissão ou recepção
    TRadioMsg msg[TAM_BUFFER];   // Fila de mensagens
} TRadioBuf;


typedef struct {
    uint32_t last_10ms   = 0;   // Controle das tarefas executadas a cada 10ms
    uint32_t last_100ms  = 0;   // Controle das tarefas executadas a cada 100ms
    uint32_t last_1000ms = 0;   // Controle das tarefas executadas a cada 1000ms
} TasksTCtr;


typedef union {
    struct {
        uint8_t  pwm_motor_B;        // (bits 0-7)   8 bits: Valor do PWM do Motor B
        uint8_t  pwm_motor_A;        // (bits 8-15)  8 bits: Valor do PWM do Motor A
        uint8_t  dir_motor_B : 2,    // (bits 16-17) 2 bits: BIN1 e BIN2  da ponte H
                 dir_motor_A : 2,    // (bits 18-19) 2 bits: AIN1 e AIN2  da ponte H
                 ign1_4b     : 4;    // (bits 20-23) 4 bits não utilizados (padding)
        uint8_t  ign2_8b;            // (bits 24-31) 8 bits não utilizados (padding)
    } config;
    uint32_t status = 0;             // Leitura/Escrita simuntânea do conjunto de variáveis.
} TMotCtrl;


typedef union {
    struct {
        unsigned  speed_avg  : 12;   // (bits 0-11)  Velocidade média (em mm/s)
        unsigned  angle      : 12;   // (bits 12-23) Ângulo restante de rotação
        unsigned  moving     :  2;   // (bits 24-25) 00 parado; 11 movimento; 01 rot. hor.; 10 rot. anti-hor.
        unsigned  ctr_pid_en :  1;   // (bit  26)    Habilitação do controlador PID
        unsigned  pwm_lim_en :  1;   // (bit  27)    Habilitação de limite de PWM  (p/ uso do PID)
        unsigned  spd_lim_en :  1;   // (bit  28)    Habilitação de limite de vel. (p/ uso do PID)
        unsigned  unused     :  3;   // (bits 29-31) 3 bits não utilizados (padding)
    } config;
    uint32_t status = 0;            // Leitura/Escrita simuntânea do conjunto de variáveis. 
} TMovStat;


typedef struct {
    volatile uint32_t tick_cntr = 0;  // Contador de ticks do Motor
    volatile uint32_t tick_last = 0;  // Timestamp do último tick do Motor
    volatile uint32_t tick_time = 0;  // Tempo entre ticks do Motor
    volatile uint16_t speed     = 0;  // Velocidade instantânea (em mm/s)
    volatile uint8_t  overflow  = 0;  // Núm. de overflows do contador
    volatile uint8_t  new_speed = 0;  // Atualização da vel. instantânea
} TEncCtrl;


typedef struct {
    float    Kp    = 0.50;      // Constante Proporcional
    float    Ki    = 0.00;      // Constante Integral
    float    Kd    = 0.00;      // Constante Diferencial
    uint32_t eval_t =  50;      // Intervalo entre cálculos (em ms)

    int16_t  target = 0;        // Valor de convergência
    int16_t  input  = 0;        // Valor de entrada
    int16_t  output = 0;        // Valor de saída
    int16_t  error  = 0;        // Erro calculado
    float    P_val = 0.0;       // Variação proporcional
    float    I_val = 0.0;       // Variação integral
    float    D_val = 0.0;       // Variação diferencial
    uint32_t last_t =  0;       // Timestamp do último cálculo
} TPidCtrl;


/* ******************************************************************* */
/* *** Variáveis globais e instanciações ***************************** */

RF24 radio(RADIO_CE, RADIO_CS);  // Instância do rádio

RF24Network network(radio);      // Instância da rede 


// Variáveis globais compartilhadas:

uint8_t netw_channel;			// Canal de rádio da rede

uint16_t  base_node, this_node;  // End. do nó remoto (base) e deste nó

TRadioBuf rx_buffer, tx_buffer;  // Buffers para mensagens (Rx & Tx)

TasksTCtr tasks;                 // Contagem de tempo para execução de tarefas

TMotCtrl  motor;                 // Configuração dos motores (direção e PWM)

TEncCtrl  enc_A, enc_B;          // Dados gerados através do sensores ópticos

TPidCtrl  pid_A, pid_B;          // Dados para o controle individual de vel.

TMovStat  mov;                   // Variáveis para controle do movimento.

/* ******************************************************************* */
/* *** Protótipos das funções **************************************** */
//
// Obs: Este bloco não é necessário para compilação mas é útil como
//      referência durante o processo de desenvolvimento.

void     tasks_10ms( void );
void     tasks_100ms( void );
void     tasks_1000ms( void );
void     task_irq_encoder_a( void );
void     task_irq_encoder_b( void );
void     task_radio_Rx( void );
void     task_radio_Tx( void );
void     dispatch_msgs( void );
int8_t   write_msg_radio_buffer( TRadioBuf&, TRadioMsg& );
int8_t   read_msg_radio_buffer( TRadioBuf&, TRadioMsg& );
void     flush_radio_buffer( TRadioBuf& ); 
bool     is_radio_buffer_full( TRadioBuf& );
bool     is_radio_buffer_empty( TRadioBuf& );
uint8_t  get_node_addr( bool );
uint32_t get_volt_bat( bool );
uint32_t get_motor_status( void );
void     set_motor_status( uint32_t );
bool     is_motor_breaked( uint8_t );
void     set_rotation( uint32_t );
bool     is_rotating( void );
void     speed_tuning( uint8_t );
void     set_speed( uint32_t );
void     set_course( uint32_t );
void     set_network_channel( uint8_t );
void     soft_reset( void );

/* ******************************************************************* */
/* *** SETUP ********************************************************* */

void setup() {

    Serial.begin(115200);               // Para testes

    base_node = BASE_ADDRESS;           // Endereço do nó base

    // Inicialização dos pinos de endereçamento do rádio
    pinMode(RADIO_A0, INPUT_PULLUP);    // Endereço deste nó: bit 0
    pinMode(RADIO_A1, INPUT_PULLUP);    // Endereço deste nó: bit 1
    
    this_node = get_node_addr(true);    // Lê endereço deste nó e envia msg 'A'

    // Inicialização do pino do LED
    pinMode(LED, OUTPUT);               // Pino do LED como saída digital
    digitalWrite(LED, LOW);

    // Inicialização dos pinos de controle da Ponte H
    pinMode(HBRID_EN, OUTPUT);          // Habilita ponte H
    digitalWrite(HBRID_EN, HIGH);       // 
    
    pinMode(MTR_AIN1, OUTPUT);          // Bit 0 - Controle da ponte H do Motor A
    pinMode(MTR_AIN2, OUTPUT);          // Bit 1 - Controle da ponte H do Motor A
    pinMode(MTR_BIN1, OUTPUT);          // Bit 0 - Controle da ponte H do Motor B
    pinMode(MTR_BIN2, OUTPUT);          // Bit 1 - Controle da ponte H do Motor B
    pinMode(MTR_PWMA, OUTPUT);          // Sinal de PWM para controle  do Motor A
    pinMode(MTR_PWMB, OUTPUT);          // Sinal de PWM para controle  do Motor B
    
    set_motor_status( 0x0 );            // Motores parados e freio elétrico

    // Inicialização dos pinos de leitura dos encoders
    pinMode(IRQ_ENC_A, INPUT);          // Pino de interrupção para o Encoder A
    pinMode(IRQ_ENC_B, INPUT);          // Pino de interrupção para o Encoder B
    pinMode(IRQ_RADIO, INPUT);          // Leitura do sinal irq do módulo do rádio
    
    analogReference(INTERNAL);          // Referência dos ADCs -> 1.1V

    SPI.begin();                        // Inicializa a interface SPI

    radio.begin();                      // Inicializa o módulo de rádio 

	netw_channel = NETW_CHANNEL;
    radio.setChannel(netw_channel);     // Canal da rede de rádio ( 0 - 125 );
    radio.setPALevel(RF24_PA_MAX);      // Potência da transmissão em 0dB ( 1mW )
//  radio.setDataRate(RF24_250KBPS);    // O padrão é 1Mbps
    radio.setCRCLength(RF24_CRC_16);    // Comprimento do CRC: 8 ou 16 bits
    radio.enableDynamicPayloads();      // Habilita mensagens de tamanho dinâmico
    radio.setRetries(4,10);             // Reenvios (em HW): 4 * 250us = 1ms ; count: 10
    radio.setAutoAck(true);             // Autoack habilitado (feito em HW)
    radio.maskIRQ(1,1,0);               // Interrupção somente quando recebe pacotes

    // Inicialização da rede
    network.begin(this_node);

    // Inicialização dos buffers
    flush_radio_buffer( &rx_buffer );
    flush_radio_buffer( &tx_buffer );

    // Habilita int para o Encoder A no pino 2 do Arduino
    pinMode(IRQ_ENC_A, INPUT_PULLUP);
    attachInterrupt(0, task_irq_encoder_a, FALLING);

    // Habilita int para o Encoder B no pino 3 do Arduino
    pinMode(IRQ_ENC_B, INPUT_PULLUP);
    attachInterrupt(1, task_irq_encoder_b, FALLING); 
    
    // Habilita o controlador PID
    mov.config.ctr_pid_en = 1;
    
    // Habilita limitação no PWM (min e máx)
    mov.config.pwm_lim_en = 1;

    // Habilita limitação na velocidade (min e máx)
    mov.config.spd_lim_en = 1;

}


/* ******************************************************************* */
/* *** LOOP PRINCIPAL ************************************************ */

void loop() {
    
    network.update();            // Mantém a rede ativa
    tasks_10ms();                // Tarefas executadas a cada 10ms
    tasks_100ms();               // Tarefas executadas a cada 100ms
    tasks_1000ms();              // Tarefas executadas a cada 1000ms

}


/* ******************************************************************* */
/* *** FUNÇÕES (implementações) ************************************** */

/* *********************************************************************
 * Threads que devem ser executadas em intervalos de 10ms
 */
void tasks_10ms( void ) {

     if( (millis() - tasks.last_10ms) > 10 ){
        tasks.last_10ms = millis();
        
        task_radio_Rx();
        dispatch_msgs();
        task_radio_Tx();
        speed_tuning( MOTOR_A );
        speed_tuning( MOTOR_B );
    }
}

/* *********************************************************************
 * Threads que devem ser executadas em intervalos de 100ms
 */
void tasks_100ms( void ) {

    if( (millis() - tasks.last_100ms) > 100 ){
        tasks.last_100ms = millis();


    }
}

/* *********************************************************************
 * Threads que devem ser executadas em intervalos de 1000ms
 */
void tasks_1000ms( void ) {

    if( (millis() - tasks.last_1000ms) > 1000 ){
        tasks.last_1000ms = millis();

    
    }
}

/* *********************************************************************
 * Trata pedido de interrupção gerado pelo sensor óptico A
 */
void task_irq_encoder_a( void ){

    noInterrupts();

    // Se houve interrupção com o motor parado consiste em erro
    // introduzido pela folga nas engrenagens.
    if( !mov.config.moving ) {
        enc_A.speed = 0;
        return;
    } 

    uint32_t us = micros();
    uint32_t tt = us - enc_A.tick_last;
    enc_A.tick_last = us;
     
    // Erro, pois t_min ~ 3000us
    if( tt < 2500 ) return;

    if( is_rotating() && enc_A.overflow ){
        // Pára motor A com PWM_A <= 0x00
        uint32_t mstat = get_motor_status(false) & 0xFFFF00FF;
        // Se o Motor B também estiver parado...
        if( !(mstat & 0x0000FFFF) ){
            // Horário -> reverte Motor A
            if( mov.config.moving == B01 ){
                bitWrite(mstat, 18, !bitRead(mstat, 18));
                bitWrite(mstat, 19, !bitRead(mstat, 19));                  
            }
            // Anti-Horário -> reverte Motor B
            else {
                bitWrite(mstat, 16, !bitRead(mstat, 16));
                bitWrite(mstat, 17, !bitRead(mstat, 17));
            }
            mov.config.moving = B00;
        }
        set_motor_status(mstat);
    }

    enc_A.tick_cntr += 1;
    if( !enc_A.tick_cntr ) 
         enc_A.overflow += 1;

    // Pesos: t_i = 50% & t_i-1 = 50%
    enc_A.tick_time = (tt + enc_A.tick_time) >> 1;
      
    // Velocidade instantânea.
    enc_A.speed = (uint16_t)((WHEEL_DIAM * 3141592)/(WHEEL_TICKS * enc_A.tick_time));
    enc_A.new_speed += 1;
      
    interrupts();
}

/* *********************************************************************
 * Trata pedido de interrupção gerado pelo sensor óptico B
 */
void task_irq_encoder_b( void ){

    noInterrupts();

    // Se houve interrupção com o motor parado consiste em erro
    // introduzido pela folga das engrenagens.
    if( !mov.config.moving ){
        enc_B.speed = 0;
		return;
	}

    uint32_t us = micros();
    uint32_t tt = us - enc_B.tick_last;
    enc_B.tick_last = us;

    // Erro, pois t_min ~ 3000us
    if( tt < 2500 ) return;

    if( is_rotating() && enc_B.overflow ){
        // Pára motor B com PWM_B <= 0x00
        uint32_t mstat = get_motor_status(false) & 0xFFFFFF00;
        // Se o Motor A também estiver parado...
        if( !(mstat & 0x0000FFFF) ){
            // Horário -> reverte Motor A
            if( mov.config.moving == B01 ){
                bitWrite(mstat, 18, !bitRead(mstat, 18));
                bitWrite(mstat, 19, !bitRead(mstat, 19));                  
            }
            // Anti-Horário -> reverte Motor B
            else {
                bitWrite(mstat, 16, !bitRead(mstat, 16));
                bitWrite(mstat, 17, !bitRead(mstat, 17));
            }
            mov.config.moving = B00;
        }
        set_motor_status(mstat);
    }
      
    enc_B.tick_cntr += 1;
    
    if( !enc_B.tick_cntr ) 
        enc_B.overflow += 1;
      
    // Pesos: t_i = 50% & t_i-1 = 50%
    enc_B.tick_time = (tt + enc_B.tick_time) >> 1;

    // Velocidade instantânea.
    enc_B.speed = (uint16_t)((WHEEL_DIAM * 3141592)/(WHEEL_TICKS * enc_B.tick_time));
    enc_B.new_speed += 1;
      
    interrupts();
}

/* *********************************************************************
 * Lê mensagens do buffer de HW e as armazena no buffer de SW
 */
void task_radio_Rx( void ){

    if( !network.available() ) return;
    
    TRadioMsg msg;
    RF24NetworkHeader header;
    
    while( network.available() && !is_radio_buffer_full(&rx_buffer) ) {

        network.read(header, &msg.data, sizeof(msg.data));
        msg.type  = header.type;
        write_msg_radio_buffer(&rx_buffer, &msg);
    }
}

/* *********************************************************************
 * Encaminha mensagens do buffer de SW para transmissão
 */
void task_radio_Tx( void ){

    TRadioMsg msg;
    while ( !is_radio_buffer_empty(&tx_buffer) ){
        
        read_msg_radio_buffer( &tx_buffer, &msg );
        RF24NetworkHeader header(base_node, msg.type);
        network.write(header, &msg.data, sizeof(msg.data));
    }
}

/* *********************************************************************
 * Processa a primeira mensagem da fila do buffer de SW de entrada 
 * recebida do rádio e a encaminha seu tratador.
 */
void dispatch_msgs( void ){

    if( is_radio_buffer_empty(&rx_buffer) ) return;
        
    TRadioMsg msg;

    read_msg_radio_buffer( &rx_buffer, &msg );

    switch ( msg.type ){

        // Leitura do endereço do nó
        case 'A':   get_node_addr( true ); 
                    break;

        // Ajusta curso (rotação, sentido e velocidade)
        case 'C':   if( is_rotating() )
                         write_msg_radio_buffer( &rx_buffer, &msg );
                    else set_course( msg.data );
                    break;

        // Ajuste individual das vels. dos motores
        case 'S':   if( is_rotating() )
                         write_msg_radio_buffer( &rx_buffer, &msg );
                    else set_speed( msg.data );
                    break;

        // Rotação do robô
        case 'R' :  if( is_rotating() )
                         write_msg_radio_buffer( &rx_buffer, &msg );
                    else set_rotation( msg.data ); 
                    Serial.println(msg.data, HEX);
                    break;

        // Leitura/Config. direta dos params. dos motores
        case 'M' :  if( is_rotating() )
                         write_msg_radio_buffer( &rx_buffer, &msg );
                    else if( msg.data > 0xFFFFF ) 
                         get_motor_status( true );
                    else {
                         mov.config.ctr_pid_en = 0;
                         set_motor_status( msg.data );
                    }
                    break;

        // Leitura da tensão da bateria
        case 'V':   get_volt_bat( true ); 
                    break;

        // Reinicio o Arduino por SW
        case 'Z': soft_reset();
				  break;
				  
		// Altera o canal de rádio do robô
        case 'N': set_network_channel( (uint8_t)msg.data );
				  break;

        // Mensagens com tipos desconhecidos são devolvidas à base
        default: write_msg_radio_buffer( &tx_buffer, &msg );
    }

}

/* *********************************************************************
 * Insere uma mensagem na fila do buffer de SW de rádio
 */
int8_t write_msg_radio_buffer( TRadioBuf *buf, TRadioMsg *msg ){

    if( buf->tam == TAM_BUFFER ) return -1;   // Erro, buffer cheio
    buf->msg[buf->tail].type = msg->type;
    buf->msg[buf->tail].data = msg->data;
    if( ++buf->tail == TAM_BUFFER ) buf->tail = 0;
    buf->tam++;
    return ( TAM_BUFFER - buf->tam );        // Retorna: espaço restante
}

/* *********************************************************************
 * Lê e apaga uma mensagem da fila do buffer de SW de rádio
 */
int8_t read_msg_radio_buffer( TRadioBuf *buf, TRadioMsg *msg ){

    if( buf->tam == 0 ) return -1;           // Erro, buffer vazio
    msg->type = buf->msg[buf->head].type;
    msg->data = buf->msg[buf->head].data;
    if( ++buf->head == TAM_BUFFER ) buf->head = 0;
    buf->tam--;
    return buf->tam;                         // Retorna: espaço ocupado
}

/* *********************************************************************
 * (Re)Inicializa buffer de SW de rádio
 */
void flush_radio_buffer( TRadioBuf *buf ) {

    buf->head   = 0;
    buf->tail   = 0;
    buf->tam    = 0;
    buf->last_t = 0;
}

/* *********************************************************************
 * Verifica se o buffer de SW de rádio está cheio
 */
bool is_radio_buffer_full( TRadioBuf *buf ){

    return buf->tam == TAM_BUFFER ? true : false ;
}

/* *********************************************************************
 * Verifica se o buffer de SW de rádio está vazio
 */
bool is_radio_buffer_empty( TRadioBuf *buf ){

    return buf->tam == 0 ? true : false ;
}

/* *********************************************************************
 * Lê endereço de rede do dispositivo configurado via dip switch
 * Atende as mensagens tipo 'A'. Os dados enviados são ignorados.
 * Obs: Esta mensagem é enviada periodicamente pelo nó base e é usada
 *      como mensagem de controle para testar se o robô está ativo.
 * Se : radio_echo == TRUE -> Envia informação para o nó base
 */
uint8_t get_node_addr( bool radio_echo ){

    uint8_t address = 0xFF;
    address  = address << 1;
    address |= digitalRead(RADIO_A1);
    address  = address << 1;
    address |= digitalRead(RADIO_A0);
    address = ~address;

    if ( radio_echo ) {
        TRadioMsg msg;
        msg.type = 'A';
        msg.data = (uint32_t)address;
        write_msg_radio_buffer( &tx_buffer, &msg );
    }
    return address;
}

/* *********************************************************************
 * Leitura da tensão da bateria ( Divisor resistivo: Vcc / 10 )
 * Atende as mensagens tipo 'V'. Os dados da mensagem são ignorados.
 * Obs: 1) O conteúdo dos dados da mensagem é ignorado.
 *      2) O valor da tensão retornado está em milivolts.
 * Se : radio_echo == TRUE -> Envia informação para o nó base
 */
uint32_t get_volt_bat( bool radio_echo ) {

    uint8_t  n   = 10;
    uint32_t adc =  0;

    for (int i = 0; i < n; i++){
        adc += analogRead(VOLT_BAT);
        delayMicroseconds(100);
    }
    
    uint32_t volt = (uint32_t)((adc*1.1/1023.0)*1000.0);
    
    if ( radio_echo ) {
        TRadioMsg msg;
        msg.type = 'V';
        msg.data = volt;
        write_msg_radio_buffer( &tx_buffer, &msg );
    }
    
    return volt;
}

/* *********************************************************************
 * Leitura da configuração dos motores
 * Atende as mensagens tipo 'M' caso o valor passado na mensagem for
 * maior que 0xFFFFF ( algum bit acima do bit 19 está setado ).
 * Se : radio_echo == TRUE -> Envia informação para o nó base
 */
uint32_t get_motor_status( bool radio_echo ){

    if ( radio_echo ) {
        TRadioMsg msg;
        msg.type = 'M';
        msg.data = motor.status;
        write_msg_radio_buffer( &tx_buffer, &msg );
    }
    return motor.status;
}

/* *********************************************************************
 * Altera diretamente a configuração de direção e PWM dos motores
 * Atende as mensagens tipo 'M', com dados ocupando 20 bits:
 * PWM  Motor B = 8bits (bits 0-7)  => 0 a 255  (valores de 0x0 a 0xFF)
 * PWM  Motor A = 8bits (bits 8-15) => 0 a 255  (valores de 0x0 a 0xFF)
 * Dir. Motor B = 2bits (bits 16 e 17) => 0 a 3 (valores de 0x0 a 0x3)
 * Dir. Motor A = 2bits (bits 18 e 19) => 0 a 3 (valores de 0x0 a 0x3)
 * -> Os bits de direção obedecem à seguinte configuração: 
 *    10 -> Motores giram no sentido horário (frente arbitrada) 
 *    01 -> Motores giram no sentido anti-horário
 *    00 -> Condição de freio elétrico (com o GND)
 *    11 -> Condição de freio elétrico (com o Vcc)
 */
void set_motor_status( uint32_t data ) {

    motor.status = data;
    
    // Desabilita ponte H
    digitalWrite(HBRID_EN, LOW);

    // Escreve bits de direção na ponte H
    #ifdef MTR_REVERSE
      digitalWrite(MTR_AIN1, bitRead(motor.config.dir_motor_A, 1));
      digitalWrite(MTR_AIN2, bitRead(motor.config.dir_motor_A, 0));
    #else
      digitalWrite(MTR_AIN1, bitRead(motor.config.dir_motor_A, 0));
      digitalWrite(MTR_AIN2, bitRead(motor.config.dir_motor_A, 1));
    #endif
    digitalWrite(MTR_BIN1, bitRead(motor.config.dir_motor_B, 0));
    digitalWrite(MTR_BIN2, bitRead(motor.config.dir_motor_B, 1));
    
    if( motor.config.dir_motor_A == motor.config.dir_motor_B )
        mov.config.moving = B11; // 11b => movendo (sem rotação)

    if( is_motor_breaked(MOTOR_A) && is_motor_breaked(MOTOR_B) )
        mov.config.moving = B00; // 00b => parado (freio elétrico)
        
    if( !motor.config.pwm_motor_A && !motor.config.pwm_motor_B )
        mov.config.moving = B00; // 00b => parado (PWM = 0)

    // Escreve bits de PWM na ponte H
    analogWrite(MTR_PWMA, motor.config.pwm_motor_A);
    analogWrite(MTR_PWMB, motor.config.pwm_motor_B);

    // (re)Habilita ponte H
    digitalWrite(HBRID_EN, HIGH);
    
}

/* *********************************************************************
 * Informa se o motor está com o freio elétrico ativado
 */
 bool is_motor_breaked( uint8_t mtr ){

    if( mtr ) 
         return !(bitRead(motor.config.dir_motor_B,0) ^ bitRead(motor.config.dir_motor_B,1));
    else return !(bitRead(motor.config.dir_motor_A,0) ^ bitRead(motor.config.dir_motor_A,1));
}

/* *********************************************************************
 * Rotação do robô
 * Atende as mensagens tipo 'R', com dados ocupando 12 bits:
 * Ângulo     = 10bits (bits 0-9) => 0° a 1023° (valores de 0x0 a 0x3FF)
 * Sentido    = 1bit (bit 10) => 0 -> horário | 1 -> anti-horário
 * Referência = 1bit (bit 11) Se este bit estiver setado, o robô mudará a
 *                            referência se o grau for maior que 180° e 
 *                            então fará a rotação do ângulo restante.
 * Obs: 1) O valor do ângulo θ de rotação corresponde ao módulo de 360.
 *      2) O ângulo θ computado será múltiplo de 7,5° (hardcoded).
 *      3) Se o robô estiver parado a rotação acontederá em relação à 
 *         referência de "frente" previamente arbitrada.
 */
void set_rotation( uint32_t data ) {
    
    uint16_t deg = (uint16_t)(data & 0x3FF)%360;
    uint8_t  ctr = (uint8_t)((data >> 10) & 0x3);

    // Furos do motor que serão contados (7,5° por furo)
    uint8_t tks = (uint8_t)((deg * 10) / 75);

    // Ajusta ângulo para múltiplo de 7,5°
    deg = (uint16_t)((tks * 75) / 10);
    mov.config.angle = deg;

    // Conf. dos motores 
    uint32_t mstat = get_motor_status(false);
    
    // Se os motores estiverem parados
    if( !mov.config.moving ) {
        
        // Parado por PWM = 0? Arbitrado PWMA = PWMB = 31
        if( !(mstat & 0x0000FF00) && !(mstat & 0x000000FF) ){
            mstat |= 0x00001F1F;
        }

        // Parados por freio elétrico? Não existe sentido definido
        // então será dada referência a uma frente arbitrada.
        if( is_motor_breaked(MOTOR_A) && is_motor_breaked(MOTOR_B) ){
            mstat &= 0xFFF0FFFF; mstat |= 0x000A0000; 
        }
        mov.config.ctr_pid_en = 0;
		TRadioMsg msg; 
        msg.type = 'S'; msg.data = 0;
        write_msg_radio_buffer( &rx_buffer, &msg );
    }

    // Processa bit de "Referência"
    if( bitRead(ctr, 1) ){
        if( deg >= 180 ){
            deg -= 180;
            mov.config.angle = deg;
        }
        // Inverte sentido dos motores
        bitWrite(mstat, 16, !bitRead(mstat, 16));
        bitWrite(mstat, 17, !bitRead(mstat, 17));
        bitWrite(mstat, 18, !bitRead(mstat, 18));
        bitWrite(mstat, 19, !bitRead(mstat, 19));
    }
    
    // Só há rotação se ângulo > 0
    if( deg ) {
        
        // Processa bit de "Sentido"
        if( !bitRead(ctr, 0) ){
            // Horário -> inverte sentido do Motor A
            bitWrite(mstat, 18, !bitRead(mstat, 18));
            bitWrite(mstat, 19, !bitRead(mstat, 19));
            mov.config.moving = B01;
        }
        else {
            // Anti-horário -> inverte sentido do Motor B
            bitWrite(mstat, 16, !bitRead(mstat, 16));
            bitWrite(mstat, 17, !bitRead(mstat, 17));
            mov.config.moving = B10;
        }
       
        // Contadores zeram no processo de overflow
        noInterrupts();
          enc_A.tick_cntr = 0 - tks;
          enc_A.overflow  = 0;
          enc_B.tick_cntr = 0 - tks;
          enc_B.overflow  = 0;
        interrupts();        
    }
    // Inicia processo de rotação
    set_motor_status( mstat );     
}

/* *********************************************************************
 * Consulta variável de movimento e informa se um processo de rotação
 * está em andamento.
 */
bool is_rotating( void ) {

    return (bitRead(mov.config.moving,0) ^ bitRead(mov.config.moving,1));
}

/* *********************************************************************
 * Ajuste dinâmico e automático de velocidade do motor.
 * Obs: Em caso do motor estar em condição de freio elétrico, o movimento
 *      resultante será na direção arbitrada como "frente padrão".
 */
void speed_tuning( uint8_t mtr ){
    
    #define LIM_I   50.0    // Módulo do limite de pid->I_val
    #define LIM_D   50.0    // Módulo do limite de pid->D_val
    
    // PID desabilitado
    if( !mov.config.ctr_pid_en ) return;

    // Seleciona o motor e respectivos dados
    TEncCtrl *enc = mtr ? &enc_B : &enc_A;
    TPidCtrl *pid = mtr ? &pid_B : &pid_A;
    
    if( mtr ){
        // Condição de freio elétrico do Motor B
        if( is_motor_breaked(MOTOR_B) )
            motor.config.dir_motor_B = MOTOR_CW;
    }
    else {
        // Condição de freio elétrico do Motor A
        if( is_motor_breaked(MOTOR_A) )
            motor.config.dir_motor_A = MOTOR_CW;
    }

    // **** Implementação do controlador PID **** //
    
    float eval_time = (float)(millis() - pid->last_t);
    
    if( ((uint32_t)eval_time) < pid->eval_t )
        return;
    
    // Intervalo de avaliação do PID (em segundos)
    eval_time  /= 1000.0;
    pid->last_t = millis();

    uint16_t pwm_in  = 0;
    uint16_t pwm_out = 0;

    if( pid->target ){

        // Limita vels. máx e min, se habilitado.
        if( mov.config.spd_lim_en ){
            if     ( pid->target < SPD_MIN ) pid->target = SPD_MIN;
            else if( pid->target > SPD_MAX ) pid->target = SPD_MAX;
        }

        pid->input = enc->speed;
        pid->error = pid->target - pid->input;

        // Calcula valores de P, I e D e gera valor de saída
        pid->P_val  = pid->error * pid->Kp;
        pid->I_val += (pid->error * pid->Ki) * eval_time;
        pid->D_val  = (pid->output - pid->input) * pid->Kd / eval_time;
        
        // Limita o módulo de pid->I_val
        if     ( pid->I_val >  LIM_I ) pid->I_val =  LIM_I;
        else if( pid->I_val < -LIM_I ) pid->I_val = -LIM_I;
        
        // Limita o módulo de pid->D_val
        if     ( pid->D_val >  LIM_D ) pid->D_val =  LIM_D;
        else if( pid->D_val < -LIM_D ) pid->D_val = -LIM_D;
        
        pid->output = (int16_t)(pid->P_val + pid->I_val + pid->D_val);
                
        // **** Converte de mm/s para pwm e aplica ajustes aos motores **** //
        
        pwm_in  = mtr ? motor.config.pwm_motor_B : motor.config.pwm_motor_A;
        
        // Relação linear com compensação do estado de carga da bateria
        pwm_out = ((pwm_in * (pid->input + pid->output)) / pid->input) & 0x00FF;
        
        // Proporciona uma arrancada suave ( garantia de movimento -> PWM_MIN )
        if( pwm_out > (pwm_in + PWM_MIN) )
            pwm_out =  pwm_in + PWM_MIN;
        
        // Limita PWM máx e min, se habilitado
        if( mov.config.pwm_lim_en ){
            if     ( pwm_out < PWM_MIN ) pwm_out = PWM_MIN;
            else if( pwm_out > PWM_MAX ) pwm_out = PWM_MAX;
        }
    }
    else {
        pid->input  = 0;
        pid->error  = 0;
        pid->output = 0;
        pid->P_val  = 0;
        pid->I_val  = 0;
        pid->D_val  = 0;
    }

/* 
    Serial.print( mtr ? "Motor B =>" : "Motor A =>" );
    Serial.print(" Target: ");
    Serial.print( pid->target );

    Serial.print("\tPIDin: ");
    Serial.print( pid->input );

    Serial.print("\tPIDerr: ");
    Serial.print( pid->error );

    Serial.print("\tPIDout: ");
    Serial.print( pid->output );

    Serial.print("\tPWMi: ");
    Serial.print( pwm_in );
    Serial.print("\tPWMo: ");
    Serial.print( pwm_out );
    
    Serial.print("\tP: ");
    Serial.print( pid->P_val );
    Serial.print("  I: ");
    Serial.print( pid->I_val );
    Serial.print("  D: ");
    Serial.print( pid->D_val );

    Serial.print("\n");
// */

    uint32_t mstat = get_motor_status(false);

    // Ajusta o valor de PWM do respectivo motor.
    if( mtr ) 
        mstat &= 0xFFFFFF00;
    else {    
        mstat &= 0xFFFF00FF;
        pwm_out <<= 8;
    }
    mstat |= (uint32_t)pwm_out;
    set_motor_status(mstat);
}

/* *********************************************************************
 * Ajuste das velocidades individuais dos motores, em mm/s
 * Atende as mensagens tipo 'S', com dados ocupando 20 bits:
 * Velocidade Motor B = 10 bits (bits 0-9)   => 0 a 1023 mm/s
 * Velocidade Motor A = 10 bits (bits 10-19) => 0 a 1023 mm/s
 * Obs: O ajuste de velocidade implica no funcionamento do controlador
 *      PID ( implementado na função speed_tuning() ).
 */
void set_speed( uint32_t data ){
    
    // Habilita obrigatoriamente o controlador PID
    mov.config.ctr_pid_en = 1;
    
    // Alterações entram em vigor imediatamente.
    pid_A.last_t = millis() - pid_A.eval_t;
    pid_B.last_t = millis() - pid_B.eval_t;
    
    // Altera velocidade desejada.
    pid_A.target = (uint16_t)((data >> 10) & 0x03FF);
    pid_B.target = (uint16_t)(data & 0x03FF);    
}

/* *********************************************************************
 * Determina direção, rotação e velocidade
 * Atende as mensagens tipo 'C', com dados ocupando 22 bits:
 * Velocidade = 10bits (bits 0-9)   => 0 a 1023 mm/s (valores de 0x0 a 0x3FF)
 * Rotação    = 12bits (bits 10-21):
 * -> Ângulo     = 10bits (bits 10-19) => 0° a 1023° (valores de 0x0 a 0x3FF)
 * -> Sentido    = 1bit (bit 20) => 0 -> horário | 1 -> anti-horário
 * -> Referência = 1bit (bit 21) Se este bit estiver setado, o robô mudará a
 *                                referência se o grau for maior que 180° e 
 *                                então fará a rotação do ângulo restante.
 * Obs: 1) A velocidade informada na mensagem é aplicada a ambos os motores
 *         usando a função set_speed();
 *      2) Os dados referentes a rotação são processados pela função
 *         set_rotation() portanto a disposição dos bits 10 a 21 da mensagem
 *         são correspondentes aos bits 0 a 11 do parâmetro desta função.
 */
void set_course( uint32_t data ){
    
    uint32_t s_data = ((data & 0x3FF) << 10) | (data & 0x3FF);
    uint32_t r_data = (data >> 10) & 0xFFF;
    
    // Execução via mensagem em buffer de SW
    // => Motivo: Um comando precisa ser completamente processado
    //            antes da execução do próximo.
    
    TRadioMsg msg_s, msg_r; 

    msg_s.type = 'S'; msg_s.data = s_data;
    write_msg_radio_buffer( &rx_buffer, &msg_s );

    if( r_data ){
        msg_r.type = 'R'; msg_r.data = r_data;
        write_msg_radio_buffer( &rx_buffer, &msg_r );
    }
}

/* *********************************************************************
 * Altera o canal de rádio do robô ( intervalo de canais: 100 a 125 )
 */
void set_network_channel( uint8_t channel ){
	
    if( channel < 100 || channel > 125 )
        return;
    digitalWrite(LED, HIGH);
    netw_channel = channel;
    radio.setChannel(netw_channel);
    delay(10);
    digitalWrite(LED, LOW);
}

/* *********************************************************************
 * Reset do Arduino via software.
 */
void soft_reset(){

    // Jump para endereço zero:
    // => Mantém os estados dos registradores do HW.
    asm volatile ("  jmp 0");  

}

/* ****************************************************************** */
/* ****************************************************************** */
