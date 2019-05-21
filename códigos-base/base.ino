
#include <avr/pgmspace.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

/* ************************************************************************* */
/* *** Definições diversas ************************************************* */

// Habilita interrupções no Rx do rádio
// Comente para receber as msgs por polling
// #define INT_ON_RADIO

#define SSPEED        115200   // Velocidade da interface serial
#define SERIAL_ECHO        0   // Eco da msg recebida (Prod: 0 | Debug: 1) 

#define TAM_BUFFER        16   // Buffer de SW para o rádio
#define BASE_ADDRESS      00   // Base tem o endereço 7 (em octal)
#define NETW_CHANNEL     100   // Canal padrão de operação do rádio

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


// Transforma um número de 4 bits em hexadecimal para caractere ascii. 
#define hex2asc(a) (( a < 10 )  ? ( (a) + 0x30 ) : ( (a) + ('a'-10) ))

// Transforma caractere ascii em um número de 4 bits em hexadecimal.
#define asc2hex(a) (((a) < 'a') ? ( (a) - '0' )  : ( ((a) - 'a')+10) )

// Lê um nibble de um inteiro de 32 bits dado seu índice (0 a 7).
#define Nibble(a, i) ((unsigned)(((a) >> (28 - 4 * (i))) & 0xF))

/* ************************************************************************* */
/* *** Definições de estruturas de dados *********************************** */

typedef struct {
    uint8_t  robot;             // 1 byte  - Endereço do outro nó 
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
    uint32_t last_10ms   = 0;
    uint32_t last_500ms  = 0;
    uint32_t last_1000ms = 0;
} TasksTCtr;


typedef struct {
   uint8_t  max_fails   = 4;
   uint8_t  addr[3]     = { 01, 02, 03 };
   uint8_t  com_fail[3] = {  0,  0,  0 };
   uint32_t last_act[3] = {  0,  0,  0 };
   uint8_t  led[3][2]   = {{L1G,L1R},{L2G,L2R},{L3G,L3R}};

} RobotsTCtr;


/* ************************************************************************* */
/* *** Variáveis globais e instanciações *********************************** */

RF24 radio(RADIO_CE, RADIO_CS);  // Instância do rádio
RF24Network network(radio);      // Instância da rede

// Canal de rádio da rede
uint8_t netw_channel;

// Buffers para mensagens (Rx & Tx)
TRadioBuf rx_buffer, tx_buffer;  

// Contagem de tempo para execução de tarefas
TasksTCtr tasks;

RobotsTCtr robot;

/* ************************************************************************* */
/* *** Protótipos das funções ********************************************** */

void    tasks_10ms( void );
void    tasks_500ms( void );
void    task_radio_Rx( void );
void    task_radio_Tx( void );
int8_t  write_msg_radio_buffer( TRadioBuf&, TRadioMsg& );
int8_t  read_msg_radio_buffer( TRadioBuf&, TRadioMsg& );
void    flush_radio_buffer( TRadioBuf& ); 
bool    is_radio_buffer_full( TRadioBuf& );
bool    is_radio_buffer_empty( TRadioBuf& );
int8_t  get_robot_i( uint8_t );
int8_t  serial_to_radio( bool );
void    radio_to_serial( TRadioMsg );
void    dispatch_msgs( void );
uint8_t get_network_channel( void );
void    set_network_channel( uint8_t );
void    soft_reset( void );


/* ************************************************************************* */
/* *** SETUP *************************************************************** */

void setup() {

    // Liga alimentação do módulo de rádio
    pinMode(RADIO_PWR, OUTPUT);
    digitalWrite(RADIO_PWR, HIGH);

    // Define pinos dos LEDs como saída
    for( int i=4; i < 10; i++ )
        pinMode( i, OUTPUT );

    Serial.begin(SSPEED);               // Inicializa a interface serial
    
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
    network.begin(BASE_ADDRESS);

    // Inicialização dos buffers
    flush_radio_buffer( &rx_buffer );
    flush_radio_buffer( &tx_buffer );

    // Habilita int para o o módulo de rádio no pino 2 do Arduino
    /* Obs: Com a interrupção de rádio habilitada está ocorrendo o  */
    /*      atraso na leitura de uma mensagem: Motivo? Investigar.  */
    #ifdef INT_ON_RADIO
      attachInterrupt(0, task_radio_Rx, FALLING);
    #endif
    
    // Sinaliza inicialização via serial
    Serial.print("A;0;0\n");

}

/* ************************************************************************* */
/* *** LOOP PRINCIPAL ****************************************************** */

void loop() {
    
    network.update();     // Mantém a rede ativa
    tasks_10ms();         // Tarefas executadas a cada 10ms
    tasks_500ms();        // Tarefas executadas a cada 500ms
}

/* ************************************************************************* */
/* *** FUNÇÕES (implementações) ******************************************** */

void tasks_10ms( void ) {

     if( (millis() - tasks.last_10ms) > 10 ){
        tasks.last_10ms = millis();
        
        #ifndef INT_ON_RADIO
          task_radio_Rx();
        #endif
        dispatch_msgs();
        serial_to_radio(SERIAL_ECHO);
        task_radio_Tx();
    }
}


void tasks_500ms( void ) {

    if( (millis() - tasks.last_500ms) > 500 ){
        tasks.last_500ms = millis();

        TRadioMsg msg;
        msg.type = 'A';
        msg.data  = 0;

        for( int i=0; i < 3; i++ ){

            if( (millis() - robot.last_act[i]) < 1000 ){
                digitalWrite( robot.led[i][0], HIGH );
                digitalWrite( robot.led[i][1], LOW );
                robot.com_fail[i] = 0;
            }
            else {
                robot.com_fail[i] += 1;
                if( robot.com_fail[i] >= robot.max_fails ){
                    digitalWrite( robot.led[i][0], LOW );
                    digitalWrite( robot.led[i][1], HIGH );
                }
            }
            // Coloca uma mensagem tipo 'A' na fila
            if( !is_radio_buffer_full(&tx_buffer) && robot.com_fail[i] > 0){
                msg.robot = robot.addr[i];
                write_msg_radio_buffer(&tx_buffer, &msg);
            }
        }
    }
}


/* ******************************************************************
 * Lê mensagens do buffer de HW e as armazena no buffer de SW
 */
void task_radio_Rx( void ){

    #ifdef INT_ON_RADIO 
      noInterrupts();
    #endif
    
    TRadioMsg msg;
    RF24NetworkHeader header;
    
    while( network.available() && !is_radio_buffer_full(&rx_buffer) ) {

        network.read(header, &msg.data, sizeof(msg.data));
        msg.type  = header.type;
        msg.robot = header.from_node;

        int8_t index = get_robot_i(msg.robot);

        if( index >= 0 ) 
            robot.last_act[index] = millis();

        write_msg_radio_buffer(&rx_buffer, &msg);    
    }
    rx_buffer.last_t = millis();
    
    #ifdef INT_ON_RADIO
      interrupts();
    #endif
}

/* ******************************************************************
 * Encaminha mensagens do buffer de SW para transmissão
 */
void task_radio_Tx( void ){

    TRadioMsg msg;
    while ( !is_radio_buffer_empty(&tx_buffer) ){

        read_msg_radio_buffer( &tx_buffer, &msg );
        RF24NetworkHeader header(msg.robot, msg.type);
        network.write(header, &msg.data, sizeof(msg.data));

        tx_buffer.last_t = millis();
    }
}

/* ******************************************************************
 * Insere uma mensagem na fila do buffer de SW de rádio
 */
int8_t write_msg_radio_buffer( TRadioBuf *buf, TRadioMsg *msg ){

    if( buf->tam == TAM_BUFFER ) return -1;   // Erro, buffer cheio
    buf->msg[buf->tail].robot = msg->robot;
    buf->msg[buf->tail].type  = msg->type;
    buf->msg[buf->tail].data  = msg->data;
    if( ++buf->tail == TAM_BUFFER ) buf->tail = 0;
    buf->tam++;
    return ( TAM_BUFFER - buf->tam );        // Retorna: espaço restante
}

/* ******************************************************************
 * Lê e apaga uma mensagem da fila do buffer de SW de rádio
 */
int8_t read_msg_radio_buffer( TRadioBuf *buf, TRadioMsg *msg ){

    if( buf->tam == 0 ) return -1;           // Erro, buffer vazio
    msg->robot = buf->msg[buf->head].robot;
    msg->type  = buf->msg[buf->head].type;
    msg->data  = buf->msg[buf->head].data;
    if( ++buf->head == TAM_BUFFER ) buf->head = 0;
    buf->tam--;
    return buf->tam;                         // Retorna: espaço ocupado
}

/* ******************************************************************
 * (Re)Inicializa buffer de SW de rádio
 */
void flush_radio_buffer( TRadioBuf *buf ) {

    buf->head   = 0;
    buf->tail   = 0;
    buf->tam    = 0;
    buf->last_t = 0;
}

/* ******************************************************************
 * Verifica se o buffer de SW de rádio está cheio
 */
bool is_radio_buffer_full( TRadioBuf *buf ){

    return buf->tam == TAM_BUFFER ? true : false ;
}

/* ******************************************************************
 * Verifica se o buffer de SW de rádio está vazio
 */
bool is_radio_buffer_empty( TRadioBuf *buf ){

    return buf->tam == 0 ? true : false ;
}

/* ******************************************************************
 * Índice do endereço do robô do vetor de endereços
 */
int8_t get_robot_i( uint8_t addr ){

    int8_t i = (sizeof(robot.addr)/sizeof(addr))-1 ;
    while ( i >= 0 ) {
        if(robot.addr[i] == addr) break;
        i--;
    }
    return i;
}

/* ******************************************************************
 * Lê um string recebido na interface serial, transforma no padrão
 * de uma mensagem de rádio e coloca na fila de transmissão.
 * 
 * ATENÇÃO: 
 * No Monitor Serial, use a opção "Nova-linha" para transmitir o '\n'
 * 
 * FORMATO DA MENSAGEM:
 * [A-Z];[0-3];[1 a 8 carateres de um número em hexadecimal]'\n'
 * 
 * [A-Z] => 1 caractere: Tipo da mensagem;
 * [0-3] => 1 caractere: Endereço do robô de destino;
 * [1 a 8 caracteres] => Dado a ser enviado;
 *  ;    => separador de campos da mensagem
 *  '\n' => quebra de linha sinaliza final da mensagem.
 */
int8_t serial_to_radio( bool serial_echo ){

    if( !Serial.available() ) return -1;
    
    uint8_t  h, i, c = '\0'; 
    uint32_t m;
    
    uint8_t str[TAM_BUFFER];      // Buffer de entrada
    memset(str,'\0',TAM_BUFFER);  // Limpeza do buffer

    // Lê o string enviado via interface serial
    i = 0;
    while( Serial.available() && i < (TAM_BUFFER-1) ){
        c = Serial.read();
        if( c != '\n' ) str[i++] = c;
        else            break;
        delayMicroseconds(100);  // Para compensar a lentidão da serial
    }
    
    // A mensagem deve ter tamanho entre 5 e 12 caracteres
    if( i < 5 || i > 12 ) return -1;

    TRadioMsg msg;
    
    if( c == '\n' ){
            
        // Processa o tipo da mensagem
        str[0] = toupper( str[0] );
        if( (str[0] >= 'A' && str[0] <= 'Z') && str[1] == ';' )
             msg.type = str[0];
        else return -1;
            
        // Processa o endereço de destino
        h = asc2hex( str[2] );
        if( (h >= 00 && h <= 03) && str[3] == ';' )
             msg.robot = h;
        else return -1;

        // Processa o conteúdo da mensagem
        i = 4; m = 0;
        while( str[i] != '\0' ){
            m = m << 4;
            m |= ( asc2hex(tolower(str[i])) & 0xF );
            i++;
        }
        msg.data = m;

        // Retransmite msg recebida para a serial
        if( serial_echo ){
            i = 0;
            while( str[i] != '\0' )
                Serial.write(str[i++]);
            Serial.write('\n');
        }

        // Tipos de msg que precisam de tratamento no nó base
        if( msg.type == 'N' || msg.type == 'Z' )
             write_msg_radio_buffer(&rx_buffer, &msg);
        else write_msg_radio_buffer(&tx_buffer, &msg);
    }
    // Número de carateres lidos via serial
    return i;
}

/* *********************************************************************
 * Lê mensagem recebida pelo rádio, as converte em string no formato
 *   CVS (conforme abaixo) e a transmite pela interface serial.
 * 
 * FORMATO DO STRING:
 * [A-Z];[0-3];[1 a 8 carateres de um número em hexadecimal]'\n'
 * 
 * [A-Z] => 1 caractere: Tipo da mensagem;
 * [1-3] => 1 caractere: Endereço do robô de origem;
 * [1 a 8 caracteres] => Dado recebido;
 *  ;    => separador de campos da mensagem
 *  '\n' => quebra de linha sinaliza final da mensagem.
 */
void radio_to_serial( TRadioMsg msg ){
    
    uint8_t i = 0;
    
    Serial.print((char)msg.type);
    Serial.print(';');
    Serial.print(msg.robot);
    Serial.print(';');
    Serial.print(msg.data, HEX);
    Serial.print('\n');

}

/* *********************************************************************
 * Lê uma mensagem do buffer de SW de entrada do rádio (dos robôs) ou
 * recebida via serial (PC) a encaminha para o seu respectivo tratador.
 * Obs: Mensagens recebidas que não necessitem de tratador no nó base
 *      são encaminhadas diretamente à serial.
 */
void dispatch_msgs( void ){

    TRadioMsg msg;
    uint8_t   to_serial;

    if( !is_radio_buffer_empty(&rx_buffer) ) {

        to_serial = true;
        
        read_msg_radio_buffer( &rx_buffer, &msg );
        
        switch ( msg.type ){

            // Mensagem de controle
            case 'A': to_serial = false;
                      break;
                      
            // Lê/Altera o canal de rádio da rede
            case 'N': to_serial = false;
                      if( msg.data < 100 || msg.data > 125 )
                           get_network_channel( );
                      else set_network_channel( (uint8_t)msg.data );
                      break;
                      
            // Reinicio o Arduino por SW
            case 'Z': to_serial = false;
                      if( msg.robot == BASE_ADDRESS )
                           soft_reset();
                      else write_msg_radio_buffer(&tx_buffer, &msg);
                      break;

        }
        // Envia msg para interface Serial
        // Se to_serial = true -> nó base tem somente a função de gateway
        //                        transparente entre os robôs e a serial. 
        if( to_serial ) radio_to_serial( msg );
    }
}

/* *********************************************************************
 * Lê o canal de rádio da rede
 */
uint8_t get_network_channel( ) {
    
    TRadioMsg msg;
    msg.robot =  0;
    msg.type  = 'N';
    msg.data  = netw_channel;
    radio_to_serial( msg );
    return netw_channel;
}

/* *********************************************************************
 * Altera o canal de rádio da rede (base e robôs)
 */
void set_network_channel( uint8_t channel ) {
    
    netw_channel = channel;
    // Broadcast de troca de canal para os robôs
    for( uint8_t i=0; i < 3; i++ ){
        RF24NetworkHeader header(robot.addr[i], 'N');
        network.write(header, &netw_channel, sizeof(netw_channel));
    }
    delay(5);
    radio.setChannel(netw_channel); 
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
