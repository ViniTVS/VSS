typedef union {
  struct {
    uint8_t  pwm_motor_B;        // (bits 0-7)   8 bits: Valor do PWM do Motor B
    uint8_t  pwm_motor_A;        // (bits 8-15)  8 bits: Valor do PWM do Motor A
    uint8_t  dir_motor_B : 2,    // (bits 16-17) 2 bits: BIN1 e BIN2  da ponte H
             dir_motor_A : 2,    // (bits 18-19) 2 bits: AIN1 e AIN2  da ponte H
             addr        : 4;    // (bits 20-23) 4 bits não utilizados (padding)
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
        uint16_t  id    :4,
                  chr   :8,
                  pad   :4;
        Data      data;
    } conf;
    uint32_t  stats = 0;
} TRadioMsg;

/*
typedef union  {
    struct {
        uint32_t  id    :4,
                  chr   :8,
                  pad   :3,
                  data1 :8,
                  data2 :9;
    } conf;
    uint32_t  stats = 0;
} TRadioMsg;
*/

TMotCtrl motor;                            //Variavel global que contra o motor.
TRadioMsg msg;                             // Mensagem que está sendo executada.

int final_time = 0;                            //tempo p/ ser percorrido x cm
int act_time = 0;                              //tempo percorrido no momento


     
void calcula_tempo (void) {
  final_time = msg.conf.data.data1 / msg.conf.data.data2;  
}

uint8_t mm_to_pwm (void){
   uint8_t saida;
   saida = 159*msg.conf.data.data2/500;
   return saida;
}

void translate_msg_in (void){
    switch (msg.conf.chr){
        case 'M':
            
            break;
        case 'R':
                    
            break;
        case 'S':
            motor.config.dir_motor_A = 0b10;
            motor.config.dir_motor_B = 0b10;
            motor.config.pwm_motor_A = mm_to_pwm();
            motor.config.pwm_motor_B = mm_to_pwm();

            break;
        case 'P':
            motor.config.dir_motor_A = 0b00;
            motor.config.dir_motor_B = 0b00;
            break;
        case 'W':
                    
            break;
        case 'L':
                    
            break;
        case 'D':
//            motor.config. 
            break;
        default:
            break;

        motor.config.addr = msg.conf.id;  // transfiro o end. da msg p/ o do 'motor'

//        msg.conf.dado.chave = 10;
    }
}

void set_motor_status( TRadioMsg state){

}