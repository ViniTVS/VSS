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

TMotCtrl motor;                            //Variavel global que contra o motor.
TRadioMsg msg;                             // Mensagem que está sendo executada.

void translate_msg_in (TRadioMsg ent){
    switch (ent.chr){
        case 'M':
            
            break;
        case 'R':
                    
            break;
        case 'S':
                    
            break;
        case 'P':
            motor.dir_motor_A = 0b00;
            motor.dir_motor_B = 0b00;
            break;
        case 'W':
                    
            break;
        case 'L':
                    
            break;
        case 'D':
                    
            break;
        default:
            break;
    }
}

void set_motor_status( TRadioMsg state){

}