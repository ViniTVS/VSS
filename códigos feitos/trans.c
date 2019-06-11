#include <stdint.h>
#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <inttypes.h>


typedef union {
  struct {
    uint8_t  pwm_motor_B;        // (bits 0-7)   8 bits: Valor do PWM do Motor B
    uint8_t  pwm_motor_A;        // (bits 8-15)  8 bits: Valor do PWM do Motor A
    uint8_t  dir_motor_B : 2,    // (bits 16-17) 2 bits: BIN1 e BIN2  da ponte H
             dir_motor_A : 2,    // (bits 18-19) 2 bits: AIN1 e AIN2  da ponte H
             ign1_4b     : 4;    // (bits 20-23) 4 bits não utilizados (padding)
    uint8_t  ign2_8b;            // (bits 24-31) 8 bits não utilizados (padding)
  } config;
  uint32_t status;           // Leitura/Escrita simuntânea do conjunto de variáveis.
} TMotCtrl;

int set_interface_attribs(int fd, int speed){
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount){
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}

//NUMERO MAGICO: 2678018048.


int main(){
    char *portname = "/dev/ttyUSB0";
    int fd;
    int wlen;
    TMotCtrl motor;
    motor.status = 1;

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200);
    //set_mincount(fd, 0);                /* set to pure timed read */

    /* simple output */
    unsigned char aaa = '\n';
    unsigned char magia[32];
    for (int i = 0; i < 31; i++)
        magia[i] = 'i';
    magia[32] = '\n';

    char sai = 'G';
    
    for(int i = 0; ;i++){
        wlen = write(fd, &magia, 8);
        if (wlen != 8) {
            printf("Error from write: %d, %d\n", wlen, errno);
        }
        else    {
            // printf("%" PRIu32 "\n", motor.status);
            printf("%c \n", sai);
            // write(fd, &aaa, 8);
        }
    }
    // tcdrain(fd);    /* delay for output */


    /* simple noncanonical input */
    /*
    do {
        unsigned char buf[80];
        int rdlen;

        rdlen = read(fd, buf, sizeof(buf) - 1);
        if (rdlen > 0) {
#ifdef DISPLAY_STRING
            buf[rdlen] = 0;
            printf("Read %d: \"%s\"\n", rdlen, buf);
#else /* display hex */
/*            unsigned char   *p;
            printf("Read %d:", rdlen);
            for (p = buf; rdlen-- > 0; p++)
                printf(" 0x%x", *p);
            printf("\n");
#endif
        } else if (rdlen < 0) {
            printf("Error from read: %d: %s\n", rdlen, strerror(errno));
        } else {  /* rdlen == 0 */
 /*           printf("Timeout from read\n");
        }               
        /* repeat read to get full message */
    // } while (1);

}