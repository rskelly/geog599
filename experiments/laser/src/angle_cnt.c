#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>    // read/write usleep
#include <stdlib.h>    // exit function
#include <inttypes.h>  // uint8_t, etc
#include <linux/i2c-dev.h> // I2C bus definitions
#include <signal.h>

int fd;
char asd_addr = 0b01001000; // Used when addr -> ground
int val1, val2;
char writeBuf[3];
char readBuf[2];

//const float vps = 360.0 / ((5.23 / 6.114) * 32767); // volts per step (derees); a2d's volt scale 
                                                    // goes up to ±6.144V; pi's output is 
                                                    // tied to the input
int quit = 0;
float fval;

void sigintHandler(int p) {
  quit = 1;
}

// fd = file handle
// addr ain#
void conf(int fd, int addr, int mode) {
  addr = 0b10000000 | (addr << 4) | mode;
  writeBuf[0] = 0b00000001;
  writeBuf[1] = addr;
  writeBuf[2] = 0b11100000;
  if(write(fd, writeBuf, 3) != 3) {
    perror("Write");
    exit(-1);
  }
  if(mode) {
    do {
      if(read(fd, writeBuf, 2) != 2) {
          perror("Read");
          exit(-1);
      }
    } while((writeBuf[0] & 0x80) == 0);
  } else {
    sleep(1);
  }
  readBuf[0] = 0;
  if(write(fd, readBuf, 1) != 1) {
    perror("Register select");
    exit(-1);
  }
}

int getit() {
  if(read(fd, readBuf, 2) != 2) {
      perror("Read conversion");
      exit(-1);
    }
    return readBuf[0] << 8 | readBuf[1];
}


int main() {

  signal(SIGINT, sigintHandler);

  printf("asd_addr: %d\n", asd_addr);

  if((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
    printf("Error: Couldn't open device. %d\n", fd);
    exit(1);
  }

  if(ioctl(fd, I2C_SLAVE, asd_addr) < 0) {
    printf("Error: Couldn't find device at address %d\n", asd_addr);
    exit(1);
  }

  //writeBuf2[0] = 0b00000001; // 7-0:  [0] config register
  //writeBuf2[1] = 0b11000000; // 15-0: [15] operational status; [14:12] ain#; [11:9] gain; [8] sigle shot
  //writeBuf2[2] = 0b10000000; // 7-0:  [7:5] data rate; [4:0] comparators

  conf(fd, 101, 1);
  val1 = getit();

  conf(fd, 100, 0);

  while(!quit) {

    // ref voltage
    //val1 = conf(fd, 101);

    // encoder
    val2 = getit();

    printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
    printf("Rotation %07.3f", (float) val2 / val1 * 360);

  }

  printf("\n");

  writeBuf[0] = 1;
  writeBuf[1] = 0b11000001; // powe down
  writeBuf[2] = 0b10000101;
  if(write(fd, writeBuf, 3) != 3) {
    perror("Write to register 1");
    exit(1);
  }

  close(fd);

  return 0;

}

