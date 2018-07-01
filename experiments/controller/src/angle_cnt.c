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
#include <math.h>

int fd;
char asd_addr = 0b01001000; // Used when addr -> ground.

//const float vps = 360.0 / ((5.23 / 6.114) * 32767); // volts per step (derees); a2d's volt scale 
                                                    // goes up to ±6.144V; pi's output is 
                                                    // tied to the input
int quit = 0;

void sigintHandler(int p) {
  quit = 1;
}

// fd = file handle
// addr ain#
void conf(int fd, int addr, int mode) {
  char writeBuf[3];
  char readBuf[2];
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
  char readBuf[2];
  if(read(fd, readBuf, 2) != 2) {
      perror("Read conversion");
      exit(-1);
    }
    return readBuf[0] << 8 | readBuf[1];
}

int main() {

  // Configure the interrupt handler.
  signal(SIGINT, sigintHandler);

  // Open the device with fd as the handle.
  if((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
    printf("Error: Couldn't open device. %d\n", fd);
    exit(1);
  }

  // Connect to the device.
  if(ioctl(fd, I2C_SLAVE, asd_addr) < 0) {
    printf("Error: Couldn't find device at address %d\n", asd_addr);
    exit(1);
  }

  // Configure the device for a continuous read of the input 
  // voltage on AN1.
  //conf(fd, 101, 0);

  // Save up some reads over about five seconds and average the volts.
  //float v = 0;
  //for(int i = 0; i < 500; ++i) {
  //  v += getit();
  //  usleep(10000);
 // }

  // The max voltage is the avg of those just read.
 // float max = v / 500;

  //printf("%f %f\n", max, max / 32676 * 6.144);
//exit(1);
  // Configure to read the angle continuously.
  conf(fd, 100, 0);

  float max = 0;
  int iters = 0;

  while(!quit) {

    // Read the encoder.
    float val = getit();

    if(val > max) max = val;
    printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
    printf("Rotation %07.3f", val / max * 360.0);

    //if(iters++ > 100000)
    //  max = 0;
    usleep(1);
  }

  printf("\n");

  char writeBuf[3];
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

