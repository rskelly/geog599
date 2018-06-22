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
uint8_t asd_addr = 0b01001000; // Used when addr -> ground
int16_t val;
uint8_t writeBuf[3];
uint8_t readBuf[2];

const float vps = 360.0 / ((5.23 / 6.114) * 32767); // volts per step (derees); a2d's volt scale 
                                                    // goes up to ±6.144V; pi's output is 
                                                    // tied to the input
int quit = 0;
int chainLen = 15;
double chain[15];
double fval;
double mean;
int chainIdx;

void sigintHandler(int p) {
  quit = 1;
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

  writeBuf[0] = 1; 
  writeBuf[1] = 0b11000000; // 15-8: [8] single shot; [11:9] gain; [14:12] ain#; [15] operational status
  writeBuf[2] = 0b10000101; // 7-0: [7:5] data rate; [4:0] comparators

  if(write(fd, writeBuf, 3) != 3) {
    perror("Write to register 1");
    exit(1);
  }

  sleep(1);

  readBuf[0] = 0;
  if(write(fd, readBuf, 1) != 1) {
    perror("Write register select");
    exit(-1);
  }

  chainIdx = 0;

  while(!quit) {

    if(read(fd, readBuf, 2) != 2) {
      perror("Read");
      exit(-1);
    }

    val = readBuf[0] << 8 | readBuf[1];

    //printf("Raw value %d\n", val);

    if(val < 0) val = 0;

    chain[chainIdx % chainLen] = val * vps;

    if(chainIdx >= chainLen) {
      mean = 0;
      double wt = 0;
      for(int i = chainIdx; i < chainIdx + chainLen; ++i) {
        double wt0 = (double) (chainLen - (i - chainIdx)) / chainLen * 10;
        mean += chain[i % chainLen] * wt0;
        wt += wt0;
      }
      fval = mean / wt;
      printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
      printf("Rotation %07.3f", fval);
    }

    ++chainIdx;
    //sleep(1);

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

