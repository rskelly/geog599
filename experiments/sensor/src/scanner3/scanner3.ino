
#include "minimu9v5.h"

#define BUFFER_SIZE 512
#define SERIAL_SIZE 255                           // Has to be a byte

#define PWM_1       23                            // PWM output pin 1
#define LASER_SYNC  ((byte) 2)                    // Laser sync pin

volatile unsigned long rangeTimes[BUFFER_SIZE];   // Time of next laser pulse on serial.
volatile unsigned short ranges[BUFFER_SIZE];      // Time of next laser pulse on serial.

volatile size_t rangeTimeIdx = 0;
volatile size_t rangeIdx = 0;
volatile size_t lastIdx = 0;
int imuStatus;                                    // If the IMU fails to initialize, set the error so the loop can deal with it.

MinIMU9v5 imu;

void setup() {

  // Set up the serial port for output.
  Serial.begin(115200);//921600);

  // Set up the serial port for the laser.
  Serial1.begin(115200);//921600);
  Serial1.setTX(1);
  Serial1.setRX(0);
  
  analogReference(DEFAULT);
  
  // Pins for the drive motor.
  pinMode(PWM_1, OUTPUT);
    
  // Pin for laser sync
  pinMode(LASER_SYNC, INPUT);
  
  // Interrupts for laser pulse and encoder update.
  attachInterrupt(LASER_SYNC, laserSyncISR, FALLING); // Falls when the shot is taken.

  // Start the motor.
  //analogWrite(PWM_1, 255);
  
  imuStatus = imu.init();

}

void loop() {

  static byte buf[SERIAL_SIZE] = {'#', '!', 0};
  static unsigned short errors[BUFFER_SIZE] = {0};
  
  // The data start index is 3. 2 is where the packet length will go.
  size_t idx = 3;
  size_t errIdx = 0;

  if(imuStatus) {
    
    addError(imuStatus, errors, errIdx);
  
  } else {

    int d = digitalRead(A4);
    addError(d, errors, errIdx);
    //collectIMU(buf, idx, errors, errIdx);
    //collectRanges(buf, idx, errors, errIdx);
  
  }
  
  collectErrors(buf, idx, errors, errIdx);
  
  buf[2] = idx - 3;       // Set the number of bytes in the transmission. Excludes the three header bytes, including this count.
  
  Serial.write(buf, idx);

}

void collectErrors(byte* buf, size_t& idx, unsigned short* errors, size_t& errIdx) {
  if(errIdx && idx < SERIAL_SIZE - 2) {
    buf[idx++] = 'E';
    size_t chIdx = idx;
    buf[idx++] = 0;
    size_t iCount = 0;
    size_t i = 0;
    while(i < errIdx && idx < SERIAL_SIZE - 2) {
      idx = writeShort(buf, idx, errors[i]);
      ++i;
      ++iCount;
    }
    buf[chIdx] = iCount;
    errIdx = 0;
  }

}

void collectIMU(byte* buf, size_t& idx, unsigned short* errors, size_t& errIdx) {

  static int16_t gyro[3] = {0};
  static int16_t accel[3] = {0};

  int err;
  size_t imuTime1, imuTime0 = micros();
  
  if((err = imu.getState(gyro, accel))) {

    addError(err, errors, errIdx);
    
  } else {

    addError(33, errors, errIdx);
    return;
    imuTime1 = micros();

    if(idx < SERIAL_SIZE - 2) {
      
      buf[idx++] = 'I';
      size_t chIdx = idx;
      buf[idx++] = 0;
      size_t iCount = 0;
      
      // FUTURE: Loop over IMU records.
      while(iCount == 0 && idx < SERIAL_SIZE - 20) {
        idx = writeShorts(buf, idx, gyro, 3);
        idx = writeShorts(buf, idx, accel, 3);
        idx = writeULong(buf, idx, imuTime0 + (imuTime1 - imuTime0) / 2);
        ++iCount;
      }
      
      buf[chIdx] = iCount;
    }
  }  
}
void collectRanges(byte* buf, size_t& idx, unsigned short* errors, size_t& errIdx) {

  // If there are ranges, collect them.
  if(rangeTimeIdx > rangeIdx)
    readRanges();
    
  // If there are ranges to write, and room to write them.
  if(lastIdx < rangeIdx && idx < SERIAL_SIZE - 2) {
    
    buf[idx++] = 'R';     // Header marker.
    size_t chIdx = idx;   // Save the index of the count slot (next line).
    size_t iCount = 0;  
    buf[idx++] = iCount;  // Number of ranges is initially zero.
    
    while(lastIdx < rangeIdx && idx < SERIAL_SIZE - 10) {
      idx = writeUShort(buf, idx, ranges[lastIdx % BUFFER_SIZE]);
      idx = writeULong(buf, idx, rangeTimes[lastIdx % BUFFER_SIZE]);
      ++lastIdx;
      ++iCount;
    }
    
    buf[chIdx] = iCount; // Set the count of ranges.
  }

}

/**
 * Collect all laser ranges up to the range index.
 */
void readRanges() {

  static byte laserBuf[BUFFER_SIZE];
  static size_t bufIdx = 0;
  static size_t readIdx = 0;

  // Grab the current available bytes.
  int avail = Serial1.available();
  while(--avail > 0) {
    laserBuf[bufIdx % BUFFER_SIZE] = Serial1.read();
    ++bufIdx;
  }

  size_t i;

  // Turn the buffer contents into ranges.
  if(bufIdx > 0) {
    while(readIdx < bufIdx - 1) {
      i = readIdx % BUFFER_SIZE;  
      if(!(laserBuf[i] & 0x80)) {
        ++readIdx;
      } else {
        ranges[rangeIdx % BUFFER_SIZE] = ((laserBuf[i] & 0x7f) << 7) | (laserBuf[i + 1] & 0x7f);
        ++rangeIdx;
        readIdx += 2;
      }
    }
  }
}

/**
 * Interrupt handler for laser sync pin.
 */
void laserSyncISR() {
  // Get the range of the measured time.
  rangeTimes[rangeTimeIdx % BUFFER_SIZE] = micros();
  ++rangeTimeIdx;
}

void addError(int err, unsigned short* errors, size_t& errIdx) {
  errors[errIdx] = err;
  errIdx = (errIdx + 1) % BUFFER_SIZE;
}


inline size_t writeULong(byte* buf, size_t pos, unsigned long v) {
  for(size_t i = 0; i < 8; ++i)
    buf[pos++] = (byte) ((v >> ((7 - i) * 8)) & 0xff);
  return pos;
}

inline size_t writeUShort(byte* buf, size_t pos, unsigned short v) {
  buf[pos++] = (v >> 8) & 0xff;
  buf[pos++] = v & 0xff;
  return pos;
}

inline size_t writeShort(byte* buf, size_t pos, short v) {
  buf[pos++] = (v >> 8) & 0xff;
  buf[pos++] = v & 0xff;
  return pos;
}

inline size_t writeShorts(byte* buf, size_t pos, short* v, size_t len) {
  for(size_t i = 0; i < len; ++i) {
    buf[pos++] = (v[i] >> 8) & 0xff;
    buf[pos++] = v[i] & 0xff;
  }
  return pos;
}
