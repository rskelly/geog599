
#include "minimu9v5.h"

#define BUFFER_SIZE 512
#define SERIAL_SIZE 255                           // Has to be a uint8_t

#define PWM_1       23                            // PWM output pin 1
#define LASER_SYNC  ((uint8_t) 2)                    // Laser sync pin

volatile uint64_t rangeTimes[BUFFER_SIZE];   // Time of next laser pulse on serial.
volatile uint16_t ranges[BUFFER_SIZE];      // Time of next laser pulse on serial.

volatile uint64_t rangeTimeIdx = 0;
volatile uint64_t rangeIdx = 0;
volatile uint64_t lastIdx = 0;

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
  analogWrite(PWM_1, 255);
  
  imuStatus = imu.init();


}

void loop() {

  static uint8_t buf[SERIAL_SIZE] = {'#', '!', 0};
  static uint16_t errors[BUFFER_SIZE] = {0};
  
  // The data start index is 3. 2 is where the packet length will go.
  uint64_t idx = 3;
  uint64_t errIdx = 0;

  if(!imuStatus) {
    collectIMU(buf, idx, errors, errIdx);
    collectRanges(buf, idx, errors, errIdx);
  } else {
    addError(imuStatus, errors, errIdx);
  }
  collectErrors(buf, idx, errors, errIdx);
  
  buf[2] = idx - 3;       // Set the number of int8_ts in the transmission. Excludes the three header int8_ts, including this count.
  
  Serial.write(buf, idx);

}

void collectErrors(uint8_t* buf, uint64_t& idx, uint16_t* errors, uint64_t& errIdx) {
  if(errIdx && idx < SERIAL_SIZE - 2) {
    buf[idx++] = 'E';
    uint64_t chIdx = idx;
    uint8_t iCount = 0;
    buf[idx++] = iCount;
    
    uint64_t i = 0;
    while(i < errIdx && iCount < 256 && idx < SERIAL_SIZE - 2) {
      idx = writeShort(buf, idx, errors[i]);
      ++i;
      ++iCount;
    }
    buf[chIdx] = iCount;
    errIdx = 0;
  }

}

void collectIMU(uint8_t* buf, uint64_t& idx, uint16_t* errors, uint64_t& errIdx) {

  static int16_t gyro[3] = {0};
  static int16_t accel[3] = {0};

  if(imuStatus) {
    addError(imuStatus, errors, errIdx);
    return;  
  }

  int err;
  uint64_t imuTime1 = micros();
  
  if((err = imu.getState(gyro, accel))) {

    //if(err != -1) // -1 indicates that a reading wasn't ready.
      addError(err, errors, errIdx);
    
  } else {

    if(idx < SERIAL_SIZE - 2) {
      
      buf[idx++] = 'I';
      uint64_t chIdx = idx;
      uint8_t iCount = 0;
      buf[idx++] = iCount;
  
      uint64_t imuTime0 = micros();
      
      // FUTURE: Loop over IMU records.
      while(iCount < 256 && idx < SERIAL_SIZE - 20) {
        idx = writeShorts(buf, idx, gyro, 3);
        idx = writeShorts(buf, idx, accel, 3);
        idx = writeULong(buf, idx, imuTime0 + (imuTime1 - imuTime0) / 2);
        ++iCount;
      }
      
      buf[chIdx] = iCount;
    }
  }  
}

void collectRanges(uint8_t* buf, uint64_t& idx, uint16_t* errors, uint64_t& errIdx) {

  // If there are ranges, collect them.
  if(rangeTimeIdx > rangeIdx)
    readRanges();
    
  // If there are ranges to write, and room to write them.
  if(lastIdx < rangeIdx && idx < SERIAL_SIZE - 2) {
    
    buf[idx++] = 'R';     // Header marker.
    uint64_t chIdx = idx;   // Save the index of the count slot (next line).
    uint64_t iCount = 0;  
    buf[idx++] = iCount;  // Number of ranges is initially zero.
    
    while(lastIdx < rangeIdx && iCount < 256 && idx < SERIAL_SIZE - 10) {
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

  static uint8_t laserBuf[BUFFER_SIZE];
  static uint64_t bufIdx = 0;
  static uint64_t readIdx = 0;

  // Grab the current available uint8_ts.
  int avail = Serial1.available();
  while(--avail > 0) {
    laserBuf[bufIdx % BUFFER_SIZE] = Serial1.read();
    ++bufIdx;
  }

  uint64_t i;

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

void addError(int err, uint16_t* errors, uint64_t& errIdx) {
  errors[errIdx] = err;
  errIdx = (errIdx + 1) % BUFFER_SIZE;
}


inline uint64_t writeULong(uint8_t* buf, uint64_t pos, uint64_t v) {
  for(int i = 0; i < 8; ++i)
    buf[pos++] = (v >> (7 - i) * 8) & 0xff;
  return pos;
}

inline uint64_t writeUShort(uint8_t* buf, uint64_t pos, uint16_t v) {
  buf[pos++] = (v >> 8) & 0xff;
  buf[pos++] = v & 0xff;
  return pos;
}

inline uint64_t writeShort(uint8_t* buf, uint64_t pos, int16_t v) {
  buf[pos++] = (v >> 8) & 0xff;
  buf[pos++] = v & 0xff;
  return pos;
}

inline uint64_t writeShorts(uint8_t* buf, uint64_t pos, int16_t* v, uint64_t len) {
  for(uint64_t i = 0; i < len; ++i) {
    buf[pos++] = (v[i] >> 8) & 0xff;
    buf[pos++] = v[i] & 0xff;
  }
  return pos;
}
