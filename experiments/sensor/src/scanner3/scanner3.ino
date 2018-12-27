#include "minimu9v5.h"

#define BUFFER_SIZE 512
#define SERIAL_SIZE 255               // Has to be a byte

#define PWM_1       ((byte) 23)       // PWM output pin 1
#define ENC_OUT     A7                // Encoder OUT pin
#define LASER_SYNC  ((byte) 2)        // Laser sync pin

volatile unsigned long rangeTimes[BUFFER_SIZE];   // Time of next laser pulse on serial.
volatile unsigned short ranges[BUFFER_SIZE];   // Time of next laser pulse on serial.
volatile unsigned short angles[BUFFER_SIZE];      // Current angle reading. 12 bits > 0 - 4096
volatile unsigned short adcValue;
volatile size_t rangeTimeIdx = 0;
volatile size_t rangeIdx = 0;
volatile size_t lastIdx = 0;

MinIMU9v5 imu;

void setup() {

  // Set up the serial port for output.
  Serial.begin(115200);//921600);

  // Set up the serial port for the laser.
  Serial1.begin(115200);//921600);
  Serial1.setTX(1);
  Serial1.setRX(0);

  imu.init();
  
  analogReference(DEFAULT);
  
  // Pins for the drive motor speed.
  pinMode(PWM_1, OUTPUT);
  
  // Pin for laser sync
  pinMode(LASER_SYNC, INPUT);
  
  // Interrupts for laser pulse and encoder update.
  attachInterrupt(LASER_SYNC, laserSyncISR, FALLING); // Falls when the shot is taken.

  // Start the motor.
  //analogWrite(PWM_1, 255);
}

void loop() {

  static byte buf[SERIAL_SIZE] = {'#', '!', 0};
  static int16_t gyro[3] = {0};
  static int16_t accel[3] = {0};
  static size_t idx;
  static size_t i;
  static size_t headChunk = 3;
  static size_t imuChunk = 20;      // Number of bytes required for IMU info.
  static size_t rangeChunk = 10;    // Number of bytes required for range info.
  static size_t rangeHeadChunk = 3; // Number of bytes required for range header info.
  
  unsigned long imuTime0;
  unsigned long imuTime1;
  size_t rhIdx;           // Index of the range count header.
  size_t rCount;          // Number of range items.
  bool hasImu;            // True if an IMU update is available.
  
  // If there are ranges, collect them.
  if(rangeTimeIdx > rangeIdx)
    laserRange();

  // Get the IMU state and times.
  imuTime0 = micros();
  if((hasImu = imu.getState(gyro, accel)))
    imuTime1 = micros();
  
  // The data start index is 3. 2 is where the packet length will go.
  idx = 3;

  // If there are ranges to write.
  if(lastIdx < rangeIdx) {
    buf[idx++] = 'R';   // Header marker.
    rhIdx = idx;        // Save the index of the count slot (next line).
    buf[idx++] = 0;     // Number of ranges.
    rCount = 0;
    while(lastIdx < rangeIdx) {
      i = lastIdx % BUFFER_SIZE;
      idx = writeUShort(buf, idx, ranges[i]);
      idx = writeULong(buf, idx, rangeTimes[i]);
      ++lastIdx;
      ++rCount;
      if(idx >= SERIAL_SIZE - headChunk - rangeHeadChunk - imuChunk)   // If there's only enough room for the IMU chunk, break out.
        break;
    }
    buf[rhIdx] = rCount; // Set the count of ranges.
  }

  if(hasImu) {
    buf[idx++] = 'I';
    idx = writeShorts(buf, idx, gyro, 3);
    idx = writeShorts(buf, idx, accel, 3);
    idx = writeULong(buf, idx, imuTime0 + (imuTime1 - imuTime0) / 2);
  }
    
  buf[2] = idx - 3;       // Set the number of bytes in the transmission. Excludes the three header bytes, including this count.
  Serial.write(buf, idx); 
}

/**
 * Interrupt handler for laser sync pin.
 */
void laserSyncISR() {

  // Get the range of the measured time.
  rangeTimes[rangeTimeIdx % BUFFER_SIZE] = micros();
  
  // Get the current angle. 
  angles[rangeTimeIdx % BUFFER_SIZE] = adcValue;
  
  ++rangeTimeIdx;
}

/**
 * Collect all laser ranges up to the range index.
 */
void laserRange() {

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
