//#include <ADC.h>
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

//ADC adc;
//MinIMU9v5 imu;

void setup() {

  // Set up the serial port for output.
  Serial.begin(115200);//921600);
  
  // Set up the serial port for the laser.
  Serial1.begin(115200);//921600);
  Serial1.setTX(1);
  Serial1.setRX(0);

  //imu.init();
  
  analogReference(DEFAULT);

  /*
  adc.setAveraging(10);
  adc.setResolution(16);
  adc.setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
  adc.setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
  adc.enableInterrupts(ADC_0);
  adc.startContinuous(ENC_OUT, ADC_0);
  */
  
  // Pins for the drive motor speed.
  pinMode(PWM_1, OUTPUT);
  
  // Input pin for encoder.
  //pinMode(ENC_OUT, INPUT);
  
  // Pin for laser sync
  pinMode(LASER_SYNC, INPUT);
  
  // Start the motor.
  //analogWrite(PWM_1, 255);

  // Interrupts for laser pulse and encoder update.
  attachInterrupt(LASER_SYNC, laserSyncISR, FALLING); // Falls when the shot is taken.

}

void loop() {

  static byte buf[SERIAL_SIZE] = {'#'};
  static uint16_t gyro[3];
  static uint16_t accel[3];
  static size_t idx;
  static size_t i;
  
  if(rangeTimeIdx > rangeIdx)
    laserRange();

  idx = 2;

  unsigned short range;
  
  while(lastIdx < rangeIdx && idx < (SERIAL_SIZE - 14)) {
    i = lastIdx % BUFFER_SIZE;
    range = ranges[i];
    //imu.getState(gyro, accel);
    if(range > 0) {
      buf[idx++] = '!';
      buf[idx++] = 'R';
      idx = writeUShort(buf, idx, range);
      idx = writeUShort(buf, idx, gyro[1]); //angles[i]);
      idx = writeULong(buf, idx, rangeTimes[i]);
    }
    ++lastIdx;
  }

  if(idx > 2) {
    buf[1] = idx - 2;
    Serial.write(buf, idx);
  }
}

void adc0_isr() {
  adcValue = 1; //adc.analogReadContinuous(ADC_0);
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

  while(Serial1.available()) {
    laserBuf[bufIdx % BUFFER_SIZE] = Serial1.read();
    ++bufIdx;
  }

  size_t i;
  
  while(readIdx < bufIdx - 1) {
    i = readIdx % BUFFER_SIZE;  
    if(!(laserBuf[i] & 0x80)) {
      ++readIdx;
      continue;
    }
    ranges[rangeIdx % BUFFER_SIZE] = ((laserBuf[i] & 0x7f) << 7) | (laserBuf[i + 1] & 0x7f);
    readIdx += 2;
    ++rangeIdx;
  }
}

inline size_t writeULong(byte* buf, size_t pos, unsigned long v) {
  for(size_t i = 0; i < 8; ++i)
    buf[pos++] = (byte) ((v >> ((7 - i) * 8)) & 0xff);
  return pos;
}

inline size_t writeUShort(byte* buf, size_t pos, unsigned short v) {
  for(size_t i = 0; i < 2; ++i)
    buf[pos++] = (byte) ((v >> ((1 - i) * 8)) & 0xff);
  return pos;
}
