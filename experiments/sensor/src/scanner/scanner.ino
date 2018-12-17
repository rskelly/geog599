#include <i2c_t3.h>
//#include <ADC.h>

//#include "minimu9v5.h"

#define BUFFER_SIZE 256

#define PWM_1       ((byte) 23)       // PWM output pin 1
#define DIR_1       ((byte) 22)       // Direction output pin 1
#define ENC_OUT     A7                // Encoder OUT pin
#define LASER_SYNC  ((byte) 2)        // Laser sync pin
//#define GYRO_ADDR   ((byte) 0x6b)     // Gyro I2C address.

volatile unsigned long rangeTimes[BUFFER_SIZE];   // Time of next laser pulse on serial.
volatile unsigned short ranges[BUFFER_SIZE];
volatile unsigned short angles[BUFFER_SIZE];      // Current angle reading. 12 bits > 0 - 4096
volatile size_t rangeIdx = 0;

//volatile unsigned long imuTime;

//ADC adc;

void setup() {

  // Set up the serial port for output.
  Serial.begin(115200);
  
  // Set up the serial port for the laser.
  Serial1.setTX(1);
  Serial1.setRX(0);
  Serial1.begin(921600);
  
  // Pins for the drive motor speed and direction.
  pinMode(PWM_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  
  // Input pin for encoder.
  pinMode(ENC_OUT, INPUT);

  // Pin for laser sync
  pinMode(LASER_SYNC, INPUT);
  
  // Set up ADC for angle encoder.
  /*
  adc.setReference(ADC_REFERENCE::REF_3V3, ADC_0);
  adc.setAveraging(1);
  adc.setResolution(12);
  adc.setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
  adc.setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  //adc.enableInterrupts(ADC_0);
  adc.startContinuous(ENC_OUT, ADC_0);
  */
  
  // Interrupts for laser pulse and encoder update.
  attachInterrupt(LASER_SYNC, laserSyncISR, FALLING); // Falls when the shot is taken.

  // Start the motor.
  analogWrite(PWM_1, 64);

}

void loop() {

  static byte buf[64] = {'#'};
  static size_t idx;

  // The buffer header is always 2 bytes.
  idx = 2;

  // If the range index has moved, there are measurements to send.
  if(rangeIdx > 0) {

    // Collect the ranges up to the current index. Returns the number of ranges to send.
    //noInterrupts();
    size_t ridx = rangeIdx;
    //interrupts();
    size_t rangeCount = 1;//laserRange(ridx);
    
    // Write the ranges.
    for(size_t i = ridx; i < (ridx + rangeCount); ++i) {
      size_t ii = i % BUFFER_SIZE;
      buf[idx++] = 'R';  
      idx = writeULong(buf, idx, rangeTimes[ii]);
      idx = writeUShort(buf, idx, ranges[ii]);
      idx = writeUShort(buf, idx, angles[ii]);
    }

  }
  
  if(idx > 2) {
    // The second element is the length of the whole buffer - 2, for the # and count itself
    buf[1] = idx - 2;
    // Write to serial.
    Serial.write(buf, idx);
    Serial.flush();
    delay(1);
  }

}

/*
void adc0_isr() {
  if(angleIdx < BUFFER_SIZE) {
    //angles[angleIdx] = adc.analogReadContinuous(ADC_0);
    //angleTimes[angleIdx] = micros();
    ++angleIdx;
  }
}
*/

/*
// Suggestion: request IMU update, use interrupt to receive it.
// Don't know how to do timing.
void triggerIMU() {
  Wire.beginTransmission(GYRO_ADDR);
  Wire.send(OUT_X_L); // Request gyro x low, will increment to all subsequent addresses.
  Wire.endTransmission();
  // Wait here?
  Wire.requestFrom(GYRO_ADDR, (size_t) 12);
}

void readIMU(byte* imuBuf) {
  while(Wire.available() < 12);
  Wire.read(imuBuf, (size_t) 12);
  imuTime = micros();
}
*/

/**
 * Interrupt handler for laser sync pin.
 */
void laserSyncISR() {
  
  // Get the range of the measured time.
  rangeTimes[rangeIdx % BUFFER_SIZE] = micros();
  
  // Get the current angle. There will be a ~10us delay here.
  angles[rangeIdx % BUFFER_SIZE] = analogRead(ENC_OUT);
  
  ++rangeIdx;
}

/**
 * Collect all laser ranges up to the range index.
 */
size_t laserRange(size_t ridx) {
  
  static byte laserBuf[BUFFER_SIZE * 2];  // The receive buffer. Each range is 2 bytes.
  static size_t rangeIdx0 = 0;            // The local buffer index.
  static size_t off = 0;                  // When an incomplete range is received, move it to the front of the buffer, and set offset to 1.
  
  size_t dif = ridx - rangeIdx0;      // The number of points to get to catch up to the index.

  // Read the packets from serial into the buffer.
  size_t rd = Serial1.readBytes(&laserBuf[off], dif * 2);

  // Convert bytes into ranges.
  for(size_t i = 0; i < rd / 2; ++i) {
    //if(laserBuf[i] & 0x80)
    //  ++i;
    ranges[rangeIdx0 % BUFFER_SIZE] = ((laserBuf[i * 2] & 0x7f) << 7) | (laserBuf[i * 2 + 1] & 0x7f);
  }
  
  // Set the local range index ahead by the number of ranges read.
  rangeIdx0 += dif;

  // Update the offset depending on whether an incomplete packet was read.
  if(off % 2) {
    laserBuf[0] = laserBuf[rd - 1];
    off = 1;
  } else {
    off = 0;
  }

  return rd / 2;
}

/**
 * Initialize the IMU over the I2C bus.
 */
 /*
void initIMU() {
  // Gyro
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, 400000);
  Wire.setDefaultTimeout(200000);
  
  // Configure gyro / accel
  Wire.beginTransmission(GYRO_ADDR);
  Wire.send(CTRL2_G);
  Wire.send((GDR_1660Hz << 4) | (GFS_245dps << 2));
  Wire.send(CTRL7_G);
  Wire.send(0);
  Wire.send(CTRL1_XL);
  Wire.send((ADR_1660Hz << 4) | (AFS_2g << 2) | (AAFB_400Hz << 1));
  Wire.send(CTRL3_C);
  Wire.send(AUTO_INC_ON << 3);
  Wire.endTransmission();
}
*/

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
