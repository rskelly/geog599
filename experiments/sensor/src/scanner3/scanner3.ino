#include <i2c_t3.h>

#define BUFFER_SIZE 64
#define SERIAL_SIZE 255               // Has to be a byte

#define PWM_1       ((byte) 23)       // PWM output pin 1
#define ENC_OUT     A7                // Encoder OUT pin
#define LASER_SYNC  ((byte) 2)        // Laser sync pin

volatile unsigned long rangeTimes[BUFFER_SIZE];   // Time of next laser pulse on serial.
volatile unsigned short ranges[BUFFER_SIZE];   // Time of next laser pulse on serial.
volatile unsigned short angles[BUFFER_SIZE];      // Current angle reading. 12 bits > 0 - 4096
volatile size_t rangeTimeIdx = 0;
volatile size_t rangeIdx = 0;
volatile size_t lastIdx = 0;

void setup() {

  // Set up the serial port for output.
  Serial.begin(921600);
  
  // Set up the serial port for the laser.
  Serial1.begin(921600);
  Serial1.setTX(1);
  Serial1.setRX(0);

  analogReference(DEFAULT);
  
  // Pins for the drive motor speed.
  pinMode(PWM_1, OUTPUT);
  
  // Input pin for encoder.
  pinMode(ENC_OUT, INPUT);
  
  // Pin for laser sync
  pinMode(LASER_SYNC, INPUT);
  
  // Start the motor.
  analogWrite(PWM_1, 64);

  // Interrupts for laser pulse and encoder update.
  attachInterrupt(LASER_SYNC, laserSyncISR, FALLING); // Falls when the shot is taken.

}

void loop() {

  static byte buf[SERIAL_SIZE] = {'#'};
  static size_t idx;
  static size_t i;
  
  if(rangeTimeIdx > rangeIdx)
    laserRange();

  idx = 2;
  
  while(lastIdx < rangeIdx && idx < (SERIAL_SIZE - 16)) {
    i = lastIdx % BUFFER_SIZE;
    buf[idx++] = 'R';
    idx = writeUShort(buf, idx, ranges[i]);
    idx = writeUShort(buf, idx, angles[i]);
    idx = writeULong(buf, idx, rangeTimes[i]);
    ++lastIdx;
  }

  if(idx > 2) {
    buf[1] = idx;
    Serial.write(buf, idx);
    delay(1);
  }
}

/**
 * Interrupt handler for laser sync pin.
 */
void laserSyncISR() {
  // Get the range of the measured time.
  rangeTimes[rangeTimeIdx % BUFFER_SIZE] = micros();
  
  // Get the current angle. There will be a ~10us delay here.
  angles[rangeTimeIdx % BUFFER_SIZE] = analogRead(ENC_OUT);
  
  ++rangeTimeIdx;
}

/**
 * Collect all laser ranges up to the range index.
 */
void laserRange() {
  static byte laserBuf[BUFFER_SIZE];
  
  size_t avail;
  byte a = 0, b;
  
  while((avail = Serial1.available()) >= 2) {  
    if(avail > BUFFER_SIZE) avail = BUFFER_SIZE;
    int rd = Serial1.readBytes(laserBuf, (avail / 2) * 2);
    for(int i = 0; i < rd; i += 2) {
      if(laserBuf[i + 1] & 0x80) {
        --i;  // TODO: Are we one ahead, or one behind?
        continue;
      }
      a = laserBuf[i];
      b = laserBuf[i + 1];
      ranges[rangeIdx % BUFFER_SIZE] = ((a & 0x7f) << 7) | (b & 0x7f);
      ++rangeIdx;
    }
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
