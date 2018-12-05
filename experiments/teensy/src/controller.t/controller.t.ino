#include <i2c_t3.h>

const int pwm1 = 23;        // PWM output pin 1
const int dir1 = 22;        // Direction output pin 1
const int encOut = 21;      // Encoder OUT pin
const int laserSync = 2;    // Laser sync pin
const byte gyroAddr = 0x6b; // Gyro I2C address.

volatile unsigned long rangeTime;   // Time of next laser pulse on serial.
volatile short range;               // Current laser range.
volatile unsigned long angleTime;   // Time of current angle reading.
volatile short angle;               // Current angle reading. 12 bits > 0 - 4096

volatile bool rangeUpdate;

void laserSyncISR() {
  // Get the pulse time.
  rangeTime = micros();
  rangeUpdate = true;
}

void setup() {
  sei();

  rangeUpdate = false;
  
  // Set the analog voltage reference.
  analogReference(DEFAULT);

  // Analog read precision.
  analogReadRes(12);
  
  // Set up the serial port for output.
  Serial.begin(115200);
  
  // Set up the serial port for the laser.
  Serial1.setRX(0);
  Serial1.setTX(1);
  Serial1.begin(115200);
  
  // Pins for the drive motor speed and direction.
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
  
  // Input pin for encoder.
  pinMode(encOut, INPUT);

  // Pin for laser sync
  pinMode(laserSync, INPUT);
  
  // Interrupts for laser pulse and encoder update.
  //attachInterrupt(digitalPinToInterrupt(laserSync), laserSyncISR, RISING);

  // Gyro
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, 400000);
  Wire.setDefaultTimeout(200000);
  
  // Configure gyro / accel
  Wire.beginTransmission(gyroAddr);
  Wire.send(0x11);
  Wire.send((0x08 << 4) | (0x00 << 2));
  Wire.send(0x16);
  Wire.send(0);
  Wire.send(0x10);
  Wire.send((0x08 << 4) | (0x00 << 2) | (0x00 << 1));
  Wire.send(0x12);
  Wire.send(0x01 << 3);
  Wire.endTransmission();
  
  // Start the motor.
  //analogWrite(pwm1, 255);
}

void loop() {

  static byte laserBuf[2];
  static byte imuBuf[12];
  
  if(true || rangeUpdate) {

    Serial1.readBytes(laserBuf, 2);
    if(laserBuf[1] & 0x80) {
      laserBuf[0] = laserBuf[1];
      while(!Serial1.available());
      laserBuf[1] = Serial1.read();
    }
    
    range = ((laserBuf[0] & 0x7f) << 7) | (laserBuf[1] & 0x7f);

    // Get the angle.
    angle = analogRead(encOut);
    // Get the angle time.
    angleTime = rangeTime;

    //Wire.beginTransmission(gyroAddr);
    //Wire.send(0x22); // Request gyro x low, will increment to all subsequent addresses.
    //Wire.endTransmission();
    //Wire.requestFrom(gyroAddr, (size_t) 12);
    //while(Wire.available() < 12);
    //Wire.read(imuBuf, (size_t) 12);
    
    static unsigned char buf[33] = {'#'};

    // Range stuff
    for(int i = 0; i < 8; ++i)
      buf[1 + i] = (char) ((rangeTime >> ((7 - i) * 8)) & 0xff);
    for(int i = 0; i < 2; ++i)
      buf[9 + i] = (char) ((range >> ((1 - i) * 8)) & 0xff);

    // Angle stuff
    for(int i = 0; i < 8; ++i)
      buf[11 + i] = (char) ((angleTime >> ((7 - i) * 8)) & 0xff);
    for(int i = 0; i < 2; ++i)
      buf[19 + i] = (char) ((angle >> ((1 - i) * 8)) & 0xff);

    // IMU stuff
    for(int i = 0; i < 12; ++i)
      buf[21 + i] = imuBuf[i];
    
    Serial.write(buf, 33);
    Serial.flush();
    
    rangeUpdate = false;
  }    
  
}
