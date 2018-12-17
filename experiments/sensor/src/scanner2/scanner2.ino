#include <ADC.h>


#define PWM_1       23       // PWM output pin 1
#define LASER_SYNC  2        // Laser sync pin

volatile unsigned long rangeTime;   // Time of next laser pulse on serial.
volatile bool rangeUpdate;          // True if a range update is expected.

ADC adc;

void setup() {

  rangeUpdate = false;

  analogReference(DEFAULT);
  
  pinMode(PWM_1, OUTPUT); // Pin for the drive motor speed
  
  pinMode(LASER_SYNC, INPUT); // Pin for laser sync
  attachInterrupt(LASER_SYNC, laserSyncISR, FALLING); // Interrupt for laser pulse 
 
  interrupts();

  analogWrite(PWM_1, 64); // Start the motor.

  Serial.begin(115200); 
}

void loop() {
  Serial.println("xxx");
  return;
  if(rangeUpdate) {
    Serial.println(rangeTime);
    rangeUpdate = false;
    delay(1);
  }
}

void laserSyncISR() {
  rangeTime = micros();
  rangeUpdate = true;
}
