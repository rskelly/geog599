#include <ADC.h>

#define ENC_OUT     A7                // Encoder OUT pin

ADC adc;

void setup() {

  // Set up the serial port for output.
  Serial.begin(921600);
  
  analogReference(DEFAULT);

  adc.setAveraging(1);
  adc.setResolution(16);
  adc.setConversionSpeed(ADC_CONVERSION_SPEED::ADACK_6_2);
  adc.setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
  
  // Input pin for encoder.
  pinMode(ENC_OUT, INPUT);
 
}

int out = 0;
void loop() {
  int x = adc.analogRead(ENC_OUT);
  if(x > out) out = x;
  Serial.println(out);
  delay(100);
}
