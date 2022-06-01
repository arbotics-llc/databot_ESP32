/*
   This example demonstrate how to read noise value from databot2.0
*/
#include<databot2.h>
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(3, LED_PIN, NEO_GRB + NEO_KHZ800);

float Noise; //variables to store the noise values

void setup() {

  Serial.begin(9600);
  if (initMIC())
  {
    Serial.println("MIC initialization sucess");
  }
  else
  {
    Serial.println("MIC initialization fail");
  }

  RGBled.setPixelColor(0, RGBled.Color(0, 0, 255));  // LED indication to know that databot is ON
  RGBled.show();
}

void loop() {
  Noise = getLoudness();
  Serial.print("Noise: ");
  Serial.print(Noise);
  Serial.println(" dBa");
  delay(100);
}
