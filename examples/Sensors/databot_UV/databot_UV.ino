/*
   This example demonstrate how to read UV index value from databot2.0 UV sensor
*/
#include<databot2.h>
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(3, LED_PIN, NEO_GRB + NEO_KHZ800);

byte UV = 0; //variable use to store the UV index value

void setup() {
  Serial.begin(9600);
  RGBled.setPixelColor(0, RGBled.Color(0, 0, 255));  // LED indication to know that databot is ON
  RGBled.show();
}

void loop() {
  UV = getUV();
  Serial.print("UV: ");
  Serial.println(UV);
  delay(100);
}
