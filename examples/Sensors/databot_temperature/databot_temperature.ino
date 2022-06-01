/*
   This example demonstrate how to read Temperature value from databot2.0 Temp1 port
*/
#include<databot2.h>
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(3, LED_PIN, NEO_GRB + NEO_KHZ800);

OneWire oneWire(4);    //using one wire to read temperature from DS18b20
DallasTemperature tempsensor1(&oneWire);

float temp1; //variables to store the temperature values

void setup() {

  Serial.begin(9600);

  RGBled.setPixelColor(0, RGBled.Color(0, 0, 255));  // LED indication to know that databot is ON
  RGBled.show();
}

void loop() {
  temp1 = getExternalTemperature(tempsensor1);
  Serial.print("Temp1: ");
  Serial.print(temp1);
  Serial.println(" °C");
  delay(100);
}
