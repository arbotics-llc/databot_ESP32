/*
   This example demonstrate how to read dual Temperature value from databot2.0 Temp1 & Temp2 port
*/
#include<databot2.h>
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(3, LED_PIN, NEO_GRB + NEO_KHZ800);

OneWire oneWire(4);    //using one wire to read temperature from DS18b20
OneWire oneWire2(23);
DallasTemperature tempsensor1(&oneWire);
DallasTemperature tempsensor2(&oneWire2);

float temp1; //variables to store the temperature1 values
float temp2; //variables to store the temperature2 values

void setup() {

  Serial.begin(9600);

  RGBled.setPixelColor(0, RGBled.Color(0, 0, 255));  // LED indication to know that databot is ON
  RGBled.show();
}

void loop() {
  temp1 = getExternalTemperature(tempsensor1);
  temp2 = getExternalTemperature(tempsensor2);
  Serial.print("Temp1: ");
  Serial.print(temp1);
  Serial.print(" °C   Temp2: ");
  Serial.print(temp2);
  Serial.println(" °C");
  delay(100);
}
