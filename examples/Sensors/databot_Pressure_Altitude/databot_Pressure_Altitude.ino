/*
   This example demonstrate how to read Pressure and Altitude value from databot2.0 SHTC3 sensor
*/
#include<databot2.h>
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(3, LED_PIN, NEO_GRB + NEO_KHZ800);

float Pressure;  //variables to store the Pressure values
float Altitude;  //variables to store the Altitue values

void setup() {

  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
  }
  RGBled.setPixelColor(0, RGBled.Color(0, 0, 255));  // LED indication to know that databot is ON
  RGBled.show();
}

void loop() {
  Pressure = getPressure() * 10;// will get pressure in Kpa so *10 for hPa
  Altitude  = getAltitude();

  Serial.print("Pressure: ");
  Serial.print(Pressure);
  Serial.print(" hPa  Altitude: ");
  Serial.println(Altitude);
  delay(100);
}
