/*
   This example demonstrate how to read CO2 value from databot2.0 sensor and compute based on that value

   databot2.0 will turn on all its LED to RED color showing alert when CO2 value is above 1000 else it will be grean color
*/
#include<databot2.h>
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(3, LED_PIN, NEO_GRB + NEO_KHZ800);
SGP30 mySensor; //create an object of the SGP30 class
float CO2;  //variables to store the Co2 values

void setup() {

  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  //Initialize sensor
  if (mySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
    while (1);
  }
  //Initializes sensor for air quality readings
  //measureAirQuality should be called in one second increments after a call to initAirQuality
  mySensor.initAirQuality();

}

void loop() {

  //First fifteen readings will be
  //CO2: 400 ppm  TVOC: 0 ppb
  delay(1000); //Wait 1 second
  //measure CO2 and TVOC levels
  mySensor.measureAirQuality();
  CO2 = mySensor.CO2;
  Serial.print("CO2: ");
  Serial.print(CO2);
  Serial.println(" ppm");
  if (CO2 >= 1000)
  {
    // LED indication for high CO2 value
    RGBled.setPixelColor(0, RGBled.Color(255, 0, 0));
    RGBled.setPixelColor(1, RGBled.Color(255, 0, 0));
    RGBled.setPixelColor(2, RGBled.Color(255, 0, 0));
    RGBled.show();
  }
  else
  {
    // LED indication for safe CO2 value
    RGBled.setPixelColor(0, RGBled.Color(0, 255, 0));
    RGBled.setPixelColor(1, RGBled.Color(0, 255, 0));
    RGBled.setPixelColor(2, RGBled.Color(0, 255, 0));
    RGBled.show();
  }
}
