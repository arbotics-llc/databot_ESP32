/*
   This example demonstrate how to read Humidity value from databot2.0 SHTC3 sensor
*/
#include<databot2.h>
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(3, LED_PIN, NEO_GRB + NEO_KHZ800);
SHTC3 mySHTC3;   // Declare an instance of the SHTC3 class

float Humidity;  //variables to store the Humidity values

void setup() {

  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  /*
      humidity sensor needs little time to get worm up around 5 sec
  */
  mySHTC3.begin();
  mySHTC3.setMode(SHTC3_CMD_CSE_RHF_LPM);  // added from V1 code
  mySHTC3.sleep(true);

  RGBled.setPixelColor(0, RGBled.Color(0, 0, 255));  // LED indication to know that databot is ON
  RGBled.show();
}

void loop() {

  mySHTC3.update();
  mySHTC3.sleep(true);
  Humidity = mySHTC3.toPercent();
  Serial.print("Humidity: ");
  Serial.print(Humidity);
  Serial.println(" %RH");
  delay(100);
}
