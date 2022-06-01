/*
   This example demonstrate how to read distance value from databot2.0

   This example is tested with arduino-esp32package V1.0.6 and also need following changes to be done inside that
   >Go to this path Arduino15\packages\esp32\hardware\esp32\1.0.6\libraries\Wire\src
   >Open file Wire.h 
   >Modify the line no.34  
   from   #define I2C_BUFFER_LENGTH 128 
   to     #define I2C_BUFFER_LENGTH 256  
   >save the changes 
*/
#include<databot2.h>
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(3, LED_PIN, NEO_GRB + NEO_KHZ800);

float distance; //variables to store the distance values
void setup() {

  Serial.begin(9600);
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);

  Wire.begin();
  Wire.setClock(400000);

  if (initTOF(true))
  {
    Serial.println("TOF initialization sucess");
  }
  else
  {
    Serial.println("TOF initialization fail");
  }

  RGBled.setPixelColor(0, RGBled.Color(0, 0, 255));  // LED indication to know that databot is ON
  RGBled.show();
}

void loop() {
  distance = getDistance();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" mm");
  delay(100);
}
