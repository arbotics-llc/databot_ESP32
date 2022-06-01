/*
   This example demonstrate how to read ambiant light value from databot2.0 APDS9960 sensor
*/
#include<databot2.h>
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(3, LED_PIN, NEO_GRB + NEO_KHZ800);

SparkFun_APDS9960 apds = SparkFun_APDS9960();   // light sensor object

uint16_t AmbLight;  //variables to store the ambiant light values

void setup() {

  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init()) {
    Serial.println("APDS-9960 initialization complete");
  } else {
    Serial.println("Something went wrong during APDS-9960 init!");
  }

  // Start running the APDS-9960 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {
    Serial.println("Light sensor is now running");
  } else {
    Serial.println("Something went wrong during light sensor init!");
  }
  RGBled.setPixelColor(0, RGBled.Color(0, 0, 255));  // LED indication to know that databot is ON
  RGBled.show();
}

void loop() {
  apds.readAmbientLight(AmbLight);
  Serial.print("Amb Light: ");
  Serial.print(AmbLight);
  Serial.println(" lux");
  delay(100);
}
