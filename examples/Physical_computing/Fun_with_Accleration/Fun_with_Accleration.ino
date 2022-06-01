/*
   This example demonstrate how to read accelerometer value from databot2.0 IMU sensor
   And based on the acclerometer values we will turn on different LED colors based on databot2.0 orientation
*/
#include<databot2.h>
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(3, LED_PIN, NEO_GRB + NEO_KHZ800);

float Ax,Ay,Az;  //variables to store the acceleration values

ArduinoICM20948 IMU;   //IMU object initialized
// IMU configuration settings
ArduinoICM20948Settings icmSettings =
{
  .i2c_speed = 400000,                // i2c clock speed
  .is_SPI = false,                    // Enable SPI, if disable use i2c
  .cs_pin = 10,                       // SPI chip select pin
  .spi_speed = 7000000,               // SPI clock speed in Hz, max speed is 7MHz
  .mode = 1,                          // 0 = low power mode, 1 = high performance mode
  .enable_gyroscope = false,           // Enables gyroscope output
  .enable_accelerometer = true,       // Enables accelerometer output
  .enable_magnetometer = false,        // Enables magnetometer output // Enables quaternion output
  .enable_gravity = false,             // Enables gravity vector output
  .enable_linearAcceleration = false,  // Enables linear acceleration output
  .enable_quaternion6 = false,         // Enables quaternion 6DOF output
  .enable_quaternion9 = false,         // Enables quaternion 9DOF output
  .enable_har = false,                 // Enables activity recognition
  .enable_steps = false,               // Enables step counter
  .gyroscope_frequency = 5,           // Max frequency = 225, min frequency = 1
  .accelerometer_frequency = 1,       // Max frequency = 225, min frequency = 1
  .magnetometer_frequency = 5,        // Max frequency = 70, min frequency = 1
  .gravity_frequency = 1,             // Max frequency = 225, min frequency = 1
  .linearAcceleration_frequency = 1,  // Max frequency = 225, min frequency = 1
  .quaternion6_frequency = 50,        // Max frequency = 225, min frequency = 50
  .quaternion9_frequency = 50,        // Max frequency = 225, min frequency = 50
  .har_frequency = 50,                // Max frequency = 225, min frequency = 50
  .steps_frequency = 50               // Max frequency = 225, min frequency = 50
};

void setup() {

  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  IMU.init(icmSettings);
}

void loop() {
  IMU_read_accl(IMU, Ax, Ay, Az);
  Serial.print("ax "); Serial.print(Ax); Serial.print(" ay "); Serial.print(Ay); Serial.print(" az "); Serial.println(Az);
  if (Ax >= 7) {
    RGBled.setPixelColor(0, RGBled.Color(255, 0, 0));
    RGBled.show();
    RGBled.setPixelColor(1, RGBled.Color(255, 0, 0));
    RGBled.show();
    RGBled.setPixelColor(2, RGBled.Color(255, 0, 0));
    RGBled.show();
  } else if (Ax <= -7) {
    RGBled.setPixelColor(0, RGBled.Color(51, 255, 51));
    RGBled.show();
    RGBled.setPixelColor(1, RGBled.Color(51, 255, 51));
    RGBled.show();
    RGBled.setPixelColor(2, RGBled.Color(51, 255, 51));
    RGBled.show();
  } else if (Ay >= 7) {
    RGBled.setPixelColor(0, RGBled.Color(204, 51, 204));
    RGBled.show();
    RGBled.setPixelColor(1, RGBled.Color(204, 51, 204));
    RGBled.show();
    RGBled.setPixelColor(2, RGBled.Color(204, 51, 204));
    RGBled.show();
  } else if (Ay <= -7) {
    RGBled.setPixelColor(0, RGBled.Color(51, 51, 255));
    RGBled.show();
    RGBled.setPixelColor(1, RGBled.Color(51, 51, 255));
    RGBled.show();
    RGBled.setPixelColor(2, RGBled.Color(51, 51, 255));
    RGBled.show();
  } else if (Az >= 7) {
    RGBled.setPixelColor(0, RGBled.Color(255, 204, 0));
    RGBled.show();
    RGBled.setPixelColor(1, RGBled.Color(255, 204, 0));
    RGBled.show();
    RGBled.setPixelColor(2, RGBled.Color(255, 204, 0));
    RGBled.show();
  } else if (Az <= -7) {
    RGBled.setPixelColor(0, RGBled.Color(255, 255, 255));
    RGBled.show();
    RGBled.setPixelColor(1, RGBled.Color(255, 255, 255));
    RGBled.show();
    RGBled.setPixelColor(2, RGBled.Color(255, 255, 255));
    RGBled.show();
  }
  delay(100);
}
