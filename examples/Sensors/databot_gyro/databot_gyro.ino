/*
   This example demonstrate how to read Gyro value from databot2.0 IMU sensor
*/
#include<databot2.h>
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(3, LED_PIN, NEO_GRB + NEO_KHZ800);

float gy_X, gy_Y, gy_Z;  //variables to store the gyro values

ArduinoICM20948 IMU;   //IMU object initialized
// IMU configuration settings
ArduinoICM20948Settings icmSettings =
{
  .i2c_speed = 400000,                // i2c clock speed
  .is_SPI = false,                    // Enable SPI, if disable use i2c
  .cs_pin = 10,                       // SPI chip select pin
  .spi_speed = 7000000,               // SPI clock speed in Hz, max speed is 7MHz
  .mode = 1,                          // 0 = low power mode, 1 = high performance mode
  .enable_gyroscope = true,           // Enables gyroscope output
  .enable_accelerometer = false,       // Enables accelerometer output
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
  RGBled.setPixelColor(0, RGBled.Color(0, 0, 255));  // LED indication to know that databot is ON
  RGBled.show();
}

void loop() {
  IMU_read_gyro(IMU, gy_X, gy_Y, gy_Z);
  Serial.print("Gyro X:");
  Serial.print(gy_X);
  Serial.print("  Gyro Y:");
  Serial.print(gy_Y);
  Serial.print("  Gyro Z:");
  Serial.println(gy_Z);
  delay(100);
}
