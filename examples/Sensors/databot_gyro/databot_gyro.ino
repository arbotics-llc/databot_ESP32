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

MPU9250 IMU1;          // NEW databot has MPU9250 instead of ICM20948

void setup() {

  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  RGBled.setPixelColor(0, RGBled.Color(0, 0, 255));  // LED indication to know that databot is ON
  RGBled.show();
  check_which_IMU();   // it will check that IMU is MPU9250(new) or ICM-20948(old)
  if (new_IMU) //MPU9250
  {
    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_1000HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    if (!IMU1.setup(MPU9250_ADD, setting)) { // change to your own address
      Dprintln("MPU9250 connection failed");
    }
  }
  else //ICM-20948
  {
    IMU.init(icmSettings);
  }
}

void loop() {
  if (new_IMU)
  {
    IMU_read_gyro(IMU1, gy_X, gy_Y, gy_Z);
  }
  else
  {
    IMU_read_gyro(IMU, gy_X, gy_Y, gy_Z);
  }
  Serial.print("Gyro X:");
  Serial.print(gy_X);
  Serial.print("  Gyro Y:");
  Serial.print(gy_Y);
  Serial.print("  Gyro Z:");
  Serial.println(gy_Z);
  delay(100);
}

void check_which_IMU()
{
  //  byte address = MPU9250_ADD;
  byte error;
  Wire.beginTransmission(MPU9250_ADD);
  error = Wire.endTransmission();
  if (error == 0)
  {
    new_IMU = true;
    Dprintln("new IMU");
    return;
  }

  byte error1;
  Wire.beginTransmission(ICM20948_ADD);
  error1 = Wire.endTransmission();
  if (error1 == 0)
  {
    new_IMU = false;
    Dprintln("old IMU");
    return;
  }
}
