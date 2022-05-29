#ifndef __Arduino_ICM20948_H__
#define __Arduino_ICM20948_H__


/*
This library is modified by dharmik for integrating with databot after V2.5
 following changes are done  inside Arduino-ICM20948.cpp file
 changed i2c address to 68 from few places else it was not working as by defauld library dont have option to change the i2c address
 removing the blocking code inside  if imu is not detected. inside check_rc function. 
 makin check_rc to bool so that it returns false if error accures.
*/
/*************************************************************************
  Defines
*************************************************************************/

typedef struct {
  int i2c_speed;
  bool is_SPI;
  int cs_pin;
  int spi_speed;
  int mode;
  bool enable_gyroscope;
  bool enable_accelerometer;
  bool enable_magnetometer;
  bool enable_gravity;
  bool enable_linearAcceleration;
  bool enable_quaternion6;
  bool enable_quaternion9;
  bool enable_har;
  bool enable_steps;
  int gyroscope_frequency;
  int accelerometer_frequency;
  int magnetometer_frequency;
  int gravity_frequency;
  int linearAcceleration_frequency;
  int quaternion6_frequency;
  int quaternion9_frequency;
  int har_frequency;
  int steps_frequency;

} ArduinoICM20948Settings;

/*************************************************************************
  Class
*************************************************************************/

class ArduinoICM20948
{
  public:

    ArduinoICM20948();

    //void init(TwoWire *theWire = &Wire, JTICM20948Settings settings);
    bool init(ArduinoICM20948Settings settings);
    void task();

    bool gyroDataIsReady();
    bool accelDataIsReady();
    bool magDataIsReady();
    bool gravDataIsReady();
    bool linearAccelDataIsReady();
    bool quat6DataIsReady();
    bool euler6DataIsReady();
    bool quat9DataIsReady();
    bool euler9DataIsReady();
    bool harDataIsReady();
    bool stepsDataIsReady();

    void readGyroData(float *x, float *y, float *z);
    void readAccelData(float *x, float *y, float *z);
    void readMagData(float *x, float *y, float *z);
    void readGravData(float* x, float* y, float* z);
    void readLinearAccelData(float* x, float* y, float* z);
    void readQuat6Data(float *w, float *x, float *y, float *z);
    void readEuler6Data(float *roll, float *pitch, float *yaw);
    void readQuat9Data(float* w, float* x, float* y, float* z);
    void readEuler9Data(float* roll, float* pitch, float* yaw);
    void readHarData(char* activity);
    void readStepsData(unsigned long* steps_count);
};


#endif