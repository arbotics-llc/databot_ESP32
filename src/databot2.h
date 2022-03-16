/*
Copyright aRbotics llc 2019
This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef DATABOT2_h
#define DATABOT2_h

#ifdef debug
#define Dprint(x) Serial.print(x)
#define Dprintln(x) Serial.println(x)
#else
#define Dprint(x)
#define Dprintln(x)
#endif

#include <WiFi.h>
#include <FS.h>
#include <SPIFFS.h>
#include "libs/ESPAsyncWebServer.h"
#include <EEPROM.h>

#include<Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "libs/ArduinoJson.h"
#include "libs/DallasTemperature.h"
#include "libs/SparkFun_SGP30_Arduino_Library.h"    // for C0
#include "libs/ICM_20948.h" // for IMU 
#define ICM_20948_ADD 0      // The value of the last bit of the I2C address.
#include "libs/SparkFun_APDS9960.h"
#include "libs/SparkFun_SHTC3.h"
#include "libs/Arduino_LPS22HB.h"
#include <driver/i2s.h>
#include "libs/arduinoFFT.h"
#include "libs/vl53l1_api.h"
#include "libs/Adafruit_NeoPixel.h"
#include "libs/Tone32.h"
#include "libs/Filter.h"


#define LED_PIN 2
#define UV_pin 34
#define BUZZER_PIN 32
#define BUZZER_CHANNEL 0
#define charge_state 14
#define rx_pin 16
#define tx_pin 17
#define batVTG 33
#define APDS9960_INT    26 // Needs to be an interrupt pin


//extern StaticJsonDocument<1000> doc;

#define SERVICE_UUID           "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_tx "0000ffe1-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_rx "0000ffe2-0000-1000-8000-00805f9b34fb"
extern std::string rxValue;
	


// define the number of bytes you want to access
#define EEPROM_SIZE 50

#define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())

/*************************************************************************************/
//MIC code
#define I2S_WS            19
#define I2S_SCK           5
#define I2S_SD            18
// size of noise sample
#define SAMPLES 1024
const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = SAMPLES;

#define OCTAVES 9
// our FFT data
static float real[SAMPLES];
static float imag[SAMPLES];
static arduinoFFT fft(real, imag, SAMPLES, SAMPLES);
static float energy[OCTAVES];
// A-weighting curve from 31.5 Hz ... 8000 Hz
static const float aweighting[] = { -39.4, -26.2, -16.1, -8.6, -3.2, 0.0, 1.2, 1.0, -1.1 };


void integerToFloat(int32_t * integer, float *vReal, float *vImag, uint16_t samples);
void calculateEnergy(float *vReal, float *vImag, uint16_t samples);
void sumEnergy(const float *bins, float *energies, int bin_size, int num_octaves);
float decibel(float v);
float calculateLoudness(float *energies, const float *weights, int num_octaves, float scale);
int initMIC();
float getLoudness();

/*************************************************************************************/


/*********************************************************************************/
// TOF code
// By default, this example blocks while waiting for sensor data to be ready.
// Comment out this line to poll for data ready in a non-blocking way instead.
#define USE_BLOCKING_LOOP

// Timing budget set through VL53L1_SetMeasurementTimingBudgetMicroSeconds().
#define MEASUREMENT_BUDGET_MS 50

// Interval between measurements, set through
// VL53L1_SetInterMeasurementPeriodMilliSeconds(). According to the API user
// manual (rev 2), "the minimum inter-measurement period must be longer than the
// timing budget + 4 ms." The STM32Cube example from ST uses 500 ms, but we
// reduce this to 55 ms to allow faster readings.
#define INTER_MEASUREMENT_PERIOD_MS 55



bool initTOF(bool range = false);      // init the TOF sensor pass true to set in long distance mode
float getDistance();				   // will measure the distance in mm

/*********************************************************************************/

void initBLE();    				 		// init the BLE server with service and characteristic UUIDs
bool isConnected(); 					// returns true if BLE device is connected else false
void advertising();						// will start advertising the BLE server
void sendPacket(const char *);    		// will send char packets over BLE
std::string getValues();           		// will receive the value over BLE and returns in string
byte getUV(); 							// will read the UV sensor and returns the uv index 
float getPressure();   					// will read the pressre and returns pressure value in Hpa
float getAltitude();					// will read the altitude based on pressure reading returns value in meter
void callback_Rx();

//DS18B20 external temperature sensor helper function
float getExternalTemperature(DallasTemperature &tempsensor);

// 
bool IMU_read(ICM_20948_I2C &imu,float &AX,float &AY,float &AZ,float &GX,float &GY,float &GZ,float &MX,float &MY,float &MZ,float &TMP);

bool IMU_read_accl(ICM_20948_I2C &imu,float &AX,float &AY,float &AZ);
bool IMU_read_gyro(ICM_20948_I2C &imu,float &GX,float &GY,float &GZ);
bool IMU_read_magneto(ICM_20948_I2C &imu,float &MX,float &MY,float &MZ);
bool IMU_read_magneto(ICM_20948_I2C &imu,float &TMP);

void initLED(Adafruit_NeoPixel &led);

int readBattery(bool percentage);

/*
SPIFF functions 
*/
bool checkStorage();     // this function will check and formates the SPIFF for the first time if EEPROM value is false
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);

/*
	EEPROM functions
*/
void EEPROMWritelong(int address, long value);
long EEPROMReadlong(long address);

//SHTC3 humidity sensor helper functions
double RHtoAbsolute(float relHumidity, float tempC);
uint16_t doubleToFixedPoint( double number);


#endif
