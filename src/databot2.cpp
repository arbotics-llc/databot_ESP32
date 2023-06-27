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

#include "databot2.h"


bool _deviceConnected = false;
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;


/*
 class for BLE callback used to know BLE is connected or not
*/

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      _deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic) {
        rxValue = pCharacteristic->getValue();

		if (rxValue.length() > 0) {
		  
		// Deserialize the JSON document
		//DeserializationError error = deserializeJson(doc, rxValue);
		// // Test if parsing succeeds.
		// if (error) 
		// {
			 // Serial.print(F("deserializeJson() failed: "));
			// Serial.println(error.c_str());    
		// }
		// else
		// {
			//call another function here
			callback_Rx();
		// }
  
        // Serial.println("*********");
        // Serial.print("Received Value: ");

        // for (int i = 0; i < rxValue.length(); i++) {
          // Serial.print(rxValue[i]);
        // } Serial.println(); // Do stuff based on the command received from the app
//        if (rxValue.find("ON") != -1)
//        {
//          Serial.println("Turning ON!"); digitalWrite(LED, HIGH);
//        }
//        else if (rxValue.find("OFF") != -1)
//        {
//          Serial.println("Turning OFF!"); digitalWrite(LED, LOW);
//        }
        // Serial.println();
        // Serial.println("*********");
      }
    }
};

/*
function: initBLE
used to intitialize the BLE name along with service and characteristic UUID. It will start advertising with the service UUID

Usage: call it in the setup to initialize the BLE functuions.
*/

void initBLE()
{
  
  // Create the BLE Device
  BLEDevice::init("DB_databot");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

 /*
  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

*/
// Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_tx,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());
  
  
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_rx,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pCharacteristic->setCallbacks(new MyCallbacks());
  
				


  // Start the service
  pService->start();




  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  
}


/*
function: isConnected
type: bool
returns true if BLE or WIFI soft ap is connected else false
*/
bool isConnected()
{
	//if(_deviceConnected || WiFi.softAPgetStationNum() > 0)
	if(_deviceConnected)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
function: advertise
it will start BLE advertise
*/
void advertising()
{
	pServer->startAdvertising(); // advertising
}

/* function: sendPacket
	used to brodcast/ notify the data over BLE
	usage just pass the char data that need to be send over BLE
*/
void sendPacket(const char *data)
{
  pCharacteristic->setValue(data);
  pCharacteristic->notify();
}

/*
function: getValues
type: std string
function will recive the data over BLE and returns it in std string formate
*/
std::string getValues()
{
	std::string value = pCharacteristic->getValue();
	return value;
}


/*
	Getting ESP unique ID
*/

uint32_t ESP_getChipId()
{
	uint32_t chipId = 0;
	for(int i=0; i<17; i=i+8) {
	  chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
	}
	return chipId;
}

/*
* Function getExternalTemperature
* -------------------------------
* reads temperature in celsius from the DS18B20 temperature probe
*
* usage note: will retry on first three failures
*
* arguments:
* tempsensor: pointer to DallasTemperature object
*
* returns: on success returns temperature in Celsius, on failure returns
* -127 
*/
float getExternalTemperature(DallasTemperature &tempsensor) {
  float temperature; 


  for( int i = 0; i < 3; i++ ) {

   tempsensor.requestTemperatures();
   temperature = tempsensor.getTempCByIndex(0);

   if( temperature != -127 ) {

     return temperature;

   }

  }
  
  return temperature;
}


float get_heatIndex(float curTemp, float curHumidity)
{
	//Reference: https://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
	//curTemp = current temp in degrees F; curHumidity = relative humidity in percent; both type float
  curTemp = curTemp* 9/5;
	//Use short formula if less than 80
  float shortHi = 0.5 * (curTemp + 61 + ((curTemp - 68) * 1.2) + (curHumidity * 0.094));
  if((shortHi + curTemp) / 2 < 80) {
    return shortHi;
  }
  
  //Otherwise use long version
  float hi = -42.379 + (2.04901523 * curTemp) + (10.14333127 * curHumidity) - (0.22475541 * curTemp * curHumidity) - 
             (6.83783 * pow(10,-3) * pow(curTemp,2)) - (5.481717 * pow(10,-2) * pow(curHumidity,2)) +
             (1.22874 * pow(10,-3) * pow(curTemp,2) * curHumidity) + (8.5282 * pow(10,-4) * curTemp * pow(curHumidity,2)) -
             (1.99 * pow(10,-6) * pow(curTemp,2) * pow(curHumidity, 2));
  float adj = 0.0;

  if(curHumidity < 13) {
    adj = ((13 - curHumidity) / 4) * sqrt((17 - abs(curTemp - 95)) / 17);
  }
  else if(curHumidity > 85 && curTemp >= 80 && curTemp <= 87) {
    adj = ((curHumidity - 85) / 10) * ((87 - curTemp) / 5);
  }
  
  return hi - adj;
}
/*
	Function: IMU_read
	Type: bool
	arguments: 
	1) IMU ICM_20948_I2C pointer
	2) float pointer address for accleration X in m/s2
	3) float pointer address for accleration Y in m/s2
	4) float pointer address for accleration Z in m/s2
	5) float pointer address for Gyro X in Rad/S
	6) float pointer address for Gyro Y in Rad/S
	7) float pointer address for Gyro Z in Rad/S
	8) float pointer address for Mag X in TH
	9) float pointer address for Mag Y in TH
	10) float pointer address for Mag Z in TH
	11) float pointer address for Temp
	
*/
//bool IMU_read(ArduinoICM20948 &imu,float &AX,float &AY,float &AZ,float &GX,float &GY,float &GZ,float &MX,float &MY,float &MZ, float &TMP)
/*
bool IMU_read(ArduinoICM20948 &imu,float &AX,float &AY,float &AZ,float &GX,float &GY,float &GZ,float &MX,float &MY,float &MZ)
{
	
		byte error = 1;
		float a,b,c;
		imu.task();
		if (imu.accelDataIsReady())
		{
			imu.readAccelData(&a, &b, &c);
			AX = a*9.80665;
			AY = b*9.80665;
			AZ = c*9.80665;
			return true;
		}
		else
		{
			error = 0;
		}
		if (imu.gyroDataIsReady())
		{
			imu.readGyroData(&a, &b, &c);
			GX=a;
			GY=b;
			GZ=c;
			return true;
		}
		else
		{
			error = 0;
		}
		if (imu.magDataIsReady())
		{
			imu.readMagData(&a, &b, &c);
			MX=a;
			MY=b;
			MZ=c;
			return true;
		}
		else
		{
			error = 0;
		}
		
		if(error)
		{
			return true;
		}
		else
		{
			return false;
		}
}
*/
	
/*
	


/*
	will only read the acclerometer readings from IMU
*/
	
bool IMU_read_accl(ArduinoICM20948 &imu,float &AX,float &AY,float &AZ)
{
	/*//old IMU library code
	if(imu.dataReady())
	{
		imu.getAGMT();
		AX= imu.accX() * 0.00980665;
		AY= imu.accY() * 0.00980665;
		AZ= imu.accZ() * 0.00980665;
		return true;
	}
	else
	{
		return false;
	}
	*/
	
	// new IMU library code
	float a,b,c;
	imu.task();
	if (imu.accelDataIsReady())
	{
		imu.readAccelData(&a, &b, &c);
		AX = a*9.80665*(-1);  // added -1 to match the new IMU values
		AY = b*9.80665;
		AZ = c*9.80665;
		return true;
	}
	else
	{
		return false;
	}
}

bool IMU_read_accl(MPU9250 &imu,float &AX,float &AY,float &AZ)
{
	if (imu.update())
	{
		AX = imu.getAccX()*9.80665;
		AY = imu.getAccY()*9.80665;
		AZ = imu.getAccZ()*9.80665;
		return true;
	}
	else
	{
		return false;
	}
}

/*
	will only read the gyro readings from IMU
*/
bool IMU_read_gyro(ArduinoICM20948 &imu,float &GX,float &GY,float &GZ)
{ 
	/*//old IMU library code
	if(imu.dataReady())
	{
		imu.getAGMT();
		GX = imu.gyrX()*DEG_TO_RAD;
		GY = imu.gyrY()*DEG_TO_RAD;
		GZ = imu.gyrZ()*DEG_TO_RAD;
		return true;
	}
	else
	{
		return false;
	}
	*/
	// new IMU library code
	float a,b,c;
	imu.task();
	if (imu.gyroDataIsReady())
	{
		imu.readGyroData(&a, &b, &c);
		GX=a*(-1);    // added -1 to match the new IMU values
		GY=b;
		GZ=c;
		return true;
	}
	else
	{
		return false;
	}
}

bool IMU_read_gyro(MPU9250 &imu,float &GX,float &GY,float &GZ)
{
	if (imu.update())
	{
		//GX=imu.getGyroX();
		//GY=imu.getGyroY();
		//swaping them
		GY=imu.getGyroX();
		GX=imu.getGyroY();
		GZ=imu.getGyroZ();
		return true;
	}
	else
	{
		return false;
	}
}

/*
	will only read the magneto readings from IMU
*/
bool IMU_read_magneto(ArduinoICM20948 &imu,float &MX,float &MY,float &MZ)
{
	/*//old IMU library code
	if(imu.dataReady())
	{
		imu.getAGMT();
		MX = imu.magX();
		MY = imu.magY();
		MZ = imu.magZ();
		return true;
	}
	else
	{
		return false;
	}
	*/
	// new IMU library code
	float a,b,c;
	imu.task();
	if (imu.magDataIsReady())
	{
		imu.readMagData(&a, &b, &c);
		MX=a*(-1) ;   // added -1 to match the new IMU values
		MY=b;
		MZ=c;
		return true;
	}
	else
	{
		return false;
	}
}
bool IMU_read_magneto(MPU9250 &imu,float &MX,float &MY,float &MZ)
{
	if (imu.update())
	{
		//MX=imu.getMagX();
		//MY=imu.getMagY();
		//swaping them
		MY=imu.getMagX();
		MX=imu.getMagY();
		MZ=imu.getMagZ();
		return true;
	}
	else
	{
		return false;
	}
}
/*
	will only read the linear acclerometer readings from IMU
*/
bool IMU_read_LinearAccl(ArduinoICM20948 &imu,float &AX,float &AY,float &AZ)
{
	float a,b,c;
	imu.task();
	if (imu.linearAccelDataIsReady())
	{
		imu.readLinearAccelData(&a, &b, &c);
		AX = a*9.80665;
		AY = b*9.80665;
		AZ = c*9.80665;
		return true;
	}
	else
	{
		return false;
	}
}
bool IMU_read_LinearAccl(MPU9250 &imu,float &AX,float &AY,float &AZ)
{
	if (imu.update())
	{
		AX = imu.getLinearAccX()*9.80665;
		AY = imu.getLinearAccY()*9.80665;
		AZ = imu.getLinearAccZ()*9.80665;
		return true;
	}
	else
	{
		return false;
	}
}
/*
	will only read the Temp readings from IMU
*/
/*  // not available in new library
bool IMU_read_temp(ArduinoICM20948 &imu,float &TMP)
{
	if(imu.dataReady())
	{
		imu.getAGMT();
		TMP = imu.temp();
		return true;
	}
	else
	{
		return false;
	}
}
*/
/*
  Function: getUV() 
  type: byte
  will read the uv sensor and returns the uv index
*/
byte getUV()
{
  int adcVAL = analogRead(UV_pin);
  //float voltage = ((adcVAL/4095)*3.3)*1000;
  float voltage = (adcVAL*3300)/4095;
  byte UVIndex;
  if(voltage<50)
  {
    UVIndex = 0;
  }else if (voltage>50 && voltage<=227)
  {
    UVIndex = 0;
  }else if (voltage>227 && voltage<=318)
  {
    UVIndex = 1;
  }
  else if (voltage>318 && voltage<=408)
  {
    UVIndex = 2;
  }else if (voltage>408 && voltage<=503)
  {
    UVIndex = 3;
  }
  else if (voltage>503 && voltage<=606)
  {
    UVIndex = 4;
  }else if (voltage>606 && voltage<=696)
  {
    UVIndex = 5;
  }else if (voltage>696 && voltage<=795)
  {
    UVIndex = 6;
  }else if (voltage>795 && voltage<=881)
  {
    UVIndex = 7;
  }
  else if (voltage>881 && voltage<=976)
  {
    UVIndex = 8;
  }
  else if (voltage>976 && voltage<=1079)
  {
    UVIndex = 9;
  }
  else if (voltage>1079 && voltage<=1170)
  {
    UVIndex = 10;
  }else if (voltage>1170)
  {
    UVIndex = 11;
  }
  return UVIndex;
}

/*
	function: getPressure()
	type: float
	will read the pressure values in Kpa and returns the value
*/
float getPressure()
{
	return BARO.readPressure();
}

/*
	function: getPressure_temperature()
	type: float
	will read the temperature from pressure sensor values in C is returned.
*/
float getPressure_temperature()
{
	return BARO.readTemperature();
}
/*
	function: getAltitude()
	type: float
	it will fetch the pressure data and will convert that to altitude data and return it in meter.
	refer for more info:  https://docs.arduino.cc/tutorials/nano-33-ble-sense/barometric_sensor
*/
float getAltitude()
{
	//H = 44330 * [1 - (P/p0)^(1/5.255) ]
	float press = BARO.readPressure();
	return (44330 * ( 1 - pow(press/101.325, 1/5.255) ));
}



/*************************************************************************************************************************************************************/
// MIC Code
//The code for the MIC sensor is mostly revised from online source so most of the calculation to get sound intensity are over my head. 
void integerToFloat(int32_t * integer, float *vReal, float *vImag, uint16_t samples)
{
	for (uint16_t i = 0; i < samples; i++) 
	{
		vReal[i] = (integer[i] >> 16) / 10.0;
		vImag[i] = 0.0;
	}
}

// calculates energy from Re and Im parts and places it back in the Re part (Im part is zeroed)
void calculateEnergy(float *vReal, float *vImag, uint16_t samples)
{
	for (uint16_t i = 0; i < samples; i++) 
	{
		vReal[i] = sq(vReal[i]) + sq(vImag[i]);
		vImag[i] = 0.0;
	}
}

// sums up energy in bins per octave
void sumEnergy(const float *bins, float *energies, int bin_size, int num_octaves)
{
// skip the first bin
  int bin = bin_size;
  for (int octave = 0; octave < num_octaves; octave++) {
    float sum = 0.0;
    for (int i = 0; i < bin_size; i++) {
      sum += real[bin++];
    }
    energies[octave] = sum;
    bin_size *= 2;
  }
}

float decibel(float v)
{
  return 10.0 * log(v) / log(10);
}

// converts energy to logaritmic, returns A-weighted sum
float calculateLoudness(float *energies, const float *weights, int num_octaves, float scale)
{
  float sum = 0.0;
  for (int i = 0; i < num_octaves; i++) {
    float energy = scale * energies[i];
    sum += energy * pow(10, weights[i] / 10.0);
    energies[i] = decibel(energy);
  }
  return decibel(sum);
}

/*  
	function: initMIC
	type: int
	returns 1 if MIC is initialized properly with I2S drivers
	returns -1 if I2S drivers are not initialized properly
	returns -2 if I2S pins are not initialized properly
	
*/
int initMIC()
{
	esp_err_t err;

  // The I2S config as per the example
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),      // Receive, not transfer
    .sample_rate = 22627,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    //.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,   // although the SEL config should be left, it seems to transmit on right
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,     // Need to change this to I2S_CHANNEL_FMT_RIGHT_LEFT after updating ESP32 package to V2.0.3
	//.communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
	.communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,       // Interrupt level 1
    .dma_buf_count = 8,     // number of buffers
    .dma_buf_len = BLOCK_SIZE,      // samples per buffer
    .use_apll = true
  };

  // The pin config as per the setup
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,       // BCKL
    .ws_io_num = I2S_WS,        // LRCL
    .data_out_num = -1,     // not used (only for speakers)
    .data_in_num = I2S_SD       // DOUT
  };

  // Configuring the I2S driver and pins.
  // This function must be called before any I2S driver read/write operations.
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
	return -1;
  }
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
	return -2;
  }
  return 1;
}

/*
  function: getLoudness
  type: float
  it will measure the loudness from the samples collected from the MIC sensor and some sofesticated FFT analysis
  and returns measured loudness in dBa
*/
float getLoudness()
{
	static int32_t samples[BLOCK_SIZE];

  // Read multiple samples at once and calculate the sound pressure
  size_t num_bytes_read;
  esp_err_t err = i2s_read(I2S_PORT,
                           (char *) samples,
                           BLOCK_SIZE,        // the doc says bytes, but its elements.
                           &num_bytes_read,
                           portMAX_DELAY);    // no timeout
  int samples_read = num_bytes_read / 8;

  // integer to float
  integerToFloat(samples, real, imag, SAMPLES);

  // apply flat top window, optimal for energy calculations
  fft.Windowing(FFT_WIN_TYP_FLT_TOP, FFT_FORWARD);
  fft.Compute(FFT_FORWARD);

  // calculate energy in each bin
  calculateEnergy(real, imag, SAMPLES);

  // sum up energy in bin for each octave
  sumEnergy(real, energy, 1, OCTAVES);

  // calculate loudness per octave + A weighted loudness
  return calculateLoudness(energy, aweighting, OCTAVES, 1.0);
}
/*************************************************************************************************************/

//TOF code
 VL53L1_Dev_t                   dev;
 VL53L1_DEV                     Dev = &dev;
 static float TOF_distance;
/*
	function: initTOF
	type: bool
	it will initialize the TOF sensor and returns true on sucess else false.
*/
bool initTOF(bool range)
{
	// This is the default 8-bit slave address (including R/W as the least
	// significant bit) as expected by the API. Note that the Arduino Wire library
	// uses a 7-bit address without the R/W bit instead (0x29 or 0b0101001).
	Dev->I2cDevAddr = 0x52;

	VL53L1_software_reset(Dev);
	int status = VL53L1_WaitDeviceBooted(Dev);
	if(!status) status = VL53L1_DataInit(Dev);
	if(!status) status = VL53L1_StaticInit(Dev);
	if(range)
	{
		if(!status) status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
	}
	else
	{
		if(!status) status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_SHORT);
	}
	if(!status) status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, (uint32_t)MEASUREMENT_BUDGET_MS * 1000);
	if(!status) status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, INTER_MEASUREMENT_PERIOD_MS);
	if(!status) status = VL53L1_StartMeasurement(Dev);

	/////////////////////////seting the ROI of the sensor
	/*
		For setting ROI refer here https://forum.pololu.com/t/vl53l1x-region-of-interest-roi/15300/9
	*/
	VL53L1_CalibrationData_t calibrationData;
	status = VL53L1_GetCalibrationData(Dev, &calibrationData);
    byte center_x = calibrationData.optical_centre.x_centre / 16.0;
	byte center_y = calibrationData.optical_centre.y_centre / 16.0;
	VL53L1_UserRoi_t roiConfig;
	if(center_x>=4 && center_x<=12 && center_y >=4 && center_y<=12)
	{
		
		roiConfig.TopLeftX = center_x-2;
		roiConfig.TopLeftY = center_y+2;
		roiConfig.BotRightX = center_x+2;
		roiConfig.BotRightY = center_y-2;
	}
	else
	{
		roiConfig.TopLeftX = 6;
		roiConfig.TopLeftY = 10;
		roiConfig.BotRightX = 10;
		roiConfig.BotRightY = 6;
	}
	
	status = VL53L1_SetUserROI(Dev, &roiConfig);
	
	if(status)
	{
		return false;
	}
	else
	{
		return true;
	}
}	


/*
	function: getDistance
	type: float
	it will measure the distance in non blocking state and returns the measured distance in mm.
	returns -1 if error in read
	returns -2 if timeout
	
*/
float getDistance()
{
	static uint16_t startMs = millis();
	uint8_t isReady;
    
	// non-blocking check for data ready
	int status = VL53L1_GetMeasurementDataReady(Dev, &isReady);
	if(!status)
	{
		if(isReady)
		{	
			static VL53L1_RangingMeasurementData_t RangingData; 
			status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
			if(!status)
			{
				// if(RangingData.RangeStatus == 0)
				// {
					TOF_distance = RangingData.RangeMilliMeter;
				// }
				// else
				// {
					// TOF_distance = 0;
				// }
			}
			else
			{
				TOF_distance = -1;
			}
			VL53L1_ClearInterruptAndStartMeasurement(Dev);
			startMs = millis();
		}
		else if((uint16_t)(millis() - startMs) > VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS)
		{
			//Serial.print(F("Timeout waiting for data ready."));
			TOF_distance = -2;
			VL53L1_ClearInterruptAndStartMeasurement(Dev);
			startMs = millis();
		}
	}
	else
	{
		TOF_distance = -1;
	}
	return TOF_distance;
}


/*
function: initLED
type: void
it will initialize the rgb led
*/
void initLED(Adafruit_NeoPixel &led)
{
	led.begin();
	led.setBrightness(50);
	led.show(); // Initialize all pixels to 'off'
}

/*
function:checkStorage
type:bool
it will check an fixed EEPROM address and if it has a invalid data then it will formate the SPIFF storage.
return true if SPIFF is properly mounted else false
*/
bool checkStorage()
{
	// initialize EEPROM with predefined size
	
	bool check = true;
	if(EEPROM.read(0) != 123)
	{
		if (!SPIFFS.begin(true))  // will formate the SPIFF if mount fails
		{
			Serial.println("mount fail 1");
			check = false;
		}
		EEPROM.write(0, 123);
		EEPROM.commit();
		
	}
	else
	{
		if (!SPIFFS.begin(false))  
		{
			Serial.println("mount fail 2");
			check = false;
		}	
		else
		{
			Serial.println("mount");
		}
	}
	return check;
}



/*
	function: writeFile
	type : void
	it will write new file in the SPIFF and replase the older one if same name found. 
*/
void writeFile(fs::FS &fs, const char * path, const char * message) 
{
//  Dprintf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Dprintln("− failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Dprintln("− file written");
  } else {
    Dprintln("− frite failed");
  }
}

/*
function: appendFile
type : void
it will append file in the SPIFF and replase the older one if same name found. 
*/

void appendFile(fs::FS &fs, const char * path, const char * message) {
  //   Serial.printf("Appending to file: %s\r\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Dprintln("− failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Dprintln("mess App");
  } else {
    Dprintln("append fail");
  }
}

int readBattery(bool percentage)
{
	int adcVAL=0;
	for(byte i=0 ; i<10 ; i++)
	{
		adcVAL = analogRead(batVTG) + adcVAL;
		delay(50);
	}
	adcVAL = adcVAL/10;
	
	//int adcVAL = analogRead(batVTG);
	//float voltage = ((adcVAL/4095)*3.3)*1000;
	float voltage = (adcVAL*3.3*2.12)/4095;        // have voltage divider so multiplication factor should be 2 but added 0.12 to compensate error in vtg reading 
	int vtg = voltage*100; 
	vtg = map(vtg, 360, 415, 0, 100);
	if(vtg<0)
	{
		vtg = 0;
	}
	else if(vtg >100)
	{
		vtg = 100;
	}
	if(percentage)
	{
		return vtg;
	}
	else
	{
		return voltage;
	}
}


/*
function: EEPROMWritelong
it will write long data to EEPROM storage 
*/
void EEPROMWritelong(int address, long value)
{
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.commit();
  EEPROM.write(address + 1, three);
  EEPROM.commit();
  EEPROM.write(address + 2, two);
  EEPROM.commit();
  EEPROM.write(address + 3, one);
  EEPROM.commit();
}
/*
function: EEPROMReadlong
it will read long data to EEPROM storage 
*/
long EEPROMReadlong(long address)
{
  //Read the 4 bytes from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);

  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

/*
function: EEPROMwriteString
it will write string data to EEPROM storage 
*/
void EEPROMwriteString(const char* toStore, int startAddr)
{
	int i = 0;
	for (; i < LENGTH(toStore); i++) {
		EEPROM.write(startAddr + i, toStore[i]);
	}
	EEPROM.write(startAddr + i, '\0');
	EEPROM.commit();
}


/*
function: EEPROMreadStringFromFlash
it will read String data to EEPROM storage 
*/
String EEPROMreadStringFromFlash(int startAddr)
{
	char in[128];
	char curIn;
	int i = 0;
	curIn = EEPROM.read(startAddr);
	for (; i < 128; i++) {
		curIn = EEPROM.read(startAddr + i);
		in[i] = curIn;
	}
	return String(in);
}


/*
* Function RHtoAbsolute
* ---------------------
* calculates the absolute humidity from relative humidity and temperature
*
* arguments:
* relHumidity: relative humidity in percent
* tempC: temperature in celsius
*
* returns: absolute humidty as grams per cubic meter of air
*/
double RHtoAbsolute(float relHumidity, float tempC) {
  double eSat = 6.11 * pow(10.0, (7.5 * tempC / (237.7 + tempC)));
  double vaporPressure = (relHumidity * eSat) / 100; //millibars
  double absHumidity = 1000 * vaporPressure * 100 / ((tempC + 273) * 461.5); //Ideal gas law with unit conversions
  return absHumidity;
}

/*
* Function doubleToFixedPoint
* ---------------------------
* converts double (Floating point) to a fixed point decimal point value
*
* usage note: float and double are the same thing our processor the ATmega 328p
*
* arguments:
* number: floating point number you want converted to fixed point
*
* returns: fixed point value
*/
uint16_t doubleToFixedPoint( double number) {
  int power = 1 << 8;
  double number2 = number * power;
  uint16_t value = floor(number2 + 0.5);
  return value;
}