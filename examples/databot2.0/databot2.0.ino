/*
  See https://databot.us.com/setup/ for more info
*/

// Uncoment below line and upload code to get serial debug outputs
//#define debug
#include<databot2.h>
#include"def.h"

DynamicJsonDocument packet(1000);

OneWire oneWire(4);
OneWire oneWire2(23);
DallasTemperature tempsensor1(&oneWire);
DallasTemperature tempsensor2(&oneWire2);
SGP30 sgp30;
ICM_20948_I2C IMU; // ICM_20948_I2C object
SparkFun_APDS9960 apds = SparkFun_APDS9960();   // light sensor object
SHTC3 mySHTC3;    //humidity and temp          // Declare an instance of the SHTC3 class
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(3, LED_PIN, NEO_GRB + NEO_KHZ800);
AsyncWebServer server(80);
/////defining the static IP for the bot
IPAddress AP_LOCAL_IP(192, 168, 1, 160);
IPAddress AP_GATEWAY_IP(192, 168, 1, 4);
IPAddress AP_NETWORK_MASK(255, 255, 255, 0);

void IRAM_ATTR chargingLED();

// webpage placeholder logic
String processor(const String& var) {

  if (var == "startstate")
  {
    return STARTbtn;
  }
  else if (var == "stopstate")
  {
    return STOPbtn;
  }
  else if (var == "envdashstate")
  {
    return env_dash_btn;
  }
  //  else if(var =="dronedashstate")
  //  {
  //    return drone_dash_btn;
  //  }
  return String();
}


void setup() {

  Serial.begin(9600);
  EEPROM.begin(EEPROM_SIZE);
  delay(5);

  if (EEPROM.read(3) != 123)
  {
    EEPROMWritelong(10, 0);
    EEPROMWritelong(20, 0);
    EEPROM.write(3, 123);
    EEPROM.commit();
  }

  initLED(RGBled);
  Wire.begin();
  Wire.setClock(400000);
  pinMode(charge_state, INPUT);
  attachInterrupt(charge_state, chargingLED, CHANGE);

  pinMode(rx_pin, INPUT);
  pinMode(tx_pin, INPUT);


  //  pinMode(buzz, OUTPUT);
  //  digitalWrite(buzz, LOW);


  /*
     humidity sensor needs little time to get worm up around 5 sec
  */
  mySHTC3.begin();
  mySHTC3.setMode(SHTC3_CMD_CSE_RHF_LPM);  // added from V1 code
  mySHTC3.sleep(true);
  mySHTC3.update();


  // C02 init
  sgp30.begin();
  sgp30.initAirQuality();
  sgp30.setHumidity(doubleToFixedPoint(RHtoAbsolute(mySHTC3.toPercent(), mySHTC3.toDegC())));


  //IMU init
  IMU.begin(Wire, ICM_20948_ADD);

  Dprint(F("Initialization of the sensor returned: "));
  Dprintln(IMU.statusString());
  if (IMU.status != ICM_20948_Stat_Ok)
  {
    Dprintln("Trying again...");
    delay(500);
  }

  pinMode(APDS9960_INT, INPUT);

  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init()) {
    Dprintln(F("APDS-9960 initialization complete"));
  } else {
    Dprintln(F("Something went wrong during APDS-9960 init!"));
  }
  apds.setAmbientLightGain(0);
  apds.setGestureGain(0);
  apds.setLEDDrive(3);
  apds.setGestureLEDDrive(3);

  //  // Start running the APDS-9960 light sensor (no interrupts)
  //  if ( apds.enableLightSensor(false) ) {
  //    Dprintln(F("Light sensor is now running"));
  //  } else {
  //    Dprintln(F("Something went wrong during light sensor init!"));
  //  }



  if (!BARO.begin()) {
    Dprintln("Failed to initialize pressure sensor!");
  }

  if (initMIC())
  {
    Dprintln("MIC initialization sucess");
  }
  else
  {
    Dprintln("MIC initialization fail");
  }

  if (initTOF())
  {
    Dprintln("TOF initialization sucess");
  }
  else
  {
    Dprintln("TOF initialization fail");
  }


  if (digitalRead(charge_state) == LOW)
  {
    charging_state = true;
  }
  else
  {
    charging_state = false;
  }
  delay(100);

  altiCalib = EEPROMReadlong(10) / 100;
  Dprint("ALTI Calibration value: ");
  Dprintln(altiCalib);

  HumCalib = EEPROMReadlong(20) / 100;
  Dprint("HUM Calibration value: ");
  Dprintln(HumCalib);

  creatTask();
  set_bot_mode();

}

void loop() {

  while(BLOCKmode)
  {
     // all loop  code in block mode add here

     ///////////////////////////////////////
  }
  
  readSensors();
  // notify changed value
  if (isConnected() || start_exp)
  {
    if (start_exp )
    {
      while (millis() - data_send_millis < refresh)
      {
        // do nothing just wait
      }
      data_send_millis = millis();
      form_packet();
    }
  }
  else
  {
    if (!env_dash_en)
    {
      LED_breadh();
    }
    else
    {
      delay(50);
    }
  }


  // disconnecting
  if (!isConnected() && oldDeviceConnected)
  {
    reset_config(); // it will reset all the experiment configuration again
    if (BLEmode)
    {
      delay(500); // give the bluetooth stack the chance to get things ready
      advertising();
      Dprintln("start advertising");
    }
    oldDeviceConnected = isConnected();
  }

  // connecting
  if (isConnected() && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = isConnected();
  }
}
