void interruptRoutine() {
  if (digitalRead(APDS9960_INT) == LOW)
  {
    gestureInt = true;
  }
}
void readSensors()
{

  if (reset_APDS9960)
  {
    reset_APDS9960 = false;
    apds.disableLightSensor();
    apds.disableGestureSensor();
    detachInterrupt(APDS9960_INT);
  }

  if (TOF_config_again)
  {
    TOF_config_again = false;
    if (short_distance)
    {
      if (initTOF(false))
      {
        Dprintln("TOF initialization sucess");
      }
    }
    else if (long_distance)
    {
      Dprintln("die");
      if (initTOF(true))
      {
        Dprintln("TOF initialization sucess");
      }
      Dprintln("alive");
    }
  }

  if (Light_config_again)
  {
    //    Light_config_again = false;
    if (ambLight || rgbLight)
    {
      // Start running the APDS-9960 light sensor (no interrupts)
      if ( apds.enableLightSensor(false) ) {
        Dprintln(F("Light sensor is now running"));
        Light_config_again = false;
      } else {
        Dprintln(F("Something went wrong during light sensor init!"));
      }
    }
    if (gesture)
    {
      attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);
      if ( apds.enableGestureSensor(true) ) {
        Dprintln(F("Gesture sensor is now running"));
        Light_config_again = false;
      } else {
        Dprintln(F("Something went wrong during gesture sensor init!"));
      }
    }
  }


  dataFormate = "m";
  if (externalTemp1)
  {
    dataFormate += "t";
    ExtTmp1 = getExternalTemperature(tempsensor1);
  }
  else
  {
    ExtTmp1 = 0;
  }

  if (externalTemp2)
  {
    dataFormate += "w";
    ExtTmp2 = getExternalTemperature(tempsensor2) - TempCalib2;
  }
  else
  {
    ExtTmp2 = 0;
  }

  if (co2 || voc)
  {
    sgp30.measureAirQuality();
    Co2 = sgp30.CO2;
    VOC = sgp30.TVOC;
    if (co2)
    {
      dataFormate += "c";
    }
    else
    {
      Co2 = 0;
    }
    if (voc)
    {
      dataFormate += "v";
    }
    else
    {
      VOC = 0;
    }
  }
  else
  {
    Co2 = 0;
    VOC = 0;
  }

  if (accl && gyro && magneto)
  {
    Dprintln("AGM measure");
    IMU_read(IMU, Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz, Itemp);
    dataFormate += "asfxyzijk";
    if (IMUtemp)
    {
      dataFormate += "o";
    }
    else
    {
      Itemp = 0;
    }
  }
  else
  {
    if (accl)
    {
      dataFormate += "asf";
      IMU_read_accl(IMU, Ax, Ay, Az);
    }
    else
    {
      Ax = 0;
      Ay = 0;
      Az = 0;
    }
    if (gyro)
    {
      dataFormate += "xyz";
      IMU_read_gyro(IMU,  Gx, Gy, Gz);
    }
    else
    {
      Gx = 0;
      Gy = 0;
      Gz = 0;
    }
    if (magneto)
    {
      dataFormate += "ijk";
      IMU_read_magneto(IMU, Mx, My, Mz);
    }
    else
    {
      Mx = 0;
      My = 0;
      Mz = 0;
    }
    if (IMUtemp)
    {
      dataFormate += "o";
      IMU_read_magneto(IMU, Itemp);
    }
    else
    {
      Itemp = 0;
    }
  }

  if (ambLight)
  {
    dataFormate += "l";
    if (!apds.readAmbientLight(AmbLight))
    {
      Dprintln("ambiant light error");
    }
  }
  else
  {
    AmbLight = 0;
  }
  if (rgbLight)
  {
    dataFormate += "rgb";
    if (!apds.readRedLight(RLight) || !apds.readGreenLight(GLight) || !apds.readBlueLight(BLight))
    {
      Dprintln("RGB light error");
    }
  }
  else
  {
    RLight = 0;
    GLight = 0;
    BLight = 0;
  }

  if (humidity || humidityTemp)
  {
    mySHTC3.update();
    mySHTC3.sleep(true);
    Humidity = mySHTC3.toPercent() - HumCalib;
    HumTemp = mySHTC3.toDegC();
    if (humidity)
    {
      dataFormate += "h";
    }
    else
    {
      humidity = 0;
    }
    if (humidityTemp)
    {
      dataFormate += "q";
    }
    else
    {
      humidityTemp = 0;
    }
  }
  else
  {
    humidity = 0;
    humidityTemp = 0;
  }


  if (uvIndex)
  {
    dataFormate += "u";
    UVindex = getUV();
  }
  else
  {
    UVindex = 0;
  }


  if (pressure)
  {
    dataFormate += "p";
    Pressure = getPressure() * 10; // will get pressure in Kpa so *10 for hPa
  }
  else
  {
    Pressure = 0;
  }
  if (alti)
  {
    dataFormate += "e";
    //    Altitude = getAltitude() - altiCalib; // will get altitude in meter
    AltitudeFilter.Filter(getAltitude() - altiCalib);
    Altitude = AltitudeFilter.Current();
  }
  else
  {
    Altitude = 0;
    AltitudeFilter.SetCurrent(0);
  }
  if (noise)
  {
    dataFormate += "n";
    Noise = getLoudness();
  }
  else
  {
    Noise = 0;
  }

  if (long_distance || short_distance)
  {
    dataFormate += "d";
    Distance = getDistance();
  }
  else
  {
    Distance = 0;
  }


  if (gesture)
  {
    dataFormate += "G";
    if ( gestureInt)
    {
      detachInterrupt(APDS9960_INT);
      gestureInt = false;
      if ( apds.isGestureAvailable() )
      {
        switch ( apds.readGesture() ) {
          case DIR_UP:
            {
              Dprintln("UP");
              currentGesture = 1;
            }
            break;
          case DIR_DOWN:
            {
              Dprintln("DOWN");
              currentGesture = 2;
            }
            break;
          case DIR_LEFT:
            {
              Dprintln("LEFT");
              currentGesture = 3;
            }
            break;
          case DIR_RIGHT:
            {
              Dprintln("RIGHT");
              currentGesture = 4;
            }
            break;
          case DIR_NEAR:
            {
              Dprintln("NEAR");
              currentGesture = 5;
            }
            break;
          case DIR_FAR:
            {
              Dprintln("FAR");
              currentGesture = 6;
            }
            break;
          default:
            {
              Dprintln("NONE");
              currentGesture = 0;
            }
        }
      }
      attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);
    }
  }
  else
  {
    currentGesture = 0;
    gestureInt = false;
  }

  if (usbCheck)
  {
    dataFormate += "RT";
    rx_state = digitalRead(rx_pin);
    tx_state = digitalRead(tx_pin);
  }
  else
  {
    rx_state = 0;
    tx_state = 0;
  }

  if (sysCheck)
  {
    if (start_exp)
    {
      vTaskResume(buzz);
      vTaskResume(led);
      //      RGBled.setPixelColor(0, RGBled.Color(random(0, 255), random(0, 255), random(0, 255)));
      //      RGBled.setPixelColor(1, RGBled.Color(random(0, 255), random(0, 255), random(0, 255)));
      //      RGBled.setPixelColor(2, RGBled.Color(random(0, 255), random(0, 255), random(0, 255)));
      //      RGBled.show();
    }
    dataFormate += "BEV";
    batteryVTG = readBattery(true);
    Dprintln(batteryVTG);
    ESPchipID = String(ESP_getChipId());
    Dprintln(ESPchipID);
  }
  //  else
  //  {
  //    batteryVTG = 0;
  //    ESPchipID = "";
  //  }

}



/*
   Will generate the header csv for the excel live data
*/
String generateCsvHeader() {
  String csvHeader = "Time";
  if (accl)
  {
    csvHeader += delimiter + "a.x" + delimiter + "a.y" + delimiter + "a.z";
  }

  if (gyro)
  {
    csvHeader += delimiter + "g.x" + delimiter + "g.y" + delimiter + "g.z";
  }

  if (magneto)
  {
    csvHeader += delimiter + "m.x" + delimiter + "m.y" + delimiter + "m.z";
  }

  if (IMUtemp)
  {
    csvHeader += delimiter + "IMU Temp";
  }

  if (externalTemp1)
  {
    csvHeader += delimiter + "Temperature1";
  }

  if (externalTemp2)
  {
    csvHeader += delimiter + "Temperature2";
  }

  if (pressure)
  {
    csvHeader += delimiter + "pressure";
  }

  if (alti)
  {
    csvHeader += delimiter + "altitude";
  }

  if (ambLight)
  {
    csvHeader += delimiter + "lux";
  }

  if (rgbLight)
  {
    csvHeader += delimiter + "R" + delimiter + "G" + delimiter + "B";
  }

  if (uvIndex)
  {
    csvHeader += delimiter + "UV Index";
  }
  if (co2)
  {
    csvHeader += delimiter + "CO2";
  }
  if (voc)
  {
    csvHeader += delimiter + "TVOC";
  }
  if (humidity)
  {
    csvHeader += delimiter + "humidity";
  }
  if (humidityTemp)
  {
    csvHeader += delimiter + "HUM Temp";
  }
  if (short_distance)
  {
    csvHeader += delimiter + "S Distance";
  }
  if (long_distance)
  {
    csvHeader += delimiter + "L Distance";
  }
  if (noise)
  {
    csvHeader += delimiter + "Dba";
  }
  csvHeader.trim();
  csvHeader += "\r\n";

  return csvHeader;

}

String generateCsvRecord() {
  String csvRecord = String(readTime);

  if (accl)
  {
    csvRecord += delimiter + Ax + delimiter + Ay + delimiter + Az;
  }

  if (gyro)
  {
    csvRecord += delimiter + Gx + delimiter + Gy + delimiter + Gz;
  }

  if (magneto)
  {
    csvRecord += delimiter + Mx + delimiter + My + delimiter + Mz;
  }

  if (IMUtemp)
  {
    csvRecord += delimiter + Itemp;
  }

  if (externalTemp1)
  {
    csvRecord += delimiter + ExtTmp1;
  }

  if (externalTemp2)
  {
    csvRecord += delimiter + ExtTmp2;
  }

  if (pressure)
  {
    csvRecord += delimiter + Pressure;
  }

  if (alti)
  {
    csvRecord += delimiter + Altitude;
  }

  if (ambLight)
  {
    csvRecord += delimiter + AmbLight;
  }

  if (rgbLight)
  {
    csvRecord += delimiter + RLight + delimiter + GLight + delimiter + BLight;
  }

  if (uvIndex)
  {
    csvRecord += delimiter + UVindex;
  }
  if (co2)
  {
    csvRecord += delimiter + Co2;
  }
  if (voc)
  {
    csvRecord += delimiter + VOC;
  }
  if (humidity)
  {
    csvRecord += delimiter + Humidity;
  }
  if (humidityTemp)
  {
    csvRecord += delimiter + HumTemp;
  }
  if (short_distance)
  {
    csvRecord += delimiter + Distance;
  }
  if (long_distance)
  {
    csvRecord += delimiter + Distance;
  }
  if (noise)
  {
    csvRecord += delimiter + Noise;
  }

  //remove leading and trailing whitespace
  csvRecord.trim();
  csvRecord += "\r\n";

  return csvRecord;
}

//to reset all the variables and databot state when BLE is disconnected
void reset_config()
{
  if (sysCheck)
  {
    vTaskSuspend(buzz);
    noTone(BUZZER_PIN, BUZZER_CHANNEL);
    vTaskSuspend(led);
  }
  RGBled.setPixelColor(0, RGBled.Color(0, 0, 0));
  RGBled.setPixelColor(1, RGBled.Color(0, 0, 0));
  RGBled.setPixelColor(2, RGBled.Color(0, 0, 0));
  RGBled.show();
  config_done = false;
  start_exp = false;
  resetTime = false;
  sendCSVheader = false;

  env_dash_en = false;

  breadhColor = 0;
  brighter = false;


  refresh = 100;
  resolution = 2;
  time_factor = 1000;
  time_decimal = 2;


  accl = false;
  gyro = false;
  magneto = false;
  IMUtemp = false;
  externalTemp1 = false;
  externalTemp2 = false;
  pressure = false;
  alti = false;
  ambLight = false;
  rgbLight = false;
  uvIndex = false;
  co2 = false;
  voc = false;
  humidity = false;
  humidityTemp = false;
  short_distance = false;
  long_distance = false;
  noise = false;

  usbCheck = false;
  sysCheck = false;
  altitudeCalibrate = false;

  gesture = false;
  gestureInt = false;

  newTempCalib = 0;
  pastTempCalib = 0;
  TempCalib2 = 0;

  reset_APDS9960 = true;
  //  apds.disableLightSensor();
  //  apds.disableGestureSensor();
  //  detachInterrupt(APDS9960_INT);
}

// breadhing patern for the LED
void LED_breadh()
{
  if (millis() - blink_milli >= 7)
  {
    blink_milli = millis();
    if (!brighter)
    {
      if (breadhColor < 255)
      {
        breadhColor++;
      }
      else
      {
        brighter = true;
      }
    }
    else
    {
      if (breadhColor > 0)
      {
        breadhColor--;
      }
      else
      {
        brighter = false;
      }
    }
    if (BLEmode)
    {
      RGBled.setPixelColor(0, RGBled.Color(0, 0, breadhColor));
    }
    else if (WiFimode)
    {
      RGBled.setPixelColor(0, RGBled.Color(0, breadhColor, 0));
    }
    if (charging_state)
    {
      RGBled.setPixelColor(2, RGBled.Color(breadhColor, 0, 0));
    }
    else
    {
      RGBled.setPixelColor(2, 0, 0, 0);
    }
    RGBled.show();
  }
}


void set_bot_mode()
{
  byte counter = 0;
here:
  counter++;
  IMU_read_accl(IMU, Ax, Ay, Az);

  if (Az < -7) //Wi-Fi mode on
  {
    WiFimode = true;
    if (checkStorage())
    {
      Dprintln("SPII mounted");
      total_bytes = SPIFFS.totalBytes();
      Dprint("Total Bytes:");
      Dprintln(total_bytes);

      used_bytes = SPIFFS.usedBytes();
      Dprint("used Bytes:");
      Dprintln(used_bytes);
    }

    WiFi.softAP(ssid.c_str(), password);

    if (!WiFi.softAPConfig(AP_LOCAL_IP, AP_GATEWAY_IP, AP_NETWORK_MASK)) {
      Dprintln("AP Config Failed");
      return;
    }
    Dprint("IP address: ");
    Dprintln(WiFi.softAPIP());

    delay(1000);

    server.on("/index3.html", HTTP_GET, [](AsyncWebServerRequest * request) {
      //get specific header by name
      int params = request->params();
      bool reset_once = true;
      if (env_dash_en)
      {
        reset_config();
      }
      for (int i = 0; i < params; i++) {
        if (reset_once)
        {
          reset_once = false;
          reset_config();
        }
        AsyncWebParameter* p = request->getParam(i);
        if (p->name() == M1)
        {
          //Mission 1 selected
          Dprintln("start M1");
          co2 = true;
          alti  = true;
          humidity = true;
          externalTemp1  = true;
          ambLight  = true;
          Light_config_again = true;
        }
        else if (p->name() == M2)
        {
          //Mission 2 selected
          Dprintln("start M2");
          magneto = true;
          TOF_config_again = true;
          long_distance = true;
        }
        else if (p->name() == M3)
        {
          //Mission 3 selected
          Dprintln("start M3");
          TOF_config_again = true;
          long_distance = true;
          rgbLight = true;
          Light_config_again = true;
        }
        else if (p->name() == M4)
        {
          //Mission 4 selected
          Dprintln("start M4");
          TOF_config_again = true;
          long_distance = true;
        }
        else if (p->name() == PARAM_start_btn)
        {
          Dprintln("Start exp");
          RGBled.setPixelColor(0, RGBled.Color(0, 255, 0));
          RGBled.setPixelColor(2, RGBled.Color(0, 0, 0));
          RGBled.show();
          start_exp = true;
          resetTime = true;
          sendCSVheader = true;
          STARTbtn = "disabled";
          env_dash_btn = "disabled";
          STOPbtn = "";
        }
        else if (p->name() == PARAM_stop_btn)
        {
          ///  experiment storage will stop because its alredy reset in the start of this function due to that start_exp becomed false
          STOPbtn = "disabled";
          STARTbtn = "";
          env_dash_btn = "";
        }
      }
      request->send(SPIFFS, "/index3.html", String(), false, processor);
    });

    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      //get specific header by name
      int params = request->params();
      bool reset_once = true;
      if (env_dash_en)
      {
        reset_config();
      }
      for (int i = 0; i < params; i++) {
        if (reset_once)
        {
          reset_once = false;
          reset_config();
        }

        AsyncWebParameter* p = request->getParam(i);
        //Serial.printf("GET[%s]: %s\n", p->name().c_str(), p->value().c_str());

        if (p->name() == ACCL)
        {
          accl = true;
        }
        else if (p->name() == GYRO)
        {
          gyro = true;
        }
        else if (p->name() == MAGNETO)
        {
          magneto = true;
        }
        else if (p->name() == IMU_TEMP)
        {
          IMUtemp = true;
        }
        else  if (p->name() == EX_TEMP1)
        {
          externalTemp1  = true;
        }
        else if (p->name() == EX_TEMP2)
        {
          externalTemp2  = true;
        }
        else if (p->name() == PRESS)
        {
          pressure  = true;
        }
        else if (p->name() == ALTI)
        {
          alti  = true;
        }
        else if (p->name() == AMB_L)
        {
          ambLight  = true;
          Light_config_again = true;
        }
        else if (p->name() == RGB_L)
        {
          rgbLight = true;
          Light_config_again = true;
        }
        else if (p->name() == UV_I)
        {
          uvIndex = true;
        }
        else if (p->name() == CO2)
        {
          co2 = true;
        }
        else if (p->name() == Voc)
        {
          voc = true;
        }
        else if (p->name() == HUM)
        {
          humidity = true;
        }
        else if (p->name() == HUM_TEMP)
        {
          humidityTemp = true;
        }
        else if (p->name() == S_DIS)
        {
          TOF_config_again = true;
          short_distance = true;
        }
        else if (p->name() == L_DIS)
        {
          TOF_config_again = true;
          long_distance = true;
        }
        else if (p->name() == NOISE)
        {
          noise = true;
        }
        else if (p->name() == PARAM_REFRESH)
        {
          refresh = (p->value()).toInt();
        }
        else if (p->name() == PARAM_start_btn)
        {
          Dprintln("Start exp");
          RGBled.setPixelColor(0, RGBled.Color(0, 255, 0));
          RGBled.setPixelColor(2, RGBled.Color(0, 0, 0));
          RGBled.show();
          start_exp = true;
          resetTime = true;
          sendCSVheader = true;
          STARTbtn = "disabled";
          env_dash_btn = "disabled";
          //          drone_dash_btn = "disabled";
          STOPbtn = "";
        }
        else if (p->name() == PARAM_stop_btn)
        {
          ///  experiment storage will stop because its alredy reset in the start of this function due to that start_exp becomed false
          STOPbtn = "disabled";
          STARTbtn = "";
          env_dash_btn = "";
          //          drone_dash_btn = "";
        }
      }
      request->send(SPIFFS, "/Index.html", String(), false, processor);
    });

    server.on("/get", HTTP_GET, [] (AsyncWebServerRequest * request) {
      if (request->hasParam(PARAM_downlode_btn))
      {
        request->send(SPIFFS, "/EXP_DATA.csv", "text/html", true);
      }
    });

    // all CSS files
    server.on("/css/block.css", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/css/block.css", "text/css");
    });
    server.on("/css/bulb-light.css", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/css/bulb-light.css", "text/css");
    });
    server.on("/css/temp.css", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/css/temp.css", "text/css");
    });
    server.on("/css/header-footer.css", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/css/header-footer.css", "text/css");
    });
    server.on("/css/drone.css", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/css/drone.css", "text/css");
    });
    server.on("/css/ssdbtn.css", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/css/ssdbtn.css", "text/css");
    });


    /// all JS files
    server.on("/js/Thermometer/ajax-library.js", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/js/Thermometer/ajax-library.js", "text/javascript");
    });
    server.on("/js/Thermometer/temp.js", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/js/Thermometer/temp.js", "text/javascript");
    });
    server.on("/js/Gauge/gauge.js-library.js", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/js/Gauge/gauge.js-library.js", "text/javascript");
    });
    server.on("/js/Gauge/gauge.js", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/js/Gauge/gauge.js", "text/javascript");
    });
    server.on("/js/ajax.js", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/js/ajax.js", "text/javascript");
    });



    /// all images
    server.on("/dbl.png", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/dbl.png", "image/png");
    });
    server.on("/co2.png", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/co2.png", "image/png");
    });
    server.on("/ap.png", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/ap.png", "image/png");
    });
    server.on("/dc.png", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/dc.png", "image/png");
    });
    server.on("/fic.png", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/fic.png", "image/png");
    });
    server.on("/h.png", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/h.png", "image/png");
    });
    server.on("/voc.png", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/voc.png", "image/png");
    });
    server.on("/dcico.png", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/dcico.png", "image/png");
    });
    server.on("/edico.png", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/edico.png", "image/png");
    });
    server.on("/dro.png", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/dro.png", "image/png");
    });

    server.on("/index2.html", HTTP_GET, [] (AsyncWebServerRequest * request) {
      if (start_exp)
      {
        //reset_config();
        request->send(SPIFFS, "/Index.html", String(), false, processor);
      }
      else
      {
        RGBled.setPixelColor(0, RGBled.Color(0, 255, 0));
        RGBled.setPixelColor(2, RGBled.Color(0, 0, 0));
        RGBled.show();
        env_dash_en = true;
        Light_config_again = true;
        ambLight = true;
        pressure = true;
        humidity = true;
        voc = true;
        co2 = true;
        externalTemp1 = true;
        request->send(SPIFFS, "/index2.html", String(), false);
      }
    });

    server.on("/co2Val", HTTP_GET, [] (AsyncWebServerRequest * request) {
      //      String tempVal = String(random(1, 10000));
      request->send(200, "text/plain", String(Co2));
    });
    server.on("/vocVal", HTTP_GET, [] (AsyncWebServerRequest * request) {
      //      String tempVal = String(random(1, 50));
      request->send(200, "text/plain", String(VOC));
    });
    server.on("/humVal", HTTP_GET, [] (AsyncWebServerRequest * request) {
      //      String tempVal = String(random(1, 100));
      request->send(200, "text/plain", String(Humidity));
    });
    server.on("/presVal", HTTP_GET, [] (AsyncWebServerRequest * request) {
      //      String tempVal = String(random(1, 100));
      request->send(200, "text/plain", String(Pressure));
    });
    server.on("/lightVal", HTTP_GET, [] (AsyncWebServerRequest * request) {
      //      String tempVal = String(random(1, 1000));
      request->send(200, "text/plain", String(AmbLight));
    });

    server.on("/tmpVal", HTTP_GET, [] (AsyncWebServerRequest * request) {
      //      String tempVal = String(random(-15, 90));
      request->send(200, "text/plain", String(ExtTmp1));
    });
    server.begin();
  }
  else if (Ax < -7) //block mode on
  {
    BLOCKmode = true;
    Dprintln("Block mode on");
    vTaskResume(buzz);
    vTaskResume(led);
    delay(1000);
    vTaskSuspend(buzz);
    noTone(BUZZER_PIN, BUZZER_CHANNEL);
    vTaskSuspend(led);
    RGBled.setPixelColor(0, RGBled.Color(0, 0, 0));
    RGBled.setPixelColor(1, RGBled.Color(0, 0, 0));
    RGBled.setPixelColor(2, RGBled.Color(0, 0, 0));
    RGBled.show();

    //add all block setup code here 
    
    /////////////////////////////////////////////////////////////

  }
  else if (Az > 7 || counter == 4) //BLE mode on
  {
    BLEmode = true;
    Dprintln("Waiting a client connection to notify...");
    initBLE();
  }
  else
  {
    // IMU not proper go back and measure again
    goto here;
  }
}

void form_packet()
{
  if (resetTime)
  {
    resetTime = false;
    timestamp = millis();
  }
  float tm = (millis() - timestamp);
  readTime = tm / time_factor;

  packet[F("a")] = Ax;
  packet[F("b")] = BLight;
  packet[F("c")] = Co2;
  packet[F("d")] = Distance;
  packet[F("e")] = Altitude;
  packet[F("f")] = Az;
  packet[F("g")] = GLight;
  packet[F("h")] = Humidity;
  packet[F("i")] = Mx;
  packet[F("j")] = My;
  packet[F("k")] = Mz;
  packet[F("l")] = AmbLight;
  packet[F("m")] = readTime;
  packet[F("n")] = Noise;
  packet[F("o")] = Itemp;
  packet[F("p")] = Pressure;
  packet[F("q")] = HumTemp;
  packet[F("r")] = RLight;
  packet[F("s")] = Ay;
  packet[F("t")] = ExtTmp1;
  packet[F("u")] = UVindex;
  packet[F("v")] = VOC;
  packet[F("w")] = ExtTmp2;
  packet[F("x")] = Gx;
  packet[F("y")] = Gy;
  packet[F("z")] = Gz;
  packet[F("B")] = batteryVTG;
  packet[F("E")] = ESPchipID;
  packet[F("V")] = firmware_version;
  packet[F("G")] = currentGesture;
  packet[F("R")] = rx_state;
  packet[F("T")] = tx_state;

  if (BLEmode)
  {
    sendPacketEx(dataFormate.c_str(), packet);
  }
  if (sendCSVheader)
  {
    sendCSVheader = false;
    String header = generateCsvHeader();
    Serial.print(header);
    if (WiFimode)
    {
      writeFile(SPIFFS, "/EXP_DATA.csv", header.c_str());
    }
  }

  String sensor_datas = generateCsvRecord();
  Serial.print(sensor_datas);
  used_bytes = SPIFFS.usedBytes();
  if (WiFimode && used_bytes <= total_bytes)
  {
    //Dprintln(used_bytes);
    appendFile(SPIFFS, "/EXP_DATA.csv", sensor_datas.c_str());
  }
}

void sendPacketEx(const char *field_ordering, DynamicJsonDocument &packet) {

  String broadcasted = "";

  JsonObject object = packet.as<JsonObject>();

  for (unsigned int i = 0; i < strlen(field_ordering); i++) {

    String ky = String(field_ordering[i]);

    if (object.containsKey(ky)) {

      JsonVariant jv = object.getMember(ky);

      broadcasted.concat(ky);
      if (ky == "m")
      {
        broadcasted.concat((String(jv.as<float>(), time_decimal)));
      }
      else if (ky == "E" || ky == "V" || ky == "G" || ky == "R" || ky == "T")
      {
        broadcasted.concat(jv.as<String>());
      }
      else
      {
        broadcasted.concat((String(jv.as<float>(), resolution)));
      }
      broadcasted.concat(SPKEX_ENDMARK);
    }
  }

  broadcasted = broadcasted + "\r\n";
  Dprintln(broadcasted);
  sendPacket(broadcasted.c_str());
}

void IRAM_ATTR chargingLED()
{
  if (digitalRead(charge_state) == LOW)
  {
    charging_state = true;
  }
  else
  {
    charging_state = false;
  }
}
