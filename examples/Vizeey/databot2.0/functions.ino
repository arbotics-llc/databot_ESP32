void initIMU()
{
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

  //  if (accl && gyroo && magneto && !Laccl)
  //  {
  //    Dprintln("AGM measure");
  //    //    IMU_read(IMU, Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz, Itemp);  // in new lib not available
  //    IMU_read(IMU, Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz);
  //    dataFormate += "asfxyzijk";
  //  }
  //  else
  //  {
  if (accl)
  {
    dataFormate += "asfA";
    if (new_IMU)
    {
      IMU_read_accl(IMU1, Ay, Ax, Az);
    }
    else
    {
      IMU_read_accl(IMU, Ax, Ay, Az);
    }
    A_A = sqrt((Ax * Ax) + (Ay * Ay) + (Az * Az));
  }
  else
  {
    Ax = 0;
    Ay = 0;
    Az = 0;
    A_A = 0;
  }
  if (gyroo)
  {
    dataFormate += "xyz";
    if (new_IMU)
    {
      IMU_read_gyro(IMU1,  Gx, Gy, Gz);
    }
    else
    {
      IMU_read_gyro(IMU,  Gx, Gy, Gz);
    }
  }
  else
  {
    Gx = 0;
    Gy = 0;
    Gz = 0;
  }

  if (Agyro) // absalute angular velocity
  {
    dataFormate += "M";
    if (new_IMU)
    {
      IMU_read_gyro(IMU1,  Gx, Gy, Gz);
    }
    else
    {
      IMU_read_gyro(IMU,  Gx, Gy, Gz);
    }

    absoluteAngularVelocity = sqrt(pow(Gx, 2) + pow(Gy, 2) + pow(Gz, 2));

    /*


        // Calculate time since last loop iteration
        dt = (millis() - Agyro_millis) / 1000.0;
        Agyro_millis = millis();
        // Integrate gyro data to get angular velocity
        float deltaGyroX = (Gx + prevGyroX) / 2.0 * dt;
        float deltaGyroY = (Gy + prevGyroY) / 2.0 * dt;
        float deltaGyroZ = (Gz + prevGyroZ) / 2.0 * dt;

        absoluteAngularVelocity += sqrt(deltaGyroX * deltaGyroX + deltaGyroY * deltaGyroY + deltaGyroZ * deltaGyroZ);

        // Store current gyro values for next iteration
        prevGyroX = Gx;
        prevGyroY = Gy;
        prevGyroZ = Gz;
    */
  }
  else
  {
    absoluteAngularVelocity = 0;
  }


  if (magneto)
  {
    dataFormate += "ijk";
    if (new_IMU)
    {
      IMU_read_magneto(IMU1, Mx, My, Mz);
    }
    else
    {
      IMU_read_magneto(IMU, Mx, My, Mz);
    }
  }
  else
  {
    Mx = 0;
    My = 0;
    Mz = 0;
  }
  if (Laccl)
  {
    dataFormate += "XYZL";
    if (new_IMU)
    {
      IMU_read_LinearAccl(IMU1, LAy, LAx, LAz);
    }
    else
    {
      IMU_read_LinearAccl(IMU, LAx, LAy, LAz);
    }
    A_LA = sqrt((LAx * LAx) + (LAy * LAy) + (LAz * LAz));
  }
  else
  {
    LAx = 0;
    LAy = 0;
    LAz = 0;
    A_LA = 0;
  }
  //  }

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

  if (heat_index1)
  {
    dataFormate += "I";
    heat_index = get_heatIndex(ExtTmp1, humidity);
  }
  else
  {
    heat_index = 0;
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

  if (Ptemp)
  {
    dataFormate += "P";
    PreTemp = getPressure_temperature() - 18.5; // will get pressure in Kpa so *10 for hPa
  }
  else
  {
    PreTemp = 0;
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
    if (gestureInt)
    {
      detachInterrupt(APDS9960_INT);
      gestureInt = false;
      if ( apds.isGestureAvailable() )
      {
        switch ( apds.readGesture() ) {
          case DIR_UP:
            {
              Dprintln("LEFT");
              currentGesture = 1;
            }
            break;
          case DIR_DOWN:
            {
              Dprintln("RIGHT");
              currentGesture = 2;
            }
            break;
          case DIR_LEFT:
            {
              Dprintln("DOWN");
              currentGesture = 3;
            }
            break;
          case DIR_RIGHT:
            {
              Dprintln("UP");
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

  //  if (buzzz && start_exp)
  //  {
  //    vTaskResume(buzz);
  //  }
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
    csvHeader += delimiter + "A.x" + delimiter + "A.y" + delimiter + "A.z" + delimiter + "A.a";
  }
  if (Laccl)
  {
    csvHeader += delimiter + "LA.x" + delimiter + "LA.y" + delimiter + "LA.z" + delimiter + "LA.a";
  }
  if (gyroo)
  {
    csvHeader += delimiter + "G.x" + delimiter + "G.y" + delimiter + "G.z";
  }
  if (Agyro)
  {
    csvHeader += delimiter + "AGyro";
  }
  if (magneto)
  {
    csvHeader += delimiter + "M.x" + delimiter + "M.y" + delimiter + "M.z";
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
    csvHeader += delimiter + "Pressure";
  }

  if (Ptemp)
  {
    csvHeader += delimiter + "Ambient Temperature";
  }

  if (alti)
  {
    csvHeader += delimiter + "Altitude";
  }

  if (ambLight)
  {
    csvHeader += delimiter + "LUX";
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
    csvHeader += delimiter + "Humidity";
  }
  if (humidityTemp)
  {
    csvHeader += delimiter + "HUM Temp";
  }
  if (heat_index1)
  {
    csvHeader += delimiter + "Heat Index";
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
    csvHeader += delimiter + "dB";
  }
  csvHeader.trim();
  csvHeader += "\r\n";

  return csvHeader;

}

String generateCsvRecord() {
  String csvRecord = String(readTime);

  if (accl)
  {
    csvRecord += delimiter + Ax + delimiter + Ay + delimiter + Az + delimiter + A_A;
  }
  if (Laccl)
  {
    csvRecord += delimiter + LAx + delimiter + LAy + delimiter + LAz + delimiter + A_LA;
  }
  if (gyroo)
  {
    csvRecord += delimiter + Gx + delimiter + Gy + delimiter + Gz;
  }
  if (Agyro)
  {
    csvRecord += delimiter + absoluteAngularVelocity;
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

  if (Ptemp)
  {
    csvRecord += delimiter + PreTemp;
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
  if (heat_index1)
  {
    csvRecord += delimiter + heat_index;
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
  //  if (buzzz)
  //  {
  //    vTaskSuspend(buzz);
  //    noTone(BUZZER_PIN, BUZZER_CHANNEL);
  //  }
  noTone(BUZZER_PIN, BUZZER_CHANNEL);
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


  Laccl = false;
  accl = false;
  gyroo = false;
  Agyro = false;
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
  heat_index1 = false;

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
      //      if (breadhColor < 255 && !DCmode)
      if (breadhColor < 255)
      {
        breadhColor++;
      }
      //      else if (breadhColor < 128 && DCmode)
      //      {
      //        breadhColor++;
      //      }
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
    //    else if (DCmode)
    //    {
    //      RGBled.setPixelColor(0, RGBled.Color(breadhColor, 0, breadhColor));
    //    }

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
  delay(50);

  if (new_IMU)
  {
    IMU_read_accl(IMU1, Ax, Ay, Az);
  }
  else
  {
    IMU_read_accl(IMU, Ax, Ay, Az);
  }

  //  IMU_read_accl(IMU, Ax, Ay, Az);
  Dprint("Ax:");
  Dprintln(Ax);
  Dprint("Ay:");
  Dprintln(Ay);
  Dprint("Az:");
  Dprintln(Az);

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

    //    if (!WiFi.softAPConfig(AP_LOCAL_IP, AP_GATEWAY_IP, AP_NETWORK_MASK)) {
    //      Dprintln("AP Config Failed");
    //      return;
    //    }
    if (!WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0))) {
      Dprintln("AP Config Failed");
      return;
    }

    Dprint("IP address: ");
    Dprintln(WiFi.softAPIP());
    // if DNSServer is started with "*" for domain name, it will reply with
    // provided IP to all DNS request
    dnsServer.start(DNS_PORT, "webserver.databot.com", apIP);

    delay(1000);
    // simple HTTP server to see that DNS server is working
    server.onNotFound([](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/Index.html", String(), false, processor);
    });

    /*
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
    */
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
        else if (p->name() == L_ACCL)
        {
          Laccl = true;
        }
        else if (p->name() == GYRO)
        {
          gyroo = true;
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
        else if (p->name() == AMBIENT_T)
        {
          pressure = true;
          Ptemp = true;
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

    server.on("/wifi", HTTP_GET, [] (AsyncWebServerRequest * request) {
      int params = request->params();
      for (int i = 0; i < params; i++) {
        AsyncWebParameter* p = request->getParam(i);
        // p->value().c_str());
        if (p->name() == PARAM_wifi_ssid)
        {
          WiFi_SSID = p->value();
          Dprintln(WiFi_SSID);
          EEPROMwriteString(WiFi_SSID.c_str(), 30);
        }
        else if (p->name() == PARAM_wifi_pss)
        {
          WiFi_PSS = p->value();
          Dprintln(WiFi_PSS);
          EEPROMwriteString(WiFi_PSS.c_str(), 60);
        }
      }
      request->send(SPIFFS, "/Index.html", String(), false, processor);
    });

    // all CSS files
    server.serveStatic("/css/block.css", SPIFFS, "/css/block.css").setCacheControl("max-age=3600");
    server.serveStatic("/css/bulb-light.css", SPIFFS, "/css/bulb-light.css").setCacheControl("max-age=3600");
    server.serveStatic("/css/temp.css", SPIFFS, "/css/temp.css").setCacheControl("max-age=3600");
    server.serveStatic("/css/header-footer.css", SPIFFS, "/css/header-footer.css").setCacheControl("max-age=3600");
    server.serveStatic("/css/drone.css", SPIFFS, "/css/drone.css").setCacheControl("max-age=3600");
    server.serveStatic("/css/ssdbtn.css", SPIFFS, "/css/ssdbtn.css").setCacheControl("max-age=3600");

    /*
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
    */

    /// all JS files
    /*
      server.serveStatic("/ajax-library.js", SPIFFS, "/ajax-library.js").setCacheControl("max-age=3600");
      server.serveStatic("/temp.js", SPIFFS, "/temp.js").setCacheControl("max-age=3600");
      server.serveStatic("/gauge_lib.js", SPIFFS, "/gauge_lib.js").setCacheControl("max-age=3600");
      server.serveStatic("/gauge.js", SPIFFS, "/gauge.js").setCacheControl("max-age=3600");
      server.serveStatic("/ajax.js", SPIFFS, "/ajax.js").setCacheControl("max-age=3600");
    */
    /*
        server.on("/ajax-library.js", HTTP_GET, [](AsyncWebServerRequest * request) {
          request->send(SPIFFS, "/ajax-library.js", "text/javascript");
        });
        server.on("/temp.js", HTTP_GET, [](AsyncWebServerRequest * request) {
          request->send(SPIFFS, "/temp.js", "text/javascript");
        });
        server.on("/gauge_lib.js", HTTP_GET, [](AsyncWebServerRequest * request) {
          request->send(SPIFFS, "/gauge_lib.js", "text/javascript");
        });
        server.on("/gauge.js", HTTP_GET, [](AsyncWebServerRequest * request) {
          request->send(SPIFFS, "/gauge.js", "text/javascript");
        });
        server.on("/ajax.js", HTTP_GET, [](AsyncWebServerRequest * request) {
          request->send(SPIFFS, "/ajax.js", "text/javascript");
        });*/

    /*
        server.on("/js/Thermometer/ajax-library.js", HTTP_GET, [](AsyncWebServerRequest * request) {
          request->send(SPIFFS, "/js/Thermometer/ajax-library.js", "text/javascript");
          //      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/js/Thermometer/ajax-library.js.gz", "text/javascript", false);
          //      response->addHeader("Content-Encoding", "gzip");
          //      request->send(response);
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
    */


    /// all images
    server.serveStatic("/s.png", SPIFFS, "/s.png").setCacheControl("max-age=36000");
    server.serveStatic("/s2.png", SPIFFS, "/s2.png").setCacheControl("max-age=36000");
    server.serveStatic("/fic.png", SPIFFS, "/fic.png").setCacheControl("max-age=36000");
    /*
      server.on("/s.png", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/s.png", "image/png");
      });
      server.on("/s2.png", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/s2.png", "image/png");
      });
      server.on("/fic.png", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(SPIFFS, "/fic.png", "image/png");
      });
    */
    /*
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
    */
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
  //  else if (Ax < -7) //block mode on
  //  {
  //    BLOCKmode = true;
  //
  //    Dprintln("Block mode on");
  //    vTaskResume(buzz);
  //    vTaskResume(led);
  //    delay(1000);
  //    vTaskSuspend(led);
  //    vTaskSuspend(buzz);
  //    noTone(BUZZER_PIN, BUZZER_CHANNEL);
  //
  //
  //
  //    RGBled.setPixelColor(0, RGBled.Color(0, 0, 0));
  //    RGBled.setPixelColor(1, RGBled.Color(0, 0, 0));
  //    RGBled.setPixelColor(2, RGBled.Color(0, 0, 0));
  //    RGBled.show();
  //
  //    delay(300);
  //    //add all block setup code here
  //    /////////////////////////////////////////////////////////////
  //  }
  //  else if (Ax > 7)
  //  {
  //    DCmode = true;
  //    WiFi.begin(WiFi_SSID.c_str(), WiFi_PSS.c_str());
  //    unsigned long tempmillis = millis();
  //    while (WiFi.status() != WL_CONNECTED && millis() - tempmillis <= 20000 ) {
  //      Dprint(".");
  //      LED_breadh();
  //    }
  //    if (WiFi.status() == WL_CONNECTED)
  //    {
  //      Dprintln("**");
  //      RGBled.setPixelColor(0, RGBled.Color(128, 0, 128));
  //      RGBled.show();
  //    }
  //  }
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
  packet[F("X")] = LAx;
  packet[F("Y")] = LAy;
  packet[F("Z")] = LAz;
  packet[F("A")] = A_A;   // absalute accleration
  packet[F("L")] = A_LA;  // absalute linear accleration
  packet[F("P")] = PreTemp; // pressure temperature
  packet[F("I")] = heat_index; // heat index
  packet[F("M")] = absoluteAngularVelocity; // absalute gyro angular velocity

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
