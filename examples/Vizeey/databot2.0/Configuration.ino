
void callback_Rx()
{

  if (!config_done)
  {

    //    doc.clear();
    StaticJsonDocument<1500> doc;
    DeserializationError error = deserializeJson(doc, rxValue);

    if (error)
    {
      Dprint(F("deserializeJson() failed: "));
      Dprintln(error.c_str());
    }
    else
    {
      config_done = true;   // configuration data is received
      refresh = doc["refresh"];
      resolution = doc["decimal"];
      time_factor = doc["timeFactor"];
      time_decimal = doc["timeDec"];
      Laccl = doc["Laccl"];
      accl = doc["accl"];
      gyroo = doc["gyro"];
      magneto = doc["magneto"];
      IMUtemp = doc["IMUtemp"];
      externalTemp1 = doc["Etemp1"];
      externalTemp2 = doc["Etemp2"];
      pressure = doc["pressure"];
      Ptemp = doc["Ptemp"];
      alti = doc["alti"];
      ambLight = doc["ambLight"];
      rgbLight = doc["rgbLight"];
      uvIndex = doc["UV"];
      co2 = doc["co2"];
      voc = doc["voc"];
      humidity = doc["hum"];
      humidityTemp = doc["humTemp"];
      Agyro = doc["Agyro"];
      heat_index1 = doc["HI1"];
      if (heat_index1)
      {
        humidity = true;
        externalTemp1 = true;
      }

      short_distance = doc["Sdist"];
      long_distance = doc["Ldist"];
      noise = doc["noise"];
      gesture = doc["gesture"];
      usbCheck = doc["usbCheck"];
      sysCheck = doc["sysCheck"];
      altitudeCalibrate = doc["altCalib"];
      humCalib = doc["humCalib"];
      DtmpCal = doc["DtmpCal"];
      JsonObject buz = doc["buz"];
      bool buzzz = buz["state"];
      if (buzzz)
      {
        int frequency =  buz["f"];
        int duration = buz["d"];
        tone(BUZZER_PIN, frequency, duration, BUZZER_CHANNEL);
      }
      JsonObject led1 = doc["led1"];
      bool led1_state = led1["state"]; // true
      if (led1_state)
      {
        int led1_R = led1["R"]; // 255
        int led1_Y = led1["Y"]; // 0
        int led1_B = led1["B"]; // 0
        RGBled.setPixelColor(0, RGBled.Color(led1_R, led1_Y, led1_B));
      }
      else
      {
        RGBled.setPixelColor(0, RGBled.Color(0, 0, 0));
      }

      JsonObject led2 = doc["led2"];
      bool led2_state = led2["state"]; // true
      if (led2_state)
      {
        int led2_R = led2["R"]; // 0
        int led2_Y = led2["Y"]; // 255
        int led2_B = led2["B"]; // 0
        RGBled.setPixelColor(1, RGBled.Color(led2_R, led2_Y, led2_B));
      }
      else
      {
        RGBled.setPixelColor(1, RGBled.Color(0, 0, 0));
      }

      JsonObject led3 = doc["led3"];
      bool led3_state = led3["state"]; // true
      if (led3_state)
      {
        int led3_R = led3["R"]; // 0
        int led3_Y = led3["Y"]; // 0
        int led3_B = led3["B"]; // 255
        RGBled.setPixelColor(2, RGBled.Color(led3_R, led3_Y, led3_B));
      }
      else
      {
        RGBled.setPixelColor(2, RGBled.Color(0, 0, 0));
      }
      RGBled.show();

      if (short_distance || long_distance)
      {
        TOF_config_again = true;
      }

      if (ambLight || rgbLight || gesture)
      {
        Light_config_again = true;
      }

      if (altitudeCalibrate || humCalib || DtmpCal)
      {
        Dprintln("Start exp");
        start_exp = true;
        resetTime = true;
      }
      sendCSVheader = true;
    }
  }
  else
  {
    if (rxValue == "1.0" && !start_exp)
    {
      Dprintln("Start exp");
      start_exp = true;
      resetTime = true;
    }
    else if (altitudeCalibrate)
    {
      //      rx_data_char = rxValue.c_str();
      strcpy(rx_data_char, rxValue.c_str());
      newAltiCalib = atof(rx_data_char);

      if (newAltiCalib != pastAltiCalib)
      {
        pastAltiCalib = newAltiCalib;
        Dprint("Calibrate alti to: ");
        Dprintln(pastAltiCalib);

        float tempAlti = Altitude + altiCalib;
        Dprint("Current alti: ");
        Dprintln(tempAlti);

        altiCalib = tempAlti - pastAltiCalib;

        Dprint("Calibration value: ");
        Dprintln(altiCalib);
        EEPROMWritelong(10, altiCalib * 100);
      }
    }
    else if (humCalib)
    {
      strcpy(rx_data_char, rxValue.c_str());
      newHumCalib = atof(rx_data_char);

      if (newHumCalib != pastHumCalib)
      {
        pastHumCalib = newHumCalib;
        Dprint("Calibrate Humi to: ");
        Dprintln(pastHumCalib);

        float tempHum = Humidity + HumCalib;
        Dprint("Current hum: ");
        Dprintln(tempHum);

        HumCalib = tempHum - pastHumCalib;

        Dprint("Calibration value: ");
        Dprintln(HumCalib);
        EEPROMWritelong(20, HumCalib * 100);
      }
    }
    else if (DtmpCal)
    {
      if (rxValue == "1.0")
      {
        newTempCalib = ExtTmp1;
        if (newTempCalib != pastTempCalib)
        {
          pastTempCalib = newTempCalib;
          Dprint("Calibrate Temp to: ");
          Dprintln(pastTempCalib);
          float tem_T2 = ExtTmp2 + TempCalib2;
          Dprint("Current temp1: ");
          Dprintln(tem_T2);
          TempCalib2 = tem_T2 - pastTempCalib;
          Dprint("Calibration value T2: ");
          Dprintln(TempCalib2);
        }
      }
    }
  }
}
