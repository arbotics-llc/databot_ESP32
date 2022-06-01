/*
   This example demonstrate how to read gesture value from databot2.0 gesture sensor
   And based on gesture it will change its LED color
*/
#include<databot2.h>
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(3, LED_PIN, NEO_GRB + NEO_KHZ800);

SparkFun_APDS9960 apds = SparkFun_APDS9960();   // light sensor object

bool gestureInt = false; //used to detect gesture data is available or not
int currentGesture = 0; //variable use to store the gesture value

void setup() {

  Serial.begin(9600);
  pinMode(APDS9960_INT, INPUT);
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init()) {
    Serial.println("APDS-9960 initialization complete");
  } else {
    Serial.println("Something went wrong during APDS-9960 init!");
  }

  apds.setAmbientLightGain(0);
  apds.setGestureGain(0);
  apds.setLEDDrive(3);
  apds.setGestureLEDDrive(3);
  attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);
  if ( apds.enableGestureSensor(true))
  {
    Serial.println("Gesture sensor is now running");
  }
  else
  {
    Serial.println("Something went wrong during gesture sensor init!");
  }
}

void loop() {
  if ( gestureInt)
  {
    detachInterrupt(APDS9960_INT);
    gestureInt = false;
    if ( apds.isGestureAvailable() )
    {
      switch ( apds.readGesture() ) {
        case DIR_UP:
          {
            Serial.println("LEFT");
            currentGesture = 1;
          }
          break;
        case DIR_DOWN:
          {
            Serial.println("RIGHT");
            currentGesture = 2;
          }
          break;
        case DIR_LEFT:
          {
            Serial.println("DOWN");
            currentGesture = 3;
          }
          break;
        case DIR_RIGHT:
          {
            Serial.println("UP");
            currentGesture = 4;
          }
          break;
        case DIR_NEAR:
          {
            Serial.println("NEAR");
            currentGesture = 5;
          }
          break;
        case DIR_FAR:
          {
            Serial.println("FAR");
            currentGesture = 6;
          }
          break;
        default:
          {
            Serial.println("NONE");
            currentGesture = 0;
          }
      }
    }
    attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);
  }
  if (currentGesture == 1) {
    RGBled.setPixelColor(0, RGBled.Color(255, 0, 0));
    RGBled.show();
  } else if (currentGesture == 2) {
    RGBled.setPixelColor(0, RGBled.Color(51, 255, 51));
    RGBled.show();
  } else if (currentGesture == 3) {
    RGBled.setPixelColor(0, RGBled.Color(51, 51, 255));
    RGBled.show();
  } else if (currentGesture == 4) {
    RGBled.setPixelColor(0, RGBled.Color(255, 255, 0));
    RGBled.show();
  } else if (currentGesture == 5) {
    RGBled.setPixelColor(0, RGBled.Color(255, 102, 0));
    RGBled.show();
  } else if (currentGesture == 6) {
    RGBled.setPixelColor(0, RGBled.Color(204, 51, 204));
    RGBled.show();
  } else {
    RGBled.setPixelColor(0, RGBled.Color(255, 255, 255));
    RGBled.show();
  }

  delay(100);
}

void interruptRoutine() // This function will be called when Gesture data is ready and Interrupt arrives
{
  if (digitalRead(APDS9960_INT) == LOW)
  {
    gestureInt = true;
  }
}
