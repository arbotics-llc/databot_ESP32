/*
   This example demonstrate how to read gesture value from databot2.0 gesture sensor
*/
#include<databot2.h>
Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(3, LED_PIN, NEO_GRB + NEO_KHZ800);

SparkFun_APDS9960 apds = SparkFun_APDS9960();   // light sensor object

bool gestureInt = false; //used to detect gesture data is available or not
int currentGesture =0;  //variable use to store the gesture value

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
  RGBled.setPixelColor(0, RGBled.Color(0, 0, 255));  // LED indication to know that databot is ON
  RGBled.show();
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
  delay(100);
}

void interruptRoutine() // This function will be called when Gesture data is ready and Interrupt arrives
{
  if (digitalRead(APDS9960_INT) == LOW)
  {
    gestureInt = true;
  }
}
