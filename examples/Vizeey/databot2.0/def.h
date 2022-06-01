
ExponentialFilter<float> AltitudeFilter(100, 0);
std::string rxValue;  // used for reading BLE rx buffer
TaskHandle_t buzz;
TaskHandle_t led;

#define SPKEX_ENDMARK ";"
String delimiter =  ",";
String STARTbtn = "";
String STOPbtn = "disabled";
String env_dash_btn = "";
//String drone_dash_btn = "";

bool env_dash_en = false;

bool WiFimode = false;
bool BLEmode = false;
bool BLOCKmode = false;
bool DCmode = false;
bool wifi_no_connect = false;

bool deviceConnected = false;
bool oldDeviceConnected = false;

unsigned long timestamp;
unsigned long data_send_millis;

int refresh = 100;
byte resolution = 2;
int time_factor = 1000;
byte time_decimal = 2;

long total_bytes;  // total SPIFF bytes available
long used_bytes;   // SPIFF used bytes

bool TOF_config_again = false;
bool Light_config_again = false;
bool gestureInt = false;
byte currentGesture;
bool reset_APDS9960 = false;

bool Laccl = false;
bool accl = false;
bool gyroo = false;
bool magneto = false;
bool IMUtemp = false;
bool externalTemp1 = false;
bool externalTemp2 = false;

bool pressure = false;
float altiCalib;
float pastAltiCalib;
float newAltiCalib;

float newHumCalib;
float pastHumCalib;
float HumCalib;

float newTempCalib;
float pastTempCalib;
float TempCalib2 = 0;

char rx_data_char[10];

bool alti = false;
bool ambLight = false;
bool rgbLight = false;
bool uvIndex = false;
bool co2 = false;
bool voc = false;
bool humidity = false;
bool humidityTemp = false;
bool short_distance = false;
bool long_distance = false;
bool noise = false;
bool gesture = false;

bool usbCheck = false;
bool sysCheck = false;
bool altitudeCalibrate = false;
bool humCalib = false;
bool DtmpCal = false;


boolean charging_state = false;
float batteryVTG;
String ESPchipID;

float A_LA,A_A,LAx, LAy, LAz, Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz, Itemp, ExtTmp1, ExtTmp2;
float Pressure, Altitude, Co2, VOC, Humidity, HumTemp, Distance, Noise;
uint16_t AmbLight, RLight, GLight, BLight;
byte UVindex;
byte rx_state, tx_state;
float readTime;
String dataFormate;

int breadhColor = 0;
bool brighter = false;
unsigned long blink_milli;
unsigned long delay_milli;

boolean config_done = false;
boolean start_exp = false;
boolean resetTime = false;
boolean sendCSVheader = false;

String WiFi_SSID, WiFi_PSS;

const char* PARAM_wifi_ssid = "ssid";
const char* PARAM_wifi_pss = "pass";
const char* PARAM_downlode_btn = "DOWNLOAD";
const char* PARAM_start_btn = "START";
const char* PARAM_stop_btn = "STOP";
const char* PARAM_REFRESH = "REFERESH";
const char* L_ACCL = "laccl";
const char* ACCL = "accl";
const char* GYRO = "gyro";
const char* MAGNETO = "magneto";
const char* IMU_TEMP = "IMUtemp";
const char* EX_TEMP1 = "externalTemp1";
const char* EX_TEMP2  = "externalTemp2";
const char* PRESS  = "pressure";
const char* ALTI  = "alti";
const char* AMB_L  = "ambLight";
const char* RGB_L  = "rgbLight";
const char* UV_I  = "uvIndex";
const char* CO2  = "co2";
const char* Voc  = "voc";
const char* HUM  = "humidity";
const char* HUM_TEMP  = "humidityTemp";
const char* S_DIS  = "short_distance";
const char* L_DIS  = "long_distance";
const char* NOISE  = "noise";
const char* M1 = "m1";
const char* M2 = "m2";
const char* M3 = "m3";
const char* M4 = "m4";



//String ssid = "databot_" + String(ESP_getChipId(), HEX);
String ssid = "databot_" + String(ESP_getChipId());
const char* password = "";




//////////////////////////////////////////////////////////////////////////////////////////////for  melody

// change this to make the song slower or faster
int tempo = 105;

// notes of the moledy followed by the duration.
// a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
// !!negative numbers are used to represent dotted notes,
// so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!!
int melody[] = {

  // Pacman
  // Score available at https://musescore.com/user/85429/scores/107109
  NOTE_B4, 16, NOTE_B5, 16, NOTE_FS5, 16, NOTE_DS5, 16, //1
  NOTE_B5, 32, NOTE_FS5, -16, NOTE_DS5, 8, NOTE_C5, 16,
  NOTE_C6, 16, NOTE_G6, 16, NOTE_E6, 16, NOTE_C6, 32, NOTE_G6, -16, NOTE_E6, 8,

  NOTE_B4, 16,  NOTE_B5, 16,  NOTE_FS5, 16,   NOTE_DS5, 16,  NOTE_B5, 32,  //2
  NOTE_FS5, -16, NOTE_DS5, 8,  NOTE_DS5, 32, NOTE_E5, 32,  NOTE_F5, 32,
  NOTE_F5, 32,  NOTE_FS5, 32,  NOTE_G5, 32,  NOTE_G5, 32, NOTE_GS5, 32,  NOTE_A5, 16, NOTE_B5, 8
};

// sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
// there are two values per note (pitch and duration), so for each note there are four bytes
int notes = sizeof(melody) / sizeof(melody[0]) / 2;

// this calculates the duration of a whole note in ms
int wholenote = (60000 * 4) / tempo;

int divider = 0, noteDuration = 0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////new IMU library code
ArduinoICM20948 IMU;
ArduinoICM20948Settings icmSettings =
{
  .i2c_speed = 400000,                // i2c clock speed
  .is_SPI = false,                    // Enable SPI, if disable use i2c
  .cs_pin = 10,                       // SPI chip select pin
  .spi_speed = 7000000,               // SPI clock speed in Hz, max speed is 7MHz
  .mode = 1,                          // 0 = low power mode, 1 = high performance mode
  .enable_gyroscope = true,           // Enables gyroscope output
  .enable_accelerometer = true,       // Enables accelerometer output
  .enable_magnetometer = true,        // Enables magnetometer output // Enables quaternion output
  .enable_gravity = false,             // Enables gravity vector output
  .enable_linearAcceleration = true,  // Enables linear acceleration output
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
