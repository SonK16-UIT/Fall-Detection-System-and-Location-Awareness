//Setting up wifi and firebase
#include <Wire.h>
#include <ESP8266WiFi.h>

#include <FS.h>          // this needs to be first, or it all crashes and burns...
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager  

#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#define API_KEY "AIzaSyANJeQ1QGFfUQ_YxznfFkSwawtbNBrC7lk"
#define DATABASE_URL "https://fetch-35954-default-rtdb.firebaseio.com/"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

//Setting up NEO-6M GPS 
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

static const int RXPin = 13, TXPin = 15; //D7 for TX and D8 for RX
static const uint32_t GPSBaud = 9600;
SoftwareSerial ss(RXPin, TXPin);
TinyGPSPlus gps;

#include <math.h>
float lower_thres,upper_thres;
int angle_fall,angle_stable=10;

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};
float RatePitchroX,RatePitchroY,RatePitchroZ;

bool fall = false;
bool ready = false;
bool trigger1 = false; 
bool trigger2 = false; 
bool trigger3 = false; 
int trigger1count = 0;
int trigger2count = 0; 
int trigger3count = 0; 
float angle_before,angle_after,angle_end;
int count=0,count1=0;

unsigned long sendDataPrevMillis = 0;
const byte button_pin = D4; //external interrupt
boolean message = false;


void IRAM_ATTR got_function() {
  message = true;
  Serial.println("BUTTON PRESSED");
}

void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    // 1. Prediction phase: Predict the current state of the system
    KalmanState = KalmanState + 0.004 * KalmanInput;
    // 2. Update the uncertainty of the prediction
    KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    // 3. Calculate the Kalman gain from the uncertainties on the predictions and measurements
    float measurementError = 3; // Standard deviation of the accelerometer measurement error assumed to be 3 degrees
    float KalmanGain = KalmanUncertainty / (KalmanUncertainty + measurementError * measurementError);
    // 4. Update the predicted state of the system with the measurement of the state through the Kalman gain
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    // 5. Update the uncertainty of the predicted state
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    // Kalman filter output
    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}
void Read_MPU(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t RatePitchroX=Wire.read()<<8 | Wire.read();
  int16_t RatePitchroY=Wire.read()<<8 | Wire.read();
  int16_t RatePitchroZ=Wire.read()<<8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  RateRoll=(float)RatePitchroX/65.5;
  RatePitch=(float)RatePitchroY/65.5;
  RateYaw=(float)RatePitchroZ/65.5;
  AccX=(float)AccXLSB/4096-0.01;
  AccY=(float)AccYLSB/4096+0.01;
  AccZ=(float)AccZLSB/4096-0.06;
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 180 / M_PI;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 180 / M_PI;
}
float Alpha_Degree()
{
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  return KalmanAngleRoll;
}
void fall_noti()
{
  if (Firebase.ready() && (millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();
    if (Firebase.RTDB.setBool(&fbdo, "GPS/Fall", fall)) {
      Serial.print("Fall Status sent to Firebase: ");
      Serial.println(fall ? "True" : "False");
    } else {
      Serial.println("Failed to write fall status to Firebase");
      Serial.println("REASON: " + fbdo.errorReason());
    }
  }
  delay(1000);
}
void coordinate_noti(){
  float latitude = gps.location.lat();
  float longitude = gps.location.lng();
  float altitude = gps.altitude.meters();
  if (Firebase.ready() && (millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();
    Serial.println("Sending coordinates to Firebase");
    if (gps.location.isValid()) {
      if (Firebase.RTDB.setFloat(&fbdo, "GPS/Latitude", latitude)) {
        Serial.print("Latitude: ");
        Serial.println(latitude, 6);
      } 
      if (Firebase.RTDB.setFloat(&fbdo, "GPS/Longitude", longitude)) {
        Serial.print("Longitude: ");
        Serial.println(longitude, 6);
      }
      if (Firebase.RTDB.setFloat(&fbdo, "GPS/Altitude", altitude)) {
        Serial.print("Altitude: ");
        Serial.println(altitude, 6);
      } else {
        Serial.println("Failed to write longitude to Firebase");
        Serial.println("REASON: " + fbdo.errorReason());
      }
    } else {
      Serial.println("Invalid GPS data");
    }
  }
  delay(1000);
}
void Predefine(){
// Retrieve lower threshold value
  if (Firebase.RTDB.getString(&fbdo, "/GPS/lower_threshold")) {
    String stringValueLower = fbdo.stringData();
    if (stringValueLower.length() >= 4) {
      String numericValueLower = stringValueLower.substring(2, stringValueLower.length() - 2);
      lower_thres = numericValueLower.toFloat();
      Serial.print("Lower Threshold: ");
      Serial.println(lower_thres);
    }
  } else {
    Serial.print("Error getting lower threshold: ");
    Serial.println(fbdo.errorReason());
  }

  // Retrieve upper threshold value
  if (Firebase.RTDB.getString(&fbdo, "/GPS/upper_threshold")) {
    String stringValueUpper = fbdo.stringData();
    if (stringValueUpper.length() >= 4) {
      String numericValueUpper = stringValueUpper.substring(2, stringValueUpper.length() - 2);
      upper_thres = numericValueUpper.toFloat();
      Serial.print("Upper Threshold: ");
      Serial.println(upper_thres);
    }
  } else {
    Serial.print("Error getting upper threshold: ");
    Serial.println(fbdo.errorReason());
  }

  // Retrieve basic angle value
  if (Firebase.RTDB.getString(&fbdo, "/GPS/angle_fall")) {
    String stringValueBasicAngle = fbdo.stringData();
    if (stringValueBasicAngle.length() >= 4) {
      String numericValueBasicAngle = stringValueBasicAngle.substring(2, stringValueBasicAngle.length() - 2);
      angle_fall = numericValueBasicAngle.toInt();
      Serial.print("Basic Angle: ");
      Serial.println(angle_fall);
    }
  } else {
    Serial.print("Error getting basic angle: ");
    Serial.println(fbdo.errorReason());
  }
  delay(1000);
}
void IsReady(){
  if (Firebase.ready() && (millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();
    if (Firebase.RTDB.setBool(&fbdo, "GPS/Ready", ready)) {
      Serial.print("Is Device Ready to be use ? ");
      Serial.println(ready ? "YES" : "NO");
    } else {
      Serial.println("Failed to write fall status to Firebase");
      Serial.println("REASON: " + fbdo.errorReason());
    }
  }
  delay(1000);
}
void setup() {
  
  Serial.begin(57600);
  
  WiFiManager wifiManager;
  //exit after config instead of connecting
  wifiManager.setBreakAfterConfig(true);
  //reset settings
  //wifiManager.resetSettings();
  if (!wifiManager.autoConnect("Fall Detection Device","12345678"))
  {
    Serial.println("Failed to connect and hit timeout");
    // If connection fails, reset
    ESP.reset();
    delay(1000);
  }
  Serial.println("WIFI connected)");

  pinMode(button_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button_pin), got_function, FALLING);

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Sign-up successful");
  } else {
    Serial.printf("Sign-up failed: %s\n", config.signer.signupError.message.c_str());
  }
  
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Predefine();

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    Read_MPU();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  ready = false;
  IsReady();
  fall_noti();
}
void loop() {

//MEASURING THRESHOLD//
  /*Read_MPU();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  float Amp = sqrt(pow(abs(AccX) + abs(RateRoll), 2) + pow(abs(AccY) + abs(RatePitch), 2)  + pow(abs(AccZ) + abs(RateYaw), 2));
  Serial.print(Amp);
  Serial.print(" ");
  Serial.println(Alpha_Degree());*/

// FALL ALGORITHM
// Gyroscope calibration
Read_MPU();
RateRoll -= RateCalibrationRoll;
RatePitch -= RateCalibrationPitch;
RateYaw -= RateCalibrationYaw;

// Calculate the angle after 1-second intervals
if (count <= 250) {
  angle_before = Alpha_Degree();
  count++;
}
if (count == 250) {
  Serial.println("Accelerometer range set to: +-8G");
  Serial.println("Gyro range set to: +-500 deg/s");
  Serial.println("----Based Angle Calculated----");
  ready = true;
  IsReady();
}

// Calculate the amplitude (Amp) based on sensor data
float Amp = sqrt(pow(abs(AccX) + abs(RateRoll), 2) + pow(abs(AccY) + abs(RatePitch), 2) + pow(abs(AccZ) + abs(RateYaw), 2));
// Check Amp threshold and activate trigger1 if no other triggers or fall is active
if (Amp >= lower_thres && !trigger2 && !trigger3 && !fall) {
  trigger1 = true;
  Serial.println("TRIGGER 1 ACTIVATED");
}

if (trigger1) {
  trigger1count++;
  if (Amp >= upper_thres) {
    trigger2 = true;
    Serial.println("TRIGGER 2 ACTIVATED");
    trigger1 = false;
    trigger1count = 0;
  }
  // Allow a 0.5-second window for Amp to break the upper threshold for trigger1
  if (trigger1count >= 125) {
    trigger1 = false;
    trigger1count = 0;
    Serial.println("TRIGGER 1 DEACTIVATED");
  }
}

if (trigger2) {
  if (trigger2count < 375) {
    angle_after = Alpha_Degree();
    trigger2count++; 
  } else {
    trigger2count = 0;
    if (abs(angle_before - angle_after) >= angle_fall) {
      trigger3 = true;
      trigger2 = false;
      Serial.println("TRIGGER 3 ACTIVATED");
    } else {
      trigger2 = false;
      Serial.println("TRIGGER 2 DEACTIVATED");
    }
  }
}

if (trigger3) {
  if (trigger3count == 0) {
    delay(3000); // 3-second waiting period
    trigger3count++;
  } else {
    if (count1 < 500) {
      angle_end = Alpha_Degree();
      count1++;
    } else {
      count1 = 0;
      if (abs(angle_end - angle_after) <= angle_stable) {
        Serial.print(angle_end);
        Serial.print(",");
        Serial.println(angle_after);
        fall = true;
        trigger3 = false;
        trigger3count = 0;
        coordinate_noti(); // Send coordinates if a fall might happen
      } else {
        Serial.print(angle_end);
        Serial.print(",");
        Serial.println(angle_after);
        trigger3 = false;
        trigger3count = 0;
        Serial.println("TRIGGER 3 DEACTIVATED");
      }
    }
  }
}

// Handle the detected fall
if (fall) {
  fall_noti();
  Serial.println("FALL DETECTED");
  Serial.println("Withdraw? PRESS THE BUTTON");
  if (message) {
    fall = false;
    delay(500);
    fall_noti();
    message = false;
  }
}

// Delay until the next loop iteration
while (micros() - LoopTimer < 4000);
LoopTimer = micros();
}
