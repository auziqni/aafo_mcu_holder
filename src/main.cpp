#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <PID_v1_bc.h> // Include the PID library
// teknisee
//-----------------------------------------------------------------------------------------------
#include <WiFi.h>          // import wifi library
#include <HTTPClient.h>    // import http library
#include <ArduinoJson.h>   // import arduino json library
#include <FirebaseESP32.h> // import firebase library

// main static global variable
String norekam = "rokam";
// const char *ssid = "teknisee";
// const char *password = "myteknisee";
const char *ssid = "Gei part 3";
const char *password = "SIMIONE19";

#define FIREBASE_HOST "https://aafo-9b7b2-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "lWOv4jbAHmdJHW3Hi6bvqenkFnOVos0psAeNUCR2"

// main dynamcic global variable
int lastmillis;
String jsonReqPayload = "";
float pitch;
int fsr1value, fsr2value;

// global varialbe firebase
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

//-----------------------------------------------------------------------------------------------

const int fsrPin1 = 35;
const int fsrPin2 = 32;

double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;
double setpoint = 75.00;
bool sessionRead = false;
int timessessionRead = 5, bacaSession = 0;

MPU6050 mpu;
// MPU6050 mpu(0x69);
Servo rightFootServo;

bool blinkState = false;

// MPU control
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float ypr[3];

#define INTERRUPT_PIN 5
int i = 0;
int mpu_offset[6] = {-2778, -838, -1234, 789, 93, -3};
int initialAngle = 0; // sudut awal kaki kanan (netral)
int minAngle = -70;   // sudut minimum gerakan kaki kanan
int maxAngle = 90;    // sudut maksimum gerakan kaki kanan
int step = 10;        // increment sudut per langkah

float yaw, roll, servoAngle;

volatile bool mpuInterrupt = false;

// PID control variables
double Setpoint, Input, Output;
double Kp = 1.0, Ki = 0.1, Kd = 0.01; // PID constants
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
double titikawalmotor = 00.0;
double titikawalmpu6050 = 00.0;

int mpuConected = 0;

// teknisee function
//-----------------------------------------------------------------------------------------------

void reconnectWifi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("WIFI NOT CONNECTED");
    Serial.print("Connecting to ");
    Serial.println(ssid);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
  }
}

void initWifi()
{
  WiFi.begin(ssid, password);

  reconnectWifi();

  Serial.println("");
  Serial.print("wifi connected. ");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("");
}

void initFirebase()
{
  config.database_url = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.begin(&config, &auth);
}

bool writeToFirebase(const char *path, const String &value, const String &typedata)
{
  bool success = false;

  if (typedata == "int")
  {
    success = Firebase.setInt(fbdo, path, value.toInt());
  }
  else if (typedata == "float")
  {
    success = Firebase.setFloat(fbdo, path, value.toFloat());
  }
  else if (typedata == "bool")
  {
    success = Firebase.setBool(fbdo, path, value.equalsIgnoreCase("true"));
  }
  else if (typedata == "string")
  {
    success = Firebase.setString(fbdo, path, value);
  }
  else
  {
    // Tipe data tidak dikenal
    Serial.println("Tipe data tidak dikenal");
    return false;
  }

  if (!success)
  {
    Serial.println("Gagal menulis data ke Firebase");
    Serial.println(fbdo.errorReason());
    return false;
  }

  return success;
}

void writeFirebase()
{
  if (Firebase.ready())
  {
    writeToFirebase("/data1/beratBelakang", String(fsr2value), "float");
    writeToFirebase("/data1/beratDepan", String(fsr1value), "float");
    writeToFirebase("/data1/sudut", String(pitch), "float");
  }
  else
  {
    Serial.println("Firebase not ready");
  }
}

String generateRandomString()
{
  String result = "";
  String characters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

  for (int i = 0; i < 5; i++)
  {
    int randomIndex = random(0, characters.length());
    result += characters[randomIndex];
  }

  return result;
}

void readFirebase()
{
  if (Firebase.ready())
  {
    Firebase.getBool(fbdo, "data1/sessionRead") && (sessionRead = fbdo.boolData());
    if (sessionRead)
    {
      norekam = generateRandomString();
      bacaSession = timessessionRead;
      writeToFirebase("/data1/sessionRead", String(false), "bool");
    }
  }
  else
  {
    Serial.println("Firebase not ready");
  }
}

void data2json()
{
  jsonReqPayload = "";
  StaticJsonDocument<200> D2W;

  D2W["norekam"] = norekam;
  D2W["sudut"] = pitch;
  D2W["beratDepan"] = fsr1value;
  D2W["beratBelakang"] = fsr2value;

  serializeJson(D2W, jsonReqPayload);
  // Serial.println(jsonReqPayload);
}

void send2psql()
{
  HTTPClient http;

  http.begin("https://www.aafo.teknisee.store/api/postpengukuran");
  http.addHeader("Content-Type", "application/json; charset=UTF-8");
  int httpCode = http.POST(jsonReqPayload);

  if (httpCode > 0)
  {
    // file found at server
    if (httpCode == HTTP_CODE_OK)
    {
      String payload = http.getString();
      Serial.println(payload);
    }
    else
    {
      // HTTP header has been send and Server response header has been handled
      Serial.printf("[HTTP] GET... code: %d\n", httpCode);

      send2psql();
    }
    delay(3000);
  }
  else
  {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    send2psql();
    delay(3000);
  }

  http.end();
}

// teknisee funtion end
//-----------------------------------------------------------------------------------------------

void dmpDataReady()
{
  mpuInterrupt = true;
}

void print_yaw_pitch_roll()
{
  // Serial.print("data : ");
  // Serial.print("yaw : ");
  // Serial.print(yaw);
  Serial.print("kode : ");
  Serial.print(norekam);
  Serial.print("\n");

  if (bacaSession > 0)
  {
    Serial.print("sesion dimulai, sisa : ");
    Serial.print(bacaSession);
  }
  else
  {
    Serial.print("sesion selesai, sisa : ");
    Serial.print(bacaSession);
  }

  Serial.print("\tpitch : ");
  Serial.print(pitch);
  // Serial.print("\troll : ");
  // Serial.print(roll);
  Serial.print("\tFSR 1 Value: ");
  Serial.print(fsr1value);
  Serial.print("\tFSR 2 Value: ");
  Serial.println(fsr2value);
}

double pid(double error)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (Kp * proportional) + (Ki * integral) + (Kd * derivative);
  return output;
}

void MPU_Connect()
{
  // MPU initialization
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(mpu_offset[0]);
  mpu.setYAccelOffset(mpu_offset[1]);
  mpu.setZAccelOffset(mpu_offset[2]);
  mpu.setXGyroOffset(mpu_offset[3]);
  mpu.setYGyroOffset(mpu_offset[4]);
  mpu.setZGyroOffset(mpu_offset[5]);

  if (devStatus == 0)
  {
    mpu.CalibrateAccel(10);
    mpu.CalibrateGyro(10);
    mpu.PrintActiveOffsets();

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void setup()
{
  Wire.begin();
  Wire.setClock(400000);
  initWifi(); // it must connect to wifi, unless it will not continue to the next line
  initFirebase();

  rightFootServo.attach(2);
  rightFootServo.write(0);

  Serial.begin(115200);

  // PID setup
  Setpoint = initialAngle; // Desired angle
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(step);
  myPID.SetOutputLimits(minAngle, maxAngle); // Limit the output to servo angle range

  // MPU initialization
  while (dmpReady == false)
  {
    MPU_Connect();
  }
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16); // mengubah kesensitifitas gyro dari 2G ke 16G
}

void loop()
{
  fsr1value = analogRead(fsrPin1);
  fsr2value = analogRead(fsrPin2);

  if (!dmpReady)
    return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    yaw = (ypr[0] * 180 / M_PI);
    pitch = (ypr[1] * 180 / M_PI);
    roll = (ypr[2] * 180 / M_PI);

    print_yaw_pitch_roll();
    Input = pitch;   // Set the input to the current pitch
    myPID.Compute(); // Perform the PID computation

    //        int i = 0;
    //        while(true){
    //          i = i + 5;
    //           rightFootServo.write(i);
    //           delay(500);
    //        }
    //        Serial.println(abs(Output));

    servoAngle = titikawalmotor + (Output + titikawalmpu6050);
    //        rightFootServo.write(abs(Output));
    //        Serial.print("\tMotor Output: ");
    // Serial.println(abs(Output));
    //         Controlling the servo based on the PID output = PWM
    if (servoAngle > -24 && servoAngle < 38)
    {
      rightFootServo.write(servoAngle);
      // Serial.print("Servo Angle: ");
      // Serial.println(servoAngle);
    }
    else if (servoAngle < -24)
    {
      rightFootServo.write(-24);
    }
    else
    {
      rightFootServo.write(39);
    }
  }

  // teknisee loop
  //-----------------------------------------------------------------------------------------------
  data2json();
  writeFirebase();
  if (millis() - lastmillis > 10000)
  {
    lastmillis = millis();
    // call function
    readFirebase();
    if (bacaSession > 0)
    {
      send2psql();
      bacaSession--;
    }
  }
  // teknisee loop end
  //-----------------------------------------------------------------------------------------------
  delay(1000);
}
