// library utama
#include <Arduino.h>
#include <WiFi.h>

#include <FirebaseESP32.h> // import firebase library

// static global varilale utama
const char *ssid = "teknisee";
const char *password = "myteknisee";
#define FIREBASE_HOST "https://aafo-9b7b2-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "lWOv4jbAHmdJHW3Hi6bvqenkFnOVos0psAeNUCR2"

// global varilale utama
int lastmillis = 0;
float sudut;
int tekananDepan, tekananBelakang;

// global varialbe firebase
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// fungsi dari komponen
//-----------------------------------------------------------------------------------------------
void init_wifi()
{
  WiFi.begin(ssid, password);

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void wifi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
  }
}
void initFirebase()
{
  config.database_url = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.begin(&config, &auth);
}

// fungsi pendukung
//-----------------------------------------------------------------------------------------------
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
    writeToFirebase("/dataNew/beratBelakang", String(tekananBelakang), "float");
    writeToFirebase("/dataNew/beratDepan", String(tekananDepan), "float");
    writeToFirebase("/dataNew/sudut", String(sudut), "float");
  }
  else
  {
    Serial.println("Firebase not ready");
  }
}

void monitorSerial()
{
  Serial.print("tekananDepan: ");
  Serial.print(tekananDepan);
  Serial.print(" tekananBelakang: ");
  Serial.print(tekananBelakang);
  Serial.print(" sudut: ");
  Serial.println(sudut);
  Serial.println();
}

void randomizeMockData()
{
  sudut = random(-450, 900) / 10.0;
  tekananDepan = random(100, 4200);
  tekananBelakang = random(100, 4200);
}

// fungsi utama
//-----------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  init_wifi();
  initFirebase();
}
void loop()
{

  wifi();
  if ((millis() - lastmillis) > 500)
  {
    randomizeMockData();
    writeFirebase();
    //---------------------
    monitorSerial();
    lastmillis = millis();
  }
}