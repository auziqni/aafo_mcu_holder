// library utama
#include <Arduino.h>
#include <WiFi.h>

// static global varilale utama
const char *ssid = "teknisee";
const char *password = "myteknisee";

// global varilale utama
int lastmillis = 0;


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

// fungsi pendukung
//-----------------------------------------------------------------------------------------------
void monitorSerial()
{
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

// fungsi utama
//-----------------------------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    init_wifi();
}
void loop()
{

    wifi();
    if ((millis() - lastmillis) > 1000)
    {

        //---------------------
        monitorSerial();
        lastmillis = millis();
    }
}