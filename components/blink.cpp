// library utama
#include <Arduino.h>

void init_blink()
{
    pinMode(LED_BUILTIN, OUTPUT);
}

void blink()
{
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    Serial.println("hidup");
    delay(500);                     // wait for a second
    digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
    Serial.println("mati");
    delay(500); // wait for a second
}

void setup()
{
    // initialize digital pin LED_BUILTIN as an output.
    Serial.begin(115200);
    init_blink();
}

// the loop function runs over and over again forever
void loop()
{
    blink();
}
