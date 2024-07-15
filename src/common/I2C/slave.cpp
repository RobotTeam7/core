#include <Arduino.h>
#include <Wire.h>

#define LED_PIN PA6

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  while (Wire.available()) { // Loop through all available bytes
    int signal = Wire.read(); // receive a byte as integer
    if (signal == 21) {
      digitalWrite(LED_PIN, HIGH);
    } else if (signal == 83) {
      digitalWrite(LED_PIN, LOW);
    }
  }
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event

}

void loop()
{
}



