#include <Arduino.h>
#include <Wire.h>

void setup() {
  // Initialize the second I2C bus for communication with the slave
  pinMode(PB6, OUTPUT);
  pinMode(PB7, OUTPUT);
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();

}

void loop() {
  // Communicate with the slave device using the second I2C bus
  Wire.beginTransmission(4); // transmit to device #4
  Wire.write(1);        // sends five bytes
  Wire.endTransmission();
  delay(1000);

  Wire.beginTransmission(4); // transmit to device #4
  Wire.write(0);        // sends five bytes
  Wire.endTransmission();
  delay(1000);
}
