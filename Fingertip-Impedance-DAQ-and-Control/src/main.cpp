#include <QuadEncoder.h>
#include <Arduino.h>

QuadEncoder encoder1(3, 5, 7);  // ENC1 using pins 0 (A) and 1 (B)

void setup() {
  Serial.begin(115200);

  /*
   * Configure the pins for the quad encoder
   */
  encoder1.setInitConfig();   // load default settings
  encoder1.init();            // initialize hardware encoder

  pinMode(26, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  digitalWrite(26, LOW);
  digitalWrite(11, HIGH);
  digitalWrite(12, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:
}