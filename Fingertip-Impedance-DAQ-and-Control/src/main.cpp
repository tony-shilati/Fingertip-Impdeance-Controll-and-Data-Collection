#include <QuadEncoder.h>
#include <Arduino.h>

/*
 * Instantiate objects
 */ 
QuadEncoder encoder1(3, 5, 7);  // ENC1 using pins 0 (A) and 1 (B)
IntervalTimer encoderTimer;

/*
 * Global Variables
 */


/*
 * Function Prototypes
 */
void readEncoder();


/*
 * Setup Code
 */
void setup() {
  Serial.begin(115200);

  /*
   * Configure the encoder pins
   */
  encoder1.setInitConfig();   // load default settings
  encoder1.init();            // initialize hardware encoder

  // Put the Quadrature chip in RS-422 mode
  pinMode(26, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  digitalWrite(26, LOW);
  digitalWrite(11, HIGH);
  digitalWrite(12, LOW);


  /*
   * Start the control and DAQ timers
   */
  encoderTimer.begin(readEncoder, 3125); // 320 Hz timer

}


/*
 * Main Loop
 */
void loop() {
  // put your main code here, to run repeatedly:
}

/*
 * Function declarations
 */

void readEncoder(){
  Serial.print("Encoder: ");
  Serial.print(encoder1.read());
  Serial.print(", ");
  Serial.println(micros());
}