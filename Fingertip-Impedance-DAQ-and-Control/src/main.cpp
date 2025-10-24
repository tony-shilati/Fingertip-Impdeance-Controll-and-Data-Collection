#include <QuadEncoder.h>
#include <Adafruit_NAU7802.h>
#include <Arduino.h>

/*////////
 * Instantiate objects
 *////////
QuadEncoder       encoder1(3, 5, 7);  // ENC1 using pins 0 (A) and 1 (B)
Adafruit_NAU7802  nau;
IntervalTimer     encoderTimer;
IntervalTimer     loadcellTimer;

/*////////
 * Global Variables
 *////////


/*////////
 * Function Prototypes
 *////////
void readEncoder();
void readLoadCell();


/*////////
 * Setup Code
 *////////
void setup() {
  // Open a serial monitor
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Serial connected!");

  /*////////
   * Config the linear servo comms
   *////////

  /*////////
   * Config the load cell amp
   */////////
  Wire.begin();                        // Initialize I2C
  Wire.setClock(400000);              // Use I2C (400 kHz)

  if (!nau.begin(&Wire)) {
    Serial.println("NAU7802 not found!");
    while (1) {}
  }

  // Configure NAU7802
  nau.setLDO(NAU7802_3V3);             // Match Teensy 3.3V supply
  nau.setGain(NAU7802_GAIN_128);       // Max gain for small load cell signals
  nau.setRate(NAU7802_RATE_320SPS);    // Maximum sample rate

  // Take 500 readings to flush out readings
  for (uint16_t i=0; i<500; i++) {
    while (! nau.available()) delay(1);
    nau.read();
  }

  while (! nau.calibrate(NAU7802_CALMOD_INTERNAL)) {
    Serial.println("Failed to calibrate internal offset, retrying!");
    delay(1000);
  }

  while (! nau.calibrate(NAU7802_CALMOD_OFFSET)) {
    Serial.println("Failed to calibrate system offset, retrying!");
    delay(1000);
  }

  Serial.println("Calibrated internal offset");

  /*////////
   * Configure the encoder
   *////////
  encoder1.setInitConfig();   // load default settings
  encoder1.init();            // initialize hardware encoder

  // Put the Quadrature chip in RS-422 mode
  pinMode(26, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  digitalWrite(26, LOW);
  digitalWrite(11, HIGH);
  digitalWrite(12, LOW);


  /*////////
   * Start the control and DAQ timers
   *////////
  encoderTimer.begin(readEncoder, 3125);  // 320 Hz timer
  loadcellTimer.begin(readLoadCell, 3125);  // Read load cell at 320 Hz

}


/*////////
 * Main Loop
 *////////
void loop() {
  // put your main code here, to run repeatedly:
}

/*////////
 * Function declarations
 *////////

void readEncoder(){
  Serial.print("E: ");
  Serial.print(encoder1.read());
  Serial.print(", ");
  Serial.println(micros());
}

void readLoadCell(){
  Serial.print("LC: ");
  Serial.print(nau.read());
  Serial.print(", ");
  Serial.println(micros());
}