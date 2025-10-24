#include <QuadEncoder.h>
#include <Adafruit_NAU7802.h>
#include <Arduino.h>
#include <FlexCAN_T4.h>

#define NUM_SERVO_COMMANDS 120 * 1000

/*////////
 * Instantiate objects
 *////////
QuadEncoder       encoder1(3, 5, 7);  // ENC1 using pins 0 (A) and 1 (B)
Adafruit_NAU7802  nau;
FlexCAN_T4        <CAN3, RX_SIZE_256, TX_SIZE_16> can3;

IntervalTimer     encoderTimer;
IntervalTimer     loadcellTimer;
IntervalTimer     servoTimer;

/*////////
 * Global Variables
 *////////
 uint16_t excitation_signal[NUM_SERVO_COMMANDS];
 unsigned int servo_commnd_index = 0;
 unsigned long start_micros;


/*////////
 * Function Prototypes
 *////////
void readEncoder();
void readLoadCell();
void commandServo();


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
  can3.begin();
  can3.setBaudRate(1000000); // 1 Mbps

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


  // Start the microsecond timer
  start_micros = micros();

  /*////////
   * Start the control and DAQ timers
   *////////
  encoderTimer.begin(readEncoder, 3125);  // 320 Hz timer
  loadcellTimer.begin(readLoadCell, 3125);  // Read load cell at 320 Hz
  servoTimer.begin(commandServo, 1000); // Send position commands at 1kHz

}


/*////////
 * Main Loop
 *////////
void loop() {
  // Restart the teensy when the sine sweep is over
  if(servo_commnd_index < NUM_SERVO_COMMANDS){
    delay(5000);
    SCB_AIRCR = 0x05FA0004; // Software reset of the teensy
  }
}

/*////////
 * Function declarations
 *////////

void readEncoder(){
  if (servo_commnd_index < NUM_SERVO_COMMANDS){
    Serial.print("Encoder: ");
    Serial.print(encoder1.read());
    Serial.print(", ");
    Serial.println(micros() - start_micros);
  }
}

void readLoadCell(){
  if (servo_commnd_index < NUM_SERVO_COMMANDS){
    Serial.print("LC: ");
    Serial.print(nau.read());
    Serial.print(", ");
    Serial.println(micros()-start_micros);
  }
}

void commandServo(){
  if (servo_commnd_index < NUM_SERVO_COMMANDS){

    CAN_message_t tx;
    tx.id = 0x3;
    tx.len = 2;

    // Fromat the message
    tx.buf[0] = excitation_signal[servo_commnd_index] & 0xFF;        // Low byte
    tx.buf[1] = (excitation_signal[servo_commnd_index] >> 8) & 0xFF; // High byte

    // Transmit the CAN command
    can3.write(tx);

    // Increment the command index
    servo_commnd_index++
  }

}