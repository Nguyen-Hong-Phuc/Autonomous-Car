#include <SPI.h>
#include <Servo.h>

// Define pin numbers
const int motorPin = 6;  // Motor control pin D6
const int servoPin = 5;  // Servo control pin D5
int speedTurn = 200;
int speedForward = 230; //0-255

Servo myServo;           // Create servo object

volatile byte receivedData = 0;  // Variable to store received data
volatile boolean newData = false;    // Flag to check if new data received

void setup() {
  Serial.begin(115200);
  myServo.attach(servoPin);   // Attach the servo on servo pin D5
  pinMode(motorPin,OUTPUT);
  SPI.attachInterrupt();      // Enable SPI interrupt
  SPCR |= _BV(SPE);               // Begin SPI communication

  // Set SCK, MOSI, and SS as inputs
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(SS, INPUT);
  // Set MISO as output
  pinMode(MISO, OUTPUT);
}

// SPI interrupt routine
ISR (SPI_STC_vect) {
  receivedData = SPDR;  // Get the received data
  newData = true;       // Set the flag
}

void loop() {
  if (newData == true) {
    Serial.println(receivedData);
    int servoAngle = receivedData;
    myServo.write(servoAngle);

    if (servoAngle > 90 + 10 || servoAngle < 90 - 10) {  // More than +/- 10 degrees from center
      analogWrite(motorPin, speedForward);  // Slower speed for sharper turns
    } else {
      analogWrite(motorPin, speedTurn);  // Normal speed for slight or no turn
    }
    
    newData = false;  // Clear the flag
  }
}
