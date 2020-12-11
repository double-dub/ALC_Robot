#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder motor1(2, 5);
Encoder motor2(3, 6);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
}

long positionLeft  = -999;
long positionRight = -999;

void loop() {
  int newLeft, newRight;
  newLeft = motor1.read();
  newRight = motor2.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.println(newLeft);
    Serial.println(newRight);
    positionLeft = newLeft;
    positionRight = newRight;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    motor1.write(0);
    motor2.write(0);
    newLeft = motor1.read();
    newRight = motor2.read();
    Serial.println(0);
    Serial.println(0);
  }
}
