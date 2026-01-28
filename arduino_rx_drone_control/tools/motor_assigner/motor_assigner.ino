#include <Servo.h>

Servo esc1;  // D3 - Back left (spins counter-clockwise)
Servo esc2;  // D5 - Front left (spins clockwise)
Servo esc3;  // D9 - Back right (spins clockwise)
Servo esc4;  // D10 - Front right (spins counter-clockwise)

static float us = 1100;

void setup() {
  esc1.attach(3);
  esc2.attach(5);
  esc3.attach(9);
  esc4.attach(10);

  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);

  delay(5000);
}

void loop() {
  esc1.writeMicroseconds(us);
  esc2.writeMicroseconds(us);
  esc3.writeMicroseconds(us);
  esc4.writeMicroseconds(us);
}
