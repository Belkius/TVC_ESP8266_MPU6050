#include <Servo.h>

Servo servo1;
Servo servo2;

void setup() {

servo1.attach(2); //D4
servo2.attach(0); //D3

servo1.write(48);
servo2.write(30);

delay(2000);

}

void loop() {


//servo.write(18);

delay(1000);

//servo.write(42);

delay(1000);

}
