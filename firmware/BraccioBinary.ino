/*
  Braccio Binary Protocol Firmware

  Protocol: [0xFF, Base, Shoulder, Elbow, WristV, WristR, Gripper]
  Baud Rate: 115200 (Faster!)
*/

#include <Braccio.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

void setup() {
  Braccio.begin();
  Serial.begin(115200); // Higher baud rate for performance
  Serial.println("Braccio Ready");
}

void loop() {
  if (Serial.available() >= 7) {
    if (Serial.read() == 0xFF) { // Sync Byte
      int b = Serial.read();
      int s = Serial.read();
      int e = Serial.read();
      int wv = Serial.read();
      int wr = Serial.read();
      int g = Serial.read();

      // Constraint Check
      b = constrain(b, 0, 180);
      s = constrain(s, 15, 165);
      e = constrain(e, 0, 180);
      wv = constrain(wv, 0, 180);
      wr = constrain(wr, 0, 180);
      g = constrain(g, 10, 73);

      Braccio.ServoMovement(20, b, s, e, wv, wr, g);
    }
  }
}
