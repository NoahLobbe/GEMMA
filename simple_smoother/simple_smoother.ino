#include <Servo.h>
struct Limb {
  Servo Joint;
  float switch1Smoothed;
  float switch1Prev;
};

int switch1;
Limb Arm;

void setup() {

  Serial.begin(115200);

  pinMode(12, INPUT_PULLUP);
  Arm.Joint.attach(2);

}

void _update(Limb& armObj, float angle) {
  armObj.switch1Smoothed = (angle * 0.04) + (armObj.switch1Prev * 0.96);

  armObj.switch1Prev = armObj.switch1Smoothed;

  armObj.Joint.write(armObj.switch1Smoothed);
}

void loop() {

  if (Serial.available() >0) {
      switch1 = Serial.parseInt();
    }
  //switch1 = digitalRead(12);      // read switch
  //switch1 = switch1 * 100;        // multiply by 100
  _update(Arm, switch1);
  // *** smoothing ***

  //switch1Smoothed = (switch1 * 0.05) + (switch1Prev * 0.95);

  //switch1Prev = switch1Smoothed;

  // *** end of smoothing ***

  Serial.print(switch1);                  // print to serial terminal/plotter
  Serial.print(" , ");   
  Serial.println(Arm.switch1Smoothed);

  //Joint.write(switch1Smoothed);

  delay(10);                      // run loop 100 times/second

}
