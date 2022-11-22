/*
 * GEMMA: Gecko Electro Magnet Motion Atraction
 * A gecko that climbs up metal surfaces by using electromagnets as feet.
 * 
 * ----Conventions----
 * constants: 'ALL_CAPS'
 * variables and class members: 'helpful_variable_name'
 * class names and objects: 'DoomButton' or 'Cheese'
 * 
 * by Noah Lobbe 
 * 
 * NOTES:
 * Thanks to James Bruton for his servo smoothing code: https://github.com/XRobots/ServoSmoothing and https://www.youtube.com/watch?v=jsXolwJskKM
 */

#include <Servo.h>


class Electromagnet {
  public:
    Electromagnet(); //initialize
    void write(bool on_state);
    void attach(int pin);
    int pin;
    bool state;
    
};

Electromagnet::Electromagnet(){
}
void Electromagnet::attach(int _pin) {
  pin = _pin;
  pinMode(pin, OUTPUT);
}
void Electromagnet::write(bool on_state) {
  digitalWrite(pin, on_state);
  state = on_state;
}

struct Limb {
  Servo Joint;
  Electromagnet EM;
  float smoothed_angle;
  float prev_smoothed_angle;
  bool move_flag = true;
  char stage_flag = 'U'; //U=up, D=down, E=electromagnet, W=wait
};

////Misc.
#define obstacle_distance_minimum 100 //mm
uint16_t obstacle_distance_mm; //mm


////timing for main loop
unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        //ms time constant for timer


////Motion
String leg_state = "HALF"; //U=up, D=down
////Servos variables and objects
//NOTE: position measurements are not neccessarily degrees, eg. 90 is half way for ServoObj.write(...) method even if a servo can only go to 80deg
#define SERVO_STARTING_POS 90 

//Lower Left Limb
#define SERVO_LL_MIN 30
#define SERVO_LL_MAX 140
#define SERVO_LL_HALF (SERVO_LL_MAX - SERVO_LL_MIN)/2
#define LOWER_LEFT_SERVO_PIN 2
#define LOWER_LEFT_EM_PIN 6 //L298N En_A -> Out_1
Limb LowerLeft;
int16_t servo_angle_input = SERVO_STARTING_POS;

//Lower Right Limb
#define SERVO_LR_MIN 155
#define SERVO_LR_MAX 45
#define SERVO_LR_HALF abs( (SERVO_LR_MAX - SERVO_LR_MIN)/2 )
#define LOWER_RIGHT_SERVO_PIN 3
#define LOWER_RIGHT_EM_PIN 7 //L298N En_B -> Out_2
Limb LowerRight;

//Upper Right Limb
#define SERVO_UR_MIN 125
#define SERVO_UR_MAX 20
#define SERVO_UR_HALF abs( (SERVO_UR_MAX - SERVO_UR_MIN)/2 )
#define UPPER_RIGHT_SERVO_PIN 4
#define UPPER_RIGHT_EM_PIN 8 //L298N En_C -> Out_3
Limb UpperRight;

//Upper Left Limb
#define SERVO_UL_MIN 40
#define SERVO_UL_MAX 130
#define SERVO_UL_HALF (SERVO_UL_MAX - SERVO_UL_MIN)/2
#define UPPER_LEFT_SERVO_PIN 5
#define UPPER_LEFT_EM_PIN 9 //L298N En_D -> Out_4
Limb UpperLeft;

/*
////Ultrasonic sensor
#define TRIGGER 11
#define ECHO  12
//????
#define speed_of_sound 340 //metres per second
#define len 100 
uint32_t values[len]; 
uint8_t counter = 0;

*/

String code;
int angle;

void setup() {
  Serial.begin(115200);

  /*
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIGGER, LOW);
  */
  
  LowerLeft.Joint.attach(LOWER_LEFT_SERVO_PIN);
  LowerLeft.Joint.write(SERVO_STARTING_POS);
  LowerLeft.EM.attach(LOWER_LEFT_EM_PIN);
  //LowerLeft.EM.write(1);
  LowerLeft.smoothed_angle = SERVO_STARTING_POS;
  LowerLeft.prev_smoothed_angle = LowerLeft.smoothed_angle; //to prevent leg from going down instantly
  
  LowerRight.Joint.attach(LOWER_RIGHT_SERVO_PIN);
  LowerRight.Joint.write(SERVO_STARTING_POS);
  LowerRight.EM.attach(LOWER_RIGHT_EM_PIN);
  //LowerRight.EM.write(1);
  LowerRight.smoothed_angle = SERVO_STARTING_POS;
  LowerRight.prev_smoothed_angle = LowerRight.smoothed_angle; //to prevent leg from going down instantly

  UpperRight.Joint.attach(UPPER_RIGHT_SERVO_PIN);
  UpperRight.Joint.write(SERVO_STARTING_POS);
  UpperRight.EM.attach(UPPER_RIGHT_EM_PIN);
  //UpperRight.EM.write(1);
  UpperRight.smoothed_angle = SERVO_STARTING_POS;
  UpperRight.prev_smoothed_angle = UpperRight.smoothed_angle; //to prevent leg from going down instantly

  UpperLeft.Joint.attach(UPPER_LEFT_SERVO_PIN);
  UpperLeft.Joint.write(SERVO_STARTING_POS);
  UpperLeft.EM.attach(UPPER_LEFT_EM_PIN);
  //UpperLeft.EM.write(1);
  UpperLeft.smoothed_angle = SERVO_STARTING_POS;
  UpperLeft.prev_smoothed_angle = UpperLeft.smoothed_angle; //to prevent leg from going down instantly

  Serial.print("\nLowerLeft smoothed_angle ");
  Serial.print(LowerLeft.smoothed_angle);
  Serial.print(" prev_smoothed_angle ");
  Serial.print(LowerLeft.prev_smoothed_angle);
  Serial.print(" halfway angle ");
  Serial.print(SERVO_LL_HALF);
  
  Serial.print("\nLowerRight smoothed_angle ");
  Serial.print(LowerRight.smoothed_angle);
  Serial.print(" prev_smoothed_angle ");
  Serial.print(LowerRight.prev_smoothed_angle);
  Serial.print(" halfway angle ");
  Serial.print(SERVO_LR_HALF);

  Serial.print("\nUpperRight smoothed_angle ");
  Serial.print(UpperRight.smoothed_angle);
  Serial.print(" prev_smoothed_angle ");
  Serial.print(UpperRight.prev_smoothed_angle);
  Serial.print(" halfway angle ");
  Serial.print(SERVO_UR_HALF);

  Serial.print("\nUpperLeft smoothed_angle ");
  Serial.print(UpperLeft.smoothed_angle);
  Serial.print(" prev_smoothed_angle ");
  Serial.print(UpperLeft.prev_smoothed_angle);
  Serial.print(" halfway angle ");
  Serial.print(SERVO_UL_HALF);
  
  delay(2000);
  
}

void updateServoVal(Limb& limbObj, int angle_input) {
  limbObj.smoothed_angle = (angle_input * 0.04) + (limbObj.prev_smoothed_angle * 0.96);
  limbObj.prev_smoothed_angle = limbObj.smoothed_angle;

  //Serial.print(" smoothed angle INSIDE ");
  //Serial.print(limbObj.smoothed_angle);
  //Serial.print(" prev_smoothed_angle INSIDE ");
  //Serial.print(limbObj.prev_smoothed_angle);
}

bool up(Limb& limbObj, int servo_target_angle, char side) {
  //returns true if finished going up
  if (side == 'L') {

    Serial.print(" L joint: " + String(limbObj.Joint.read()) + " target: " + String(servo_target_angle-1) );
    if (limbObj.Joint.read() >= servo_target_angle-1) { //rounding issue from Joint.smoothed_angle being float; when smoothed_angle=129.99 Joint.read() returns 129 instead of 130. Could check with smoothed_angle but a different might have been writen to servo; .read() prevents that
      return true;
    } else {
      updateServoVal(limbObj, servo_target_angle);
      return false;
    }
  } else if (side == 'R') {
    
    Serial.print(" R joint: " + String(limbObj.Joint.read()) + " aim: " + String(servo_target_angle) );
    if (limbObj.Joint.read() <= servo_target_angle) { 
      return true;
    } else {
     updateServoVal(limbObj, servo_target_angle);
     return false;
    }
  }
}

bool down(Limb& limbObj, int servo_target_angle, char side) {
  //returns true if finished going up
  if (side == 'L') {

    Serial.print(" L joint: " + String(limbObj.Joint.read()) + " aim: " + String(servo_target_angle) );
    if (limbObj.Joint.read() <= servo_target_angle) { //servo.read() loses the decimal (from limbObj.smoothed_angle) e.g 129.9999 becomes 129 so if servo_max=130 then it won't ever succeed unless servo_max -=1
      return true;
    } else {
      updateServoVal(limbObj, servo_target_angle);
      return false;
    }
  } else if (side == 'R') {

    Serial.print(" R joint: " + String(limbObj.Joint.read()) + " aim: " + String(servo_target_angle-1) );
    if (limbObj.Joint.read() >= servo_target_angle-1) { //rounding issue from Joint.smoothed_angle being float; when smoothed_angle=129.99 Joint.read() returns 129 instead of 130. Could check with smoothed_angle but a different might have been writen to servo; .read() prevents that
      return true;
    } else {
     updateServoVal(limbObj, servo_target_angle);
     return false;
    }
  }
}

void updateLimb(Limb& limbObj, int servo_max, int servo_min, char side) {
  //'side' is required as the Right servos need to go in opposite direction to Left servos
 
  if (limbObj.stage_flag == 'U') {//up
    if (side == 'L') {
      if (limbObj.Joint.read() >= servo_max-1) { //servo.read() loses the decimal (from limbObj.smoothed_angle) e.g 129.9999 becomes 129 so if servo_max=130 then it won't ever succeed unless servo_max -=1
        limbObj.stage_flag = 'E';
      } else {
        updateServoVal(limbObj, servo_max);
      }
    } else if (side == 'R') {
      if (limbObj.Joint.read() <= servo_max) { 
        limbObj.stage_flag = 'E';
      } else {
        updateServoVal(limbObj, servo_max);
      }
    }
    
  } else if (limbObj.stage_flag == 'E') { 
    //turn on EM
    limbObj.stage_flag = 'D';
      
  } else if (limbObj.stage_flag == 'D') {//down
    if (side == 'L') {
      if (limbObj.Joint.read() <= servo_min) {
        limbObj.stage_flag = 'W';
      } else {
        updateServoVal(limbObj, servo_min);
      }
    } else if (side == 'R') {
      if (limbObj.Joint.read() >= servo_min-1) { // For right side, min equals something like 130; therefore....
        limbObj.stage_flag = 'W';
      } else {
        updateServoVal(limbObj, servo_min);
      }
    }
    
  } else if (limbObj.stage_flag == 'W') {//wait
        limbObj.stage_flag = 'U';
        Serial.print("\n..................................Waiting..................................");
        //delay(1000);
  }
  
}


void loop(){
  
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {  //main loop 
    //
    
    //inputs
    if (Serial.available() >0) {
      String input = Serial.readString();
      leg_state = input;//(char)Serial.read();//convert to char
      Serial.println("\nEntered: " + String(leg_state) + " ");
      delay(1000);
      //servo_angle_input = 0.91*servo_angle_input + 4; //https://www.desmos.com/calculator/fachybyxjl
      //code = input.substring(0,2);
      //angle = input.substring(2).toInt();
      //Serial.println("\nCode entered: " + code + " ");
      //Serial.println("Angle entered: " + String(angle) + " ");

      
      
    }

    ////calculations
    //Serial.print("\ncurrent_leg_flag: " + String(current_leg_flag));

    
    //Serial.print("\n");
    
    
    //motion
    /*
    if (LowerLeft.move_flag) {
      //Serial.print(" current_leg_flag: " + current_leg_flag);
      //Serial.print(" current_leg_stage_flag: " + current_leg_stage_flag);
      updateLimb(LowerLeft, "LL", SERVO_LL_MAX, SERVO_LL_MIN, 'L');      
    } 
    
    if (UpperRight.move_flag) {
      //Serial.print(" current_leg_flag: " + current_leg_flag);
    //Serial.print(" current_leg_stage_flag: " + current_leg_stage_flag);
      updateLimb(UpperRight, "UR", SERVO_UR_MAX, SERVO_UR_MIN, 'R');
    } 
    
    if (LowerRight.move_flag) {
      //Serial.print(" current_leg_flag: " + current_leg_flag);
    //Serial.print(" current_leg_stage_flag: " + current_leg_stage_flag);
      updateLimb(LowerRight, "LR", SERVO_LR_MAX, SERVO_LR_MIN, 'R');
    } 

    if (UpperLeft.move_flag) {
      //Serial.print(" current_leg_flag: " + current_leg_flag);
    //Serial.print(" current_leg_stage_flag: " + current_leg_stage_flag);
      updateLimb(UpperLeft, "UL", SERVO_UL_MAX, SERVO_UL_MIN, 'L');
    } */
    

    

    
    
    if (leg_state == "U") { //all up
      bool LL_up = up(LowerLeft, SERVO_LL_MAX,'L');
      bool LR_up = up(LowerRight, SERVO_LR_MAX,'R');
      bool UR_up = up(UpperRight, SERVO_UR_MAX,'R');
      bool UL_up = up(UpperLeft, SERVO_UL_MAX,'L');

      Serial.print("\n LL  Angle: " + String(round(LowerLeft.smoothed_angle)) + " Up: " + String(LL_up) );
      Serial.print(" | LR  Angle: " + String(round(LowerRight.smoothed_angle)) + " Up: " + String(LR_up) );
      Serial.print(" | UR  Angle: " + String(round(UpperRight.smoothed_angle)) + " Up: " + String(UR_up) );
      Serial.print(" | UL  Angle: " + String(round(UpperLeft.smoothed_angle)) + " Up: " + String(UL_up) );
      //Serial.print(" | All up: " + String(LL_up && LR_up && UR_up && UL_up) );
      
      if (LL_up && LR_up && UR_up && UL_up) {
        Serial.print(" | ...UP...");
        //delay(1000);
        //leg_state = 'U';
      }
      
    } else if (leg_state == "UU") { //front legs up
      bool UR_up = up(UpperRight, SERVO_UR_MAX,'R');
      bool UL_up = up(UpperLeft, SERVO_UL_MAX,'L');

      Serial.print("\n | UR  Angle: " + String(round(UpperRight.smoothed_angle)) + " Up: " + String(UR_up) );
      Serial.print(" | UL  Angle: " + String(round(UpperLeft.smoothed_angle)) + " Up: " + String(UL_up) );
      //Serial.print(" | All up: " + String(LL_up && LR_up && UR_up && UL_up) );
      
      if ( UR_up && UL_up) {
        Serial.print(" | ...UP...");
        //delay(1000);
        //leg_state = 'U';
      }
      
    } else if (leg_state == "LU") { //back legs up
      bool LL_up = up(LowerLeft, SERVO_LL_MAX,'L');
      bool LR_up = up(LowerRight, SERVO_LR_MAX,'R');

      Serial.print("\n LL  Angle: " + String(round(LowerLeft.smoothed_angle)) + " Up: " + String(LL_up) );
      Serial.print(" | LR  Angle: " + String(round(LowerRight.smoothed_angle)) + " Up: " + String(LR_up) );
      //Serial.print(" | All up: " + String(LL_up && LR_up && UR_up && UL_up) );
      
      if (LL_up && LR_up ) {
        Serial.print(" | ...UP...");
        //delay(1000);
        //leg_state = 'U';
      }
      
    } else if (leg_state == "D") { //all down
      bool LL_dn = down(LowerLeft, SERVO_LL_MIN,'L');
      bool LR_dn = down(LowerRight, SERVO_LR_MIN,'R');
      bool UR_dn = down(UpperRight, SERVO_UR_MIN,'R');
      bool UL_dn = down(UpperLeft, SERVO_UL_MIN,'L');

      Serial.print("\n LL  Angle: " + String(round(LowerLeft.smoothed_angle)) + " Dn: " + String(LL_dn) );
      Serial.print(" | LR  Angle: " + String(round(LowerRight.smoothed_angle)) + " Dn: " + String(LR_dn) );
      Serial.print(" | UR  Angle: " + String(round(UpperRight.smoothed_angle)) + " Dn: " + String(UR_dn) );
      Serial.print(" | UL  Angle: " + String(round(UpperLeft.smoothed_angle)) + " Dn: " + String(UL_dn) );
      //Serial.print(" | All down: " + String(LL_dn && LR_dn && UR_dn && UL_dn) );
      
      if (LL_dn && LR_dn && UR_dn && UL_dn) {
        Serial.print(" | ...DOWN...");
        //delay(1000);
        //leg_state = 'U';
      }
    } else if (leg_state == "UD") { //front legs down
      bool UR_dn = down(UpperRight, SERVO_UR_MIN,'R');
      bool UL_dn = down(UpperLeft, SERVO_UL_MIN,'L');

      Serial.print("\n | UR  Angle: " + String(round(UpperRight.smoothed_angle)) + " Dn: " + String(UR_dn) );
      Serial.print(" | UL  Angle: " + String(round(UpperLeft.smoothed_angle)) + " Dn: " + String(UL_dn) );
      //Serial.print(" | All down: " + String(LL_dn && LR_dn && UR_dn && UL_dn) );
      
      if (UR_dn && UL_dn) {
        Serial.print(" | ...DOWN...");
        //delay(1000);
        //leg_state = 'U';
      }
    } else if (leg_state == "LD") { //back legs down
      bool LL_dn = down(LowerLeft, SERVO_LL_MIN,'L');
      bool LR_dn = down(LowerRight, SERVO_LR_MIN,'R');

      Serial.print("\n LL  Angle: " + String(round(LowerLeft.smoothed_angle)) + " Dn: " + String(LL_dn) );
      Serial.print(" | LR  Angle: " + String(round(LowerRight.smoothed_angle)) + " Dn: " + String(LR_dn) );
      //Serial.print(" | All down: " + String(LL_dn && LR_dn && UR_dn && UL_dn) );
      
      if (LL_dn && LR_dn ) {
        Serial.print(" | ...DOWN...");
        //delay(1000);
        //leg_state = 'U';
      }
    } else { //all halfway
      bool LL_hf = down(LowerLeft, 90,'L');
      bool LR_hf = down(LowerRight, 90,'R');
      bool UR_hf = down(UpperRight, 90,'R');
      bool UL_hf = down(UpperLeft, 90,'L');

      Serial.print("\n LL  Angle: " + String(round(LowerLeft.smoothed_angle)) + " Dn: " + String(LL_hf) );
      Serial.print(" | LR  Angle: " + String(round(LowerRight.smoothed_angle)) + " Dn: " + String(LR_hf) );
      Serial.print(" | UR  Angle: " + String(round(UpperRight.smoothed_angle)) + " Dn: " + String(UR_hf) );
      Serial.print(" | UL  Angle: " + String(round(UpperLeft.smoothed_angle)) + " Dn: " + String(UL_hf) );
      //Serial.print(" | All down: " + String(LL_dn && LR_dn && UR_dn && UL_dn) );
      
      if (LL_hf && LR_hf && UR_hf && UL_hf) {
        Serial.print(" | ...HALF...");
        //delay(1000);
        //leg_state = 'U';
      }
    }
    
    /*
    if (code == "LL") {
      updateServoVal(LowerLeft, angle);     
      
    } else if (code == "UR") {
      updateServoVal(UpperRight, angle);
      
    } else if (code == "LR") {
      updateServoVal(LowerRight, angle);
      
    } else if (code == "UL") {
      updateServoVal(UpperLeft, angle);
    }*/

    ////updates and renders

    //updateServoVal(LowerLeft, servo_angle_input);

    //Serial.print(" smoothed angle OUTSIDE ");
    //Serial.print(LowerLeft.smoothed_angle, 20);
    //Serial.print(" converted to Int ");
    //Serial.print(round(LowerLeft.smoothed_angle), DEC);


    //hold servo positions/update current moving one
    LowerLeft.Joint.write(LowerLeft.smoothed_angle);
    LowerRight.Joint.write(LowerRight.smoothed_angle);
    UpperRight.Joint.write(UpperRight.smoothed_angle);
    UpperLeft.Joint.write(UpperLeft.smoothed_angle);
    
    
    

    

    previousMillis = currentMillis;

    
  } // end of main loop
  
}
