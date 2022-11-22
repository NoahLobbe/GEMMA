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
};


////Misc.
#define speed_of_sound 340 //metres per second
#define len 100 
uint32_t values[len]; 
uint8_t counter = 0;

#define obstacle_distance_minimum 100 //mm
uint16_t obstacle_distance_mm; //mm

////timing for main loop
unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 5;        //ms time constant for timer

////Motion
String current_leg_flag = "LL"; //LL->Lower Left, ..... Cycles in order: LL -> UR -> LR -> UL -> LL...
String current_leg_stage_flag = "UP"; //UP, EM, DOWN, DONE

////Servos variables and objects
//NOTE: position measurements are not neccessarily degrees, eg. 90 is half way for ServoObj.write(...) method even if a servo can only go to 80deg
#define SERVO_STARTING_POS 90 

//Lower Left Limb
#define SERVO_LL_MIN 30
#define SERVO_LL_MAX 140
#define SERVO_LL_HALF_POS (SERVO_LL_MAX - SERVO_LL_MIN)/2
#define LOWER_LEFT_SERVO_PIN 2
#define LOWER_LEFT_EM_PIN 6 //L298N En_A -> Out_1
Limb LowerLeft;
int16_t servo_angle_input = SERVO_STARTING_POS;

//Lower Right Limb
#define SERVO_LR_MIN 155
#define SERVO_LR_MAX 45
#define SERVO_LR_HALF_POS (SERVO_LR_MAX - SERVO_LR_MIN)/2
#define LOWER_RIGHT_SERVO_PIN 3
#define LOWER_RIGHT_EM_PIN 7 //L298N En_B -> Out_2
Limb LowerRight;

//Upper Right Limb
#define SERVO_UR_MIN 125
#define SERVO_UR_MAX 20
#define SERVO_UR_HALF_POS (SERVO_UR_MAX - SERVO_UR_MIN)/2
#define UPPER_RIGHT_SERVO_PIN 4
#define UPPER_RIGHT_EM_PIN 8 //L298N En_C -> Out_3
Limb UpperRight;

//Upper Left Limb
#define SERVO_UL_MIN 40
#define SERVO_UL_MAX 130
#define SERVO_UL_HALF_POS (SERVO_UL_MAX - SERVO_UL_MIN)/2
#define UPPER_LEFT_SERVO_PIN 5
#define UPPER_LEFT_EM_PIN 9 //L298N En_D -> Out_4
Limb UpperLeft;

/*
////Ultrasonic sensor
#define TRIGGER 11
#define ECHO  12

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
  Serial.print(" LowerLeft prev_smoothed_angle ");
  Serial.print(LowerLeft.prev_smoothed_angle);
  
  Serial.print("\nLowerRight smoothed_angle ");
  Serial.print(LowerRight.smoothed_angle);
  Serial.print(" LowerRight prev_smoothed_angle ");
  Serial.print(LowerRight.prev_smoothed_angle);

  Serial.print("\nUpperRight smoothed_angle ");
  Serial.print(UpperRight.smoothed_angle);
  Serial.print(" UpperRight prev_smoothed_angle ");
  Serial.print(UpperRight.prev_smoothed_angle);

  Serial.print("\nUpperLeft smoothed_angle ");
  Serial.print(UpperLeft.smoothed_angle);
  Serial.print(" UpperLeft prev_smoothed_angle ");
  Serial.print(UpperLeft.prev_smoothed_angle);
  
  delay(2000);
  
}

void updateServoVal(Limb& limbObj, int angle_input) {
  limbObj.smoothed_angle = (angle_input * 0.04) + (limbObj.prev_smoothed_angle * 0.96);
  limbObj.prev_smoothed_angle = limbObj.smoothed_angle;

  Serial.print("\n smoothed angle INSIDE ");
 Serial.print(limbObj.smoothed_angle);
  Serial.print(" prev_smoothed_angle INSIDE ");
  Serial.print(limbObj.prev_smoothed_angle);
}

void updateLimb(Limb& limbObj, int servo_max, int servo_min, String next_leg_code, char side) {
  //'side' is required as the Right servos need to go in opposite direction to Left servos
  
  if (current_leg_stage_flag == "UP") {
    if (side == 'L') {
      if (limbObj.Joint.read() >= servo_max-1) { //servo.read() loses the decimal (from limbObj.smoothed_angle) e.g 129.9999 becomes 129 so if servo_max=130 then it won't ever succeed unless servo_max -=1
        current_leg_stage_flag = "EM";
      } else {
        updateServoVal(limbObj, servo_max);
      }
    } else if (side == 'R') {
      if (limbObj.Joint.read() <= servo_max) { 
        current_leg_stage_flag = "EM";
      } else {
        updateServoVal(limbObj, servo_max);
      }
    }
    
  } else if (current_leg_stage_flag == "EM") { 
    //turn on EM
    current_leg_stage_flag = "DOWN";
      
  } else if (current_leg_stage_flag == "DOWN") {
    if (side == 'L') {
      if (limbObj.Joint.read() <= servo_min) {
        current_leg_stage_flag = "DONE";
      } else {
        updateServoVal(limbObj, servo_min);
      }
    } else if (side == 'R') {
      Serial.print(limbObj.Joint.read());
      if (limbObj.Joint.read() >= servo_min-1) {//servo.read() loses the decimal (from limbObj.smoothed_angle) e.g 129.9999 becomes 129 so if servo_max=130 then it won't ever succeed unless servo_max -=1
        current_leg_stage_flag = "DONE";
      } else {
        updateServoVal(limbObj, servo_min);
      }
    }
    
  } else if (current_leg_stage_flag == "DONE") {
        current_leg_stage_flag = "UP";
        current_leg_flag = next_leg_code;
  }
  
}


void loop(){
  
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {  //main loop 
    //
    
    //inputs
    if (Serial.available() >0) {
      String input = Serial.readString();
      //servo_angle_input = 0.91*servo_angle_input + 4; //https://www.desmos.com/calculator/fachybyxjl
      code = input.substring(0,2);
      angle = input.substring(2).toInt();
      Serial.println("\nCode entered: " + code + " ");
      Serial.println("Angle entered: " + String(angle) + " ");

      
      
    }

    ////calculations
    //Serial.print("\ncurrent_leg_flag: " + String(current_leg_flag));

    
    //Serial.print("\n");
    
    
    //motion
    if (current_leg_flag == "LL") {
      Serial.print(" current_leg_flag: " + current_leg_flag);
    Serial.print(" current_leg_stage_flag: " + current_leg_stage_flag);
      updateLimb(LowerLeft, SERVO_LL_MAX, SERVO_LL_MIN, "UR", 'L');     
      
    } else if (current_leg_flag == "UR") {
      Serial.print(" current_leg_flag: " + current_leg_flag);
    Serial.print(" current_leg_stage_flag: " + current_leg_stage_flag);
      updateLimb(UpperRight, SERVO_UR_MAX, SERVO_UR_MIN, "LR", 'R');
      
    } else if (current_leg_flag == "LR") {
      Serial.print(" current_leg_flag: " + current_leg_flag);
    Serial.print(" current_leg_stage_flag: " + current_leg_stage_flag);
      updateLimb(LowerRight, SERVO_LR_MAX, SERVO_LR_MIN, "UL", 'R');
      
    } else if (current_leg_flag == "UL") {
      Serial.print(" current_leg_flag: " + current_leg_flag);
    Serial.print(" current_leg_stage_flag: " + current_leg_stage_flag);
      updateLimb(UpperLeft, SERVO_UL_MAX, SERVO_UL_MIN, "LL", 'L');
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
  //delay(10);
}
/*
void stepLegUp(String leg) {
  Serial.println("Stepping up");
  if (leg == "LL") { ////Lower left
    LowerLeft.EM.write(0); //turn off
    LowerLeft.Joint.write(SERVO_MAX_POS); //move up
    delay(1000);
    LowerLeft.EM.write(1); //turn on
    LowerLeft.Joint.write(SERVO_HALF_POS);
    
  } else if (leg == "LR") { ////lower right
    LowerRight.EM.write(0); //turn off
    LowerRight.Joint.write(SERVO_MAX_POS); //move up
    delay(1000);
    LowerRight.EM.write(1); //turn on
    LowerRight.Joint.write(SERVO_HALF_POS);
    
  } else if (leg == "UR") { ////upper right
    UpperRight.EM.write(0); //turn off
    UpperRight.Joint.write(SERVO_MAX_POS); //move up
    delay(1000);
    UpperRight.EM.write(1); //turn on
    UpperRight.Joint.write(SERVO_HALF_POS);
    
  } else if (leg == "UL") { ////upper left
    UpperLeft.EM.write(0); //turn off
    UpperLeft.Joint.write(SERVO_MAX_POS);
    delay(1000);
    UpperLeft.EM.write(1); //turn on
    UpperLeft.Joint.write(SERVO_HALF_POS);
    
  }
}

void stepLegDown(String leg) {
  Serial.println("Stepping down");
  if (leg == "LL") { ////Lower left
    LowerLeft.EM.write(0); //turn off
    LowerLeft.Joint.write(SERVO_MIN_POS); //move down
    delay(1000);
    LowerLeft.EM.write(1); //turn on
    LowerLeft.Joint.write(SERVO_HALF_POS);
    
  } else if (leg == "LR") { ////lower right
    LowerRight.EM.write(0); //turn off
    LowerRight.Joint.write(SERVO_MIN_POS); //move down
    delay(1000);
    LowerRight.EM.write(1); //turn on
    LowerRight.Joint.write(SERVO_HALF_POS);
    
  } else if (leg == "UR") { ////upper right
    UpperRight.EM.write(0); //turn off
    UpperRight.Joint.write(SERVO_MIN_POS); //move down
    delay(1000);
    UpperRight.EM.write(1); //turn on
    UpperRight.Joint.write(SERVO_HALF_POS);
    
  } else if (leg == "UL") { ////upper left
    UpperLeft.EM.write(0); //turn off
    UpperLeft.Joint.write(SERVO_MIN_POS); //move down
    delay(1000);
    UpperLeft.EM.write(1); //turn on
    UpperLeft.Joint.write(SERVO_HALF_POS);
    
  }
}

void loop() {
  ////inputs
  //ultrasonic
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10); //send a 10uS trigger pulse
  digitalWrite(TRIGGER, LOW);

  uint16_t duration_uS = pulseIn(ECHO, HIGH, 0);


  ////Calculations
  if (counter >= len) {
    counter = 0;
    uint32_t average = 0; //reset averager
    for (int i=0; i<len; i++) {
      average += values[i];
    }
    average = average/len;
    //Serial.print("\t average uS: ");
    //Serial.print(average, 10);
  }
 
  values[counter] = duration_uS; //add new value

  uint16_t obstacle_distance_mm = 0.5 * speed_of_sound * duration_uS * 1e-3;

  if (obstacle_distance_mm <= obstacle_distance_minimum) {
    current_direction = 'R';
    current_leg = "LL";
  } else {
    current_direction = 'F';
  }

  Serial.print("\nDirection: "); 
  Serial.print(current_direction); 
  Serial.print(" current leg: "); 
  Serial.print(current_leg); 
  

  ////user control
  if (Serial.available() >0) {
    String answer = Serial.readString();
    Serial.println("\nCode entered: " + String(answer) );
    String code_string = answer.substring(0,2);
    int code_number = answer.substring(2).toInt();

    if (code_string == "AA"){
      LowerLeft.EM.write(code_number);
      LowerRight.EM.write(code_number);
      UpperRight.EM.write(code_number);
      UpperLeft.EM.write(code_number);
      
    } else if (code_string == "DD"){
      Serial.print("\nuS: "); 
      Serial.print(duration_uS); 
      Serial.print(" distance to obstacle (mm): ");
      Serial.print(obstacle_distance_mm);
      
    } else if (code_string == "LL"){
      stepLegUp(code_string);
      stepLegDown(code_string);
      
    } else if (code_string == "LR"){
      stepLegUp(code_string);
      stepLegDown(code_string);
      
    } else if (code_string == "UR"){
      stepLegUp(code_string);
      stepLegDown(code_string);
      
    } else if (code_string == "UL"){
      stepLegUp(code_string);
      stepLegDown(code_string);
    }
  }

  ////execute  
  if (current_direction == 'F') { //go forwards
    if (current_leg == "LL") {
      stepLegUp(current_leg);
      stepLegDown(current_leg);
      current_leg = "UR";
    
    } else if (current_leg == "LR") {
      stepLegUp(current_leg);
      stepLegDown(current_leg);
      current_leg = "UL";
    
    } else if (current_leg == "UR") {
      stepLegUp(current_leg);
      stepLegDown(current_leg);
      current_leg = "LR";
    
    } else if (current_leg == "UL") {
      stepLegUp(current_leg);
      stepLegDown(current_leg);
      current_leg = "LL";
    
    }
  } else if (current_direction == 'R') { //turn right
    if (current_leg == "LL") {
      stepLegUp(current_leg);
      stepLegDown(current_leg);
      current_leg = "UR";
    
    } else if (current_leg == "LR") {
      current_leg = "UL";
    
    } else if (current_leg == "UR") {
      stepLegDown(current_leg);
      current_leg = "UL";
    
    } else if (current_leg == "UL") {
      stepLegUp(current_leg);
      stepLegDown(current_leg);
      current_leg = "LR";
    
    }
    
  } 

  

  delay(60); //to prevent the ultrasonic sensor self interference
  counter++;
 
}*/
