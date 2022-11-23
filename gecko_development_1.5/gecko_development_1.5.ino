/*
 * GEMMA: Gecko Electro Mechanical Motion Atraction
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


struct Limb {
  Servo Joint;
  float smoothed_angle;
  float prev_smoothed_angle;
  bool move_flag = true;
  char stage_flag = 'U'; //U=up, D=down, E=electromagnet, W=wait
};



////timing for main loop
unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 5;        //ms time constant for timer


////Motion
 /* 3 letter motion commands
  * AGX = All Go X
  * FGX = Front Go X
  * BGX = Back Go X
  * LGX = Left Go X
  * RGX = Right Go X
  * 
  * DLX = Diagonal Left (\) Go X 
  * DRX = Diagonal Right (/) Go X
  * 
  * XXU = Up
  * XXH = Halfway
  * XXD = Down
  * 
  * LLX = LowerLeft go X
  * LRX = LowerRight go X
  * URX = UpperRight go X
  * ULX = UpperLeft go X
  */

const char straight_motion_array[] = { //best straight line 
  'A', 'G', 'D', 
  'L', 'L', 'U',
  'U', 'R', 'U',
  'U', 'L', 'U',
  'A', 'G', 'D',
  'L', 'R', 'U',
  'U', 'R', 'U',
  'U', 'L', 'U'
};

const char left_turn_motion_array[] = { //not currently used
  'A', 'G', 'D',
  'L', 'R', 'U',
  'U', 'R', 'U',
  'L', 'L', 'U'
};
const char right_turn_motion_array[] = { 
  'A', 'G', 'D',
  'L', 'L', 'U',
  'U', 'L', 'U',
  'L', 'R', 'U'
};

#define STRAIGHT_MOTION_LENGTH sizeof(straight_motion_array)/sizeof(straight_motion_array[0])
#define LEFT_TURN_MOTION_LENGTH sizeof(left_turn_motion_array)/sizeof(left_turn_motion_array[0])
#define RIGHT_TURN_MOTION_LENGTH sizeof(right_turn_motion_array)/sizeof(right_turn_motion_array[0])
int start_slice_index = 0;
int previous_start_slice_index = start_slice_index-1; //mustn't start the same as start_slice_index

String direction_flag = "FORWARD";
char limb_group; //eg AG = All legs
char limb_side; //eg LG = Left legs
char leg_direction; //eg AGU = 
String double_char_code; //becomes (String)limb_group + (String)limb_side; //cast chars to strings to prevent integer addition


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


////Ultrasonic sensor
#define TRIGGER 11
#define ECHO  12
#define speed_of_sound 340 //metres per second
uint8_t dist_sensor_time_counter = 0;

#define obstacle_distance_minimum 100 //mm
uint16_t obstacle_distance_mm;



void setup() {

  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIGGER, LOW);
  
  LowerLeft.Joint.attach(LOWER_LEFT_SERVO_PIN);
  LowerLeft.Joint.write(SERVO_STARTING_POS);//SERVO_LL_MIN);//
  LowerLeft.smoothed_angle = SERVO_STARTING_POS;
  LowerLeft.prev_smoothed_angle = LowerLeft.smoothed_angle; //to prevent leg from going down instantly
  
  LowerRight.Joint.attach(LOWER_RIGHT_SERVO_PIN);
  LowerRight.Joint.write(SERVO_STARTING_POS);
  LowerRight.smoothed_angle = SERVO_STARTING_POS;
  LowerRight.prev_smoothed_angle = LowerRight.smoothed_angle; //to prevent leg from going down instantly

  UpperRight.Joint.attach(UPPER_RIGHT_SERVO_PIN);
  UpperRight.Joint.write(SERVO_STARTING_POS);
  UpperRight.smoothed_angle = SERVO_STARTING_POS;
  UpperRight.prev_smoothed_angle = UpperRight.smoothed_angle; //to prevent leg from going down instantly

  UpperLeft.Joint.attach(UPPER_LEFT_SERVO_PIN);
  UpperLeft.Joint.write(SERVO_STARTING_POS);
  UpperLeft.smoothed_angle = SERVO_STARTING_POS;
  UpperLeft.prev_smoothed_angle = UpperLeft.smoothed_angle; //to prevent leg from going down instantly

 
  delay(2500);
}

void updateServoVal(Limb& limbObj, int angle_input) {
  limbObj.smoothed_angle = (angle_input * 0.04) + (limbObj.prev_smoothed_angle * 0.96);
  limbObj.prev_smoothed_angle = limbObj.smoothed_angle;

}

bool up(Limb& limbObj, int servo_target_angle, char side) {
  //returns true if finished going up
  if (side == 'L') {
    if (limbObj.Joint.read() >= servo_target_angle-1) { //rounding issue from Joint.smoothed_angle being float; when smoothed_angle=129.99 Joint.read() returns 129 instead of 130. Could check with smoothed_angle but a different might have been writen to servo; .read() prevents that
      return true;
    } else {
      updateServoVal(limbObj, servo_target_angle);
      return false;
    }
    
  } else if (side == 'R') {    
    if (limbObj.Joint.read() <= servo_target_angle) { 
      return true;
    } else {
     updateServoVal(limbObj, servo_target_angle);
     return false;
    }
  }
}

bool down(Limb& limbObj, int servo_target_angle, char side) {
  //returns true if finished going down
  if (side == 'L') {
    if (limbObj.Joint.read() <= servo_target_angle) { //servo.read() loses the decimal (from limbObj.smoothed_angle) e.g 129.9999 becomes 129 so if servo_max=130 then it won't ever succeed unless servo_max -=1
      return true;
    } else {
      updateServoVal(limbObj, servo_target_angle);
      return false;
    }
    
  } else if (side == 'R') {
    if (limbObj.Joint.read() >= servo_target_angle-1) { //rounding issue from Joint.smoothed_angle being float; when smoothed_angle=129.99 Joint.read() returns 129 instead of 130. Could check with smoothed_angle but a different might have been writen to servo; .read() prevents that
      return true;
    } else {
     updateServoVal(limbObj, servo_target_angle);
     return false;
    }
  }
}

bool updateLimb(char direction_char, Limb& limbObj, int up_angle, int down_angle, char side) {
  if (direction_char == 'U') {
    return up(limbObj, up_angle, side);
  } else if (direction_char == 'D') {
    return down(limbObj, down_angle, side);
  } else {//halfway
    return up(limbObj, 90, side);
  }
}

bool isLimb(String limb_code) {
  if ( (limb_code=="LL") || (limb_code=="LR") || (limb_code=="UR") || (limb_code=="UL") ) {
    return true;
  } else {
    return false;
  }
}

Limb& getLimb(String limb_code) {
  if (limb_code == "LL") {
    return LowerLeft;    
  } else if (limb_code == "LR") {
    return LowerRight;    
  } else if (limb_code == "UR") {
    return UpperRight;    
  } else if (limb_code == "UL") {
    return UpperLeft;
  }
}

int getServoMax(String limb_code) {
  if (limb_code == "LL") {
    return SERVO_LL_MAX;   
  } else if (limb_code == "LR") {
    return SERVO_LR_MAX;
  } else if (limb_code == "UR") {
    return SERVO_UR_MAX;
  } else if (limb_code == "UL") {
    return SERVO_UL_MAX;
  } else {
    return SERVO_STARTING_POS;
  }
}
int getServoMin(String limb_code) {
  if (limb_code == "LL") {
    return SERVO_LL_MIN;   
  } else if (limb_code == "LR") {
    return SERVO_LR_MIN;   
  } else if (limb_code == "UR") {
    return SERVO_UR_MIN;   
  } else if (limb_code == "UL") {
    return SERVO_UL_MIN;
  } else {
    return SERVO_STARTING_POS;
  }
}

void updateSliceIndex() {
  previous_start_slice_index = start_slice_index;
  start_slice_index += 3;

  int motion_array_length;
  if (direction_flag == "RIGHT") {
    motion_array_length = RIGHT_TURN_MOTION_LENGTH;
  } else { //forward
    motion_array_length = STRAIGHT_MOTION_LENGTH; 
  }
  
  if (start_slice_index >= motion_array_length) {
    start_slice_index =0 ;
  }
}


void loop(){

  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {  //timing loop
    
    //ultrasonic sensor
    uint16_t duration_uS; //uS = microseconds
    if (dist_sensor_time_counter >= 60/interval) { //eg. if interval = 10, then there only needs to be 6 passes to run this
      digitalWrite(TRIGGER, HIGH);
      delayMicroseconds(10); //send a 10uS trigger pulse
      digitalWrite(TRIGGER, LOW);
  
      duration_uS = pulseIn(ECHO, HIGH, 0);
      uint16_t obstacle_distance_mm = 0.5 * speed_of_sound * duration_uS * 1e-3;
    
      if (obstacle_distance_mm <= obstacle_distance_minimum) {
        direction_flag = "RIGHT";
      } else {
        direction_flag = "FORWARD";
      }
      dist_sensor_time_counter = 0; //reset
    }

    
    //determining limb movements
    if (previous_start_slice_index != start_slice_index) {
      
      if (direction_flag == "FORWARD") {
        limb_group = straight_motion_array[start_slice_index];
        limb_side = straight_motion_array[start_slice_index+1];
        leg_direction = straight_motion_array[start_slice_index+2];
        double_char_code = (String)limb_group + (String)limb_side; //cast chars to strings to prevent integer addition
        
      } else if (direction_flag == "RIGHT") {
        limb_group = right_turn_motion_array[start_slice_index];
        limb_side = right_turn_motion_array[start_slice_index+1];
        leg_direction = right_turn_motion_array[start_slice_index+2];
        double_char_code = (String)limb_group + (String)limb_side; //cast chars to strings to prevent integer addition
      }

    }
   
    //motion
    bool limb_movement_done; //servos don't finish their movement in one run (smoothing)
    if (isLimb(double_char_code)) {
      limb_movement_done = updateLimb(leg_direction, getLimb(double_char_code), getServoMax(double_char_code), getServoMin(double_char_code), limb_side);
      
    } else if (double_char_code == "AG") { //all limbs
      bool LL = updateLimb(leg_direction, LowerLeft, SERVO_LL_MAX, SERVO_LL_MIN, 'L');
      bool LR = updateLimb(leg_direction, LowerRight, SERVO_LR_MAX, SERVO_LR_MIN, 'R');
      bool UR = updateLimb(leg_direction, UpperRight, SERVO_UR_MAX, SERVO_UR_MIN, 'R');
      bool UL = updateLimb(leg_direction, UpperLeft, SERVO_UL_MAX, SERVO_UL_MIN, 'L');
      
      if (LL && LR && UR && UL) {
        limb_movement_done = true;
      } else {
        limb_movement_done = false;
      }
      
    } else if (double_char_code == "FG") { //front limbs
      bool UR = updateLimb(leg_direction, UpperRight, SERVO_UR_MAX, SERVO_UR_MIN, 'R');
      bool UL = updateLimb(leg_direction, UpperLeft, SERVO_UL_MAX, SERVO_UL_MIN, 'L');
      
      if ( UR && UL) {
        limb_movement_done = true;
      } else {
        limb_movement_done = false;
      }
      
    } else if (double_char_code == "BG") { //back limbs
      bool LL = updateLimb(leg_direction, LowerLeft, SERVO_LL_MAX, SERVO_LL_MIN, 'L');
      bool LR = updateLimb(leg_direction, LowerRight, SERVO_LR_MAX, SERVO_LR_MIN, 'R');
      
      if (LL && LR) {
        limb_movement_done = true;
      } else {
        limb_movement_done = false;
      }
      
    } else if (double_char_code == "LG") { //Left limbs
      bool LL = updateLimb(leg_direction, LowerLeft, SERVO_LL_MAX, SERVO_LL_MIN, 'L');
      bool UL = updateLimb(leg_direction, UpperLeft, SERVO_UL_MAX, SERVO_UL_MIN, 'L');
      
      if (LL && UL) {
        limb_movement_done = true;
      } else {
        limb_movement_done = false;
      }
      
    } else if (double_char_code == "RG") { //Right limbs
      bool LR = updateLimb(leg_direction, LowerRight, SERVO_LR_MAX, SERVO_LR_MIN, 'R');
      bool UR = updateLimb(leg_direction, UpperRight, SERVO_UR_MAX, SERVO_UR_MIN, 'R');
     
      if (LR && UR) {
        limb_movement_done = true;
      } else {
        limb_movement_done = false;
      }
      
    } else if (double_char_code == "DL") { //Diagonal Left (\)
      bool LR = updateLimb(leg_direction, LowerRight, SERVO_LR_MAX, SERVO_LR_MIN, 'R');
      bool UL = updateLimb(leg_direction, UpperLeft, SERVO_UL_MAX, SERVO_UL_MIN, 'L');
      
      if (LR && UL) {
        limb_movement_done = true;
      } else {
        limb_movement_done = false;
      }
      
    } else if (double_char_code == "DR") { //Diagonal Right (/)
      bool LL = updateLimb(leg_direction, LowerLeft, SERVO_LL_MAX, SERVO_LL_MIN, 'L');
      bool UR = updateLimb(leg_direction, UpperRight, SERVO_UR_MAX, SERVO_UR_MIN, 'R');
      
      if (LL && UR) {
        limb_movement_done = true;
      } else {
        limb_movement_done = false;
      }
      
    }

    //update servo positions
    LowerLeft.Joint.write(LowerLeft.smoothed_angle);
    LowerRight.Joint.write(LowerRight.smoothed_angle);
    UpperRight.Joint.write(UpperRight.smoothed_angle);
    UpperLeft.Joint.write(UpperLeft.smoothed_angle);
    
    //update variables
    previousMillis = currentMillis; 
    dist_sensor_time_counter++; 
    
    if (limb_movement_done) {
      updateSliceIndex(); //next run gets the next 3 instruction chars
    }
    
  } // end of main loop
  
}
