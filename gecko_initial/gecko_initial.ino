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
};


//Misc.
#define speed_of_sound 340 //metres per second
#define len 100 
uint32_t values[len]; 
uint8_t counter = 0;

#define obstacle_distance_minimum 100 //mm
uint16_t obstacle_distance_mm; //mm

#define SERVO_STARTING_POS 90 //not neccessarily 90, rather half way for ServoObj.write(...) method
String current_leg = "LL"; //LL->Lower Left, .....
char current_direction = 'F'; //F->Forward

//Lower Left Limb
#define LOWER_LEFT_SERVO_PIN 2
#define LOWER_LEFT_EM_PIN 6 //L298N En_A -> Out_1
Limb LowerLeft;

//Lower Right Limb
#define LOWER_RIGHT_SERVO_PIN 3
#define LOWER_RIGHT_EM_PIN 7 //L298N En_B -> Out_2
Limb LowerRight;

//Upper Right Limb
#define UPPER_RIGHT_SERVO_PIN 4
#define UPPER_RIGHT_EM_PIN 8 //L298N En_C -> Out_3
Limb UpperRight;

//Upper Left Limb
#define UPPER_LEFT_SERVO_PIN 5
#define UPPER_LEFT_EM_PIN 9 //L298N En_D -> Out_4
Limb UpperLeft;


//Ultrasonic sensor
#define TRIGGER 11
#define ECHO  12



void setup() {
  LowerLeft.Joint.attach(LOWER_LEFT_SERVO_PIN);
  LowerLeft.Joint.write(SERVO_STARTING_POS);
  LowerLeft.EM.attach(LOWER_LEFT_EM_PIN);
  LowerLeft.EM.write(1);
  
  LowerRight.Joint.attach(LOWER_RIGHT_SERVO_PIN);
  LowerRight.Joint.write(SERVO_STARTING_POS);
  LowerRight.EM.attach(LOWER_RIGHT_EM_PIN);
  LowerRight.EM.write(1);

  UpperRight.Joint.attach(UPPER_RIGHT_SERVO_PIN);
  UpperRight.Joint.write(SERVO_STARTING_POS);
  UpperRight.EM.attach(UPPER_RIGHT_EM_PIN);
  UpperRight.EM.write(1);

  UpperLeft.Joint.attach(UPPER_LEFT_SERVO_PIN);
  UpperLeft.Joint.write(SERVO_STARTING_POS);
  UpperLeft.EM.attach(UPPER_LEFT_EM_PIN);
  UpperLeft.EM.write(1);

  
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIGGER, LOW);

  Serial.begin(9600);


  delay(2000);
  
}

void stepLegUp(String leg) {
  Serial.println("Stepping up");
  if (leg == "LL") { ////Lower left
    LowerLeft.EM.write(0); //turn off
    LowerLeft.Joint.write(180); //move up
    delay(1000);
    LowerLeft.EM.write(1); //turn on
    LowerLeft.Joint.write(90);
    
  } else if (leg == "LR") { ////lower right
    LowerRight.EM.write(0); //turn off
    LowerRight.Joint.write(180); //move up
    delay(1000);
    LowerRight.EM.write(1); //turn on
    LowerRight.Joint.write(90);
    
  } else if (leg == "UR") { ////upper right
    UpperRight.EM.write(0); //turn off
    UpperRight.Joint.write(180); //move up
    delay(1000);
    UpperRight.EM.write(1); //turn on
    UpperRight.Joint.write(90);
    
  } else if (leg == "UL") { ////upper left
    UpperLeft.EM.write(0); //turn off
    UpperLeft.Joint.write(180);
    delay(1000);
    UpperLeft.EM.write(1); //turn on
    UpperLeft.Joint.write(90);
    
  }
}

void stepLegDown(String leg) {
  Serial.println("Stepping down");
  if (leg == "LL") { ////Lower left
    LowerLeft.EM.write(0); //turn off
    LowerLeft.Joint.write(0); //move down
    delay(1000);
    LowerLeft.EM.write(1); //turn on
    LowerLeft.Joint.write(90);
    
  } else if (leg == "LR") { ////lower right
    LowerRight.EM.write(0); //turn off
    LowerRight.Joint.write(0); //move down
    delay(1000);
    LowerRight.EM.write(1); //turn on
    LowerRight.Joint.write(90);
    
  } else if (leg == "UR") { ////upper right
    UpperRight.EM.write(0); //turn off
    UpperRight.Joint.write(0); //move down
    delay(1000);
    UpperRight.EM.write(1); //turn on
    UpperRight.Joint.write(90);
    
  } else if (leg == "UL") { ////upper left
    UpperLeft.EM.write(0); //turn off
    UpperLeft.Joint.write(0); //move down
    delay(1000);
    UpperLeft.EM.write(1); //turn on
    UpperLeft.Joint.write(90);
    
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
 
}
