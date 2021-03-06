#include <Servo.h>


// Objects
Servo baseServo;

Servo arm1Servo;

Servo arm2Servo;


// Defined Variables
#define LOOP_DELAY 25
#define MAX_ANGLE 105
#define MIN_ANGLE -105


// Variables -- establishing variables for clockwise and counterclockwise pins for arm1, arm2, and base
int CW_base_pin = 8;
bool CW_base_state = true;

int CW_arm1_pin = 9;
bool CW_arm1_state = true;

int CW_arm2_pin = 10;
bool CW_arm2_state = true;

int CCW_base_pin = 11;
bool CCW_base_state = true;

int CCW_arm1_pin = 12;
bool CCW_arm1_state = true;

int CCW_arm2_pin = 13;
bool CCW_arm2_state = true;

int val_base;

int val_arm1;

int val_arm2;



// going from -105 to 105 (servo goes from 0 - 180, so you have to map // your angles)
int base_angle = 0;

int arm1_angle = 0;

int arm2_angle = 0;


void setup() {
  // put your setup code here, to run once:

  // Initialize 3 servos
  baseServo.attach(5); // attaches the servo on pin 5 to the servo object baseServo

  arm1Servo.attach(6); // attaches the servo on pin 6 to the servo object arm1Servo
  
  arm2Servo.attach(7); // attaches the servo on pin 7 to the servo object arm2Servo

  // Initializing button pins as inputs
  pinMode(CW_base_pin, INPUT_PULLUP);

  pinMode(CW_arm1_pin, INPUT_PULLUP);

  pinMode(CW_arm2_pin, INPUT_PULLUP);

  pinMode(CCW_base_pin, INPUT_PULLUP);

  pinMode(CCW_arm1_pin, INPUT_PULLUP);

  pinMode(CCW_arm2_pin, INPUT_PULLUP);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  // Read 6 GPIO
  CW_base_state = digitalRead(CW_base_pin);

  CW_arm1_state = digitalRead(CW_arm1_pin);

  CW_arm2_state = digitalRead(CW_arm2_pin);

  CCW_base_state = digitalRead(CCW_base_pin);

  CCW_arm1_state = digitalRead(CCW_arm1_pin);

  CCW_arm2_state = digitalRead(CCW_arm2_pin);


  // Check Base servo buttons, and update servo pos. If needed
  if (!CW_base_state) {
    ++base_angle;
  } 
  if (!CCW_base_state) {
    --base_angle;
  }

  base_angle = base_angle > MAX_ANGLE ? MAX_ANGLE : base_angle;
  base_angle = base_angle < MIN_ANGLE ? MIN_ANGLE : base_angle;
  
  // Update servo
  val_base = map(base_angle, MIN_ANGLE, MAX_ANGLE, 0, 180);
  baseServo.write(val_base);


  // Check arm1 servo buttons, and update servo pos. If needed
  if (!CW_arm1_state) {
    ++arm1_angle;
  } 
  if (!CCW_arm1_state) {
    --arm1_angle;
  }

  arm1_angle = arm1_angle > MAX_ANGLE ? MAX_ANGLE : arm1_angle;
  arm1_angle = arm1_angle < MIN_ANGLE ? MIN_ANGLE : arm1_angle;
  
  // Update servo
  val_arm1 = map(arm1_angle, MIN_ANGLE, MAX_ANGLE, 0, 180);
  arm1Servo.write(val_arm1);

  
  // Check arm2 servo buttons, and update servo pos. If needed
  if (!CW_arm2_state) {
    ++arm2_angle;
  } 
  if (!CCW_arm2_state) {
    --arm2_angle;
  }
  
  arm2_angle = arm2_angle > MAX_ANGLE ? MAX_ANGLE : arm2_angle;
  arm2_angle = arm2_angle < MIN_ANGLE ? MIN_ANGLE : arm2_angle;
  
  // Update servo
  val_arm2 = map(arm2_angle, MIN_ANGLE, MAX_ANGLE, 0, 180);
  arm2Servo.write(val_arm2);

  delay(LOOP_DELAY);
  
}




//Look at knob example for val variable and BaseServo -- you need to initialize the val variable and the BaseServo object
