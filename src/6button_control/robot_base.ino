#include <Servo.h>


// Objects
Servo baseServo;

Servo arm1Servo;

Servo arm2Servo;

// Variables -- establishing variables for clockwise and counterclockwise pins for arm1, arm2, and base
int CW_base_pin = 0;
bool CW_base_state = false;

int CW_arm1_pin = 0;
bool CW_arm1_state = false;

int CW_arm2_pin = 0;
bool CW_arm2_state = false;

int CCW_base_pin = 0;
bool CCW_base_state = false;

int CCW_arm1_pin = 0;
bool CCW_arm1_state = false;

int CCW_arm2_pin = 0;
bool CCW_arm2_state = false;

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
  baseServo.attach(1); // attaches the servo on pin - to the servo object baseServo

  arm1Servo.attach(2); // attaches the servo on pin - to the servo object arm1Servo
  
  arm2Servo.attach(3); // attaches the servo on pin - to the servo object arm2Servo

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
  if (CW_base_state) {
    ++base_angle;
  } 
  if (CCW_base_state) {
    --base_angle;
  }
  // Update servo
  val_base = map(base_angle, -105, 105, 0, 180);
  baseServo.write(val_base);


  // Check arm1 servo buttons, and update servo pos. If needed
  if (CW_arm1_state) {
    ++arm1_angle;
  } 
  if (CCW_arm1_state) {
    --arm1_angle;
  }
  // Update servo
  val_arm1 = map(arm1_angle, -105, 105, 0, 180);
  arm1Servo.write(val_arm1);

  
  // Check Base servo buttons, and update servo pos. If needed
  if (CW_arm2_state) {
    ++arm2_angle;
  } 
  if (CCW_arm2_state) {
    --arm2_angle;
  }
  // Update servo
  val_arm2 = map(arm2_angle, -105, 105, 0, 180);
  arm2Servo.write(val_arm2);

  delay(25);
  
}




//Look at knob example for val variable and BaseServo -- you need to initialize the val variable and the BaseServo object
