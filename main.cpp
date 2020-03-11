// ME210 Final Project
// Team 21: Those Meddling Kids
// March 8, 2020

// Libraries 
#include <Arduino.h>
#include <Metro.h>
#include <Servo.h>

// Pin Definitions
#define IR_INPUT_1                14
#define OUTER_TAPE_SENSOR         19 
#define MOTOR_DIRECTION_LEFT_1    9
#define MOTOR_DIRECTION_LEFT_2    10
#define MOTOR_DIRECTION_RIGHT_1   4
#define MOTOR_DIRECTION_RIGHT_2   3
#define ANGLE_PIN                 16  
#define LED_PIN                   13
#define LIMIT_SWITCH_PIN          22

// Threshold Definitions
#define LINE_THRESHOLD              750     // out of 1023
#define TURNING_DUTY_CYCLE          155     // out of 256
#define DRIVING_DUTY_CYCLE          255
#define BACKUP_TIME_INTERVAL        400
#define GAME_INTERVAL               130000 // 2 min & 10 seconds   
#define TURN_30_DEGREES_INTERVAL    300
#define TURN_90_DEGREES_INTERVAL    750   
#define TURN_180_DEGREES_INTERVAL   2000
#define TURN_360_DEGREES_INTERVAL   4000 

// Function Prototypes
void checkGlobalEvents(void); 
void turnClockwise(void);
void driveBackwards(void);
void driveForwards(void);
void respondToBeacon(void);
uint8_t lineDetected(uint8_t sensor);
void respondToLine(void);
void respondToTurningTimer(void);
void stopEverything(void);
void getIRSignal(void);
void disengageCaster(); 
void reengageCaster();
void moveOutwards(void);
void respondToBackupTimer(void);
void turnClockwise30(void);
uint8_t limitSwitchActivated(void); 
void getMaxSignal(void);

// Global Variables 
static Metro gameTimer = Metro(GAME_INTERVAL);
static Metro backwardsTimer = Metro(BACKUP_TIME_INTERVAL);
static Metro turn30deg = Metro(TURN_30_DEGREES_INTERVAL);
static Metro turn90deg = Metro(TURN_90_DEGREES_INTERVAL);
static Metro turn180deg = Metro(TURN_180_DEGREES_INTERVAL);
static Metro turn360deg = Metro(TURN_360_DEGREES_INTERVAL);
Servo casterWheelAngle;  

static uint8_t battleTime = true; 
uint16_t angle = 110; 
uint16_t maxIRSignal = 0; 
uint16_t IR_THRESHOLD = 0; 

typedef enum {
 BEACON_SEARCH, ORIENT_OUTWARDS, MOVING_OUTWARDS, BACKING_UP_FROM_LINE, GET_TANGENT, LINE_FOLLOWING_CW, 
 GOING_FORWARD, TURNING_CW, BACKING_UP, FORWARDS_ONLY
} States_t;

States_t state;
States_t lineFollowingState; 

void setup() {
  gameTimer.reset(); 
  pinMode(IR_INPUT_1, INPUT); 
  pinMode(OUTER_TAPE_SENSOR, INPUT); 
  pinMode(LIMIT_SWITCH_PIN, INPUT);
  pinMode(MOTOR_DIRECTION_LEFT_1, OUTPUT);
  pinMode(MOTOR_DIRECTION_RIGHT_1, OUTPUT);
  pinMode(MOTOR_DIRECTION_LEFT_2, OUTPUT);
  pinMode(MOTOR_DIRECTION_RIGHT_2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  casterWheelAngle.attach(ANGLE_PIN);
  casterWheelAngle.write(angle);
  digitalWrite(LED_PIN, HIGH);
  
  state = BEACON_SEARCH; // initialize looking for beacon 
  turnClockwise(); // turn robot to locate IR signal
  attachInterrupt(digitalPinToInterrupt(IR_INPUT_1), getMaxSignal,RISING);
  turn360deg;
}

void loop() {
  while (battleTime){
    checkGlobalEvents(); 
    switch (state) {
      case BEACON_SEARCH:
        if (turn360deg.check()) {
          IR_THRESHOLD = maxIRSignal / 10 * 9; 
          detachInterrupt(IR_INPUT_1);
          attachInterrupt(digitalPinToInterrupt(IR_INPUT_1), getIRSignal, RISING);
        }
        break;
      case ORIENT_OUTWARDS: 
        if (turn180deg.check()) moveOutwards(); 
        break; 
      case MOVING_OUTWARDS:
        if (lineDetected(OUTER_TAPE_SENSOR)) respondToLine(); 
        break;
      case BACKING_UP_FROM_LINE: 
        if (backwardsTimer.check()) respondToBackupTimer(); 
        break; 
      case GET_TANGENT:
        if (turn90deg.check()) respondToTurningTimer();  
        break; 
      case LINE_FOLLOWING_CW:
        switch(lineFollowingState){
          case GOING_FORWARD:
            if (lineDetected(OUTER_TAPE_SENSOR)) { 
              driveBackwards();
              backwardsTimer.reset();
              }
            break;
          case BACKING_UP: 
            if (backwardsTimer.check()) turnClockwise30(); 
            break;
          case TURNING_CW: 
            if (turn30deg.check()) driveForwards(); 
            break;
          default: break; 
        }
        break; 
      case FORWARDS_ONLY:
        if (!limitSwitchActivated()) {
          lineFollowingState = GOING_FORWARD; 
          state = LINE_FOLLOWING_CW; 
          }
        break; 
      default:    // Should never get into an unhandled state
        Serial.println("What is this I do not even...");
        break; 
    }
  }
}

void checkGlobalEvents(){
  if (gameTimer.check()) stopEverything(); 
  if (limitSwitchActivated() && (state != FORWARDS_ONLY)) {
    driveForwards(); 
    state = FORWARDS_ONLY; 
  }
}

void turnClockwise(){
  // Powers all 4 main body motors to the same speed, & sets left side to forward & right side to backwards
  analogWrite(MOTOR_DIRECTION_LEFT_1, LOW);
  analogWrite(MOTOR_DIRECTION_LEFT_2, LOW);
  analogWrite(MOTOR_DIRECTION_RIGHT_1, LOW);
  analogWrite(MOTOR_DIRECTION_RIGHT_2, LOW);
  turn90deg.reset();
  lineFollowingState = TURNING_CW;
  analogWrite(MOTOR_DIRECTION_LEFT_1, TURNING_DUTY_CYCLE);
  analogWrite(MOTOR_DIRECTION_LEFT_2, LOW);
  analogWrite(MOTOR_DIRECTION_RIGHT_1, LOW);
  analogWrite(MOTOR_DIRECTION_RIGHT_2, TURNING_DUTY_CYCLE);
}

void driveForwards(){
  // Sets directions of both motors to forwards 
  analogWrite(MOTOR_DIRECTION_LEFT_1, LOW);
  analogWrite(MOTOR_DIRECTION_LEFT_2, LOW);
  analogWrite(MOTOR_DIRECTION_RIGHT_1, LOW);
  analogWrite(MOTOR_DIRECTION_RIGHT_2, LOW);
  disengageCaster(); 
  lineFollowingState = GOING_FORWARD; 
  analogWrite(MOTOR_DIRECTION_LEFT_1, DRIVING_DUTY_CYCLE);
  analogWrite(MOTOR_DIRECTION_LEFT_2, LOW);
  analogWrite(MOTOR_DIRECTION_RIGHT_1, DRIVING_DUTY_CYCLE);
  analogWrite(MOTOR_DIRECTION_RIGHT_2, LOW);
}

void driveBackwards(){
  // Sets directions of both motors to backwards 
  analogWrite(MOTOR_DIRECTION_LEFT_1, LOW);
  analogWrite(MOTOR_DIRECTION_LEFT_2, LOW);
  analogWrite(MOTOR_DIRECTION_RIGHT_1, LOW);
  analogWrite(MOTOR_DIRECTION_RIGHT_2, LOW);
  reengageCaster();
  lineFollowingState = BACKING_UP; 
  backwardsTimer.reset(); 
  analogWrite(MOTOR_DIRECTION_LEFT_1, LOW);
  analogWrite(MOTOR_DIRECTION_LEFT_2, DRIVING_DUTY_CYCLE);
  analogWrite(MOTOR_DIRECTION_RIGHT_1, LOW);
  analogWrite(MOTOR_DIRECTION_RIGHT_2, DRIVING_DUTY_CYCLE);
}

void disengageCaster(){
  if (angle == 110) {
    angle = 0;
    casterWheelAngle.write(angle);
  } else {
    casterWheelAngle.write(angle);
  }
}

void reengageCaster(){
  if (angle == 0) {
    angle = 110;
    casterWheelAngle.write(angle);
  } else {
    casterWheelAngle.write(angle); 
  }
}

void getMaxSignal(){ 
  // If new max signal detected on rising edge, updates max value 
  uint16_t IR_Value = analogRead(IR_INPUT_1);
  if (IR_Value > maxIRSignal) { 
    maxIRSignal = IR_Value; 
  }
}

void getIRSignal() {
  // Compares current IR signal to 90% of max signal found in calibration stage (getMaxSignal function)
  uint16_t IR_Value = analogRead(IR_INPUT_1);
  if (IR_Value > IR_THRESHOLD){
    respondToBeacon(); 
  }
}

void respondToBeacon(){
  // Reverses motors, sets state to moving toward outer circle 
  detachInterrupt(digitalPinToInterrupt(IR_INPUT_1));
  turnClockwise();
  turn180deg.reset();
  state = ORIENT_OUTWARDS; 
}

uint8_t lineDetected(uint8_t sensor){
  // Checks to see if sensor is detecting reflectance (low value) or absorbance (high value)
  uint16_t lineValue = analogRead(sensor);
  if (lineValue > LINE_THRESHOLD) return true; 
  return false; 
}

uint8_t limitSwitchActivated(){
  // Normally open limit switch, checks to see if it is closed 
  uint8_t limitSwitch = digitalRead(LIMIT_SWITCH_PIN);
  return limitSwitch; 
}

void respondToLine(){
  driveBackwards();
  backwardsTimer.reset(); 
  state=  BACKING_UP_FROM_LINE;
}

void moveOutwards(){
  driveForwards();
  state = MOVING_OUTWARDS; 
  attachInterrupt(digitalPinToInterrupt(IR_INPUT_1), getIRSignal,RISING);
}

void respondToTurningTimer(){
  driveForwards();
  state = LINE_FOLLOWING_CW;
  lineFollowingState = GOING_FORWARD;
}

void respondToBackupTimer(){
  turnClockwise(); 
  turn90deg.reset(); 
  state = GET_TANGENT; 
}

void turnClockwise30(){
  turn30deg.reset(); 
  turnClockwise(); 
  lineFollowingState = TURNING_CW; 
}

void stopEverything() {
  analogWrite(MOTOR_DIRECTION_LEFT_1, LOW);
  analogWrite(MOTOR_DIRECTION_LEFT_2, LOW);
  analogWrite(MOTOR_DIRECTION_RIGHT_1, LOW);
  analogWrite(MOTOR_DIRECTION_RIGHT_2, LOW);
  casterWheelAngle.write(110);
  digitalWrite(LED_PIN, LOW);
  battleTime = false; 
}