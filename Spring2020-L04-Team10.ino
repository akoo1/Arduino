/*
  KNW Project
*/

// --- Definitions --- //
// Motor Pins
#define SERVO_ONE 3
#define SERVO_TWO 4
#define SERVO_THREE 5
// Directions
#define BACKWARD -1
#define FORWARD 1
#define LEFT 2
#define RIGHT 3
// Ping Sensor(s)
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define BACK_MOTOR 2

#include <cRobot.h>
#include <LiquidCrystal_I2C.h>
#include <Time.h>
#include <TimeAlarms.h>

cRobot* robot;
LiquidCrystal_I2C lcd(0x3F, 16, 2);
// IR ir(4);

void setup() {
  robot = new cRobot();
  robot->setupLCD();
  robot->setupPings();
  robot->setupPWM();
  robot->setupKeypad();
  // lcd.init();
  // lcd.begin(16, 2);
  // lcd.backlight();
  // lcd.autoscroll();
  
  Serial.begin(9600);
  delay(500);
  
  // ir.init();
}

double dist = robot->getPing(0);

void goThreeMeters() {
  robot->clearLCD();
  robot->moveCursor(0, 0);
  robot->printLCD("1...");
  delay(1000);
  robot->printLCD("2...");
  delay(1000);
  robot->printLCD("3...");
  delay(1000);
  robot->printLCD("Go!");
  dist = robot->getPing(0);
  robot->moveCursor(0, 1);
  for (int i = 0; i < 8; i++) { // do this loop for up to 1000mS
    dist = robot->getPing(0);
    robot->setMotorSpeed(LEFT_MOTOR, 1);
    robot->setMotorSpeed(RIGHT_MOTOR, 1);
    if (dist >= 20)
      robot->printLCD("O");
    else
      robot->printLCD("X");
      
    delay(500);
  }
  robot->setMotorSpeed(LEFT_MOTOR, 0);
  robot->setMotorSpeed(RIGHT_MOTOR, 0);
}

void stopMotors() {robot->stopMotors();}

void loop() {
  Serial.print("Distance: ");
  Serial.print(dist);
  Serial.println(" cm");
  robot->stopMotors();
  
  robot->moveCursor(0, 0);
  
  int key = robot->getKeypadInput();
  // robot->printLCD(key);
  
  if (key) {
    robot->printLCD(key);
    if (key == 'A')
      goThreeMeters();
  }
  
  // while (1) delay(5000);
  
  
  
  // while (true) {
  //   dist = robot->getPing(0);
  //   robot->setMotorSpeed(LEFT_MOTOR, 1);
  //   robot->setMotorSpeed(RIGHT_MOTOR, 1);
  //   if (dist >= 10)
  //     robot->printLCD("X");
  //   else
  //     robot->printLCD("O");
  // }
  
  // goThreeMeters();
  // Alarm.timerOnce(3, stopMotors);
    
}
