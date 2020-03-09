/*
 cRobot.h - Library for cRobot code.
 Created by Griffin P. Foster, March 3, 2020.
 Released into the public domain.
*/

#ifndef SOFTWARE_CROBOT_H
#define SOFTWARE_CROBOT_H

#define FORWARD 1
#define BACKWARD -1
#define LEFT 2
#define RIGHT 3
#define ZONE_ONE 5
#define ZONE_TWO 6
#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define BACK_LEFT 2
#define BACK_RIGHT 3
#define SIDE_LEFT 4
#define SIDE_RIGHT 5

#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_PWMServoDriver.h>

class cRobot {
protected:
    // Instance variables used in conjunction with the IR sensor
     unsigned char necState;
     int num_chars;
     unsigned long prev_time;

     unsigned char IRChar, IRCharBitMask, buffer[8];
     bool receiverState = false;
     unsigned long cur_time, ticks;
     // Instance variables used in conjunction with the keypad
     bool entered;
     int numEntered;
    // char DATA[17];
     byte ROWS = 4;
     byte COLS = 4;
     char keys[4][4] = {
         {'1', '2', '3', 'A'},
         {'4', '5', '6', 'B'},
         {'7', '8', '9', 'C'},
         {'*', '0', '#', 'D'}};

     byte rowPins[4] = {44, 42, 40, 38};
     byte colPins[4] = {39, 41, 43, 45};
     
private:
    LiquidCrystal_I2C* _lcd;
    Adafruit_PWMServoDriver* _pwm;
    int _direction = FORWARD;
    Keypad* _keypad = nullptr;
    bool _entered;
    int _numEntered;
    char DATA[17];
    int _zone = ZONE_ONE;

public:
// ------ Main ------ //
    cRobot();
    ~cRobot();
    void EmergencyStop();
    // void setZone(int zone) { _zone = zone; };
// ------ END ------ //

// ------ Ping Sensor(s) ------ //
    void setupPing(int id);
    void setupPings();
    double getPing(int id);
    // Check for objects in direction
    bool frontIsClear();
    bool leftIsClear();
    bool rightIsClear();
    bool backIsClear();
    // Print Distances
    void printDistances();
// ------ END ------ //

// ------ Servo Motor(s) ------ //
    void setupServos();
    double getAngle(int id);
    void setAngle(int id, int angle);
    double calculateAngle(int angle);
    void setTempAngle(int id, int angle, int delay_s);
    void printAngle(int id);

// ------ END ------ //

// ------ DC Motor(s) ------ //
//    void setupMotor(int);
    void setupPWM();
    void setMotorSpeed(int id, int speed);
    void setSpeed(int speed);
    void setDirection(int direction); // 0:backward 1:forward 2:left 3:right
    // STOP
    void stopMotor(int pin);
    void stopMotors();
    void stopLeftMotor();
    void stopRightMotor();
    void stopBackMotor();
// ------ END ------ //

// ------ Inclinometer ------ //
    void setupIncline(int pin);
    // Get incline
    int getIncline();
// ------ END ------ //

// ------ Keypad ------ //
    void setupKeypad();
    void resetKeypad();
    int getKeypadInput();
    int getKeypadInput(int row);
// ------ END ------ //

// ------ LCD ------ //
    void setupLCD(long address = 0x3F);
    void resetLCD(long address = 0x3F);
    void clearLCD();
    void moveCursor(int col, int row);
    void clearLine(int row);
    void printLCD(char* input);
    void printLCD(int input);
    void printLCD(long input);
    void printLCD(double input, short decimalPlaces);
    void printLCD(double input);
    void printLCD(char input);
// ------ END ------ //

// ------ Salinity Sensor ------ //
    void setupSalinitySensor();
    void deploySalinitySensor();
    void deploySalinitySensor(int duration); // deploy for specific duration
    float checkSalinity();
// ------ END ------ //

  int scanIR(int id);
  unsigned char* getIR();
};


#endif //SOFTWARE_CROBOT_H
