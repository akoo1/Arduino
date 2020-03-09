//
// Created by Griff on 3/3/2020.
//

//#include <Arduino.h>
//#if ARDUINO >= 100
//#include <Arduino.h>
//#else
//#include <WProgram.h>
//#include <pins_arduino.h>
//#include <WConstants.h>
//
//#endif
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

#include "cRobot.h"

#define FORWARD 1
#define BACKWARD -1
#define LEFT 2
#define RIGHT 3
#define CLEAR_DISTANCE 10.0
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define BACK_MOTOR 2
#define TRIG 0
#define ECHO 1
#define SALINITY_SERVO 0
#define TWO_SERVO 1
#define THREE_SERVO 2
#define DC_INPUT_RANGE 1023
#define DC_CENTER 337
#define DC_WIDTH 183
#define DC_MIN DC_CENTER - DC_WIDTH
#define DC_MAX DC_CENTER + DC_WIDTH
#define TOP_SPEED 100 // speed (x/100)
#define SERVO_FREQUENCY 60
#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define PCA_SERVO_180_INPUT_RANGE 120
#define PCA_SERVO_180_CENTER 400
// Configured to be greater than +/- 90deg (240ish for 90degrees),
// because the students will need to calibrate the servos.
#define PCA_SERVO_180_WIDTH 320
// Calculations based on above configuration.
#define PCA_SERVO_180_MIN PCA_SERVO_180_CENTER - PCA_SERVO_180_WIDTH
#define PCA_SERVO_180_MAX PCA_SERVO_180_CENTER + PCA_SERVO_180_WIDTH
#define DC_FREQUENCY 50 // ~50 - 60
#define DC_INPUT_RANGE 1023
#define DC_CENTER 337
#define DC_WIDTH 183
#define DC_MIN DC_CENTER - DC_WIDTH
#define DC_MAX DC_CENTER + DC_WIDTH
#define TOP_SPEED 100 // speed (x/100)
#define SERVO_FREQUENCY 60
#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define SPEED_OF_SOUND 776.5
// IR DETAILS
#define EVENT_RISING 1
#define EVENT_FALLING 2
#define EVENT_NONE 0
#define IR_WAIT 100000

int ping_pins[4][2] = {{52, 53},
                       {50, 51},
                       {48, 49},
                       {46, 47}};
int motor_pins[3] = {0, 1, 2};
int servo_pins[3] = {3, 4, 5};
int keypad_pins[8] = {45, 44, 43, 42, 41, 40, 39, 38}; // left->right (wires)
char keys[4][4] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
};
byte ROWS = 4;
byte COLS = 4;
byte _rowPins[4] = {39, 41, 43, 45};
byte _colPins[4] = {47, 49, 51, 53};

/*
 * @comp: cRobot
 * @desc: Setup cRobot.
 */
cRobot::cRobot() {
    _lcd = nullptr;
    _pwm = nullptr;
    _keypad = nullptr;
    _direction = FORWARD;
}
cRobot::~cRobot() {
    delete[] _lcd;
    delete[] _pwm;
    delete[] _keypad;
    _direction = 0;
    _zone = 0;
}

/*
 * @comp: General
 * @desc: Emergency Stop in event of abrupt error.
 */
void cRobot::EmergencyStop() {
    stopMotors();
//    Serial.println("ATTENTION: Emergency Stop has been activated and all motors have stopped!");
}

/*
 * @comp: Ping Sensor(s)
 * @desc: Check if front|back|left|right is clear (inches).
 */
bool cRobot::frontIsClear() { return getPing(FRONT_LEFT) > CLEAR_DISTANCE && getPing(FRONT_RIGHT) > CLEAR_DISTANCE; }
bool cRobot::backIsClear() { return getPing(BACK_LEFT) > CLEAR_DISTANCE && getPing(BACK_RIGHT) > CLEAR_DISTANCE; }
bool cRobot::leftIsClear() { return getPing(SIDE_LEFT) > CLEAR_DISTANCE; }
bool cRobot::rightIsClear() { return getPing(SIDE_RIGHT) > CLEAR_DISTANCE; }

/*
 * @comp: Ping Sensor(s)
 * @desc: Setup all Ping Sensors.
 */
void cRobot::setupPing(int id) {
    pinMode(ping_pins[id][TRIG], OUTPUT);
    pinMode(ping_pins[id][ECHO], INPUT);
}
void cRobot::setupPings() {
    for (int i = 0; i < 4; i++)
        setupPing(i);
}
/*
 * @comp: Ping Sensor(s)
 * @desc: Get distance of Ping Sensor.
 */
double cRobot::getPing(int id) {
    // double distance = 0, ping = 0;
    // digitalWrite(ping_pins[id][TRIG], LOW);
    // delayMicroseconds(2000);
    // digitalWrite(ping_pins[id][TRIG], HIGH);
    // delayMicroseconds(15);
    // digitalWrite(ping_pins[id][TRIG], LOW);
    // delayMicroseconds(10);

    // ping = pulseIn(ping_pins[id][ECHO], HIGH);
    
    // ping = ping / 1000000;          // Convert distance to seconds by dividing by 1000000 (microseconds in a second)
    // ping = ping / 3600;             // Convert distance to hours by dividing by 3600 (seconds in an hour)
    // distance = SPEED_OF_SOUND * ping; // This will be in miles, since speed of sound was miles per hour
    // distance = distance / 2;        // Divide distance by 2 to account for travel back and forth
    // distance = distance * 63360;    // Convert miles to inches by multipling by 63360 (inches per mile)
    // return distance;
    long duration, cm;

    // The PING is triggered by a HIGH pulse of 2 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(ping_pins[id][TRIG], LOW);
    delayMicroseconds(2);
    digitalWrite(ping_pins[id][TRIG], HIGH);
    delayMicroseconds(5);
    digitalWrite(ping_pins[id][TRIG], LOW);
    
    duration = pulseIn(ping_pins[id][ECHO], HIGH); // returned in microseconds ms

    // convert the time into a distance
    // 73.746 ms per inch (sound travels at 1130 ft/sec)
    // 29ish ms per cm (roughly 342 m/sec)
    // duration is time there and back, so divid by 2
    return cm = duration / 29 / 2;
}


/*
 * @comp: Ping Sensor(s)
 * @desc: Print distances in Serial Monitor from all Ping Sensors.
 */
void cRobot::printDistances() {
    _lcd->print(getPing(FRONT_LEFT));
    _lcd->print(getPing(FRONT_RIGHT));
    _lcd->setCursor(0, 1);
    _lcd->print(getPing(BACK_LEFT));
    _lcd->print(getPing(BACK_RIGHT));
}

/*
 * @comp: Motor(s)
 * @desc: Setup all DC Motors.
 */
void cRobot::setupPWM() {
    _pwm = new Adafruit_PWMServoDriver();
    _pwm->begin();
    _pwm->setPWMFreq(DC_FREQUENCY);
}

/*
 * @comp: DC Motor(s)
 * @desc: Set speed of Robot.
 */
void cRobot::setMotorSpeed(int id, int speed) {
    // raw_input(-180) -> full speed
    // raw_input(125) -> complete stop
    // speed /= -180;
    if (speed == 0)
      return _pwm->setPWM(id, 0, 0);
    // speed /= -1.8;
    int pulseLength = map(
            speed,
            -1 * DC_INPUT_RANGE,
            DC_INPUT_RANGE,
            DC_MIN,
            DC_MAX
    );
    pulseLength = constrain(pulseLength, DC_MIN, DC_MAX);
    _pwm->setPWM(id, 0, pulseLength);
}
void cRobot::setSpeed(int speed) {
    switch (_direction) {
        case FORWARD: // Forward Movement
            setMotorSpeed(LEFT_MOTOR, speed);
            setMotorSpeed(RIGHT_MOTOR, speed);
            stopBackMotor();
            break;
        case BACKWARD: // Backward Movement
            setMotorSpeed(LEFT_MOTOR, -1 * speed);
            setMotorSpeed(RIGHT_MOTOR, -1 * speed);
            stopBackMotor();
            break;
        case LEFT: // Left Movement (Lateral)
            setMotorSpeed(LEFT_MOTOR, speed / 2);
            setMotorSpeed(RIGHT_MOTOR, -1 * speed / 2);
            setMotorSpeed(BACK_MOTOR, speed / 2);
            break;
        case RIGHT: // Right Movement (Lateral)
            setMotorSpeed(LEFT_MOTOR, -1 * speed / 2);
            setMotorSpeed(RIGHT_MOTOR, speed / 2);
            setMotorSpeed(BACK_MOTOR, -1 * speed / 2);
            break;
    }
}

/*
 * @comp: DC Motor(s)
 * @desc: Stop all DC Motors.
 */
void cRobot::stopMotors() {
    stopLeftMotor();
    stopRightMotor();
    stopBackMotor();
}

/*
 * @comp: DC Motor(s)
 * @desc: Stop specific motor (left|right|back).
 */
void cRobot::stopMotor(int id) { setMotorSpeed(id, 0); }
void cRobot::stopLeftMotor() { setMotorSpeed(LEFT_MOTOR, 0); }
void cRobot::stopRightMotor() { setMotorSpeed(RIGHT_MOTOR, 0); }
void cRobot::stopBackMotor() { setMotorSpeed(BACK_MOTOR, 0); }

/*
 * @comp: Servo Motor(s)
 * @desc: Setup Servo Motors.
 */
double cRobot::calculateAngle(int angle) {
//    int pulse_width, analog_value;
//    pulse_width = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
//    analog_value = int(float(pulse_width) / 1000000 * SERVO_FREQUENCY * 4096);
//    Serial.print("Analog Value: ");
//    Serial.println(analog_value);
//    return analog_value;
    int pulselen = map(
            angle,
            0 * PCA_SERVO_180_INPUT_RANGE,
            2 * PCA_SERVO_180_INPUT_RANGE,
            PCA_SERVO_180_MIN,
            PCA_SERVO_180_MAX);

    // Shift to range of; PWM Signal: 1ms - 2ms will give
    // full reverse to full forward, 1.5ms is neutral
    pulselen = constrain(pulselen, PCA_SERVO_180_MIN, PCA_SERVO_180_MAX);
    return pulselen;
}
void cRobot::setAngle(int id, int angle) {
    _pwm->setPWM(id, 0, calculateAngle(angle));
}
//void cRobot::setTempAngle(int id, int angle, int delay_s) {
//    int prevAngle = getAngle(id);
//    setAngle(id, angle);
//    delay(delay_s * 1000);
//    setAngle(id, prevAngle);
//}
//void cRobot::printAngle(int id) {
//    printLCD(getAngle(id));
//    printLCD("°");
//}

/*
 * @comp: Keypad
 * @desc: Setup Keypad.
 */
void cRobot::setupKeypad() {
    _keypad = new Keypad(makeKeymap(keys), _rowPins, _colPins, ROWS, COLS);
    _entered = false;
    _numEntered = 0;
}
/*
 * @comp: Keypad
 * @desc: Get Keypad Input.
 */
int cRobot::getKeypadInput() {
    _lcd->clear();
    return getKeypadInput(0);
}
int cRobot::getKeypadInput(int row)
{
    int inInt = -1;
    char key;
    bool letterflag = false;
    char letter;

    // to get multi input from keypad into int
    while (!entered)
    {
        key = _keypad->getKey();
        if (key)
        {
            // _lcd->setCursor(numEntered, row);
            // _lcd->print(key);

            // max input is 16, neglects most recent and enters
            if (key == '#' || numEntered == 16)
            {
                DATA[numEntered] = '\0'; // null temrinator for atoi conversion
                entered = true;
            }
            else if (key == '*' && numEntered > 0)
            { // backspace
                numEntered--;
                // _lcd->setCursor(numEntered, row);
                // _lcd->print(" ");
            }
            else if (key != '#' && key != '*')
            {
                DATA[numEntered] = key;
                numEntered++;
            }
            if (key == 'A' || key == 'B' || key == 'C' || key == 'D')
            {
                letterflag = true;
                letter = key;
            }
        }

        if (entered)
        {
            if (letterflag)
            {
                inInt = (int)letter;
            }
            else
            {
                inInt = atoi(DATA);
            }
            memset(DATA, 0, sizeof(DATA));
            numEntered = 0;
        }
    } // end while

    entered = false;
    return inInt; // returns -1 if nothing pressed
}

/*
 * @comp: Keypad
 * @desc: Clear Keypad Input.
 */
void cRobot::resetKeypad() {
    delete _keypad;
    _keypad = nullptr;
    setupKeypad();
}

/*
 * @comp: LCD Screen(s)
 * @desc: Setup LCD Display.
 */
void cRobot::setupLCD(long address) {
    _lcd = new LiquidCrystal_I2C(0x3F, 16, 2);
    _lcd->init();
    _lcd->begin(16, 2);
    _lcd->backlight();
    // _lcd->autoscroll();
    // _lcd->home();
}
void cRobot::resetLCD(long address) {
    // delete[] _lcd;
    // _lcd = nullptr;
    setupLCD(address);
}
void cRobot::moveCursor(int col, int row) {
    _lcd->setCursor(col, row);
}
void cRobot::clearLine(int row) {
    _lcd->setCursor(0, row);
    _lcd->print("                "); // 16 characters
    _lcd->setCursor(0, row);
}
void cRobot::clearLCD() {
    _lcd->clear();
}
void cRobot::printLCD(char* input) {
    _lcd->print(input);
}
void cRobot::printLCD(double input, short decimalPlaces) {
    long multiplier = pow(10, decimalPlaces);
    long wholeValue = (long) input;
    printLCD(wholeValue);             //whole number value
    printLCD('.');              //decimal point
    input -= wholeValue;              //gives us just the digits after the decimal
    input *= multiplier;              //put digits we want to print in front of decimal
    long decimalDigits = (long) input; //discard everything we won't use
    printLCD(decimalDigits);          //print remaining digits as param specifies
}
void cRobot::printLCD(double input) {
    printLCD(input, (short) 3);
}
void cRobot::printLCD(int input) {
    _lcd->print(input);
}
void cRobot::printLCD(long input) {
    _lcd->print(input);
}
void cRobot::printLCD(char input) {
    _lcd->print(input);
}

int cRobot::scanIR(int id)
{
  // takes 13 ms per char to broadcast from a beacon
  char reading;
  unsigned char event;
  // reset the buffer
  memset(buffer, 0, sizeof(buffer));
  num_chars = 0;

  unsigned long IRCounter = 0;

  // Cycle while we try to watch for a character.
  while (IRCounter < IR_WAIT)
  {
      // Digital level from IR receiver will be inverted
      if (digitalRead(id))
      {
          if (receiverState)
              event = EVENT_FALLING;
          else
              event = EVENT_NONE;
          receiverState = false;
      }
      else
      {
          if (!receiverState)
              event = EVENT_RISING;
          else
              event = EVENT_NONE;
          receiverState = true;
      }
      if (event != EVENT_NONE)
      {
          cur_time = micros();
          ticks = cur_time - prev_time;
          if (necState == 0)
          { // Expecting rising edge of leading pulse
              if (event == EVENT_RISING)
              {
                  necState = 1;
                  // digitalWrite(ledPin,HIGH);
              }
          }
          else if (necState == 1)
          { // Expecting falling edge of leading pulse
              if (event == EVENT_FALLING)
              {
                  if (ticks > 8900L)
                  {
                      necState = 2; // Check for leading pulse > 8.9msec
                  }
                  else
                  { // Stray short pulse found, reset NEC state
                      // digitalWrite(ledPin,LOW);
                      necState = 0;
                  }
              }
          }
          else if (necState == 2)
          { // Expecting rising edge of first pulse after leading pulse
              if (event == EVENT_RISING)
              {
                  if (ticks > 3375L)
                  { // Check for space after leading pulse > 3.375 msec
                      IRCharBitMask = 0x80;
                      IRChar = 0;
                      necState = 3;
                  }
                  else
                  { // Space too short, reset NEC state to wait for another leading pulse
                      // digitalWrite(ledPin,LOW);
                      necState = 0;
                  }
              }
          }
          else if (necState == 3)
          { // Expecting falling edge of data pulse
              if (event == EVENT_FALLING)
              {
                  if (ticks < 648)
                  {
                      necState = 4; // Check if data pulse width < 648 usec
                  }
                  else
                  { // Width too short, reset NEC state to wait for another leading pulse
                      // digitalWrite(ledPin,LOW);
                      necState = 0;
                  }
              }
          }
          else if (necState == 4)
          { // Expecting rising edge of pulse after data pulse
              if (event == EVENT_RISING)
              {
                  if (ticks > 1120)
                  { // Record a '1' bit for space > 1120 usec
                      IRChar = IRChar | IRCharBitMask;
                  }
                  IRCharBitMask = IRCharBitMask >> 1;

                  if (IRCharBitMask == 0)
                  {                               // Check if eighth bit received and character complete (!!!!!)
                      buffer[num_chars] = IRChar; // Record complete character received in circular output buffer
                      num_chars++;
                      // buffer_in = (buffer_in + 1) & 0x07;
                      // digitalWrite(ledPin,LOW);
                      necState = 0; // Reset NEC state to wait for another leading pulse
                  }
                  else
                  {
                      necState = 3; // Wait for falling edge of data pulse
                  }
              }
          }
          prev_time = cur_time;
      }
      IRCounter++;
  }
  // Serial.println(output);
  return num_chars;
}

unsigned char* cRobot::getIR()
{
    return buffer;
}
