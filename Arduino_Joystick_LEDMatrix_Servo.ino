/*
  This file is part of the Arduino_Joystick_LEDMatrix_Servo library.
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <LedControl.h>
#include <Servo.h>
#include <math.h>

//Compiler variables
#define USE_SERIAL false //Controls if serial port will output messages for debugging

// Arduino pin numbers
const int SW_pin = 2; // digital pin connected to joystick switch output
const int X_pin = 0; // analog pin connected to X output
const int Y_pin = 1; // analog pin connected to Y output
const int SERVO_pin = 9; // digital pin connected to servo
const int LED_Ctrl_CLK = 10; // digital pin for LED matrix CLK
const int LED_Ctrl_LOAD_CS = 11; //  digital pin for LED matrix LOAD(CS)
const int LED_Ctrl_DataIn = 12; //  digital pin for LED matrix DataIn
const int LED_Ctrl_numDevices = 1; // LED matrix devices quantity

//Program constant values (do not change)
const int LED_MATRIX_STEP = 128; //The value each LED in the matrix represents (1024/8=128)
const double RAD_TO_DEG_FACTOR = 180.0 * M_1_PI; //Factor to convert from Rad to Deg

//Program parameters
const int POSITION_THRESHOLD = 10; //Minimum position difference to take in account
#if USE_SERIAL
//Delay values for USE_SERIAL = true
const int DELAY_MAIN_LOOP = 3500; //Delay in milliseconds for main loop
const int DELAY_SETUP = 500; //Delay in milliseconds at the end of setup
#else
//Delay values for USE_SERIAL = false
const int DELAY_MAIN_LOOP = 250; //Delay in milliseconds for main loop
const int DELAY_SETUP = 50; //Delay in milliseconds at the end of setup
#endif

// X and Y original values
int *xOrig, *yOrig;
// X, Y and switch current values in joystick
int *xValue, *yValue, *swValue;
// Instance to control servo
Servo joystickServo;

// Instance to control LED matrix
LedControl ledCtrl = LedControl(LED_Ctrl_DataIn, LED_Ctrl_CLK, LED_Ctrl_LOAD_CS, LED_Ctrl_numDevices);


void initSerialPort() {
#if USE_SERIAL
  // Initialize serial port
  Serial.begin(9600);
#endif
}

void initJoystick(int **xOrig, int **yOrig, int **xValue, int **yValue, int **swValue) {
  //Initialize pointers to the x and y axis joystick positions
  *xOrig = (int*)malloc(sizeof xOrig);
  *yOrig = (int*)malloc(sizeof yOrig);
  *xValue = (int*)malloc(sizeof xValue);
  *yValue = (int*)malloc(sizeof yValue);
  *swValue = (int*)malloc(sizeof swValue);
  //Initialize joystick pins
  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);
  // Store the initial values of x and y axis in joystick rest position
  **xValue = **xOrig = analogRead(X_pin);
  **yValue = **yOrig = analogRead(Y_pin);

#if USE_SERIAL
  Serial.print("X-orig:");
  Serial.println(**xOrig);
  Serial.print("Y-orig:");
  Serial.println(**yOrig);
#endif
}


void initServo(Servo* joystickServo) {
  //Initialize servo values
  joystickServo->attach(SERVO_pin);
  joystickServo->write(0);
}

void initLEDControl(LedControl* ledCtrl) {
  //Initialize LED matrix, set brightness and clear all LEDs
  ledCtrl->shutdown(0, false);
  ledCtrl->setIntensity(0, 4);
  ledCtrl->clearDisplay(0);
}



void setLEDMatrixValues(int* xValue, int* yValue, LedControl* ledCtrl) {
  //Index (zero-based coordinate) of the LED to set in the matrix
  int xLEDIdx, yLEDIdx;
  xLEDIdx = *xValue / LED_MATRIX_STEP;
  yLEDIdx = *yValue / LED_MATRIX_STEP;

  //Clear all LEDs in the matrix
  ledCtrl->clearDisplay(0);
  //Switch on the led in the (xLEDIdx, yLEDIdx) coordinate of the LED matrix at addr=0
  ledCtrl->setLed(0, yLEDIdx, xLEDIdx, true);

#if USE_SERIAL
  Serial.print("X-LED: ");
  Serial.println(xLEDIdx);
  Serial.print("Y-LED: ");
  Serial.println(yLEDIdx);
#endif

}

void getJoystickValues(int* xValue, int* yValue, int* swValue) {
  *xValue = analogRead(X_pin);
  *yValue = analogRead(Y_pin);
  *swValue = digitalRead(SW_pin);
#if USE_SERIAL
  Serial.print("Switch:  ");
  Serial.println(*swValue);
  Serial.print("X-axis: ");
  Serial.println(*xValue);
  Serial.print("Y-axis: ");
  Serial.println(*yValue);
#endif
}

void setServoAngle(int joystickAngle, Servo* joystickServo) {
  int servoAngle;
  servoAngle = abs(joystickAngle);
  joystickServo->write(servoAngle);
}

int getJoystickAngle(int* xValue, int* yValue, int* xOrig, int* yOrig) {
  //Difference in joystick's position in x and y axis.
  int xDelta, yDelta;
  //The joystick angle in degrees
  int angDegs;
  //The joystick angle in radians
  double angRads;

  // In case original position, return angle zero
  if (*xValue == *xOrig && *yValue == *yOrig) {
    return 0;
  }

  //Get the x and y axis position difference
  xDelta = *xValue - *xOrig;
  yDelta = *yOrig - *yValue;

#if USE_SERIAL
  Serial.print("X-delta: ");
  Serial.println(xDelta);
  Serial.print("Y-delta: ");
  Serial.println(yDelta);
#endif

  //If the position difference is below threshold, return to center
  if ((abs(xDelta) + abs(yDelta)) < POSITION_THRESHOLD) return 0;
  //Use arctan function to calculate angle in radians
  angRads = atan2((double)yDelta, (double)xDelta);

  //Convert angle from radians to degrees and return value
  angDegs = radToDeg(angRads);
#if USE_SERIAL
  Serial.print("Joystick Angle: ");
  Serial.println(angDegs);
#endif
  return angDegs;

}

// Convert radian value to integer degrees
int radToDeg(double angRads) {
  return (int)floor(angRads * RAD_TO_DEG_FACTOR);
}

void setup() {
  initSerialPort();
  initJoystick(&xOrig, &yOrig,  &xValue, &yValue, &swValue);
  initServo(&joystickServo);
  initLEDControl(&ledCtrl);
  delay(DELAY_SETUP);
}

void loop() {
  // The integer value of the joystick in degrees
  int joystickAngle;
  getJoystickValues(xValue, yValue, swValue);
  joystickAngle = getJoystickAngle(xValue, yValue, xOrig, yOrig);
  setServoAngle(joystickAngle, &joystickServo);
  setLEDMatrixValues(xValue, yValue, &ledCtrl);
  delay(DELAY_MAIN_LOOP);
}
