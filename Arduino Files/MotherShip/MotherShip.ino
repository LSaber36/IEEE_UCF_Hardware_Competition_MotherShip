/*
   The  purpose of this file is to make controlling the GBE as easy as possible
   Functions for controlling the motor:

   void pulseMotor(int dir)
   void moveGEBDist(int dir, float dist, float scale)
   void moveGEBSteps(int dir, float steps, float scale)
   void homeGBE(float scale, int *z)
   void setDir(int dir)
   void setScale(int scale)
   void enableMotor(void)
   void disableMotor(void)
   void pushServo(void)
   void pushServoSlow(void)

   All of these functions can be rewritten or appended to make their use easier
*/

#include <Servo.h>

String textIn, textOut;

#define m1a 33  // motor 1 logic pin a
#define m1b 31  // motor 1 logic pin b
#define m1e 6   // motor 1 enable pin
#define m1_hall1 A10  // motor 1 sensor 1 pin
#define m1_hall2 A11  // motor 1 sensor 2 pin
int m1[3] = {m1a, m1b, m1e};  // holds pins for set functions

#define m2a 30  // motor 2 logic pin a
#define m2b 32  // motor 2 logic pin b
#define m2e 3   // motor 2 enable pin
#define m2_hall1 A6  // motor 2 sensor 1 pin
#define m2_hall2 A7  // motor 2 sensor 2 pin
int m2[3] = {m2a, m2b, m2e};  // holds pins for set functions

#define m3a 25  // motor 3 logic pin a
#define m3b 27  // motor 3 logic pin b
#define m3e 5   // motor 3 enable pin
#define m3_hall1 A8  // motor 3 sensor 1 pin
#define m3_hall2 A9  // motor 3 sensor 2 pin
int m3[3] = {m3a, m3b, m3e};  // holds pins for set functions

#define m4a 26  // motor 4 logic pin a
#define m4b 24  // motor 4 logic pin b
#define m4e 4   // motor 4 enable pin
#define m4_hall1 A4  // motor 4 sensor 1 pin
#define m4_hall2 A5  // motor 4 sensor 2 pin
int m4[3] = {m4a, m4b, m4e};  // holds pins for set functions

// lazy susan bracket
#define m5a 36  // motor 4 logic pin a
#define m5b 38  // motor 4 logic pin b
#define m5e 9   // motor 4 enable pin
#define m5_hall1 44  // motor 4 sensor 1 pin
#define m5_hall2 46  // motor 4 sensor 2 pin
int m5[3] = {m5a, m5b, m5e};  // holds pins for set functions


//line sensors
#define LINE1 A12
#define LINE2 A13
#define LINE3 A14
#define LINE4 A15
int line1 = 0;
int line2 = 0;
int line3 = 0;
int line4 = 0;


int motorSpeed = 255;


Servo servo;

String text;
int z = 0;  // height of the GBE
int dir = 0, sc = 1;
int stepsPerCenti = 250;
int stepsPerRev = 200;
int lowerStopVal = 0;
int upperStopVal = 0;

#define STEP_PIN 8
#define DIR_PIN 50
#define MS1 43
#define MS2 45
#define RST 49
#define SLEEP 41
#define ENABLE 47

#define SERVO_PIN 7

#define UPPER_STOP A0
#define LOWER_STOP A1


void setCW(int *motor)
{
  digitalWrite(motor[0], LOW);
  digitalWrite(motor[1], HIGH);
  Serial.println("motor set CW");
}

void setCCW(int *motor)
{
  digitalWrite(motor[0], HIGH);
  digitalWrite(motor[1], LOW);
  Serial.println("motor set CCW");
}

void setAllCW()
{
  setCW(m1);
  setCW(m2);
  setCW(m3);
  setCW(m4);
}

void setAllCCW()
{
  setCCW(m1);
  setCCW(m2);
  setCCW(m3);
  setCCW(m4);
}

void startAllMotors()
{
  startMotor(m1);
  startMotor(m2);
  startMotor(m3);
  startMotor(m4);
}

void startMotor(int *motor)
{
  analogWrite(motor[2], motorSpeed);
}

void startAllMotorsDelay()
{
  startMotor(m1);
  delay(1000);
  startMotor(m2);
  delay(1000);
  startMotor(m3);
  delay(1000);
  startMotor(m4);
}

void stopAllMotors()
{
  stopMotor(m1);
  stopMotor(m2);
  stopMotor(m3);
  stopMotor(m4);
}

void stopMotor(int *motor)
{
  digitalWrite(motor[2], LOW);
}

void moveForward()
{
  setCW(m1);
  setCW(m2);
  setCCW(m3);
  setCCW(m4);
  startAllMotors();
}

void moveBackward()
{
  setCCW(m1);
  setCCW(m2);
  setCW(m3);
  setCW(m4);
  startAllMotors();
}

void moveLeft()
{
  setCCW(m1);
  setCW(m2);
  setCCW(m3);
  setCW(m4);
  startAllMotors();
}

void moveRight()
{
  setCW(m1);
  setCCW(m2);
  setCW(m3);
  setCCW(m4);
  startAllMotors();
}

void rotateCW()
{
  setAllCW();
  startAllMotors();
}

void rotateCCW()
{
  setAllCCW();
  startAllMotors();
}



void pulseMotor(int dir)
{
  digitalWrite(STEP_PIN, HIGH);
  delay(1);
  digitalWrite(STEP_PIN, LOW);
  delay(1);
}

void moveGEBDist(int dir, float dist, float scale)
{
  // centi: 1 step(1), 1/2 step(2), 1/4 step(4), 1/8 step(8)
  int i = 0;

  //how many steps to 1 centimeter
  float stepsPerCenti = 250; // 250 steps/cm
  float numSteps = dist * stepsPerCenti * scale;

  enableMotor();
  digitalWrite(STEP_PIN, LOW);

  setDir(dir);
  setScale(scale);

  for (i = 0; i < numSteps; i++)
  {
    upperStopVal = digitalRead(UPPER_STOP);
    lowerStopVal = digitalRead(LOWER_STOP);

    if (upperStopVal == 1  ||  lowerStopVal == 1)
    {
      return;
    }

    pulseMotor(dir);
  }
}

void moveGEBSteps(int dir, float steps, float scale)
{
  int i;

  enableMotor();

  for (i = 0; i < steps; i++)
  {
    if (UPPER_STOP == 1  ||  LOWER_STOP == 1)
    {
      return;
    }

    pulseMotor(dir);
  }
}

void homeGBE(float scale, int *z)
{
  enableMotor();
  Serial.println("Homing GBE");

  while (lowerStopVal == 0)  // LOWER_STOP is not pressed
  {
    lowerStopVal = digitalRead(LOWER_STOP);
    pulseMotor(0);
  }
  *z = 0;  // dereference z and set it equal to 0

  Serial.println("Done");
}

void setDir(int dir)
{
  if (dir == 0) // down
  {
    digitalWrite(DIR_PIN, LOW); //set direction ccw
  }
  else if (dir == 1)  // up
  {
    digitalWrite(DIR_PIN, HIGH); //set direction cw
  }
}

void setScale(int scale)
{
  if (scale == 1)  // whole steps
  {
    digitalWrite(MS1, LOW);
    digitalWrite(MS2, LOW);
  }
  else if (scale == 2)  // 1/2 steps
  {
    digitalWrite(MS1, HIGH);
    digitalWrite(MS2, LOW);
  }
  else if (scale == 4)  // 1/4 steps
  {
    digitalWrite(MS1, LOW);
    digitalWrite(MS2, HIGH);
  }
  else if (scale == 8)  // 1/8 steps
  {
    digitalWrite(MS1, HIGH);
    digitalWrite(MS2, HIGH);
  }
}

void enableMotor(void)
{
  Serial.println("Motor Enabled");
  digitalWrite(RST, HIGH);
}

void disableMotor(void)
{
  Serial.println("Motor Disabled");
  digitalWrite(RST, LOW);
}

void pushServo(void)
{
  servo.write(120);
  delay(1000);
  servo.write(75);
}

void pushServoSlow(void)
{
  int i;
  int delayTime = 10;

  for (i = 75; i < 120; i++)
  {
    servo.write(i);
    delay(delayTime);
  }
  delay(1000);

  for (i = 120; i > 75; i--)
  {
    servo.write(i);
    delay(delayTime);
  }
}

void eject1(void)
{
  Serial.println("Ejecting block");
  moveGEBDist(1, .85, 2);
  pushServoSlow();
}

void eject2(void)
{
  Serial.println("Ejecting block");

}

void pushServoOut(void)
{
  servo.write(120);
}

void pushServoIn(void)
{
  servo.write(75);
}



void setup()
{
  pinMode(m1a, OUTPUT);
  pinMode(m1b, OUTPUT);
  pinMode(m1e, OUTPUT);
  pinMode(m1_hall1, INPUT);

  pinMode(m2a, OUTPUT);
  pinMode(m2b, OUTPUT);
  pinMode(m2e, OUTPUT);
  pinMode(m2_hall1, INPUT);

  pinMode(m3a, OUTPUT);
  pinMode(m3b, OUTPUT);
  pinMode(m3e, OUTPUT);
  pinMode(m3_hall1, INPUT);

  pinMode(m4a, OUTPUT);
  pinMode(m4b, OUTPUT);
  pinMode(m4e, OUTPUT);
  pinMode(m4_hall1, INPUT);


  pinMode(SERVO_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);


  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(RST, OUTPUT);
  pinMode(SLEEP, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  pinMode(UPPER_STOP, INPUT);
  pinMode(LOWER_STOP, INPUT);

  pinMode(LINE1, INPUT);
  pinMode(LINE2, INPUT);
  pinMode(LINE3, INPUT);
  pinMode(LINE4, INPUT);


  Serial.begin(9600);
  Serial1.begin(38400);
  Serial.setTimeout(10);
  Serial1.setTimeout(10);
  Serial.println("Begining Program...");
  Serial.println("");

  servo.attach(SERVO_PIN);


  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(SLEEP, HIGH);
  digitalWrite(ENABLE, LOW);
  digitalWrite(RST, HIGH);

  //servo.write(90);
  servo.write(50);

  digitalWrite(MS1, LOW);  // whole steps
  digitalWrite(MS2, LOW);


  moveForward();
  delay(500);
  stopAllMotors();
}

void loop()
{
  upperStopVal = digitalRead(UPPER_STOP);
  lowerStopVal = digitalRead(LOWER_STOP);

  line1 = digitalRead(LINE1);
  line2 = digitalRead(LINE2);
  line3 = digitalRead(LINE3);
  line4 = digitalRead(LINE4);


  // computer serial
  if (Serial.available())
  {
    textOut = Serial.readString();
    // not auto new line sent in textOut
    //Serial.println(textOut);
    Serial1.println(textOut);
  }

  // bluetooth serial
  if (Serial1.available())
  {
    textIn = Serial1.readString();
    // auto new line sent in textIn
    Serial.print(textIn);
    //Serial1.print(textIn);

    // main motor commands
    if (textIn == "forward\n")
    {
      Serial.println("Testing forward");
      Serial1.println("Testing forward");
      moveForward();
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "backward\n")
    {
      Serial.println("Testing backward");
      Serial1.println("Testing backward");
      moveBackward();
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "left\n")
    {
      Serial.println("Testing left");
      Serial1.println("Testing left");
      moveLeft();
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "right\n")
    {
      Serial.println("Testing right");
      Serial1.println("Testing right");
      moveRight();
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "rotateCW\n")
    {
      Serial.println("Testing rotateCW");
      Serial1.println("Testing rotateCW");
      rotateCW();
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "rotateCCW\n")
    {
      Serial.println("Testing rotateCCW");
      Serial1.println("Testing rotateCCW");
      rotateCCW();
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "setDist\n")
    {
      Serial.println("Testing setDist");
      Serial1.println("Testing setDist");

      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "stopAllMotors\n")
    {
      Serial.println("Testing stopAllMotors");
      Serial1.println("Testing stopAllMotors");
      stopAllMotors();
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "CWdelay\n")
    {
      Serial.println("Testing CWdelay");
      Serial1.println("Testing CWdelay");
      stopAllMotors();
      setAllCW();
      startAllMotorsDelay();
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "CCWdelay\n")
    {
      Serial.println("Testing CCWdelay");
      Serial1.println("Testing CCWdelay");
      stopAllMotors();
      setAllCCW();
      startAllMotorsDelay();
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    // GBE commands
    else if (textIn == "whole\n")
    {
      Serial.println("step set to whole");
      Serial1.println("step set to whole");
      sc = 1;
      setScale(1);
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "half\n")
    {
      Serial.println("step set to half");
      Serial1.println("step set to half");
      sc = 2;
      setScale(2);
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "quarter\n")
    {
      Serial.println("step set to quarter");
      Serial1.println("step set to quarter");
      sc = 4;
      setScale(4);
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "eighth\n")
    {
      Serial.println("step set to eighth");
      Serial1.println("step set to eighth");
      sc = 8;
      setScale(8);
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "enable\n")
    {
      Serial.println("GBE enabled");
      Serial1.println("GBE enabled");
      enableMotor();
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "disable\n")
    {
      Serial.println("GBE disabled");
      Serial1.println("GBE disabled");
      disableMotor();
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "up\n")
    {
      Serial.println("direction set to up");
      Serial1.println("direction set to up");
      dir = 1;
      setDir(1);
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "down\n")
    {
      Serial.println("direction set to down");
      Serial1.println("direction set to down");
      dir = 0;
      setDir(0);
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "push\n")
    {
      Serial.println("pushing");
      Serial1.println("pushing");
      pushServo();
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "pushSlow\n")
    {
      Serial.println("pushing slow");
      Serial1.println("pushing slow");
      pushServoSlow();
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "moveDist\n")
    {
      Serial.println("moving set dist");
      Serial1.println("moving set dist");
      moveGEBDist(dir, 2, sc);
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "step\n")
    {
      Serial.println("direction set to down");
      Serial1.println("direction set to down");
      dir = 0;
      setDir(0);
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "fast\n")
    {
      motorSpeed = 255;
      Serial.print("motorSpeed: ");
      Serial.println(motorSpeed);
      Serial1.println("motorSpeed: ");
      Serial1.println(motorSpeed);

      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "slow\n")
    {
      motorSpeed = 125;
      Serial.print("motorSpeed: ");
      Serial.println(motorSpeed);
      Serial1.println("motorSpeed: ");
      Serial1.println(motorSpeed);
      motorSpeed = 125;
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "getSensorData\n")
    {
      Serial1.print(line1);
      Serial1.print(" : ");
      Serial1.print(line2);
      Serial1.print(" : ");
      Serial1.print(line3);
      Serial1.print(" : ");
      Serial1.println(line4);

      Serial.print(line1);
      Serial.print(" : ");
      Serial.print(line2);
      Serial.print(" : ");
      Serial.print(line3);
      Serial.print(" : ");
      Serial.println(line4);
    }
  }
}
