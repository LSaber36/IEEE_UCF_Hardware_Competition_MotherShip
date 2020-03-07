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

Servo servo;

String text;
int z = 0;  // height of the GBE
int dir = 0, sc = 1;
int stepsPerCenti = 250;
int stepsPerRev = 200;
int lowerStopVal = 0;
int upperStopVal = 0;

#define STEP_PIN 5
#define DIR_PIN 2
#define MS1 7
#define MS2 8
#define RST 9
#define SLEEP 10
#define ENABLE 11

#define SERVO_PIN 3

#define UPPER_STOP A0
#define LOWER_STOP A1


void setup() {
  servo.attach(SERVO_PIN);

  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);

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

  Serial.begin(9600);
  Serial.setTimeout(10);
  Serial.println("Program Starting...");

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(SLEEP, HIGH);
  digitalWrite(ENABLE, LOW);
  digitalWrite(RST, HIGH);

  servo.write(90);
  servo.write(75);

  digitalWrite(MS1, LOW);  // whole steps
  digitalWrite(MS2, LOW);
}

void loop()
{
  upperStopVal = digitalRead(UPPER_STOP);
  lowerStopVal = digitalRead(LOWER_STOP);

  if (Serial.available())
  {
    text = Serial.readString();

    Serial.print(text);
    Serial.println(" ");

    if (text == "enable")
    {
      enableMotor();
    }
    else if (text == "disable")
    {
      disableMotor();
    }
    else if (text == "up")
    {
      Serial.println("Direction set to up");
      //digitalWrite(DIR_PIN, HIGH); //set direction cw
      dir = 1;
      setDir(1);
    }
    else if (text == "down")
    {
      Serial.println("Direction set to down");
      //digitalWrite(DIR_PIN, LOW); //set direction ccw
      dir = 0;
      setDir(0);
    }
    else if (text == "step")
    {
      Serial.println("rotating motor 1 rev");

      moveGEBSteps(dir, stepsPerRev, 1);
    }
    else if (text == "whole")
    {
      Serial.println("step set to whole");
      sc = 1;
      setScale(1);
    }
    else if (text == "half")
    {
      Serial.println("step set to half");
      sc = 2;
      setScale(2);
    }
    else if (text == "quarter")
    {
      Serial.println("step set to quarter");
      sc = 4;
      setScale(4);
    }
    else if (text == "eighth")
    {
      Serial.println("step set to eighth");
      sc = 8;
      setScale(8);
    }
    else if (text == "push")
    {
      pushServo();
    }
    else if (text == "pushSlow")
    {
      pushServoSlow();
    }
    else if (text == "home")
    {
      homeGBE(1, &z);
    }
    else if (text == "fun")
    {
      moveGEBDist(dir, 2, sc);
    }
    else if (text == "in")
    {
      pushServoIn();
    }
    else if (text == "out")
    {
      pushServoOut();
    }
    else if (text == "eject1")
    {
      eject1();
    }
    else if (text == "eject2")
    {
      eject2();
    }
  }
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
