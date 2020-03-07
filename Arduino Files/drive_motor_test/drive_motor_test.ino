String textIn, textOut;

#define m1a 39  // motor 1 logic pin a
#define m1b 37  // motor 1 logic pin b
#define m1e 6   // motor 1 enable pin
#define m1_hall1 A10  // motor 1 sensor 1 pin
#define m1_hall2 A11  // motor 1 sensor 2 pin
int m1[3] = {m1a, m1b, m1e};  // holds pins for set functions

#define m2a 36  // motor 2 logic pin a
#define m2b 38  // motor 2 logic pin b
#define m2e 3   // motor 2 enable pin
#define m2_hall1 A6  // motor 2 sensor 1 pin
#define m2_hall2 A7  // motor 2 sensor 2 pin
int m2[3] = {m2a, m2b, m2e};  // holds pins for set functions

#define m3a 27  // motor 3 logic pin a
#define m3b 29  // motor 3 logic pin b
#define m3e 5   // motor 3 enable pin
#define m3_hall1 A8  // motor 3 sensor 1 pin
#define m3_hall2 A9  // motor 3 sensor 2 pin
int m3[3] = {m3a, m3b, m3e};  // holds pins for set functions

#define m4a 28  // motor 4 logic pin a
#define m4b 26  // motor 4 logic pin b
#define m4e 4   // motor 4 enable pin
#define m4_hall1 A4  // motor 4 sensor 1 pin
#define m4_hall2 A5  // motor 4 sensor 2 pin
int m4[3] = {m4a, m4b, m4e};  // holds pins for set functions



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
  digitalWrite(motor[2], HIGH);
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


  Serial.begin(9600);
  Serial1.begin(38400);
  Serial.setTimeout(10);
  Serial1.setTimeout(10);
  Serial.println("Begining Program...");
  Serial.println("");
}

void loop()
{
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

    if (textIn == "testCCW\n")
    {
      Serial.println("Testing CCW");
      Serial1.println("Testing CCW");
      stopAllMotors();
      setAllCW();
      startAllMotors();
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "testCW\n")
    {
      Serial.println("Testing CW");
      Serial1.println("Testing CW");
      stopAllMotors();
      setAllCCW();
      startAllMotors();
      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "forward\n")
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
    else if (textIn == "moveGBEUp\n")
    {
      Serial.println("moveGBEUp");
      Serial1.println("moveGBEUp");

      Serial.println("Proceed");
      Serial1.println("Proceed");
    }
    else if (textIn == "moveGBEDown\n")
    {
      Serial.println("moveGBEDown");
      Serial1.println("moveGBEDown");

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
    else if (textIn == "push\n")
    {
      Serial.println("Testing push");
      Serial1.println("Testing push");

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
  }
}
