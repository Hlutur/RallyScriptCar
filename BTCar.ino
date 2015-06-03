#include <Servo.h>

/**
 * Motor shield (L298N) 
 */
const int ENA = 5; // Port 5: Enable A
const int ENB = 6; // Port 6: Enable B
// IN1: 5v; IN2: GND -> Motor A Direction 1
// IN1: GND; IN2: 5v -> Motor A Direction 2
const int IN1 = 2; 
const int IN2 = 3;
// IN3: 5v; IN4: GND -> Motor B Direction 1
// IN3: GND; IN4: 5V -> Motor B Direction 2
const int IN3 = 4;
const int IN4 = 7;

#define CARID "Rabbit"

/** 
 * Ultrasonic Sensor
 */
const int TrigPin = 8;// 8;
const int EchoPin = 9;// 9;
const int Head = 11;
// Position for Servo
const int Central = 90;
const int LeftPos = 160;
const int RightPos = 20;
// PWM
const int PWM255 = 255;
const int PWM128 = 128;
// Constants
const int D = 1;
const int timeDelay = 30;
/**
 * Global Variables
 */
char incomingByte;
Servo myHead;
// safe distance 20 cm
const int SafeDistance = 20;
const int SafeSideDistance = 10;
boolean stopFlag = false;

void setup() 
{
  // Motor
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  // Ultrasonic Sensor
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  myHead.attach(Head);
  // Serial
  Serial.begin(9600);
  Serial.print(CARID);
  Serial.println(" ready to race!");
}

boolean fastForward = false;

void loop()
{
    if (fastForward){
        if(!checkSafe()) {
          Brake();
          fastForward = false;
          stopFlag = true;
        }
    }
  if(Serial.available() > 0)
  {
    //Bluetooth Command
    incomingByte = Serial.read();
    Serial.println(incomingByte);
    switch(incomingByte)
    {
    case 'f':
      if(!stopFlag) {
        Forward();
        fastForward = true;
        if(!checkSafe()) {
          Brake();
          fastForward = false;
          stopFlag = true;
        }
      }
      break;
    case 'b':
      fastForward = false;
      Backward();
      if(checkSafe())
        stopFlag = false;
      break;
    case 'l':
      fastForward = false;
      if(!stopFlag) {
        Left(PWM255);
        if(!checkSafe()) {
          Brake();
          stopFlag = true;
        }
      }
      else
      {
        Left(PWM128);
        if(!leftSafe()) {
          Brake();
          stopFlag = true;
        }
      }
      break;
    case 'r':
     fastForward = false;
     if(!stopFlag) {
        Right(PWM255);
        if(!checkSafe()) {
          Brake();
          stopFlag = true;
        }
      }
      else
      {
        Right(PWM128);
        if(!rightSafe()) {
          Brake();
          stopFlag = true;
        }
      }
      break;
    case 's':
      fastForward = false;
      Brake();
      break;
    default:
      break;  
    } //Switch
    if (!fastForward){
      StopMove();
    }
  }
}

/** 
 * Forward
 */
void Forward()
{
  analogWrite(ENA, PWM255);
  analogWrite(ENB, PWM255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.print("car/");
  Serial.print(CARID);
  Serial.println("/move|f|direction");
}

/** 
 * Backward
 */
void Backward()
{
  analogWrite(ENA, PWM255);
  analogWrite(ENB, PWM255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.print("car/");
  Serial.print(CARID);
  Serial.println("/move|b|direction");
}

/** 
 * Left
 */
void Right(int pwm)
{
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.print("car/");
  Serial.print(CARID);
  Serial.println("/move|r|direction");
}

/**
 * Right
 */
void Left(int pwm)
{
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.print("car/");
  Serial.print(CARID);
  Serial.println("/move|l|direction");
}

/** 
 * Break
 */
void Brake()
{
  analogWrite(ENA, PWM255);
  analogWrite(ENB, PWM255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  Serial.print("car/");
  Serial.print(CARID);
  Serial.println("/move|stop|direction");
  delay(timeDelay);
}

/**
 * Speed of Sound: 343 m/s
 */
int _last = 1000;
int getDistance() 
{
  long time, distance, result;
  // Low (5us) // 2us
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  // High (15us) // 10us
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  // Low
  digitalWrite(TrigPin, LOW);
  time = pulseIn(EchoPin, HIGH);
  result = round(distance = (time / 2) / 29.1);
  if (result > 1000) {
    result = _last;
  } else {
    _last = result;
  }
  Serial.print("car/");
  Serial.print(CARID);
  Serial.print("/dist|");
  Serial.print(result);
  Serial.println("|cm");
  return result;
}

/**
 * Get Distance (Head Position)
 */
int getDistanceAtPosition(int position) 
{
  myHead.write(position);
  delay(timeDelay);
  return getDistance();
}

/** 
 * Check Safety
 * @Return true: safe; false: danger (brake)
 */
boolean checkSafe()
{
  int distance = getDistanceAtPosition(Central);
 // Serial.print("checksafe ");
 // Serial.println(distance);
  boolean flag = true;
  if (distance < SafeDistance)
    flag = false;
  return flag;
}

boolean leftSafe()
{
  boolean flag = true;
  int dCenter = getDistanceAtPosition(Central);
  int dLeft = getDistanceAtPosition(LeftPos);
  if (dCenter < SafeDistance)
  {
    if (dCenter < SafeSideDistance)
      flag = false;
    else
    {
      if (dLeft < SafeDistance)
        flag = false;
      else
        flag = true;
    }
  }
  else
    stopFlag = false; 
  return flag;
}

boolean rightSafe()
{
  boolean flag = true;
  int dCenter = getDistanceAtPosition(Central);
  int dRight = getDistanceAtPosition(RightPos);
  if (dCenter < SafeDistance)
  {
    if (dCenter < SafeSideDistance)
      flag = false;
    else
    {
      if (dRight < SafeDistance)
        flag = false;
      else
        flag = true;  
    }
  }
  else
    stopFlag = false; 
  return flag;
}


void StopMove()
{
  analogWrite(ENA, PWM255);
  analogWrite(ENB, PWM255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

