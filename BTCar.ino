// RallyScript

#include <Arduino.h>
#include <Wire.h>
#include <HMC5883L_Simple.h>
#include <SPI.h>  
#include <Pixy.h>

// Create a compass
HMC5883L_Simple Compass;

// Create Pixy camera
Pixy pixy;

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

// safe distance 20 cm
const int SafeDistance = 15;
const int SafeSideDistance = 10;
boolean stopFlag = false;

boolean rFindCC = false;
boolean lFindCC = false;
float findDir = 0.0;
String findThisCC = String("");
boolean _ccInRange = false;

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
  
  // Compass
  Wire.begin();
  // The declination for your area can be obtained from http://www.magnetic-declination.com/
  //   Bergen      ,  0° 28' EAST
  Compass.SetDeclination(0, 28, 'E');  
  Compass.SetSamplingMode(COMPASS_SINGLE);
  // Sensitivity: Options are 088, 130 (default), 190, 250, 400, 470, 560, 810
  Compass.SetScale(COMPASS_SCALE_810);
  
  // Pixy-cam
  pixy.init();
  
  // Serial
  Serial.begin(9600);
  Serial.print(CARID);
  Serial.println(" ready to race!");
}

boolean fastForward = false;

void loop()
{
  if (fastForward){
      getHeading();
      if(!checkSafe()) {
        Brake();
        fastForward = false;
        stopFlag = true;
      }
  }
  if (lFindCC || rFindCC){
    findCC();
  }
    
  if(Serial.available() > 0)
  {
    //Bluetooth Command
    incomingByte = Serial.read();
    Serial.println(incomingByte);
    switch(incomingByte)
    {
    case 'f':
      //if(!stopFlag) {
      {
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
    case '>': // find CC, turn right
      rFindCC = true;
      lFindCC = false;
      initFindCC();
      break;
    case '<': // find CC, turn left
      rFindCC = false;
      lFindCC = true;
      initFindCC();
    case 'h': // auto home on CC. no parameters clears home mode
      initHome();
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
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
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
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.print("car/");
  Serial.print(CARID);
  Serial.println("/move|b|direction");
  getHeading();
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
  getHeading();
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
  getHeading();
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

float getHeading()
{
  float heading = Compass.GetHeadingDegrees();
  delayMicroseconds(5);
  heading += Compass.GetHeadingDegrees();
  delayMicroseconds(5);
  heading += Compass.GetHeadingDegrees();
  heading = heading / 3;
  Serial.print("car/");
  Serial.print(CARID);
  Serial.print("/heading|");
  Serial.print(heading);
  Serial.println("|deg");
  getPixyBlocks();
  return heading;
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
 * Check Safety
 * @Return true: safe; false: danger (brake)
 */
boolean checkSafe()
{
  int distance = getDistance();
 // Serial.print("checksafe ");
 // Serial.println(distance);
  boolean flag = true;
  if (distance < SafeDistance)
    flag = false;
  return flag;
}

boolean leftSafe()
{
  return true; // Always safe without the servo
}

boolean rightSafe()
{
  return true; // Always safe without the servo
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

void getPixyBlocks()
{
  int j;
  uint16_t blocks;
  char buf[32]; 
  
  _ccInRange = false;
  
  // grab blocks!
  blocks = pixy.getBlocks();
  
  // If there are detect blocks, print them!
  if (blocks)
  {
    int j;
    for (j=0;j<blocks;j++)
    {
      int x = pixy.blocks[j].x;

      Serial.print("car/");
      Serial.print(CARID);
      Serial.print("/cc/");
      Serial.print(pixy.blocks[j].signature,OCT);
      Serial.print("/{x:");
      Serial.print(x);
      Serial.print(",w:");
      Serial.print(pixy.blocks[j].width);
      Serial.println("}");
      
      String block = String(pixy.blocks[j].signature,OCT);
      if (block.equals(findThisCC)){
        if ((x>89)&&(x<111)){
          _ccInRange = true;
        }
      }      
    }
  }    
}

boolean CCinRange()
{
  return _ccInRange;
  /*
    int j;
  uint16_t blocks;
  char buf[32]; 
  
  // grab blocks!
  blocks = pixy.getBlocks();
  
  // If there are detect blocks, print them!
  if (blocks)
  {
    int j;
    for (j=0;j<blocks;j++)
    {
      int x = pixy.blocks[j].x;
      if (block.equals(findThisCC)){
        if ((x>89)&&(x<111)){
          return true;
        } else {
          // continue search
        }
      }
    }
  }
  return false; 
 */ 
}

static int turns = 0;
static boolean check_for_turn = false;
void initFindCC()
{
  // 1. Read initial bearing
  findDir = getHeading();
  turns = 0;
  check_for_turn = false;
  // 2. Parse string representing CC to find
  int cc = Serial.parseInt();
  if (cc == 0){
    // invalid
    rFindCC = false;
    lFindCC = false;
    Brake();
    return;
  }
  findThisCC = String(cc);
  Serial.print("Leter etter ");Serial.println(findThisCC);
}

void findCC()
{
  // 1. Check if correct CC is within tolerance
  float currentDir = getHeading();
  if (rFindCC && (!lFindCC)){
    Right(PWM255);
  } else if (lFindCC && (!rFindCC)){
    Left(PWM255);
  } else {
    Brake;
  }
  delay(100); // rotate for 0.1 sek
  turns++;
  if (CCinRange()){
    // stop turn - found item
    rFindCC = false;
    lFindCC = false;
    Brake();
    Serial.print("car/");
    Serial.print(CARID);
    Serial.print("/cc/");
    Serial.print(findThisCC);
    Serial.println("/InRange|true");
  }
  if (turns > 5){
    if ((currentDir > findDir)&&(currentDir < (findDir+10)))
    {
      rFindCC = false;
      lFindCC = false;
      Brake();
      Serial.print("car/");
      Serial.print(CARID);
      Serial.print("/cc/");
      Serial.print(findThisCC);
      Serial.println("/InRange|false");
    }
  }
}

void initHome()
{
  int cc = Serial.parseInt();
  if (cc == 0){
    _home = false;
    return;
  }
  _home = true;
  findThisCC = String(cc);
}

