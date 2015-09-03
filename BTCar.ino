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
const int ENB = 10; // Port 6: Enable B
// IN1: 5v; IN2: GND -> Motor A Direction 1
// IN1: GND; IN2: 5v -> Motor A Direction 2
const int IN1 = 6;  // rød - dirb
const int IN2 = 7;  // ikke i bruk
// IN3: 5v; IN4: GND -> Motor B Direction 1
// IN3: GND; IN4: 5V -> Motor B Direction 2
const int IN3 = 8; // gul - dira
const int IN4 = 9; // ikke i bruk

#define CARID "Turtle"
#define PRETOPIC "fullstackfest/car/"

/** 
 * Ultrasonic Sensor
 */
const int TrigPin = 4;// 8;
const int EchoPin = 3;// 9;

// PWM
int PWM255 = 255;
int PWM128 = 128;
// Constants
const int D = 1;
const int timeDelay = 30;
/**
 * Global Variables
 */
char incomingByte;

// safe distance 15 cm
const int SafeDistance = 20;
const int SafeSideDistance = 10;
boolean stopFlag = false;

boolean rFindCC = false;
boolean lFindCC = false;
float findDir = 0.0;
String findThisCC = String("");
boolean _ccInRange = false;
int _ccCenter = 100;
boolean _home = false;
float currentDir;
float _forwardDir;

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
unsigned long _scanPixy = millis();
unsigned long _heartBeat = millis();

void loop()
{
  if ((_scanPixy+500)<millis()){
    _scanPixy = millis();
    getPixyBlocks();
  }
  if ((_heartBeat+5000)<millis()){
    _heartBeat = millis();
    doHeartBeat();
  }
  if (fastForward){
      getHeading();
      adjustDir();
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
      break;
    case 'h': // auto home on CC. no parameters clears home mode
      initHome();
      break;
    case 'v': // set velocity
      {
        int velocity = Serial.parseInt();
        if ((velocity >=0) && (velocity < 256)){
          PWM255 = velocity;
          PWM128 = velocity / 2;
          Brake();
        }
      }
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
  Serial.print(PRETOPIC);
  Serial.print(CARID);
  Serial.println("/move|{\"dir\":\"f\"}");
  getHeading();
  _forwardDir = currentDir;
}

void adjustDir()
{
  if ((currentDir - _forwardDir)>5.0){
    // Serial.print("L adjusting ");Serial.print(currentDir);Serial.print(" ");Serial.println(_forwardDir);
    analogWrite(ENA, 0);
    delay(300);
    analogWrite(ENA, PWM255);
  } else if ((currentDir - _forwardDir)<-5.0){
    // Serial.print("R adjusting ");Serial.print(currentDir);Serial.print(" ");Serial.println(_forwardDir);
    analogWrite(ENB, 0);
    delay(300);
    analogWrite(ENB, PWM255);
  }
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
  Serial.print(PRETOPIC);
  Serial.print(CARID);
  Serial.println("/move|{\"dir\":\"b\"}");
  getHeading();
  delay(100);
  getDistance();
}

void Right2(int pwm)
{
  float dir = Serial.parseInt();
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  getHeading();
  if (dir > 0){
    float newHead = currentDir - dir + 10;
    boolean checkDir = true;
    if (newHead < 0){
      newHead = newHead + 360.0;
      checkDir = false;
    }
    boolean turn = true;
    while (turn){
      getHeading();
      if (checkDir){
        turn = currentDir > newHead;
      } else if (currentDir > newHead){
        checkDir = true;
      }
    }
    Brake();
  } else {
    delay(100);
  }
  Serial.print(PRETOPIC);
  Serial.print(CARID);
  Serial.println("/move|{\"dir\":\"r\"}");
}

void Right(int pwm)
{
  float dir = Serial.parseInt();
  getHeading();
  float newHead = currentDir - dir;
  newHead = (newHead < 10) ? 10 : newHead;
  Serial.println("right");
  float origHead = currentDir;
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  boolean turn = true;
  if (dir > 0.0){
    dir = (dir < 0.0) ? 0.0 : ((dir > 180.0 ) ? 180.0 : dir);
    float lastHead = currentDir;
    int koeff = 0;
    {
      while (turn){
        delay(5);
        getHeading();
        if ((lastHead < currentDir)&&((currentDir-lastHead)>100)){
          koeff = koeff--;
        }
        float calcDir = currentDir - (360.0*koeff);
        // Serial.print(newHead);Serial.print(" ");Serial.print(newHead + dir);Serial.print(" ");Serial.print(calcDir);Serial.print(" ");Serial.print(lastHead);Serial.print(" ");
        // Serial.print(currentDir);Serial.print(" ");Serial.println(koeff);
        lastHead = currentDir;
        if (calcDir <= newHead){
          turn = false;
        }
      }
    }
  } else {
    delay(100);
  }
  Brake();
  Serial.print(PRETOPIC);
  Serial.print(CARID);
  Serial.println("/move|{\"dir\":\"r\"}");
  getHeading();
}

void Left(int pwm)
{
  float dir = Serial.parseInt();
  getHeading();
  float newHead = currentDir + dir;
  newHead = (newHead < 10) ? 10 : newHead;
  float origHead = currentDir;
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  boolean turn = true;
  if (dir > 0.0){
    dir = (dir < 0.0) ? 0.0 : ((dir > 180.0 ) ? 180.0 : dir);
    float lastHead = currentDir;
    int koeff = 0;
    {
      while (turn){
        delay(5);
        getHeading();
        if ((lastHead > currentDir)&&((lastHead-currentDir)>100)){
          koeff = koeff++;
        }
        lastHead = currentDir;
        float calcDir = currentDir + (360.0*koeff);
        if (calcDir >= newHead){
          turn = false;
        }
      }
    }
  } else {
    delay(100);
  }
  Brake();
  Serial.print(PRETOPIC);
  Serial.print(CARID);
  Serial.print(pwm);
  Serial.println("/move|{\"dir\":\"l\"}");
  getHeading();
}

void Left2(int pwm)
{
  float dir = Serial.parseInt();
  if (dir > 0.0){
    dir = (dir < 0.0) ? 0.0 : ((dir > 180.0 ) ? 180.0 : dir);
    getHeading();
    float newHead = currentDir + dir - 10;
    newHead = (newHead < 10) ? 10 : newHead;
    float origHead = currentDir;
    analogWrite(ENA, pwm);
    analogWrite(ENB, pwm);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    boolean turn = true;
    float lastHead = currentDir;
    if (newHead > 360.0){
      Left(360);
      int l = newHead-360;
      Left(l);
    } else
    {
      while (turn){
        delay(50);
        getHeading();
        turn = !(newHead < currentDir);
        if (lastHead > currentDir){
          turn > false;
        }
        if ((currentDir < 90.0) && (newHead > 300.0)){
          turn = false;
        }
        // Serial.print(origHead);Serial.print(" ");Serial.print(currentDir);Serial.print(" ");Serial.print(newHead);Serial.print(" ");Serial.println(lastHead);
        lastHead = currentDir;
        Serial.println(turn);
      }
    }
  } else {
    delay(100);
  }
  Brake();
  Serial.print(PRETOPIC);
  Serial.print(CARID);
  Serial.print(pwm);
  Serial.println("/move|{\"dir\":\"l\"}");
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
  Serial.print(PRETOPIC);
  Serial.print(CARID);
  Serial.println("/move|{\"dir\":\"stop\"}");
  delay(timeDelay);
}

float getHeading()
{
  float heading = Compass.GetHeadingDegrees();
  delayMicroseconds(2);
  heading += Compass.GetHeadingDegrees();
  delayMicroseconds(2);
  heading += Compass.GetHeadingDegrees();
  heading = heading / 3;
  Serial.print(PRETOPIC);
  Serial.print(CARID);
  Serial.print("/heading|{\"deg\":");
  Serial.print(heading);
  Serial.println("}");
  getPixyBlocks();
  currentDir = heading;
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
  result = round(distance = (time / 2) / 34.3); // speed of sound 343 m/s 20C
  if (result > 1000) {
    result = _last;
  } else {
    _last = result;
  }
  Serial.print(PRETOPIC);
  Serial.print(CARID);
  Serial.print("/dist|{\"cm\":");
  Serial.print(result);
  Serial.println("}");
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
  getDistance();
  return true; // Always safe without the servo
}

boolean rightSafe()
{
  getDistance();
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
  getDistance();
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
      String id = String(pixy.blocks[j].signature,OCT);
      if (id.length()==3){
        Serial.print(PRETOPIC);
        Serial.print(CARID);
        Serial.print("/cc|{\"id\":");
        Serial.print(id);
        Serial.print(",\"x\":");
        Serial.print(x);
        Serial.print(",\"w\":");
        Serial.print(pixy.blocks[j].width);
        Serial.println("}");
        
        String block = String(pixy.blocks[j].signature,OCT);
        if (block.equals(findThisCC)){
          if ((x>111)&&(x<131)){
            _ccInRange = true;
            _ccCenter = x;
          }
        }
      }      
    }
  }    
}

boolean CCinRange()
{
  return _ccInRange;
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
  // Serial.print("Leter etter ");Serial.println(findThisCC);
}

void findCC()
{
  // 1. Check if correct CC is within tolerance
  boolean giveUp = false;
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
    Serial.print(PRETOPIC);
    Serial.print(CARID);
    Serial.print("/cc|{\"id\":");
    Serial.print(findThisCC);
    Serial.println(",\"inRange\":\"true\"}");
  }
  if (rFindCC){
    if (currentDir < findDir){
      check_for_turn = true;
    }
    if (currentDir > findDir){
      giveUp = true;
    }
  }
  if (lFindCC){
    if (currentDir > findDir){
      check_for_turn = true;
    }
    if (currentDir < findDir){
      giveUp = true;
    }
  }
  if (turns > 3){
    if ((currentDir > findDir)&&(currentDir < (findDir+10)))
    {
      giveUp = true;
    }
  }
  if (giveUp){
      rFindCC = false;
      lFindCC = false;
      Brake();
      Serial.print(PRETOPIC);
      Serial.print(CARID);
      Serial.print("/cc|{\"id\":");
      Serial.print(findThisCC);
      Serial.println(",\"inRange\":\"false\"}");
  }
}

void doHeartBeat()
{
      Serial.print(PRETOPIC);
      Serial.print(CARID);
      Serial.print("/heartbeat|{\"t\":");
      Serial.print(_heartBeat / 1000.0 );
      Serial.println(" }");
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

