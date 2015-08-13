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
long findDir = 0.0;
char findThisCC[10]= "";

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
  // Magnetic Declination is the correction applied according to your present location
  // in order to get True North from Magnetic North, it varies from place to place.
  // 
  // The declination for your area can be obtained from http://www.magnetic-declination.com/
  // Take the "Magnetic Declination" line that it gives you in the information, 
  //
  // Examples:
  //   Christchurch, 23° 35' EAST
  //   Wellington  , 22° 14' EAST
  //   Dunedin     , 25° 8'  EAST
  //   Auckland    , 19° 30' EAST
  //   Bergen      ,  0° 28' EAST
  Compass.SetDeclination(0, 28, 'E');  
  
  // The device can operate in SINGLE (default) or CONTINUOUS mode
  //   SINGLE simply means that it takes a reading when you request one
  //   CONTINUOUS means that it is always taking readings
  // for most purposes, SINGLE is what you want.
  Compass.SetSamplingMode(COMPASS_SINGLE);
  
  // The scale can be adjusted to one of several levels, you can probably leave it at the default.
  // Essentially this controls how sensitive the device is.
  //   Options are 088, 130 (default), 190, 250, 400, 470, 560, 810
  // Specify the option as COMPASS_SCALE_xxx
  // Lower values are more sensitive, higher values are less sensitive.
  // The default is probably just fine, it works for me.  If it seems very noisy
  // (jumping around), incrase the scale to a higher one.
  Compass.SetScale(COMPASS_SCALE_810);
  
  // Pixy-cam
  pixy.init();
  
  // Serial
  Serial.begin(19200);
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

int getHeading()
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
  return 1;
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
  
  // grab blocks!
  blocks = pixy.getBlocks();
  
  // If there are detect blocks, print them!
  if (blocks)
  {
    int j;
    for (j=0;j<blocks;j++)
    {
      Serial.print("car/");
      Serial.print(CARID);
      Serial.print("/cc/");
      Serial.print(pixy.blocks[j].signature,OCT);
      Serial.print("/{x:");
      Serial.print(pixy.blocks[j].x);
      Serial.print(",w:");
      Serial.print(pixy.blocks[j].width);
      Serial.println("}");
    }
  }    
}

void initFindCC()
{
  // 1. Read initial bearing
  // 2. Parse string representing CC to find
}

void findCC()
{
  // 1. Check if correct CC is within tolerance
  // 2. If not - Check if past initial bearing. 
  //       If true notify CC not found on MQTT
  //     else 
  //       Rotate a step in correct direction
  // 3. If found, publish on MQTT
}

