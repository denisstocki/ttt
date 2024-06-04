#include <Arduino.h>

#include "Wheels.h"
#include "TimerOne.h"

#define SET_MOVEMENT(side,f,b) digitalWrite( side[0], f);\
                               digitalWrite( side[1], b)

static unsigned long calculateDelay(uint16_t cm);
static void doBeep();


static volatile int lwCnt, rwCnt;
static long int intPeriod = 500000;

Wheels::Wheels() 
{
  lwCnt = 0;
  rwCnt = 0;
}

void Wheels::inc_lwcnt()
{
  lwCnt++;
}

void Wheels::inc_rwcnt()
{
  rwCnt++;
}

int Wheels::get_lwcnt()
{
  return lwCnt;
}

int Wheels::get_rwcnt()
{
  return rwCnt;
}

void Wheels::attachRight(int pF, int pB, int pS)
{
    pinMode(pF, OUTPUT);
    pinMode(pB, OUTPUT);
    pinMode(pS, OUTPUT);
    this->pinsRight[0] = pF;
    this->pinsRight[1] = pB;
    this->pinsRight[2] = pS;
}


void Wheels::attachLeft(int pF, int pB, int pS)
{
    pinMode(pF, OUTPUT);
    pinMode(pB, OUTPUT);
    pinMode(pS, OUTPUT);
    this->pinsLeft[0] = pF;
    this->pinsLeft[1] = pB;
    this->pinsLeft[2] = pS;
}

void Wheels::setSpeedRight(uint8_t s)
{
    analogWrite(this->pinsRight[2], s);
    this->speed_right = s;
}

void Wheels::setSpeedLeft(uint8_t s)
{
    analogWrite(this->pinsLeft[2], s);
    this->speed_left = s;
}

void Wheels::setSpeed(uint8_t s)
{
    setSpeedLeft(s);
    setSpeedRight(s);
}

void Wheels::attach(int pRF, int pRB, int pRS, int pLF, int pLB, int pLS)
{
    this->attachRight(pRF, pRB, pRS);
    this->attachLeft(pLF, pLB, pLS);
}

void Wheels::forwardLeft() 
{
    SET_MOVEMENT(pinsLeft, HIGH, LOW);
    this->left_direction = 1;
    lwCnt = 0;
}

void Wheels::forwardRight() 
{
    SET_MOVEMENT(pinsRight, HIGH, LOW);
    this->right_direction = 1;
    rwCnt = 0;
}

void Wheels::backLeft()
{
    SET_MOVEMENT(pinsLeft, LOW, HIGH);
    this->left_direction = 2;
}

void Wheels::backRight()
{
    SET_MOVEMENT(pinsRight, LOW, HIGH);
    this->right_direction = 2;
}

void Wheels::forward()
{
    this->forwardLeft();
    this->forwardRight();
}

void Wheels::back()
{
    this->backLeft();
    this->backRight();
    Timer1.detachInterrupt();
    Timer1.attachInterrupt(doBeep, intPeriod);
}

void Wheels::stopLeft()
{
    SET_MOVEMENT(pinsLeft, LOW, LOW);
    this->left_direction = 0;
}

void Wheels::stopRight()
{
    SET_MOVEMENT(pinsRight, LOW, LOW);

    this->right_direction = 0;
}

void Wheels::stop()
{
    this->stopLeft();
    this->stopRight();
    digitalWrite(BEEPER, 0);
}

void Wheels::goForward(uint16_t cm)
{
  unsigned long d = calculateDelay(cm);
  this->checkDistance = d;
  forward();
}

void Wheels::goBack(uint16_t cm)
{
  unsigned long d = calculateDelay(cm);
  this->checkDistance = d;
  back();
}

static unsigned long calculateDelay(uint16_t cm)
 {
    // 80 ticks ~ 22cm
    unsigned long d = 80;
    return d * ( (double)cm / 22);
 }

static void doBeep() {
  digitalWrite(BEEPER, digitalRead(BEEPER) ^ 1);
}

bool Wheels::checkStop()
{
  if(!this->checkDistance)
  {
    return false;
  }
  if(lwCnt >= this->checkDistance || rwCnt >= this->checkDistance)
  {
    this->stop();
    this->checkDistance = 0;
    return true;
  }
  return false;
}

 void Wheels::turnLeft(byte angle)
 {

    this->backLeft();
    this->forwardRight();
    this->checkDistance = calculateDelay(angle);
 }

 void Wheels::turnLeftI()
 {
    this->backLeft();
    this->forwardRight();
 }

  void Wheels::turnRight(byte angle)
 {

    this->forwardLeft();
    this->backRight();
    this->checkDistance = calculateDelay(angle);
 }

 void Wheels::turnRightI()
 {
    this->forwardLeft();
    this->backRight();
 }

