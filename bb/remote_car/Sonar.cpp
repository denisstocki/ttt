/*----LIBRARIES----*/
#include "Sonar.h"
/*----LIBRARIES----*/


/*----DEFINES----*/
#define FRONT_POSITION 90
/*----DEFINES----*/


/*----SONAR/setup----*/
void Sonar::setup(
  unsigned char pinServo
) {
  this -> pinServo = pinServo;




































  pinMode(ANALPIN_SONAR_TRIG, OUTPUT);
  pinMode(ANALPIN_SONAR_ECHO, INPUT);

  this -> serwo.attach(this -> pinServo);
  this -> setSonarToFront();
}
/*----SONAR/setup----*/


/*----SONAR/setSonarToFront----*/
void Sonar::setSonarToFront(void) {
  this -> serwo.write(FRONT_POSITION);
}
/*----SONAR/setSonarToFront----*/


/*----SONAR/measureDistanceInFront----*/
unsigned long Sonar::measureDistanceInFront(void) {
  unsigned long tot;
  unsigned long distance;

  digitalWrite(ANALPIN_SONAR_TRIG, HIGH);
  delay(10);
  digitalWrite(ANALPIN_SONAR_TRIG, LOW);

  tot = pulseIn(ANALPIN_SONAR_ECHO, HIGH);
  distance = tot / 58;

  return distance;
}
/*----SONAR/measureDistanceInFront----*/


/*----SONAR/measureDistanceOnAngle----*/
unsigned long Sonar::measureDistanceOnAngle(unsigned char angle) {
  unsigned long distance;

  this -> serwo.write(angle);
  delay(300);
  distance = this -> measureDistanceInFront();

  return distance;
}
/*----SONAR/measureDistanceOnAngle----*/

