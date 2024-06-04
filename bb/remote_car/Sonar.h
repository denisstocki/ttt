/*------------------------------------
FILE: Sonar.h
PURPOSE: Handling measurement of distance
    to any obstacles seen in front of the
    sonar.
------------------------------------*/


/*----GUARDBANDS----*/
#ifndef SONAR_H
#define SONAR_H
/*----GUARDBANDS----*/


/*----LIBRARIES----*/
#include <Arduino.h>
#include <Servo.h>
/*----LIBRARIES----*/


/*----DEFINES----*/
#define ANALPIN_SONAR_TRIG A2
#define ANALPIN_SONAR_ECHO A3



/*----DEFINES----*/


/*----CLASS_SONAR----*/
class Sonar {

    private:
        unsigned char pinServo;

        Servo serwo;

    public:
        void setup(unsigned char pinServo);

        void setSonarToFront(void);

        unsigned long measureDistanceInFront(void);
        unsigned long measureDistanceOnAngle(unsigned char angle);

};
/*----CLASS_SONAR----*/


#endif /* SONAR_H */
