/*----LIBRARIES----*/
#include "Wheels.h"
/*----LIBRARIES----*/


/*----DEFINES----*/
#define SET_MOVEMENT(side, fwd, bwd) digitalWrite(side[0], fwd);\
                                     digitalWrite(side[1], bwd)
/*----DEFINES----*/


/*----WHEELS/Wheels----*/
Wheels::Wheels(void) {
    this -> cntLt = 0;
    this -> cntRt = 0;
    this -> dirLt = WHEELS_DIR_STP;
    this -> dirRt = WHEELS_DIR_STP;
}
/*----WHEELS/Wheels----*/


/*----WHEELS/Setup----*/
void Wheels::setup(
    unsigned char pinFwdLt, 
    unsigned char pinBwdLt, 
    unsigned char pinEnLt,
    unsigned char pinFwdRt, 
    unsigned char pinBwdRt, 
    unsigned char pinEnRt
) {
    pinMode(pinFwdLt, OUTPUT);
    pinMode(pinBwdLt, OUTPUT);
    pinMode(pinEnLt, OUTPUT);
    pinMode(pinFwdRt, OUTPUT);
    pinMode(pinBwdRt, OUTPUT);
    pinMode(pinEnRt, OUTPUT);

    this -> pinsLt[0] = pinFwdLt;
    this -> pinsLt[1] = pinBwdLt;
    this -> pinsLt[2] = pinEnLt;
    this -> pinsRt[0] = pinFwdRt;
    this -> pinsRt[1] = pinBwdRt;
    this -> pinsRt[2] = pinEnRt;

    analogWrite(this -> pinsLt[2], SPEED);
    analogWrite(this -> pinsRt[2], SPEED);
} 
/*----WHEELS/Setup----*/


/*----WHEELS/goForwardLt----*/
void Wheels::goForwardLt(void) {
    SET_MOVEMENT(pinsLt, HIGH, LOW);
    this -> dirLt = WHEELS_DIR_FWD;
}
/*----WHEELS/goForwardLt----*/


/*----WHEELS/goForwardRt----*/
void Wheels::goForwardRt(void) {
    SET_MOVEMENT(pinsRt, HIGH, LOW);
    this -> dirRt = WHEELS_DIR_FWD;
}
/*----WHEELS/goForwardRt----*/


/*----WHEELS/goBackwardLt----*/
void Wheels::goBackwardLt(void) {
    SET_MOVEMENT(pinsLt, LOW, HIGH);
    this -> dirLt = WHEELS_DIR_BWD;
}
/*----WHEELS/goBackwardLt----*/


/*----WHEELS/goBackwardRt----*/
void Wheels::goBackwardRt(void) {
    SET_MOVEMENT(pinsRt, LOW, HIGH);
    this -> dirRt = WHEELS_DIR_BWD;
}
/*----WHEELS/goBackwardRt----*/


/*----WHEELS/stopLt----*/
void Wheels::stopLt(void) {
    SET_MOVEMENT(pinsLt, LOW, LOW);
    this -> dirLt = WHEELS_DIR_STP;
}
/*----WHEELS/stopLt----*/


/*----WHEELS/stopRt----*/
void Wheels::stopRt(void) {
    SET_MOVEMENT(pinsRt, LOW, LOW);
    this -> dirRt = WHEELS_DIR_STP;
}
/*----WHEELS/stopRt----*/


/*----WHEELS/goForward----*/
void Wheels::goForward(void) {
    this -> goForwardLt();
    this -> goForwardRt();
}
/*----WHEELS/goForward----*/


/*----WHEELS/goBackward----*/
void Wheels::goBackward(void) {
    this -> goBackwardLt();
    this -> goBackwardRt();
}
/*----WHEELS/goBackward----*/


/*----WHEELS/stop----*/
void Wheels::stop(void) {
    this -> stopLt();
    this -> stopRt();
}
/*----WHEELS/stop----*/


/*----WHEELS/goLeft----*/
void Wheels::goLeft(void) {
    this -> goBackwardLt();
    this -> goForwardRt();
}
/*----WHEELS/goLeft----*/


/*----WHEELS/goRight----*/
void Wheels::goRight(void) {
    this -> goForwardLt();
    this -> goBackwardRt();
}
/*----WHEELS/goRight----*/


/*----WHEELS/goLeft90----*/
void Wheels::goLeft90(void) {
    this -> goLeft();
    delay(200);
    this -> stop();
}
/*----WHEELS/goLeft90----*/


/*----WHEELS/goRight90----*/
void Wheels::goRight90(void) {
    this -> goRight();
    delay(200);
    this -> stop();
}
/*----WHEELS/goRight90----*/

