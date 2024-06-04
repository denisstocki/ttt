/*---------LIBRARIES---------*/
#include <Arduino.h>
#include <IRremote.hpp>
#include "Wheels.h"
#include "Display.h"
#include "Ticker.h"
#include "Sonar.h"

/*---------LIBRARIES---------*/


/*---------------DIGITAL---------------*/
#define DIGPIN_WHEELS_LBWD 7  // > PURPLE
#define DIGPIN_WHEELS_LFWD 8  // > GREY
#define DIGPIN_WHEELS_LSPD 5  // > WHITE
#define DIGPIN_WHEELS_RBWD 4  // > BLUE
#define DIGPIN_WHEELS_RFWD 10 // > GREEN
#define DIGPIN_WHEELS_RSPD 6  // > YELLOW
#define DIGPIN_IRRECV 9       // > LEFT
#define DIGPIN_SERVO 3        // > ???
/*---------------DIGITAL---------------*/


/*-------------------ANALOG-------------------*/
#define ANALPIN_WHEELS_LINT A0
#define ANALPIN_WHEELS_RINT A1
#define ANALPIN_LCD_SDA A4      // > LEFT INNER
#define ANALPIN_LCD_SCL A5      // > LEFT OUTER
/*-------------------ANALOG-------------------*/


/*-------DEFINES-------*/
#define MAX_DISTANCE 60
#define DISPLAY_HANDLE_PERIOD 300
#define WHEELS_HANDLE_PERIOD 100
#define SONAR_HANDLE_PERIOD 100
#define REMOTE_HANDLE_PERIOD 300
#define CAR_REMOTE_CODE_ZERO        25
#define CAR_REMOTE_CODE_ONE         69
#define CAR_REMOTE_CODE_TWO         70
#define CAR_REMOTE_CODE_THREE       71
#define CAR_REMOTE_CODE_FOUR        68
#define CAR_REMOTE_CODE_FIVE        54
#define CAR_REMOTE_CODE_SIX         67
#define CAR_REMOTE_CODE_SEVEN       7
#define CAR_REMOTE_CODE_EIGHT       21
#define CAR_REMOTE_CODE_NINE        9
#define CAR_REMOTE_CODE_STAR        194//22
#define CAR_REMOTE_CODE_HASH        13
#define CAR_REMOTE_CODE_OK          56//28
#define CAR_REMOTE_CODE_UPARR       24
#define CAR_REMOTE_CODE_DOWNARR     74//82
#define CAR_REMOTE_CODE_LEFTARR     16//8
#define CAR_REMOTE_CODE_RIGHTARR    90
/*-------DEFINES-------*/


/*--------TYPES--------*/
typedef enum CarStateE {
  CAR_STATE_IDLE,
  CAR_STATE_STOP,
  CAR_STATE_FWD,
  CAR_STATE_BWD,
  CAR_STATE_TURN,
  CAR_STATE_SEARCH
} CarStateE;

typedef enum CarModeE {
  CAR_MODE_AUTOMATIC,
  CAR_MODE_MANUAL
} CarModeE;
/*--------TYPES--------*/


/*--------VARIABLES--------*/
constexpr unsigned char ADDR = 0x27; // ... || 0x3f

bool isActive;

unsigned char animationIdx;

unsigned int speedLt;
unsigned int speedRt;

unsigned int lastCntLt;
unsigned int lastCntRt;

unsigned long lastSpeedMeasTime;

CarModeE carMode;

Wheels wheels;
Sonar sonar;
Display display(ADDR);
/*--------VARIABLES--------*/


void handleWheels() {
    unsigned int currCntLt = wheels.getCntLt();
    unsigned int currCntRt = wheels.getCntRt();
    unsigned int diffCntLt = currCntLt - lastCntLt;
    unsigned int diffCntRt = currCntRt - lastCntRt;
    unsigned long currSpeedMeasTime = millis();
    unsigned long diffSpeedMeasTime = currSpeedMeasTime - lastSpeedMeasTime;

    lastSpeedMeasTime = currSpeedMeasTime;

    lastCntLt = currCntLt;
    lastCntRt = currCntRt;

    speedLt = ((double)((double)(22 * (double)(diffCntLt / 80)) / (double)(diffSpeedMeasTime / 1000))) / 1;
    speedRt = ((double)((double)(22 * (double)(diffCntRt / 80)) / (double)(diffSpeedMeasTime / 1000))) / 1;

    speedLt = random(0, 999);
    speedRt = random(0, 999);
}


void handleSonar() {
    unsigned long distance, d0, d180;

    distance = sonar.measureDistanceInFront();

    if (distance > MAX_DISTANCE) { return; }
    Serial.println("SONAR| Obstacle in front of the car.");

    wheels.stop();
    Serial.println("SONAR| Stop the car.");
    display.writeSonarInfo();

    d0 = sonar.measureDistanceOnAngle(0);
    Serial.println("SONAR| Measuring left hand side of the car.");
    d180 = sonar.measureDistanceOnAngle(180);
    Serial.println("SONAR| Measuring right hand side of the car.");

    if (d0 < MAX_DISTANCE && d180 < MAX_DISTANCE) {
      Serial.println("SONAR| Both hand sides of the car are unsafe.");
        wheels.goLeft90();
        delay(50);
        wheels.goLeft90();
        Serial.println("SONAR| Go backward.");
        delay(500);
        display.clear();
    wheels.goForward();
    sonar.setSonarToFront();
    handleDisplay();
        return;
    }

    if (d0 > d180) {
      Serial.println("SONAR| Left hand side of the car is the safest.");
        wheels.goLeft90();
    } else {
    Serial.println("SONAR| Right hand side of the car is the safest.");
        wheels.goRight90();
    }

    delay(500);

    display.clear();
    wheels.goForward();
    sonar.setSonarToFront();
    handleDisplay();
}


void handleDisplay() {
    display.update(speedLt, speedRt, wheels.getDirLt(), wheels.getDirRt(), animationIdx++);
}


/*--------VARIABLES--------*/
Ticker tickerWheels(WHEELS_HANDLE_PERIOD, handleWheels);
Ticker tickerSonar(SONAR_HANDLE_PERIOD, handleSonar);
Ticker tickerDisplay(DISPLAY_HANDLE_PERIOD, handleDisplay);
/*--------VARIABLES--------*/


/*--------------------SETUP--------------------*/
void setup(
    void
) {
  wheels.setup(
    DIGPIN_WHEELS_LFWD, 
    DIGPIN_WHEELS_LBWD, 
    DIGPIN_WHEELS_LSPD, 
    DIGPIN_WHEELS_RFWD, 
    DIGPIN_WHEELS_RBWD, 
    DIGPIN_WHEELS_RSPD
  );

  IrReceiver.begin(
    DIGPIN_IRRECV, 
    ENABLE_LED_FEEDBACK
  );

  sonar.setup(DIGPIN_SERVO);

  pinMode(ANALPIN_WHEELS_LINT, INPUT);
  pinMode(ANALPIN_WHEELS_RINT, INPUT);

  PCICR  = 0x02;
  PCMSK1 = 0x03;

  display.setup();

  Serial.begin(9600);
  Serial.setTimeout(200);

  randomSeed(analogRead(0));

  animationIdx = 0;
  lastSpeedMeasTime = millis();

  lastCntLt = 0;
  lastCntRt = 0;

  carMode = CAR_MODE_MANUAL;
  isActive = true;
}
/*--------------------SETUP--------------------*/


/*--------------------LOOP--------------------*/
void loop(
  void
) {
    if (!isActive) {
        return;
    }

    tickerWheels.triggerFunction();
    tickerDisplay.triggerFunction();

    if (carMode == CAR_MODE_AUTOMATIC) {
        tickerSonar.triggerFunction();

        if (IrReceiver.decode()) {    
            IrReceiver.resume();

            if (IrReceiver.decodedIRData.command == CAR_REMOTE_CODE_STAR) {
                wheels.stop();
                carMode = CAR_MODE_MANUAL;
            }
        }
    } else {
        if (IrReceiver.decode()) {    
            IrReceiver.resume();
            Serial.println(IrReceiver.decodedIRData.command);

            switch (IrReceiver.decodedIRData.command) {
                case CAR_REMOTE_CODE_UPARR:     wheels.goForward(); break;
                case CAR_REMOTE_CODE_LEFTARR:   wheels.goLeft(); break;
                case CAR_REMOTE_CODE_DOWNARR:   wheels.goBackward(); break;
                case CAR_REMOTE_CODE_RIGHTARR:  wheels.goRight(); break;
                case CAR_REMOTE_CODE_OK:        wheels.stop(); break;
                case CAR_REMOTE_CODE_STAR:      wheels.goForward(); carMode = CAR_MODE_AUTOMATIC; break;
            }
        }
    }
}
/*--------------------LOOP--------------------*/


/*--------------------INCREMENT--------------------*/
ISR(PCINT1_vect){
    if ((PINC & (1 << PC0))) {
        wheels.incCntLt();
    }

    if ((PINC & (1 << PC1))) {
        wheels.incCntRt();
    }
}
/*--------------------INCREMENT--------------------*/
