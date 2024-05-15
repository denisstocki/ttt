#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>
#include <Servo.h>

#include "Car-periodic-timer.h"
#include "Car-lcd-alphabet.h"
#include "Car-lcd.h"

#define DIGPIN_RECV 2

#define DIGPIN_SERVO 9

#define DIGPIN_ENLT 6 // white
#define DIGPIN_INLT1 11 // grey
#define DIGPIN_INLT2 10 // purple
#define DIGPIN_INRT1 8 // blue
#define DIGPIN_INRT2 7 // green
#define DIGPIN_ENRT 5 // yellow

#define ANALPIN_TRIG A2
#define ANALPIN_ECHO A3

#define ANALPIN_INTINPUTLT A0
#define ANALPIN_INTINPUTRT A1

#define MAX_DISTANCE 30

#define LCD_ADDR 0x27
#define LCD_SIZE_HORI 16
#define LCD_SIZE_VERT 2

#define PERIOD_RECV 200
#define PERIOD_LCD 200
#define PERIOD_BUZZ 1000
#define PERIOD_WHEELS 100
#define PERIOD_SENSOR 60

#define RECV_CODE_ZERO 25
#define RECV_CODE_ONE 69
#define RECV_CODE_TWO 70
#define RECV_CODE_THREE 71
#define RECV_CODE_FOUR 68
#define RECV_CODE_FIVE 54
#define RECV_CODE_SIX 67
#define RECV_CODE_SEVEN 7
#define RECV_CODE_EIGHT 21
#define RECV_CODE_NINE 9
#define RECV_CODE_OK 28
#define RECV_CODE_STAR 22
#define RECV_CODE_HASH 13
#define RECV_CODE_UP 24
#define RECV_CODE_DOWN 82
#define RECV_CODE_LEFT 8
#define RECV_CODE_RIGHT 90

#define WHEELS_SPEED_MAX 255
#define WHEELS_SPEED_MIN 70
#define WHEELS_SPEED_ZERO 10

#define WHEELS_GEAR_RANGE ((WHEELS_SPEED_MAX + 1 ) / 8)

#define U8_MAX 255

#define ANIMATION_STATE_FORWARD 0
#define ANIMATION_STATE_BACKWARD 1
#define ANIMATION_STATE_LEFT 2
#define ANIMATION_STATE_RIGHT 3
#define ANIMATION_STATE_STOP 4

#define CM_TO_CNT_RATIO 2

#define SET_MOVEMENT(side,f,b) digitalWrite( side[0], f);\
                               digitalWrite( side[1], b)

uint8_t animation_frame_vertical = 0;
uint8_t animation_frame_horizontal = 0;

uint8_t animation_state = ANIMATION_STATE_STOP;

static bool buzzing;
static bool decoded;

float speedLt;
float speedRt;

unsigned long startTime = 0;

volatile uint32_t counterLt;
volatile uint32_t counterRt;

volatile uint32_t lastSensorMeasDistance;

uint32_t lastWheelsMeasTime;

float pi = 3.14;

const int DURATION1 = 100;  
const int DURATION2 = 400;
const int SPEED = 100;



IRrecv recv(DIGPIN_RECV);
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_SIZE_HORI, LCD_SIZE_VERT);
Servo serwo;

int servo() {
  int maxDistance = 0;
  int angleToGo;
  int distance;

  for (byte angle = 0; angle < 180; angle += 20) {
    serwo.write(angle);
    distance = lastSensorMeasDistance;
    if (distance > maxDistance) {
      angleToGo = angle;
      maxDistance = distance;
    }
    delay(500);
  }
  serwo.write(90);
  return angleToGo;
}

void wheelsTurn(
  int angle
){
  if (angle > 0) {
    animation_state = ANIMATION_STATE_RIGHT;
    wheelsForwardRt();
    delay(angle*11);
  } else {
    animation_state = ANIMATION_STATE_LEFT;
    wheelsForwardLt();
    delay(-angle*11);
  }
}

int verifyWheelSpeed(
    int speed
) {
    if (speed > WHEELS_SPEED_MAX) { 
        return WHEELS_SPEED_MAX; 
    } else if (speed < WHEELS_SPEED_ZERO) { 
        return 0; 
    } else if (speed < WHEELS_SPEED_MIN) { 
        return WHEELS_SPEED_MIN; 
    } else { 
        return speed; 
    }
}

void wheelsSetSpeed(
  uint8_t speed
) {
  speed = verifyWheelSpeed(speed);

  analogWrite(DIGPIN_ENLT, speed);
  analogWrite(DIGPIN_ENRT, speed);
}

void wheelsForwardLt(
) {
  animation_state = ANIMATION_STATE_FORWARD;
  digitalWrite(DIGPIN_INLT1, HIGH);
  digitalWrite(DIGPIN_INLT2, LOW);
  Serial.println("WHEELS| 'LT FWD'");
}

void wheelsForwardRt(
) {
  animation_state = ANIMATION_STATE_FORWARD;
  digitalWrite(DIGPIN_INRT1, HIGH);
  digitalWrite(DIGPIN_INRT2, LOW);
  Serial.println("WHEELS| 'RT FWD'");
}

void wheelsBackwardLt(
    int speed
) {
    speed = verifyWheelSpeed(speed);

  digitalWrite(DIGPIN_INLT1, LOW);
  digitalWrite(DIGPIN_INLT2, HIGH);
  analogWrite(DIGPIN_ENLT, speed);
  Serial.println("WHEELS| 'LT BWD'");
}

void wheelsBackwardRt(
    int speed
) {
    speed = verifyWheelSpeed(speed);

  digitalWrite(DIGPIN_INRT1, LOW);
  digitalWrite(DIGPIN_INRT2, HIGH);
  analogWrite(DIGPIN_ENRT, speed);
  Serial.println("WHEELS| 'RT BWD'");
}

void wheelsStop()
{
    digitalWrite(DIGPIN_INLT1, LOW);
  digitalWrite(DIGPIN_INLT2, LOW);
  digitalWrite(DIGPIN_INRT1, LOW);
  digitalWrite(DIGPIN_INRT2, LOW);
  Serial.println("WHEELS| 'STP'");
}

void write_number(uint8_t row, uint8_t col, int num) {
  lcd.setCursor(col, row);
  lcd.print(num);
}

void write_bitchar(uint8_t row, uint8_t col, int sym, int symId) {
  lcd.createChar(symId, bitchar_alphabet[sym]);
  lcd.setCursor(col, row);
  lcd.write(symId);
}

void write_animation_bitchar(uint8_t row, uint8_t col, int animation, uint8_t frame, int symId) {
  lcd.createChar(symId, bitchar_alphabet_animation[animation][frame]);
  lcd.setCursor(col, row);
  lcd.write(symId);
}

void write_animation_uparrow(uint8_t row, uint8_t col, uint8_t frame, int symId1, int symId2) {
  write_animation_bitchar(0, col, 0, frame, symId1);
  write_animation_bitchar(1, col, 0, (frame + 8) % 16, symId2);
}

void write_animation_downarrow(uint8_t row, uint8_t col, uint8_t frame, int symId1, int symId2) {
  write_animation_bitchar(0, col, 1, frame, symId1);
  write_animation_bitchar(1, col, 1, (frame + 8) % 16, symId2);
}

void write_animation_leftarrow(uint8_t row, uint8_t col, uint8_t frame, int symId1, int symId2) {
  write_animation_bitchar(row, col, 2, frame, symId1);
  write_animation_bitchar(row, col + 1, 2, frame, symId1);
}

void write_animation_rightarrow(uint8_t row, uint8_t col, uint8_t frame, int symId1, int symId2) {
  write_animation_bitchar(row, col, 3, frame, symId1);
  write_animation_bitchar(row, col + 1, 3, frame, symId1);
}

void measureDistance(
  /* NO ARGUMENTS */
) {
  uint32_t tot;
  uint32_t distance;

  digitalWrite(ANALPIN_TRIG, HIGH);
  delay(10);
  digitalWrite(ANALPIN_TRIG, LOW);

  tot = pulseIn(ANALPIN_ECHO, HIGH);

  distance = tot / 58;

  lastSensorMeasDistance = distance;
}

void handleSensor(
  void
) {
  measureDistance();

  int cond = 1;
  int angle = 0;

  while (true) {
    switch (cond) {
      case 0: {
        wheelsStop();
        measureDistance();
        if (lastSensorMeasDistance >= MAX_DISTANCE) {
          cond = 1;
        } else {
          cond = 2;
        }
        break;
      }
      case 1: {
        wheelsForwardLt();
        wheelsForwardRt();
        measureDistance();
        if (lastSensorMeasDistance >= MAX_DISTANCE) {
          cond = 1;
        } else {
          cond = 0;
        }
        break;
      }
      case 2: {
        wheelsStop();
        angle = 90 - servo();
        if (lastSensorMeasDistance >= MAX_DISTANCE) {
          cond = 1;
        } else {
          cond = 3;
        }
        break;
      }
      case 3: {
        wheelsTurn(angle);
        if (lastSensorMeasDistance < MAX_DISTANCE) {
          cond = 1;
        } else {
          cond = 0;
        }
        break;
      }
    }
  }
}

void handleRecv(
    void
) {
    if (IrReceiver.decode() && !decoded) {
        int command = IrReceiver.decodedIRData.command;
        IrReceiver.resume();

        if (command != RECV_CODE_ZERO) {
            Serial.println("REMOTE| not '0'");
            return;
        }

        Serial.println("REMOTE| '0'");
        decoded = true;
    }

    if (IrReceiver.decode() && decoded) {
        // Serial.println(recv.decodedIRData.command);

        int command = IrReceiver.decodedIRData.command;

        switch (command) {

            case RECV_CODE_TWO:
                Serial.println("REMOTE| '2'");
                animation_state = ANIMATION_STATE_FORWARD;
                startTime = millis();
                while (millis() - startTime <= DURATION1) {
                  wheelsForwardLt();
                  wheelsForwardRt();
                }
                wheelsStop();
                break;

            case RECV_CODE_FOUR:
                Serial.println("REMOTE| '4'");
                animation_state = ANIMATION_STATE_LEFT;
                counterLt += 100;
                wheelsBackwardLt(WHEELS_SPEED_MIN);
                wheelsForwardRt(WHEELS_SPEED_MIN);
                break;

            case RECV_CODE_FIVE:
                Serial.println("REMOTE| '5'");
                animation_state = ANIMATION_STATE_STOP;
                wheelsStop();
                break;
            
            case RECV_CODE_SIX:
                Serial.println("REMOTE| '6'");
                animation_state = ANIMATION_STATE_RIGHT;
                counterRt += 100;
                wheelsForwardLt(WHEELS_SPEED_MIN);
                wheelsBackwardRt(WHEELS_SPEED_MIN);
                break;
            
            case RECV_CODE_EIGHT:
                Serial.println("REMOTE| '8'");
                animation_state = ANIMATION_STATE_BACKWARD;
                startTime = millis();
                while (millis() - startTime <= DURATION1) {
                  wheelsBackwardLt();
                  wheelsBackwardRt();
                }
                wheelsStop();
                break;
            
            default:
                Serial.println("REMOTE| '-'");
                break;

        }

        decoded = false;

        IrReceiver.resume();
    }
}

void handleLcd(
    void
) {
    /**
     * SYM_ID_0 => LEFT HORIZONTAL ARROW
     * SYM_ID_1 => SEPARATOR
     * SYM_ID_2 => UPPER PART OF LEFT VERTICAL ARROW
     * SYM_ID_3 => LOWER PART OF LEFT VERTICAL ARROW
     * SYM_ID_4 => UPPER PART OF RIGHT VERTICAL ARROW
     * SYM_ID_5 => LOWER PART OF RIGHT VERTICAL ARROW
     * SYM_ID_6 => RIGHT HORIZONTAL ARROW
    */

    char buffer[2];
    char buffer1[2];

    int gear = 0;

    if ((int)speedLt < 10) {
      write_bitchar(POS_ROW_0, POS_COL_1, SYM_BLANK, SYM_ID_7);
      write_bitchar(POS_ROW_0, POS_COL_2, SYM_BLANK, SYM_ID_7);
      write_number(POS_ROW_0, POS_COL_0, (int) speedLt);
    } else if ((int)speedLt < 1000) {
      write_bitchar(POS_ROW_0, POS_COL_2, SYM_BLANK, SYM_ID_7);
      write_number(POS_ROW_0, POS_COL_0, (int) speedLt);
    } else {
      write_number(POS_ROW_0, POS_COL_0, 999);
    }

    if ((int)speedRt < 10) {
      write_bitchar(POS_ROW_0, POS_COL_15, SYM_BLANK, SYM_ID_7);
      write_bitchar(POS_ROW_0, POS_COL_13, SYM_BLANK, SYM_ID_7);
      write_number(POS_ROW_0, POS_COL_14, (int) speedRt);
    } else if ((int)speedRt < 100) {
      write_bitchar(POS_ROW_0, POS_COL_13, SYM_BLANK, SYM_ID_7);
      write_number(POS_ROW_0, POS_COL_14, (int) speedRt);
    } else if ((int)speedRt < 1000) {
      write_number(POS_ROW_0, POS_COL_13, (int) speedRt);
    } else {
      write_number(POS_ROW_0, POS_COL_13, 999);
    }

    /* LEFT SEPARATOR */
    write_bitchar(POS_ROW_0, POS_COL_3, SYM_SEPARATOR_DOWN, SYM_ID_1);
    write_bitchar(POS_ROW_1, POS_COL_3, SYM_SEPARATOR_DOWN, SYM_ID_1);
    /* RIGHT SEPARATOR */
    write_bitchar(POS_ROW_0, POS_COL_12, SYM_SEPARATOR_DOWN, SYM_ID_1);
    write_bitchar(POS_ROW_1, POS_COL_12, SYM_SEPARATOR_DOWN, SYM_ID_1);

    switch (animation_state) {

        case ANIMATION_STATE_BACKWARD:
            if ((int)speedLt < 2) {
              gear = 1;
            } else {
              gear = ((int)speedLt / WHEELS_GEAR_RANGE) + 2;
            }
            /* RIGHT MIDDLE SYMBOL */
            // write_bitchar(POS_ROW_0, POS_COL_8, SYM_BLANK, SYM_ID_5);
            // write_bitchar(POS_ROW_0, POS_COL_7, SYM_BLANK, SYM_ID_5);
            // lcd.setCursor(POS_COL_8, POS_COL_0);             // Ustaw kursor w pozycji (0, 0)
            // lcd.print(gear);
            /* LEFT HORIZONTAL ARROW */
            write_bitchar(POS_ROW_1, POS_COL_0, SYM_ARROW_LEFT, SYM_ID_0); 
            write_bitchar(POS_ROW_1, POS_COL_1, SYM_ARROW_LEFT, SYM_ID_0);
            /* RIGHT HORIZONTAL ARROW */
            write_bitchar(POS_ROW_1, POS_COL_14, SYM_ARROW_RIGHT, SYM_ID_6); 
            write_bitchar(POS_ROW_1, POS_COL_15, SYM_ARROW_RIGHT, SYM_ID_6);
            /* LEFT MIDDLE SYMBOL */
            lcd.setCursor(POS_COL_7, POS_COL_0);             // Ustaw kursor w pozycji (0, 0)
            lcd.print("R");
            lcd.setCursor(POS_COL_8, POS_COL_0);             // Ustaw kursor w pozycji (0, 0)
            lcd.print(gear);
            /* LEFT VERTICAL ARROW */
            write_animation_downarrow(U8_MAX, POS_COL_5, animation_frame_vertical, SYM_ID_2, SYM_ID_3);
            /* RIGHT VERTICAL ARROW */
            write_animation_downarrow(U8_MAX, POS_COL_10, animation_frame_vertical, SYM_ID_4, SYM_ID_5);
            animation_frame_vertical = (animation_frame_vertical + 1) % 16;
            break;

        case ANIMATION_STATE_FORWARD:
            if ((int)speedLt < 2) {
              gear = 1;
            } else {
              gear = ((int)speedLt / WHEELS_GEAR_RANGE) + 2;
            }
            /* RIGHT MIDDLE SYMBOL */
            // write_bitchar(POS_ROW_0, POS_COL_8, SYM_BLANK, SYM_ID_5);
            // write_bitchar(POS_ROW_0, POS_COL_7, SYM_BLANK, SYM_ID_5);
            // lcd.setCursor(POS_COL_8, POS_COL_0);             // Ustaw kursor w pozycji (0, 0)
            // lcd.print(gear);
            /* LEFT HORIZONTAL ARROW */
            write_bitchar(POS_ROW_1, POS_COL_0, SYM_ARROW_LEFT, SYM_ID_0); 
            write_bitchar(POS_ROW_1, POS_COL_1, SYM_ARROW_LEFT, SYM_ID_0);
            /* RIGHT HORIZONTAL ARROW */
            write_bitchar(POS_ROW_1, POS_COL_14, SYM_ARROW_RIGHT, SYM_ID_6); 
            write_bitchar(POS_ROW_1, POS_COL_15, SYM_ARROW_RIGHT, SYM_ID_6);
            /* LEFT MIDDLE SYMBOL */
            lcd.setCursor(POS_COL_7, POS_COL_0);             // Ustaw kursor w pozycji (0, 0)
            lcd.print("D");
            lcd.setCursor(POS_COL_8, POS_COL_0);             // Ustaw kursor w pozycji (0, 0)
            lcd.print(gear);
            /* LEFT VERTICAL ARROW */
            write_animation_uparrow(U8_MAX, POS_COL_5, animation_frame_vertical, SYM_ID_2, SYM_ID_3);
            /* RIGHT VERTICAL ARROW */
            write_animation_uparrow(U8_MAX, POS_COL_10, animation_frame_vertical, SYM_ID_4, SYM_ID_5);
            animation_frame_vertical = (animation_frame_vertical + 1) % 16;
            break;

        case ANIMATION_STATE_LEFT:
            // /* CLEAR LEFT VERTICAL ARROW */
            // write_bitchar(POS_ROW_0, POS_COL_5, SYM_BLANK, SYM_ID_2); 
            // write_bitchar(POS_ROW_1, POS_COL_5, SYM_BLANK, SYM_ID_3);
            // /* CLEAR RIGHT VERTICAL ARROW */
            // write_bitchar(POS_ROW_0, POS_COL_10, SYM_BLANK, SYM_ID_2);
            // write_bitchar(POS_ROW_1, POS_COL_10, SYM_BLANK, SYM_ID_3);
            /* RIGHT HORIZONTAL ARROW */
            write_bitchar(POS_ROW_1, POS_COL_14, SYM_ARROW_RIGHT, SYM_ID_6); 
            write_bitchar(POS_ROW_1, POS_COL_15, SYM_ARROW_RIGHT, SYM_ID_6);
            // /* LEFT MIDDLE SYMBOL */
            // write_bitchar(POS_ROW_0, POS_COL_7, SYM_LETTER_L, SYM_ID_4);
            // /* RIGHT MIDDLE SYMBOL */
            // write_bitchar(POS_ROW_0, POS_COL_8, SYM_LETTER_T, SYM_ID_5);
            lcd.setCursor(POS_COL_7, POS_COL_0);             // Ustaw kursor w pozycji (0, 0)
            lcd.print("LT");
            /* LEFT HORIZONTAL ARROW */
            write_animation_leftarrow(POS_ROW_1, POS_COL_0, animation_frame_horizontal, SYM_ID_0, SYM_ID_0);
            animation_frame_horizontal = (animation_frame_horizontal + 1) % 2;
            /* LEFT VERTICAL ARROW */
            write_animation_downarrow(U8_MAX, POS_COL_5, animation_frame_vertical, SYM_ID_2, SYM_ID_3);
            /* RIGHT VERTICAL ARROW */
            write_animation_uparrow(U8_MAX, POS_COL_10, animation_frame_vertical, SYM_ID_4, SYM_ID_5);
            animation_frame_vertical = (animation_frame_vertical + 1) % 16;
            break;

        case ANIMATION_STATE_RIGHT:
            // /* CLEAR LEFT VERTICAL ARROW */
            // write_bitchar(POS_ROW_0, POS_COL_5, SYM_BLANK, SYM_ID_2); 
            // write_bitchar(POS_ROW_1, POS_COL_5, SYM_BLANK, SYM_ID_3);
            // /* CLEAR RIGHT VERTICAL ARROW */
            // write_bitchar(POS_ROW_0, POS_COL_10, SYM_BLANK, SYM_ID_2);
            // write_bitchar(POS_ROW_1, POS_COL_10, SYM_BLANK, SYM_ID_3);
            /* LEFT HORIZONTAL ARROW */
            write_bitchar(POS_ROW_1, POS_COL_0, SYM_ARROW_LEFT, SYM_ID_0); 
            write_bitchar(POS_ROW_1, POS_COL_1, SYM_ARROW_LEFT, SYM_ID_0);
            // /* LEFT MIDDLE SYMBOL */
            // write_bitchar(POS_ROW_0, POS_COL_7, SYM_LETTER_R, SYM_ID_4);
            // /* RIGHT MIDDLE SYMBOL */
            // write_bitchar(POS_ROW_0, POS_COL_8, SYM_LETTER_T, SYM_ID_5);
            lcd.setCursor(POS_COL_7, POS_COL_0);             // Ustaw kursor w pozycji (0, 0)
            lcd.print("RT");
            /* RIGHT HORIZONTAL ARROW */
            write_animation_rightarrow(POS_ROW_1, POS_COL_14, animation_frame_horizontal, SYM_ID_6, SYM_ID_6);
            animation_frame_horizontal = (animation_frame_horizontal + 1) % 2;
            /* LEFT VERTICAL ARROW */
            write_animation_uparrow(U8_MAX, POS_COL_5, animation_frame_vertical, SYM_ID_2, SYM_ID_3);
            /* RIGHT VERTICAL ARROW */
            write_animation_downarrow(U8_MAX, POS_COL_10, animation_frame_vertical, SYM_ID_4, SYM_ID_5);
            animation_frame_vertical = (animation_frame_vertical + 1) % 16;
            break;

        case ANIMATION_STATE_STOP:
            /* CLEAR RIGHT MIDDLE SYMBOL */
            // write_bitchar(POS_ROW_0, POS_COL_8, SYM_BLANK, SYM_ID_5);
            /* CLEAR LEFT VERTICAL ARROW */
            write_bitchar(POS_ROW_0, POS_COL_5, SYM_BLANK, SYM_ID_2); 
            write_bitchar(POS_ROW_1, POS_COL_5, SYM_BLANK, SYM_ID_3);
            /* CLEAR RIGHT VERTICAL ARROW */
            write_bitchar(POS_ROW_0, POS_COL_10, SYM_BLANK, SYM_ID_4);
            write_bitchar(POS_ROW_1, POS_COL_10, SYM_BLANK, SYM_ID_5);
            /* LEFT HORIZONTAL ARROW */
            write_bitchar(POS_ROW_1, POS_COL_0, SYM_ARROW_LEFT, SYM_ID_0); 
            write_bitchar(POS_ROW_1, POS_COL_1, SYM_ARROW_LEFT, SYM_ID_0);
            /* RIGHT HORIZONTAL ARROW */
            write_bitchar(POS_ROW_1, POS_COL_15, SYM_ARROW_RIGHT, SYM_ID_6);
            write_bitchar(POS_ROW_1, POS_COL_14, SYM_ARROW_RIGHT, SYM_ID_6);
            /* LEFT MIDDLE SYMBOL */
            // write_bitchar(POS_ROW_0, POS_COL_7, SYM_LETTER_N, SYM_ID_4);
            lcd.setCursor(POS_COL_7, POS_COL_0);             // Ustaw kursor w pozycji (0, 0)
            lcd.print("P");
            lcd.setCursor(POS_COL_8, POS_COL_0);             // Ustaw kursor w pozycji (0, 0)
            lcd.print(" ");
            break;

        default:
            break;

    }
}

void handleWheels(
    void
) {

    // Serial.print("Curr meas time: ");
    // Serial.println(currWheelsMeasTime);
    // Serial.println();

    uint32_t cntLt = counterLt;
    uint32_t cntRt = counterRt;

    // Serial.print("Counter left: ");
    // Serial.println(cntLt);
    // Serial.print("Counter right: ");
    // Serial.println(cntRt);
    // Serial.println();

    counterLt = 0;
    counterRt = 0;

    speedLt = (float) (((cntLt * 9)/50.00)*1000)/(2*pi*3);;
    speedRt = (float) (((cntRt * 9)/50.00)*1000)/(2*pi*3);;

    // Serial.print("Speed left: ");
    // Serial.println(speedLt);
    // Serial.print("Speed right: ");
    // Serial.println(speedRt);
    // Serial.println();
}

CarPeriodicTimer timerRecv(PERIOD_RECV, handleRecv);
CarPeriodicTimer timerLcd(PERIOD_LCD, handleLcd);
CarPeriodicTimer timerWheels(PERIOD_WHEELS, handleWheels);
CarPeriodicTimer timerSensor(PERIOD_SENSOR, handleSensor);

void setup(
    void
) {

    pinMode(ANALPIN_TRIG, OUTPUT);
    pinMode(ANALPIN_ECHO, INPUT);

    pinMode(DIGPIN_ENLT, OUTPUT);
    pinMode(DIGPIN_INLT1, OUTPUT);
    pinMode(DIGPIN_INLT2, OUTPUT);
    
    pinMode(DIGPIN_ENRT, OUTPUT);
    pinMode(DIGPIN_INRT1, OUTPUT);
    pinMode(DIGPIN_INRT2, OUTPUT);

    wheelsSetSpeed(SPEED);

    pinMode(ANALPIN_INTINPUT0, INPUT);
    pinMode(ANALPIN_INTINPUT1, INPUT);

    Serial.begin(9600);

    PCICR = 0x02;
    PCMSK1 = 0x03;

    speedLt = 0.0;
    speedRt = 0.0;

    counterLt = 0;
    counterRt = 0;

    decoded = false;

    bool unlocked = false;
    uint32_t password = 4244; // 4424

    IrReceiver.begin(DIGPIN_RECV, ENABLE);

    lcd.init();
    lcd.backlight();

    lcd.setCursor(POS_COL_5, POS_COL_0);             // Ustaw kursor w pozycji (0, 0)
    lcd.print("LOCKED");

    while (!unlocked) {
      bool entered = false;
      uint32_t guess = 0;
      uint32_t mult = 1;

      while (!entered) {
          if (IrReceiver.decode()) {
            int command = IrReceiver.decodedIRData.command;

            switch (command) {

                case RECV_CODE_ZERO:
                    guess += mult * 0;
                    break;

                case RECV_CODE_ONE:
                    guess += mult * 1;
                    break;

                case RECV_CODE_TWO:
                    guess += mult * 2;
                    break;

                case RECV_CODE_THREE:
                    guess += mult * 3;
                    break;

                case RECV_CODE_FOUR:
                    guess += mult * 4;
                    break;

                case RECV_CODE_FIVE:
                    guess += mult * 5;
                    break;

                case RECV_CODE_SIX:
                    guess += mult * 6;
                    break;

                case RECV_CODE_SEVEN:
                    guess += mult * 7;
                    break;
                
                case RECV_CODE_EIGHT:
                    guess += mult * 8;
                    break;
                
                case RECV_CODE_NINE:
                    guess += mult * 9;
                    break;

                case RECV_CODE_OK:
                    entered = true;
                    break;
                
                default:
                    break;

            }

            mult *= 10;
            IrReceiver.resume();
          }
      }

      if (guess == password) {
        unlocked = true;
        Serial.println("DECODER| 'GOOD'");
      } else {
        Serial.println("DECODER| 'BAD'");
        lcd.setCursor(POS_COL_6, POS_COL_1);             // Ustaw kursor w pozycji (0, 0)
        lcd.print("FAIL");
        delay(400);
        write_bitchar(POS_ROW_1, POS_COL_6, SYM_BLANK, SYM_ID_7);
        write_bitchar(POS_ROW_1, POS_COL_7, SYM_BLANK, SYM_ID_7);
        write_bitchar(POS_ROW_1, POS_COL_8, SYM_BLANK, SYM_ID_7);
        write_bitchar(POS_ROW_1, POS_COL_9, SYM_BLANK, SYM_ID_7);
      }

    }

    lcd.setCursor(POS_COL_6, POS_COL_1);             // Ustaw kursor w pozycji (0, 0)
    lcd.print("SUCC");
    delay(400);
    write_bitchar(POS_ROW_1, POS_COL_6, SYM_BLANK, SYM_ID_7);
    write_bitchar(POS_ROW_1, POS_COL_7, SYM_BLANK, SYM_ID_7);
    write_bitchar(POS_ROW_1, POS_COL_8, SYM_BLANK, SYM_ID_7);
    write_bitchar(POS_ROW_1, POS_COL_9, SYM_BLANK, SYM_ID_7);
delay(400);

    lcd.setCursor(POS_COL_5, POS_COL_0);             // Ustaw kursor w pozycji (0, 0)
    lcd.print("UNLOCK");
    delay(800);
    write_bitchar(POS_ROW_0, POS_COL_5, SYM_BLANK, SYM_ID_7);
    write_bitchar(POS_ROW_0, POS_COL_6, SYM_BLANK, SYM_ID_7);
    write_bitchar(POS_ROW_0, POS_COL_7, SYM_BLANK, SYM_ID_7);
    write_bitchar(POS_ROW_0, POS_COL_8, SYM_BLANK, SYM_ID_7);
    write_bitchar(POS_ROW_0, POS_COL_9, SYM_BLANK, SYM_ID_7);
    write_bitchar(POS_ROW_0, POS_COL_10, SYM_BLANK, SYM_ID_7);

    lastWheelsMeasTime = millis();

    wheelsForwardLt();
    wheelsForwardRt();
    // Serial.print("Last meas time: ");
    // Serial.println(lastWheelsMeasTime);
    // Serial.println();
}

void loop(
    void
) {
    timerLcd.handle();
    timerRecv.handle();
    timerWheels.handle();
    timerSensor.handle();
}
