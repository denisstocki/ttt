#include "Wheels.h"
#include "MyLCD.h"
#include "TimerOne.h"
#include "PinChangeInterrupt.h"
#define DECODE_NEC
#include <IRremote.hpp>
#include <Servo.h>


/* DEFINES */

#define LWB 7 // LEFT BACKWARDS - PURPLE
#define LWF 8 // LEFT FORWARD - GREY
#define RWB 4 // RIGHT BACKWARDS - BLUE
#define RWF 10 // RIGHT FORWARD - GREEN
#define LEFT_SPEED_PIN 5 // WHITE
#define RIGHT_SPEED_PIN 6 // YELLOW

#define IR_RECEIVE_PIN 2

#define INTINPUT0 A0 // LEFT
#define INTINPUT1 A1 // RIGHT

// SDA/A4
// SCL/A5


// piny dla sonaru (HC-SR04)
#define TRIG A2
#define ECHO A3
#define SERVO 3

#define DEACTIVATE 0

#define STOP_DISTANCE 60

/* TYPES */
typedef enum State {
  STATE_IDLE,
  STATE_FORWARD,
  STATE_BACKWARD,
  STATE_SEARCH,
  STATE_TURN,
} state_E;

/* STATIC VARIABLES */

Wheels w;
Servo serwo;
volatile char cmd;
unsigned long start_time;

bool auto_mode;
state_E car_state = STATE_IDLE;

byte LCDAddress = 0x27;
//MyLCD lcd(LCDAddress, 16, 2);

/* SETUP FUNCTION*/

void setup() {
  // Wheels
  w.attach(RWF, RWB, RIGHT_SPEED_PIN, LWF, LWB, LEFT_SPEED_PIN);
  pinMode(BEEPER, OUTPUT);
  Timer1.initialize();

  // Wheels sensor
  pinMode(INTINPUT0, INPUT);
  pinMode(INTINPUT1, INPUT);
  PCICR  = 0x02;
  PCMSK1 = 0x03;

  // IRC 
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  // LCD
  // lcd.init();

  // Sonar
  pinMode(TRIG, OUTPUT);    // TRIG startuje sonar
  pinMode(ECHO, INPUT);     // ECHO odbiera powracający impuls

  Serial.begin(9600);

  serwo.attach(SERVO);

  Serial.setTimeout(200);

  start_time = millis();
  Serial.println("Forward: WAD");
  Serial.println("Back: ZXC");
  Serial.println("Stop: S");

  auto_mode = false;
}

/* MAIN LOOP*/

void loop() {
  if(DEACTIVATE)
  {
    return;
  }

  if(auto_mode)
  {
    bool r = w.checkStop();
    if(r && car_state == STATE_TURN)
    {
      w.forward();
      car_state = STATE_FORWARD;
    }
    if(car_state == STATE_IDLE)
    {
      w.forward();
      car_state = STATE_FORWARD;
    }
    check_for_obstacles();

    if (IrReceiver.decode()) {    
      IrReceiver.resume();
      IrReceiver.printIRResultShort(&Serial);
      if(IrReceiver.decodedIRData.command == 0x16)
      {
        auto_mode = !(auto_mode); Serial.print("Auto mode: "); Serial.println(auto_mode);
        w.stop();
        car_state = STATE_IDLE;
      }
      //lcd.update(w.speed_left, w.speed_right, w.left_direction, w.right_direction);
    }
    return;
  }

  /* SENSOR HANDLER */
  check_for_obstacles();

  /* SERIAL HANDLER */

  if(Serial.available())
  {
    cmd = Serial.read();
    switch(cmd)
    {
      case 'w': w.forward(); break;
      case 'x': w.back(); break;
      case 'a': w.forwardLeft();break;
      case 'd': w.forwardRight(); break;
      case 'z': w.backLeft();break;
      case 'c': w.backRight();break;
      case 's': w.stop();break;
      case '1': w.setSpeedLeft(75); break;
      case '2': w.setSpeedLeft(200); break;
      case '9': w.setSpeedRight(75); break;
      case '0': w.setSpeedRight(200); break;
      case '5': w.setSpeed(100); break;
      case 't': w.goForward(200); break;
    }
    //lcd.update(w.speed_left, w.speed_right, w.left_direction, w.right_direction);
  }

  /* TIMER EVENT */

  if(millis() - start_time > 300)
  {
    //lcd.write_distance(w.get_lwcnt(), w.get_rwcnt());
    start_time = millis();
  }
  w.checkStop();

  /* REMOTE HANDLER */

  if (IrReceiver.decode()) {    
    IrReceiver.resume(); // Enable receiving of the next value
    // Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data
    IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
    switch(IrReceiver.decodedIRData.command)
    {
      case 0x18: w.forward(); car_state = STATE_FORWARD;break;
      case 0x1C: w.stop(); car_state = STATE_IDLE; break;
      case 0x52: w.back(); car_state = STATE_BACKWARD; break;
      case 0x45: w.setSpeed(100); break;
      case 0x46: w.setSpeed(150); break;
      case 0x47: w.setSpeed(200); break;
      case 0x44: w.setSpeed(0); break;
      case 0x8: w.turnLeftI(); break;
      case 0x5A: w.turnRightI(); break;
      case 0x19: serwo.write(0); break;
      case 0x15: serwo.write(90); break;
      case 0x16: auto_mode = !(auto_mode); w.setSpeed(160); w.setSpeedRight(220); Serial.print("Auto mode: "); Serial.println(auto_mode); break;
    }
    //lcd.update(w.speed_left, w.speed_right, w.left_direction, w.right_direction);
  }
}

/* INTERUPT HANDLER */

ISR(PCINT1_vect) {
   if( (PINC & (1 << PC0)) ) 
    w.inc_rwcnt();

  if( (PINC & (1 << PC1)) )
    w.inc_lwcnt();
}

unsigned int tellDistance() {
  unsigned long tot;      // czas powrotu (time-of-travel)
  unsigned int distance;
  digitalWrite(TRIG, HIGH);
  delay(10);
  digitalWrite(TRIG, LOW);
  tot = pulseIn(ECHO, HIGH);
  distance = tot/58;
  return distance;
}

unsigned int lookAndTellDistance(byte angle) {
  
  unsigned long tot;      // czas powrotu (time-of-travel)
  unsigned int distance;

  Serial.print("Patrzę w kącie ");
  Serial.println(angle);
  serwo.write(angle);
  delay(300);
/* uruchamia sonar (puls 10 ms na `TRIGGER')
 * oczekuje na powrotny sygnał i aktualizuje
 */
  digitalWrite(TRIG, HIGH);
  delay(10);
  digitalWrite(TRIG, LOW);
  tot = pulseIn(ECHO, HIGH);

/* prędkość dźwięku = 340m/s => 1 cm w 29 mikrosekund
 * droga tam i z powrotem, zatem:
 */
  distance = tot/58;
  /*
  if(distance < 100)
  {
    Serial.print(": widzę coś w odległości ");
    Serial.println(distance);
  }
  */
  Serial.print(": widzę coś w odległości ");
  Serial.println(distance);
  return distance;
}

void check_for_obstacles()
{
  unsigned int dist = tellDistance();

  if(car_state == STATE_FORWARD && dist < STOP_DISTANCE && (w.left_direction != 0 || w.right_direction != 0))
  {
    w.stop();
    car_state = STATE_SEARCH;
  }

  if(car_state == STATE_SEARCH)
  {
    unsigned int best_dist = 0;
    byte best_i = 0;
    for(byte i = 0; i <= 180; i += 180)
    {
      dist = lookAndTellDistance(i);
      if( dist > best_dist )
      {
        best_dist = dist;
        best_i = i;
      }
    }
    serwo.write(90);

    if(best_dist > STOP_DISTANCE)
    {
      // turn by best_i
      car_state = STATE_TURN;
      if(best_i > 90)
      {
        w.turnLeft(15);
      }
      else if(best_i < 90)
      {
        w.turnRight(15);
      }else
      {
        w.forward();
        car_state = STATE_FORWARD;
      }
    }
  }
}
