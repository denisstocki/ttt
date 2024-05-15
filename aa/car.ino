/* > Inner libraries include */
#include "Car-remote.h"
#include "lcd-monitor.h"
#include "Ticker.h"
/* > Outer libraries include */
#include <Servo.h>
#include <LiquidCrystal_I2C.h>

/* > Defines for digital pins usage */
#define DIGPIN_ENA 10
#define DIGPIN_IN1 4
#define DIGPIN_IN2 5
#define DIGPIN_IN3 7
#define DIGPIN_IN4 8
#define DIGPIN_ENB 11

#define DIGPIN_RECV 2

#define DIGPIN_SERVO 9

/* > Defines for analog pins usage */
#define INTINPUT0 A0
#define INTINPUT1 A1

#define TRIG A2
#define ECHO A3

/* > Defines for lcd */
#define LCD_ADDR 0x27
#define LCD_SIZE_HORIZONTAL 16
#define LCD_SIZE_VERTICAL 2

uint8_t animation_frame_index = 0;

void update_lcd_monitor()
{
  float speeds_sum = 3.7f;

  lcd_monitor_display_int(0, 0, 64);
  lcd_monitor_display_int(0, 14, 64);
  lcd_monitor_display_special_character_nonanim(1, 0, 3);
  lcd_monitor_display_special_character_nonanim(1, 1, 4);
  lcd_monitor_display_int(0, 8, 2);
  lcd_monitor_display_special_character_nonanim(1, 15, 5);
  lcd_monitor_display_special_character_nonanim(1, 14, 4);

  

  lcd_monitor_display_special_character_nonanim(0, 3, 1);
  lcd_monitor_display_special_character_nonanim(1, 3, 0);
  lcd_monitor_display_special_character_nonanim(0, 12, 1);
  lcd_monitor_display_special_character_nonanim(1, 12, 0);
  lcd_monitor_display_special_character_nonanim(0, 7, 2);

lcd_monitor_display_special_character(0, 5, 0, animation_frame_index);
  lcd_monitor_display_special_character(1, 5, 0, animation_frame_index);
  lcd_monitor_display_special_character(0, 10, 0, animation_frame_index);
  lcd_monitor_display_special_character(1, 10, 0, animation_frame_index);
  animation_frame_index = (animation_frame_index + 1) % 16;
  // if(speeds_sum == 0)
  // {
    // lcd_monitor_display_special_character(0, 7, LCD_STOP_ANIMATION, animation_frame_index);
    // animation_frame_index = (animation_frame_index + 1) % LCD_FRAMES_NUMBER;
  // }
  // else if(speeds_sum > 0)
  // {
  //   lcd_monitor_display_special_character(0, 7, LCD_FORWARD_ANIMATION, animation_frame_index);
  //   animation_frame_index = (animation_frame_index + 1) % LCD_FRAMES_NUMBER;
  // }
  // else
  // {
  //   lcd_monitor_display_special_character(0, 7, LCD_BACKWARD_ANIMATION, animation_frame_index);
  //   animation_frame_index = (animation_frame_index + 1) % LCD_FRAMES_NUMBER;
  // }
}

/* > Variables definitions */
IRrecv recv(DIGPIN_RECV);
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_SIZE_HORIZONTAL, LCD_SIZE_VERTICAL);
Ticker lcd_monitor_ticker(500, update_lcd_monitor);

void setup(
    void
) {
    // lcd.begin(LCD_SIZE_HORIZONTAL, LCD_SIZE_VERTICAL);
    // lcd.setCursor(1, 0);
    // lcd.println("Press a button.");

    // recv.enableIRIn();
    lcd_monitor_initialize();
}

void loop(
    void
) {
    // Helps recognize codes for remote buttons
    // if (recv.decode()) {
    //   lcd.clear();
    //   lcd.setCursor(8, 0);
    //   lcd.println(recv.decodedIRData.command);

    //   recv.resume();
    // }
    // lcd_monitor_ticker.check();

}

