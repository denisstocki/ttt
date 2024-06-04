#include <Arduino.h>

#ifndef MYLCD_h
#define MYLCD_h

#include "LiquidCrystal_I2C.h"

class MyLCD {
  public:
    MyLCD(uint8_t lcd_Addr, uint8_t lcd_cols, uint8_t lcd_rows);
    void init();
    void update(uint8_t left_speed, uint8_t right_speed, uint8_t left_direction, uint8_t right_direction);
    void write_back(uint8_t pos);
    void write_speed(uint8_t left_speed, uint8_t right_speed);
    void write_stop();
    void write_forward(uint8_t pos);
    void write_distance(int lw_cnt, int rw_cnt);
  private:
    LiquidCrystal_I2C* lcd;
};


#endif