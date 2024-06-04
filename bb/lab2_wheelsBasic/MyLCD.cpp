#include "MyLCD.h"

uint8_t arrowDown[8] =
{
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b10101,
    0b01110,
    0b00100,
    0b00000
};

uint8_t arrowUp[8] =
{
    0b00100,
    0b01110,
    0b11111,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00000
};

MyLCD::MyLCD(uint8_t lcd_Addr, uint8_t lcd_cols, uint8_t lcd_rows)
{
  lcd = new LiquidCrystal_I2C(lcd_Addr, lcd_cols, lcd_rows);
}

void MyLCD::init()
{
  lcd->init();
  lcd->backlight();

  lcd->createChar(0, arrowDown);
  lcd->createChar(1, arrowUp);
  lcd->setCursor(0,1);
  lcd->print('X');
  lcd->setCursor(15,1);
  lcd->print('X'); 
  write_speed(0, 0);
}

void MyLCD::write_forward(uint8_t pos)
{
  lcd->setCursor(pos, 1);
  lcd->write(1);
}

void MyLCD::write_back(uint8_t pos)
{
  lcd->setCursor(pos, 1);
  lcd->write(0);
}

void MyLCD::update(uint8_t left_speed, uint8_t right_speed, uint8_t left_direction, uint8_t right_direction)
{
  lcd->clear();
  write_speed(left_speed, right_speed);
  if(right_direction == 1){
    write_forward(0);
  } else if (right_direction == 0)
  {
    write_stop();
  } else if(right_direction == 2)
  {
    write_back(0);
  }

  if(left_direction == 1){
    write_forward(15);
  } else if(left_direction == 2)
  {
    write_back(15);
  }
}

void MyLCD::write_stop()
{
  lcd->setCursor(0,1);
  lcd->print('X');
  lcd->setCursor(15,1);
  lcd->print('X'); 
}

void MyLCD::write_speed(uint8_t left_speed, uint8_t right_speed)
{
  lcd->setCursor(0, 0);
  lcd->print(left_speed);

  if(right_speed < 10){
    lcd->setCursor(15, 0);
  }else if(right_speed < 100)
  {
    lcd->setCursor(14, 0);
  } else{
    lcd->setCursor(13, 0);
  }
  lcd->print(right_speed);
}

void MyLCD::write_distance(int lw_cnt, int rw_cnt)
{
    lcd->setCursor(3, 1);
    lcd->print(lw_cnt);
    lcd->setCursor(10, 1);
    lcd->print(rw_cnt);
}