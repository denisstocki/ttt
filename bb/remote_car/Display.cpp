/*----INCLUDES----*/
#include "Display.h"
/*----INCLUDES----*/


/*----VARIABLES----*/
unsigned char specialCharacters[DISPLAY_CHAR_COUNT][8] = {
{
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b11111,
    0b01110,
    0b00100,
    0b00000
},
{
    0b00100,
    0b01110,
    0b11111,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00000
}
};
/*----VARIABLES----*/


/*----DISPLAY/Display----*/
Display::Display(unsigned char addr) {
    this -> lcd = new LiquidCrystal_I2C(addr, MAX_COLS, MAX_ROWS);
}
/*----DISPLAY/Display----*/


/*----DISPLAY/setup----*/
void Display::setup(void) {
    this -> lcd -> init();
    this -> lcd -> backlight();

    this -> lcd -> createChar(DISPLAY_CHAR_ARROW_DOWN, specialCharacters[DISPLAY_CHAR_ARROW_DOWN]);
    this -> lcd -> createChar(DISPLAY_CHAR_ARROW_UP, specialCharacters[DISPLAY_CHAR_ARROW_UP]);

    this -> update(0, 0, WHEELS_DIR_STP, WHEELS_DIR_STP, 0);
}
/*----DISPLAY/setup----*/


/*----DISPLAY/update----*/
void Display::update(
    unsigned int speedLt, 
    unsigned int speedRt, 
    WheelsDirectionE dirLt, 
    WheelsDirectionE dirRt,
    unsigned char animationIdx
) {
    unsigned char digitsRt;

    speedLt = min(static_cast<int>(speedLt), 999);
    speedRt = min(static_cast<int>(speedRt), 999);

    digitsRt = log10(abs(speedRt)) + 1;
    Serial.println(digitsRt);
    this -> writeString("   ", 0, 0);
    this -> writeString("   ", 13, 0);

    this -> writeNumber(speedLt, 0, 0);
    this -> writeNumber(speedRt, 16 - digitsRt, 0);

    if (dirLt == WHEELS_DIR_FWD) {
      this -> writeString(" ", 4, (animationIdx + 1) % 2);
        this -> writeSpecialChar(DISPLAY_CHAR_ARROW_UP, 4, animationIdx % 2);
    } else if (dirLt == WHEELS_DIR_BWD) {
    this -> writeString(" ", 4, (animationIdx + 1) % 2);
        this -> writeSpecialChar(DISPLAY_CHAR_ARROW_DOWN, 4, animationIdx % 2);
    } else {
        this -> writeString("X", 4, 0);
        this -> writeString("X", 4, 1);
    }
    
    if (dirRt == WHEELS_DIR_FWD) {
    this -> writeString(" ", 11, (animationIdx + 1) % 2);
        this -> writeSpecialChar(DISPLAY_CHAR_ARROW_UP, 11, animationIdx % 2);
    } else if (dirRt == WHEELS_DIR_BWD) {
    this -> writeString(" ", 11, (animationIdx + 1) % 2);
        this -> writeSpecialChar(DISPLAY_CHAR_ARROW_DOWN, 11, animationIdx % 2);
    } else {
        this -> writeString("X", 11, 0);
        this -> writeString("X", 11, 1);
    }
}
/*----DISPLAY/update----*/


/*----DISPLAY/writeNumber----*/
void Display::writeNumber(unsigned int number, unsigned char col, unsigned char row) {
    this -> lcd -> setCursor(col, row);
    this -> lcd -> print(number);
}
/*----DISPLAY/writeNumber----*/


/*----DISPLAY/writeString----*/
void Display::writeString(char* text, unsigned char col, unsigned char row) {
    this -> lcd -> setCursor(col, row);
    this -> lcd -> print(text);
}
/*----DISPLAY/writeString----*/


/*----DISPLAY/writeSpecialChar----*/
void Display::writeSpecialChar(DisplayCharIdE id, unsigned char col, unsigned char row) {
    this -> lcd -> setCursor(col, row);
    this -> lcd -> write(id);
}
/*----DISPLAY/writeSpecialChar----*/


/*----DISPLAY/writeSonarInfo----*/
void Display::writeSonarInfo(void) {
    this -> lcd -> clear();
    this -> lcd -> setCursor(6, 0);
    this -> lcd -> print("SONAR");
    this -> lcd -> setCursor(6, 1);
    this -> lcd -> print("MEAS");
}
/*----DISPLAY/writeSonarInfo----*/


/*----DISPLAY/clear----*/
void Display::clear(void) {
    this -> lcd -> clear();
}
/*----DISPLAY/clear----*/
