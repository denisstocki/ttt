/*------------------------------------
FILE: Display.h
PURPOSE: Handling visualization of car's
    parameters on an LCD display.
------------------------------------*/


/*----GUARDBANDS----*/
#ifndef DISPLAY_H
#define DISPLAY_H
/*----GUARDBANDS----*/


/*----INCLUDES----*/
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "Wheels.h"
/*----INCLUDES----*/


/*----DEFINES----*/
#define MAX_ROWS 2
#define MAX_COLS 16
/*----DEFINES----*/


/*----TYPES----*/
enum DisplayCharIdE {
    DISPLAY_CHAR_ARROW_DOWN,
    DISPLAY_CHAR_ARROW_UP,
    DISPLAY_CHAR_COUNT
};
/*----TYPES----*/


/*----CLASS_DISPLAY----*/
class Display {

    private:
        static constexpr unsigned char MAX_NON_DEFAULT_CHARS = 8;
        LiquidCrystal_I2C* lcd;

    public:
        Display(unsigned char addr);

        void setup(void);
        void update(unsigned int speedLt, unsigned int speedRt, 
            WheelsDirectionE dirLt, WheelsDirectionE dirRt, unsigned char animationIdx);
        void writeSonarInfo(void);
        void clear(void);

    private:
        void writeNumber(unsigned int number, unsigned char col, unsigned char row);
        void writeString(char* text, unsigned char col, unsigned char row);
        void writeSpecialChar(DisplayCharIdE id, unsigned char col, unsigned char row);

};
/*----CLASS_DISPLAY----*/


#endif /* DISPLAY_H */
