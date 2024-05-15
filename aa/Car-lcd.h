/* > Guardbands */
#ifndef CAR_LCD_H
#define CAR_LCD_H

/* > Outer libraries include */

enum Matrix_row {
    POS_ROW_0 = 0,
    POS_ROW_1,
    COUNT_ROWS
};

enum Matrix_col {
    POS_COL_0 = 0,
    POS_COL_1,
    POS_COL_2,
    POS_COL_3,
    POS_COL_4,
    POS_COL_5,
    POS_COL_6,
    POS_COL_7,
    POS_COL_8,
    POS_COL_9,
    POS_COL_10,
    POS_COL_11,
    POS_COL_12,
    POS_COL_13,
    POS_COL_14,
    POS_COL_15,
    COUNT_COLS
};

enum Symbol_id {
    SYM_ID_0 = 0,
    SYM_ID_1,
    SYM_ID_2,
    SYM_ID_3,
    SYM_ID_4,
    SYM_ID_5,
    SYM_ID_6,
    SYM_ID_7,
    COUNT_SYMS
};

// class CarLcd {

//     private:
//         LiquidCrystal_I2C lcd;

//         void write_number(uint8_t row, uint8_t col, int num);
//         void write_bitchar(uint8_t row, uint8_t col, int sym, int symId);

//     public:
//         void init();
//         void handle();

// }

#endif /* CAR_LCD_H */