/*------------------------------------
FILE: Wheels.h
PURPOSE: Handling functionality of remote
    car wheels.
------------------------------------*/


/*----GUARDBANDS----*/
#ifndef WHEELS_H
#define WHEELS_H
/*----GUARDBANDS----*/


/*----LIBRARIES----*/
#include <Arduino.h>
/*----LIBRARIES----*/


/*----DEFINES----*/
#define PINS_PER_SIDE 3
/*----DEFINES----*/


/*----TYPES----*/
enum WheelsDirectionE {
    WHEELS_DIR_FWD,
    WHEELS_DIR_BWD,
    WHEELS_DIR_STP
};
/*----TYPES----*/


/*----CLASS_WHEELS----*/
class Wheels {

    private:
        static constexpr unsigned char SPEED = 80;

        unsigned char pinsLt[PINS_PER_SIDE];
        unsigned char pinsRt[PINS_PER_SIDE];

        volatile unsigned int cntLt;
        volatile unsigned int cntRt;

        WheelsDirectionE dirLt;
        WheelsDirectionE dirRt;

    public:
        Wheels(void);

        void setup(unsigned char pinFwdLt, unsigned char pinBwdLt, unsigned char pinEnLt,
            unsigned char pinFwdRt, unsigned char pinBwdRt, unsigned char pinEnRt);

        void goForward(void);
        void goBackward(void);

        void goLeft(void);
        void goRight(void);

        void goLeft90(void);
        void goRight90(void);

        void stop(void);

        void incCntLt(void) { cntLt++; }
        void incCntRt(void) { cntRt++; }

        unsigned int getCntLt(void) { return cntLt; }
        unsigned int getCntRt(void) { return cntRt; }

        WheelsDirectionE getDirLt(void) { return dirLt; }
        WheelsDirectionE getDirRt(void) { return dirRt; }

    private:
        void goForwardLt(void);
        void goForwardRt(void);

        void goBackwardLt(void);
        void goBackwardRt(void);

        void stopLt(void);
        void stopRt(void);

};
/*----CLASS_WHEELS----*/


#endif /* WHEELS_H */
