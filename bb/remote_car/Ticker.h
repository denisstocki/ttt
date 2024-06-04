/*------------------------------------
FILE: Ticker.h
PURPOSE: Handling a given 
    event after a given period of time.
------------------------------------*/


/*----GUARDBANDS----*/
#ifndef TICKER_H
#define TICKER_H
/*----GUARDBANDS----*/


/*----LIBRARIES----*/
#include <Arduino.h>
/*----LIBRARIES----*/


/*----CLASS_TICKER----*/
class Ticker {

    private:
        unsigned long lastTriggerTime;
        unsigned long timePeriodToTrigger;
        void (*functionToTrigger)(void);

    public:
        Ticker(unsigned long pTT, void (*fTT)(void));

        unsigned long getTimePeriodToTrigger(void) const { return timePeriodToTrigger; }
        void triggerFunction(void);

};
/*----CLASS_TICKER----*/


#endif /* TICKER_H */

