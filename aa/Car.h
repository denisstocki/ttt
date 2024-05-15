/* > Guardbands */
#ifndef CAR_H
#define CAR_H

/* > Inner libraries include */
#include "Car-periodic-timer.h"
#include "Car-remote.h"
#include "Car-lcd.h"

/* > Defines */
#define TIMER_PERIOD_REMOTE 300
#define TIMER_PERIOD_LCD    500

class Car {

    private:
        CarLcd lcd;

        CarPeriodicTimer timerRemote;
        CarPeriodicTimer timerLcd;

        void handleLcd();
        void handleRemote();
}

#endif /* CAR_H */