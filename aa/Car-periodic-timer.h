/* > Guardbands */
#ifndef CAR_PERIODIC_TIMER_H
#define CAR_PERIODIC_TIMER_H

/* > Outer libraries include */
#include <Arduino.h>

/* > CarPeriodicTimer class */
class CarPeriodicTimer {

    private:
        unsigned long period;
        unsigned long prevTime;
        void (*handler)(void);

    public:
        CarPeriodicTimer(unsigned long period, void(*handler)(void));
        
        void handle();
};

#endif /* CAR_PERIODIC_TIMER_H */