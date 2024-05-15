/* > Inner libraries include */
#include "Car-periodic-timer.h"

CarPeriodicTimer::CarPeriodicTimer(
    unsigned long period,
    void (*handler)(void)
) {
    this -> period   = period;
    this -> handler  = handler;
    this -> prevTime = 0;
}

void CarPeriodicTimer::handle(
    void
) {
    unsigned long currTime = millis();

    if (currTime - prevTime > period) {
        (*handler)();
        prevTime = currTime;
    }
}