/*----LIBRARIES----*/
#include "Ticker.h"
/*----LIBRARIES----*/


/*----TICKER/Ticker----*/
Ticker::Ticker(unsigned long pTT, void (*fTT)(void)) {
  this -> timePeriodToTrigger = pTT;
  this -> functionToTrigger = fTT;
  this -> lastTriggerTime = 0;
}
/*----TICKER/Ticker----*/


/*----TICKER/trigger----*/
void Ticker::triggerFunction(void) {
  unsigned long currentTriggerTime;
  unsigned long diffTriggerTime;

  currentTriggerTime = millis();
  diffTriggerTime = currentTriggerTime - (this -> lastTriggerTime);

  if (diffTriggerTime < (this -> timePeriodToTrigger)) {
    return;
  }

  if (this -> functionToTrigger != nullptr) {
    (this -> functionToTrigger)();
    this -> lastTriggerTime = currentTriggerTime;
  }
}
/*----TICKER/trigger----*/

