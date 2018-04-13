#ifndef PERIODICACTION_H
#define PERIODICACTION_H

#include "Arduino.h"


/**
  Non-interruptive mechanism for periodically executing an action.

  @author Andrew H. Fagg

*/
  
class PeriodicAction {
  public:
   PeriodicAction(unsigned long dt, void(*func)());
   void step();
   int getOverrun();
  private:
   unsigned long dt; 
   unsigned long time;
   void (*func)();
   int overrun;
};

#endif

