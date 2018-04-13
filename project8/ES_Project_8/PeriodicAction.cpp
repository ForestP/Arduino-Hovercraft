#include "PeriodicAction.h"

/**
  Non-interruptive mechanism for periodically executing an action.
  
  Each action (or tasks) is defined by a function call and  a period between 
  executions of the function.  Note: this implementation assumes that Step() is
  called frequently enough to check the clock.  Clock overruns by a ms or more are
  counted & do not (necessarily) result in a drift in the period.

  @author Andrew H. Fagg

  Usage: 
  1. Initialization: create one or more PeriodicAction objects:

  PeriodicAction action1 = new Periodic(100, &sensorStep);
  
  2. For each object, call step often (usually within the loop() function):

  action1->step()


  This assumes that the following function is defined:

  void sensorStep(){...}

  The function sensorStep() will then be called once every 100 ms

*/


/**
  PeriodicAction constructor
  
  @param dt = period between function calls (ms)
  @param func = pointer to function to be called (void*(func)())
  
*/

PeriodicAction::PeriodicAction(unsigned long dt, void(*func)())
{
  this->dt = dt;
  this->func = func;  
  this->time = 0;
  this->overrun = 0;
}

/**
  Take a single step of checking whether the timer has expired.  If so, then the 
  associated function is called
*/

void PeriodicAction::step() {
  
  // Current time
  unsigned long tm_current = millis();
  // If we have not yet set time (tm), then just set it to the current time.
  
  if(this->time == 0) 
    this->time = tm_current;
  
  
  // Change since the last time step
  // Note: timer will wrap around after 50 days!
  unsigned long dt = tm_current - this->time;
  
  
  // Are we ready to do the control work?
  if(dt >= this->dt){
    // Yes - increment "last" counter appropriately
    this->time += this->dt;
    
    // Check to see if we are more than one ms late
    if(dt > this->dt) { 
      this->overrun++;
    }
    
    // Call the function
    (*this->func)();
   
  }
}

/**
  Return the number of times that the timer has overrun the clock by more than one ms
  
  @return Number of times that the timer has overrun by more than one ms
*/

int PeriodicAction::getOverrun() {
  return(this->overrun);
}

