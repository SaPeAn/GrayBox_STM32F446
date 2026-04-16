#include "scheduler.h"


static uint8 SchedulerRegistredEvents = 0;
static uint8 SchedulerRegistredShortEvents = 0;
tEvent SchedulerEvent[MAX_EVENT];
tSEvent SchedulerShortEvent[MAX_SHORTEVENT];

uint8 SchedAddEvent(void (*func)(void), uint16 period)
{
  if(period == 0) return SchedulerRegistredEvents;
  SchedulerEvent[SchedulerRegistredEvents].callfunc = func;
  SchedulerEvent[SchedulerRegistredEvents].period = period;
  SchedulerEvent[SchedulerRegistredEvents].run_flag = ON;
  return SchedulerRegistredEvents++;
}

uint8 SchedAddShortEvent(void (*func)(void), uint16 period)
{
  if(period == 0) return SchedulerRegistredShortEvents;
  SchedulerShortEvent[SchedulerRegistredShortEvents].callfunc = func;
  SchedulerShortEvent[SchedulerRegistredShortEvents].period = period;
  SchedulerShortEvent[SchedulerRegistredShortEvents].run_flag = ON;
  return SchedulerRegistredShortEvents++;
}

void SchedRemoveAllEvents(void)
{
  SchedulerRegistredEvents = 0;
}

void SchedRemoveAllShortEvents(void)
{
  SchedulerRegistredShortEvents = 0;
}

void SchedEventProcess(void)
{
  for(uint8 i = 0; i < SchedulerRegistredEvents; i++)
  {
    if(SchedulerEvent[i].eventcounter >= SchedulerEvent[i].period)
    {
      SchedulerEvent[i].eventcounter = 0;
      SchedulerEvent[i].callfunc();
    }
  }
}

void SchedPeriodIncr(void)
{
  // event period increment
  for(uint8 i = 0; i < SchedulerRegistredEvents; i++) {
    if(SchedulerEvent[i].run_flag) 
      SchedulerEvent[i].eventcounter++;
  }
  // short event processing
  for(uint8 i = 0; i < SchedulerRegistredShortEvents; i++)
  {
	SchedulerShortEvent[i].eventcounter++;
	if(SchedulerShortEvent[i].eventcounter >= SchedulerShortEvent[i].period)
	{
	  SchedulerShortEvent[i].eventcounter = 0;
	  SchedulerShortEvent[i].callfunc();
	}
  }
}

void  SchedRemoveEvent(void (*func)(void))
{
    uint8 temp_fl = 0;
    for(uint8 i = 0; i < SchedulerRegistredEvents; i++){
        if(SchedulerEvent[i].callfunc == func || temp_fl) {
            temp_fl = 1;
            SchedulerEvent[i].callfunc = SchedulerEvent[i+1].callfunc;
            SchedulerEvent[i].period = SchedulerEvent[i+1].period;
            SchedulerEvent[i].run_flag = SchedulerEvent[i+1].run_flag;
            SchedulerEvent[i].eventcounter = SchedulerEvent[i+1].eventcounter;
        }
    }
    if(temp_fl) SchedulerRegistredEvents--;
    temp_fl = 0;
    for(uint8 i = 0; i < SchedulerRegistredShortEvents; i++){
		if(SchedulerShortEvent[i].callfunc == func || temp_fl) {
			temp_fl = 1;
			SchedulerShortEvent[i].callfunc = SchedulerShortEvent[i+1].callfunc;
			SchedulerShortEvent[i].period = SchedulerShortEvent[i+1].period;
			SchedulerShortEvent[i].run_flag = SchedulerShortEvent[i+1].run_flag;
			SchedulerShortEvent[i].eventcounter = SchedulerShortEvent[i+1].eventcounter;
		}
	}
    if(temp_fl) SchedulerRegistredShortEvents--;
}

void  SchedPauseEvent(void (*func)(void))
{
    for(uint8 i = 0; i < SchedulerRegistredEvents; i ++){
        if(SchedulerEvent[i].callfunc == func){
            SchedulerEvent[i].run_flag = OFF;
        }
    }
    for(uint8 i = 0; i < SchedulerRegistredShortEvents; i ++){
		if(SchedulerShortEvent[i].callfunc == func){
			SchedulerShortEvent[i].run_flag = OFF;
		}
	}
}

void  SchedResumeEvent(void (*func)(void))
{
    for(uint8 i = 0; i < SchedulerRegistredEvents; i ++){
        if(SchedulerEvent[i].callfunc == func){
            SchedulerEvent[i].run_flag = ON;
        }
    }
    for(uint8 i = 0; i < SchedulerRegistredShortEvents; i ++){
		if(SchedulerShortEvent[i].callfunc == func){
			SchedulerShortEvent[i].run_flag = ON;
		}
	}
}

void  SchedEventSetPeriod(void (*func)(void), uint16 period)
{
    for(uint8 i = 0; i < SchedulerRegistredEvents; i ++){
        if(SchedulerEvent[i].callfunc == func){
            SchedulerEvent[i].period = period;
        }
    }
    for(uint8 i = 0; i < SchedulerRegistredShortEvents; i ++){
		if(SchedulerShortEvent[i].callfunc == func){
			SchedulerShortEvent[i].period = period;
		}
	}
}
