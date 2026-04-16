
#ifndef SCHEDULER_H
#define	SCHEDULER_H

#include "common.h"

// the event can be short, which will be executed directly in the interrupt
// or usual, which will be executed in cycle of program when function "SchedEventProcess" will be called

#define      MAX_EVENT        30     // max quantity of usual events
#define      MAX_SHORTEVENT   30     // max quantity of short events
#define      ON               1
#define      OFF              0


typedef struct {
  void     (*callfunc)(void);
  uint16   period;
  uint16   eventcounter;
  uint8    run_flag;
} tEvent;

typedef struct {
  void     (*callfunc)(void);
  uint16   period;
  uint16   eventcounter;
  uint8    run_flag;
} tSEvent;

extern tEvent SchedulerEvent[MAX_EVENT];
extern tSEvent SchedulerShortEvent[MAX_SHORTEVENT];

void  SchedPeriodIncr(void);
uint8 SchedAddEvent(void (*)(void), uint16);
uint8 SchedAddShortEvent(void (*)(void), uint16);
void  SchedRemoveEvent(void (*)(void));
void  SchedRemoveAllEvents(void);
void  SchedRemoveAllShortEvents(void);
void  SchedPauseEvent(void (*)(void));
void  SchedResumeEvent(void (*)(void));
void  SchedEventProcess(void);
void  SchedEventSetPeriod(void (*)(void), uint16);

#endif	

