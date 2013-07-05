/*
 * CSC460Proj3.c
 *
 * Created: 05/07/2013 10:38:25 AM
 *  Author: Rob & Nick
 */ 


#include <avr/io.h>
#include "os.h"




/** max. number of processes supported */  
#define MAXPROCESS		8       

/** max. number of processes supported */  
#define MAXEVENT      16       

/** time resolution */
#define TICK			    5     // resolution of system clock in milliseconds
#define QUANTUM       10    // a quantum for RR tasks

/** thread runtime stack */
#define MAXSTACK      256   // bytes

/* scheduling levels */

/** a scheduling level: system tasks with first-come-first-served policy 
 * \sa \ref system, Task_Create().
 */
#define SYSTEM    3 

/** a scheduling level: periodic tasks with predefined intervals 
 * \sa \ref periodic, Task_Create().
 */
#define PERIODIC  2 

/** A scheduling level: first-come-first-served cooperative tasks
 * \sa \ref sporadic, Task_Create(). 
 */
#define RR        1      

#ifndef NULL
#define NULL     0   /* undefined */
#endif
#define IDLE     0  

typedef struct event EVENT;  

void OS_Abort(void);  
int   Task_Create_System(void (*f)(void), int arg);
int   Task_Create_RR(    void (*f)(void), int arg);
int   Task_Create_Period(void (*f)(void), int arg, unsigned int period, unsigned int wcet, unsigned int start);

/** 
 * Terminate the calling process
 *
 *  When a process returns, i.e., it executes its last instruction in 
 *  the associated function/code, it is automatically terminated.
 */
void Task_Terminate(void);    

/** Voluntarily relinquish the processor. */
void Task_Next(void);

/** Retrieve the assigned parameter. 
  * \sa Task_Create(). 
  */
int Task_GetArg(void);          


  /*=====  Events API ===== */

/**
 * \return a non-NULL EVENT descriptor if successful; NULL otherwise.
 *
 *  Initialize a new, non-NULL Event descriptor.
 */
EVENT *Event_Init(void);

/**  
  * \param e an Event descriptor
  *
  * clear any outstanding occurrence of event "e" 
  */
void Event_Clear( EVENT *e );  

/**  
  * \param e an Event descriptor
  *
  * If the event "e" hasn't occurred, then the calling task is blocked;
  * otherwise, the event is consumed and the calling task continues.
  */
void Event_Wait( EVENT *e );  

/**  
  * \param e an Event descriptor
  *
  * If the event "e" hasn't occurred, then the calling task is blocked;
  * otherwise, the event is consumed and the calling task waits for the
  * next occurrence of "e".
  */
void Event_Wait_Next( EVENT *e );  

/**  
  * \param e an Event descriptor
  *
  * records an event of type "e" has occurred; if a task is waiting on
  * this event, then it will be resumed.
  * At most one occurrence of an event (just 1 bit) is recorded.
  * When a task is resumed as a result of a signal on an event, the event is
  * considered to have been consumed.
  * When an event is consumed, it is automatically reset.
  */
void Event_Signal( EVENT *e );

/**  
  * \param e an Event descriptor
  *
  * This is the same as Event_Signal(e) except that it is called by
  * by an ISR (interrupt handler). 
  * For the time being, you may treat Event_Async_Signal() and Event_Siganl()
  * the same. However, when you understand how to optimize the implementation
  * when the caller is indeed an ISR, then you can change it.
  * (This is a BONUS challenge!!!)
  */
void Event_Async_Signal( EVENT *e );
unsigned int Now();  // number of milliseconds since the RTOS boots.

int OS_Init(void)
{
    
}