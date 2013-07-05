#ifndef _OS_H_  
#define _OS_H_  
/**
 * \file os.h
 * \brief A simple RTOS interface  
 *
 * \mainpage A Simple RTOS
 * This is a simple RTOS that supports pre-emptive multithreading,
 * interprocess synchronization using Events.
 * 
 * \b Note: Please don't edit the interface file "os.h".
 *
 * \author Dr. Mantis Cheng
 * \date 24 May 2013
 *
 *
 * \section assumptions GLOBAL ASSUMPTIONS 
 *
 *    (ATMEL specific)
 *   - Timer2 and SWI interrupts are reserved.
 * 
 *   - All runtime exceptions (where assumptions are violated) or other  
 *     unrecoverable errors get handled by calling OS_Abort().  
 *   - All unspecified runtime errors have undefined behaviours, 
 *     e.g., stack overflow.
 *
 *   Our RTOS scheduler timer resolution is defined by TICK. Hence, all timing parameters must
 *   be defined in multiples of TICKs. For PERIODIC tasks, its period, wcet and start time
 *   must be a multiple of TICKs. For RR tasks, its quantum is also defined in terms of TICKs.
 *
 *
 * \section policy SCHEDULING POLICY   
 *
 *   There are three scheduling levels: SYSTEM, PERIODIC and RR.
 *   These levels are prioritized with SYSTEM being the highest, and 
 *   RR being the lowest.
 *
 *   Preemption occurs immediately. Whenever preemption is feasible, it takes
 *   place instantly. As soon as a higher priority task becomes ready, it
 *   preempts all lower priority tasks.
 *
 *
 * \section system SYSTEM TASKS
 *
 *   SYSTEM (level) tasks are FCFS; they run to completion, i.e., until they
 *   terminate, block or yield. Thus, they are non-preemptible, not even by
 *   other SYSTEM tasks. They should only be used for critical system level
 *   activities, e.g., error or fault recovery. Running too many SYSTEM tasks 
 *   could affect the real time performance of all other low level tasks, 
 *   including PERIODIC tasks; therefore, use SYSTEM tasks sparingly and keep
 *   their execution time short. 
 *
 *
 * \section periodic PERIODIC TASKS
 *
 *   PERIODIC tasks are scheduled based on a per-task scheduling period,
 *   worst-case execution time (wcet) and start time. Periodic tasks are time-critical;
 *   they are \b NOT allowed to wait on any synchronization primitives, e.g.
 *   Event_Wait(). For example,
 *
 *   void p() { ... }
 *
 *   e.g., Task_Create_Periodic( p, 1, 20, 1, 2 );
 *
 *   This will create a PERIODIC task executing the function p() with a period of
 *   20 TICKs (100 msecs, or 10 Hz), a WCET of 1 TICK (5 msec) and will start after 2 TICKs
 *   at time 2 TICKs (10 msecs) after booting. WCET must be less than the period.
 *
 *   That is, after 10 msecs, this task will execute at time 10, 110, 210, 310, etc., with a
 *   maximum execution time of 5 msec during each period of 100 msec.
 *
 *   It is an \b error (exception) if this task executes more than 5 msecs during each cycle.
 * 
 *   This task may call Task_Next() before its allowed WCET. The remaining time till the
 *   end of its period is known as its "DELAYED" time.
 *
 *   It is the responsibility of the Application Engineers to ensure that no two PERIODIC
 *   tasks overlapping in execution. It is an \b error if two PERIODIC tasks become ready
 *   at the same time!!!
 *
 *   It is an \b error if a PERIODIC task executes longer than the allowed WCET.
 *   All timing violations should be caught and then reported.
 *
 *   When a PERIODIC task is preempted, its worst-case execution time will be stretched.
 *   In a similar way, when a RR task is preempted, its allowed quantum will be stretched.
 *   In theory, the RTOS should only accumulate the actual execution time; but in practice,
 *   the actual execution time could only be approximated due to the limitation of the
 *   scheduling timer resolution (1 TICK).
 *   
 *   It is an error if a PERIODIC task waits on an Event.
 *
 *
 * \section roundrobin ROUND-ROBIN TASKS
 *   
 *   RR tasks are FIFO, and they run at the lowest priority level. 
 *   If a RR task becomes ready, it will be executed only when there are no other SYSTEM 
 *   or PERIODIC tasks running. Each RR task gets 1 quantum (2 TICKs).
 *   At the end of its quantum, a RR re-enters at the end its level.
 *   However, if a RR task is preempted before its quantum expires, it re-enters at the 
 *   front of its level.
 *
 *   Only SYSTEM and RR tasks may wait on an Event. Any task can signal an Event. 
 *
 *
 * \section ipc EVENTS
 *
 *   An Event is an abstraction of an interrupt, an occurrence of an asynchronous event.
 *   Typical H/W will have an interrupt occurred bit to record when a H/W device generates
 *   an interrupt. The S/W ISR must clear the interrupt occurred bit in order to receive
 *   the next interrupt occurrence from the same device.
 *   
 *   An Event has a 1-bit memory. It can remember at most one outstanding occurrence.
 *   A task (except PERIODIC) may wait on an event "e". When another task signals "e",
 *   the waiting task will resume. A signal of event "e" may be generated before another
 *   task is waiting for it; as a result, this outstanding occurrence is recorded. When
 *   a task later waits for an event which has occurred, then this outstanding occurrence
 *   of "e" is consumed and the task continues immediately.
 *
 *   A task may wait for the "next" occurrence of an event, i.e., any outstanding occurrence
 *   is consumed. Thus, Event_Wait_Next(e) is the same as <Event_Clear(e); Event_Wait(e)>.
 *   Event_Clear(e) is a no-op if "e" hasn't occurred yet.
 *
 *   At most one task is allowed to wait on any given event. It is an error if two tasks wait
 *   on the same event.
 *
 *
 * \section clock CLOCK AND HIGH RESOLUTION TIMER  
 *    
 *   Every RTOS should maintain a "clock" for simple timing services, e.g., measuring timing
 *   interval between two time epochs. The higher the resolution the clock, the more accurate
 *   the time measurement is. An RTOS also uses a timer for scheduling decision, e.g., when to
 *   run a PERIODIC task. If the scheduler is invoked very often, e.g., every millisecond, it
 *   will create an unacceptable overhead where a high percentage of CPU time is wasted on
 *   on scheduling decision and not executing user tasks. So, typically a 5 milliseconds
 *   scheduling interval is a good compromise between overhead and response time.
 *    
 *   Now() is our system clock which gives a millisecond accuracy for timing measurement.
 *   We use Timer2 for this purpose to generate a scheduling timer interrupt every 5 milliseconds.
 *   However, Now() should provide a time resolution of 1 millisecond. By combining the total
 *   number of TICKs and the current free running counter inside a timer, we can provide a more
 *   precise high resolution timer.  
 *
 *
 * \section boot OS BOOTING
 *
 *   Our RTOS is compiled together with an application. It doesn't provide
 *   a "main()" function, which is a part of the application. By convention,
 *   the "main()" is the first function to be called by the C runtime code, 
 *   "crt0.S". For our RTOS, we shall change this convention as follows: 
 *     -# OS_Init() is called from crt0.S as the very first C function to be
 *        executed instead of main().
 *     -# Upon completion of OS_Init(), the application's main() is then
 *        created as the first and only SYSTEM level task.
 *     -# In main(), the rest of the application tasks are then created.
 *     -# In order for all other application tasks to run, our main() task
 *        must either terminate or block. (For example, main()
 *        may become a "watchdog" task to reset the entire application.)
 *
 */  
  
   
/*==================================================================  
 *             T Y P E S   &   C O N S T A N T S  
 *==================================================================  
 */  

/*================
  *    C O N S T A N T S
  *================
  */
/* limits */

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


/*================
  *    T Y P E S
  *================
  */
/** An Event descriptor
 * \sa Event_Init().
 */
typedef struct event EVENT;  


/*================
  *    G L O B A L S
  *================
  */
 
  
  
/*==================================================================  
 *             A C C E S S    P R O C E D U R E S  
 *==================================================================  
 */  

  /*=====  OS   Initialization ===== */

/** Initialize this RTOS; must be called first before "main()", i.e.,
 * it is called by "crt.S".
 * \sa \ref boot.
 */
void OS_Init(void);


/** Abort the execution of this RTOS due to an unrecoverable erorr.
 * \sa \ref assumptions. 
 */
void OS_Abort(void);  


  /*=====  Task API ===== */

 /**
   * \param f  a parameterless function to be created as a process instance
   * \param arg an integer argument to be assigned to this process instanace
   * \return 0 if not successful; otherwise non-zero.
   * \sa Task_GetArg()
   *
   *  A new process is created to execute the parameterless
   *  function \a f with an initial parameter \a arg, which is retrieved
   *  by a call to Task_GetArg().  If a new process cannot be
   *  created, 0 is returned; otherwise, it returns non-zero.
   *
   * \sa \ref policy
   */
int   Task_Create_System(void (*f)(void), int arg);
int   Task_Create_RR(    void (*f)(void), int arg);

 /**
   * \param f a parameterless function to be created as a process instance
   * \param arg an integer argument to be assigned to this process instanace
   * \param period its execution period in TICKs
   * \param wcet its worst-case execution time in TICKs, must be less than "period"
   * \param start its start time in TICKs
   * \return 0 if not successful; otherwise non-zero.
   * \sa Task_GetArg()
   *
   *  A new process is created to execute the parameterless
   *  function \a f with an initial parameter \a arg, which is retrieved
   *  by a call to Task_GetArg().  If a new process cannot be
   *  created, 0 is returned; otherwise, it returns non-zero.
   *
   * \sa \ref policy
   */
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

   
  /*=====  System Clock API ===== */
  
/**  
  * Returns the number of milliseconds since OS_Init(). Note that this number
  * wraps around after it overflows as an unsigned integer. The arithmetic
  * of 2's complement will take care of this wrap-around behaviour if you use
  * this number correctly.
  * Let  T = Now() and we want to know when Now() reaches T+1000.
  * Now() is always increasing. Even if Now() wraps around, (Now() - T) always
  * >= 0. As long as the duration of interest is less than the wrap-around time,
  * then (Now() - T >= 1000) would mean we have reached T+1000.
  * However, we cannot compare Now() against T directly due to this wrap-around
  * behaviour.
  * Now() will wrap around every 65536 milliseconds. Therefore, for measurement
  * purposes, it should be used for durations less than 65 seconds.
  */
unsigned int Now();  // number of milliseconds since the RTOS boots.

#endif /* _OS_H_ */
