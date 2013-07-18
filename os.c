/*
 * CSC460Proj3.c
 *
 * Created: 05/07/2013 10:38:25 AM
 *  Author: Rob & Nick
 *  Based off of Scott Craig and Justin Tanner's OS, provided and edited by Neil MacMillan
 */ 

#define F_CPU 11059200
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>

#include "os.h"
#include "error_code.h"

/** The number of clock cycles in one "tick" or 5 ms */
#define TICK_CYCLES     (F_CPU / 1000 * TICK)

#define LED_DDR DDRB
#define LED_PORT PORTB

/** LEDs for OS_Abort() */
#define LED_RED_MASK    _BV(PB1)	// LED2 on xplained

/** LEDs for OS_Abort() */
#define LED_GREEN_MASK    _BV(PB2)	// LED3 on xplained
/** Error message used in OS_Abort() */
static uint8_t volatile error_msg = ERR_RUN_1_USER_CALLED_OS_ABORT;

#define Disable_Interrupt()		asm volatile ("cli"::)
#define Enable_Interrupt()		asm volatile ("sei"::)
extern void Enter_Kernel();
extern void Exit_Kernel(); 
/**
 * @brief This is the set of states that a task can be in at any given time.
 */
typedef enum {
	DEAD = 0, RUNNING, READY, WAITING, SLEEPING
} taskState_t;

typedef struct Task_Struct Task_t;

struct Task_Struct 
{
	void (* function)(void);
	int argument;
	uint8_t stack[MAXSTACK];
	uint8_t* volatile stackPointer;
	uint8_t level;
	taskState_t state;
	uint8_t ticksTaken;
	//periodic only
	uint16_t wcet; //worst case execution time
	uint16_t period;
	uint16_t offset;
	uint16_t ticksUntilReady;
	//intrusive linked list
	Task_t * next;
	Task_t * prev;
};

/**
 * @brief Contains all information of a given task.
 */
struct event {
	unsigned flag:1;
	Task_t * waiting_task;
};

static uint8_t num_events_created = 0;
static EVENT event_list[MAXEVENT];
static volatile unsigned int cur_ticks = 0;

/**
 * @brief This is the set of kernel requests, i.e., a request code for each system call.
 * Uncomment when needed
 */
typedef enum {
	NONE = 0,
	TIMER_EXPIRED,
	TASK_CREATE,
	TASK_TERMINATE,
	TASK_NEXT,
	//TASK_GET_ARG,
	//TASK_SLEEP,
	//EVENT_INIT,
	//EVENT_WAIT,
	//EVENT_SIGNAL,
	//EVENT_BROADCAST,
	//EVENT_SIGNAL_AND_NEXT,
	//EVENT_BROADCAST_AND_NEXT,
	KERNEL_REQUEST_COUNT
} kernelRequest_t;

/**
 * @brief Contains pointers to head and tail of a linked list.
 */
typedef struct {
	/** The first item in the queue. NULL if the queue is empty. */
	Task_t * head;
	/** The last item in the queue. Undefined if the queue is empty. */
	Task_t * tail;
} taskQueue_t;


//forward declerations
//static void enterKernel(void) __attribute((noinline, naked));
//static void exitKernel(void)  __attribute((noinline, naked));
void TIMER2_COMPA_vect(void) __attribute__ ((signal, naked));

void queueInit(taskQueue_t * q)
{
		q->head = q->tail = NULL;
}

void queuePush(taskQueue_t * q, Task_t * task)
{
	if(q->head == NULL) {
		q->head = q->tail = task;
	} else {
		q->tail->next = task;
		task->prev = q->tail;
		q->tail = task;
	}
}

void queueAddFront(taskQueue_t * q, Task_t * task)
{
	if(q->head == NULL) {
		q->head = q->tail = task;
    } else {
		q->head->prev = task;
		task->next = q->head;
		q->head = task;
	}
}

Task_t * queuePop(taskQueue_t * q)
{
	if (q->head == NULL){OS_Abort();}
	Task_t * temp = q->head;
	if (q->head == q->tail)
	{
		q->head->next = q->head->prev = NULL;
		q->head = q->tail = NULL;
	} 
	else
	{
		q->head->next->prev = NULL;
		q->head = q->head->next;
		temp->prev = temp->next = NULL;
	}
	return temp;
}

void queueAddSortedByTicksUntilReady(taskQueue_t * q, Task_t * task)
{
	if (q->head == NULL)
	{
		q->head = q->tail = task;
		return;
	} else if (q->head == q->tail)
	{
		if(q->head->ticksUntilReady > task->ticksUntilReady) {
			queueAddFront(q,task);
		} else {
			queuePush(q,task);
		}
	}
	for (Task_t * ptr = q->tail; ptr != q->head; ptr = ptr->prev)
	{
		if (ptr->prev->ticksUntilReady > task->ticksUntilReady)
		{
			task->prev = ptr->prev->prev;
			task->next = ptr->prev;
			ptr->prev->prev = task;
			ptr->prev->next = ptr;		
		}
	}
}

//Kernel Globals
static Task_t  Tasks[MAXPROCESS + 1]; //extra space for idle
static uint8_t NUM_TASKS_IN_USE = 0; 
static Task_t* currentTask = NULL;
volatile uint8_t* currentTaskStackPointer;
volatile uint8_t* kernelStackPointer;
static Task_t* const idleTask = &Tasks[MAXPROCESS];
static volatile kernelRequest_t kernelRequest;
static volatile uint8_t kernelNewTaskLevel = 0;
//one for each RR, PERIODIC, and SYSTEM 
//RR, PERIODIC and SYSTEM, are defined to 1-3 respectively
//having 4 of these allows nice indexing like readyQueue[RR], without needing -1 on every index
//TODO may not need queue for system
static taskQueue_t readyQueue[4]; 

void OS_Abort(void) {
	uint8_t i, j;
	uint8_t flashes, mask;

	cli();

	/* Initialize port for output */
	LED_DDR = LED_RED_MASK | LED_GREEN_MASK;

	if (error_msg < ERR_RUN_1_USER_CALLED_OS_ABORT) {
		flashes = error_msg + 1;
		mask = LED_GREEN_MASK;
	} else {
		flashes = error_msg + 1 - ERR_RUN_1_USER_CALLED_OS_ABORT;
		mask = LED_RED_MASK;
	}

	for (;;) {
		LED_PORT = (uint8_t) (LED_RED_MASK| LED_GREEN_MASK);

		for (i = 0; i < 100; ++i) {
			_delay_ms(25);
		}

		LED_PORT = (uint8_t) 0;

		for (i = 0; i < 40; ++i) {
			_delay_ms(25);
		}

		for (j = 0; j < flashes; ++j) {
			LED_PORT = mask;

			for (i = 0; i < 10; ++i) {
				_delay_ms(25);
			}

			LED_PORT = (uint8_t) 0;

			for (i = 0; i < 10; ++i) {
				_delay_ms(25);
			}
		}

		for (i = 0; i < 20; ++i) {
			_delay_ms(25);
		}
	}
}

static void Task_Create_Common(void (*f)(void), int arg, uint8_t level, Task_t * task)
{
	NUM_TASKS_IN_USE++;
	task->function   = f;
	task->argument   = arg;
	task->level      = level;
	task->wcet       = 0;
	task->state      = READY;
	task->ticksTaken = 0;
	kernelRequest    = TASK_CREATE;
	/* The stack grows down in memory, so the stack pointer is going to end up
	 * pointing to the location 32 + 1 + 2 + 2 = 37 bytes above the bottom, to make
	 * room for (from bottom to top):
	 *   the address of Task_Terminate() to destroy the task if it ever returns,
	 *   the address of the start of the task to "return" to the first time it runs,
	 *   register 31,
	 *   the stored SREG, and
	 *   registers 30 to 0.
	 */
	uint8_t *sp;
	sp       = (uint8_t *) &(task->stack[MAXSTACK - 1]);
	//Clear the contents of the workspace
	memset(&(task->stack),0,MAXSTACK);
	
	//Store terminate at the bottom of stack to protect against stack underrun.
	*(unsigned char *)sp-- = ((unsigned int)Task_Terminate) & 0xff;
	*(unsigned char *)sp-- = (((unsigned int)Task_Terminate) >> 8) & 0xff;

	//Place return address of function at bottom of stack
	*(unsigned char *)sp-- = ((unsigned int)f) & 0xff;
	*(unsigned char *)sp-- = (((unsigned int)f) >> 8) & 0xff;
	sp = sp - 33;
	task->stackPointer = sp;
}

int   Task_Create_System(void (*f)(void), int arg)
{
	Disable_Interrupt();
	
	if (NUM_TASKS_IN_USE >= MAXPROCESS + 1) return -1; //error to many procs
	Task_t * task = queuePop(&readyQueue[0]);
	Task_Create_Common(f, arg, SYSTEM, task);
	queuePush(&readyQueue[SYSTEM], task);
	kernelNewTaskLevel = SYSTEM;
	
	Enter_Kernel();
	
	return 0;
}

int   Task_Create_RR(void (*f)(void), int arg)
{
	Disable_Interrupt();
	
	if (NUM_TASKS_IN_USE >= MAXPROCESS + 1) return -1; //error to many procs
	Task_t * task = queuePop(&readyQueue[0]);
	Task_Create_Common(f, arg, RR, task);
	queuePush(&readyQueue[RR], task);
	kernelNewTaskLevel = RR;
	
	Enter_Kernel();

	return 0;
}

int   Task_Create_Period(void (*f)(void), int arg, unsigned int period, unsigned int wcet, unsigned int start)
{
	Disable_Interrupt();
	
	if (NUM_TASKS_IN_USE >= MAXPROCESS + 1) return -1; //error to many procs
	Task_t * task = queuePop(&readyQueue[0]);
	Task_Create_Common(f, arg, PERIODIC, task);
	task->wcet = wcet;
	task->ticksUntilReady = start;
	queueAddSortedByTicksUntilReady(&readyQueue[PERIODIC], task); 
	kernelNewTaskLevel = PERIODIC;
	
	Enter_Kernel();
	
	return 0;
}

void  Task_Terminate(void) 
{
	Disable_Interrupt();
	kernelRequest = TASK_TERMINATE;
	Enter_Kernel();
	//if we return from enter kernel we're screwed
	//and will exceute random code
	OS_Abort();
}

void  Task_Next(void)
{
	Disable_Interrupt();
	
	kernelRequest = TASK_NEXT;
	Enter_Kernel();
}

int   Task_GetArg(void)          
{
	int arg;
	uint8_t sreg;

	sreg = SREG;
	cli();

	arg = currentTask->argument;

	SREG = sreg;

	return arg;
} 

/* begin event section */

/* 
generate an EVENT pointer 
(which is really just an array index number) 
*/
EVENT *Event_Init(void) {
	EVENT* event_ptr = NULL;
	uint8_t sreg;

	sreg = SREG;
	cli();

	if (num_events_created < MAXEVENT) {
		event_ptr = (EVENT *) (uint16_t) (num_events_created+1);
	} else {
		OS_Abort();
	}
	
	SREG = sreg;
	
	return event_ptr;

}

/* 
clears occurances of event e 
*/
void  Event_Clear( EVENT *e ) {
	event_list[(uint16_t)e].flag = 0;
}  

/* 
checks flag, 
if 1, resumes, 
if 0, waits for next event
POTENTIAL BUG: code isn't cli'd 
*/
void  Event_Wait( EVENT *e ) {
	uint8_t sreg;
	sreg = SREG;
	cli();
	
	if (currentTask->level == PERIODIC) OS_Abort();
	/*abort if a periodic task tries to wait*/
	if (event_list[(uint16_t)e].flag == 1){ 
		event_list[(uint16_t)e].flag = 0; 
	} else { /* if no events have arrived */
		if (event_list[(uint16_t)e].waiting_task != NULL) OS_Abort();
		/* only one task can wait per event */
		queuePop(&readyQueue[currentTask->level]);
		event_list[(uint16_t)e].waiting_task = currentTask;
		currentTask->state = SLEEPING;
	}
	
	SREG = sreg;
}

/*
readies a waiting task
gets signalled by an event, takes the task off the event list
adds it to the front of the active queue
POTENTIAL BUG: make sure if this task is added to the front
and the current task becomes #2, it still works properly
and make sure this added task doesnt preempt a current task if 
they have the same priority
*/
void taskToReady(EVENT* e) {
	if (event_list[(uint16_t)e].waiting_task==NULL) OS_Abort();
	queueAddFront(&readyQueue[currentTask->level], event_list[(uint16_t)e].waiting_task);
	event_list[(uint16_t)e].waiting_task = NULL;
	currentTask->state = READY;
}

/* 
clears flag, waits for next event 
*/
void  Event_Wait_Next( EVENT *e ) {
	event_list[(uint16_t)e].flag = 0;
	if (event_list[(uint16_t)e].waiting_task != NULL) OS_Abort();
	queuePop(&readyQueue[currentTask->level]);
	event_list[(uint16_t)e].waiting_task = currentTask;
}

/* 
signals waiting task, or, if there are none, sets the flag
 */
void  Event_Signal( EVENT *e ) {
	if (event_list[(uint16_t)e].waiting_task!=NULL) {
		taskToReady(e);
	}
	else event_list[(uint16_t)e].flag = 1;

}

/* we do nothing with this yes, requires ISR, will do later */
void  Event_Async_Signal( EVENT *e ) {
	//basically the same as eventstatic EVENT event_list[MAXEVENT];_signal, but for a bonus and uses ISR
}


/* end event section */

unsigned int Now() {
	uint8_t sreg;
	unsigned int i = 0;
	sreg = SREG;
	cli();
	i = TCNT2;
	SREG = sreg;
	i = (i/11059) + cur_ticks;
	return i;
	
}

void Reset_Now(unsigned int global_ticks) {
	cur_ticks = global_ticks;
}


/**
 *  @brief The idle task does nothing but busy loop.
 */
static void idle(void) 
{
	for (;;) {
	};
}

static void kernelSchedule(void)
{
	if (currentTask->state != RUNNING || currentTask == idleTask)
	{
		if (readyQueue[SYSTEM].head != NULL)
		{
			currentTask = queuePop(&readyQueue[SYSTEM]);
		}
		else if (readyQueue[PERIODIC].head != NULL && readyQueue[PERIODIC].head->ticksUntilReady <= 0)
		{
			currentTask = queuePop(&readyQueue[PERIODIC]);
		}
		else if (readyQueue[RR].head != NULL)
		{
			currentTask = queuePop(&readyQueue[RR]);
		}
		else
		{
			currentTask = idleTask;
		}
		currentTaskStackPointer = currentTask->stackPointer;
		currentTask->state = RUNNING;
	}
}

static void kernelUpdate(void)
{
	switch (kernelRequest) {
	case TASK_NEXT:
		currentTask->ticksUntilReady = currentTask->wcet;
		queuePush(&readyQueue[currentTask->level], currentTask);
		currentTask->state = READY;
		currentTask->ticksTaken = 0;
	break;
	case TIMER_EXPIRED:
		for (Task_t *p = readyQueue[PERIODIC].head; p != NULL && p != readyQueue[PERIODIC].tail; p = p->next)
		{
			p->ticksUntilReady--;
		}
		currentTask->ticksTaken++;
		//TODO these should have more granularity with Now
		if (currentTask->level == PERIODIC && currentTask->ticksTaken > currentTask->wcet )
		{
			OS_Abort();
		} else if (currentTask->level == RR && currentTask->ticksTaken*5 >= QUANTUM)
		{
			currentTask->state = READY;
			currentTask->ticksTaken = 0;
			queuePush(&readyQueue[currentTask->level], currentTask);
		}
	break;
	case TASK_CREATE:
		if (kernelNewTaskLevel > currentTask->level)
		{
			//might get preempted
			//TODO do stuff with now
			//round robin being preempted, add it to the front of the ready queue
			if (currentTask->level == RR)
			{
				currentTask->state = READY;
				queueAddFront(&readyQueue[RR], currentTask);
			}
		}
	break;
	case TASK_TERMINATE:
		currentTask->state = DEAD;
		queuePush(&readyQueue[0], currentTask);
	break;
	case NONE:
	break;
	case KERNEL_REQUEST_COUNT:
		OS_Abort();
	break;
	}
	kernelRequest = NONE;
}

static void kernelMainLoop(void) 
{
	for (;;) {
		kernelUpdate();
		kernelSchedule();
		Exit_Kernel();
	}
}

extern void r_main(void);

void OS_Init(void)
{
	for (int i = 0; i<= SYSTEM; i++)
	{
		queueInit(&readyQueue[i]);
	}
		
	//setup timers
	TCCR2B &= ~(_BV(CS22)| _BV(CS21));
	TCCR2B |= (_BV(CS20));		// start the timer at fclk/1
	
	//push all descriptors onto ready to use descriptor queue
	for (int i = 0; i < MAXPROCESS; i++)
	{
		Tasks[i].state = DEAD;
		queuePush(&readyQueue[0], &Tasks[i]);
	}
    //create idle task
	Tasks[MAXPROCESS].state = READY;
	Task_Create_Common(idle, 0, 0, &Tasks[MAXPROCESS]);
	
	/* Set up Timer 2 Output Compare interrupt,the TICK clock. */
	TIMSK2 |= _BV(OCIE2A);
	
	OCR2A = TCNT2 + TICK_CYCLES;
	/* Clear flag. */
	TIFR2 = _BV(OCF2A);
	
	LED_DDR = 0xFF;
	
	/* Initialize the now counter */
	cur_ticks = 0;
	
	//Special case do not want to enter kernel on first creation
	//Task_Create_System(r_main, 0);
	Task_t * task = queuePop(&readyQueue[0]);
	Task_Create_Common(r_main, 0, SYSTEM, task);
	queuePush(&readyQueue[SYSTEM], task);
	
	currentTask = idleTask;
	currentTaskStackPointer = currentTask->stackPointer;
	currentTask->state = RUNNING;
	kernelRequest = NONE;

	kernelMainLoop();
}

int main(void)
{
	OS_Init();
	for(;;);
	return 0;
}
/**
 * @fn TIMER2_COMPA_vect
 *
 * @brief The interrupt handler for output compare interrupts on Timer 2
 *
 * Used to enter the kernel when a tick expires.
 *
 * Assumption: We are still executing on the cur_task stack.
 * The return address inside the current task code is on the top of the stack.
 *
 * The "naked" attribute prevents the compiler from adding instructions
 * to save and restore register values. It also prevents an
 * automatic return instruction.
 */
void TIMER2_COMPA_vect(void)
{
	cur_ticks++;
	OCR2A += TICK_CYCLES;
	kernelRequest = TIMER_EXPIRED;

	Enter_Kernel();
	
	asm volatile ("ret\n"::);
}
