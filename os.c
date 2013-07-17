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

/*
 * Context switching
 */
/**
 * @brief Push all the registers and SREG onto the stack.
 */
#define    SAVE_CTX()    SAVE_CTX_TOP();SAVE_CTX_BOTTOM();
/**
 * It is important to keep the order of context saving and restoring exactly
 * in reverse. Also, when a new task is created, it is important to
 * initialize its "initial" context in the same order as a saved context.
 *
 * Save r31 and SREG on stack, disable interrupts, then save
 * the rest of the registers on the stack. In the locations this macro
 * is used, the interrupts need to be disabled, or they already are disabled.
 */

#define    SAVE_CTX_TOP()       asm volatile (\
    "push   r31             \n\t"\
    "in     r31,__SREG__    \n\t"\
    "cli                    \n\t"::); /* Disable interrupt */

#define STACK_SREG_SET_I_BIT()    asm volatile (\
    "ori    r31, 0x80        \n\t"::);

#define    SAVE_CTX_BOTTOM()       asm volatile (\
    "push   r31             \n\t"\
    "push   r30             \n\t"\
    "push   r29             \n\t"\
    "push   r28             \n\t"\
    "push   r27             \n\t"\
    "push   r26             \n\t"\
    "push   r25             \n\t"\
    "push   r24             \n\t"\
    "push   r23             \n\t"\
    "push   r22             \n\t"\
    "push   r21             \n\t"\
    "push   r20             \n\t"\
    "push   r19             \n\t"\
    "push   r18             \n\t"\
    "push   r17             \n\t"\
    "push   r16             \n\t"\
    "push   r15             \n\t"\
    "push   r14             \n\t"\
    "push   r13             \n\t"\
    "push   r12             \n\t"\
    "push   r11             \n\t"\
    "push   r10             \n\t"\
    "push   r9              \n\t"\
    "push   r8              \n\t"\
    "push   r7              \n\t"\
    "push   r6              \n\t"\
    "push   r5              \n\t"\
    "push   r4              \n\t"\
    "push   r3              \n\t"\
    "push   r2              \n\t"\
    "push   r1              \n\t"\
    "push   r0              \n\t"::);

/**
 * @brief Push all the registers and SREG onto the stack.
 */
#define    SAVE_CTX()    SAVE_CTX_TOP();SAVE_CTX_BOTTOM();

/**
 * @brief Pop all registers and the status register.
 */
#define    RESTORE_CTX()    asm volatile (\
    "pop    r0              \n\t"\
    "pop    r1              \n\t"\
    "pop    r2              \n\t"\
    "pop    r3              \n\t"\
    "pop    r4              \n\t"\
    "pop    r5              \n\t"\
    "pop    r6              \n\t"\
    "pop    r7              \n\t"\
    "pop    r8              \n\t"\
    "pop    r9              \n\t"\
    "pop    r10             \n\t"\
    "pop    r11             \n\t"\
    "pop    r12             \n\t"\
    "pop    r13             \n\t"\
    "pop    r14             \n\t"\
    "pop    r15             \n\t"\
    "pop    r16             \n\t"\
    "pop    r17             \n\t"\
    "pop    r18             \n\t"\
    "pop    r19             \n\t"\
    "pop    r20             \n\t"\
    "pop    r21             \n\t"\
    "pop    r22             \n\t"\
    "pop    r23             \n\t"\
    "pop    r24             \n\t"\
    "pop    r25             \n\t"\
    "pop    r26             \n\t"\
    "pop    r27             \n\t"\
    "pop    r28             \n\t"\
    "pop    r29             \n\t"\
    "pop    r30             \n\t"\
    "pop    r31             \n\t"\
	"out    __SREG__, r31    \n\t"\
    "pop    r31             \n\t"::);

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
	//TODO maybe just use unsigned int? (params to task create periodic)
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
static unsigned int cur_ticks = 0;

/**
 * @brief This is the set of kernel requests, i.e., a request code for each system call.
 * Uncomment when needed
 */
typedef enum {
	NONE = 0,
	TIMER_EXPIRED,
	TASK_CREATE,
	//TASK_TERMINATE,
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
static void enterKernel(void) __attribute((noinline, naked));
static void exitKernel(void)  __attribute((noinline, naked));
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
static Task_t* const idleTask = &Tasks[MAXPROCESS];
static volatile uint16_t kernelStackPointer;
static volatile kernelRequest_t kernelRequest;
static volatile uint8_t kernelNewTaskLevel = 0;
//one for each RR, PERIODIC, and SYSTEM 
//RR, PERIODIC and SYSTEM, are defined to 1-3 respectively
//having 4 of these allows nice indexing like readyQueue[RR], without needing -1 on every index
//TODO may not need queue for system
static taskQueue_t readyQueue[4]; 

static void enterKernel(void) {
	/*
	 * The PC was pushed on the stack with the call to this function.
	 * Now push on the I/O registers and the SREG as well.
	 */
	SAVE_CTX();

	/*
	 * The last piece of the context is the SP. Save it to a variable.
	 */
	currentTask->stackPointer = (uint8_t*) SP;

	/*
	 * Now restore the kernel's context, SP first.
	 */
	SP = kernelStackPointer;

	/*
	 * Now restore I/O and SREG registers.
	 */
	RESTORE_CTX();

	/*
	 * return explicitly required as we are "naked".
	 *
	 * The last piece of the context, the PC, is popped off the stack
	 * with the ret instruction.
	 */
	asm volatile ("ret\n"::);
}


/**
 * @fn exitKernel
 *
 * @brief The actual context switching code begins here.
 *
 * This function is called by the kernel. Upon entry, we are using
 * the kernel stack, on top of which is the address of the instruction
 * after the call to exitKernel().
 *
 * Assumption: Our kernel is executed with interrupts already disabled.
 *
 * The "naked" attribute prevents the compiler from adding instructions
 * to save and restore register values. It also prevents an
 * automatic return instruction.
 */
static void exitKernel(void) {
	/*
	 * The PC was pushed on the stack with the call to this function.
	 * Now push on the I/O registers and the SREG as well.
	 */
	SAVE_CTX();

	/*
	 * The last piece of the context is the SP. Save it to a variable.
	 */
	kernelStackPointer = SP;

	/*
	 * Now restore the task's context, SP first.
	 */
	SP = (uint16_t) (currentTask->stackPointer);

	/*
	 * Now restore I/O and SREG registers.
	 */
	RESTORE_CTX();

	/*
	 * return explicitly required as we are "naked".
	 * Interrupts are enabled or disabled according to SREG
	 * recovered from stack, so we don't want to explicitly
	 * enable them here.
	 *
	 * The last piece of the context, the PC, is popped off the stack
	 * with the ret instruction.
	 */
	asm volatile ("ret\n"::);
}

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
	uint8_t *stackBottom;
	stackBottom       = &(task->stack[MAXSTACK - 1]);
	uint8_t *stackTop = stackBottom - (32 + 1 + 2 + 2);
	stackTop[2]       = (uint8_t) 0; //register r1 is 0
	/* stackTop[31] is r30. */
	stackTop[32]      = (uint8_t) _BV(SREG_I); /* set SREG_I bit in stored SREG. */
	/* stackTop[33] is r31. */
	
	/* We are placing the address (16-bit) of the functions
	 * onto the stack in reverse byte order (least significant first, followed
	 * by most significant).  This is because the "return" assembly instructions
	 * (ret and reti) pop addresses off in BIG ENDIAN (most sig. first, least sig.
	 * second), even though the AT90 is LITTLE ENDIAN machine.
	 */
	stackTop[34] = (uint8_t) ((uint16_t) f >> 8);
	stackTop[35] = (uint8_t) ((uint16_t)  f);
	stackTop[36] = (uint8_t) ((uint16_t) Task_Terminate >> 8);
	stackTop[37] = (uint8_t) (uint16_t)  Task_Terminate;
	task->stackPointer = stackTop;
}

int   Task_Create_System(void (*f)(void), int arg)
{
	uint8_t sreg;
	sreg = SREG;
	cli();
	
	if (NUM_TASKS_IN_USE >= MAXPROCESS + 1) return -1; //error to many procs
	Task_t * task = queuePop(&readyQueue[0]);
	Task_Create_Common(f, arg, SYSTEM, task);
	queuePush(&readyQueue[SYSTEM], task);
	kernelNewTaskLevel = SYSTEM;
	
	enterKernel();
	
	SREG = sreg;
	return 0;
}

int   Task_Create_RR(void (*f)(void), int arg)
{
	uint8_t sreg;
	sreg = SREG;
	cli();
	
	if (NUM_TASKS_IN_USE >= MAXPROCESS + 1) return -1; //error to many procs
	Task_t * task = queuePop(&readyQueue[0]);
	Task_Create_Common(f, arg, RR, task);
	queuePush(&readyQueue[RR], task);
	kernelNewTaskLevel = RR;
	
	enterKernel();
	
	SREG = sreg;
	return 0;
}

int   Task_Create_Period(void (*f)(void), int arg, unsigned int period, unsigned int wcet, unsigned int start)
{
	uint8_t sreg;
	sreg = SREG;
	cli();
	
	if (NUM_TASKS_IN_USE >= MAXPROCESS + 1) return -1; //error to many procs
	Task_t * task = queuePop(&readyQueue[0]);
	Task_Create_Common(f, arg, PERIODIC, task);
	task->wcet = wcet;
	task->ticksUntilReady = start;
	queueAddSortedByTicksUntilReady(&readyQueue[PERIODIC], task); 
	kernelNewTaskLevel = PERIODIC;
	
	enterKernel();
	
	SREG = sreg;
	return 0;
}

void  Task_Terminate(void) 
{
	uint8_t volatile sreg;
	
	sreg = SREG;
	cli();
	
	//TODO probably need to context switch and reschedule
	queuePush(&readyQueue[0], currentTask);
	
	SREG = sreg;
}

void  Task_Next(void)
{
	uint8_t volatile sreg;
	
	sreg = SREG;
	cli();
	
	kernelRequest = TASK_NEXT;
	enterKernel();
	
	SREG = sreg;
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
		exitKernel();
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
	currentTask->state = RUNNING;
	kernelRequest = NONE;

	kernelMainLoop();
}

int main(void)
{
	SP = 0x4000;
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
	/*
	 * Save the interrupted task's context on its stack,
	 * and save the stack pointer.
	 *
	 * On the cur_task's stack, the registers and SREG are
	 * saved in the right order, but we have to modify the stored value
	 * of SREG. We know it should have interrupts enabled because this
	 * ISR was able to execute, but it has interrupts disabled because
	 * it was stored while this ISR was executing. So we set the bit (I = bit 7)
	 * in the stored value.
	 */
	SAVE_CTX_TOP();

	STACK_SREG_SET_I_BIT();

	SAVE_CTX_BOTTOM();

	currentTask->stackPointer = (uint8_t*) SP;

	/*
	 * Now that we already saved a copy of the stack pointer
	 * for every context including the kernel, we can move to
	 * the kernel stack and use it. We will restore it again later.
	 */
	SP = kernelStackPointer;

	/*
	 * Inform the kernel that this task was interrupted.
	 */
	//kernel_request = TIMER_EXPIRED;
	 cur_ticks++;
	/*
	 * Prepare for next tick interrupt.
	 */
	OCR2A += TICK_CYCLES;

	kernelRequest = TIMER_EXPIRED;

	/*
	 * Restore the kernel context. (The stack pointer is restored again.)
	 */
	SP = kernelStackPointer;

	/*
	 * Now restore I/O and SREG registers.
	 */
	RESTORE_CTX();

	/*
	 * We use "ret" here, not "reti", because we do not want to
	 * enable interrupts inside the kernel.
	 * Explicitly required as we are "naked".
	 *
	 * The last piece of the context, the PC, is popped off the stack
	 * with the ret instruction.
	 */
	asm volatile ("ret\n"::);
}
