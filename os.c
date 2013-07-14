/*
 * CSC460Proj3.c
 *
 * Created: 05/07/2013 10:38:25 AM
 *  Author: Rob & Nick
 *  Based off of Scott Craig and Justin Tanner's OS, provided and edited by Neil MacMillan
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>

#include "os.h"

/** The number of clock cycles in one "tick" or 5 ms */
#define TICK_CYCLES     (F_CPU / 1000 * TICK)

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
 * @brief Contains all information of a given task.
 */

static uint8_t num_events_created = 0;
static EVENT event_list[MAXEVENT];
static unsigned int cur_ticks = 0;
static unsigned int inittime = 0;
static unsigned int endtime = 0;

typedef struct  Event_Struct {
	unsigned flag:1;
	Task_t waiting_task;
} EVENT;

/**
 * @brief This is the set of states that a task can be in at any given time.
 */
typedef enum {
	DEAD = 0, RUNNING, READY, WAITING, SLEEPING
} taskState_t;

typedef struct Task_Struct 
{
	void (* function)(void) = NULL;
	int argument;
	uint8_t stack[MAXSTACK];
	uint8_t volatile * stackPointer = NULL;
	uint8_t level;
	taskState_t state;
	//periodic only
	//TODO maybe just use unsigned int? (params to task create periodic)
	uint16_t wcet; //worst case execution time
	uint16_t period;
	uint16_t offset;
	uint16_t ticksUntilReady;
	//intrusive linked list
	Task_t * next;
	Task_t * prev;
} Task_t;

/**
 * @brief This is the set of kernel requests, i.e., a request code for each system call.
 */
typedef enum {
	NONE = 0,
	TIMER_EXPIRED,
	TASK_CREATE,
	TASK_TERMINATE,
	TASK_NEXT,
	TASK_GET_ARG,
	TASK_SLEEP,
	EVENT_INIT,
	EVENT_WAIT,
	EVENT_SIGNAL,
	EVENT_BROADCAST,
	EVENT_SIGNAL_AND_NEXT,
	EVENT_BROADCAST_AND_NEXT
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
	if(q->head == SNULL) {
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
	if (q->head = NULL)
	{
		q->head = q->tail = task;
		return;
	} else if (q->head == q->tail)
	{
		if(q->head->ticksUntilReady > task->ticksUntilReady) {
			queueAddFront(q,task);
		} else {
			queuePush(q,task)
		}
	}
	for (Task_t * ptr = q->tail; ptr != q->head; ptr = ptr->prev)
	{
		if (ptr->prev->ticksUntilReady > task->ticksUntilReady)
		{
			task->prev = ptr->prev->prev;
			task->next = ptr->prev
			ptr->prev->prev = task;
			ptr->prev->next = ptr;		
		}
	}
}

//Kernel Globals
static volatile Task_t  Tasks[MAXPROCESS + 1]; //extra space for idle
static volatile uint8_t NUM_TASKS_IN_USE = 0; 
static volatile Task_t* currentTask = NULL;
static volatile Task_t* const idleTask = &Tasks[MAXPROCESS];
static volatile uint16_t kernelStackPointer;
static volatile kernelRequest_t kernelRequest;
//one for each RR, PERIODIC, and SYSTEM 
//RR, PERIODIC and SYSTEM, are defined to 1-3 respectively
//having 4 of these allows nice indexing like readyQueue[RR], without needing -1 on every index
//TODO may not need queue for system
static volatile taskQueue_t readyQueue[4]; 

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

int Task_Create_Common(void (*f)(void), int arg, uint8_t level, Task_t * task)
{
	task->function = f;
	task->argument = arg;
	task->level    = level;
	task->wcet     = 0;
	task->state    = READY;
	/* The stack grows down in memory, so the stack pointer is going to end up
	 * pointing to the location 32 + 1 + 2 + 2 = 37 bytes above the bottom, to make
	 * room for (from bottom to top):
	 *   the address of Task_Terminate() to destroy the task if it ever returns,
	 *   the address of the start of the task to "return" to the first time it runs,
	 *   register 31,
	 *   the stored SREG, and
	 *   registers 30 to 0.
	 */
	uint8_t *stackTop;
	stackTop     = task->stack[0];
	stackTop[2]  = (uint8_t) 0; //register r1 is 0
	/* stackTop[31] is r30. */
	stackTop[32] = (uint8_t) _BV(SREG_I); /* set SREG_I bit in stored SREG. */
	/* stackTop[33] is r31. */
	
	/* We are placing the address (16-bit) of the functions
	 * onto the stack in reverse byte order (least significant first, followed
	 * by most significant).  This is because the "return" assembly instructions
	 * (ret and reti) pop addresses off in BIG ENDIAN (most sig. first, least sig.
	 * second), even though the AT90 is LITTLE ENDIAN machine.
	 */
	stackTop[34] = (uint8_t) ((uint16_t) f >> 8);
	stackTop[35] = (uint8_t) (uint16_t)  f);
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
	NUM_TASKS_IN_USE++;
	
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
	Task_Create_Common(f, arg, RR), task;
	queuePush(&readyQueue[RR], task);
	NUM_TASKS_IN_USE++;
	
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
	Tasks[NUM_TASKS_IN_USE].wcet = wcet;
	Tasks[NUM_TASKS_IN_USE].ticksUntilReady = start;
	queueAddSortedByTicksUntilReady(&readyQueue[PERIODIC], task); 
	NUM_TASKS_IN_USE++;
	
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

	arg = currentTask->arg;

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
	event_list[e]->flag = 0;
}  

/* 
checks flag, 
if 1, resumes, 
if 0, waits for next event
POTENTIAL BUG: code isn't cli'd 
*/
void  Event_Wait( EVENT *e ) {
	if (e->flag == 1){
		e->flag=0;
	if (currentTask->level == PERIODIC) OS_Abort;
	/*abort if a periodic task tries to wait*/
	if (event_list[e]->flag == 1){ 
	/* check if an event is already waiting */
		
		event_list[e]->flag==0; 
		/* if so, consume flag, set task to sleepqueue, pop it off sleep queue */
		
		queuePop(readyQueue[currentTask->level]); 
		/* POTENTIAL BUG: make sure that we do indeed always need to send the task to the sleep queue*/
		
		event_list[e]->waiting_task = currentTask;
		Event_Signal(e);

	} else { /* if no events have arrived */
		if (event_list[e]->waiting_task != NULL) OS_Abort;
		/* only one task can wait per event */
		queuePop(readyQueue[currentTask->level]);
		event_list[e]->waiting_task = currentTask;
	}
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
	if (event_list[e]->waiting_task==NULL) OS_Abort;
	queueAddFront(event_list[e]->waiting_task);
	event_list[e]->waiting_task = NULL;
}

/* 
clears flag, waits for next event 
*/
void  Event_Wait_Next( EVENT *e ) {
	e->flag = 0;
	e->waiting_task = currentTask;
	event_list[e]->flag == 0;
	if (event_list[e]->waiting_task != NULL) OS_Abort;
	queuePop(readyQueue[currentTask->level]);
	event_list[e]->waiting_task = currentTask;
}

/* 
signals waiting task, or, if there are none, sets the flag
 */
void  Event_Signal( EVENT *e ) {
	if (event_list[e]->waiting_task!=NULL) {
		taskToReady(e);
	}
	else event_list[e]->flag = 1;

}

/* we do nothing with this yes, requires ISR, will do later */
void  Event_Async_Signal( EVENT *e ) {
	//basically the same as eventstatic EVENT event_list[MAXEVENT];_signal, but for a bonus and uses ISR
}


/* end event section */

unsigned int Now() {
	unsigned int i = 0;
	sreg = SREG;
	cli();
	i = TCNT2;
	SREG = sreg;
	i = (i/11590) + cur_ticks;
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
	if (currentTask->state != RUNNING && currentTask == idleTask)
	{
		if (readyQueue[SYSTEM].head != NULL)
		{
			currentTask = queuePop(&readyQueue[SYSTEM].head);
		}
		else if (readyQueue[PERIODIC].head != NULL && readyQueue[PERIODIC].head->ticksUntilReady == 0)
		{
			currentTask = queuePop(&readyQueue[PERIODIC].head);
		}
		else if (readyQueue[RR].head != NULL)
		{
			currentTask = queuePop(&readyQueue[RR].head);
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
		queuePush(&readyQueue[currentTask->level]);
		currentTask->state = READY;
	break;
	}
}

static void kernelMainLoop(void) 
{
	for (;;) {
		
	}
}

static void kernel_update_ticker(void) {
	/* update the tick count each time we enter this function (timed interrupt fires once/tick) */
	cur_ticks++;
}

int OS_Init(void)
{
	int i;
	//push all descriptors onto ready to use descriptor queue
	for (int i = 0; i < MAXPROCESS; i++)
	{
		Tasks[i].state = DEAD;
		queuePush(&readyQueue[0], &Tasks[i]);
	}
    //create idle task
	Tasks[MAXPROCESS].state = READY;
	Task_Create_Common(idle, NULL, NULL, &Tasks[MAXPROCESS]);
	NUM_TASKS_IN_USE++;
	
	//setup timers
	TCCR2B &= ~(_BV(CS12)| _BV(CS11));
	TCCR2B |= (_BV(CS10));		// start the timer at fclk/1
	
	/* Set up Timer 2 Output Compare interrupt,the TICK clock. */
	TIMSK2 |= _BV(OCIE2A);
	
	OCR2A = TCNT2 + TICK_CYCLES;
	/* Clear flag. */
	TIFR2 = _BV(OCF2A);
}

static void enterKernel(void) {
	/*
	 * The PC was pushed on the stack with the call to this function.
	 * Now push on the I/O registers and the SREG as well.
	 */
	SAVE_CTX();

	/*
	 * The last piece of the context is the SP. Save it to a variable.
	 */
	cur_task->sp = (uint8_t*) SP;

	/*
	 * Now restore the kernel's context, SP first.
	 */
	SP = kernel_sp;

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
void TIMER2_COMPA_vect(void) __attribute__ ((signal, naked));
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
