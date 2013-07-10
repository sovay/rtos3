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
    "pop    r0                \n\t"\
    "pop    r1                \n\t"\
    "pop    r2                \n\t"\
    "pop    r3                \n\t"\
    "pop    r4                \n\t"\
    "pop    r5                \n\t"\
    "pop    r6                \n\t"\
    "pop    r7                \n\t"\
    "pop    r8                \n\t"\
    "pop    r9                \n\t"\
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

typedef struct  Event_Struct {
	unsigned flag:1;
	Task_t waiting_task;
} EVENT;

typedef struct Task_Struct 
{
	void (* function)(void) = NULL;
	int argument;
	uint8_t stack[MAXSTACK];
	uint8_t volatile * stackPointer = NULL;
	uint8_t level;
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
 * @brief Contains pointers to head and tail of a linked list.
 */
typedef struct {
	/** The first item in the queue. NULL if the queue is empty. */
	Task_t * head;
	/** The last item in the queue. Undefined if the queue is empty. */
	Task_t * tail;
} queue_t;

void queueInit(queue_t * q)
{
		q->head = q->tail = NULL;
}

void queuePush(queue_t * q, Task_t * task)
{
	if(q->head == NULL) {
		q->head = q->tail = task;
	} else {
		q->tail->next = task;
		task->prev = q->tail;
		q->tail = task;
	}
}

void queueAddFront(queue_t * q, Task_t * task)
{
	if(q->head == SNULL) {
		q->head = q->tail = task;
    } else {
		q->head->prev = task;
		task->next = q->head;
		q->head = task;
	}
}

Task_t * queuePop(queue_t * q)
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

void queueAddSortedByTicksUntilReady(queue_t * q, Task_t * task)
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

//Kernal Globals
static Task_t  Tasks[MAXPROCESS];
static uint8_t NUM_TASKS_IN_USE = 0; 
static Task_t* currentTask = NULL;
static queue_t readyQueue[3];
static queue_t sleepQueue[3];

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

int Task_Create_Common(void (*f)(void), int arg, uint8_t level)
{
	Tasks[NUM_TASKS_IN_USE].function = f;
	Tasks[NUM_TASKS_IN_USE].argument = arg;
	Tasks[NUM_TASKS_IN_USE].level    = level;
	Tasks[NUM_TASKS_IN_USE].stackPointer = &Tasks[NUM_TASKS_IN_USE].stack;
	Tasks[NUM_TASKS_IN_USE].wcet = 0;
}

int   Task_Create_System(void (*f)(void), int arg)
{
	if (NUM_TASKS_IN_USE >= MAXPROCESS) return -1; //error to many procs
	Task_Create_Common(f, arg, SYSTEM);
	//TODO do scheduling or w/e else
	NUM_TASKS_IN_USE++;
	return 0;
}

int   Task_Create_RR(void (*f)(void), int arg)
{
	if (NUM_TASKS_IN_USE >= MAXPROCESS) return -1; //error to many procs
	Task_Create_Common(f, arg, RR);
	//TODO do scheduling or w/e else
	NUM_TASKS_IN_USE++;
	return 0;
}

int   Task_Create_Period(void (*f)(void), int arg, unsigned int period, unsigned int wcet, unsigned int start)
{
	if (NUM_TASKS_IN_USE >= MAXPROCESS) return -1; //error to many procs
	Task_Create_Common(f, arg, PERIODIC);
	Tasks[NUM_TASKS_IN_USE].wcet = wcet;
	//TODO do scheduling or w/e else
	NUM_TASKS_IN_USE++;
	return 0;
}

//TODO this only works on back of array
//TODO factor out NUM_TASKS_IN_USE at least as an array index
void  Task_Terminate(void) 
{
}

void  Task_Next(void);

int   Task_GetArg(void)          
{
	return Tasks[currentTask].argument;
}

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
	return event_ptr;

}
void  Event_Clear( EVENT *e ) {
	e->flag = 0;
}  

/* checks flag, if 1, resumes, if 0, waits for next event */
void  Event_Wait( EVENT *e ) {
	if (e->flag == 1){
		e->flag==0;
		Event_Signal(e);
	} else {
		e->waiting_task = currentTask;
	}
}

/* clears flag, waits for next event */
void  Event_Wait_Next( EVENT *e ) {
	e->flag == 0;
	e->waiting_task = currentTask;
}

/* signals waiting task, or, if there are none, sets the flag */
void  Event_Signal( EVENT *e ) {
	if (e->waiting_task!=NULL) {
		// resume(we->waiting_task);
	}
	else e->flag = 1;

}

/* we do nothing with this yes, requires ISR, will do later */
void  Event_Async_Signal( EVENT *e ) {
	//basically the same as event_signal, but for a bonus and uses ISR
}

unsigned int Now();  


int OS_Init(void)
{
    
}