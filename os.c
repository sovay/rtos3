/*
 * CSC460Proj3.c
 *
 * Created: 05/07/2013 10:38:25 AM
 *  Author: Rob & Nick
 */ 


#include <avr/io.h>
#include "os.h"

/**
 * @brief Contains all information of a given task.
 */
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

void  OS_Abort(void)
{
	for(;;) { 
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

EVENT *Event_Init(void);
void  Event_Clear( EVENT *e );  
void  Event_Wait( EVENT *e );  
void  Event_Wait_Next( EVENT *e );  
void  Event_Signal( EVENT *e );
void  Event_Async_Signal( EVENT *e );
unsigned int Now();  


int OS_Init(void)
{
    
}