/*
 * CSC460Proj3.c
 *
 * Created: 05/07/2013 10:38:25 AM
 *  Author: Rob & Nick
 */ 


#include <avr/io.h>
#include "os.h"

/** PPP and PT defined in user application. */
extern const unsigned char PPP[];

/** PPP and PT defined in user application. */
extern const unsigned int PT;

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
	uint8_t name;
	uint8_t wcet; //worst case execution time
} Task_t;

/**
 * @brief Contains pointers to head and tail of a linked list.
 */
typedef struct {
	/** The first item in the queue. NULL if the queue is empty. */
	task_descriptor_t* head;
	/** The last item in the queue. Undefined if the queue is empty. */
	task_descriptor_t* tail;
} queue_t;

//Kernal Globals
static Task_t  Tasks[MAXPROCESS];
static uint8_t NUM_TASKS_IN_USE = 0; 
static Task_t* currentTask = NULL;

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
	Tasks[NUM_TASKS_IN_USE].name = name;
	Tasks[NUM_TASKS_IN_USE].wcet = wcet;
	//TODO do scheduling or w/e else
	NUM_TASKS_IN_USE++;
	return 0;
}

void  Task_Terminate(void);    
void  Task_Next(void);
int   Task_GetArg(void);          
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