/*
 * minithread.c:
 *	This file provides a few function headers for the procedures that
 *	you are required to implement for the minithread assignment.
 *
 *	EXCEPT WHERE NOTED YOUR IMPLEMENTATION MUST CONFORM TO THE
 *	NAMING AND TYPING OF THESE PROCEDURES.
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include "minithread.h"
#include "queue.h"
#include "synch.h"

#include <assert.h>

/*
 * A minithread should be defined either in this file or in a private
 * header file.  Minithreads have a stack pointer with to make procedure
 * calls, a stackbase which points to the bottom of the procedure
 * call stack, the ability to be enqueueed and dequeued, and any other state
 * that you feel they must have.
 */


/* minithread functions */

/*THREADS STATES*/
#define WAITING 0
#define RUNNABLE 1
#define RUNNING 2


//the currently running thread
minithread_t running_thread = NULL;
int new_id = 0;

//the queue of runnable threads
queue_t runnables;
//the queue of waiting threads, including the new ones.
queue_t waitings;

struct minithread {
  stack_pointer_t stack_top;
  stack_pointer_t stack_base;
  int thread_id;
  int state; 
};

minithread_t minithread_fork(proc_t proc, arg_t arg) {
    minithread_t new_thread = (minithread_t)malloc(sizeof(struct minithread));
    minithread_allocate_stack(new_thread->stack_base, new_thread->stack_top);
    new_thread->state = RUNNABLE;
    new_thread->thread_id = new_id;
    new_id = new_id + 1;
    queue_append(runnables, new_thread);
    return new_thread;
}

minithread_t minithread_create(proc_t proc, arg_t arg) {
    minithread_t new_thread = (minithread_t)malloc(sizeof(struct minithread));
    minithread_allocate_stack(new_thread->stack_base, new_thread->stack_top);
    new_thread->state = WAITING;
    new_thread->thread_id = new_id;
    new_id = new_id + 1;
    queue_append(waitings, new_thread);
    return new_thread;
}

minithread_t minithread_self() {
    return running_thread;
}

int minithread_id() {
    return running_thread->thread_id;
}

void minithread_stop() {
  running_thread->state = WAITING;
  queue_append(waitings, running_thread);
  running_thread = NULL;
}

void minithread_start(minithread_t t) {
  t->state = RUNNABLE;
  queue_append(runnables, t);
}

void minithread_yield() {
  running_thread->state = RUNNABLE;
  queue_append(runnables, running_thread);
  void *thread = NULL;
  queue_dequeue(runnables, &thread);
  minithread_switch(running_thread->stack_top, ((minithread_t)thread)->stack_top);
  running_thread = (minithread_t)thread;
  running_thread->state = RUNNING;
}

/*
 * Initialization.
 *
 * 	minithread_system_initialize:
 *	 This procedure should be called from your C main procedure
 *	 to turn a single threaded UNIX process into a multithreaded
 *	 program.
 *
 *	 Initialize any private data structures.
 * 	 Create the idle thread.
 *       Fork the thread which should call mainproc(mainarg)
 * 	 Start scheduling.
 *
 */
void minithread_system_initialize(proc_t mainproc, arg_t mainarg) {
  runnables = queue_new();
  waitings = queue_new();
  minithread_t idle = (minithread_t)malloc(sizeof(struct minithread));
  idle->stack_top = NULL;
  idle->stack_base = NULL;

  minithread_t main_t = minithread_fork(mainproc, mainarg)

  void *thread = NULL;
  queue_dequeue(runnables, &thread);
  
  running_thread = (minithread_t)thread;
  running_thread->state = RUNNING;



  /*queue_t queue = queue_new();
  int x = 4;
  int y = queue_length(queue);
  void *datum = NULL;
  printf("size = %d\n", y);
  queue_append(queue, &x);
  y = queue_length(queue);
  printf("new size = %d\n", y);
  queue_dequeue(queue, &datum);
  y = *((int *)datum);
  printf("value = %d\n", y);*/
}


