#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "defs.h"
#include "synch.h"
#include "queue.h"
#include "minithread.h"

/*
 *	You must implement the procedures and types defined in this interface.
 */


/*
 * Semaphores.
 */
struct semaphore {
    tas_lock_t lock;
    int count;
    queue_t queue;
};


/*
 * semaphore_t semaphore_create()
 *	Allocate a new semaphore.
 */
semaphore_t semaphore_create() {
	semaphore_t semaphore = (semaphore_t)malloc(sizeof(struct semaphore));
    return semaphore;
}

/*
 * semaphore_destroy(semaphore_t sem);
 *	Deallocate a semaphore.
 */
void semaphore_destroy(semaphore_t sem) {
	free(sem);
}

 
/*
 * semaphore_initialize(semaphore_t sem, int cnt)
 *	initialize the semaphore data structure pointed at by
 *	sem with an initial value cnt.
 */
void semaphore_initialize(semaphore_t sem, int cnt) {
	sem->value = cnt;
}


/*
 * semaphore_P(semaphore_t sem)
 *	P on the sempahore.
 */
void semaphore_P(semaphore_t sem) {
	while(test_and_set(&s->lock) == 1);
	if (--s->count < 0) { enqueue on wait list, s->lock = 0; run something else; }
	else { s->lock = 0; } 
}

/*
 * semaphore_V(semaphore_t sem)
 *	V on the sempahore.
 */
void semaphore_V(semaphore_t sem) {
	while(test_and_set(&s->lock) == 1);
	if (++s->count <= 0) { dequeue from wait list, make runnable; }
	s->lock = 0; 
}
