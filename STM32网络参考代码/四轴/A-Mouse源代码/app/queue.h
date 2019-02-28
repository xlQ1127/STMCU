#ifndef __QUEUE_H_
#define __QUEUE_H_
#include "stdbool.h"
typedef struct queue 
{
	short *pBase;
	short front;    //
	short rear;    //
	short maxsize; //
}QUEUE,*PQUEUE;

void CreateQueue(PQUEUE Q,short maxsize);
void TraverseQueue(PQUEUE Q);
bool FullQueue(PQUEUE Q);
bool EmptyQueue(PQUEUE Q);
bool Enqueue(PQUEUE Q, short val);
bool Dequeue(PQUEUE Q, short *val);
#endif
