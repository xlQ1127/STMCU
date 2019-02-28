#include <stdio.h>
#include <stdlib.h>
#include "rt_heap.h"
#include "led.h"
#include "stdbool.h"
#include "queue.h"
/***********************************************
Function: Create a empty stack;
************************************************/
void CreateQueue(PQUEUE Q,short maxsize)
{
	short t=0;
	Q->pBase=(short *)malloc(sizeof(short)*maxsize);
	while(NULL==(short *)malloc(sizeof(short)*maxsize))
	{
		//printf("Memory allocation failure");
		//exit(-1);        //????
		if(t>10) 
		{LED1 = !LED1;t=0;}
		t++;
	}
	Q->front=0;         //?????
	Q->rear=maxsize-1;
	Q->maxsize=maxsize;
	for(t=0;t<maxsize;t=0)
	{
		Q->pBase[t]=15000;
	}
}
/***********************************************
Function: Print the stack element;
************************************************/

bool FullQueue(PQUEUE Q)
{
	if(Q->front==(Q->rear+1)%Q->maxsize)    //判断队列是否已满
		return true;
	else
		return false;
}
bool EmptyQueue(PQUEUE Q)
{
	if(Q->front==Q->rear)    //??????
		return true;
	else
		return false;
}

//入队
bool Enqueue(PQUEUE Q, short val)
{
	if(FullQueue(Q))
		return false;
	else
	{
		Q->pBase[Q->rear]=val;
		Q->rear=(Q->rear+1)%Q->maxsize;
		return true;
	}
}
//出队

bool Dequeue(PQUEUE Q, short *val)
{
	if(EmptyQueue(Q))
	{
		return false;
	}
	else
	{
		*val=Q->pBase[Q->front];
		Q->front=(Q->front+1)%Q->maxsize;
		return true;
	}
}
