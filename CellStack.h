// CellStack.h

#ifndef _CELLSTACK_h
#define _CELLSTACK_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Cell.h"
struct node
{
	Cell* thisCell;
	node* nextNode;
};

class CellStack
{
 public:
	 CellStack();
	void push(Cell* cellPointer);
	Cell* pop();
	bool isEmpty();
	bool contains(Cell* cell);
private:
	node* topNode = NULL;

};

CellStack::CellStack()
{
	topNode = NULL;
}

void CellStack::push(Cell* cellPointer)
{
	node* newNode = new node;
	newNode->thisCell = cellPointer;
	if (topNode == NULL)
	{
		newNode->nextNode = NULL;
	}
	else
	{
		newNode->nextNode = topNode->nextNode;
	}
	topNode = newNode;
}

Cell* CellStack::pop()
{
	Cell* result = topNode->thisCell;
	node* temp = topNode->nextNode;
	delete topNode;
	topNode = temp;
	return result;
}

bool CellStack::isEmpty()
{
	if (topNode == NULL)
		return true;
	return false;
}

bool CellStack::contains(Cell* cell)
{
	if (topNode != NULL)
	{
		node* temp = topNode;

		while (temp != NULL)
		{
			if (temp->thisCell == cell)
			{
				return true;
			}
			temp = temp->nextNode;
		}
	}
	return false;
}
#endif

