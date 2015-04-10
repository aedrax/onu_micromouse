// Cell.h

#ifndef _CELL_h
#define _CELL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

struct Cell
{
	//byte x;
	//byte y;
	byte position; //positions are stored as0bXXXXYYYY
	byte distance;
	byte data;
};

#endif

