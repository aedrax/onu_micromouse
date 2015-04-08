#ifndef _CELL_H
#define _CELL_H

class Cell
{
public:
	Cell();

	byte x;

	byte y;

	bool north;

	bool east;

	bool south;

	bool west;

	byte distance;

	bool explored;

};


Cell::Cell()
{
	explored = false;
	north = false;
	east = false;
	south = false;
	west = false;
}


#endif