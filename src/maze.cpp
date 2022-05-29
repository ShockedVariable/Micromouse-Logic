#include <algorithm>
#include <array>
#include <iostream>
#include <stack>
#include <vector>
#include "directions.hpp"
#include "maze.hpp"
#include "micromouse.hpp"


Maze::Maze()
: finalPath{false}
{
	// Maze array initialization

	for(std::array<bool,MazeSize>& arr: vertical)
	{
		arr.fill(false);
	}

	for(std::array<bool,MazeSize+1>& arr: horizontal)
	{
		arr.fill(false);
		arr[0] = true;
		arr[MazeSize] = true;
	}

	int temp = (MazeSize - 1) / 2;

	for(unsigned int j = 0; j < MazeSize; ++j)
	{
		for(unsigned int i = 0; i < MazeSize; ++i)
		{
			board[i][j] = abs(int(temp - i + ((temp < i) ? 1 : 0))) + abs(int(temp - j + ((temp < j) ? 1 : 0)));
		}
	}

	// Initial Values
	vertical[0].fill(true);
	vertical[MazeSize].fill(true);

	// Optimizations

	for(std::array<bool,MazeSize>& arr: explored)
	{
		arr.fill(false);
	}
}

std::array<bool*,4> Maze::getBlockWalls(short x, short y)
{
	std::array<bool*,4> walls{&vertical[y][x],&horizontal[y][x+1],&vertical[y+1][x],&horizontal[y][x]};
	return walls; 
	//Up, Right, Down, Left
}

void Maze::exploreMaze(MicroMouse& mouse)
{
	if(! explored[mouse.getYpos()][mouse.getXpos()])
	{
	    finalPath = false;
		std::array<bool*,4> block = this->getBlockWalls(mouse.getXpos(),mouse.getYpos());
		std::array<bool,3> walls = mouse.getWalls();
		*block[mouse.getDirection()] = walls[1];
		*block[shiftClockwise(mouse.getDirection())] = walls[2];
		*block[shiftCounterClockwise(mouse.getDirection())] = walls[0];

		explored[mouse.getYpos()][mouse.getXpos()] = true;

		this->fixMaze(mouse.getXpos(),mouse.getYpos());
	}
}