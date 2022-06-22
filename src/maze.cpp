#include <Arduino.h>
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
		
		*block[mouse.getDir()] = walls[1];
		*block[shiftClockwise(mouse.getDir())] = walls[2];
		*block[shiftCounterClockwise(mouse.getDir())] = walls[0];

		explored[mouse.getYpos()][mouse.getXpos()] = true;

		this->fixMaze(mouse.getXpos(),mouse.getYpos());
	}
}

std::vector<DirPoint> Maze::getSurroundingPoints(Point p, std::array<bool*,4>& walls)
{
	std::vector<DirPoint> blocks;
	if(p.y > 0 && !(*walls[0]))
	{
		blocks.push_back({Point{p.x,(short)(p.y-1)}, Direction::FORWARDS});
	}
	if(p.x < MazeSize-1 && !(*walls[1]))
	{
		blocks.push_back({Point{(short)(p.x+1),p.y},Direction::RIGHT});
	}
	if(p.y < MazeSize-1 && !(*walls[2]))
	{
		blocks.push_back({Point{p.x,(short)(p.y+1)},Direction::BACKWARDS});
	}
	if(p.x > 0 && !(*walls[3]))
	{
		blocks.push_back({Point{(short)(p.x-1),p.y},Direction::LEFT});
	}
	Serial7.printf("Board Vals: ");
	for (DirPoint d: blocks)
	{
		Serial7.printf("%d ", this->board[d.position.x][d.position.y]);
	}
	Serial7.printf("\r\n");
	
	
	return blocks;
}

DirPoint getMinPoint(std::vector<DirPoint>& surrPoints, MicroMouse& mouse, std::array<std::array<int,MazeSize>,MazeSize>& board)
{
	return *std::min_element(std::begin(surrPoints),std::end(surrPoints),
			[&](const DirPoint& a, const DirPoint& b)
			{
				return board[a.position.y][a.position.x] - ((a.direction == mouse.getDir()) ? .5 : 0) < board[b.position.y][b.position.x] - ((b.direction == mouse.getDir()) ? .5 : 0);
			}
			);
}

void Maze::moveMouse(MicroMouse& mouse)
{
	std::array<bool*,4> walls = this->getBlockWalls(mouse.getXpos(),mouse.getYpos());
	Point p{mouse.getXpos(),mouse.getYpos()};
	std::vector<DirPoint> surrPoints = getSurroundingPoints(p,walls);


	DirPoint minPoint = getMinPoint(surrPoints,mouse, this->board);

	mouse.setXpos(minPoint.position.x);
	mouse.setYpos(minPoint.position.y);
	mouse.setDir(minPoint.direction);

	if(minPoint.direction == mouse.getDir())
	{
		mouse.goForward();
		delay(500);
	}
	else if(minPoint.direction == shiftClockwise(mouse.getDir()))
	{
		mouse.turnRight();
		delay(500);
		mouse.goForward();
		delay(500);
	}
	else if(minPoint.direction == shiftCounterClockwise(mouse.getDir()))
	{
		mouse.turnLeft();
		delay(500);
		mouse.goForward();
		delay(500);
	}
	else
	{
		mouse.turnRight();
		delay(500);
		mouse.turnRight();
		delay(500);
		mouse.goForward();
		delay(500);
	}
}


void Maze::fixMaze(short x, short y)
{
	if(this->inGoal(x,y))
	{
		return;
	}

	Point start{x,y};
	std::stack<Point> fixStack;
	fixStack.push(start);  
	std::array<bool*,4> currBlock;  
	Point currPoint;
	std::vector<DirPoint> surrPoints;
	int newDist;
	Point minPoint;

	while(!fixStack.empty())
	{
		currPoint = fixStack.top();
		fixStack.pop();
		currBlock = this->getBlockWalls(currPoint.x,currPoint.y);
		surrPoints = getSurroundingPoints(currPoint,currBlock);
		minPoint = std::min_element(std::begin(surrPoints),std::end(surrPoints),
			[&](const DirPoint& a, const DirPoint& b){return board[a.position.y][a.position.x] < board[b.position.y][b.position.x];}
			)->position;
		newDist = board[minPoint.y][minPoint.x];
		if(newDist + 1 != board[currPoint.y][currPoint.x])
		{
			board[currPoint.y][currPoint.x] = newDist + 1;
			for(DirPoint dp : surrPoints)
			{
				if(!this->inGoal(dp.position.x,dp.position.y))
				{
					fixStack.push(dp.position);
				}
			}
		}
	}
}

void Maze::setWalls(short x, short y, bool up, bool right, bool down, bool left)
{
	std::array<bool*,4> block = this->getBlockWalls(x,y);
	*block[0] = up;
	*block[1] = right;
	*block[2] = down;
	*block[3] = left;
}

bool Maze::inGoal(short x, short y)
{
	return (x == MazeSize/2 || x == MazeSize/2 - 1) && (y == MazeSize/2 || y == MazeSize/2 - 1);
}

bool Maze::checkPath()
{
	if(finalPath)
	{
		return true;
	}
	else
	{
		finalPath = true;
		return false;
	}
}