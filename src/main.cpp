#include <Arduino.h>
#include "directions.hpp"
#include "followpath.hpp"
#include "micromouse.hpp"
#include "newmaze.hpp"
#include "pathfinder.hpp"
#include "pins.hpp"
// #include "testhelper.hpp"

MicroMouse mm(0, MazeSize - 1, Direction::FORWARDS);

FollowPath path{mm};

Maze maze{};

bool WaitForSignal(Maze& maze)
{
    // Wait till some signal/button is pressed
    return !maze.checkPath();
}

void setup() 
{
	mm.initConnections();
	mm.attachInterrupts();
	// setupTestMaze();

	mm.findCenter();

	delay(2000);
	
	// Serial7 is default for bluetooth module.
	Serial7.begin(9600);
}

void loop()
{
    while (mm.getDistFR() < 900);

	while (WaitForSignal(maze))
	{
		// mouse = Mouse{0,MazeSize - 1,0};
		while (!maze.inGoal(mm.getXpos(), mm.getYpos()))
		{
			maze.exploreMaze(mm);
			maze.moveMouse(mm);
			// Serial7.printf("%d, %d, %d\r\n", mm.getXpos(), mm.getYpos(), mm.getDir());
			
		}
	}

	// Everything above this point needs to be tested before moving on. The flood fill algorithm still needs to be reviewed by testing.
	// Everything below this point needs the code to be reviewed.

	// mm = MicroMouse{0,MazeSize - 1,0};
	PathFinder pathFinder{mm};

	runPathFinder(pathFinder,maze);

	pathFinder.runPath();

	// while (1)
	// {
	// 	std::array<bool, 3> w = mm.getWalls();
	// 	// Serial7.printf("%d, %d, %d\r\n", w[0], w[1], w[2]);
	// 	delay(500);
	// }
	
	// delay(500);
	// path.pushMovement(Direction::FORWARDS);
    // path.pushMovement(Direction::FORWARDS);
    // path.pushMovement(Direction::FORWARDS);

    // path.pushMovement(Direction::RIGHT);

    // path.pushMovement(Direction::FORWARDS);

    // path.pushMovement(Direction::RIGHT);

    // path.pushMovement(Direction::FORWARDS);
    // path.pushMovement(Direction::FORWARDS);
    // path.pushMovement(Direction::FORWARDS);

    // path.pushMovement(Direction::LEFT);

    // path.pushMovement(Direction::FORWARDS);

    // path.pushMovement(Direction::LEFT);

    // path.pushMovement(Direction::FORWARDS);
    // path.pushMovement(Direction::FORWARDS);

    // path.pushMovement(Direction::RIGHT);

    // path.pushMovement(Direction::FORWARDS);

    // path.runList();
	

	delay(2500);
    
}

//no resistance, battery full charge
//160 measures about 6v
//150 measure about 6v
//140 measure about 5.9v

//reccomend 

//Ran with motor limit 140, 
//with motors maxed out, all emitters on, all leds on, (buzzer off), no resistance at wheels. it drew about 0.9A.


//cell = 18 cm
//cell gap = 1.2 cm
//diameter wheel to wheel 9.1 cm
//wheel diameter is 2.8 cm
//90 ticks for 1 revolution the wheel
//
//approx 197 ticks for to move one cell
//approx 73 ticks to turn 90 degrees


// calibration tip: for the first couple of seconds
// let the mouse remember what is the "center" as reference point 