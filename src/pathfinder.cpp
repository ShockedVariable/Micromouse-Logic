#include "directions.hpp"
#include "newmaze.hpp"
#include "micromouse.hpp"
#include "pathfinder.hpp"

PathFinder::PathFinder(MicroMouse& mouse)
    : MicroMouse(0, MazeSize - 1, Direction::FORWARDS), best_path{mouse}
{
}

void PathFinder::goForward(const int& blocks)
{
    shiftDirection(x_pos, y_pos, dir);
    best_path.pushMovement(Direction::FORWARDS);
}

void PathFinder::turnRight(const int& blocks)
{
    dir = shiftClockwise(dir);
    best_path.pushMovement(Direction::RIGHT);
}

void PathFinder::turnLeft(const int& blocks)
{
    dir = shiftCounterClockwise(dir);
    best_path.pushMovement(Direction::LEFT);
}

void PathFinder::runPath()
{
    best_path.runList();
}

void runPathFinder(PathFinder& pathfinder, Maze& maze)
{
     while (!maze.inGoal(pathfinder.getXpos(), pathfinder.getYpos()))
    {
        maze.moveMouse(pathfinder);
    }
}