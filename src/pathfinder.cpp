#include "directions.hpp"
#include "maze.hpp"
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
    dir = shiftClockwise();
}