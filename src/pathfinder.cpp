#include "directions.hpp"
#include "maze.hpp"
#include "micromouse.hpp"
#include "pathfinder.hpp"

PathFinder::PathFinder(MicroMouse& mouse)
    : MicroMouse(0, MazeSize - 1, Direction::FORWARDS), best_path{mouse}
{
}

void PathFinder::turnRight(const int& amt)
{
    
}