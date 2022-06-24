#ifndef pathfinder_hpp
#define pathfinder_hpp

#include "followpath.hpp"
#include "micromouse.hpp"
#include "newmaze.hpp"

class PathFinder: public MicroMouse
{
public:
    PathFinder(MicroMouse& mouse);

    virtual void goForward(const int& blocks = 1) override;

    virtual void turnRight(const int& blocks = 1) override;

    virtual void turnLeft(const int& blocks = 1) override;

    void runPath();

private:
    FollowPath best_path;
};

void runPathFinder(PathFinder& pathfinder, Maze& maze);

#endif