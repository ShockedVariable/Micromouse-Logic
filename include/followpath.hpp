#ifndef followpath_hpp
#define followpath_hpp

#include <deque>
#include "micromouse.hpp"

struct Command
{
    Direction movement;
    int amount;
};

class FollowPath
{
public:

    FollowPath(MicroMouse& mouse);

    void pushMovement(const Direction& move);

    void runList();

private:
    std::deque<Command> pathList;
    MicroMouse* mouse;
};




#endif