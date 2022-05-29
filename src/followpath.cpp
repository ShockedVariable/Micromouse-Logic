#include <deque>
#include "followpath.hpp"
#include "micromouse.hpp"

FollowPath::FollowPath(MicroMouse& mouse)
    : mouse{&mouse}
{
}

void FollowPath::pushMovement(const Direction& move)
{
    if (!pathList.empty() && pathList.back().movement == move)
    {
        ++pathList.back().amount;
    }
    else
    {
        pathList.push_back(Command{move, 1});
    }
}

void FollowPath::runList()
{
    Command curr_move;

    while (!pathList.empty())
    {
        curr_move = pathList.front();
        pathList.pop_front();

        switch(curr_move.movement)
        {
            case Direction::LEFT:
                mouse->turnLeft(curr_move.amount);
                break;
            case Direction::RIGHT:
                mouse->turnRight(curr_move.amount);
                break;
            case Direction::FORWARDS:
                mouse->goForward(curr_move.amount);
                break;
            default:
                break;
        }
    }
}