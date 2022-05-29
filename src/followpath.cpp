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

        switch (curr_move.movement)
        {
            case Direction::LEFT:
                for (int i = 0; i < curr_move.amount; i++)
                {
                    mouse->turnLeft();
                }
                break;
            case Direction::RIGHT:
                for (int i = 0; i < curr_move.amount; i++)
                {
                    mouse->turnRight();
                }
                break;
            case Direction::FORWARDS:
                mouse->goForward(curr_move.amount);
                break;
        }
    }
}