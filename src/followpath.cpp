#include <Arduino.h>
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
        
        // Note: We do not need to worry about moving backwards, since it is never a move we queue.
        // Also, not certain if we need the delays.
        switch (curr_move.movement)
        {
            case Direction::LEFT:
                for (int i = 0; i < curr_move.amount; ++i)
                {
                    mouse->turnLeft(1);
                    // delay(500);
                }
                break;
            case Direction::RIGHT:
                for (int i = 0; i < curr_move.amount; ++i)
                {
                    mouse->turnRight(1);
                    // delay(500);
                }
                break;
            case Direction::FORWARDS:
                for (int i = 0; i < curr_move.amount; ++i)
                { 
                    mouse->goForward(1);
                    // delay(500);
                }
                break;
            default:
                break;
        }
    }
}