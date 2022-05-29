#include "directions.hpp"

Direction shiftClockwise(const Direction& d)
{   
    const unsigned char direction_temp = (d + 1) % 4;
    Direction dir;
    
    switch(direction_temp)
    {
        case 0:
            dir = Direction::FORWARDS;
            break;
        case 1:
            dir = Direction::RIGHT;
            break;
        case 2:
            dir= Direction::BACKWARDS;
            break;
        case 3:
            dir = Direction::LEFT;
            break;
        default:
            break;
    }

    return dir;
}

Direction shiftCounterClockwise(const Direction& d)
{
    if (!d)
    {
        return Direction::LEFT;
    }

    const unsigned char direction_temp = (d - 1) % 4; 
    Direction dir;

    switch(direction_temp)
    {
        case 0:
            dir = Direction::FORWARDS;
            break;
        case 1:
            dir = Direction::RIGHT;
            break;
        case 2:
            dir= Direction::BACKWARDS;
            break;
        case 3:
            dir = Direction::LEFT;
            break;
        default:
            break;
    }
        
    return dir;
}