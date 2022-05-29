#include "directions.hpp"

char shiftClockwise(const Direction& d)
{
    return (d + 1) % 4;
}

char shiftCounterClockwise(const Direction& d)
{
    if (!d)
    {
        return Direction::LEFT;
    }

    return (d - 1) % 4;
}