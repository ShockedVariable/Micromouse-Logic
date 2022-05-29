#ifndef directions_hpp
#define directions_hpp

// This enum abstracts away needing to know which number is for
// moving forward, backwards, or stopping.
enum Direction 
{
  FORWARDS = 0, RIGHT = 1, BACKWARDS = 2, LEFT = 3, STOP = 4
};

char shiftClockwise(const Direction& d);

char shiftCounterClosewise(const Direction& d);

#endif