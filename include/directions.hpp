#ifndef directions_hpp
#define directions_hpp


// This enum abstracts away needing to know which number is for
// moving forward, backwards, or stopping.
enum Direction 
{
  FORWARDS = 0, BACKWARDS = 1, STOP = 2
};

#endif