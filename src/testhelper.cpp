// #include <array>
// #include <limits>
// #include "directions.hpp"
// #include "maze.hpp"
// #include "testhelper.hpp"

// TestMaze* tester_maze;

// void setupTestMaze()
// {
//     tester_maze = new TestMaze{};
// }

// TestMaze& getTestMaze()
// {
//     return *tester_maze;
// }

// std::array<bool, 3> getMouseWalls(const short& x, const short& y, const Direction& direction)
// {
//     std::array<bool*, 4> block = tester_maze->getBlockWalls(x, y);
//     std::array<bool, 3> walls{{*(block[shiftCounterClockwise(direction)]),*(block[direction]),*(block[shiftClockwise(direction)])}};

//     return walls;
// }