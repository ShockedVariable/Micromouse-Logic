#ifndef newmaze_hpp
#define newmaze_hpp

#include <array>
#include <vector>
#include "directions.hpp"
#include "micromouse.hpp"

#define MazeSize 16

struct Point
{
    short x;
    short y;
};

struct DirPoint
{
    Point position;
    Direction direction;
};

class Maze
{
public:
    Maze();

    std::array<bool*, 4> getBlockWalls(const short& x, const short& y);

    bool inGoal(const short& x, const short& y);

    void floodFill(const short& x, const short& y);

    std::vector<DirPoint> getSurroundingPoints(const Point& p, const std::array<bool*, 4>& walls);

    DirPoint getMinPoint(const std::vector<DirPoint>& surrPoints, MicroMouse& mouse, const std::array<std::array<int, MazeSize>, MazeSize>& board);

    void moveMouse(MicroMouse& mouse);

    void exploreMaze(MicroMouse& mouse);
    
    bool checkPath();

    void printDistBoard();

    void printVertical();

    void printHorizontal();

private:
    bool final_path;

    std::array<std::array<int, MazeSize>, MazeSize> dist_board;
    std::array<std::array<bool, MazeSize>, MazeSize + 1> vertical;
    std::array<std::array<bool, MazeSize + 1>, MazeSize> horizontal;
    std::array<std::array<int, MazeSize>, MazeSize> explored;
};


#endif