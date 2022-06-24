#include <Arduino.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <stack>
#include <vector>
#include "newmaze.hpp"

Maze::Maze()
    : final_path{false}
{
    for (auto& arr : vertical)
    {
        arr.fill(false);
    }

    for (auto& arr: horizontal)
    {
        arr.fill(false);
        arr.at(0) = true;
        arr.at(MazeSize) = true;
    }

    for (auto& arr: explored)
    {
        arr.fill(false);
    }

    vertical.at(0).fill(true);
    vertical.at(MazeSize).fill(true);

    const unsigned int temp = (MazeSize - 1) / 2;
    
    for (unsigned int j = 0; j < MazeSize; ++j)
    {
        for (unsigned int i = 0; i < MazeSize; ++i)
        {
            dist_board.at(i).at(j) = std::abs(int(temp - i + ((temp < i) ? 1 : 0))) + std::abs(int(temp - j + ((temp < j) ? 1 : 0)));
        }
    }
}

std::array<bool*, 4> Maze::getBlockWalls(const short& x, const short& y)
{
    // [Up, Right, Down, Left]
    return std::array<bool*, 4>{&vertical.at(y).at(x), &horizontal.at(y).at(x + 1), &vertical.at(y + 1).at(x), &horizontal.at(y).at(x)};
}

bool Maze::inGoal(const short& x, const short& y)
{
    return (x == MazeSize / 2 || x == MazeSize / 2 - 1) && (y == MazeSize / 2 || y == MazeSize / 2 - 1);
}

std::vector<DirPoint> Maze::getSurroundingPoints(const Point& p, const std::array<bool*, 4>& walls)
{
    std::vector<DirPoint> blocks;

    if (p.y > 0 && !(*walls.at(0)))
    {
        blocks.push_back(DirPoint{Point{p.x, static_cast<short>(p.y - 1)}, Direction::FORWARDS});
    }

    if (p.x < MazeSize - 1 && !(*walls.at(1)))
    {
        blocks.push_back(DirPoint{Point{static_cast<short>(p.x + 1), p.y}, Direction::RIGHT});
    }

    if (p.y < MazeSize - 1 && !(*walls.at(2)))
    {
        blocks.push_back(DirPoint{Point{p.x, static_cast<short>(p.y + 1)}, Direction::BACKWARDS});
    }

    if (p.x > 0 && !(*walls.at(3)))
    {
        blocks.push_back(DirPoint{Point{static_cast<short>(p.x - 1), p.y}, Direction::LEFT});
    }

    return blocks;
}

// Why are we subtracting 0.5 in the true case for the lambda function?
// Why not just do board.at(a.position.y).at(a.position.x) < board.at(b.position.y).at(b.position.x)?
// Are are biasing in favor of directions that are in the same direction as the current direction?
DirPoint Maze::getMinPoint(const std::vector<DirPoint>& surrPoints, MicroMouse& mouse, const std::array<std::array<int, MazeSize>, MazeSize>& board)
{
    return *std::min_element(surrPoints.begin(), surrPoints.end(), [&](const DirPoint& a, const DirPoint& b)
    {
        return board.at(a.position.y).at(a.position.x) - ((a.direction == mouse.getDir()) ? 0.5f : 0) < board.at(b.position.y).at(b.position.x) - ((b.direction == mouse.getDir()) ? 0.5f : 0);
    });
}

void Maze::moveMouse(MicroMouse& mouse)
{
    std::array<bool*, 4> walls = getBlockWalls(mouse.getXpos(), mouse.getYpos());
    Point p{static_cast<short>(mouse.getXpos()), static_cast<short>(mouse.getYpos())};
    std::vector<DirPoint> surr_points = getSurroundingPoints(p, walls);

    DirPoint min_point = getMinPoint(surr_points, mouse, dist_board);

    mouse.setXpos(min_point.position.x);
    mouse.setYpos(min_point.position.y);
    
    if (min_point.direction == mouse.getDir())
    {
        mouse.goForward();
        // delay(500); // not sure if needed.
    }
    else if (min_point.direction == shiftClockwise(mouse.getDir()))
    {
        mouse.turnRight();
        // delay(500); // not sure if needed.
        mouse.goForward();
        // delay(500); // not sure if needed.
    }
    else if (min_point.direction == shiftCounterClockwise(mouse.getDir()))
    {
        mouse.turnLeft();
        // delay(500); // not sure if needed.
        mouse.goForward();
        // delay(500); // not sure if needed.
    }
    else
    {
        mouse.turnRight();
        // delay(500); // not sure if needed.
        mouse.turnRight();
        // delay(500); // not sure if needed.
        mouse.goForward();
        // delay(500); // not sure if needed.
    }

    mouse.setDir(min_point.direction);
}

void Maze::floodFill(const short& x, const short& y)
{
    if (inGoal(x, y))
    {
        return;
    }

    Point start{x, y};
    Point curr_point;
    Point min_point;
    int new_dist;

    std::stack<Point> fix_stack;
    std::array<bool*, 4> curr_blocks;
    std::vector<DirPoint> surr_points;

    fix_stack.push(start);

    while (!fix_stack.empty())
    {
        curr_point = fix_stack.top();
        fix_stack.pop();
        curr_blocks = getBlockWalls(curr_point.x, curr_point.y);
        surr_points = getSurroundingPoints(curr_point, curr_blocks);
        min_point = std::min_element(surr_points.begin(), surr_points.end(), [&](const DirPoint& a, const DirPoint& b)
        {
            return dist_board.at(a.position.y).at(a.position.x) < dist_board.at(b.position.y).at(b.position.x);
        })->position;

        // need to understand everything below this point.
        new_dist = dist_board.at(min_point.y).at(min_point.x);

        if (new_dist + 1 != dist_board.at(curr_point.y).at(curr_point.x))
        {
            dist_board.at(curr_point.y).at(curr_point.x) = new_dist + 1;

            for (DirPoint dp: surr_points)
            {
                if (!inGoal(dp.position.x, dp.position.y))
                {
                    fix_stack.push(dp.position);
                }
            }
        }
    }
}

void Maze::exploreMaze(MicroMouse& mouse)
{
    if (!explored.at(mouse.getYpos()).at(mouse.getXpos()))
    {
        final_path = false;
        std::array<bool*, 4> block = getBlockWalls(mouse.getXpos(), mouse.getYpos());
        Walls walls = mouse.detectWalls();

        *block.at(shiftCounterClockwise(mouse.getDir())) = walls.l;
        *block.at(mouse.getDir()) = walls.f;
        *block.at(shiftClockwise(mouse.getDir())) = walls.r;

        explored.at(mouse.getYpos()).at(mouse.getXpos()) = true;

        floodFill(mouse.getXpos(), mouse.getYpos());
    }
        
}

bool Maze::checkPath()
{
    if (final_path)
    {
        return true;
    }
    else
    {
        final_path = true;
        return false;
    }
}

void Maze::printDistBoard()
{
    for (unsigned int i = 0; i < MazeSize; ++i)
    {
        for (unsigned int j = 0; j < MazeSize; ++j)
        {
            Serial7.printf("%d\t", dist_board.at(i).at(j));
        }
        Serial7.print("\r\n");
    }
}

void Maze::printVertical()
{
    for (unsigned int i = 0; i < MazeSize + 1; ++i)
    {
        for (unsigned int j = 0; j < MazeSize; ++j)
        {
            Serial7.printf("%d\t", vertical.at(i).at(j));
        }
        Serial7.print("\r\n");
    }
}

void Maze::printHorizontal()
{
    for (unsigned int i = 0; i < MazeSize; ++i)
    {
        for (unsigned int j = 0; j < MazeSize + 1; ++j)
        {
            Serial7.printf("%d\t", horizontal.at(i).at(j));
        }
        Serial7.print("\r\n");
    }
}