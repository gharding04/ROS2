#define ROW 9
#define COL 10

#include <bits/stdc++.h>

struct Point{
    int x;
    int y;
};

struct cell{
    Point parent;
    double totalCost, cost, heuristic;
};

typedef std::pair<int, int> Coord;
 
typedef std::pair<double, std::pair<int, int> > Node;
 
class Search{
    public:

    int row = ROW, col = COL;

    cell cells[ROW][COL];

    bool closedList[ROW][COL];

	std::set<Node> openList;

    int map[ROW][COL];
    int startX, startY, destX, destY;
    double newCost, newH, newTotal;

    void initializeMap();

    void initializeMap(float width);

    void setMap(int map[][COL]);

    void setObstacle(int x, int y, int type);

    void setOpen(int x, int y);

    bool isValid(int x, int y);

    bool isOpen(int x, int y, bool includeHoles = false);

    bool isDestination(int x, int y);

    double calculateHeuristic(int x, int y);

    void setStart(Point start);

    void setDest(Point dest);

    void initializeCells();

    bool checkSuccessor(int i, int deltaX, int j, int deltaY, bool includeHoles = false);

    void printPath(std::stack<Coord> Path);

    std::stack<Coord> getPath();

    std::stack<Coord> aStar(bool includeHoles = false);

    std::stack<Coord> aStar(Point src, Point dest, bool includeHoles = false);
    
    std::stack<Coord> aStar(int grid[][COL], Point src, Point dest, bool includeHoles = false);

    std::stack<Coord> getSimplifiedPath(std::stack<Coord> rpath);
};