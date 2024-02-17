#define ROW 50
#define COL 68

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

    bool isOpen(int x, int y);

    bool isDestination(int x, int y);

    double calculateHeuristic(int x, int y);

    void setStart(Point start);

    void setDest(Point dest);

    void initializeCells();

    bool checkSuccessor(int i, int deltaX, int j, int deltaY);

    void printPath(std::stack<Coord> Path);

    std::stack<Coord> getPath();

    std::stack<Coord> aStar(Point src, Point dest);
    
    std::stack<Coord> aStar(int grid[][COL], Point src, Point dest);

    std::stack<Coord> getSimplifiedPath(std::stack<Coord> rpath);
};