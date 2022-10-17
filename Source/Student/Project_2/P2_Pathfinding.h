#pragma once
#include "Misc/PathfindingDetails.hpp"

class AStarPather
{
public:
    /* 
        The class should be default constructible, so you may need to define a constructor.
        If needed, you can modify the framework where the class is constructed in the
        initialize functions of ProjectTwo and ProjectThree.
    */

    /* ************************************************** */
    // DO NOT MODIFY THESE SIGNATURES
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest &request);
    /* ************************************************** */

    /*
        You should create whatever functions, variables, or classes you need.
        It doesn't all need to be in this header and cpp, structure it whatever way
        makes sense to you.
    */
#define MAPWIDTH 40
#define MAPHEIGHT 40



    enum class onList
    {
        NONE,
        OPEN,
        CLOSED,
    };

    enum class NeighborKind
    {
	    UP,
        DOWN,
        RIGHT,
        LEFT,
        UPLEFT,
        UPRIGHT,
        DOWNLEFT,
        DOWNRIGHT
    };

    struct Node
    {
        GridPos pos;
        int xParent, yParent;
        double cost;
        float given;
        onList list;

        bool operator==(const Node& other) const
        {
            return pos == other.pos;
        }
    };

    Node SortOpenListAndPop();
    double HeuristicCost(Heuristic setting, GridPos curr, GridPos target);
    std::vector<GridPos> GetNeighboringChildPoses(GridPos parentPos
        , std::vector<NeighborKind>& neighborKinds);
    void SetColor();

    void SetPath(WaypointList& list, const Node& goalNode
        , const Node& startNode, bool enableRubberbanding,
        bool enableSmoothing);
    bool isItMovableGrid(const GridPos& pos);
    void InitializeNodes();

    void AddOnList(onList type, Node node);
    void DeleteOnList(onList type, Node node);

    std::vector<std::vector<Node>> nodes;
    std::list<Node> openList;
    std::list<Node> closedList;
    PathResult result = PathResult::COMPLETE;
    double sqrt2 = sqrt(2);

};