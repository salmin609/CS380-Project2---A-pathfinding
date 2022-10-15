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

    struct Node
    {
        GridPos pos;
        int xParent, yParent;
        float cost;
        float given;
        onList list;

        bool operator==(const Node& other) const
        {
            return pos == other.pos;
        }

        //debug
        //int indexW, indexH;
    };

    Node SortOpenListAndPop();
    float HeuristicCost(Heuristic setting, GridPos curr, GridPos target);
    double sqrt2 = sqrt(2);
    std::vector<GridPos> GetNeighboringChildPoses(GridPos parentPos);
    void SetColor();
    void AddOnList(onList type, Node node);
    void DeleteOnList(onList type, Node node);
    void SetPath(WaypointList& list, const Node& goalNode
        , const Node& startNode);
    bool isItMovableGrid(const GridPos& pos);
    void InitializeNodes();

    std::vector<std::vector<Node>> nodes;
    std::list<Node> openList;
    std::list<Node> closedList;
    PathResult result = PathResult::COMPLETE;

    //GridPos ogStart{999, 999};
    //GridPos ogGoal{ 999, 999 };

};