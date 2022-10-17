#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
	return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
	return false;
}

bool ProjectTwo::implemented_jps_plus()
{
	return false;
}
#pragma endregion

bool AStarPather::initialize()
{
	// handle any one-time setup requirements you have

	/*
		If you want to do any map-preprocessing, you'll need to listen
		for the map change message.  It'll look something like this:

		Callback cb = std::bind(&AStarPather::your_function_name, this);
		Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

		There are other alternatives to using std::bind, so feel free to mix it up.
		Callback is just a typedef for std::function<void(void)>, so any std::invoke'able
		object that std::function can wrap will suffice.
	*/

	//Initialize Nodes

	//InitializeNodes();

	return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{
	/*
		Free any dynamically allocated memory or any other general house-
		keeping you need to do during shutdown.
	*/
}



PathResult AStarPather::compute_path(PathRequest& request)
{
	/*
		This is where you handle pathing requests, each request has several fields:

		start/goal - start and goal world positions
		path - where you will build the path upon completion, path should be
			start to goal, not goal to start
		heuristic - which heuristic calculation to use
		weight - the heuristic weight to be applied
		newRequest - whether this is the first request for this path, should generally
			be true, unless single step is on

		smoothing - whether to apply smoothing to the path
		rubberBanding - whether to apply rubber banding
		singleStep - whether to perform only a single A* step
		debugColoring - whether to color the grid based on the A* state:
			closed list nodes - yellow
			open list nodes - blue

			use terrain->set_color(row, col, Colors::YourColor);
			also it can be helpful to temporarily use other colors for specific states
			when you are testing your algorithms

		method - which algorithm to use: A*, Floyd-Warshall, JPS+, or goal bounding,
			will be A* generally, unless you implement extra credit features

		The return values are:
			PROCESSING - a path hasn't been found yet, should only be returned in
				single step mode until a path is found
			COMPLETE - a path to the goal was found and has been built in request.path
			IMPOSSIBLE - a path from start to goal does not exist, do not add start position to path
	*/

	// WRITE YOUR CODE HERE
	//GridPos startPosInGrid = terrain->get_grid_position(request.start);
	//GridPos goalPosInGrid = terrain->get_grid_position(request.goal);
	//if(ogStart != startPosInGrid || ogGoal != goalPosInGrid)
	


	// Just sample code, safe to delete
	GridPos start = terrain->get_grid_position(request.start);
	GridPos goal = terrain->get_grid_position(request.goal);


	if (result != PathResult::PROCESSING)
	{
		InitializeNodes();

		Node startNode = nodes[start.col][start.row];

		AddOnList(onList::OPEN, startNode);
	}
	
	while (!openList.empty())
	{
		Node parentNode = SortOpenListAndPop();

		if (parentNode.pos == goal)
		{
			Node startNode = nodes[start.col][start.row];
			SetPath(request.path, parentNode, startNode, request.settings.rubberBanding,
				request.settings.smoothing);
			result = PathResult::COMPLETE;
			return result;
		}

		//Find neighboring child nodes
		std::vector<NeighborKind> neighborKinds;
		std::vector<GridPos> neighboringGrids = GetNeighboringChildPoses(parentNode.pos, neighborKinds);

		const size_t vecSize = neighboringGrids.size();
		for (int i = 0; i < vecSize; ++i)
		{
			const GridPos childPos = neighboringGrids[i];
			const NeighborKind kind = neighborKinds[i];

			Node childNode = nodes[childPos.col][childPos.row];


			if(kind == NeighborKind::DOWN || kind == NeighborKind::UP || kind == NeighborKind::LEFT || kind == NeighborKind::RIGHT)
				childNode.given = parentNode.given + 1.f;
			else
				childNode.given = parentNode.given + static_cast<float>(sqrt2);

			childNode.cost = childNode.given +
				HeuristicCost(request.settings.heuristic, childNode.pos, goal) * request.settings.weight;
			childNode.xParent = parentNode.pos.row;
			childNode.yParent = parentNode.pos.col;

 			const onList& childNodeStatus = childNode.list;

			//Child node is not on the list
			if (childNodeStatus == onList::NONE)
			{
				AddOnList(onList::OPEN, childNode);
			}
			//Means child node is on the open / closed list
			else
			{
				//Find that child node
				Node otherChildNode;

				if (childNodeStatus == onList::OPEN)
					otherChildNode = *std::find(openList.begin(), openList.end(), childNode);
				else
					otherChildNode = *std::find(closedList.begin(), closedList.end(), childNode);

				if (childNode.cost < otherChildNode.cost)
				{
					//take the old expensive one off both lists
					if (childNodeStatus == onList::OPEN)
						DeleteOnList(onList::OPEN, childNode);
					else
						DeleteOnList(onList::CLOSED, childNode);

					//put this new cheaper one on the open list
					AddOnList(onList::OPEN, childNode);
				}


			}
		}

		AddOnList(onList::CLOSED, parentNode);

		if(request.settings.debugColoring)
			SetColor();

		if (request.settings.singleStep)
		{
			result = PathResult::PROCESSING;
			return result;
		}
	}
	result = PathResult::IMPOSSIBLE;
	return result;
}

AStarPather::Node AStarPather::SortOpenListAndPop()
{
	openList.sort([](const Node& A, const Node& B)
		{
			return A.cost < B.cost;
		});

	const Node frontNode = openList.front();
	openList.pop_front();

	return frontNode;
}

double AStarPather::HeuristicCost(Heuristic setting, GridPos curr, GridPos target)
{
	switch (setting)
	{
	case Heuristic::OCTILE:
	{
		const int xDiff = abs(curr.row - target.row);
		const int yDiff = abs(curr.col - target.col);

		const double firstVal = std::min(xDiff, yDiff) * sqrt2;
		const int secondVal = std::max(xDiff, yDiff);
		const int thirdVal = std::min(xDiff, yDiff);

		return firstVal + secondVal - thirdVal;
	}
	case Heuristic::CHEBYSHEV:
	{
		const int xDiff = abs(curr.row - target.row);
		const int yDiff = abs(curr.col - target.col);

		return std::max(xDiff, yDiff);
	}
	case Heuristic::MANHATTAN:
	{
		const int xDiff = abs(curr.row - target.row);
		const int yDiff = abs(curr.col - target.col);

		return xDiff + yDiff;
	}
	case Heuristic::EUCLIDEAN:
	{
		const int xDiff = abs(curr.row - target.row);
		const int yDiff = abs(curr.col - target.col);

		return sqrt(xDiff * xDiff + yDiff * yDiff);
	}
	case Heuristic::NUM_ENTRIES:
	default:
		return 0.f;
	}

}

std::vector<GridPos> AStarPather::GetNeighboringChildPoses(GridPos parentPos, std::vector<NeighborKind>& neighborKinds)
{
	std::vector<GridPos> result;

	//Up
	GridPos targetPos = GridPos{parentPos.row, parentPos.col + 1};

	const bool isUpMovable = isItMovableGrid(targetPos);

	if (isUpMovable)
	{
		result.push_back(targetPos);
		neighborKinds.push_back(NeighborKind::UP);
	}

	//Down
	targetPos = GridPos{ parentPos.row, parentPos.col - 1 };

	const bool isDownMovable = isItMovableGrid(targetPos);

	if (isDownMovable)
	{
		result.push_back(targetPos);
		neighborKinds.push_back(NeighborKind::DOWN);
	}

	//Left
	targetPos = GridPos{ parentPos.row - 1, parentPos.col };

	const bool isLeftMovable = isItMovableGrid(targetPos);

	if (isLeftMovable)
	{
		result.push_back(targetPos);
		neighborKinds.push_back(NeighborKind::LEFT);
	}

	//Right
	targetPos = GridPos{ parentPos.row + 1, parentPos.col };

	const bool isRightMovable = isItMovableGrid(targetPos);

	if (isRightMovable)
	{
		result.push_back(targetPos);
		neighborKinds.push_back(NeighborKind::RIGHT);
	}



	//Diagonal
	//UpLeft
	//Should check up / left movable
	if (isLeftMovable && isUpMovable)
	{
		targetPos = GridPos{ parentPos.row - 1, parentPos.col + 1 };

		const bool isLeftUpMovable = isItMovableGrid(targetPos);

		if (isLeftUpMovable)
		{
			result.push_back(targetPos);
			neighborKinds.push_back(NeighborKind::UPLEFT);
		}
	}

	//UpRight
	if (isRightMovable && isUpMovable)
	{
		targetPos = GridPos{ parentPos.row + 1, parentPos.col + 1 };

		const bool isRightUpMovable = isItMovableGrid(targetPos);

		if (isRightUpMovable)
		{
			result.push_back(targetPos);
			neighborKinds.push_back(NeighborKind::UPRIGHT);
		}
	}

	//DownLeft
	if (isLeftMovable && isDownMovable)
	{
		targetPos = GridPos{ parentPos.row - 1, parentPos.col - 1 };

		const bool isLeftDownMovable = isItMovableGrid(targetPos);

		if (isLeftDownMovable)
		{
			result.push_back(targetPos);
			neighborKinds.push_back(NeighborKind::DOWNLEFT);
		}
	}

	//DownRight
	if (isRightMovable && isDownMovable)
	{
		targetPos = GridPos{ parentPos.row + 1, parentPos.col - 1 };

		const bool isRightDownMovable = isItMovableGrid(targetPos);

		if (isRightDownMovable)
		{
			result.push_back(targetPos);
			neighborKinds.push_back(NeighborKind::DOWNRIGHT);
		}
	}

	return result;
}

void AStarPather::SetColor()
{
	for (const auto element : openList)
	{
		terrain->set_color(element.pos.row, element.pos.col, Color(0.f, 0.f, 1.f));
	}

	for (const auto element : closedList)
	{
		terrain->set_color(element.pos.row, element.pos.col, Color(1.f, 1.f, 0.f));
	}

}

void AStarPather::AddOnList(onList type, Node node)
{
	node.list = type;
	nodes[node.pos.col][node.pos.row] = node;

	if (type == onList::OPEN)
		openList.push_back(node);
	else if (type == onList::CLOSED)
		closedList.push_back(node);
}

void AStarPather::DeleteOnList(onList type, Node node)
{
	if (type == onList::OPEN)
	{
		openList.remove(node);
	}
	else if (type == onList::CLOSED)
	{
		closedList.remove(node);
	}

	nodes[node.pos.col][node.pos.row].list = onList::NONE;
}

void AStarPather::SetPath(WaypointList& list, const Node& goalNode, const Node& startNode, bool enableRubberbanding,
	bool enableSmoothing)
{
	Node tempNode = goalNode;
	std::vector<Vec3> pathLists;

	while (!(tempNode == startNode))
	{
		Vec3 check = terrain->get_world_position(tempNode.pos);

		pathLists.push_back(check);

		tempNode = nodes[tempNode.yParent][tempNode.xParent];
	}


	if (pathLists.size() > 2 && enableRubberbanding)
	{
		auto beginElement = pathLists.begin();

		while (beginElement != pathLists.end() - 2)
		{
			GridPos first = terrain->get_grid_position(*beginElement);
			GridPos second = terrain->get_grid_position(*(beginElement + 1));
			GridPos third = terrain->get_grid_position(*(beginElement + 2));

			const int minRow = std::min(std::min(first.row, second.row), std::min(second.row, third.row));
			const int minCol = std::min(std::min(first.col, second.col), std::min(second.col, third.col));
			const int maxRow = std::max(std::max(first.row, second.row), std::max(second.row, third.row));
			const int maxCol = std::max(std::max(first.col, second.col), std::max(second.col, third.col));

			bool canMerge = true;

			for (int y = minCol; y <= maxCol; ++y)
			{
				for (int x = minRow; x <= maxRow; ++x)
				{
					if (terrain->is_wall(x, y))
						canMerge = false;
				}
			}

			if (canMerge)
			{
				pathLists.erase(beginElement + 1);
			}
			else
			{
				++beginElement;
			}
		}
	}
	const size_t listSize = pathLists.size();

	if(!enableSmoothing)
	{
		for (size_t i = 0; i < listSize; ++i)
		{
			list.push_front(pathLists[i]);
		}
		list.push_front(terrain->get_world_position(startNode.pos));
	}
	else
	{

		for(size_t i = 0; i < listSize - 2; ++i)
		{
			Vec3 v1, v2, v3, v4;

			if(i == 0)
			{
				v1 = pathLists[i];
				v2 = pathLists[i];
				v3 = pathLists[i + 1];
				v4 = pathLists[i + 2];
			}
			else if(i == listSize - 3)
			{
				v1 = pathLists[i];
				v2 = pathLists[i + 1];
				v3 = pathLists[i + 2];
				v4 = pathLists[i + 2];
			}
			else
			{
				v1 = pathLists[i];
				v2 = pathLists[i + 1];
				v3 = pathLists[i + 2];
				v4 = pathLists[i + 3];
			}

			for(float t = 0.f; t <= 1.f; t += 0.25f)
			{
				Vec3 val = Vec3::CatmullRom(v1, v2, v3, v4, t);

				list.push_front(val);
			}
		}
	}


}

bool AStarPather::isItMovableGrid(const GridPos& pos)
{
	return terrain->is_valid_grid_position(pos) && !terrain->is_wall(pos);
}

void AStarPather::InitializeNodes()
{
	openList.clear();
	closedList.clear();

	const int mapHeight = terrain->get_map_height();
	const int mapWidth = terrain->get_map_width();

	if (nodes.size() != mapHeight)
	{
		nodes.resize(mapHeight);

		for (int i = 0; i < mapHeight; ++i)
		{
			nodes[i].resize(mapWidth);
		}
	}


	for (int i = 0; i < mapHeight; ++i)
	{
		for (int j = 0; j < mapWidth; ++j)
		{
			nodes[i][j].xParent = 0;
			nodes[i][j].yParent = 0;
			nodes[i][j].cost = 0.f;
			nodes[i][j].given = 0.f;
			nodes[i][j].list = onList::NONE;
			nodes[i][j].pos = GridPos{j, i};
		}
	}
}


