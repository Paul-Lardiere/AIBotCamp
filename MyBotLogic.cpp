#include "MyBotLogic.h"

#include "Globals.h"
#include "ConfigData.h"
#include "InitData.h"
#include "TurnData.h"
#include <algorithm>

#define LOG(x) BOT_LOGIC_LOG(mLogger, x, true)
#define LOGLOG(x) { std::stringstream ss; ss << x; LOG(ss.str().c_str()); }
#define ASSERT(x) ASSERT_RET(x, )
#define ASSERT_RET(x, ret)\
    if(!(x)) {\
        LOG("Broken assertion: " #x);\
        return ret;\
    }

#ifdef GLOBAL_LOGGER
Logger mLogger;
#endif

#ifdef _DEBUG
template<class A, class B>
std::basic_ostream<A, B>& operator<<(std::basic_ostream<A, B>& s, const std::pair<int, int>& p)
{
	return s << "(" << p.first << "," << p.second << ")";
}
template<class A, class B, class C>
std::basic_ostream<A, B>& operator<<(std::basic_ostream<A, B>& s, const std::vector<C>& p)
{
	s << "(";
	for (const auto& c : p) s << c << ",";
	return s << ")";
}
#endif

MyBotLogic::MyBotLogic()
{
	//Write Code Here
}

MyBotLogic::~MyBotLogic()
{
	//Write Code Here
}

void MyBotLogic::Configure(const SConfigData& _configData)
{
#ifdef BOT_LOGIC_DEBUG
	mLogger.Init(_configData.logpath, "MyBotLogic.log");
#endif

	BOT_LOGIC_LOG(mLogger, "Configure", true);

	//Write Code Here
}

void MyBotLogic::Init(const SInitData& _initData)
{
	BOT_LOGIC_LOG(mLogger, "Init", true);
	_graph.InitGraph(_initData.tileInfoArraySize, _initData.tileInfoArray, _initData.objectInfoArray, _initData.objectInfoArraySize);
	BOT_LOGIC_LOG(mLogger, _graph.printGraph(), true);

	// Calculate the closest goal to each npc without doubled ones
	SNPCInfo* npcCurrent = _initData.npcInfoArray;

	for (int i = 0; i < _initData.nbNPCs; i++)
	{
		_goalForEachNpc[npcCurrent->uid] = _graph.GetClosestGoalInfo(npcCurrent);
		npcCurrent++;
	}

	// Reinitialisation of the NPC
	npcCurrent = _initData.npcInfoArray;

	
	// Calculate the path for each NPC
	for (int i = 0; i < _initData.nbNPCs; i++)
	{
		_pathForEachNpc[npcCurrent->uid] = PathFinderAStar(npcCurrent, Heuristic{ _goalForEachNpc[npcCurrent->uid] });
		_pathPositionForEachNpc[npcCurrent->uid] = 0;
		npcCurrent++;
	}
}

void MyBotLogic::GetTurnOrders(const STurnData& _turnData, std::list<SOrder>& _orders)
{
	/*
	SNPCInfo* npcCurrent = _turnData.npcInfoArray;

	for (int i = 0; i < _turnData.npcInfoArraySize; i++)
	{
		if (_pathPositionForEachNpc[npcCurrent->uid] < _pathForEachNpc[npcCurrent->uid].size())
		{
			EHexCellDirection dir = _pathForEachNpc[npcCurrent->uid][_pathPositionForEachNpc[npcCurrent->uid]];

			//if (MovementPossible(_turnData.npcInfoArray, _turnData.npcInfoArraySize, npcCurrent))
			{
				_pathPositionForEachNpc[npcCurrent->uid] += 1;

				// Give the order to the npc
				SOrder order = { EOrderType::Move, npcCurrent->uid, dir };
				_orders.push_back(order);
			}
		}
		npcCurrent++;
	}
	*/
}


std::vector<EHexCellDirection> MyBotLogic::PathFinderAStar(SNPCInfo* npcCurrent, Heuristic heuristic)
{
	// Get the coordinates of the npc's goal
	Graph::coordinates goalCoordinates = _goalForEachNpc[npcCurrent->uid];
	
	// Initialize the record for the start node
	Node startNode = *(_graph.getNode(Graph::coordinates{npcCurrent->q, npcCurrent->r}));
	startNode.setCost_so_far(goalCoordinates, 0.f);
	startNode.setHeuristic(goalCoordinates, heuristic.estimate(Graph::coordinates{npcCurrent->q, npcCurrent->r}));
	
	// Initializing the record vectors
	std::vector<Node> openNodes{};
	openNodes.push_back(startNode);
	std::vector<Node> closedNodes{};
	Node currentNode;

	while (!openNodes.empty())
	{
		// Find the smallest element in the open list
		currentNode = FindClosestNode(openNodes, goalCoordinates);

		// If it is a goal node, then terminate
		//if (currentNode.tile.type == EHexCellType::Goal)
		//	break;

		// Otherwise get its outgoing connections
		Node::adjencyList adjacentNodes = currentNode.getAdjencyList();

		// Loop through each connections
		for (std::pair<EHexCellDirection, Node*> currentAdjencyNode : adjacentNodes)
		{
			// Get the estimated cost to the end tile
			Node endNode = *currentAdjencyNode.second;
			float endNodeCostSoFar = currentNode.getCost_so_far(goalCoordinates) + 1;

			float endNodeHeuristic;

			// If the end tile is closed we may have to skip or remove it from the closed list
			if (find(begin(closedNodes), end(closedNodes), endNode) != end(closedNodes))
			{
				// Find the record corresponding in the closedNodes vector
				// endTileRecord = find_if(begin(closedNodes), end(closedNodes), FindTile{ endTile })[0];

				// if we didn't find a shorter route, skip
				if (endNode.getCost_so_far(goalCoordinates) <= endNodeCostSoFar)
					continue;

				// Otherwise, remove it from the closed list
				openNodes.erase(std::remove(begin(closedNodes), end(closedNodes), endNode), end(closedNodes));

				// Calculate the heuristic without calling the function again
				endNodeHeuristic = endNode.getTotalEstimatedCost(goalCoordinates) - endNode.getCost_so_far(goalCoordinates);
			}
			// skip if the tile is open and we've not found a better route
			else if (find(begin(openNodes), end(openNodes), endNode) != end(openNodes))
			{
				// Find the record corresponding in the closedNodes vector
				// endTileRecord = find_if(begin(openNodes), end(openNodes), FindTile{ endTile })[0];

				// If our route is not better then skip
				if (endNode.getCost_so_far(goalCoordinates) <= endNodeCostSoFar)
					continue;

				// Calculate the heuristic without calling the function again
				endNodeHeuristic = endNode.getTotalEstimatedCost(goalCoordinates) - endNode.getCost_so_far(goalCoordinates);
			}
			// Otherwise, we know that we have an unvisited node so we can create the record for it
			else
			{
				endNodeHeuristic = heuristic.estimate(endNode.getNodeCoordinates());
			}

			// Update the record
			endNode.setCost_so_far(goalCoordinates, endNodeCostSoFar);
//			endTileRecord.connection = currentConnection;
			endNode.setHeuristic(goalCoordinates, endNodeHeuristic);

			// Add the Record to the open list
			// TODO : Modifier pour ne trouver que les node qui ont les mêmes coordonnees et pas forcement celle avec la meme exact node
			if (find(begin(openNodes), end(openNodes), endNode) == end(openNodes))
			{
				openNodes.push_back(endNode);
			}
		}

		// We have finished looking at the connections for the current node, so we can add it to the closed ones
		openNodes.erase(std::remove(begin(openNodes), end(openNodes), currentNode), end(openNodes));

		closedNodes.push_back(currentNode);
	}
	// Find if we have found the goal or we ran out of tiles
	if (currentNode.getTileInfo().type != EHexCellType::Goal)
	{
		return std::vector<EHexCellDirection>{}; // No solutions
	}
	else
	{
		// compile the list of connections in the path
		std::vector<EHexCellDirection> path{};

		while (currentNode != startNode)
		{
			// Search for the next tile closest to the start 
			Node::adjencyList adjencyList = currentNode.getAdjencyList();

			for (std::pair<EHexCellDirection, Node*> nodeAdjency : adjencyList)
			{
				if (nodeAdjency.second->getCost_so_far(goalCoordinates) == currentNode.getCost_so_far(goalCoordinates) - 1)
				{
					path.push_back(allDirectionReversed[nodeAdjency.first]);
					currentNode = *nodeAdjency.second;
					break;
				}
			}
		}

		std::reverse(begin(path), end(path));

		return path;
	}
}

Node MyBotLogic::FindClosestNode(std::vector<Node> nodes, Graph::coordinates goalCoordinates)
{
	float minEstimatedTotalCost = nodes[0].getTotalEstimatedCost(goalCoordinates);
	int indexMin = 0;

	for (int i = 1; i < nodes.size(); i++)
	{
		if (nodes[i].getTotalEstimatedCost(goalCoordinates) < minEstimatedTotalCost)
		{
			indexMin = i;
			minEstimatedTotalCost = nodes[i].getTotalEstimatedCost(goalCoordinates);
		}
	}

	return nodes[indexMin];
}