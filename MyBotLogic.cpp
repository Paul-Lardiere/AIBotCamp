#include "MyBotLogic.h"

#include "Globals.h"
#include "ConfigData.h"
#include "InitData.h"
#include "TurnData.h"
#include <algorithm>
#include <format>

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
	SNPCInfo* npcCurrent = _initData.npcInfoArray;

	BOT_LOGIC_LOG(mLogger, "Init", true);
	_graph.InitGraph(_initData.tileInfoArraySize, _initData.tileInfoArray, _initData.objectInfoArray, _initData.objectInfoArraySize, _initData.npcInfoArray, _initData.nbNPCs);
	BOT_LOGIC_LOG(mLogger, _graph.printGraph(), true);

	maxTourNb = _initData.maxTurnNb;

	/*
	// Reinitialisation of the NPC
	npcCurrent = _initData.npcInfoArray;
	maxTourNb = _initData.maxTurnNb;
	// Calculate the path for each NPC
	for (int i = 0; i < _initData.nbNPCs; i++)
	{
		_pathForEachNpc[npcCurrent[i].uid] = PathFinderAStar(npcCurrent[i], Heuristic{ _goalForEachNpc[npcCurrent[i].uid] }, maxTourNb);
		_pathPositionForEachNpc[npcCurrent[i].uid] = 0;
		_graph.setOccupiedNode(coordinates{ npcCurrent[i].q, npcCurrent[i].r }, true);
	}
	*/
	
}

void MyBotLogic::GetTurnOrders(const STurnData& _turnData, std::list<SOrder>& _orders)
{
	//BOT_LOGIC_LOG(mLogger, std::format("graph:"), true);
	//for (int i = 0; i < _turnData.tileInfoArraySize; ++i)
	//{
	//	BOT_LOGIC_LOG(mLogger, std::format("({},{}), type:{}", _turnData.tileInfoArray[i].q, _turnData.tileInfoArray[i].r, static_cast<int>(_turnData.tileInfoArray[i].type)), true);
	//}



	SNPCInfo* npcCurrent = _turnData.npcInfoArray;
	_graph.updateGraph(_turnData.tileInfoArraySize, _turnData.tileInfoArray, _turnData.objectInfoArray, _turnData.objectInfoArraySize, _turnData.npcInfoArray, _turnData.npcInfoArraySize);
	
	/*BOT_LOGIC_LOG(mLogger, _graph.printGraph(), true);
	BOT_LOGIC_LOG(mLogger, std::format("{}",_goalForEachNpc.size()), true);	

	BOT_LOGIC_LOG(mLogger, std::format("{}", _graph._goals.size()), true);*/


	
	if (!_graph.hasEnoughGoals(_turnData.npcInfoArraySize)) {
		BOT_LOGIC_LOG(mLogger, "exploration", true);

		exploration(_turnData,_orders);
	}
	else
	{
		if (_goalForEachNpc.size() < _turnData.npcInfoArraySize)
		{
			for (int i = 0; i < _turnData.npcInfoArraySize; i++)
			{
				coordinates coordGoal = _graph.GetClosestGoalInfo(npcCurrent[i]);
				if (coordGoal != coordinates{-1, -1})
					_goalForEachNpc[npcCurrent[i].uid] = coordGoal;
			}
			// Reinitialisation of the NPC
			npcCurrent = _turnData.npcInfoArray;
			// Calculate the path for each NPC
			for (int i = 0; i < _turnData.npcInfoArraySize; i++)
			{
				_pathForEachNpc[npcCurrent[i].uid] = PathFinderAStar(npcCurrent[i], Heuristic{ _goalForEachNpc[npcCurrent[i].uid] }, maxTourNb);
				_pathPositionForEachNpc[npcCurrent[i].uid] = 0;
				_graph.setOccupiedNode(coordinates{ npcCurrent[i].q, npcCurrent[i].r }, true);
			}
		}

		for (int i = 0; i < _turnData.npcInfoArraySize; i++)
		{
			if (_pathPositionForEachNpc[npcCurrent[i].uid] < _pathForEachNpc[npcCurrent[i].uid].size())
			{
				EHexCellDirection dir = _pathForEachNpc[npcCurrent[i].uid][_pathPositionForEachNpc[npcCurrent[i].uid]];

				coordinates coordDest = getCoordinatesDirection(coordinates{ npcCurrent[i].q, npcCurrent[i].r }, dir);

				if (_graph.isFinished(coordDest))
				{
					// Calcul again A*
					_pathForEachNpc[npcCurrent[i].uid] = PathFinderAStar(npcCurrent[i], Heuristic{ _goalForEachNpc[npcCurrent[i].uid] }, maxTourNb);
					_pathPositionForEachNpc[npcCurrent[i].uid] = 0;
				}
				else if (!_graph.IsNodeOccupied(coordDest))
				{
					// We free the old node
					_graph.setOccupiedNode(coordinates{ npcCurrent[i].q, npcCurrent[i].r }, false);
					// We go on the next node
					_graph.setOccupiedNode(coordDest, true);
					_pathPositionForEachNpc[npcCurrent[i].uid] += 1;

					if (_pathPositionForEachNpc[npcCurrent[i].uid] == _pathForEachNpc[npcCurrent[i].uid].size())
						_graph.setFinished(coordDest);

					// Give the order to the npc
					SOrder order = { EOrderType::Move, npcCurrent[i].uid, dir };
					_orders.push_back(order);
				}
			}
		}
	}
}


std::vector<EHexCellDirection> MyBotLogic::PathFinderAStar(SNPCInfo npcCurrent, Heuristic heuristic, int maxTurnNb)
{
	// Get the coordinates of the npc's goal
	Graph::coordinates goalCoordinates = _goalForEachNpc[npcCurrent.uid];
	
	// Initialize the record for the start node
	Node* startNode = (_graph.getNode(Graph::coordinates{npcCurrent.q, npcCurrent.r}));
	startNode->setCost_so_far(goalCoordinates, 0.f);
	startNode->setHeuristic(goalCoordinates, heuristic.estimate(Graph::coordinates{npcCurrent.q, npcCurrent.r}));

	// Initializing the record vectors
	std::vector<Node*> openNodes{};
	openNodes.push_back(startNode);
	std::vector<Node*> closedNodes{};
	Node* currentNode;

	while (!openNodes.empty())
	{
		// Find the smallest element in the open list
		currentNode = FindClosestNode(openNodes, goalCoordinates);

		// If it is a goal node, then terminate
		if (currentNode->getTileInfo().type == EHexCellType::Goal)
			break;

		// Otherwise get its outgoing connections
		Node::adjencyList adjacentNodes = currentNode->getAdjencyList();

		// Loop through each connections
		for (std::pair<EHexCellDirection, Node*> currentAdjencyNode : adjacentNodes)
		{
			if (currentAdjencyNode.second->finished)
				continue;

			// Get the estimated cost to the end tile
			Node* endNode = currentAdjencyNode.second;
			float endNodeCostSoFar = currentNode->getCost_so_far(goalCoordinates) + 1;

			if (endNodeCostSoFar > maxTurnNb)
				continue;

			float endNodeHeuristic;

			// If the end tile is closed we may have to skip or remove it from the closed list
			if (find(begin(closedNodes), end(closedNodes), endNode) != end(closedNodes))
			{
				// if we didn't find a shorter route, skip
				if ((endNode->getCost_so_far(goalCoordinates) <= endNodeCostSoFar))
					continue;
				
				// Otherwise, remove it from the closed list
				closedNodes.erase(std::remove(begin(closedNodes), end(closedNodes), endNode), end(closedNodes));

				// Calculate the heuristic without calling the function again
				endNodeHeuristic = endNode->getTotalEstimatedCost(goalCoordinates) - endNode->getCost_so_far(goalCoordinates);
			}
			// skip if the tile is open and we've not found a better route
			else if (find(begin(openNodes), end(openNodes), endNode) != end(openNodes))
			{
				// If our route is not better then skip
				if ((endNode->getCost_so_far(goalCoordinates) <= endNodeCostSoFar))
					continue;

				// Otherwise, remove it from the closed list
				openNodes.erase(std::remove(begin(openNodes), end(openNodes), endNode), end(openNodes));

				// Calculate the heuristic without calling the function again
				endNodeHeuristic = endNode->getTotalEstimatedCost(goalCoordinates) - endNode->getCost_so_far(goalCoordinates);
			}
			// Otherwise, we know that we have an unvisited node so we can create the record for it
			else
			{
				endNodeHeuristic = heuristic.estimate(endNode->getNodeCoordinates());
			}

			// Update the record
			endNode->setCost_so_far(goalCoordinates, endNodeCostSoFar);
			endNode->setHeuristic(goalCoordinates, endNodeHeuristic);

			// Add the Record to the open list
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
	if (currentNode->getTileInfo().type != EHexCellType::Goal)
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
			Node::adjencyList adjencyList = currentNode->getAdjencyList();

			for (std::pair<EHexCellDirection, Node*> nodeAdjency : adjencyList)
			{
				if (nodeAdjency.second->getCost_so_far(goalCoordinates) == currentNode->getCost_so_far(goalCoordinates) - 1)
				{
					path.push_back(allDirectionReversed[nodeAdjency.first]);
					currentNode = nodeAdjency.second;
					break;
				}
			}
		}

		std::reverse(begin(path), end(path));

		return path;
	}
}

Node* MyBotLogic::FindClosestNode(std::vector<Node*> nodes, Graph::coordinates goalCoordinates)
{
	float minEstimatedTotalCost = nodes[0]->getTotalEstimatedCost(goalCoordinates);
	int indexMin = 0;

	for (int i = 1; i < nodes.size(); i++)
	{
		if (nodes[i]->getTotalEstimatedCost(goalCoordinates) < minEstimatedTotalCost)
		{
			indexMin = i;
			minEstimatedTotalCost = nodes[i]->getTotalEstimatedCost(goalCoordinates);
		}
	}

	return nodes[indexMin];
}

void MyBotLogic::exploration(const STurnData& turnData, std::list<SOrder>& _orders)
{


	for (int i = 0; i < turnData.npcInfoArraySize; ++i) {
		coordinates coordNPC{ turnData.npcInfoArray[i].q, turnData.npcInfoArray[i].r };
		Node *node = _graph.getNode(coordNPC);
		Node::adjencyList adjacentNodes = node->getAdjencyList();

		int mini = turnData.tileInfoArraySize;
		EHexCellDirection direction = W;

		// Loop through each connections
		for (std::pair<EHexCellDirection, Node*> currentAdjencyNode : adjacentNodes)
		{
			if (currentAdjencyNode.second->timesExplored < mini && !currentAdjencyNode.second->isOccupied())
			{
				mini = currentAdjencyNode.second->timesExplored;
				direction = currentAdjencyNode.first;
			}
		}

		SOrder order = { EOrderType::Move, turnData.npcInfoArray[i].uid, direction};
		_graph.addTimesExplored(getCoordinatesDirection(coordNPC, direction ));
		_graph.setOccupiedNode(coordNPC, false);
		_graph.setOccupiedNode(getCoordinatesDirection(coordNPC, direction), true);
		_orders.push_back(order);
	}

}

MyBotLogic::coordinates MyBotLogic::getCoordinatesDirection(coordinates coordinate, EHexCellDirection direction)
{
	coordinates coordDest = coordinate;
	switch (direction)
	{
	case W:
		coordDest.second--;
		break;
	case NW:
		coordDest.first--;
		break;
	case NE:
		coordDest.first--;
		coordDest.second++;
		break;
	case E:
		coordDest.second++;
		break;
	case SE:
		coordDest.first++;
		break;
	case SW:
		coordDest.first++;
		coordDest.second--;
		break;
	default:
		break;
	}

	return coordDest;
}
