#include "MyBotLogic.h"

#include "Globals.h"
#include "ConfigData.h"
#include "InitData.h"
#include "TurnData.h"
#include <algorithm>
#include <format>

#ifdef _DEBUG
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
	maxTourNb = -1;
	nbNPC = -1;
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

	//BOT_LOGIC_LOG(mLogger, "Configure", true);

	//Write Code Here
}

void MyBotLogic::Init(const SInitData& _initData)
{
	// Initialisation du Graph
	_graph.InitGraph(_initData.tileInfoArraySize, _initData.tileInfoArray, _initData.objectInfoArray, _initData.objectInfoArraySize, _initData.npcInfoArray, _initData.nbNPCs);

	maxTourNb = _initData.maxTurnNb;
	nbNPC = _initData.nbNPCs;
}

void MyBotLogic::GetTurnOrders(const STurnData& _turnData, std::list<SOrder>& _orders)
{
	// Update the graph in case we discover new tiles from exploration
	_graph.updateGraph(_turnData.tileInfoArraySize, _turnData.tileInfoArray, _turnData.objectInfoArray, _turnData.objectInfoArraySize, _turnData.npcInfoArray, nbNPC);
	
	if (!_graph.hasEnoughGoals(nbNPC, _turnData.npcInfoArray)) {
		_goalForEachNpc.clear();
		exploration(_turnData, _orders); // Search for goals
	}
	else
	{
		SNPCInfo* npcs = _turnData.npcInfoArray;

		// Assign the goals to each npc if it's not done
		if (!allGoalsAreAssigned()) {
			assigneGoalsToEachNPC(npcs);
			calculatePathToEachNPC(npcs);
		}

		moveEachNPC(npcs, _orders);
	}
}


std::vector<EHexCellDirection> MyBotLogic::PathFinderAStar(SNPCInfo npcCurrent, Heuristic heuristic, int maxTurnNb)
{
	// Get the coordinates of the npc's goal
	Graph::coordinates goalCoordinates = _goalForEachNpc[npcCurrent.uid];

	// Initialize the record for the start node
	Node* startNode = (_graph.getNode(Graph::coordinates{ npcCurrent.q, npcCurrent.r }));
	startNode->setCost_so_far(goalCoordinates, 0);
	startNode->setHeuristic(goalCoordinates, heuristic.estimate(Graph::coordinates{ npcCurrent.q, npcCurrent.r }));

	// Initializing the record vectors
	std::vector<Node*> openNodes{};
	openNodes.push_back(startNode);
	std::vector<Node*> closedNodes{};
	Node* currentNode{ startNode };

	while (!openNodes.empty())
	{
		// Find the smallest element in the open list
		currentNode = FindClosestNode(openNodes, goalCoordinates);

		// If it is a goal node, then terminate
		if (goalFound(currentNode))
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
			int endNodeCostSoFar = currentNode->getCost_so_far(goalCoordinates) + 1;

			if (endNodeCostSoFar > maxTurnNb)
				continue;

			int endNodeHeuristic;

			// If the end tile is closed we may have to skip or remove it from the closed list
			if (nodeVectorContains(closedNodes, endNode))
			{
				// if we didn't find a shorter route, skip
				if (!isShorterPath(endNode, endNodeCostSoFar, goalCoordinates))
					continue;

				endNodeHeuristic = eraseFromVectorAndGetHeuristic(closedNodes, endNode, goalCoordinates);
			}
			// skip if the tile is open and we've not found a better route
			else if (nodeVectorContains(openNodes, endNode))
			{
				// If our route is not better then skip
				if (!isShorterPath(endNode, endNodeCostSoFar, goalCoordinates))
					continue;

				endNodeHeuristic = eraseFromVectorAndGetHeuristic(openNodes, endNode, goalCoordinates);
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
	if (!goalFound(currentNode))
		return std::vector<EHexCellDirection>{}; // No solutions

	// compile the list of connections in the path
	std::vector<EHexCellDirection> path{};

	while (currentNode != startNode)
		currentNode = searchNextTileToTheGoal(currentNode, goalCoordinates, path);

	std::reverse(begin(path), end(path));

	return path;

}

Node* MyBotLogic::FindClosestNode(std::vector<Node*> nodes, Graph::coordinates goalCoordinates)
{
	int minEstimatedTotalCost = nodes[0]->getTotalEstimatedCost(goalCoordinates);
	int indexMin = 0;

	for (int i = 1; i < nodes.size(); i++)
		if (nodes[i]->getTotalEstimatedCost(goalCoordinates) < minEstimatedTotalCost)
		{
			indexMin = i;
			minEstimatedTotalCost = nodes[i]->getTotalEstimatedCost(goalCoordinates);
		}

	return nodes[indexMin];
}

void MyBotLogic::exploration(const STurnData& turnData, std::list<SOrder>& _orders)
{
	for (int i = 0; i < turnData.npcInfoArraySize; ++i) {

		coordinates coordNPC{ turnData.npcInfoArray[i].q, turnData.npcInfoArray[i].r };

		EHexCellDirection dir = _graph.getBestDirectionExploration(coordNPC);

		SOrder order = { EOrderType::Move, turnData.npcInfoArray[i].uid, dir };

		coordinates coordDest = getCoordinatesDirection(coordNPC, dir);

		_graph.addTimesExplored(coordDest);
		_graph.setOccupiedNode(coordNPC, false);
		_graph.setOccupiedNode(coordDest, true);

		_orders.push_back(order);
	}

}

void MyBotLogic::assigneGoalsToEachNPC(SNPCInfo* npcs)
{
	for (int i = 0; i < nbNPC; i++)
		_goalForEachNpc[npcs[i].uid] = _graph.GetClosestGoalInfo(npcs[i]);
}

void MyBotLogic::calculatePathToEachNPC(SNPCInfo* npcs)
{
	for (int i = 0; i < nbNPC; i++)
	{
		int idNPC = npcs[i].uid;
		coordinates coordNPC{ npcs[i].q, npcs[i].r };

		calculateAndInitializePath(idNPC, npcs[i]);
		_graph.setOccupiedNode(coordNPC, true);
	}
}

void MyBotLogic::moveEachNPC(SNPCInfo* npcs, std::list<SOrder>& _orders)
{
	for (int i = 0; i < nbNPC; i++)
	{
		int idNPC = npcs[i].uid;

		if (hasArrived(idNPC))
			continue;

		coordinates coordNPC{ npcs[i].q, npcs[i].r };
		EHexCellDirection direction = _pathForEachNpc[idNPC][_pathPositionForEachNpc[idNPC]];
		coordinates coordDest = getCoordinatesDirection(coordNPC, direction);

		if (_graph.isNodeFinished(coordDest)) // If a npc is at destination on the next tile and will not move again
			calculateAndInitializePath(idNPC, npcs[i]); // Calcul again A* to find another path to the goal

		if (_graph.IsNodeOccupied(coordDest))
			continue; // we wait if out next tile is occupied

		applyModificationsToGraph(coordNPC, coordDest, idNPC);

		// Give the order to the npc
		SOrder order = { EOrderType::Move, idNPC, direction };

		_orders.push_back(order);
	}
}

void MyBotLogic::calculateAndInitializePath(int idNPC, SNPCInfo npc)
{
	_pathForEachNpc[idNPC] = PathFinderAStar(npc, Heuristic{ _goalForEachNpc[idNPC] }, maxTourNb);
	_pathPositionForEachNpc[idNPC] = 0;
}

void MyBotLogic::applyModificationsToGraph(coordinates coordNPC, coordinates coordDest, int idNPC)
{
	// We free the old node
	_graph.setOccupiedNode(coordNPC, false);
	// We go on the next node
	_graph.setOccupiedNode(coordDest, true);
	_pathPositionForEachNpc[idNPC] += 1;

	if (_pathPositionForEachNpc[idNPC] == _pathForEachNpc[idNPC].size())
		_graph.setFinished(coordDest);
}

bool MyBotLogic::isNextTileToTheStart(Node* nodeAdjency, Node* currentNode, coordinates goalCoordinates)
{
	int CostSoFarAdjNode = nodeAdjency->getCost_so_far(goalCoordinates);
	int CostSoFarCurrentNode = currentNode->getCost_so_far(goalCoordinates);

	return (CostSoFarAdjNode == CostSoFarCurrentNode - 1);
}

Node* MyBotLogic::searchNextTileToTheGoal(Node* currentNode, coordinates goalCoordinates, std::vector<EHexCellDirection>& path)
{
	// Search for the next tile closest to the start 
	Node::adjencyList adjencyList = currentNode->getAdjencyList();

	for (std::pair<EHexCellDirection, Node*> nodeAdjency : adjencyList)
	{
		if (!isNextTileToTheStart(nodeAdjency.second, currentNode, goalCoordinates))
			continue;

		path.push_back(allDirectionReversed[nodeAdjency.first]); // Ajout de la direction inverse dans le path
		currentNode = nodeAdjency.second;
		break;
	}
	return currentNode;
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

int MyBotLogic::eraseFromVectorAndGetHeuristic(std::vector<Node*>& nodeVector, Node* endNode, coordinates goalCoordinates)
{
	nodeVector.erase(std::remove(begin(nodeVector), end(nodeVector), endNode), end(nodeVector));

	// Calculate the heuristic without calling the function again
	return endNode->getTotalEstimatedCost(goalCoordinates) - endNode->getCost_so_far(goalCoordinates);
}