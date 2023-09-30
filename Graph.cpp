#include "Graph.h"
#include <algorithm>
#include <format>


/// <summary>
/// Construit le graph a partir de de la liste de TileInfo on ignorant les tile forbidden
/// </summary>
/// <param name="size"></param>
/// <param name="_tileList"></param>
void Graph::InitGraph(size_t size, const STileInfo* _tileList, SObjectInfo* objectInfoArray, int objectInfoArraySize, SNPCInfo* npcInfoArray, int npcInfoArraySize)
{
	for (auto it = objectInfoArray + 0;it != objectInfoArray + objectInfoArraySize; ++it)
		_objectInfoArray[std::pair<coordinates, EHexCellDirection>{coordinates{ it->q,it->r }, it->cellPosition}] = it;
	//_objectInfoArraySize = objectInfoArraySize;
	std::for_each(_tileList, _tileList + size, [this](auto&& initTileInfo) {
		if (initTileInfo.type != Forbidden)
			_initMap[coordinates{ initTileInfo.q, initTileInfo.r }] = initTileInfo;
		});

	for (int i = 0; i < npcInfoArraySize; ++i) {
		coordinates coordNPC{ npcInfoArray[i].q,npcInfoArray[i].r };
		const STileInfo* tileNPC = std::find_if(_tileList + 0, _tileList + size, [&coordNPC](STileInfo tile) {
			return ((tile.q == coordNPC.first) && (tile.r == coordNPC.second));
			});

		Node* node = new Node(*tileNPC);
		node->initIdGraph(++_idGraphUnaffected);
		node->timesExplored = 1;
		addNode(node);
		createGraph(node);
	}
}

Graph::~Graph()
{
	for (auto it = _nodes.begin(); it != _nodes.end(); ++it)
	{

		delete it->second;
	};
}



/// <summary>
/// fonction recursive de cr�ation du graph
/// </summary>
/// <param name="node"></param>
void Graph::createGraph(Node* node)
{
	//BOT_LOGIC_LOG(mLogger, format("NODE : {},{}", node->getTileInfo().q, node->getTileInfo().r), true);
	STileInfo tile = node->getTileInfo();

	coordinates coord = coordinates{ tile.q, tile.r };

	/*BOT_LOGIC_LOG(mLogger, "", true);

	BOT_LOGIC_LOG(mLogger, format("node : {},{}", node->getTileInfo().q, node->getTileInfo().r), true);*/
	updateDirection(W, (tile.q), (tile.r) - 1, node);
	updateDirection(NW, (tile.q) - 1, (tile.r), node);
	updateDirection(NE, (tile.q) - 1, (tile.r) + 1, node);
	updateDirection(E, (tile.q), (tile.r) + 1, node);
	updateDirection(SE, (tile.q) + 1, (tile.r), node);
	updateDirection(SW, (tile.q) + 1, (tile.r) - 1, node);
}

void Graph::updateGraph(size_t size, const STileInfo* _tileList, SObjectInfo* objectInfoArray, int objectInfoArraySize, SNPCInfo* npcInfoArray, int npcInfoArraySize)
{

	_initMap.clear();
	for (auto it = objectInfoArray + 0;it != objectInfoArray + objectInfoArraySize; ++it)
		_objectInfoArray[std::pair<coordinates, EHexCellDirection>{coordinates{ it->q,it->r }, it->cellPosition}] = it;
	_objectInfoArraySize = objectInfoArraySize;
	std::for_each(_tileList, _tileList + size, [this](auto&& initTileInfo) {
		if (initTileInfo.type != Forbidden)
			_initMap[coordinates{ initTileInfo.q, initTileInfo.r }] = initTileInfo;
		});

	for (int i = 0; i < npcInfoArraySize; ++i) {
		coordinates coordNPC{ npcInfoArray[i].q,npcInfoArray[i].r };
		Node* node = _nodes[coordNPC];
		createGraph(node);
	}
	for (auto it = _objectInfoArray.begin(); it != _objectInfoArray.end(); ++it) {
		coordinates coordObj = coordinates{ it->second->q, it->second->r };
		if (isInitialized(coordObj) && (getNodes()[coordObj]->getAdjencyList().find(it->second->cellPosition) != getNodes()[coordObj]->getAdjencyList().end()) && isInitialized(getNodes()[coordObj]->getNodeDirection(it->second->cellPosition)->getNodeCoordinates())) {
			getNodes()[coordObj]->getNodeDirection(it->second->cellPosition)->getAdjencyList().erase(static_cast<EHexCellDirection>((it->second->cellPosition + 3) % 6));
			getNodes()[coordObj]->getAdjencyList().erase(it->second->cellPosition);
			getNode(coordObj)->initIdGraph(++_idGraphUnaffected);

		}
	}
	for (int i = 0; i < npcInfoArraySize; ++i) {
		coordinates coordNPC{ npcInfoArray[i].q,npcInfoArray[i].r };
		updateIdGraph(_nodes[coordNPC], _nodes[coordNPC]->getIdGraph());
		for (auto it = _nodes.begin(); it != _nodes.end(); ++it)
			it->second->updated = false;
	}
}

void Graph::updateDirection(EHexCellDirection direction, int q, int r, Node* node)
{

	if (!node->inAdjacentList(allDirection[direction]) && exist(coordinates{ q,r })) {

		Node* newNode;

		if (!isInitialized(coordinates{ q, r })) {
			newNode = new Node(_initMap[coordinates{ q, r }]);
			//newNode->initIdGraph(++_idGraphUnaffected );
			addNode(newNode);


			//newNode->initIdGraph(node->getIdGraph());
			node->addToAdjencyList(allDirection[direction], newNode);
			newNode->addToAdjencyList(allDirectionReversed[direction], node);


			createGraph(newNode);
		}
		else {
			newNode = _nodes[coordinates{ q, r }];

			node->addToAdjencyList(allDirection[direction], newNode);
			newNode->addToAdjencyList(allDirectionReversed[direction], node);
			/*newNode->setIdGraph(node->getIdGraph());
			node->setIdGraph(newNode->getIdGraph());*/

		}

	}

}

/// <summary>
/// ajoute un node au graph
/// </summary>
/// <param name="node"></param>
void Graph::addNode(Node* node)
{
	_size++;
	_nodes.insert({ coordinates {node->getTileInfo().q, node->getTileInfo().r}, node });

	if (node->getTileInfo().type == EHexCellType::Goal)
		_goals[{ node->getTileInfo().q, node->getTileInfo().r }] = -1;
}

/// <summary>
/// retourne true si le noeud existe (coord2D)
/// </summary>
/// <param name="coordinates"></param>
/// <returns></returns>
bool Graph::exist(coordinates coordinates)
{
	return _initMap.find(coordinates) != _initMap.end();
}

bool Graph::isNotWalled(coordinates coordinateNode1, coordinates coordinateNode2, EHexCellDirection direction)
{
	return std::find_if(_objectInfoArray.begin(), _objectInfoArray.end(), [&](auto& objectInfo) {
		return (objectInfo.second->q == coordinateNode1.first && objectInfo.second->r == coordinateNode1.second && objectInfo.second->cellPosition == direction)
			|| (objectInfo.second->q == coordinateNode2.first && objectInfo.second->r == coordinateNode2.second && objectInfo.second->cellPosition == (direction + 3) % 6);
		}) == _objectInfoArray.end();
}

void Graph::updateIdGraph(Node* node, int id)
{
	if (node->updated == false) {
		node->initIdGraph(id);
		for (auto it = node->getAdjencyList().begin(); it != node->getAdjencyList().end(); ++it)
		{
			node->updated = true;
			this->updateIdGraph(it->second, id);

		}
	}
}

bool Graph::isInitialized(coordinates coordinates)
{
	return _nodes.find(coordinates) != _nodes.end();
}

Graph::coordinates Graph::GetClosestGoalInfo(SNPCInfo npcCurrent)
{
	size_t minDist = _size;
	int distance;
	int indexGraphNpc = getNode(coordinates{ npcCurrent.q, npcCurrent.r })->getIdGraph();

	Graph::coordinates closestgoalCoordinates{ -1, -1 };

	for (std::pair<Graph::coordinates, int> goal : _goals) {
		if (isUsedByAnotherNPC(goal) || !hasSameGraphIndex(indexGraphNpc, goal))
			continue;

		distance = distanceHexCoordNpc(goal.first, npcCurrent);
		if (minDist > distance)
		{
			minDist = distance;
			closestgoalCoordinates = goal.first;
		}
	}

	// register the npc id for the closest goal
	if (closestgoalCoordinates != coordinates{ -1, -1 })
		_goals[closestgoalCoordinates] = npcCurrent.uid;

	return closestgoalCoordinates;
}

int Graph::distanceHexCoordNpc(coordinates coordinates, SNPCInfo npcInfo)
{
	int qdiff = coordinates.first - npcInfo.q;
	int rdiff = coordinates.second - npcInfo.r;
	int sdiff = -(qdiff + rdiff);

	return (abs(qdiff) + abs(rdiff) + abs(sdiff)) / 2;
}

std::string Graph::printGraph()
{
	std::string ret;
	for (auto it = _nodes.cbegin(); it != _nodes.cend(); ++it)
	{
		ret += std::format("Node ({},{}) : idGraph {}\n", it->second->getTileInfo().q, it->second->getTileInfo().r, static_cast<int>(it->second->getIdGraph()));
		for (auto it2 = it->second->getAdjencyList().begin(); it2 != it->second->getAdjencyList().end(); ++it2)
			ret += std::format("({},{}) ", it2->second->getTileInfo().q, it2->second->getTileInfo().r);
		ret += "\n";
	}

	return ret;
}

bool Graph::hasEnoughGoals(int nbNpc, SNPCInfo* npcInfo)
{
	if (nbNpc > _goals.size())
		return false; // Too many NPC compared to the number of goals

	std::map<int, int> nbGoalsPerGraphIndex;
	std::map<int, int> nbNPCPerGraphIndex;

	for (std::pair<coordinates, int> goal : _goals)
		++(nbGoalsPerGraphIndex[getNode(goal.first)->getIdGraph()]);

	for (int npcIndex = 0; npcIndex < nbNpc; ++npcIndex)
	{
		int indexGraphNPC = (getNode(coordinates{ npcInfo[npcIndex].q, npcInfo[npcIndex].r })->getIdGraph());
		++(nbNPCPerGraphIndex[indexGraphNPC]);

		if (nbNPCPerGraphIndex[indexGraphNPC] > nbGoalsPerGraphIndex[indexGraphNPC])
			return false; // Too many NPC compared to the number of goals in one graph index
	}

	return true;
}

