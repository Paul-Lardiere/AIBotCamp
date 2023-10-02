#include "Graph.h"
#include <algorithm>
#include <format>
#include <math.h>


/// <summary>
/// Construit le graph a partir de de la liste de TileInfo on ignorant les tile forbidden
/// </summary>
/// <param name="size"></param>
/// <param name="_tileList"></param>
void Graph::InitGraph(size_t nbTiles, const STileInfo* _tileList, SObjectInfo* objectInfoArray, int objectInfoArraySize, SNPCInfo* npcInfoArray, int npcInfoArraySize)
{
	for (auto it = objectInfoArray + 0;it != objectInfoArray + objectInfoArraySize; ++it)
		_objectInfoArray[std::pair<coordinates, EHexCellDirection>{coordinates{ it->q,it->r }, it->cellPosition}] = it;
	//_objectInfoArraySize = objectInfoArraySize;
		
	std::for_each(_tileList, _tileList + nbTiles, [this](auto&& initTileInfo) {
		if (initTileInfo.type != Forbidden)
			_initMap[coordinates{ initTileInfo.q, initTileInfo.r }] = initTileInfo;
		});

	for (int i = 0; i < npcInfoArraySize; ++i) {
		coordinates coordNPC{ npcInfoArray[i].q,npcInfoArray[i].r };

		Node* node = new Node(_initMap[coordNPC]);
		node->initIdGraph(++_idGraphUnaffected);
		node->timesExplored = 1;
		addNode(node);
		createGraph(node, _tileList, nbTiles);
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
/// fonction recursive de création du graph
/// </summary>
/// <param name="node"></param>
void Graph::createGraph(Node* node, const STileInfo* tiles, int nbTiles)
{
	STileInfo tile = node->getTileInfo();

	coordinates coord = coordinates{ tile.q, tile.r };

	updateDirection(W, (tile.q), (tile.r) - 1, node, tiles, nbTiles);
	updateDirection(NW, (tile.q) - 1, (tile.r), node, tiles, nbTiles);
	updateDirection(NE, (tile.q) - 1, (tile.r) + 1, node, tiles, nbTiles);
	updateDirection(E, (tile.q), (tile.r) + 1, node, tiles, nbTiles);
	updateDirection(SE, (tile.q) + 1, (tile.r), node, tiles, nbTiles);
	updateDirection(SW, (tile.q) + 1, (tile.r) - 1, node, tiles, nbTiles);
}

void Graph::updateGraph(size_t nbTiles, const STileInfo* _tileList, SObjectInfo* objectInfoArray, int objectInfoArraySize, SNPCInfo* npcInfoArray, int npcInfoArraySize)
{
	for (auto it = objectInfoArray + 0;it != objectInfoArray + objectInfoArraySize; ++it) // pour chaque objet
		_objectInfoArray[std::pair<coordinates, EHexCellDirection>{coordinates{ it->q,it->r }, it->cellPosition}] = it; // on l'ajoute à la liste des objets fct(position, direction)

	_objectInfoArraySize = objectInfoArraySize;

	std::vector<STileInfo> newTiles{};
	int nbNewTiles{};

	std::for_each(_tileList, _tileList + nbTiles, [this, &newTiles, &nbNewTiles](auto&& initTileInfo) { // pour chaque tile qui n'est pas forbidden, l'ajouter ou réécrire dans la liste _initMap
		if (initTileInfo.type != Forbidden && !exist(coordinates{ initTileInfo.q, initTileInfo.r }))
		{
			_initMap[coordinates{ initTileInfo.q, initTileInfo.r }] = initTileInfo;
			newTiles.push_back(initTileInfo);
			++nbNewTiles;
		}
		});

	for (int i = 0; i != nbNewTiles; ++i)
	{
		coordinates coordTile = coordinates{ newTiles[i].q, newTiles[i].r };
		Node* newNode = new Node(_initMap[coordTile]);
		addNodeToGraph(newNode, _tileList, nbTiles);
	}

	for (auto it = _objectInfoArray.begin(); it != _objectInfoArray.end(); ++it) { // pour chaque objet
		coordinates coordObj = coordinates{ it->second->q, it->second->r }; // récuperer ses coordonnees

		if (isInitialized(coordObj) 
			&& (getNodes()[coordObj]->getAdjencyList().find(it->second->cellPosition) != getNodes()[coordObj]->getAdjencyList().end()) 
			&& isInitialized(getNodes()[coordObj]->getNodeDirection(it->second->cellPosition)->getNodeCoordinates())
			) 
		{
			getNodes()[coordObj]->getNodeDirection(it->second->cellPosition)->getAdjencyList().erase(static_cast<EHexCellDirection>((it->second->cellPosition + 3) % 6)); // on supprime de la liste d'adjacence la node current si on trouve un mur entre les deux
			getNodes()[coordObj]->getAdjencyList().erase(it->second->cellPosition); // on supprime la direction des nodes d'adjacence des la node current
			getNode(coordObj)->initIdGraph(++_idGraphUnaffected); // on réinitialise le graph à partir de la node de direction avec un id plus grand // ici vient le probleme du graph avec un id de 70
		}
	}


	for (int i = 0; i < npcInfoArraySize; ++i) { // pour chaque npc
		coordinates coordNPC{ npcInfoArray[i].q,npcInfoArray[i].r };
		updateIdGraph(_nodes[coordNPC], _nodes[coordNPC]->getIdGraph()); // on update l'id du graph
	}

	for (auto it = _nodes.begin(); it != _nodes.end(); ++it) 
		it->second->updated = false; // on dit que toutes les nodes ont été traité.
}

void Graph::updateDirection(EHexCellDirection direction, int q, int r, Node* node, const STileInfo* tiles, int nbTiles)
{
	if (exist(coordinates{ q,r }) && !node->inAdjacentList(allDirection[direction])) {

		if (!isInitialized(coordinates{ q, r })) {
			Node* newNode = new Node(_initMap[coordinates{ q, r }]);
			//newNode->initIdGraph(++_idGraphUnaffected );
			newNode->initIdGraph(node->getIdGraph());
			addNode(newNode);


			node->addToAdjencyList(allDirection[direction], newNode);
			newNode->addToAdjencyList(allDirectionReversed[direction], node);


			createGraph(newNode, tiles, nbTiles);
		}
		else {
			// Ajout des nodes l'une à l'autre dans leur liste d'adjacence
			Node* destNode = _nodes[coordinates{ q, r }];

			node->addToAdjencyList(allDirection[direction], destNode);
			destNode->addToAdjencyList(allDirectionReversed[direction], node);
		}

	}

}

void Graph::updateDirectionBis(EHexCellDirection direction, int q, int r, Node* node, const STileInfo* tiles, int nbTiles)
{

	if (exist(coordinates{ q,r }) && !node->inAdjacentList(allDirection[direction])) {
		Node* destNode;

		if (!isInitialized(coordinates{ q, r })) {
			destNode = new Node(_initMap[coordinates{ q, r }]);
			//newNode->initIdGraph(++_idGraphUnaffected );
			destNode->initIdGraph(node->getIdGraph());
			addNode(destNode);
		}
		else {
			// Ajout des nodes l'une à l'autre dans leur liste d'adjacence
			destNode = _nodes[coordinates{ q, r }];
		}

		node->addToAdjencyList(allDirection[direction], destNode);
		destNode->addToAdjencyList(allDirectionReversed[direction], node);

	}

}

/// <summary>
/// ajoute un node au graph
/// </summary>
/// <param name="node"></param>
void Graph::addNode(Node* node)
{
	_size++;
	_nodes[coordinates {node->getTileInfo().q, node->getTileInfo().r}] = node;

	if (node->getTileInfo().type == EHexCellType::Goal)
		_goals[{ node->getTileInfo().q, node->getTileInfo().r }] = -1; // initialisation du goal sans lui affecter un npc
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
		node->updated = true;
		node->initIdGraph(id);
		for (auto it = node->getAdjencyList().begin(); it != node->getAdjencyList().end(); ++it)
			this->updateIdGraph(it->second, id);
	}
}

void Graph::addNodeToGraph(Node* node, const STileInfo* tiles, int nbTiles)
{
	addNode(node);
	STileInfo tile = node->getTileInfo();

	coordinates coord = coordinates{ tile.q, tile.r };

	updateDirectionBis(W, (tile.q), (tile.r) - 1, node, tiles, nbTiles);
	updateDirectionBis(NW, (tile.q) - 1, (tile.r), node, tiles, nbTiles);
	updateDirectionBis(NE, (tile.q) - 1, (tile.r) + 1, node, tiles, nbTiles);
	updateDirectionBis(E, (tile.q), (tile.r) + 1, node, tiles, nbTiles);
	updateDirectionBis(SE, (tile.q) + 1, (tile.r), node, tiles, nbTiles);
	updateDirectionBis(SW, (tile.q) + 1, (tile.r) - 1, node, tiles, nbTiles);
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
		ret += std::format("Node ({},{}) : idGraph {}\n			", it->second->getTileInfo().q, it->second->getTileInfo().r, static_cast<int>(it->second->getIdGraph()));
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

EHexCellDirection Graph::getBestDirectionExploration(coordinates coordNPC)
{
	Node* npcNode = getNode(coordNPC);

	float maxattraction = -10000000;
	Node::adjencyList adjList = npcNode->getAdjencyList();

	EHexCellDirection bestDirection = W;


	for (std::pair<EHexCellDirection, Node*> currentAdjencyNode : adjList)
	{
		EHexCellDirection dir = currentAdjencyNode.first;
		Node* nodeAdjacent = currentAdjencyNode.second;

		npcNode->setCountedInAttraction(true);

		float attraction = getCoefAttraction(nodeAdjacent, 1);
		if (attraction < maxattraction)
			continue;

		maxattraction = attraction;
		bestDirection = dir;

		clearCountedInAttraction();
	}


	return bestDirection;
}

float Graph::getCoefAttraction(Node* node, int distance)
{
	float coefAttraction = 1.0f - node->timesExplored;

	std::vector<Node*> nodeToAddInAttraction{};

	for (std::pair<EHexCellDirection, Node*> currentAdjencyNode : node->getAdjencyList())
	{
		EHexCellDirection dir = currentAdjencyNode.first;
		Node* nodeAdjacent = currentAdjencyNode.second;
		if (nodeAdjacent->isCountedInAttraction())
			continue;

		if (nodeAdjacent->timesExplored == 0)
			coefAttraction += 1.0f;

		nodeAdjacent->setCountedInAttraction(true);
		nodeToAddInAttraction.push_back(nodeAdjacent);
	}
	
	coefAttraction = coefAttraction / distance;

	std::for_each(nodeToAddInAttraction.begin(), nodeToAddInAttraction.end(), [&coefAttraction, distance, this](Node* node) {
		coefAttraction += getCoefAttraction(node, distance + 1);
		});

	return coefAttraction;
}	

void Graph::clearCountedInAttraction()
{
	for (std::pair<coordinates, Node*> currentNode : _nodes) {
		currentNode.second->setCountedInAttraction(false);
	}
}

