#pragma once
#include "Globals.h"
#include "Node.h"
#include <string>
#include <map>
#include <vector>



using distance_type = int;

class Graph
{
public:
	using coordinates = std::pair<int, int>;
private:
	size_t _size{};
	std::map<coordinates, Node*> _nodes;
	std::map<coordinates, STileInfo> _initMap;
	std::map<coordinates, STileInfo> _updateMap;
	std::map<std::pair<coordinates,EHexCellDirection>,SObjectInfo*> _objectInfoArray;
	int _objectInfoArraySize = -1;
	int _idGraphUnaffected = 0;




	Graph() = default;

	void createGraph(Node* node);

	bool exist(coordinates coordinates);
	bool isInitialized(coordinates coordinates);
	void addNode(Node* node);
	void updateDirection(EHexCellDirection direction, int q, int r, Node* node);
	bool isNotWalled(coordinates coordinateNode1, coordinates coordinateNode2, EHexCellDirection direction);
	void updateIdGraph(Node* node, int id);

	//conditions
	bool isUsedByAnotherNPC(std::pair<Graph::coordinates, int> goal) { return goal.second != -1; };
	bool hasSameGraphIndex(int indexGraphNPC, std::pair<Graph::coordinates, int> goal) { return indexGraphNPC == getNode(goal.first)->getIdGraph(); }

public:

	// goals and npc id if taken (-1 otherwise)
	std::map<coordinates, int> _goals;


	Graph(const Graph&) = delete;
	Graph& operator=(const Graph&) = delete;


	static auto& get() {
		static Graph singleton;
		return singleton;
	}

	void InitGraph(size_t size, const STileInfo* _tileList, SObjectInfo* objectInfoArray, int objectInfoArraySize, SNPCInfo* npcInfoArray, int npcInfoArraySize);
	void updateGraph(size_t size, const STileInfo* _tileList, SObjectInfo* objectInfoArray, int objectInfoArraySize, SNPCInfo* npcInfoArray, int npcInfoArraySize);

	~Graph();
	const size_t getSize() { return _size; };

	std::map<coordinates, Node*> getNodes() { return _nodes; };
	Node* getNode(coordinates coord) { return _nodes[coord]; };

	coordinates GetClosestGoalInfo(SNPCInfo npcCurrent);
	int distanceHexCoordNpc(coordinates coordinates, SNPCInfo npcInfo);

	void setOccupiedNode(coordinates coord, bool isOccupied) { _nodes[coord]->setOccupied(isOccupied); }
	bool IsNodeOccupied(coordinates coord) { return _nodes[coord]->isOccupied(); }

	bool isNodeFinished(coordinates coord) { return _nodes[coord]->finished; }
	void setFinished(coordinates coord) { _nodes[coord]->finished = true; }
	
	void addTimesExplored(coordinates coord) { ++_nodes[coord]->timesExplored; }
	int getTimesExplored(coordinates coord) {return _nodes[coord]->timesExplored;};
	
	std::string printGraph();

	bool hasEnoughGoals(int nbNpc, SNPCInfo* npcInfo);
};

