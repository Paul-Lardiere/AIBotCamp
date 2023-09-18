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
	SObjectInfo* _objectInfoArray;
	int _objectInfoArraySize;

	// goals and npc id if taken (-1 otherwise)
	std::map<coordinates, int> _goals;

	Graph() = default;

	void createGraph(Node* node);
	bool exist(coordinates coordinates);
	bool isInitialized(coordinates coordinates);
	void addNode(Node* node);
	void updateDirection(EHexCellDirection direction, int q, int r, Node* node);
	bool isNotWalled(coordinates coordinateNode1, coordinates coordinateNode2, EHexCellDirection direction);

public:
	Graph(const Graph&) = delete;
	Graph& operator=(const Graph&) = delete;

	static auto& get() {
		static Graph singleton;
		return singleton;
	}

	void InitGraph(size_t size, const STileInfo* _tileList, SObjectInfo* objectInfoArray, int objectInfoArraySize);
	~Graph();
	const size_t getSize() { return _size; };

	std::map<coordinates, Node*> getNodes() { return _nodes; };
	Node* getNode(coordinates coord) { return _nodes[coord]; };

	coordinates GetClosestGoalInfo(SNPCInfo npcCurrent);
	int distanceHexCoordNpc(coordinates coordinates, SNPCInfo npcInfo);

	std::string printGraph();
};

