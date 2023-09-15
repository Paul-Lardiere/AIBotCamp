#pragma once
#include "Globals.h"
#include <vector>
#include <map>
class Node
{

	using adjencyList = std::map<EHexCellDirection, Node*>;

private:

	int _distToGoal = 100;
	STileInfo _tile;
	adjencyList _adjencyList;
	bool occupied = false;

public:
	Node() = default;
	Node(STileInfo tile) : _tile(tile) {};
	bool finished = false;

	void addToAdjencyList(EHexCellDirection direction, Node* node);
	bool inAdjacentList(EHexCellDirection direction);
	adjencyList& getAdjencyList() { return _adjencyList; };

	EHexCellDirection getBetterDirection();

	Node* getNodeDirection(EHexCellDirection direction);
	STileInfo getTileInfo() { return _tile; };
	void setDistToGoal(int distToGoal) { _distToGoal = distToGoal; };
	int getDistToGoal() { return _distToGoal; };

};