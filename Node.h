#pragma once
#include "Globals.h"
#include <vector>
#include <map>
class Node
{
	using coordinates = std::pair<int, int>;
	using adjencyList = std::map<EHexCellDirection, Node*>;

private:

	STileInfo _tile;
	adjencyList _adjencyList;

	// cost so far du noeud pour arriver au goal qui se trouve au coordoonées en clé
	std::map<coordinates, float> _cost_so_far;
	std::map<coordinates, float> _heuristic;
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

	void setCost_so_far(coordinates coordinate, float cost_so_far) { _cost_so_far.insert({coordinate, cost_so_far}); };
	float getCost_so_far(coordinates coordinate) { return _cost_so_far.at(coordinate); };

	void setHeuristic(coordinates coordinate, float heuristic) { _heuristic.insert({ coordinate, heuristic }); };
	float getHeuristic(coordinates coordinate) { return _heuristic.at(coordinate); };

};