#pragma once
#include "Globals.h"
#include <vector>
#include <map>
class Node
{
	using coordinates = std::pair<int, int>;
public:
	using adjencyList = std::map<EHexCellDirection, Node*>;

private:

	STileInfo _tile;
	adjencyList _adjencyList;

	// cost so far du noeud pour arriver au goal qui se trouve au coordoon�es en cl�
	std::map<coordinates, int> _cost_so_far;
	std::map<coordinates, int> _heuristic;
	int _idGraph{};
	bool _occupied = false;

public:
	Node() = default;
	Node(STileInfo tile) : _tile(tile) {};
	bool finished = false;
	int timesExplored = 0;
	int esperance = 0;
	bool updated = false;

	void addToAdjencyList(EHexCellDirection direction, Node* node);
	bool inAdjacentList(EHexCellDirection direction);
	adjencyList& getAdjencyList() { return _adjencyList; };

	Node* getNodeDirection(EHexCellDirection direction);
	STileInfo getTileInfo() { return _tile; };
	coordinates getNodeCoordinates() { return coordinates{ _tile.q, _tile.r }; }

	int getIdGraph() { return _idGraph; };
	void setIdGraph(int idGrpah) { _idGraph = std::min(idGrpah, _idGraph); };
	void initIdGraph(int  idGraph) { _idGraph = idGraph; };

	int getEsperance() { return esperance; };
	void setEsperance(int esp) { esperance = esp; };

	void setCost_so_far(coordinates coordinate, int cost_so_far) { _cost_so_far[coordinate] = cost_so_far; };
	int getCost_so_far(coordinates coordinate) { 
		if (_cost_so_far.contains(coordinate))
			return _cost_so_far.at(coordinate); 
		return -1;
	};

	void setHeuristic(coordinates coordinate, int heuristic) { _heuristic.insert({ coordinate, heuristic }); };
	int getHeuristic(coordinates coordinate) {
		if (_heuristic.contains(coordinate))
			return _heuristic.at(coordinate);
		return -1;
	}

	int getTotalEstimatedCost(coordinates coordinate) { 
		if (_heuristic.contains(coordinate) && _cost_so_far.contains(coordinate))
			return _heuristic.at(coordinate) + _cost_so_far.at(coordinate); 
		return -1;
	}

	void setOccupied(bool isOccupied) { _occupied = isOccupied; }
	bool isOccupied() { return _occupied; }


	bool operator==(Node node) const {
		return ((node.getTileInfo().q == _tile.q) &&
			(node.getTileInfo().r == _tile.r));
	}

	bool operator!=(Node node) const {
		return ((node.getTileInfo().q != _tile.q) ||
			(node.getTileInfo().r != _tile.r));
	}
};