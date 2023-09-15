#include "Node.h"
#include <iterator>

/// <summary>
/// Ajoute un noeud a la liste
/// </summary>
/// <param name="direction"></param>
/// <param name="node"></param>
void Node::addToAdjencyList(EHexCellDirection direction, Node* node)
{
	_adjencyList.insert({ std::pair<EHexCellDirection, Node*>(direction, node) });

}



Node* Node::getNodeDirection(EHexCellDirection direction)
{
	return _adjencyList[direction];
}



bool Node::inAdjacentList(EHexCellDirection direction)
{
	return _adjencyList.find(direction) != _adjencyList.end();
}

EHexCellDirection Node::getBetterDirection()
{

	int minDist = 100, secondMin;
	EHexCellDirection betterDirection = CENTER, secondBetter;

	for (auto it = _adjencyList.begin(); it != _adjencyList.end(); it++)
	{
		if (it->second->getDistToGoal() <= minDist) {
			secondMin = minDist;
			minDist = it->second->getDistToGoal();
			secondBetter = betterDirection;
			betterDirection = it->first;

		}
	}

	if (_adjencyList.at(betterDirection)->occupied == true) {
		if (minDist == secondMin) {
			betterDirection = secondBetter;
		}
		else {
			betterDirection = CENTER;
		}

	}
	else {
		occupied = false;
		_adjencyList.at(betterDirection)->occupied = true;
		_distToGoal++;
	}

	return betterDirection;
}
