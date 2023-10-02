#include "Node.h"
#include <iterator>

/// <summary>
/// Ajoute un noeud a la liste
/// </summary>
/// <param name="direction"></param>
/// <param name="node"></param>
void Node::addToAdjencyList(EHexCellDirection direction, Node* node)
{
	_adjencyList[direction] = node;
}



Node* Node::getNodeDirection(EHexCellDirection direction)
{
	return _adjencyList[direction];
}



bool Node::inAdjacentList(EHexCellDirection direction)
{
	return _adjencyList.find(direction) != _adjencyList.end();
}

