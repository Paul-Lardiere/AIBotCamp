#pragma once

#include <vector>
#include <map>

enum EHexCellType
{
	Goal,
	Forbidden,
	Default
};

enum EHexCellDirection
{
	W, NW, NE, E, SE, SW, CENTER
};
static const EHexCellDirection allDirection[] = { W, NW, NE, E, SE, SW };
static const EHexCellDirection allDirectionReversed[] = { E, SE ,SW, W, NW, NE };

enum EObjectType
{
	Wall,
	Window,
	Door,
	PressurePlate
};

enum EObjectState
{
	Opened,
	Closed
};

enum EOrderType
{
	Move,
	Interact
};

enum EInteractionType
{
	OpenDoor,
	CloseDoor,
	SearchHiddenDoor,
};

struct SOrder
{
	EOrderType orderType;
	int npcUID;
	EHexCellDirection direction;
	int objectUID;
	EInteractionType interactionType;
};

struct STileInfo
{
	int q;
	int r;
	EHexCellType type;
};

struct SObjectInfo
{
	int uid;
	int q;
	int r;
	EHexCellDirection cellPosition;

	int* types;
	int typesSize;

	int* states;
	int statesSize;

	int* connectedTo;
	int connectedToSize;
};

struct SNPCInfo
{
	int uid;
	int q;
	int r;
	int visionRange;
};

struct Heuristic
{
	using coordinates = std::pair<int, int>;

	Heuristic(coordinates end) : endTileCoordinates{ end } { }
	coordinates endTileCoordinates;

	int estimate(coordinates startcoordinates) {
		int qdiff = endTileCoordinates.first - startcoordinates.first;
		int rdiff = endTileCoordinates.second - startcoordinates.second;
		int sdiff = -(qdiff + rdiff);

		return (std::abs(qdiff) + abs(rdiff) + abs(sdiff)) / 2;
	}
};
