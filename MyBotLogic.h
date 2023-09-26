#pragma once

#include "BotLogicIF.h"
#include "Logger.h"
#include <Graph.h>
#include <vector>

#ifdef _DEBUG
#define BOT_LOGIC_DEBUG
#define GLOBAL_LOGGER
#endif

#ifdef BOT_LOGIC_DEBUG
#define BOT_LOGIC_LOG(logger, text, autoEndLine) logger.Log(text, autoEndLine)
#else
#define BOT_LOGIC_LOG(logger, text, autoEndLine) 0
#endif

struct SConfigData;
struct STurnData;

//Custom BotLogic where the AIBot decision making algorithms should be implemented.
//This class must be instantiated in main.cpp.
class MyBotLogic : public virtual BotLogicIF
{
	using coordinates = std::pair<int, int>;
private:
	Graph& _graph = Graph::get();

	std::map<int, Graph::coordinates> _goalForEachNpc;
	std::map<int, std::vector<EHexCellDirection>> _pathForEachNpc;
	std::map<int, int> _pathPositionForEachNpc;
	int maxTourNb;
	int nbNPC;

	std::vector<EHexCellDirection> PathFinderAStar(SNPCInfo npcCurrent, Heuristic heuristic, int maxTurnNb);
	Node* FindClosestNode(std::vector<Node*> nodes, Graph::coordinates goalCoordinates);
	void exploration(const STurnData& _turndata, std::list<SOrder>& _orders);
	coordinates getCoordinatesDirection(coordinates coordinate, EHexCellDirection direction);
	void assigneGoalsToEachNPC(SNPCInfo* npcs);
	void calculatePathToEachNPC(SNPCInfo* npcs);
	void moveEachNPC(SNPCInfo* npcs, std::list<SOrder>& _orders);
	void calculateAndInitializePath(int idNPC, SNPCInfo npc);
	void applyModificationsToGraph(coordinates coordNPC, coordinates coordDest, int idNPC);

	// conditions
	bool allGoalsAreAssigned() { return _goalForEachNpc.size() == nbNPC; };
	bool hasArrived(int idNPC) { return _pathPositionForEachNpc[idNPC] == _pathForEachNpc[idNPC].size(); }

public:
	MyBotLogic();
	virtual ~MyBotLogic();

	virtual void Configure(const SConfigData& _configData);
	virtual void Init(const SInitData& _initData);
	virtual void GetTurnOrders(const STurnData& _turnData, std::list<SOrder>& _orders);



protected:
#ifndef GLOBAL_LOGGER
	Logger mLogger;
#endif
};

#ifdef GLOBAL_LOGGER
extern Logger mLogger;
#endif
