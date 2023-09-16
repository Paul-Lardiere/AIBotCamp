#pragma once

#include "BotLogicIF.h"
#include "Logger.h"
#include <Graph.h>

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
private:
	Graph& _graph = Graph::get();
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
