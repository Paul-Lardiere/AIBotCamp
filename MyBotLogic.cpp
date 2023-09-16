#include "MyBotLogic.h"

#include "Globals.h"
#include "ConfigData.h"
#include "InitData.h"
#include "TurnData.h"

#define LOG(x) BOT_LOGIC_LOG(mLogger, x, true)
#define LOGLOG(x) { std::stringstream ss; ss << x; LOG(ss.str().c_str()); }
#define ASSERT(x) ASSERT_RET(x, )
#define ASSERT_RET(x, ret)\
    if(!(x)) {\
        LOG("Broken assertion: " #x);\
        return ret;\
    }

#ifdef GLOBAL_LOGGER
Logger mLogger;
#endif

#ifdef _DEBUG
template<class A, class B>
std::basic_ostream<A, B>& operator<<(std::basic_ostream<A, B>& s, const std::pair<int, int>& p)
{
	return s << "(" << p.first << "," << p.second << ")";
}
template<class A, class B, class C>
std::basic_ostream<A, B>& operator<<(std::basic_ostream<A, B>& s, const std::vector<C>& p)
{
	s << "(";
	for (const auto& c : p) s << c << ",";
	return s << ")";
}
#endif



MyBotLogic::MyBotLogic()
{
	//Write Code Here
}

MyBotLogic::~MyBotLogic()
{
	//Write Code Here
}

void MyBotLogic::Configure(const SConfigData& _configData)
{
#ifdef BOT_LOGIC_DEBUG
	mLogger.Init(_configData.logpath, "MyBotLogic.log");
#endif

	BOT_LOGIC_LOG(mLogger, "Configure", true);

	//Write Code Here
}

void MyBotLogic::Init(const SInitData& _initData)
{
	BOT_LOGIC_LOG(mLogger, "Init", true);
	_graph.InitGraph(_initData.tileInfoArraySize, _initData.tileInfoArray, _initData.objectInfoArray, _initData.objectInfoArraySize);
	BOT_LOGIC_LOG(mLogger, _graph.printGraph(), true);
	//Write Code Here
}

void MyBotLogic::GetTurnOrders(const STurnData& _turnData, std::list<SOrder>& _orders)
{
	BOT_LOGIC_LOG(mLogger, "GetTurnOrders", true);

	//Write Code Here
}