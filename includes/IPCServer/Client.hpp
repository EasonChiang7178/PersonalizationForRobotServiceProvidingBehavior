
/******************************************************
NTU EECS Advanced Control Lab, Intelligent Robot Lab

2010 Kuo-Chen Huang

******************************************************/
#ifndef IPC_CLIENT_H
#define IPC_CLIENT_H

#include "Comm.hpp"
#include "../Thread/Thread.h"
#include "ipc.h"
#include <string>

	// Import the Wander FSM we implemented
//#include "../Wander_ActionPlanner_FSM/WanderFSM.h"

using std::string;

namespace ROBOT{

class MsgInfo{
public:
	MsgInfo(){}
	MsgInfo( string n, string f, HANDLER_TYPE ptr ){
		msg_name = n;
		msg_format = f;
		msg_handler = ptr;
	}
	string msg_name;
	string msg_format;
	HANDLER_TYPE msg_handler;
};

class ClientThread: public Thread {
public:
	ClientThread() {}
	virtual void* run() {
		while(!shouldStop()){
			//printf("|||||\n");
			IPC_RETURN_TYPE  IPCListener = IPC_listenClear(5);
			//printf("-----\n");
			yield();
			//if (IPCListener == 0)
			//	printf("IPC_Listen State: IPC_Error\n");
			//else if (IPCListener == 1)
			//	printf("IPC_Listen State: IPC_OK\n");
			//else
			//	printf("IPC_Listen State: IPC_Timeout\n");
		}
		return 0;
	}
};

} // namespace

#endif
