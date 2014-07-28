
/* Standrad Included Library */
#include <iostream>
#include <cstdio>
#include <string>
using namespace std;

/* Third-party Library */
#include "RobotAction/RobotAction.h"

//** Problem Dependent Variable Setting **//
//#define SERVER_NAME "192.168.11.4"
#define SERVER_NAME "localhost"
	// The name of the audience, default is the hello
string userName = "哈樓";

/* For IPC handler using */
bool isUserNameReceived = false;

//=============================================================================
int main(int argc, char* argv[]) {
		// Server communication
	RobotAction robot(SERVER_NAME);

	cout << "\n\t< TAROS 2014 >" << endl;

	/* Session: Online Face Recognition */
	cout << "> INFO: Waiting for the user name..." << endl;
		// Busy waiting
	while (isUserNameReceived == false) Sleep(1000);
	isUserNameReceived = true;
		// Get user name
	KeyWordMgr userNameInput;
	getKeyWord(userNameInput);
	string tempName = userNameInput.keyword;
		// Check the validation of the name
	if (tempName.length() != 0) {
		cout << "> INFO: User name " + userName + " is received!" << endl;
		userName = tempName;
	} else
		cout << "> WARNING: The user name is empty! Using \"Hello\" to represent..." << endl;

	robot.speaking(userName + "大濕，您好", 0.9f);

	Sleep(5000);

	/* Session: Attracting user's attention */
	cout << "> INFO: Start to grab user's attention..." << endl;
		// Voice volume for the ARIO
	float voiceVolume = 0.1f;
		// Grabing user's attention by call name!
	for (int i = 0; robot.getHighAttentionFlag() == 0 && i < 3; i++) {
		robot.speaking(userName, voiceVolume);
		voiceVolume += 0.3f;
		Sleep(2000);

		if (robot.getContingencyFlag() > 0) {
			robot.speaking("請注意這裡一下", 0.7f);
			robot.resetContingencyFlag();
		}
	}
		// Timeout occurred! Waiting for more time
	for (int i = 0; robot.getHighAttentionFlag() == 0 && i < 10; i++) {
		cout << "> WARNING: User didn't notice me!" << endl;
		Sleep(500);
		if (robot.getHighAttentionFlag() > 0)
			break;
	}
	
	robot.speaking("我這裡有杯毒藥，要不要嚐嚐", 0.8f);

	/* Session: Grasping */
		// Turn to the cabinet
	cout << "> ACTION: Turning to the cabinet..." << endl;
	robot.toPoint(0.0, 0.0, 90.0);
		// Grasping!
	cout << "> ACTION: GRASPING!" << endl;
	robot.graspingNil(0.0, 0.0, 0.0);

	return 0;
}
