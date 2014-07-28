
/* Standrad Included Library */
#include <conio.h>		// To use getch()
#include <iostream>
#include <string>
using namespace std;

/* Third-party Library */
#include "IPCserver/client.hpp"
//#include "RobotAction/RobotAction.h"

//** Problem Dependent Variable Setting **//
//#define SERVER_NAME "localhost"
#define SERVER_NAME "192.168.11.4"

/** Declration of Variables **/
char keyboardInput = '\0';

//=============================================================================
int main(int argc, char** argv) {
		// Server communication
	//RobotAction robotAction(SERVER_NAME);
	init_comm();
	connect_to_server();
	subscribe(TOTAL_MSG_NUM);
	publish(KEY_WORD, ATTENTIONLEVEL, TOTAL_MSG_NUM);
	listen();

	cout << "\n\t< Wizard of Oz - TAROS 2014 >" << endl;

	while (keyboardInput != 'Q') {
		keyboardInput = _getch();
		switch (keyboardInput) {
			case 'Q':
				cout << keyboardInput << endl;
				break;
			
			case 'A':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}
			case 'Z':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}
			case 'W':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}
			case 'S':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}
			case 'X':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}
			case 'E':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}
			case 'D':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}

			case 'C':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				AttentionLevelMgr ATR;
				ATR.attentionLevel = static_cast< AttentionLevel_HAE_type >(1);
				sendAttentionLevel(ATR);
				Sleep(sizeof(ATR));
				break;
			}

			case 'R':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}

			case 'F':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}

			case 'V':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}

			case 'T':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}

			case 'G':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}

			case 'B':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}

			case 'Y':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}

			case 'H':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				AttentionLevelMgr ATR;
				ATR.attentionLevel = static_cast< AttentionLevel_HAE_type >(2);
				sendAttentionLevel(ATR);
				Sleep(sizeof(ATR));
			}

			case 'N':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				KeyWordMgr testName;
				sprintf(testName.keyword, "Tim");
				sendKeyWord(testName);
				Sleep(sizeof(testName));
				break;
			}

			case 'U':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}

			case 'J':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}

			case 'M':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}

			case 'I':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}
			case 'K':
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;

			case 'O':
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;

			case 'L':
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;

			case 'P':
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
		}

		Sleep(1000);
	}

	cout << "> INFO: The program is end by user." << endl;
	return 0;
}