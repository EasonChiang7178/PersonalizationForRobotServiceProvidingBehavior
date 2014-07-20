
/* Standrad Included Library */
#include <conio.h>		// To use getch()
#include <iostream>
#include <string>
using namespace std;

/* Third-party Library */
#include "IPCserver/client.hpp"
#include "RobotAction/RobotAction.h"

//** Problem Dependent Variable Setting **//
#define PARTNERNAME "®a©ú"
#define SERVER_NAME "localhost"
//#define SERVER_NAME "192.168.11.4"

	// LCM core
#include "lcm\lcm-cpp.hpp"
	// LCM message data type
#include "lcm\BodyDirectionLcm.hpp"
	// LCM shared consts
//#include "lcm\LcmComm.hpp"
using namespace lcm;

/** Declration of Variables **/
char keyboardInput = '\0';

//=============================================================================
int main(int argc, char** argv) {
		// Server communication
	/*init_comm();
	connect_to_server(SERVER_NAME);
	subscribe(RESULT_NAVI, RESULT_ARM, RESULT_SPEAK, HAE, ATTENTIONLEVEL, TOTAL_MSG_NUM);
	publish(KEY_WORD, SUBGOAL, ACTION_NAVI, ACTION_ARM, ACTION_SPEAK, PERCEPTION_HAE, REQUEST_INFERENCE, TOTAL_MSG_NUM);
	listen();*/

	/* Initialize LCM */
	LCM lcm("udpm://239.255.76.67:7667?ttl=1");
	if (!lcm.good())
	{
		cout << "> ERROR: Cannot initialize LCM" << endl;
		return 1;
	}

	RobotAction robotAction(SERVER_NAME);

	cout << "\n\t< Keyboard Event Trigger >" << endl;

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
				
				string accepted = "ARIO";
				KeyWordMgr recognizedSpeech;
				sprintf(recognizedSpeech.keyword, accepted.c_str());
				sendKeyWord(recognizedSpeech);
				Sleep(sizeof(recognizedSpeech));
				break;
			}
			case 'Z':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				string accepted = "Rejected";
				KeyWordMgr recognizedSpeech;
				sprintf(recognizedSpeech.keyword, accepted.c_str());
				sendKeyWord(recognizedSpeech);
				Sleep(sizeof(recognizedSpeech));
				break;
			}
			case 'W':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.toPoint(1.8, 0.0, 90.0);
				break;
			}
			case 'S':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.toPoint(1.8, 0.6, 90.0);
				break;
			}
			case 'X':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.headShake(15, -45);
				break;
			}
			case 'E':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.turningFace(-45);
				break;
			}
			case 'D':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.toPoint(0.6, 0.6, 45.0);
				break;
			}

			case 'C':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.rotation(30);
				break;
			}

			case 'R':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.toPoint(0.0, 0.0, 0.0);
				break;
			}

			case 'F':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.turningFace(0);
				break;
			}

			case 'V':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.armWave(5);
				robotAction.armWave(4);
				robotAction.armWave(3);
				robotAction.armWave(2);
				robotAction.armWave(1);
				break;
			}

			case 'T':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.makeSounds("C:\\Users\\Mac\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\models\\Soundness\\AOE_bugle_2.wav");
				break;
			}

			case 'G':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.makeSounds("C:\\Users\\Mac\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\models\\Soundness\\WindowsMessage.wav");
				break;
			}

			case 'B':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.speaking(string(PARTNERNAME), 0.7f);
				break;
			}

			case 'Y':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.sensingFD(200);
				cout << robotAction.getFaceDirection() << endl;
				break;
			}

			case 'N':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.speaking("There is a message left for you, do you want to reply it now?", 0.7f);
				break;
			}

			case 'U':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.forwardApproach(0, 0.75);
				break;
			}

			case 'J':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.turnFaceToHuman();
				break;
			}

			case 'M':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.movingToAroundOfHuman(0, 1.5, -30.0);
				break;
			}

			case 'I':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.sensingPU(200);
				int pu = robotAction.getPU();
				cout << "PU: " << pu << endl;
				break;
			}
			case 'K':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				int puInput = 2;
				cout << "> pu: ";
				cin >> puInput;

				BodyDirectionLcm bodyMgr;
				bodyMgr.pu = puInput;
				lcm.publish("BODY_DIRECTION", &bodyMgr);

				break;
			}

			case 'O':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				AttentionLevelMgr ATR;
				ATR.attentionLevel = static_cast< AttentionLevel_HAE_type >(0);
				sendAttentionLevel(ATR);
				Sleep(sizeof(ATR));
				break;
			}

			case 'L':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				AttentionLevelMgr ATR;
				ATR.attentionLevel = static_cast< AttentionLevel_HAE_type >(1);
				sendAttentionLevel(ATR);
				Sleep(sizeof(ATR));
				break;
			}

			case 'P':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				AttentionLevelMgr ATR;
				ATR.attentionLevel = static_cast< AttentionLevel_HAE_type >(2);
				sendAttentionLevel(ATR);
				Sleep(sizeof(ATR));
				break;
			}
		}
	}
	cout << "The program is end by user." << endl;
	disconnect_to_server();
	return 0;
}