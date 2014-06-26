
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

/** Declration of Variables **/
char keyboardInput = '\0';

//=============================================================================
//int main() {
//	PeopleMgr targetPos;
//	targetPos.x[0] = 210.0;
//	targetPos.y[0] = 54;
//
//	OdometryMgr curPos;
//	curPos.x = 0.0;
//	curPos.y = 0.0;
//	curPos.theta = 0.0;
//
//	double r = sqrt(pow(targetPos.x[0] / 100.0, 2) + pow(targetPos.y[0] / 100.0, 2));
//	double theta_h = -1 * 41.0;
//	double theta_r = atan2(targetPos.y[0], targetPos.x[0]) * 180 / M_PI;
//
//	double Xgh = r;
//	double Ygh = 0;
//	double thetag = 180.0;
//
//		// Translation to robot_coordinate
//	double Xgh_R = Xgh - r * cos(theta_h / 180 * M_PI);
//	double Ygh_R = Ygh - r * sin(theta_h / 180 * M_PI);
//		// Rotation
//	double radianHtoR = (180.0 + theta_h - theta_r) / 180.0 * M_PI;
//	double Xgr = cos(radianHtoR) * Xgh_R + sin(radianHtoR) * Ygh_R;
//	double Ygr = -1 * sin(radianHtoR) * Xgh_R + cos(radianHtoR) * Ygh_R;
//	double thetar = thetag + (180.0 + theta_h - theta_r);
//
//	return 0;
//}

int main() {
	RobotAction robotAction(SERVER_NAME);

	while(1) {
		for( int i(-90); i < 90; i += 5)
			robotAction.movingToFrontOfHuman(0, i);
		Sleep(100);
	}

	return 0;
}

int main2(int argc, char** argv) {
		// Server communication
	/*init_comm();
	connect_to_server(SERVER_NAME);
	subscribe(RESULT_NAVI, RESULT_ARM, RESULT_SPEAK, HAE, ATTENTIONLEVEL, TOTAL_MSG_NUM);
	publish(KEY_WORD, SUBGOAL, ACTION_NAVI, ACTION_ARM, ACTION_SPEAK, PERCEPTION_HAE, REQUEST_INFERENCE, TOTAL_MSG_NUM);
	listen();*/

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

				robotAction.headShake(15, -45.0);
				break;
			}
			case 'E':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.turningFace(-45.0);
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

				robotAction.turningFace(0.0);
				break;
			}

			case 'V':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.armWave();
				break;
			}

			case 'T':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.makeSounds("C:\\Users\\Mac\\Desktop\\Shu\\ThesisProgram\\models\\Soundness\\AOE_bugle_2.wav");
				break;
			}

			case 'G':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.makeSounds("C:\\Users\\Mac\\Desktop\\Shu\\ThesisProgram\\models\\Soundness\\WindowsMessage.wav");
				break;
			}

			case 'B':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.speaking(string(PARTNERNAME));
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

				robotAction.speaking("There is a message left for you, do you want to reply it now?");
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

				//robotAction.movingToFrontOfHuman(0);
				break;
			}

			case 'I':
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;

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
	}
	cout << "The program is end by user." << endl;
	disconnect_to_server();
	return 0;
}