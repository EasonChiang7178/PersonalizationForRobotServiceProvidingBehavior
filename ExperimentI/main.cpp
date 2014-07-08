
/* Standrad Included Library */
#include <conio.h>		// To use getch()
#include <iostream>
#include <string>
using namespace std;

/* Third-party Library */
#include "IPCserver/client.hpp"
#include "RobotAction/RobotAction.h"

//** Problem Dependent Variable Setting **//
#define PARTNERNAME "Chen"
//#define SERVER_NAME "localhost"
#define SERVER_NAME "192.168.11.4"

/** Declration of Variables **/
char keyboardInput = '\0';

//=============================================================================
int main(int argc, char** argv) {
		// Server communication
	RobotAction robotAction(SERVER_NAME);

	cout << "\n\t< Wizard of Oz >" << endl;

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
				
				robotAction.setMotionSpeed(1);
				robotAction.forwardApproach(0, 0.9);
				break;
			}
			case 'Z':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setMotionSpeed(1);
				robotAction.movingToAroundOfHuman(0, 1.5, -30.0);
				break;
			}
			case 'W':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setMotionSpeed(2);
				robotAction.armWave(1);
				break;
			}
			case 'S':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setMotionSpeed(1);
				robotAction.armWave(2);
				break;
			}
			case 'X':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setMotionSpeed(0);
				robotAction.armWave(3);
				break;
			}
			case 'E':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setSpeechVolume(2);
				robotAction.makeSounds("C:\\Users\\Mac\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\models\\Soundness\\AOE_bugle_2.wav");
				break;
			}
			case 'D':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setSpeechVolume(2);
				robotAction.makeSounds("C:\\Users\\Mac\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\models\\Soundness\\WindowsMessage.wav");
				break;
			}

			case 'C':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				
				break;
			}

			case 'R':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setSpeechVolume(2);
				robotAction.speaking(PARTNERNAME, 0.9f);
				break;
			}

			case 'F':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setSpeechVolume(1);
				robotAction.speaking(PARTNERNAME, 0.6f);
				break;
			}

			case 'V':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setSpeechVolume(0);
				robotAction.speaking(PARTNERNAME, 0.3f);
				break;
			}

			case 'T':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setSpeechVolume(1);
				robotAction.speaking("�A�n�A�ڬO�R�L", 0.6f);
				Sleep(3456);
				robotAction.setSpeechVolume(2);
				robotAction.speaking("�������p", 0.9);
				break;
			}

			case 'G':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setSpeechVolume(1);
				robotAction.speaking("�A�n�A�ڬO�R�L�������H�A�ګD�`��F��", 0.7f);
				break;
			}

			case 'B':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setSpeechVolume(0);
				robotAction.speaking("����" + string(PARTNERNAME) + "�A�а�", 0.3f);
				Sleep(4000);
				robotAction.setSpeechVolume(0);
				robotAction.speaking("����", 0.3f);
				Sleep(3000);
				robotAction.setSpeechVolume(1);
				robotAction.speaking("����������������", 0.6f);
				Sleep(2000);
				robotAction.setSpeechVolume(2);
				robotAction.speaking("�������������������w���w���w���w", 0.9f);
				break;
			}

			case 'Y':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setSpeechVolume(1);
				robotAction.speaking("����" + string(PARTNERNAME) + "�A�����@�ӬG�Ƶ��Ať", 0.6f);
				break;
			}

			case 'N':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setSpeechVolume(1);
				robotAction.speaking("���@�ѡA�B�ͶR�F�@���ơA��⪺���l�a�զ���A��o�����ڭ̬ݮɡA", 0.6f);
				robotAction.speaking("�@����ѤQ���P���쪺�P�ǻ��G", 0.6f);
				robotAction.setSpeechVolume(2);
				robotAction.speaking("�ڡA�n���ѽL����", 0.9f);
				robotAction.setSpeechVolume(1);
				robotAction.speaking("�ڬݭ˦��I���Z�ȡC�ڻ�", 0.6f);
				robotAction.speaking("�u���@�����񨧿|�C�@��~���s�j���Ȫ��P�Ǻ򱵵ۻ��C", 0.6f);
				robotAction.speaking("�ڭ̤��T����j���A�P�˪��@���ơA", 0.6f);
				robotAction.setSpeechVolume(0);
				robotAction.speaking("���L�H����", 0.3f);
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
		robotAction.setSpeechVolume(0);
		robotAction.setMotionSpeed(0);
	}
	cout << "The program is end by user." << endl;
	disconnect_to_server();
	return 0;
}