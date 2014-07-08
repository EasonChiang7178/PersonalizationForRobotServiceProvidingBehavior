
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
				robotAction.speaking("你好，我是愛微", 0.6f);
				Sleep(3456);
				robotAction.setSpeechVolume(2);
				robotAction.speaking("爾機器喵", 0.9);
				break;
			}

			case 'G':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setSpeechVolume(1);
				robotAction.speaking("你好，我是愛微爾機器人，我非常能幹喔", 0.7f);
				break;
			}

			case 'B':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setSpeechVolume(0);
				robotAction.speaking("那個" + string(PARTNERNAME) + "，請問", 0.3f);
				Sleep(4000);
				robotAction.setSpeechVolume(0);
				robotAction.speaking("懂姿", 0.3f);
				Sleep(3000);
				robotAction.setSpeechVolume(1);
				robotAction.speaking("董姿董姿董姿董姿", 0.6f);
				Sleep(2000);
				robotAction.setSpeechVolume(2);
				robotAction.speaking("董姿董姿董姿董茲跳針跳針跳針跳針", 0.9f);
				break;
			}

			case 'Y':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setSpeechVolume(1);
				robotAction.speaking("那個" + string(PARTNERNAME) + "，我講一個故事給你聽", 0.6f);
				break;
			}

			case 'N':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				robotAction.setSpeechVolume(1);
				robotAction.speaking("有一天，朋友買了一件衣料，綠色的底子帶白色方格，當她拿給我們看時，", 0.6f);
				robotAction.speaking("一位對圍棋十分感興趣的同學說：", 0.6f);
				robotAction.setSpeechVolume(2);
				robotAction.speaking("啊，好像棋盤似的", 0.9f);
				robotAction.setSpeechVolume(1);
				robotAction.speaking("我看倒有點像稿紙。我說", 0.6f);
				robotAction.speaking("真像一塊塊綠豆糕。一位外號叫大食客的同學緊接著說。", 0.6f);
				robotAction.speaking("我們不禁哄堂大笑，同樣的一件衣料，", 0.6f);
				robotAction.setSpeechVolume(0);
				robotAction.speaking("關他人屁事", 0.3f);
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