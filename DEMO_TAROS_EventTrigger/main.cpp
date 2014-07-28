
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

void Exec(string ProgramName);
void Kill(string ProgramName);

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
			
				// Activate IPC server
			case 'A':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				Exec("IPCServer");
				break;
			}

				// Deactivate IPC server
			case 'Z':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				Kill("IPCServer");
				break;
			}
			case 'W':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}
				// Start all programs
			case 'S':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
					// DEMO main body
				Exec("TAROS_2014");
					// RobotAction components
				Exec("GraspingExcuting");
				Exec("ArmHead_ST");
				Exec("HumanTracking");
				Exec("uTTS");
					// Perception components
				Exec("BodyDirection");
				Exec("FaceDetection");
				Exec("VoiceActivityDetector");
					// Social Attention Inference Model
				Exec("SocialAttentionInferenceModel");
				break;
			}
			case 'X':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}

				// Kill all programs
			case 'E':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
					// DEMO main body
				Kill("TAROS_2014");
					// RobotAction components
				Kill("GraspingExcuting");
				Kill("ArmHead_ST");
				Kill("HumanTracking");
				Kill("uTTS");
					// Perception components
				Kill("BodyDirection");
				Kill("FaceDetection");
				Kill("VoiceActivityDetector");
					// Social Attention Inference Model
				Kill("SocialAttentionInferenceModel");
				break;
			}
			case 'D':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}

				// Sending "Contingency"
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

				// Turning robot to the cabinet
			case 'T':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				/* Prepare subgoal message */
				SubgoalMgr goal;
				goal.x = 0.0;
				goal.y = 0.0;
				goal.theta = 90.0;

				/* Send subgoal message */
				sendSubgoal(goal);
				Sleep(sizeof(goal) + 1000);
				break;
			}

				// Commend to grasp
			case 'G':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				/* Preparing the action information */
				ActionSearchGrasp graspAction;
				graspAction.start = true;
				graspAction.mode = GRASP_MODE;
				graspAction.goal_x = 0.0;
				graspAction.goal_y = 0.0;
				graspAction.goal_theta = 0.0;

					// Sending!
				sendActionSearchGrasp(graspAction);
				Sleep(sizeof(graspAction) + 1000);
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

				// Sending "HighAttention"
			case 'H':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				AttentionLevelMgr ATR;
				ATR.attentionLevel = static_cast< AttentionLevel_HAE_type >(2);
				sendAttentionLevel(ATR);
				Sleep(sizeof(ATR));
			}

				// Sending user name for testing
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

				// Execute the programs related to motors
			case 'M':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				Exec("GraspingExcuting");
				Exec("ArmHead_ST");
				break;
			}

			case 'I':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
				break;
			}

				// Kill the programs related to motors
			case 'K':
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				Kill("GraspingExcuting");
				Kill("ArmHead_ST");
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

void Exec(string ProgramName){
	string psexecDir, psexecHost, psexecUser, psexecPswd;
	string psexecParam, psexecProgDir, psexecProgName;
	string systemCallParam;

	psexecDir = "..\\models\\pstool\\psexec ";
	psexecParam = "/accepteula -d -i 1 ";

	if(ProgramName == "IPCServer"){
		psexecHost		= "\\\\192.168.11.4 ";
		psexecUser		= "-u robot ";
		psexecPswd		= "-p robot ";
		psexecProgDir	= "-w C:\\Users\\robot\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\ ";
		psexecProgName	= "C:\\Users\\robot\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\central3.9.1.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else if(ProgramName == "uTTS"){
		psexecHost		= "\\\\192.168.11.3 ";
		psexecUser		= "-u mac ";
		psexecPswd		= "-p mac ";
		psexecProgDir	= "-w C:\\Users\\Mac\\Desktop\\Shu\\uTTS_finish_volumeControlable\\uTTS\\Debug\\ ";
		psexecProgName	= "C:\\Users\\Mac\\Desktop\\Shu\\uTTS_finish_volumeControlable\\uTTS\\Debug\\uTTS.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else if(ProgramName == "AdaptiveInitEngagementAgent"){
		psexecHost		= "\\\\192.168.11.3 ";
		psexecUser		= "-u mac ";
		psexecPswd		= "-p mac ";
		psexecProgDir	= "-w C:\\Users\\Mac\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\ ";
		psexecProgName	= "C:\\Users\\Mac\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\Human-AwareInteractiveLearner.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else if(ProgramName == "HumanTracking"){
		psexecHost		= "\\\\192.168.11.2 ";
		psexecUser		= "-u robot ";
		psexecPswd		= "-p robot ";
		psexecProgDir	= "-w E:\\desktop\\Shu\\Human_Tracking_2.0_lcm\\Debug\\ ";
		psexecProgName	= "E:\\desktop\\Shu\\Human_Tracking_2.0_lcm\\Debug\\HumanTracking.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else if(ProgramName == "Navi_echo"){
		psexecHost		= "\\\\192.168.11.2 ";
		psexecUser		= "-u robot ";
		psexecPswd		= "-p robot ";
		psexecProgDir	= "-w E:\\desktop\\Navi2.3.2_Echo\\Release\\ ";
		psexecProgName	= "E:\\desktop\\Navi2.3.2_Echo\\Release\\Navi2.0.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else if(ProgramName == "ArmHead"){
		psexecHost		= "\\\\192.168.11.2 ";
		psexecUser		= "-u robot ";
		psexecPswd		= "-p robot ";
		psexecProgDir	= "-w E:\\desktop\\Shu\\HeadArm_Component\\Debug\\ ";
		psexecProgName	= "E:\\desktop\\Shu\\HeadArm_Component\\Debug\\HeadArm.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else if(ProgramName == "BodyDirection"){
		psexecHost		= "\\\\192.168.11.4 ";
		psexecUser		= "-u robot ";
		psexecPswd		= "-p robot ";
		psexecProgDir	= "-w C:\\Users\\robot\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\ ";
		psexecProgName	= "C:\\Users\\robot\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\BodyDirection.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else if(ProgramName == "FaceDetection"){
		psexecHost		= "\\\\192.168.11.3 ";
		psexecUser		= "-u mac ";
		psexecPswd		= "-p mac ";
		psexecProgDir	= "-w C:\\Users\\Mac\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\ ";
		psexecProgName	= "C:\\Users\\Mac\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\FaceDetection.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else if(ProgramName == "VoiceActivityDetector"){
		psexecHost		= "\\\\192.168.11.3 ";
		psexecUser		= "-u mac ";
		psexecPswd		= "-p mac ";
		psexecProgDir	= "-w C:\\Users\\Mac\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\ ";
		psexecProgName	= "C:\\Users\\Mac\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\VoiceActivityDetector.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else if(ProgramName == "SocialAttentionInferenceModel"){
		psexecHost		= "\\\\192.168.11.4 ";
		psexecUser		= "-u robot ";
		psexecPswd		= "-p robot ";
		psexecProgDir	= "-w C:\\Users\\robot\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\ ";
		psexecProgName	= "C:\\Users\\robot\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\SAM_Inference.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else if(ProgramName == "GraspingExcuting"){
		psexecHost		= "\\\\192.168.11.4 ";
		psexecUser		= "-u robot ";
		psexecPswd		= "-p robot ";
		psexecProgDir	= "-w C:\\Users\\robot\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\ ";
		psexecProgName	= "C:\\Users\\robot\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\SAM_Inference.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else if(ProgramName == "ArmHead_ST"){
		psexecHost		= "\\\\192.168.11.4 ";
		psexecUser		= "-u robot ";
		psexecPswd		= "-p robot ";
		psexecProgDir	= "-w C:\\Users\\robot\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\ ";
		psexecProgName	= "C:\\Users\\robot\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\SAM_Inference.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else
		cout << "No " << ProgramName << endl;

}

void Kill(string ProgramName){

	string pskillDir, pskillHost, pskillUser, pskillPswd;
	string pskillParam, pskillProgName; 
	string systemCallParam;

	if(ProgramName == "IPCServer"){
		pskillDir = "..\\models\\pstool\\pskill ";
		pskillParam = "";

		pskillHost		= "\\\\192.168.11.4 ";
		pskillUser		= "-u robot ";
		pskillPswd		= "-p robot ";
		pskillProgName	= "central3.9.1.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else if(ProgramName == "uTTS"){
		pskillDir = "..\\models\\pstool\\pskill ";
		pskillParam = "";

		pskillHost		= "\\\\192.168.11.3 ";
		pskillUser		= "-u mac ";
		pskillPswd		= "-p mac ";
		pskillProgName	= "uTTS.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else if(ProgramName == "AdaptiveInitEngagementAgent"){
		pskillDir = "..\\models\\pstool\\pskill ";
		pskillParam = "";

		pskillHost		= "\\\\192.168.11.3 ";
		pskillUser		= "-u mac ";
		pskillPswd		= "-p mac ";
		pskillProgName	= "Human-AwareInteractiveLearner.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else if(ProgramName == "HumanTracking"){
		pskillDir = "..\\models\\pstool\\pskill ";
		pskillParam = "";

		pskillHost		= "\\\\192.168.11.2 ";
		pskillUser		= "-u robot ";
		pskillPswd		= "-p robot ";
		pskillProgName	= "HumanTracking.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else if(ProgramName == "Navi_echo"){
		pskillDir = "..\\models\\pstool\\pskill ";
		pskillParam = "";

		pskillHost		= "\\\\192.168.11.2 ";
		pskillUser		= "-u robot ";
		pskillPswd		= "-p robot ";
		pskillProgName	= "Navi2.0.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else if(ProgramName == "ArmHead"){
		pskillDir = "..\\models\\pstool\\pskill ";
		pskillParam = "";

		pskillHost		= "\\\\192.168.11.2 ";
		pskillUser		= "-u robot ";
		pskillPswd		= "-p robot ";
		pskillProgName	= "HeadArm.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else if(ProgramName == "BodyDirection"){
		pskillDir = "..\\models\\pstool\\pskill ";
		pskillParam = "";

		pskillHost		= "\\\\192.168.11.4 ";
		pskillUser		= "-u robot ";
		pskillPswd		= "-p robot ";
		pskillProgName	= "BodyDirection.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else if(ProgramName == "FaceDetection"){
		pskillDir = "..\\models\\pstool\\pskill ";
		pskillParam = "";

		pskillHost		= "\\\\192.168.11.3 ";
		pskillUser		= "-u mac ";
		pskillPswd		= "-p mac ";
		pskillProgName	= "FaceDetection.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else if(ProgramName == "VoiceActivityDetector"){
		pskillDir = "..\\models\\pstool\\pskill ";
		pskillParam = "";

		pskillHost		= "\\\\192.168.11.3 ";
		pskillUser		= "-u mac ";
		pskillPswd		= "-p mac ";
		pskillProgName	= "VoiceActivityDetector.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else if(ProgramName == "SocialAttentionInferenceModel"){
		pskillDir = "..\\models\\pstool\\pskill ";
		pskillParam = "";

		pskillHost		= "\\\\192.168.11.4 ";
		pskillUser		= "-u robot ";
		pskillPswd		= "-p robot ";
		pskillProgName	= "SAM_Inference.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else if(ProgramName == "GraspingExcuting"){
		pskillDir = "..\\models\\pstool\\pskill ";
		pskillParam = "";

		pskillHost		= "\\\\192.168.11.4 ";
		pskillUser		= "-u robot ";
		pskillPswd		= "-p robot ";
		pskillProgName	= "SAM_Inference.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else if(ProgramName == "ArmHead_ST"){
		pskillDir = "..\\models\\pstool\\pskill ";
		pskillParam = "";

		pskillHost		= "\\\\192.168.11.4 ";
		pskillUser		= "-u robot ";
		pskillPswd		= "-p robot ";
		pskillProgName	= "SAM_Inference.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else
		cout << "Kill:No " << ProgramName << endl;
}