
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

//#define TESTGRASPING

/* For IPC handler using */
bool isUserNameReceived = false;

void Exec(string ProgramName);
void Kill(string ProgramName);

//=============================================================================
int main(int argc, char* argv[]) {
		// Server communication
	RobotAction robot(SERVER_NAME);

	cout << "\n\t< TAROS 2014 >" << endl;

#ifndef TESTGRASPING
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
		userName = tempName;
		cout << "> INFO: User name " + userName + " is received!" << endl;
	} else
		cout << "> WARNING: The user name is empty! Using \"Hello\" to represent..." << endl;

	//robot.speaking(userName + "，您好", 0.9f);
	Exec("FaceDetection");
	Sleep(3000);
	Exec("SocialAttentionInferenceModel");
	Sleep(2000);

	/* Session: Attracting user's attention */
	cout << "> INFO: Start to grab user's attention..." << endl;
		// Reset all flags for social attention (ToA is referred for Theory of Awareness)
	robot.resetToA();
		// Voice volume for the ARIO
	float voiceVolume = 0.1f;
		// Grabing user's attention by calling his/her name!
	//for (int i = 0; robot.getHighAttentionFlag() == 0 && i < 5; i++) {
	//	robot.speaking(userName, voiceVolume);
	//	if (voiceVolume < 1.0)
	//		voiceVolume += 0.3f;
	//	Sleep(2500);

	//	if (robot.getContingencyFlag() > 0 && robot.getHighAttentionFlag() == 0) {
	//		robot.speaking("請注意這裡一下", 0.7f);
	//		robot.resetContingencyFlag();
	//		Sleep(5000);
	//	}
	//}
		// Timeout occurred! Waiting for more time
	//for (int i = 0; robot.getHighAttentionFlag() == 0 && i < 10; i++) {
	//	cout << "> WARNING: User didn't notice me!" << endl;
	//	Sleep(1000);
	//	if (robot.getHighAttentionFlag() > 0)
	//		break;
	//}
	
	robot.speaking(userName + "，我有幫你準備了一份小禮物，麻煩請稍帶片刻", 0.8f);
	Kill("FaceDetection");
	Kill("SocialAttentionInferenceModel");
#endif // TESTGRASPING

	/* Session: Grasping */
		// Turn to the cabinet
	cout << "> ACTION: Turning to the cabinet..." << endl;
	robot.toPoint(0.0, 0.0, 91.0);
		// Grasping!
	cout << "> ACTION: GRASPING!" << endl;
	robot.graspingNil(0.5, 0.0, 0.0);

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
		psexecProgDir	= "-w C:\\Users\\Mac\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Release\\ ";
		psexecProgName	= "C:\\Users\\Mac\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Release\\FaceDetection.exe ";

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
		psexecHost		= "\\\\192.168.11.2 ";
		psexecUser		= "-u robot ";
		psexecPswd		= "-p robot ";
		psexecProgDir	= "-w E:\\desktop\\Search_Grasp_Executing\\Release\\ ";
		psexecProgName	= "E:\\desktop\\Search_Grasp_Executing\\Release\\GraspingExecuting.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else if(ProgramName == "ArmHead_ST"){
		psexecHost		= "\\\\192.168.11.2 ";
		psexecUser		= "-u robot ";
		psexecPswd		= "-p robot ";
		psexecProgDir	= "-w E:\\desktop\\Shao-Ting\\HeadArm_Component_2014_DEMO\\Debug\\ ";
		psexecProgName	= "E:\\desktop\\Shao-Ting\\HeadArm_Component_2014_DEMO\\Debug\\HeadArm.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else if(ProgramName == "GraspingNil"){
		psexecHost		= "\\\\192.168.11.4 ";
		psexecUser		= "-u robot ";
		psexecPswd		= "-p robot ";
		psexecProgDir	= "-w C:\\Users\\robot\\Desktop\\SearchPlanning_backup\\build\\Release ";
		psexecProgName	= "C:\\Users\\robot\\Desktop\\SearchPlanning_backup\\build\\Release\\SearchPlanning.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else if(ProgramName == "TAROS_2014"){
		psexecHost		= "\\\\192.168.11.4 ";
		psexecUser		= "-u robot ";
		psexecPswd		= "-p robot ";
		psexecProgDir	= "-w C:\\Users\\robot\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\ ";
		psexecProgName	= "C:\\Users\\robot\\Desktop\\Shu\\PersonalizationForRobotServiceProvidingBehavior\\Bin\\Debug\\DEMO_TAROS_2014.exe ";

		systemCallParam = psexecDir + psexecHost + psexecUser + psexecPswd + 
			psexecParam + psexecProgDir + psexecProgName;
		system(systemCallParam.c_str());

		cout << "Exec:" << ProgramName << endl;
	}
	else if(ProgramName == "FaceRecognition"){
		psexecHost		= "\\\\192.168.11.3 ";
		psexecUser		= "-u mac ";
		psexecPswd		= "-p mac ";
		psexecProgDir	= "-w C:\\Users\\Mac\\Desktop\\TSChu\\TIROS_0730_FACE_DEMO\\Debug\\ ";
		psexecProgName	= "C:\\Users\\Mac\\Desktop\\TSChu\\TIROS_0730_FACE_DEMO\\Debug\\FaceRecog.exe ";

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

		pskillHost		= "\\\\192.168.11.2 ";
		pskillUser		= "-u robot ";
		pskillPswd		= "-p robot ";
		pskillProgName	= "GraspingExecuting.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else if(ProgramName == "ArmHead_ST"){
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
	else if(ProgramName == "GraspingNil"){
		pskillDir = "..\\models\\pstool\\pskill ";
		pskillParam = "";

		pskillHost		= "\\\\192.168.11.4 ";
		pskillUser		= "-u robot ";
		pskillPswd		= "-p robot ";
		pskillProgName	= "SearchPlanning.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else if(ProgramName == "TAROS_2014"){
		pskillDir = "..\\models\\pstool\\pskill ";
		pskillParam = "";

		pskillHost		= "\\\\192.168.11.4 ";
		pskillUser		= "-u robot ";
		pskillPswd		= "-p robot ";
		pskillProgName	= "DEMO_TAROS_2014.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else if(ProgramName == "FaceRecognition"){
		pskillDir = "..\\models\\pstool\\pskill ";
		pskillParam = "";

		pskillHost		= "\\\\192.168.11.3 ";
		pskillUser		= "-u mac ";
		pskillPswd		= "-p mac ";
		pskillProgName	= "FaceRecog.exe ";

		systemCallParam = pskillDir + pskillHost + pskillUser + pskillPswd + 
			pskillParam + pskillProgName;

		system(systemCallParam.c_str());

		cout << "Kill:" << ProgramName << endl;
	}
	else
		cout << "Kill:No " << ProgramName << endl;
}