
/* Standrad Included Library */
#include <conio.h>		// To use getch()
#include <iostream>
#include <string>
using namespace std;

//** Problem Dependent Variable Setting **//

/** Declration of Variables **/
char keyboardInput = '\0';

void Exec(string ProgramName);
void Kill(string ProgramName);

//=============================================================================
int main(int argc, char** argv) {
	cout << "\n\t< OneKeyToOpen >" << endl;

	while (keyboardInput != 'Q') {
		keyboardInput = _getch();
		switch (keyboardInput) {
			case 'Q':
				cout << keyboardInput << endl;
				break;

			case 'A':
				cout << keyboardInput << endl;
				Exec("IPCServer");
				break;

			case 'Z':
				cout << keyboardInput << endl;
				Kill("IPCServer");
				break;

			case 'B':
				cout << keyboardInput << endl;
				Exec("BodyDirection");
				break;

			// Start all programs
			case 'S':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
					// Human-Aware Interactive Learner
				Exec("AdaptiveInitEngagementAgent");
					// RobotAction components
				Exec("Navi_echo");
				Exec("ArmHead");
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

			// Kill all programs
			case 'E':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';
					// Attention-Aware Interactive Learner
				Kill("AdaptiveInitEngagementAgent");
					// RobotAction components
				Kill("Navi_echo");
				Kill("ArmHead");
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

			// Execute the programs related to motors
			case 'M':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				Exec("Navi_echo");
				Exec("ArmHead");
				break;
			}

			// Kill the programs related to motors
			case 'K':
			{
				cout << keyboardInput << endl;
				keyboardInput = '\0';

				Kill("Navi_echo");
				Kill("ArmHead");
				break;
			}
		}
	}

	cout << "The program is end by user." << endl;
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
		psexecProgDir	= "-w E:\\desktop\\Tung-Yen\\Human_Tracking_2.0\\Debug\\ ";
		psexecProgName	= "E:\\desktop\\Tung-Yen\\Human_Tracking_2.0\\Debug\\HumanTracking_newmac.exe ";

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
		pskillProgName	= "HumanTracking_newmac.exe ";

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
	else
		cout << "Kill:No " << ProgramName << endl;
}