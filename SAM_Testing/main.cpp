
/* Standrad Included Library */
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <list>
#include <map>
#include <conio.h>			// To use getch(), kbhit()
using namespace std;

/* Third-party Library */
	// To use bayesain network and it's inference algorithm
#include "SmileInference\SmileInference.h"

//** Problem Dependent Variable Setting **//
	// The timestep of the dynamic Bayesian network
#define STEPNUMBER	4
	// How long a timestep is (ms)
#define STEPTIME	250
	// How long a delay for messages request
#define DELAYTIME	200

/** Declration of Variables **/
	// Path to unrolled DBN model
string dynamicBN = "../models/DBN_Model/res_SocialAttentionInferenceModel_Unrolled.xdsl";
	// Path to the test data
string testingDataPath = "../models/DBN_Model/TrainingData/SocialAttentionInferenceModel_Learning.txt";
	// Path to the output result
string cmPath = "../models/DBN_Model/TrainingData/SocialAttentionInference_Testing_cm.txt";
	// Buffer result for Human Attention Level
static int attentionLevel = 0;

	// Observation of HAE
typedef enum {None_HAE_Face, Far_Center_Face, Near_Center_Face, Center_Face} Face_Direction_HAE_type;
typedef enum {None_HAE_Body, Far_Center_Body, Near_Center_Body, Center_Body} Body_Direction_HAE_type;
class HAEMgr
{
public:
	Face_Direction_HAE_type		face_direction;
	Body_Direction_HAE_type		body_direction;
	int							voice_detection;	// 0, No Voice; 1, Voice Exists
};
#define HAE_NAME	"HAEMgr"
#define HAE_FORMAT	"{int, int, int}"

	// State of HAE, output to RL agent
typedef enum{Neglect, AttentionMedium, AttentionHigh, InterestInRobot} AttentionLevel_HAE_type;
class AttentionLevelMgr{
public:
	AttentionLevel_HAE_type attentionLevel;		// content of query result
};
#define ATTENTIONLEVEL_NAME		"AttentionLevelMgr"
#define ATTENTIONLEVEL_FORMAT	"{int}"

HAEMgr sensingData;

//=============================================================================
int main(int argc, char* argv[]) {

	cout << "\n\t< Attention Level Testing >" << endl;

	/* Declaration and Initialzatoin of variables */
		// The path to unrolled network, only unrolled network can be inferred by SMILE lib
	if (argc > 1)
		dynamicBN = *(argv + 1);
		// Load the Bayesian network
	DSL_network theNet;
	if (theNet.ReadFile(dynamicBN.c_str(), DSL_XDSL_FORMAT) != DSL_OKAY) {
		cout << "> WARNING: Cannot read network... exiting" << endl;
		getchar();
		exit(1);
	}
		// Use clustering algorithm, actually it is the Junction Tree Algorithm
	theNet.SetDefaultBNAlgorithm(DSL_ALG_BN_LAURITZEN);
		// The nodes in the network
	vector< string > nameOfNodes;
	nameOfNodes.push_back("faceDirection");		nameOfNodes.push_back("bodyDirection");	nameOfNodes.push_back("audioInput");
	nameOfNodes.push_back("robotMotionSpeed");	nameOfNodes.push_back("robotPose");		nameOfNodes.push_back("robotSoundVolume");
	nameOfNodes.push_back("audioNoise");		nameOfNodes.push_back("attentionContext");
	nameOfNodes.push_back("attentionLevel");
		// The nodes in the network
	vector< string > evidencesNodesName;
	evidencesNodesName.push_back("faceDirection");		evidencesNodesName.push_back("bodyDirection");	evidencesNodesName.push_back("audioInput");
	evidencesNodesName.push_back("robotMotionSpeed");	evidencesNodesName.push_back("robotPose");		evidencesNodesName.push_back("robotSoundVolume");
		// Extract each nodes' attributes name in the network
	vector< map< int, string> > attributeNameArray;
	for (int i = 0; i < static_cast< int>(nameOfNodes.size()); i++)
		attributeNameArray.push_back(bn::getBNOutcomeNames(theNet, nameOfNodes[i]));
		// To store the history observation information
	int envirContext_audioLevel = 0;
	int envirContext_attenContext = 0;
	list< int > faceFeature;
	list< int > bodyFeature;
	list< int > audioFeature;
	list< int > robotSpeed;
	list< int > robotPose;
	list< int > robotVolume;
	//list< int > labelFeature;
		// String for nodes manipulate
	stringstream ss;
	string targetStep;
	ss << (STEPNUMBER - 1); ss >> targetStep;
		// Initialize confusion matrix
	vector< vector< int > > confusionMatrixHAE(3);
	for (int i = 0; i < static_cast< int >(confusionMatrixHAE.size()); i++)
		confusionMatrixHAE[i].resize(3);

	/* Open the testing data */
	fstream ftestData;
	ftestData.open(testingDataPath.c_str(), ios::in);
	if (!ftestData) {
		cout << "> WARNING: Fail to open the file: " << testingDataPath << endl
			 << ">          Please press enter to exit..." << endl;
		getchar();
	}

	string dummyStr;
	int inputFeature, inputLabel;
		// Drop out the unnessarey label
	ftestData >> dummyStr; ftestData >> dummyStr;
	for (int label = 0; label < 7; label++)
		for (int step = 0; step < STEPNUMBER; step++)
			ftestData >> dummyStr;

	/** Inference main loop **/
	int stepPassed = 0;

	while (true) {
		/* Press 'Q' (81), 'q' (113) or esc (27) to leave */
		if (kbhit()) {
			int command = getch();
			if (command == 81 || command == 113 || command == 27)
				break;
		}

		/* Setting the data received */
		/* AudioNoise */
		ftestData >> dummyStr;
		for (auto itOutcomeName = attributeNameArray[6].begin(); itOutcomeName != attributeNameArray[6].end(); itOutcomeName++)
			if (dummyStr == itOutcomeName->second)
				inputFeature = itOutcomeName->first;
		envirContext_audioLevel = inputFeature;

		/* Human Task Context */
		ftestData >> dummyStr;
		for (auto itOutcomeName = attributeNameArray[7].begin(); itOutcomeName != attributeNameArray[7].end(); itOutcomeName++)
			if (dummyStr == itOutcomeName->second)
				inputFeature = itOutcomeName->first;
		envirContext_attenContext = inputFeature;

		for (int j = 0; j < STEPNUMBER; j++) {
			/* Face */
			ftestData >> dummyStr;
			for (auto itOutcomeName = attributeNameArray[0].begin(); itOutcomeName != attributeNameArray[0].end(); itOutcomeName++)
				if (dummyStr == itOutcomeName->second)
					inputFeature = itOutcomeName->first;
			faceFeature.push_back(inputFeature);

			/* Body */
			ftestData >> dummyStr;
			for (auto itOutcomeName = attributeNameArray[1].begin(); itOutcomeName != attributeNameArray[1].end(); itOutcomeName++)
				if (dummyStr == itOutcomeName->second)
					inputFeature = itOutcomeName->first;
			bodyFeature.push_back(inputFeature);

			/* Audio */
			ftestData >> dummyStr;
			for (auto itOutcomeName = attributeNameArray[2].begin(); itOutcomeName != attributeNameArray[2].end(); itOutcomeName++)
				if (dummyStr == itOutcomeName->second)
					inputFeature = itOutcomeName->first;
			audioFeature.push_back(inputFeature);

			/* Robot Speed */
			ftestData >> dummyStr;
			for (auto itOutcomeName = attributeNameArray[3].begin(); itOutcomeName != attributeNameArray[3].end(); itOutcomeName++)
				if (dummyStr == itOutcomeName->second)
					inputFeature = itOutcomeName->first;
			robotSpeed.push_back(inputFeature);

			/* Robot Pose */
			ftestData >> dummyStr;
			for (auto itOutcomeName = attributeNameArray[4].begin(); itOutcomeName != attributeNameArray[4].end(); itOutcomeName++)
				if (dummyStr == itOutcomeName->second)
					inputFeature = itOutcomeName->first;
			robotPose.push_back(inputFeature);

			/* Robot Volume */
			ftestData >> dummyStr;
			for (auto itOutcomeName = attributeNameArray[5].begin(); itOutcomeName != attributeNameArray[5].end(); itOutcomeName++)
				if (dummyStr == itOutcomeName->second)
					inputFeature = itOutcomeName->first;
			robotVolume.push_back(inputFeature);

			/* Attention Level */
			ftestData >> dummyStr;
			if (j == STEPNUMBER - 1) {
				for (auto itOutcomeName = attributeNameArray[8].begin(); itOutcomeName != attributeNameArray[8].end(); itOutcomeName++)
					if (dummyStr == itOutcomeName->second)
						inputLabel = itOutcomeName->first;
			}
		}

		if (ftestData.eof() == true)
			break;

		auto itFace = faceFeature.begin(), itBody = bodyFeature.begin(), itAudio = audioFeature.begin();
		auto itRobotSpeed = robotSpeed.begin(), itRobotPose = robotPose.begin(), itRobotVolume = robotVolume.begin();
		for (int timeStep = 0; timeStep < STEPNUMBER; timeStep++) {
				// To store evidences of one time step, for setting the observation
			vector< string > evidencesOfNodes;
				// Prepare the evidence
			evidencesOfNodes.push_back(attributeNameArray[0][*(itFace++)]);
			evidencesOfNodes.push_back(attributeNameArray[1][*(itBody++)]);
			evidencesOfNodes.push_back(attributeNameArray[2][*(itAudio++)]);
			evidencesOfNodes.push_back(attributeNameArray[3][*(itRobotSpeed++)]);
			evidencesOfNodes.push_back(attributeNameArray[4][*(itRobotPose++)]);
			evidencesOfNodes.push_back(attributeNameArray[5][*(itRobotVolume++)]);
				// Set the evidence to network
			bn::setEvidenceOfBN(theNet, evidencesNodesName, evidencesOfNodes, timeStep);
		}
			// Inference!
		theNet.UpdateBeliefs();
			// double for confidence, string for outcome name
		map< double, string > outcomes;
		if (STEPNUMBER != 1)
			outcomes = bn::getBNOutcome(theNet, nameOfNodes[8] + "_" + targetStep);
		else
			outcomes = bn::getBNOutcome(theNet, nameOfNodes[8]);
		/* If the probabity of first and second candidate is too close, sample one ramdomly between two */
		map< double, string >::reverse_iterator itOutcomes = outcomes.rbegin();
		/*double firstConfidence = 0.0, secondConfidence = 0.0;
		firstConfidence = (*itOutcomes).first;
		itOutcomes++;
		secondConfidence = (*itOutcomes).first;
		itOutcomes--;
		if (fabs(firstConfidence - secondConfidence) < 0.101) {
			int randNum = rand() % 2;
			if (randNum == 1)
				itOutcomes++;
		}*/
		string inferredOutcome = (*itOutcomes).second;

		/* Prepare the result to send */
		for (auto itOutcomeName = attributeNameArray[8].begin(); itOutcomeName != attributeNameArray[8].end(); itOutcomeName++)
			if (inferredOutcome == itOutcomeName->second)
				attentionLevel = itOutcomeName->first;

			// Drop the oldest data
		for (int i = 0; i < STEPNUMBER; i++) {
			faceFeature.pop_front(); bodyFeature.pop_front(); audioFeature.pop_front();
			robotSpeed.pop_front(); robotPose.pop_front(); robotVolume.pop_front();
		}

		/** Display the inference result! **/
		cout << "> " << stepPassed++ << ", TRUE:" << inputLabel << ", TEST: " << attentionLevel << "\t";
		if (inputLabel != attentionLevel)
			cout << ":(" << endl;
		else
			cout << ":)" << endl;

		//cout << "\n\n> Attention Inferred: \t" << inferredOutcome << endl;
		//	// Print all the result of inference
		//for (map< double, string >::reverse_iterator it = outcomes.rbegin(); it != outcomes.rend(); it++)
		//	cout << right << setw(11) << "|-" << left << setw(20) << (*it).second << setw(15) << " Confidence = " << (*it).first << endl;
		//cout << endl;

		confusionMatrixHAE[attentionLevel][inputLabel] += 1;

		//Sleep(STEPTIME - 3 * DELAYTIME);
	}

	/* Write down the confusion matrix */
	fstream fcm;
	fcm.open(cmPath.c_str(), ios::out);
	if (!fcm) {
		cout << "> WARNING: Fail to open the file: " << testingDataPath << endl
			 << ">          Please press enter to exit..." << endl;
		getchar();
	}

	cout << "> Report the output: " << endl;
	for (auto itTest = confusionMatrixHAE.begin(); itTest != confusionMatrixHAE.end(); itTest++) {
		for (auto itTrue = itTest->begin(); itTrue != itTest->end(); itTrue++) {
			(itTrue == itTest->end() - 1) ? fcm << *itTrue : fcm << *itTrue << " ";
			(itTrue == itTest->end() - 1) ? cout << *itTrue : cout << *itTrue << " ";
		}
		fcm << endl;
		cout << endl;
	}

	getchar();
	
	fcm.close();
	ftestData.close();
	cout << "> Human Attention Estimator Testing End!" << endl << endl;
		// Disconnect to IPC server, Clean up socket.
	return 0;
}