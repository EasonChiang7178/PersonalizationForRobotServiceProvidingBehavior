
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
#define STEPNUMBER	2
	// How long a timestep is (ms)
#define STEPTIME	750
	// How long a delay for messages request
#define DELAYTIME	200

/** Declration of Variables **/
	// Path to unrolled DBN model
string dynamicBN = "../models/DBN_Model/res_HumanAttentionEstimator_Unrolled.xdsl";
	// To count whether if all the message received
static int receivedCount = 0;
	// Buffer result for Human Attention Level
static int attentionLevel = 0;
	// Path to the test data
string testingDataPath = "../models/DBN_Model/TrainingData/HumanAttentionEstimator_Learning.txt";
	// Path to the output result
string cmPath = "../models/DBN_Model/TrainingData/HumanAttentionEstimator_Testing_cm.txt";

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
	nameOfNodes.push_back("faceDirection");		nameOfNodes.push_back("bodyDirection");
	nameOfNodes.push_back("voiceDetection");	nameOfNodes.push_back("attentionLevel");
		// The nodes in the network
	vector< string > evidencesNodesName;
	evidencesNodesName.push_back("faceDirection");	evidencesNodesName.push_back("bodyDirection");
	evidencesNodesName.push_back("voiceDetection");
		// Extract each nodes' attributes name in the network
	vector< map< int, string> > attributeNameArray;
	for (int i = 0; i < static_cast< int>(nameOfNodes.size()); i++)
		attributeNameArray.push_back(bn::getBNOutcomeNames(theNet, nameOfNodes[i]));
		// To store the history observation information
	list< int > faceFeature;
	list< int > bodyFeature;
	list< int > voiceFeature;
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
	for (int label = 0; label < 4; label++)
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
		for (int i = 0; i < STEPNUMBER; i++) {
			ftestData >> dummyStr;
			for (auto itOutcomeName = attributeNameArray[0].begin(); itOutcomeName != attributeNameArray[0].end(); itOutcomeName++)
				if (dummyStr == itOutcomeName->second)
					inputFeature = itOutcomeName->first;
			sensingData.face_direction = static_cast< Face_Direction_HAE_type>(inputFeature);
			faceFeature.push_back(static_cast< int >(sensingData.face_direction));
		}

		for (int i = 0; i < STEPNUMBER; i++) {
			ftestData >> dummyStr;
			for (auto itOutcomeName = attributeNameArray[1].begin(); itOutcomeName != attributeNameArray[1].end(); itOutcomeName++)
				if (dummyStr == itOutcomeName->second)
					inputFeature = itOutcomeName->first;
			sensingData.body_direction = static_cast< Body_Direction_HAE_type>(inputFeature);
			bodyFeature.push_back(static_cast< int >(sensingData.body_direction));
		}

		for (int i = 0; i < STEPNUMBER; i++) {
			ftestData >> dummyStr;
			for (auto itOutcomeName = attributeNameArray[2].begin(); itOutcomeName != attributeNameArray[2].end(); itOutcomeName++)
				if (dummyStr == itOutcomeName->second)
					inputFeature = itOutcomeName->first;
			sensingData.voice_detection = inputFeature;
			voiceFeature.push_back(static_cast< int >(sensingData.voice_detection));
		}

		for (int i = 0; i < STEPNUMBER; i++) {
			ftestData >> dummyStr;
			if (i == STEPNUMBER - 1) {
				for (auto itOutcomeName = attributeNameArray[3].begin(); itOutcomeName != attributeNameArray[3].end(); itOutcomeName++)
					if (dummyStr == itOutcomeName->second)
						inputLabel = itOutcomeName->first;
			}
		}

		if (ftestData.eof() == true)
			break;
		
		//cout << "> F: " << sensingData.face_direction << ", B: " << sensingData.body_direction << ", V: " << sensingData.voice_detection
		//	 << "  Label: " << inputLabel << endl;

		auto itFace = faceFeature.begin(), itBody = bodyFeature.begin(), itVoice = voiceFeature.begin();
		for (int timeStep = 0; timeStep < STEPNUMBER; timeStep++) {
				// To store evidences of one time step, for setting the observation
			vector< string > evidencesOfNodes;
				// Prepare the evidence
			evidencesOfNodes.push_back(attributeNameArray[0][*itFace]);
			evidencesOfNodes.push_back(attributeNameArray[0][*itBody]);
			evidencesOfNodes.push_back(attributeNameArray[0][*itVoice]);
				// Set the evidence to network
			bn::setEvidenceOfBN(theNet, evidencesNodesName, evidencesOfNodes, timeStep);
		}
			// Inference!
		theNet.UpdateBeliefs();
			// double for confidence, string for outcome name
		map< double, string > outcomes;
		if (STEPNUMBER != 1)
			outcomes = bn::getBNOutcome(theNet, nameOfNodes[3] + "_" + targetStep);
		else
			outcomes = bn::getBNOutcome(theNet, nameOfNodes[3]);
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
		for (auto itOutcomeName = attributeNameArray[3].begin(); itOutcomeName != attributeNameArray[3].end(); itOutcomeName++)
			if (inferredOutcome == itOutcomeName->second)
				attentionLevel = itOutcomeName->first;

			// Drop the oldest data
		for (int i = 0; i < STEPNUMBER; i++) {
			faceFeature.pop_front(); bodyFeature.pop_front(); voiceFeature.pop_front();
		}

		/** Display the inference result! **/
		cout << "\n\n> Attention Inferred: \t" << inferredOutcome << endl;
			// Print all the result of inference
		for (map< double, string >::reverse_iterator it = outcomes.rbegin(); it != outcomes.rend(); it++)
			cout << right << setw(11) << "|-" << left << setw(20) << (*it).second << setw(15) << " Confidence = " << (*it).first << endl;
		cout << endl;

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