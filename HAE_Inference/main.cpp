
/* Standrad Included Library */
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <list>
#include <map>
#include <conio.h>			// To use getch(), kbhit()
using namespace std;

/* Third-party Library */
	// Import Inter-Process Communication Server, Kuo-Chen version
#include "IPCserver\Client.hpp"
	// To use bayesain network and it's inference algorithm
#include "SmileInference\SmileInference.h"

//** Problem Dependent Variable Setting **//
	// The timestep of the dynamic Bayesian network
#define STEPNUMBER	2
	// How long a timestep is (ms)
#define STEPTIME	750
	// How long a delay for messages request
#define DELAYTIME	200

#define NDEBUG

/** Declration of Variables **/
	// Path to unrolled DBN model
string dynamicBN = "../models/DBN_Model/res_HumanAttentionEstimator_Unrolled.xdsl";
	// To count whether if all the message received
static int receivedCount = 0;
	// Buffer result for Human Attention Level
static int attentionLevel = 0;

HAEMgr sensingData;

/** Declration of Functions **/
	// Request all sensing data for HAE; param 1 delayTime for the message sending
HAEMgr requestSensingHAE(int delayTime);
	// Handler for receiving HAE message
void HAE_handler();
	// Handler for receiving RequestInference message
void RequestInference_handler();
	// Wait for message received; param 1 delayTime = delayTime / 10
bool buzyWaitForMgr(const int delayTime);

//=============================================================================
int main(int argc, char* argv[]) {
	/** Connect to IPC Server **/
	init_comm();
	connect_to_server();
	//subscribe(HAE, PERCEPTION_HAE, TOTAL_MSG_NUM);
	subscribe(HAE, REQUEST_INFERENCE, TOTAL_MSG_NUM);
	publish(PERCEPTION_HAE, ATTENTIONLEVEL, TOTAL_MSG_NUM);
	listen();

	cout << "\n\t< Human Attention Estimator Inference >" << endl;
	cout << "> Press ENTER to inference";
	getchar();

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
		// String for nodes manipulate
	stringstream ss;
	string targetStep;
	ss << (STEPNUMBER - 1); ss >> targetStep;

	/** Inference main loop **/
	int stepPassed = 0;
	while (true) {
		/* Press 'Q' (81), 'q' (113) or esc (27) to leave */
		if (kbhit()) {
			int command = getch();
			if (command == 81 || command == 113 || command == 27)
				break;
		}

		/* Request the perceived data */
		sensingData = requestSensingHAE(DELAYTIME);

		/* Setting the data received */
		faceFeature.push_back(static_cast< int >(sensingData.face_direction));
		bodyFeature.push_back(static_cast< int >(sensingData.body_direction));
		voiceFeature.push_back(static_cast< int >(sensingData.voice_detection));
		cout << "> F: " << sensingData.face_direction << ", B: " << sensingData.body_direction << ", V: " << sensingData.voice_detection << endl;

		/* Ready to inference the result */
			// If too few data
		if (stepPassed++ < STEPNUMBER - 1) {
			Sleep(STEPTIME - 3 * DELAYTIME);
			continue;
		}

		auto itFace = faceFeature.begin(), itBody = bodyFeature.begin(), itVoice = voiceFeature.begin();
		for (int timeStep = 0; timeStep < STEPNUMBER; timeStep++) {
				// To store evidences of one time step, for setting the observation
			vector< string > evidencesOfNodes;
				// Prepare the evidence
			evidencesOfNodes.push_back(attributeNameArray[0][*itFace]);
			evidencesOfNodes.push_back(attributeNameArray[1][*itBody]);
			evidencesOfNodes.push_back(attributeNameArray[2][*itVoice]);
				// Set the evidence to network
			bn::setEvidenceOfBN(theNet, evidencesNodesName, evidencesOfNodes, timeStep);
		}
			// Inference!
		theNet.UpdateBeliefs();
			// double for confidence, string for outcome name
		map< double, string > outcomes = bn::getBNOutcome(theNet, nameOfNodes[3] + "_" + targetStep);

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
		faceFeature.pop_front(); bodyFeature.pop_front(); voiceFeature.pop_front();

		/** Display the inference result! **/
		cout << "\n\n> Attention Inferred: \t" << inferredOutcome << endl;
			// Print all the result of inference
		for (map< double, string >::reverse_iterator it = outcomes.rbegin(); it != outcomes.rend(); it++)
			cout << right << setw(11) << "|-" << left << setw(20) << (*it).second << setw(15) << " Confidence = " << (*it).first << endl;
		cout << endl;

		Sleep(STEPTIME - 3 * DELAYTIME);
	}
	cout << "> Human Attention Estimator End!" << endl << endl;
		// Disconnect to IPC server, Clean up socket.
	disconnect_to_server();
	return 0;
}

//=============================================================================
	// Request all sensing data for HAE; param 1 delayTime for the message sending
HAEMgr requestSensingHAE(int delayTime) {
	HAEMgr receivedData, tempData;
	getHAE(receivedData);

	PerceptionHAEMgr requestData;
	/* Audio */
	requestData.sensing = voiceDetection;
	sendPerceptionHAE(requestData);
	Sleep(sizeof(requestData) + DELAYTIME);

	if (buzyWaitForMgr(20) == false)
		cout << "> WARNING: Receive Data Time Out, Audio" << endl;
	
	getHAE(tempData);
	receivedData.voice_detection = tempData.voice_detection;

	/* Face */
	requestData.sensing = faceDirectionDiscrete;
	sendPerceptionHAE(requestData);
	Sleep(sizeof(requestData) + DELAYTIME);

	if (buzyWaitForMgr(20) == false)
		cout << "> WARNING: Receive Data Time Out, Face" << endl;

	
	getHAE(tempData);
	receivedData.face_direction = tempData.face_direction;

	/* Body */
	requestData.sensing = bodyDirectionDiscrete;
	sendPerceptionHAE(requestData);
	Sleep(sizeof(requestData) + DELAYTIME);

	if (buzyWaitForMgr(20) == false)
		cout << "> WARNING: Receive Data Time Out, Body" << endl;
	
	getHAE(tempData);
	receivedData.body_direction = tempData.body_direction;

	return receivedData;
}

bool buzyWaitForMgr(const int delayTime) {
	for (int i = 0; i < 10 && receivedCount < 1; i++) {
		Sleep(20);
		if (i == 9) {
			receivedCount = 0;
			return false;
		}
	}
	receivedCount = 0;
	return true;
}

	// Handler for receiving HAE message
void HAE_handler() {
	receivedCount += 1;
}

	// Handler for receiving RequestInference message
void RequestInference_handler() {
	AttentionLevelMgr percetData;
	getAttentionLevel(percetData);

	percetData.attentionLevel = static_cast< AttentionLevel_HAE_type >(attentionLevel);
		// Send Attention Level
	sendAttentionLevel(percetData);
	Sleep(sizeof(percetData));
	printf("\n> Send Success! (AttentionLevel: %d)\n", attentionLevel);

	return;
}
