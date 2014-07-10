
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
	// LCM core
#include "lcm\lcm-cpp.hpp"
	// LCM shared consts
#include "lcm\LcmComm.hpp"
	// LCM message data types
#include "lcm\BodyDirectionLcm.hpp"
#include "lcm\FaceDetectionLcm.hpp"
#include "lcm\VoiceDetectorLcm.hpp"
	// Handlers for LCM receiving
#include "lcm\LcmHandlers.hpp"

//** Problem Dependent Variable Setting **//
	// The timestep of the dynamic Bayesian network
#define STEPNUMBER	3
	// How long a timestep is (ms)
#define STEPTIME	600
	// How long a delay for messages request
#define DELAYTIME	60
/** BAD example **/
#define CONTEXT		0 //0, neutral; 1, concentrating; 2, sleepy; 3, social

//#define NDEBUG

/** Declration of Variables **/
	// Path to unrolled DBN model
string dynamicBN = "../models/DBN_Model/res_SocialAttentionInferenceModel_Unrolled.xdsl";
	// To count whether if all the message received
static int receivedCount = 0;
	// Buffer result for Human Attention Level
static int attentionLevel = 0;
	// LCM object
static lcm::LCM lcmObject(LCM_CTOR_PARAMS);

/** Declration of Functions **/
	// Request all sensing data for SAM; param 1 delayTime for the message sending
void requestSensingSAM(int delayTime, HAEMgr& dataHAE, RobotParameterMgr& robotPara, int& robotPosition);
	// Request the environment context information for SAM; param 1 delayTime for the message sending
void requestEnvirContextSAM(int delayTime, int& audioNoiseLevel, int& attenContextLevel);
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
	subscribe(HAE, REQUEST_INFERENCE, PEOPLE, ROBOTPARAMETER, TOTAL_MSG_NUM);
	publish(PERCEPTION_HAE, ATTENTIONLEVEL, PERCEPTION, TOTAL_MSG_NUM);
	listen();

	/* Test LCM */
	if (!lcmObject.good())
	{
		cout << "> ERROR: Cannot initialize LCM" << endl;
		return 1;
	}

	cout << "\n\t< Social Attention Model Inference >" << endl;
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
	list< int > faceFeature;
	list< int > bodyFeature;
	list< int > audioFeature;
	list< int > robotSpeed;
	list< int > robotPose;
	list< int > robotVolume;
		// String for nodes manipulate
	stringstream ss;
	string targetStep;
	ss << (STEPNUMBER - 1); ss >> targetStep;

	/** Inference Start! **/
	int stepPassed = 0;

	/* Setting the environment context */
	cout << "> Collecting the environment context..." << endl;
		// Requesting the environment context
	int envirContext_audioLevel = 0, envirContext_attenContext = 0;
	requestEnvirContextSAM(DELAYTIME, envirContext_audioLevel, envirContext_attenContext);
		// Setting to the net
	vector< string > envirContextNodesName, envirContextOfNodes;
	envirContextNodesName.push_back("audioNoise"); envirContextNodesName.push_back("attentionContext");
	envirContextOfNodes.push_back(attributeNameArray[6][envirContext_audioLevel]);
	envirContextOfNodes.push_back(attributeNameArray[7][envirContext_attenContext]);
	bn::setEvidenceOfBN(theNet, envirContextNodesName, envirContextOfNodes, 0);

	cout << "> EN: " << envirContext_audioLevel << ", AC: " << envirContext_attenContext << endl;

	/* Main loop */
	while (true) {
		/* Press 'Q' (81), 'q' (113) or esc (27) to leave */
		if (kbhit()) {
			int command = getch();
			if (command == 81 || command == 113 || command == 27)
				break;
		}

		/* Request the perceived data */
		HAEMgr				sensingData;
		RobotParameterMgr	parameterData;
		int					robotPositionData;
		requestSensingSAM(DELAYTIME, sensingData, parameterData, robotPositionData);

		/* Setting the data received */
		faceFeature.push_back(static_cast< int >(sensingData.face_direction));
		bodyFeature.push_back(static_cast< int >(sensingData.body_direction));
		audioFeature.push_back(static_cast< int >(sensingData.voice_detection));
		robotSpeed.push_back(static_cast< int >(parameterData.speed));
		robotPose.push_back(static_cast< int >(robotPositionData));
		robotVolume.push_back(static_cast< int >(parameterData.volume));
		cout << "> F:  " << sensingData.face_direction << ", B:  " << sensingData.body_direction << ", V:  " << sensingData.voice_detection << endl;
		cout << "> RS: " << parameterData.speed << ", RP: " << robotPositionData << ", RV: " << parameterData.volume << endl;
		
		/* Ready to inference the result */
			// If too few data to run the inference, continue collect data
		if (stepPassed++ < STEPNUMBER - 1) {
			Sleep(STEPTIME - 3 * DELAYTIME);
			continue;
		}

		auto itFace = faceFeature.begin(), itBody = bodyFeature.begin(), itAudio = audioFeature.begin();
		auto itRobotSpeed = robotSpeed.begin(), itRobotPose = robotPose.begin(), itRobotVolume = robotVolume.begin();
		for (int timeStep = 0; timeStep < STEPNUMBER; timeStep++) {
				// To store evidences of one time step, for setting the observation
			vector< string > evidencesOfNodes;
				// Prepare the evidence
			evidencesOfNodes.push_back(attributeNameArray[0][*itFace]);
			evidencesOfNodes.push_back(attributeNameArray[1][*itBody]);
			evidencesOfNodes.push_back(attributeNameArray[2][*itAudio]);
			evidencesOfNodes.push_back(attributeNameArray[3][*itRobotSpeed]);
			evidencesOfNodes.push_back(attributeNameArray[4][*itRobotPose]);
			evidencesOfNodes.push_back(attributeNameArray[5][*itRobotVolume]);
				// Set the evidence to network
			bn::setEvidenceOfBN(theNet, evidencesNodesName, evidencesOfNodes, timeStep);
		}
			// Inference!
		theNet.UpdateBeliefs();
			// double for confidence, string for outcome name
		map< double, string > outcomes = bn::getBNOutcome(theNet, nameOfNodes[8] + "_" + targetStep);

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
		faceFeature.pop_front(); bodyFeature.pop_front(); audioFeature.pop_front();
		robotSpeed.pop_front(); robotPose.pop_front(); robotVolume.pop_front();

		/** Display the inference result! **/
		cout << "\n\n> Attention Inferred: \t" << inferredOutcome << endl;
			// Print all the result of inference
		for (map< double, string >::reverse_iterator it = outcomes.rbegin(); it != outcomes.rend(); it++)
			cout << right << setw(11) << "|-" << left << setw(20) << (*it).second << setw(15) << " Confidence = " << (*it).first << endl;
		cout << endl;

		Sleep(STEPTIME - 6 * DELAYTIME);
	}
	cout << "> Social Attention Inference End!" << endl << endl;
		// Disconnect to IPC server, Clean up socket.
	disconnect_to_server();
	return 0;
}

//=============================================================================
bool buzyWaitForMgr(const int delayTime) {
	for (int i = 0; i < 10 && receivedCount < 1; i++) {
		Sleep(delayTime);
		if (i == 9) {
			receivedCount = 0;
			return false;
		}
	}
	receivedCount = 0;
	return true;
}

	// Request all sensing data for HAE; param 1 delayTime for the message sending
void requestSensingSAM(int delayTime, HAEMgr& dataHAE, RobotParameterMgr& robotPara, int& robotPosition) {
	/* Receive data through LCM */
		// Handler object, received data is stored inside
	HAEHandlerLcm handler;
		// Pointer for subscription, used for unsubscribing channel
	lcm::Subscription* sub_ptr;

		// Subscribe BODY_DIRECTION channel, receive data, then unsubscribe
	sub_ptr = lcmObject.subscribe(BODY_DIRECTION, &HAEHandlerLcm::handleBody, &handler);
	lcmObject.handle();
	lcmObject.unsubscribe(sub_ptr);

		// Subscribe FACE_DETECTION channel, receive data, then unsubscribe
	sub_ptr = lcmObject.subscribe(FACE_DETECTION, &HAEHandlerLcm::handleFace, &handler);
	lcmObject.handle();
	lcmObject.unsubscribe(sub_ptr);

		// Subscribe VOICE_DETECTION channel, receive data, then unsubscribe
	sub_ptr = lcmObject.subscribe(VOICE_DETECTION, &HAEHandlerLcm::handleVoice, &handler);
	lcmObject.handle();
	lcmObject.unsubscribe(sub_ptr);

		// Copy data to dataHAE
	dataHAE.body_direction =	static_cast<Body_Direction_HAE_type>	(handler.body_direction);
	dataHAE.pu =				static_cast<PU_type>					(handler.pu);
	dataHAE.face_direction =	static_cast<Face_Direction_HAE_type>	(handler.face_direction);
	dataHAE.voice_detection =	static_cast<AudioVolume_type>			(handler.voice_detection);
	dataHAE.body_direction_cont = handler.body_direction_cont;

	/** Requesting data about parameters of the robot **/
	RobotParameterMgr receivedDataRP, tempDataRP;
	getRobotParameter(receivedDataRP);

	PerceptionMgr requestDataRP;
	/* Speech Volume and Motion Speed */
	requestDataRP.sensing = robotParameter;
	sendPerception(requestDataRP);
	Sleep(sizeof(requestDataRP) + DELAYTIME);

	if (buzyWaitForMgr(20) == false)
		cout << "> WARNING: Receive Data Time Out, Speech Volume and Motion Speed" << endl;

	getRobotParameter(tempDataRP);
	receivedDataRP.speed = tempDataRP.speed;
	receivedDataRP.volume = tempDataRP.volume;

	robotPara = receivedDataRP;

	/** Requesting the distance between the robot and the target **/
	PerceptionMgr legRequest;
	legRequest.sensing = legDetection;
	sendPerception(legRequest);
	Sleep(sizeof(legDetection));

	if (buzyWaitForMgr(40) == false)
		cout << "> WARNING: Receive Data Time Out, Laser" << endl;
	PeopleMgr targetPos;
	getPeople(targetPos);

	/* Find the person who is nearest to the robot */
	double possibleCandidateX = 0.0, possibleCandidateY = 0.0;
	if(targetPos.count > 0) {
		/* Find the person who is nearest to the robot */
		float minX = 0.0, minY = 0.0, squaredDistance = 0.0, minSquaredDistance = 999999.0;
		for (int i = 0; i < targetPos.count; i++) {
			squaredDistance = pow(targetPos.x[i],2) + pow(targetPos.y[i],2);
			if (squaredDistance <= minSquaredDistance) {
				minSquaredDistance = squaredDistance;
				minX = targetPos.x[i];
				minY = targetPos.y[i];
			}
		}
		possibleCandidateX = minX / 100.0;
		possibleCandidateY = minY / 100.0;
	}

	double distance = sqrt(pow(possibleCandidateX, 2) + pow(possibleCandidateY, 2));
	if (distance >= 3.0)
		robotPosition = 0;
	else if (distance >= 2.1)
		robotPosition = 1;
	else if (distance >= 1.2)
		robotPosition = 2;
	else
		robotPosition = 0;

	return;
}

	// Request the environment context information for SAM; param 1 delayTime for the message sending
void requestEnvirContextSAM(int delayTime, int& audioNoiseLevel, int& attenContextLevel) {
	/* Initialization LCM object */
		// Handler object, received data is stored inside
	HAEHandlerLcm handler;
		// Pointer for subscription, used for unsubscribing channel
	lcm::Subscription* sub_ptr;

		// Subscribe VOICE_DETECTION channel, receive data, then unsubscribe
	sub_ptr = lcmObject.subscribe(VOICE_DETECTION, &HAEHandlerLcm::handleVoice, &handler);

	/* Request 5 times audio input to calibrate the environmental noise */
	vector< int > audioCount(4);
	for (int i = 0; i < 5; i++) {
		lcmObject.handle();
		audioCount[static_cast< int >(handler.voice_detection)]++;

		cout << "> Audio Detected: " << handler.voice_detection << endl;

		Sleep(1000);
	}
	lcmObject.unsubscribe(sub_ptr);

		// Found the maximum one
	audioNoiseLevel = 0;
	int countMax = 0;
	for (unsigned int i(0); i < audioCount.size(); i++) {
		if (countMax <= audioCount[i]) {
			countMax = audioCount[i];
			audioNoiseLevel = i;
		}
	}

	/* Request human-context message */
	/*requestData.sensing = attentionContext;
	sendPerceptionHAE(requestData);

	if (buzyWaitForMgr(20) == false)
		cout << "> WARNING: Receive Data Time Out, Attention-Context" << endl;

	getHAE(tempDataHAE);*/
	/** TODO **/
	cout << "> Human Task Context Detected: " << CONTEXT << endl;
	attenContextLevel = CONTEXT;

	return;
}

	// Handler for receiving HAE message
void HAE_handler() {
	receivedCount += 1;
}

	// Handler for receiving robot parameter
void RobotParameter_handler() {
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