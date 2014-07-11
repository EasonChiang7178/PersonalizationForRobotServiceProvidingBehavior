
/* Standrad Included Library */
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <conio.h>			// To use getch(), kbhit()
using namespace std;

/* Third-party Library */
	// Import Inter-Process Communication Server
#include "IPCserver\Client.hpp"
	// LCM core
#include "lcm\lcm-cpp.hpp"
	// LCM shared consts
#include "lcm\LcmComm.hpp"
	// Handlers for LCM receiving
#include "lcm\LcmHandlers.hpp"

//** Problem Dependent Variable Setting **//
	// How long a timestep is (ms)
#define STEPTIME	250
	// How long a delay for messages request
#define DELAYTIME	20
/** BAD example **/
#define CONTEXT		0 //0, neutral; 1, concentrating; 2, sleepy; 3, social

/** Declration of Variables **/
	// To count whether if all the message received
static int receivedCount = 0;
	// Path to store the raw data
string rawData = "../models/DBN_Model/TrainingData/raw_dataset/SocialAttentionModel_raw.txt";
	// Lcm object
static lcm::LCM lcmObject(LCM_CTOR_PARAMS);

/** Declration of Functions **/
	// Request all sensing data for SAM; param 1 delayTime for the message sending
void requestSensingSAM(int delayTime, HAEMgr& dataHAE, RobotParameterMgr& robotPara, int& robotPosition);
	// Request the environment context information for SAM; param 1 delayTime for the message sending
void requestEnvirContextSAM(int delayTime, int& audioNoiseLevel, int& attenContextLevel);
	// Wait for message received; param 1 delayTime = delayTime / 10
bool buzyWaitForMgr(const int delayTime);
	// Sorting the people we found
map< float, int > sortingPeople(const lcmLegDetect& people);

int main(int argc, char* argv[])
{
	/** Connect to IPC Server **/
	init_comm();
	connect_to_server();
	subscribe(PERCEPTION_HAE, ROBOTPARAMETER, TOTAL_MSG_NUM);
	publish(PERCEPTION, TOTAL_MSG_NUM);
	listen();

	/* Test LCM */
	if (!lcmObject.good())
	{
		cout << "> ERROR: Cannot initialize lcm" << endl;
		return 1;
	}

		// Ready? GO!
	cout << "\n\n		< Social Attention Model Recorder >" << endl;
	cout << "> Press ENTER to record data...";
	getchar();

	fstream frawData;
	frawData.open(rawData.c_str(), ios::out);
	if (!frawData) {
		cout << "> WARNING: Fail to open the file: " << rawData << endl
			 << ">          Please press enter to exit..." << endl;
		getchar();
		return 0;
	}

	int envirContext_audioLevel = 0, envirContext_attenContext = 0;
	requestEnvirContextSAM(DELAYTIME, envirContext_audioLevel, envirContext_attenContext);
	cout << "> EN: " << envirContext_audioLevel << ", AC: " << envirContext_attenContext << endl;

	/* Recording main loop */
	while (true) {
			// Press 'Q' (81), 'q' (113) or esc (27) to leave
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

		/* Store the raw data received */
		frawData << envirContext_audioLevel << " " << envirContext_attenContext << " "
				 << sensingData.face_direction << " " << sensingData.body_direction << " " << sensingData.voice_detection << " "
				 << parameterData.speed << " " << robotPositionData << " " << parameterData.volume << endl;
		cout << "> F:  " << sensingData.face_direction << ", B:  " << sensingData.body_direction << ", V:  " << sensingData.voice_detection << endl;
		cout << "> RS: " << parameterData.speed << ", RP: " << robotPositionData << ", RV: " << parameterData.volume << endl;
		
		/* To saving the current image */
		PerceptionHAEMgr requestData;
		requestData.sensing = faceDirectionDiscrete;
		sendPerceptionHAE(requestData);
		Sleep(sizeof(requestData) + DELAYTIME);

	if (buzyWaitForMgr(20) == false)
		cout << "> WARNING: Receive Data Time Out, Face" << endl;

		Sleep(STEPTIME);
	}

	frawData.close();
	disconnect_to_server();
	cout << "\n> Recording End!" << endl;

	return 0;
}

	// Request the environment context information for SAM; param 1 delayTime for the message sending
void requestEnvirContextSAM(int delayTime, int& audioNoiseLevel, int& attenContextLevel) {
	/* Initialization LCM object */
		// Handler object, received data is stored inside
	HAEHandlerLcm handler;
		// Pointer for subscription, used for unsubscribing channel
	lcm::Subscription* sub_ptr;

	/* Request 5 times audio input to calibrate the environmental noise */
	vector< int > audioCount(4);
	for (int i = 0; i < 5; i++) {
		// Subscribe VOICE_DETECTION channel, receive data, then unsubscribe
		sub_ptr = lcmObject.subscribe(VOICE_DETECTION, &HAEHandlerLcm::handleVoice, &handler);
		lcmObject.handle();
		lcmObject.unsubscribe(sub_ptr);
		audioCount[static_cast< int >(handler.voice_detection)]++;
		cout << "> Audio Detected: " << handler.voice_detection << endl;

		Sleep(500);
	}
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
	//requestData.sensing = attentionContext;
	//sendPerceptionHAE(requestData);

	//if (buzyWaitForMgr(20) == false)
	//	cout << "> WARNING: Receive Data Time Out, Attention-Context" << endl;

	//getHAE(tempDataHAE);
	/** TODO **/
	cout << "> Human Task Context Detected: " << CONTEXT << endl;
	attenContextLevel = CONTEXT;

	return;
}

	// Request all sensing data for HAE; param 1 delayTime for the message sending
void requestSensingSAM(int delayTime, HAEMgr& dataHAE, RobotParameterMgr& robotPara, int& robotPosition) {
	/* Receive data through LCM */
		// Handler object, received data is stored inside
	HAEHandlerLcm handler;
	lcmLegDetect handlerLeg;
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
	sub_ptr = lcmObject.subscribe(LEG_CHANNEL, &lcmLegDetect::handleMessage, &handlerLeg);
	lcmObject.handle();
	lcmObject.unsubscribe(sub_ptr);

	/* Find the person who is nearest to the robot */
	auto peopleSorted = sortingPeople(handlerLeg);
	auto itPeopleSorted = peopleSorted.begin();

	float nearestDistance = itPeopleSorted->first;
	if (nearestDistance >= 3.0)
		robotPosition = 0;
	else if (nearestDistance >= 2.1)
		robotPosition = 1;
	else if (nearestDistance >= 1.2)
		robotPosition = 2;
	else
		robotPosition = 0;

	return;
}

	// Handler for receiving robot parameter
void RobotParameter_handler() {
	receivedCount += 1;
}

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

	// Sorting the people we found
map< float, int > sortingPeople(const lcmLegDetect& people) {
	map< float, int > sortedResult;
		// For every candidate detected
	for (int i = 0; i < people.count; i++) {
		float squaredDistance = pow(people.x[i], 2) + pow(people.y[i], 2);
		sortedResult.insert(pair< float, int >(squaredDistance, i));
	}

	return sortedResult;
}