
/* Standrad Included Library */
#include<iostream>
#include<fstream>
#include <conio.h>			// To use getch(), kbhit()
using namespace std;

/* Third-party Library */
	// Import Inter-Process Communication Server
#include "IPCserver\Client.hpp"

//** Problem Dependent Variable Setting **//
	// How long a timestep is (ms)
#define STEPTIME	750
	// How long a delay for messages request
#define DELAYTIME	200

/** Declration of Variables **/
	// To count whether if all the message received
static int receivedCount = 0;
	// Path to store the raw data
string rawData = "../models/DBN_Model/TrainingData/raw_dataset_11/HumanAttentionEstimator_raw_11.txt";

/** Declration of Functions **/
	// Handler for receiving HAE message
void HAE_handler();
	// Request all sensing data for HAE; param 1 delayTime for the message sending
HAEMgr requestSensingHAE(int delayTime);
	// Wait for message received; param 1 delayTime = delayTime / 10
bool buzyWaitForMgr(const int delayTime);

int main(int argc, char* argv[])
{
	/** Connect to IPC Server **/
	init_comm();
	connect_to_server();
	subscribe(HAE, TOTAL_MSG_NUM);
	publish(PERCEPTION_HAE, TOTAL_MSG_NUM);
	listen();

		// Ready? GO!
	cout << "\n\n		< Human Attention Estimator Recorder >" << endl;
	cout << "> Press ENTER to record data (Human Attention Estimator)";
	getchar();

	HAEMgr sensingData;

	fstream frawData;
	frawData.open(rawData.c_str(), ios::out);
	if (!frawData) {
		cout << "> WARNING: Fail to open the file: " << rawData << endl
			 << ">          Please press enter to exit..." << endl;
		getchar();
	}

	/* Recording main loop */
	while (true) {
			// Press 'Q' (81), 'q' (113) or esc (27) to leave
		if (kbhit()) {
			int command = getch();
			if (command == 81 || command == 113 || command == 27)
				break;
		}
		
		/* Request the perceived data */
		sensingData = requestSensingHAE(DELAYTIME);

		/* Store the raw data received */
		frawData << sensingData.face_direction << " " << sensingData.body_direction << " " << sensingData.voice_detection << endl;
		cout << sensingData.face_direction << " " << sensingData.body_direction << " " << sensingData.voice_detection << endl;

		Sleep(STEPTIME - 3 * DELAYTIME);
	}

	frawData.close();
	disconnect_to_server();
	cout << "\n> Recording End!" << endl;

	return 0;
}

void HAE_handler() {
	receivedCount += 1;
}

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