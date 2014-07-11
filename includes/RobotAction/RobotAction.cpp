
#include "RobotAction.h"
	// LCM core
#include "lcm\lcm-cpp.hpp"
	// LCM shared consts
#include "lcm\LcmComm.hpp"
using namespace lcm;

// ======================================================================
//						Variables for handler use
// ======================================================================
	// Whether TTS is speaking
bool TTSspeaking = false;
	// Whether robot head is turning
bool armHeadManipulating = false;
	// Whether robot reach the point
bool naviExecuting = false;
	// To count whether if all the message received
int receivedCount = 0;
	// To store the keyword listened
string humanSpeechInput = "";

int robotSpeed = 0;
int robotVolume = 0;

// ======================================================================
//							Initialization
// ======================================================================

RobotAction::RobotAction() : lcmObject(LCM_CTOR_PARAMS) {
	/* Connect to the IPC server */
	init_comm();
	connect_to_server();
	subcribeAndPublish();
	listen();

	/* Initialize LCM */
	if (!lcmObject.good()) {
		cout << "> ERROR: Cannot initialize LCM" << endl;
		exit(1);
	}

		// Initialize timeout
	this->setTimeout(25);
}

RobotAction::RobotAction(const string& IPAddress) : lcmObject(LCM_CTOR_PARAMS) {
	/* Connect to the IPC server */
	init_comm();
	connect_to_server(IPAddress.c_str());
	subcribeAndPublish();
	listen();

	/* Initialize LCM */
	if (!lcmObject.good()) {
		cout << "> ERROR: Cannot initialize LCM" << endl;
		exit(1);
	}

		// Initialize timeout
	this->setTimeout(25);
}

RobotAction::RobotAction(const RobotAction& copy) {
	this->setTimeout(copy.getTimeout());
}

RobotAction::~RobotAction() {
	disconnect_to_server();
}

RobotAction& RobotAction::operator=(const RobotAction& copy){
	this->setTimeout(copy.getTimeout());

	return *this;
}

void RobotAction::subcribeAndPublish() {
	subscribe(PERCEPTION, KEY_WORD, ODOMETRY, RESULT_NAVI, RESULT_ARM, RESULT_SPEAK, ATTENTIONLEVEL, TOTAL_MSG_NUM);
	publish(ROBOTPARAMETER, MESSAGE_FREQ, PERCEPTION, SUBGOAL, ACTION_NAVI, ACTION_ARM, ACTION_SPEAK, ARM_POSITION, REQUEST_INFERENCE, TOTAL_MSG_NUM);
	
}

// ======================================================================
//						 Primitive Robot Actions
// ======================================================================

const bool RobotAction::doNothing(const int& sleepTime) {
	if (sleepTime > timeout) {
		cout << "> WARNING: Sleep time is too long, sleep in timout (s)" << endl;
		Sleep(timeout);
	} else
		Sleep(sleepTime);

	return true;
}

const bool RobotAction::headShake(const int& swingRange, const int& destDegree) {
	ostringstream ostrP, ostrN, ostrD;
	ostrP << swingRange; ostrN << "-" << swingRange; ostrD << destDegree;

	bool success = true;
	time_t headShake_start = this->getCurrentTime();

	cout << "> Turn to " << ostrP.str() <<" degree..." << endl;
	if (this->turningFace(swingRange) == false) success = false;
	Sleep(200);
	cout << "> Turn to " << ostrN.str() <<" degree..." << endl;
	if (this->turningFace(-1 * swingRange) == false) success = false;
	Sleep(200);
	cout << "> Turn to " << ostrD.str() <<" degree..." << endl;
	if (this->turningFace(destDegree) == false) success = false;

	if (this->executingTime(headShake_start) > timeout) {
		success = false;
		cout << "> WARNING: Action, Head Shake, timeout" << endl;
	}
	return success;
}

const bool RobotAction::turningFace(const int& goalDegree) {
	this->getCurrentTime();
	ActionArmMgr actionmgr;
	ArmPositionMgr armHeadPara;

	/* Sending command to turn the robot head */
	actionmgr.armState = ARM_HEAD_SHAKE;
	armHeadPara.headDeg = goalDegree;

	sendArmPosition(armHeadPara);
	Sleep(sizeof(armHeadPara) + 100);

	sendActionArm(actionmgr);
	Sleep(sizeof(actionmgr));
	armHeadManipulating = true;

	/* Wait until receive ARM_FINISH from ResultArm or timeout */
	while(this->executingTime() <= timeout) {
			// Received ARM_FINISHED message
		if (armHeadManipulating == false) {
			/* Reset the state of arm */
			actionmgr.armState = ARM_STOP;
			sendActionArm(actionmgr);
			Sleep(sizeof(actionmgr));

			return true;
		}
		Sleep(50);
	}
	/* Timeout */
	cout << "> WARNING: Action, Turning Head, timeout" << endl;
	actionmgr.armState = ARM_STOP;
	sendActionArm(actionmgr);
	Sleep(sizeof(actionmgr));
	return false;
}

const int RobotAction::turnFaceToHuman() {
	/* Receive data through LCM */
	lcmLegDetect handlerLeg;
	lcm::Subscription* sub_ptr;

	/* Request current human pose */
	sub_ptr = lcmObject.subscribe(LEG_CHANNEL, &lcmLegDetect::handleMessage, &handlerLeg);
	lcmObject.handle();
	lcmObject.unsubscribe(sub_ptr);

	/* Find the person who is nearest to the robot */
	auto peopleSorted = sortingPeople(handlerLeg);
	if (peopleSorted.empty() == true) {
		cout << "> WARNING: No people found!" << endl;
		return false;
	}
	auto itPeopleSorted = peopleSorted.begin();

	/* Prepare the angle to turn the head */
	int goalDegree = -1 * static_cast< int >(atan2(handlerLeg.y[itPeopleSorted->second],
												   handlerLeg.x[itPeopleSorted->second]) / M_PI * 180.0);

	if (abs(goalDegree) > 35) {
		cout << "> WARNING: Head cannot turn to this angle, " << goalDegree << endl;
		goalDegree = 35;
	} else if (abs(goalDegree) < -35) {
		cout << "> WARNING: Head cannot turn to this angle, " << goalDegree << endl;
		goalDegree = -35;
	}

	cout << "> Turning Face To " << goalDegree << endl;
	this->turningFace(goalDegree);
	return -1 * goalDegree;
}

const bool RobotAction::armWave(const int& motionSpeed) {
	this->getCurrentTime();
	ActionArmMgr actionmgr;
	ArmPositionMgr armHeadPara;

	/* Sending command to wave the robot's arm */
	armHeadPara.move_time = motionSpeed;
	sendArmPosition(armHeadPara);
	Sleep(sizeof(armHeadPara) + 100);

	actionmgr.armState = ARM_WAVE;
	sendActionArm(actionmgr);
	Sleep(sizeof(actionmgr));
	armHeadManipulating = true;

	/* Wait until receive ARM_FINISH from ResultArm or timeout */
	while(this->executingTime() <= timeout) {
			// Received ARM_FINISHED message
		if (armHeadManipulating == false) {
			/* Reset the state of arm */
			actionmgr.armState = ARM_STOP;
			sendActionArm(actionmgr);
			Sleep(sizeof(actionmgr));

			return true;
		}
		Sleep(50);
	}
	/* Timeout */
	cout << "> WARNING: Action, Arm Wave, timeout" << endl;
	actionmgr.armState = ARM_STOP;
	sendActionArm(actionmgr);
	Sleep(sizeof(actionmgr));
	return false;
}

	// @Param: x, the heading direction of the navi starting; theta, 0 where the navi executed;
const bool RobotAction::toPoint(const double& x, const double& y, const double& theta) {
	/* Prepare subgoal message */
	SubgoalMgr goal;
	goal.x = x;
	goal.y = y;
	goal.theta = theta;

	/* Send subgoal message */
	sendSubgoal(goal);
	naviExecuting = true;
	Sleep(sizeof(goal) + 500);

	this->turningFace(0);
	this->getCurrentTime();

	Result_Navi resultNaviStatus;
	getResultNavi(resultNaviStatus);

	while (this->executingTime() <= timeout) {
		if (naviExecuting == false || resultNaviStatus.current_status == FINISHED)
			return true;
		getResultNavi(resultNaviStatus);
		Sleep(50);
	}
		// Timeout
	cout << "> WARNING: Action, ToPoint, timeout" << endl;
	naviExecuting = false;
	return false;
}

const bool RobotAction::rotation(const double& swingRange) {
	MessageFreqMgr requestOdo;
	sendMessageFreq(requestOdo);
	Sleep(sizeof(requestOdo));

	this->buzyWaitForMgr(500);
	OdometryMgr curPos;
	getOdometry(curPos);

	this->getCurrentTime();

	/* Prepare subgoal message */
	SubgoalMgr goal;
	goal.x = curPos.x;
	goal.y = curPos.y;
	goal.theta = curPos.theta + swingRange * 2.0; // Multiple 2 to speed up the rotation

	/* Send subgoal message */
	sendSubgoal(goal);
	Sleep(sizeof(goal) + 2000);
	
		// Turn to the another direction
	goal.theta = curPos.theta + swingRange * -2.0; 
	sendSubgoal(goal);
	Sleep(sizeof(goal) + 4000);

	/* Turn back */
	goal.theta = curPos.theta + swingRange * 2.0; // To speed up the turn back
	sendSubgoal(goal);
	Sleep(sizeof(goal) + 2000);

	goal.theta = curPos.theta;
	sendSubgoal(goal);
	naviExecuting = true;
	Sleep(sizeof(goal));
	
	while (this->executingTime() <= timeout) {
		if (naviExecuting == false)
			return true;
		Sleep(50);
	}
		// Timeout
	cout << "> WARNING: Action, Rotation, timeout" << endl;

	return false;
}

const bool RobotAction::forwardApproach(const int& speed, const double& distance) {
	/* Request current robot pose */
	MessageFreqMgr requestOdo;
	sendMessageFreq(requestOdo);
	Sleep(sizeof(requestOdo));

	this->buzyWaitForMgr(500);
	OdometryMgr curPos;
	getOdometry(curPos);

	/* Request current human pose */
	lcmLegDetect handlerLeg;
	lcm::Subscription* sub_ptr;

	sub_ptr = lcmObject.subscribe(LEG_CHANNEL, &lcmLegDetect::handleMessage, &handlerLeg);
	lcmObject.handle();
	lcmObject.unsubscribe(sub_ptr);

	/* Find the person who is nearest to the robot */
	auto peopleSorted = sortingPeople(handlerLeg);
	if (peopleSorted.empty() == true) {
		cout << "> WARNING: No people found!" << endl;
		return false;
	}
	auto itPeopleSorted = peopleSorted.begin();

	/* Preparing the subgoal message */
	double dX = handlerLeg.x[itPeopleSorted->second] / 100.0;
	double dY = handlerLeg.y[itPeopleSorted->second] / 100.0;

	if (sqrt(pow(dX, 2) + pow(dY, 2)) < 1.1) {
		cout << "> WARNING: Too close to human!" << endl;
		return false;
	}

	SubgoalMgr goal;
	goal.theta = atan2(dY, dX) * 180 / M_PI;
	goal.x = curPos.x + distance * cos(goal.theta / 180 * M_PI);
	goal.y = curPos.y + distance * sin(goal.theta / 180 * M_PI);

	cout << "Xr: " << curPos.x << ", Yr: " << curPos.y << endl;
	cout << "Xh: " << dX << ", Yh: " << dY << endl;
	cout << "Xg: " << goal.x << ", Yg: " << goal.y << endl;

	/* Send subgoal message */
	sendSubgoal(goal);
	naviExecuting = true;
	Sleep(sizeof(goal) + 500);

	this->turningFace(0);
	this->getCurrentTime();

	Result_Navi resultNaviStatus;
	getResultNavi(resultNaviStatus);

	while (this->executingTime() <= timeout) {
		if (naviExecuting == false || resultNaviStatus.current_status == FINISHED)
			return true;
		getResultNavi(resultNaviStatus);
		Sleep(50);
	}
		// Timeout
	cout << "> WARNING: Action, ForwardApproach, timeout" << endl;
	naviExecuting = false;
	return false;
}

const bool RobotAction::movingToAroundOfHuman(const int& speed, const float& distance, const double& angle2FrontOfHuman) {
	int headAngle = this->turnFaceToHuman();
	Sleep(1000);

	/* Request current robot pose */
	MessageFreqMgr requestOdo;
	sendMessageFreq(requestOdo);
	Sleep(sizeof(requestOdo));

	this->buzyWaitForMgr(250);
	OdometryMgr curPos;
	getOdometry(curPos);

	/* Request current human pose */
	lcmLegDetect handlerLeg;
	lcm::Subscription* sub_ptr;

	sub_ptr = lcmObject.subscribe(LEG_CHANNEL, &lcmLegDetect::handleMessage, &handlerLeg);
	lcmObject.handle();
	lcmObject.unsubscribe(sub_ptr);

	/* Find the person who is nearest to the robot */
	auto peopleSorted = sortingPeople(handlerLeg);
	if (peopleSorted.empty() == true) {
		cout << "> WARNING: No people found!" << endl;
		return false;
	}
	auto itPeopleSorted = peopleSorted.begin();

	/* Request body direction */
	HAEHandlerLcm bodyDirMgr;

	sub_ptr = lcmObject.subscribe(BODY_DIRECTION, &HAEHandlerLcm::handleBody, &bodyDirMgr);
	lcmObject.handle();
	lcmObject.unsubscribe(sub_ptr);

	HAEMgr receivedBodyDir;
	receivedBodyDir.body_direction_cont = bodyDirMgr.body_direction_cont;

	//PerceptionHAEMgr requestBody;
	//requestBody.sensing = bodyDirectionCont;
	//sendPerceptionHAE(requestBody);
	//Sleep(sizeof(requestBody));

	//this->buzyWaitForMgr(250);
	//HAEMgr receivedBodyDir;
	//getHAE(receivedBodyDir);

	/* Prepare subgoal message */
	float possibleCandidateX = handlerLeg.x[itPeopleSorted->second] / 100.0f;
	float possibleCandidateY = handlerLeg.y[itPeopleSorted->second] / 100.0f;
	double dist_robot2human = sqrt(pow(possibleCandidateX, 2) + pow(possibleCandidateY, 2));
	double theta_BD = (-1 * receivedBodyDir.body_direction_cont);
	double theta_robotHeading = atan2(possibleCandidateY, possibleCandidateX) * 180 / M_PI;
	
	/* Coordinate Transformation */
	CoordTrans coordinateTrans;
		// From human coordinate (B) to robot head coordinate (RH)
	double X_g_H = distance * cos(angle2FrontOfHuman / 180 * M_PI), Y_g_H = distance * sin(angle2FrontOfHuman / 180 * M_PI), theta_g_H = 180.0 + angle2FrontOfHuman;
	double X_g_RH = 0.0, Y_g_RH = 0.0, theta_g_RH = 0.0;
	coordinateTrans.trans2D(X_g_H, Y_g_H, theta_g_H,
							dist_robot2human * cos(theta_BD / 180 * M_PI), dist_robot2human * sin(theta_BD / 180 * M_PI), 180.0 + theta_BD - theta_robotHeading, 
							X_g_RH, Y_g_RH, theta_g_RH);
	
		// From robot head coordinate (RH) to robot coordinate (R)
	double X_g_R = 0.0, Y_g_R = 0.0, theta_g_R = 0.0;
	coordinateTrans.trans2D(X_g_RH, Y_g_RH, theta_g_RH,
							0, 0, -1 * headAngle,
							X_g_R, Y_g_R, theta_g_R);

		// From robot coordinate (R) to world coordinate (W)
	double X_g_W = 0.0, Y_g_W = 0.0, theta_g_W = 0.0;
	coordinateTrans.trans2D(X_g_R, Y_g_R, theta_g_R,
							curPos.x - 0.0, curPos.y - 0.0, -1 * curPos.theta,
							X_g_W, Y_g_W, theta_g_W);
	
	/* For visualization, used for debug */
		// Transform the centor point of human
	//double X_c_H = 0.0, Y_c_H = 0.0, Y_c_R, Y_c_W, X_c_R, X_c_W;
	//coordinateTrans.trans2D(X_c_H, Y_c_H, theta_g_H,
	//						dist_robot2human * cos(theta_BD / 180 * M_PI), dist_robot2human * sin(theta_BD / 180 * M_PI), 180.0 + theta_BD - theta_robotHeading, 
	//						X_c_R, Y_c_R, theta_g_R);
	//coordinateTrans.trans2D(X_c_R, Y_c_R, theta_g_R,
	//						curPos.x - 0.0, curPos.y - 0.0, -1 * curPos.theta,
	//						X_c_W, Y_c_W, theta_g_W);

	//int imgX_g = 0, imgY_g = 0, imgX_c = 0, imgY_c = 0;
	//coordinateTrans.plane2Img(X_g_W, Y_g_W, imgX_g, imgY_g);
	//coordinateTrans.plane2Img(X_c_W, Y_c_W, imgX_c, imgY_c);
	//coordinateTrans.insertPoint(imgX_g, imgY_g);
	//coordinateTrans.insertPoint(imgX_c, imgY_c);
	//coordinateTrans.drawPoint();

	SubgoalMgr goal;
	goal.theta = theta_g_W;
	goal.x = X_g_W;
	goal.y = Y_g_W;

	/* Print information for debug */
	cout << "BodyDirection: " << receivedBodyDir.body_direction_cont << endl;
	cout << "X_Goal: " << goal.x << ", Y_Goal: " << goal.y << ", Theta_Goal: " << goal.theta << endl;

	/* Send subgoal message */
	sendSubgoal(goal);
	naviExecuting = true;
	Sleep(sizeof(goal) + 500);

	this->turningFace(0);
	this->getCurrentTime();

	Result_Navi resultNaviStatus;
	getResultNavi(resultNaviStatus);

	while (this->executingTime() <= timeout) {
		if (naviExecuting == false || resultNaviStatus.current_status == FINISHED)
			return true;
		getResultNavi(resultNaviStatus);
		Sleep(50);
	}
		// Timeout
	cout << "> WARNING: Action, MovingToFrontOfHuman, timeout" << endl;
	naviExecuting = false;
	return false;
}

const bool RobotAction::makeSounds(const string& pathToAudioFile) {
	//if (WinExec(("\"C:\\Program Files\\Windows Media Player\\wmplayer\"" + pathToAudioFile).c_str(), SW_HIDE) > 31) {
		//cout << "> WARNING: Action, MakeSounds, error" << endl;
		//return false;
	//}
	WinExec(("\"C:\\Program Files\\Windows Media Player\\wmplayer\"" + pathToAudioFile).c_str(), SW_HIDE);
	Sleep(2000);
	return true;
}

const bool RobotAction::speaking(const string& textToSpeak, const float& voiceVolume) {
	this->getCurrentTime();

		// To check whethler TTS is speaking
	if (TTSspeaking == false) {
		Action_Speak speak;
		sprintf(speak.words, textToSpeak.c_str());
		speak.voiceVolume = voiceVolume;
		sendActionSpeak(speak);
		Sleep(sizeof(speak));

		TTSspeaking = true;
	}

	/* Wait until receive SPEAK_FINIHED from uTTS or timeout */
	while(this->executingTime() <= timeout) {
			// Speaking end
		if (TTSspeaking == false)
			return true;
	}

	cout << "> WARNING: Action, speaking: " << textToSpeak <<", timeout" << endl;
	return false;
}

const int RobotAction::setSpeechVolume(const int& volume) {
	speechVolume = volume;
	robotVolume = speechVolume;
	return 1;
}

const int RobotAction::setMotionSpeed(const int& speed) {
	motionSpeed = speed;
	robotSpeed = motionSpeed;
	return 1;
}

const int RobotAction::getMotionSpeed() const {
	robotSpeed = motionSpeed;
	return motionSpeed;
}

const int RobotAction::getSpeechVolume() const {
	robotVolume = speechVolume;
	return speechVolume;
}

// ======================================================================
//						High Level Inference Result
// ======================================================================
const bool RobotAction::sensingATR(const int& waitingTime) {
	bool success = true;
		// Query the AttentionLevel from HAE_Inference
	RequestInferenceMgr perceptData;
	perceptData.sensing = AttentionLevel;
	sendRequestInference(perceptData);
	Sleep(sizeof(perceptData) + waitingTime);

	if (buzyWaitForMgr(waitingTime) == false) {
		cout << "> WARNING: Receive Data Time Out, HAE" << endl;
		success = false;
	}

	AttentionLevelMgr resultATR;
	getAttentionLevel(resultATR);
	curAttentionLevel = static_cast< int >(resultATR.attentionLevel);

	return success;
}

const bool RobotAction::sensingFD(const int& waitingTime) {
	bool success = true;

	HAEMgr receivedData;
	PerceptionHAEMgr requestData;
	
	/* Face */
	requestData.sensing = faceDirectionDiscrete;
	sendPerceptionHAE(requestData);
	Sleep(sizeof(requestData) + waitingTime);

	if (buzyWaitForMgr(waitingTime) == false) {
		cout << "> WARNING: Receive Data Time Out, Face" << endl;
		success = false;
	}

	getHAE(receivedData);
	curFaceDirection = static_cast< int >(receivedData.face_direction);

	return success;
}

const bool RobotAction::sensingPU(const int& waitingTime) {
	bool success = true;

	HAEMgr receivedData;
	PerceptionHAEMgr requestData;
	
	/* PU */
	requestData.sensing = puMeasurement;
	sendPerceptionHAE(requestData);
	Sleep(sizeof(requestData) + waitingTime);

	if (buzyWaitForMgr(waitingTime) == false) {
		cout << "> WARNING: Receive Data Time Out, PU" << endl;
		success = false;
	}

	getHAE(receivedData);
	curPU = static_cast< int >(receivedData.pu);

	return success;
}

bool RobotAction::buzyWaitForMgr(const int& delayTime) {
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

const int RobotAction::setKeywordListened() {
	this->keywordListened = humanSpeechInput;
	humanSpeechInput = "";
	return 1;
}

	// Sorting the people we found
map< float, int > RobotAction::sortingPeople(const lcmLegDetect& people) {
	map< float, int > sortedResult;
		// For every candidate detected
	for (int i = 0; i < people.count; i++) {
		float squaredDistance = pow(people.x[i], 2) + pow(people.y[i], 2);
		sortedResult.insert(pair< float, int >(squaredDistance, i));
	}

	return sortedResult;
}

// ======================================================================
//							Message Handlers
// ======================================================================
void ResultSpeak_handler() {
	TTSspeaking = false;
}

void ResultNavi_handler() {
	naviExecuting = false;
}

void ResultArm_handler() {
	armHeadManipulating = false;
}

void Attention_Level_handler() {
	receivedCount += 1;
}

void HAE_handler() {
	receivedCount += 1;
}

void Odometry_handler() {
	receivedCount += 1;
}

void PeopleMgr_handler() {
	receivedCount += 1;
}

void KeyWord_handler(KeyWordMgr data) {
	humanSpeechInput = data.keyword;
}

void Perception_handler(PerceptionMgr data) {
	RobotParameterMgr robot;
	robot.speed = static_cast< robotMotionSpeed_type >(robotSpeed);
	robot.volume = static_cast< robotSpeechVolume_type >(robotVolume);
	sendRobotParameter(robot);
	Sleep(sizeof(robot));
}