/******************************************************
NTU EECS Advanced Control Lab, Intelligent Robot Lab

2010 Kuo-Chen Huang

******************************************************/
#ifndef COMM_HPP
#define COMM_HPP

#include "Messages.hpp"


//--------------------------------------------------//
// message types
//--------------------------------------------------//
enum MessageType{ LASER_POSE, GOAL, SUBGOAL, ODOMETRY,
				  URG, HEAD_MOTOR, HAND_POSE,
				  FACIAL_EXP, STATE, PEOPLE, KEY_WORD, FACE_DETECT, FACE_RECOG, LOC_STATE, ACTION_SPEAK, RESULT_SPEAK,
				  NAVI_PAR, ACTION_NAVI, RESULT_NAVI, SERVER_VEL, INVALID_GOAL, POSE_MODE, MESSAGE_FREQ,
				  ACTION_GCALENDAR, RESULT_GCALENDAR,SCHEDULE_INFO, CHECK_INFO, QUERY_RESULT, PERCEPTION, DBN, FFORMATION, ACTION_APSTATE, RESULT_APSTATE,
				  ACTION_ARM, RESULT_ARM,								  // Messages for Arm Manipulating
				  PERCEPTION_HAE, HAE, ATTENTIONLEVEL, REQUEST_INFERENCE, // Messages for Toward Robot Attention Estimator (HAE)
				  TOTAL_MSG_NUM};

void init_comm();
void connect_to_server(const char* server_name = "localhost");
void disconnect_to_server();
void subscribe(MessageType, ...);
void publish(MessageType, ...);
void listen();

// get messages
void getLaserPose(LaserPoseMgr& mgr);
void getURGLaser(URGLaser& mgr);
void getSubgoal(SubgoalMgr& mgr);
void getGoal(GoalMgr &mgr);
void getOdometry(OdometryMgr& mgr);
void getHeadMotor(HeadMotorMgr& mgr);
void getHandPos(HandPoseMgr& mgr);
void getFacialExp(FacialExpMgr& mgr);
void getState(StateMgr& mgr);
void getPeople(PeopleMgr& mgr);
void getKeyWord(KeyWordMgr& mgr);
void getLocState(LocState& mgr);
void getActionSpeak(Action_Speak& mgr);
void getResultSpeak(Result_Speak& mgr);
//for Navi
void getNaviParMsg( NaviParMsg& mgr );
void getActionNavi(Action_Navi & mgr);
void getResultNavi(Result_Navi& mgr);
void getServerVel(ServerVelMgr & mgr);
void getPoseMode(PoseModeMgr & mgr);
void getInvalidGoal(InvalidGoalMgr &mgr);
void getMessageFreq(MessageFreqMgr &mgr);
//Navi
//for google calendar
void getAction_GCalendar(Action_GCalendarMgr &mgr);
void getResult_GCalendar(Result_GCalendarMgr &mgr);
void getScheduleInfo(ScheduleInfoMgr &mgr);
void getCheckInfo(CheckInfoMgr &mgr);
void getQueryResult(QueryResultMgr &mgr);
//google canendar
void getAction_apState(Action_apStateMgr& mgr);
void getResult_apState(Result_apStateMgr& mgr);
void getPerception(PerceptionMgr& mgr);
void getDBN(DBNMgr& mgr);
void getFformation(FformationMgr& mgr);
void getFaceDetect(FaceDetect& mgr);
void getFaceRecog(FaceRecogMgr &mgr);
	/* For arm manipulating */
void getActionArm(ActionArmMgr& mgr);
void getResultArm(ResultArmMgr& mgr);
	/* For Toward Robot Attention Estimator (HAE) */
void getPerceptionHAE(PerceptionHAEMgr &mgr);
void getHAE(HAEMgr &mgr);
void getAttentionLevel(AttentionLevelMgr &mgr);
void getRequestInference(RequestInferenceMgr &mgr);

// set messages
void setLaserPose(LaserPoseMgr& mgr);
void setURGLaser(URGLaser& mgr);
void setSubgoal(SubgoalMgr& mgr);
void setGoal(GoalMgr &mgr);
void setOdometry(OdometryMgr& mgr);
void setHeadMotor(HeadMotorMgr& mgr);
void setHandPose(HandPoseMgr& mgr);
void setFacialExp(FacialExpMgr& mgr);
void setState(StateMgr& mgr);
void setPeople(PeopleMgr& mgr);
void setKeyWord(KeyWordMgr& mgr);
void setLocState(LocState& mgr);
void setActionSpeak(Action_Speak& mgr);
void setResultSpeak(Result_Speak& mgr);
//for Navi
void setNaviParMsg( NaviParMsg& mgr );
void setActionNavi(Action_Navi & mgr);
void setResultNavi(Result_Navi& mgr);
void setServerVel( ServerVelMgr& mgr );
void setPoseMode( PoseModeMgr& mgr );
void setInvalidGoal(InvalidGoalMgr & mgr);
void setMessageFreq(MessageFreqMgr &mgr);
//Navi
//for google calendar
void setAction_GCalendar(Action_GCalendarMgr &mgr);
void setResult_GCalendar(Result_GCalendarMgr &mgr);
void setScheduleInfo(ScheduleInfoMgr &mgr);
void setCheckInfo(CheckInfoMgr &mgr);
void setQueryResult(QueryResultMgr &mgr);
//google canendar
void setAction_apState(Action_apStateMgr& mgr);
void setResult_apState(Result_apStateMgr& mgr);
void setPerception(PerceptionMgr& mgr);
void setDBN(DBNMgr& mgr);
void setFformation(FformationMgr& mgr);
void setFaceDetect(FaceDetect& mgr);
void setFaceRecog(FaceRecogMgr &mgr);
	/* For arm manipulating */
void setActionArm(ActionArmMgr& mgr); 
void setResultArm(ResultArmMgr& mgr); 
	/* For Toward Robot Attention Estimator (HAE) */
void setPerceptionHAE(PerceptionHAEMgr &mgr);
void setHAE(HAEMgr &mgr);
void setAttentionLevel(AttentionLevelMgr &mgr);
void setRequestInference(RequestInferenceMgr &mgr);

// send messages
int sendLaserPose(LaserPoseMgr& mgr);
int sendURGLaser(URGLaser& mgr);
int sendSubgoal(SubgoalMgr& mgr);
int sendGoal(GoalMgr &mgr);
int sendOdometry(OdometryMgr& mgr);
int sendHeadMotor(HeadMotorMgr& mgr);
int sendHandPose(HandPoseMgr& mgr);
int sendFacialExp(FacialExpMgr& mgr);
int sendState(StateMgr& mgr);
int sendPeople(PeopleMgr& mgr);
int sendKeyWord(KeyWordMgr& mgr);
int sendLocState(LocState& mgr);
int sendActionSpeak(Action_Speak& mgr);
int sendResultSpeak(Result_Speak& mgr);
//for Navi
int sendNaviParMsg( NaviParMsg& mgr );
int sendResultNavi(Result_Navi& mgr);
int sendActionNavi(Action_Navi & mgr);
int sendServerVel(ServerVelMgr & mgr);
int sendPoseMode(PoseModeMgr & mgr);
int sendInvalidGoal(InvalidGoalMgr & mgr);
int sendAction_apState(Action_apStateMgr& mgr);
int sendResult_apState(Result_apStateMgr& mgr);
int sendMessageFreq(MessageFreqMgr &mgr);
//for Navi
//for google calendar
int sendAction_GCalendar(Action_GCalendarMgr &mgr);
int sendResult_GCalendar(Result_GCalendarMgr &mgr);
int sendScheduleInfo(ScheduleInfoMgr &mgr);
int sendCheckInfo(CheckInfoMgr &mgr);
int sendQueryResult(QueryResultMgr &mgr);
//google canendar
int sendPerception(PerceptionMgr& mgr);
int sendDBN(DBNMgr& mgr);
int sendFformation(FformationMgr& mgr);
int sendFaceDetect(FaceDetect& mgr);
int snedFaceRecog(FaceRecogMgr &mgr);
	/* For arm manipulating */
int sendActionArm(ActionArmMgr& mgr); 
int sendResultArm(ResultArmMgr& mgr); 
	/* For Toward Robot Attention Estimator (HAE) */
int sendPerceptionHAE(PerceptionHAEMgr &mgr);
int sendHAE(HAEMgr &mgr);
int sendAttentionLevel(AttentionLevelMgr &mgr);
int sendRequestInference(RequestInferenceMgr &mgr);

#endif
