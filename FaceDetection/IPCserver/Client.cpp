/******************************************************
NTU EECS Advanced Control Lab, Intelligent Robot Lab

2010 Kuo-Chen Huang

******************************************************/

#include "IPCserver\Client.hpp"
#include <cstdarg>
#include <map>
#include <sstream>
#include <string>
//#include "uTTS.h"

#ifdef WIN32
#include "process.h"
#else
#include <pthread.h>
//#include "msleep.h"
#endif

using namespace std;
using namespace ROBOT;

//--------------------------------------------------//
// message container ( names, formats and handlers )
//--------------------------------------------------//
MsgInfo msg_info[TOTAL_MSG_NUM];

//--------------------------------------------------//
// message handlers
//--------------------------------------------------//

void Goal_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	GoalMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(GoalMgr));
	setGoal( data );
	IPC_freeByteArray(callData);
}


void LaserPose_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	LaserPoseMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(LaserPoseMgr));
	setLaserPose( data );
	IPC_freeByteArray(callData);
}

void Subgoal_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	SubgoalMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(SubgoalMgr));
	setSubgoal( data );
	IPC_freeByteArray(callData);
}


void Odometry_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	OdometryMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(OdometryMgr));
	setOdometry( data );
	IPC_freeByteArray(callData);
}


void URGlaser_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	URGLaser data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(URGLaser));
	setURGLaser( data );
	IPC_freeByteArray(callData);
}

// Hand

void HandPose_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	HandPoseMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(HandPoseMgr));
	setHandPose( data );
	IPC_freeByteArray(callData);
}

//HeadMotor

void HeadMotor_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	HeadMotorMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(HeadMotorMgr));
	setHeadMotor( data );
	IPC_freeByteArray(callData);
}

// facial expression

void FacialExp_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	FacialExpMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(FacialExpMgr));
	setFacialExp( data );
	IPC_freeByteArray(callData);
}

//robot state

void State_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	StateMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(StateMgr));
	setState( data );
	IPC_freeByteArray(callData);
	/**/
//	taskPlanerNamespace::TaskIpc::setReceiveStateFlag(); 
}

void People_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	PeopleMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(PeopleMgr));
	setPeople( data );
	IPC_freeByteArray(callData);
}

void KeyWord_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	KeyWordMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(KeyWordMgr));
	setKeyWord( data );
	IPC_freeByteArray(callData);
}

void FaceDetect_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	FaceDetect data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(FaceDetect));
	setFaceDetect( data );
	IPC_freeByteArray(callData);
}

void FaceRecog_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	FaceRecogMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(FaceRecogMgr));
	setFaceRecog( data );
	IPC_freeByteArray(callData);
}

void LocState_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	LocState data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(LocState));
	setLocState( data );
	IPC_freeByteArray(callData);
}

void ActionSpeak_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	//Result_Speak result;
	Action_Speak data;

	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(Action_Speak));
	setActionSpeak( data );
	
	/*
	result.status = 1;  //tts is used
	sendResultSpeak(result); //告知tts使用中
	
	TTSAPI tts;
	tts.TTS_Speaker(data.words);
	
	result.status = 2; //tts工作結束
	sendResultSpeak(result); //告知tts目前可使用
	*/
	IPC_freeByteArray(callData);
}

void ResultSpeak_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	Result_Speak data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(Result_Speak));
	setResultSpeak( data );
	IPC_freeByteArray(callData);
}

//for Navi

void ActionNavi_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	Action_Navi data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(Action_Navi));
	setActionNavi( data );
	IPC_freeByteArray(callData);
}

void ResultNavi_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	Result_Navi data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(Result_Navi));
	setResultNavi( data );
	IPC_freeByteArray(callData);
}

void ServerVel_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	ServerVelMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(ServerVelMgr));
	setServerVel( data );
	IPC_freeByteArray(callData);
}

void PoseMode_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	PoseModeMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(PoseModeMgr));
	setPoseMode( data );
	IPC_freeByteArray(callData);
}

void InvalidGoal_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	InvalidGoalMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(InvalidGoalMgr));
	setInvalidGoal( data );
	IPC_freeByteArray(callData);
}

void MessageFreq_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	MessageFreqMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(MessageFreqMgr));
	setMessageFreq( data );
	IPC_freeByteArray(callData);
}

void NaviPar_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	NaviParMsg data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(NaviParMsg));
	setNaviParMsg( data );
	IPC_freeByteArray(callData);
}

//For Google Calendar

void Action_GC_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	Action_GCalendarMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(Action_GCalendarMgr));
	setAction_GCalendar( data );
	IPC_freeByteArray(callData);
}

void Result_GC_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	Result_GCalendarMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(Result_GCalendarMgr));
	setResult_GCalendar( data );
	IPC_freeByteArray(callData);
}

void ScheduleInfo_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	ScheduleInfoMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(ScheduleInfoMgr));
	setScheduleInfo( data );
	IPC_freeByteArray(callData);
}

void CheckInfo_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	CheckInfoMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(CheckInfoMgr));
	setCheckInfo( data );
	IPC_freeByteArray(callData);
}

void QueryResult_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	QueryResultMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(QueryResultMgr));
	setQueryResult( data );
	IPC_freeByteArray(callData);
}

void Perception_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	PerceptionMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(PerceptionMgr));
	setPerception( data );
	IPC_freeByteArray(callData);
}

void DBN_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	DBNMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(DBNMgr));
	setDBN( data );
	IPC_freeByteArray(callData);
}

void Fformation_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	FformationMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(FformationMgr));
	setFformation( data );
	IPC_freeByteArray(callData);
}

// TP //

void action_apState_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	Action_apStateMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(Action_apStateMgr));
	setAction_apState(data);
	IPC_freeByteArray(callData);
	/**/
//	taskPlanerNamespace::TaskIpc::setReceiveFlag(); 
}

void result_apState_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	Result_apStateMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(Result_apStateMgr));
	setResult_apState(data);
	IPC_freeByteArray(callData);
	/**/
//	taskPlanerNamespace::TaskIpc::setReceiveFlag(); 
}

	/* For arm manipulating */
void actionarm_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	ActionArmMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(ActionArmMgr));
	setActionArm( data );
	IPC_freeByteArray(callData);
}

void resultarm_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData)
{
	ResultArmMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(ResultArmMgr));
	setResultArm( data );
	IPC_freeByteArray(callData);
}
	/* For arm manipulating */

	/* For Toward Robot Attention Estimator (HAE) */
void Perception_HAE_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData) {
	PerceptionHAEMgr data;
	extern void Perception_HAE_handler();

	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(PerceptionHAEMgr));
	setPerceptionHAE(data);

	if (data.sensing == faceDirectionDiscrete) {
		printf("> Sending... faceDirectionDiscrete\n");
		Perception_HAE_handler();
	}

	IPC_freeByteArray(callData);
}

void HAE_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData) {
	HAEMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(HAEMgr));
	setHAE(data);
	IPC_freeByteArray(callData);
}

void Attention_Level_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData) {
	AttentionLevelMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(AttentionLevelMgr));
	setAttentionLevel(data);
	IPC_freeByteArray(callData);
}

void Reqeust_Inference_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData) {
	RequestInferenceMgr data;
	//extern void RequestInference_handler();
	
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(RequestInferenceMgr));
	setRequestInference(data);

	//RequestInference_handler();
	IPC_freeByteArray(callData);
}
	/* For Toward Robot Attention Estimator (HAE) */

void Robot_Parameter_message_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData) {
	RobotParameterMgr data;
	IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &data, sizeof(RobotParameterMgr));
	setRobotParameter(data);
	IPC_freeByteArray(callData);
}

//--------------------------------------------------//
// IPC client
//--------------------------------------------------//
static ClientThread listenThread;

void register_messages();
void define_message(const char *msgName, const char *formatString);

void connect_to_server(const char* server_name)
{
	ostringstream ipc_name;
#ifdef WIN32
	ipc_name << "FaceDetection_" << _getpid();
#else
	ipc_name << "Client" << getpid();
#endif

	IPC_connectModule(ipc_name.str().c_str(), server_name);
	IPC_setVerbosity(IPC_Print_Errors);
	IPC_setVerbosity(IPC_Print_Warnings);

	register_messages();
}

void disconnect_to_server()
{
	listenThread.stop();
	listenThread.join();
	
	IPC_disconnect();
}

void subscribe(MessageType first, ...)
{
	MessageType msg_type = first;
	va_list arg_list;
	va_start( arg_list, first );

	while( msg_type < TOTAL_MSG_NUM ){
		const char* name = msg_info[msg_type].msg_name.c_str();
		const char* format = msg_info[msg_type].msg_format.c_str();
		HANDLER_TYPE handler = msg_info[msg_type].msg_handler;
		printf( "subscribe %s\n", name );
		define_message( name, format );
		IPC_subscribe( name, handler, NULL);
		msg_type = (MessageType)va_arg( arg_list, int );
	}

	va_end( arg_list );
}

void publish(MessageType first, ...)
{
	MessageType msg_type = first;
	va_list arg_list;
	va_start( arg_list, first );

	while( msg_type < TOTAL_MSG_NUM ){
		const char* name = msg_info[msg_type].msg_name.c_str();
		const char* format = msg_info[msg_type].msg_format.c_str();
		printf( "publish %s\n", name );
		define_message( name, format );
		msg_type = (MessageType)va_arg( arg_list, int );
	}

	va_end( arg_list );
}

void listen()
{
	if( !listenThread.isRunning() )
		listenThread.start();
}

// register messages
void register_messages()
{
	msg_info[LASER_POSE] = MsgInfo( LASER_POSE_NAME, LASER_POSE_FORMAT, LaserPose_message_handler );
	msg_info[KEY_WORD]  = MsgInfo( KEY_WORD_NAME, KEY_WORD_FORMAT, KeyWord_message_handler );
	msg_info[SUBGOAL]  = MsgInfo( SUBGOAL_NAME, SUBGOAL_FORMAT, Subgoal_message_handler );
	msg_info[ODOMETRY]  = MsgInfo( ODOMETRY_NAME, ODOMETRY_FORMAT, Odometry_message_handler );
	msg_info[HEAD_MOTOR]  = MsgInfo( HEADMOTOR_NAME, HEADMOTOR_FORMAT, HeadMotor_message_handler );
	msg_info[HAND_POSE]  = MsgInfo( HANDPOS_NAME, HANDPOS_FORMAT, HandPose_message_handler );
	msg_info[FACIAL_EXP]  = MsgInfo(FACIAlEXP_NAME, FACIAlEXP_FORMAT, FacialExp_message_handler );
	msg_info[STATE]  = MsgInfo( STATE_NAME, STATE_FORMAT, State_message_handler );
	msg_info[PEOPLE]  = MsgInfo( PEOPLE_NAME, PEOPLE_FORMAT, People_message_handler );
	msg_info[FACE_DETECT]  = MsgInfo( FACE_DETECT_NAME, FACE_DETECT_FORMAT, FaceDetect_message_handler );
	msg_info[FACE_RECOG]  = MsgInfo( FACE_RECOG_NAME, FACE_RECOG_FORMAT, FaceRecog_message_handler );

	msg_info[LOC_STATE]  = MsgInfo( LOC_STATE_NAME, LOC_STATE_FORMAT, LocState_message_handler );
	msg_info[ACTION_SPEAK]  = MsgInfo( ACTION_SPEAK_NAME, ACTION_SPEAK_FORMAT, ActionSpeak_message_handler );
	msg_info[RESULT_SPEAK]  = MsgInfo( RESULT_SPEAK_NAME, RESULT_SPEAK_FORMAT, ResultSpeak_message_handler );
	msg_info[NAVI_PAR]  = MsgInfo( NAVIPAR_NAME, NAVIPAR_FORMAT, NaviPar_message_handler );
	msg_info[ACTION_NAVI]  = MsgInfo( ACTION_NAVI_NAME, ACTION_NAVI_FORMAT, ActionNavi_message_handler );
	msg_info[RESULT_NAVI]  = MsgInfo( RESULT_NAVI_NAME, RESULT_NAVI_FORMAT, ResultNavi_message_handler );
	msg_info[SERVER_VEL]  = MsgInfo( SERVER_VEL_NAME, SERVER_VEL_FORMAT, ServerVel_message_handler );
	msg_info[INVALID_GOAL]  = MsgInfo( INVALID_GOAL_NAME, INVALID_GOAL_FORMAT, InvalidGoal_message_handler );
	msg_info[POSE_MODE]  = MsgInfo( POSE_MODE_NAME, POSE_MODE_FORMAT, PoseMode_message_handler );
	msg_info[MESSAGE_FREQ]  = MsgInfo( MESSAGE_FREQ_NAME, MESSAGE_FREQ_FORMAT, MessageFreq_message_handler );

	msg_info[ACTION_GCALENDAR]  = MsgInfo( ACTION_GCALENDAR_NAME, ACTION_GCALENDAR_FORMAT, Action_GC_message_handler );
	msg_info[RESULT_GCALENDAR]  = MsgInfo( RESULT_GCALENDAR_NAME, RESULT_GCALENDAR_FORMAT, Result_GC_message_handler );
	msg_info[SCHEDULE_INFO]  = MsgInfo( SCHEDULE_INFO_NAME, SCHEDULE_INFO_FORMAT, ScheduleInfo_message_handler );
	msg_info[CHECK_INFO]  = MsgInfo( CHECK_INFO_NAME, CHECK_INFO_FORMAT, CheckInfo_message_handler );
	msg_info[QUERY_RESULT]  = MsgInfo( QUERY_RESULT_NAME, QUERY_RESULT_FORMAT, QueryResult_message_handler );

	msg_info[PERCEPTION]  = MsgInfo( PERCEPTION_NAME, PERCEPTION_FORMAT, Perception_message_handler );
	msg_info[DBN]  = MsgInfo( DBN_NAME, DBN_FORMAT, DBN_message_handler );
	msg_info[FFORMATION]  = MsgInfo( FFORMATION_NAME, FFORMATION_FORMAT, Fformation_message_handler );
	msg_info[ACTION_APSTATE]  = MsgInfo( ACTION_APSTATE_NAME, ACTION_APSTATE_FORMAT, action_apState_message_handler );
	msg_info[RESULT_APSTATE]  = MsgInfo( RESULT_APSTATE_NAME, RESULT_APSTATE_FORMAT, result_apState_message_handler );
		/* For arm manipulating */
	msg_info[ACTION_ARM] = MsgInfo( ACTIONARM_NAME, ACTIONARM_FORMAT, actionarm_message_handler );
	msg_info[RESULT_ARM] = MsgInfo( RESULTARM_NAME, RESULTARM_FORMAT, resultarm_message_handler );
		/* For Toward Robot Attention Estimator (HAE) */
	msg_info[PERCEPTION_HAE]	=	MsgInfo(PERCEPTION_HAE_NAME, PERCEPTION_HAE_FORMAT, Perception_HAE_message_handler);
	msg_info[HAE]	=	MsgInfo(HAE_NAME, HAE_FORMAT, HAE_message_handler);
	msg_info[ATTENTIONLEVEL]	=	MsgInfo(ATTENTIONLEVEL_NAME, ATTENTIONLEVEL_FORMAT, Attention_Level_message_handler);
	msg_info[REQUEST_INFERENCE]	=	MsgInfo(REQUEST_INFERENCE_NAME, REQUEST_INFERENCE_FORMAT, Reqeust_Inference_message_handler);
	msg_info[ROBOTPARAMETER]	=	MsgInfo(ROBOTPARAMETER_NAME, ROBOTPARAMETER_FORMAT, Robot_Parameter_message_handler);
}

void define_message(const char *msgName, const char *formatString)
{
	if(!IPC_isMsgDefined(msgName)){
		printf("defining message(%s)!\n", msgName);
		IPC_defineMsg(msgName, IPC_VARIABLE_LENGTH, formatString);
	}
}

//--------------------------------------------------//
// send messages
//--------------------------------------------------//
	/* Free Message */

int sendLaserPose(LaserPoseMgr& mgr){
	return (int)IPC_publishData(LASER_POSE_NAME, &mgr);
}
int sendURGLaser(URGLaser& mgr){
	return (int)IPC_publishData(URGLASER_NAME, &mgr);
}
int sendSubgoal(SubgoalMgr& mgr){
	return (int)IPC_publishData(SUBGOAL_NAME, &mgr);
}

int sendGoal(GoalMgr& mgr){
	return (int)IPC_publishData(GOAL_NAME, &mgr);
}

int sendOdometry(OdometryMgr& mgr){
	return (int)IPC_publishData(ODOMETRY_NAME, &mgr);
}

int sendHeadMotor(HeadMotorMgr& mgr){;
	return (int)IPC_publishData(HEADMOTOR_NAME, &mgr);
}
int sendHandPose(HandPoseMgr& mgr){
	return (int)IPC_publishData(HANDPOS_NAME, &mgr);
}
int sendFacialExp(FacialExpMgr& mgr){
	return (int)IPC_publishData(FACIAlEXP_NAME, &mgr);
}
int sendState(StateMgr& mgr){
	return (int)IPC_publishData(STATE_NAME, &mgr);
}
int sendPeople(PeopleMgr& mgr){
	return (int)IPC_publishData(PEOPLE_NAME, &mgr);
}
int sendKeyWord(KeyWordMgr& mgr){
	return (int)IPC_publishData(KEY_WORD_NAME, &mgr);
}
int sendLocState(LocState& mgr){
	return (int)IPC_publishData(LOC_STATE_NAME, &mgr);
}
int sendActionSpeak(Action_Speak&mgr){
	return (int)IPC_publishData(ACTION_SPEAK_NAME, &mgr);
}
int sendResultSpeak(Result_Speak&mgr){
	return (int)IPC_publishData(RESULT_SPEAK_NAME, &mgr);
}
//for Navi
int sendNaviParMsg( NaviParMsg&mgr){
	return (int)IPC_publishData(NAVIPAR_NAME, &mgr);
}
int sendActionNavi(Action_Navi& mgr){
	return (int)IPC_publishData(ACTION_NAVI_NAME, &mgr);
}
int sendResultNavi(Result_Navi&mgr){
	return (int)IPC_publishData(RESULT_NAVI_NAME, &mgr);
}
int sendServerVel(ServerVelMgr& mgr){
	return (int)IPC_publishData(SERVER_VEL_NAME, &mgr);
}
int sendPoseMode(PoseModeMgr& mgr){
	return (int)IPC_publishData(POSE_MODE_NAME, &mgr);
}
int sendInvalidGoal(InvalidGoalMgr& mgr){
	return (int)IPC_publishData(INVALID_GOAL_NAME, &mgr);
}
int sendMessageFreq(MessageFreqMgr& mgr){
	return (int)IPC_publishData(MESSAGE_FREQ_NAME, &mgr);
}
//for Navi
//for google calendar
int sendAction_GCalendar(Action_GCalendarMgr &mgr){
	return (int)IPC_publishData(ACTION_GCALENDAR_NAME, &mgr);
}
int sendResult_GCalendar(Result_GCalendarMgr &mgr){
	return (int)IPC_publishData(RESULT_GCALENDAR_NAME, &mgr);
}
int sendScheduleInfo(ScheduleInfoMgr& mgr){
	return (int)IPC_publishData(SCHEDULE_INFO_NAME, &mgr);
}
int sendCheckInfo(CheckInfoMgr& mgr){
	return (int)IPC_publishData(CHECK_INFO_NAME, &mgr);
}
int sendQueryResult(QueryResultMgr& mgr){
	return (int)IPC_publishData(QUERY_RESULT_NAME, &mgr);
}
//google canendar

int sendPerception(PerceptionMgr& mgr){
	return (int)IPC_publishData(PERCEPTION_NAME, &mgr);
}
int sendDBN(DBNMgr& mgr){
	return (int)IPC_publishData(DBN_NAME, &mgr);
}
int sendFformation(FformationMgr& mgr){
	return (int)IPC_publishData(FFORMATION_NAME, &mgr);
}
int sendFaceDetect(FaceDetect& mgr){
	return (int)IPC_publishData(FACE_DETECT_NAME, &mgr);
}
int snedFaceRecog(FaceRecogMgr& mgr){
	return (int)IPC_publishData(FACE_RECOG_NAME, &mgr);
}
int sendAction_apState(Action_apStateMgr& mgr){
	return (int)IPC_publishData(ACTION_APSTATE_NAME, &mgr);
}
int sendResult_apState(Result_apStateMgr& mgr){
	return (int)IPC_publishData(RESULT_APSTATE_NAME, &mgr);
}

	/* For arm manipulating */
int sendActionArm(ActionArmMgr& mgr){
	return (int)IPC_publishData(ACTIONARM_NAME, &mgr);
}

int sendResultArm(ResultArmMgr& mgr){
	return (int)IPC_publishData(RESULTARM_NAME, &mgr);
}

	/* For Toward Robot Attention Estimator (HAE) */
int sendPerceptionHAE(PerceptionHAEMgr& mgr){
	return (int)IPC_publishData(PERCEPTION_HAE_NAME, &mgr);
}
int sendHAE(HAEMgr& mgr){
	return (int)IPC_publishData(HAE_NAME, &mgr);
}
int sendAttentionLevel(AttentionLevelMgr& mgr){
	return (int)IPC_publishData(ATTENTIONLEVEL_NAME, &mgr);
}

int sendRequestInference(RequestInferenceMgr& mgr){
	return (int)IPC_publishData(REQUEST_INFERENCE_NAME, &mgr);
}

int sendRobotParameter(RobotParameterMgr& mgr) {
	return (int) IPC_publishData(ROBOTPARAMETER_NAME, &mgr);
}