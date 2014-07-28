
/******************************************************
NTU EECS Advanced Control Lab, Intelligent Robot Lab

2010 Kuo-Chen Huang

******************************************************/
#include "Comm.hpp"

//--------------------------------------------------//
// messages
//--------------------------------------------------//

// laser_pose
static LaserPoseMgr laserpose_mgr;
static OdometryMgr corr_odometry;
//
static URGLaser urglaser_mgr;
static SubgoalMgr subgoal_mgr;
static GoalMgr goal_mgr;
static OdometryMgr odometry_mgr;
static HeadMotorMgr head_motor_mgr;
static HandPoseMgr handpose_mgr;
static FacialExpMgr facial_exp_mgr;
static StateMgr state_mgr;
static PeopleMgr people_mgr;
static KeyWordMgr key_word_mgr;
static LocState loc_state_mgr;
static FaceDetect detect_mgr;
static Action_Speak action_speak_mgr;
static Result_Speak result_speak_mgr;
//for Navi
static NaviParMsg navi_par_mgr;
static Action_Navi action_navi_mgr;
static Result_Navi result_navi_mgr;
static ServerVelMgr server_vel_mgr;
static PoseModeMgr pose_mode_mgr;
static InvalidGoalMgr invalid_goal_mgr;
static MessageFreqMgr message_freq_mgr;
//Navi
//for google calendar
static Action_GCalendarMgr action_gc_mgr;
static Result_GCalendarMgr result_gc_mgr;
static ScheduleInfoMgr schedule_info_mgr;
static CheckInfoMgr check_info_mgr;
static QueryResultMgr query_Result_mgr;
//google calendar
static Action_apStateMgr action_apstate_mgr;
static Result_apStateMgr result_apstate_mgr;
static PerceptionMgr perception_mgr;
static DBNMgr dbn_mgr;
static FformationMgr fformation_mgr;
static FaceDetect face_detect_mgr;
static FaceRecogMgr face_recog_mgr;
	/* For arm manipulating */
static ArmPositionMgr arm_position_mgr;
static ActionArmMgr action_arm_mgr;
static ResultArmMgr result_arm_mgr;
	/* For Toward Robot Attention Estimator (HAE) */
static PerceptionHAEMgr perception_HAE_mgr;
static HAEMgr hae_mgr;
static AttentionLevelMgr attention_level_mgr;
static RequestInferenceMgr request_inference_mgr;
static RobotParameterMgr robot_parameter_mgr;
	/* For grasping (ver. Nil) */
static ActionSearchGrasp action_search_grasp;
static ResultSearchGrasp result_search_grasp;

#ifdef WIN32

#include "../thread/locks.hpp"

static win32CriticalSection mgr_lock[TOTAL_MSG_NUM];
#define RD_LOCK( A ) mgr_lock[A].lock()
#define WR_LOCK( A ) mgr_lock[A].lock()
#define UNLOCK( A )  mgr_lock[A].unlock()

#else

static pthread_rwlock_t mgr_lock[TOTAL_MSG_NUM];
#define RD_LOCK( A ) pthread_rwlock_rdlock( &mgr_lock[A] )
#define WR_LOCK( A ) pthread_rwlock_wrlock( &mgr_lock[A] )
#define UNLOCK( A )  pthread_rwlock_unlock( &mgr_lock[A] )

#endif

#include <cmath>
#define M_PI       3.14159265358979323846

void init_comm()
{
#ifndef WIN32
	for(int i = 0 ; i < TOTAL_MSG_NUM ; ++i){
		pthread_rwlock_init(&mgr_lock[i], NULL);
	}
#endif
}

//--------------------------------------------------//
// get messages
//--------------------------------------------------//
void getLaserPose(LaserPoseMgr& mgr)
{
	OdometryMgr prev_odo, current_odo;

	RD_LOCK( LASER_POSE );
	mgr = laserpose_mgr;
	prev_odo = corr_odometry;
	UNLOCK( LASER_POSE );

	RD_LOCK( ODOMETRY );
	current_odo = odometry_mgr;
	UNLOCK( ODOMETRY );

	double dx = current_odo.x - prev_odo.x;
	double dy = current_odo.y - prev_odo.y;
	
	if( dx != 0 || dy != 0){
		double dist = sqrt( dx * dx + dy *dy );
		double moving_direction = atan2( dy, dx ) - prev_odo.theta + mgr.theta;
		
		mgr.x += dist * cos( moving_direction );
		mgr.y += dist * sin( moving_direction );
		double theta = current_odo.theta - prev_odo.theta + mgr.theta;

		if (theta < -M_PI || theta > M_PI){
			double multiplier = floor(theta / (2*M_PI));
			theta = theta - multiplier*2*M_PI;
			if (theta > M_PI)
				theta -= 2*M_PI;
			if (theta < -M_PI)
				theta += 2*M_PI;
		}
		mgr.theta = theta;
	}
}

void getURGLaser(URGLaser& mgr)
{
	RD_LOCK( URG );
	mgr = urglaser_mgr;
	UNLOCK( URG );
}

void getSubgoal(SubgoalMgr& mgr)
{
	RD_LOCK( SUBGOAL );
	mgr = subgoal_mgr;
	UNLOCK( SUBGOAL );
}

void getGoal(GoalMgr& mgr)
{
	RD_LOCK( GOAL );
	mgr = goal_mgr;
	UNLOCK( GOAL );
}

void getOdometry(OdometryMgr& mgr)
{
	RD_LOCK( ODOMETRY );
	mgr = odometry_mgr;
	UNLOCK( ODOMETRY );
}

void getHeadMotor(HeadMotorMgr& mgr){
	RD_LOCK( HEAD_MOTOR );
	mgr = head_motor_mgr;
	UNLOCK( HEAD_MOTOR );
}

void getHandPose(HandPoseMgr& mgr){
	RD_LOCK( HAND_POSE );
	mgr = handpose_mgr;
	UNLOCK( HAND_POSE );
}

void getFacialExp(FacialExpMgr& mgr)
{
	RD_LOCK( FACIAL_EXP );
	mgr = facial_exp_mgr;
	UNLOCK( FACIAL_EXP );
}

void getState(StateMgr& mgr)
{
	RD_LOCK( STATE );
	mgr = state_mgr;
	UNLOCK( STATE );
}

void getPeople(PeopleMgr& mgr)
{
	RD_LOCK( PEOPLE );
	mgr = people_mgr;
	UNLOCK( PEOPLE );
}

void getKeyWord(KeyWordMgr& mgr)
{
	// writer lock !!
	WR_LOCK( KEY_WORD );	
	mgr = key_word_mgr;
	UNLOCK( KEY_WORD );
}

void getLocState(LocState& mgr){
	RD_LOCK( LOC_STATE );
	mgr = loc_state_mgr;
	UNLOCK( LOC_STATE );
}

void getActionSpeak( Action_Speak& mgr )
{
	RD_LOCK( ACTION_SPEAK );
	mgr = action_speak_mgr;
	UNLOCK( ACTION_SPEAK );
}

void getResultSpeak( Result_Speak& mgr )
{
	RD_LOCK( RESULT_SPEAK );
	mgr = result_speak_mgr;
	UNLOCK( RESULT_SPEAK );
}

//for Navi
void getNaviParMsg( NaviParMsg& mgr )
{
	RD_LOCK( NAVI_PAR );
	mgr = navi_par_mgr;
	UNLOCK( NAVI_PAR );
}

void getActionNavi(Action_Navi & mgr)
{
	RD_LOCK( ACTION_NAVI );
	mgr = action_navi_mgr;
	UNLOCK( ACTION_NAVI );
}

void getResultNavi(Result_Navi & mgr)
{
	RD_LOCK( RESULT_NAVI );
	mgr = result_navi_mgr;
	UNLOCK( RESULT_NAVI );
}

void getServerVel(ServerVelMgr & mgr)
{
	RD_LOCK( SERVER_VEL );
	mgr = server_vel_mgr;
	UNLOCK( SERVER_VEL );
}

void getPoseMode(PoseModeMgr & mgr)
{
	RD_LOCK( POSE_MODE );
	mgr = pose_mode_mgr;
	UNLOCK( POSE_MODE );
}

void getInvalidGoal(InvalidGoalMgr &mgr)
{
	RD_LOCK( INVALID_GOAL );
	mgr = invalid_goal_mgr;
	UNLOCK( INVALID_GOAL );
}

void getMessageFreq(MessageFreqMgr &mgr)
{
	RD_LOCK( MESSAGE_FREQ );
	mgr = message_freq_mgr;
	UNLOCK( MESSAGE_FREQ );
}
//Navi
//for google calendar
void getAction_GCalendar(Action_GCalendarMgr& mgr)
{
	RD_LOCK( ACTION_GCALENDAR );
	mgr = action_gc_mgr;
	UNLOCK( ACTION_GCALENDAR );
}

void getResult_GCalendar(Result_GCalendarMgr& mgr)
{
	RD_LOCK( RESULT_GCALENDAR );
	mgr = result_gc_mgr;
	UNLOCK( RESULT_GCALENDAR );
}

void getScheduleInfo(ScheduleInfoMgr &mgr)
{
	RD_LOCK( SCHEDULE_INFO );
	mgr = schedule_info_mgr;
	UNLOCK( SCHEDULE_INFO );
}

void getCheckInfo(CheckInfoMgr &mgr)
{
	RD_LOCK( CHECK_INFO );
	mgr = check_info_mgr;
	UNLOCK( CHECK_INFO );
}

void getQueryResult(QueryResultMgr& mgr)
{
	RD_LOCK( QUERY_RESULT );
	mgr = query_Result_mgr;
	UNLOCK( QUERY_RESULT );
}
//google canendar
void getAction_apState(Action_apStateMgr& mgr)
{
	RD_LOCK( FFORMATION );
	mgr = action_apstate_mgr;
	UNLOCK( FFORMATION );
}

void getResult_apState(Result_apStateMgr& mgr)
{
	RD_LOCK( FFORMATION );
	mgr = result_apstate_mgr;
	UNLOCK( FFORMATION );
}

void getPerception(PerceptionMgr& mgr)
{
	RD_LOCK( PERCEPTION );
	mgr = perception_mgr;
	UNLOCK( PERCEPTION );
}

void getDBN(DBNMgr& mgr)
{
	RD_LOCK( DBN );
	mgr = dbn_mgr;
	UNLOCK( DBN );
}

void getFformaiton(FformationMgr& mgr)
{
	RD_LOCK( FFORMATION );
	mgr = fformation_mgr;
	UNLOCK( FFORMATION );
}

void getFaceDetect(FaceDetect& mgr)
{
	RD_LOCK( FACE_DETECT );
	mgr = detect_mgr;
	UNLOCK( FACE_DETECT );
}

void getFaceRecog(FaceRecogMgr& mgr)
{
	RD_LOCK( FACE_RECOG );
	mgr = face_recog_mgr;
	sprintf(face_recog_mgr.Name, "");
	//for(int i=0; i<face_recog_mgr.count; i++){
	//	sprintf(face_recog_mgr.Name[i], "");
	//}
	UNLOCK( FACE_RECOG );
}

	/* For arm manipulating */
void getArmPosition(ArmPositionMgr& mgr)
{
	RD_LOCK( ARM_POSITION );
	mgr = arm_position_mgr;
	UNLOCK( ARM_POSITION );
}

void getActionArm(ActionArmMgr& mgr)
{
	RD_LOCK( ACTION_ARM );
	mgr = action_arm_mgr;
	UNLOCK( ACTION_ARM );
}

void getResultArm(ResultArmMgr& mgr)
{
	RD_LOCK( RESULT_ARM );
	mgr = result_arm_mgr;
	UNLOCK( RESULT_ARM );
}

	/* For Toward Robot Attention Estimator (HAE) */
void getPerceptionHAE(PerceptionHAEMgr &mgr) {
	RD_LOCK( PERCEPTION_HAE );
	mgr = perception_HAE_mgr;
	UNLOCK( PERCEPTION_HAE );
}

void getHAE(HAEMgr &mgr) {
	RD_LOCK( HAE );
	mgr = hae_mgr;
	UNLOCK( HAE );
}

void getAttentionLevel(AttentionLevelMgr &mgr) {
	RD_LOCK( ATTENTIONLEVEL );
	mgr = attention_level_mgr;
	UNLOCK( ATTENTIONLEVEL );
}

void getRequestInference(RequestInferenceMgr &mgr) {
	RD_LOCK( REQUEST_INFERENCE );
	mgr = request_inference_mgr;
	UNLOCK( REQUEST_INFERENCE );
}

void getRobotParameter(RobotParameterMgr &mgr) {
	RD_LOCK( ROBOTPARAMETER );
	mgr = robot_parameter_mgr;
	UNLOCK( ROBOTPARAMETER );
}

	/* For grasping (ver. Nil) */
void getActionSearchGrasp( ActionSearchGrasp& msg )
{
	RD_LOCK(ACTION_SEARCH_GRASP);
	msg = action_search_grasp;
	UNLOCK(ACTION_SEARCH_GRASP);
}

void getResultSearchGrasp( ResultSearchGrasp& msg )
{
	RD_LOCK(RESULT_SEARCH_GRASP);
	msg = result_search_grasp;
	UNLOCK(RESULT_SEARCH_GRASP);
}

//--------------------------------------------------//
// set messages
//--------------------------------------------------//
/*
void setLaserPose(LaserPoseMgr& lmgr, OdometryMgr& omgr)
{
	WR_LOCK( LASERPOSE );
	laserpose_mgr = lmgr;
	corr_odometry = omgr;
	UNLOCK( LASERPOSE );
}
*/
void setLaserPose(LaserPoseMgr& mgr)
{
	WR_LOCK( LASER_POSE );
	laserpose_mgr = mgr;
	UNLOCK( LASER_POSE );
}

void setURGLaser(URGLaser& mgr)
{
	WR_LOCK( URG );
	urglaser_mgr = mgr;
	UNLOCK( URG );
}

void setSubgoal(SubgoalMgr& mgr)
{
	WR_LOCK( SUBGOAL );
	subgoal_mgr = mgr;
	UNLOCK( SUBGOAL );
}

void setGoal(GoalMgr& mgr)
{
	WR_LOCK( GOAL );
	goal_mgr = mgr;
	UNLOCK( GOAL );
}

void setOdometry(OdometryMgr& mgr)
{
	WR_LOCK( ODOMETRY );
	odometry_mgr = mgr;
	UNLOCK( ODOMETRY );
}

void setHeadMotor(HeadMotorMgr& mgr)
{
	WR_LOCK( HEAD_MOTOR );
	head_motor_mgr = mgr;
	UNLOCK( HEAD_MOTOR );
}

void setHandPose(HandPoseMgr& mgr)
{
	WR_LOCK( HAND_POSE );
	handpose_mgr = mgr;
	UNLOCK( HAND_POSE );
}

void setFacialExp(FacialExpMgr& mgr)
{
	WR_LOCK( FACIAL_EXP );
	facial_exp_mgr = mgr;
	UNLOCK( FACIAL_EXP );
}

void setState(StateMgr& mgr)
{
	WR_LOCK( STATE );
	state_mgr = mgr;
	UNLOCK( STATE );
}

void setPeople(PeopleMgr& mgr)
{
	WR_LOCK( PEOPLE );
	people_mgr = mgr;
	UNLOCK( PEOPLE );
}

void setKeyWord(KeyWordMgr& mgr)
{
	WR_LOCK( KEY_WORD );
	key_word_mgr = mgr;
	UNLOCK( KEY_WORD );
}

void setLocState(LocState& mgr)
{
	WR_LOCK( LOC_STATE );
	loc_state_mgr = mgr;
	UNLOCK( LOC_STATE );
}

void setActionSpeak( Action_Speak& mgr )
{
	WR_LOCK( ACTION_SPEAK );
	action_speak_mgr = mgr;
	UNLOCK( ACTION_SPEAK );
}

void setResultSpeak( Result_Speak& mgr )
{
	RD_LOCK( RESULT_SPEAK );
	result_speak_mgr = mgr;
	UNLOCK( RESULT_SPEAK );
}

//for Navi
void setNaviParMsg( NaviParMsg& mgr )
{
	WR_LOCK( NAVI_PAR );
	navi_par_mgr = mgr;
	UNLOCK( NAVI_PAR );
}

void setActionNavi(Action_Navi & mgr)
{
	RD_LOCK( ACTION_NAVI );
	action_navi_mgr = mgr;
	UNLOCK( ACTION_NAVI );
}

void setResultNavi(Result_Navi & mgr)
{
	RD_LOCK( RESULT_NAVI );
	result_navi_mgr = mgr;
	UNLOCK( RESULT_NAVI );
}

void setServerVel(ServerVelMgr & mgr)
{
	RD_LOCK( SERVER_VEL );
	server_vel_mgr = mgr;
	UNLOCK( SERVER_VEL );
}

void setPoseMode(PoseModeMgr & mgr)
{
	RD_LOCK( POSE_MODE );
	pose_mode_mgr = mgr;
	UNLOCK( POSE_MODE );
}

void setInvalidGoal(InvalidGoalMgr &mgr)
{
	RD_LOCK( INVALID_GOAL );
	invalid_goal_mgr = mgr;
	UNLOCK( INVALID_GOAL );
}

void setMessageFreq(MessageFreqMgr &mgr)
{
	RD_LOCK( MESSAGE_FREQ );
	message_freq_mgr = mgr;
	UNLOCK( MESSAGE_FREQ );
}
//Navi
//for google calendar
void setAction_GCalendar(Action_GCalendarMgr& mgr)
{
	WR_LOCK( ACTION_GCALENDAR );
	action_gc_mgr = mgr;
	UNLOCK( ACTION_GCALENDAR );
}

void setResult_GCalendar(Result_GCalendarMgr& mgr)
{
	RD_LOCK( RESULT_GCALENDAR );
	result_gc_mgr = mgr;
	UNLOCK( RESULT_GCALENDAR );
}

void setScheduleInfo(ScheduleInfoMgr &mgr)
{
	RD_LOCK( SCHEDULE_INFO );
	schedule_info_mgr = mgr;
	UNLOCK( SCHEDULE_INFO );
}

void setCheckInfo(CheckInfoMgr &mgr)
{
	RD_LOCK( CHECK_INFO );
	check_info_mgr = mgr;
	UNLOCK( CHECK_INFO );
}

void setQueryResult(QueryResultMgr& mgr)
{
	RD_LOCK( QUERY_RESULT );
	query_Result_mgr = mgr;
	UNLOCK( QUERY_RESULT );
}
//google canendar
void setAction_apState(Action_apStateMgr& mgr)
{
	WR_LOCK(FFORMATION);
	action_apstate_mgr = mgr;
	UNLOCK(FFORMATION);
}

void setResult_apState(Result_apStateMgr& mgr)
{
	WR_LOCK(FFORMATION);
	result_apstate_mgr = mgr;
	UNLOCK(FFORMATION);
}

void setPerception(PerceptionMgr& mgr)
{
	WR_LOCK(PERCEPTION );
	perception_mgr = mgr;
	UNLOCK(PERCEPTION  );
}

void setDBN(DBNMgr& mgr)
{
	WR_LOCK(DBN);
	dbn_mgr = mgr;
	UNLOCK(DBN);
}

void setFformation(FformationMgr& mgr)
{
	WR_LOCK(FFORMATION);
	fformation_mgr = mgr;
	UNLOCK(FFORMATION);
}

void setFaceDetect(FaceDetect& mgr)
{
	WR_LOCK( FACE_DETECT );
	detect_mgr = mgr;
	UNLOCK( FACE_DETECT );
}

void setFaceRecog(FaceRecogMgr& mgr)
{
	WR_LOCK( FACE_RECOG );
	face_recog_mgr = mgr;
	UNLOCK( FACE_RECOG );
}

	/* For arm manipulating */
void setArmPosition(ArmPositionMgr& mgr)
{
	WR_LOCK(ARM_POSITION);
	arm_position_mgr = mgr;
	UNLOCK(ARM_POSITION);
}

void setActionArm(ActionArmMgr& mgr)
{
	WR_LOCK( ACTION_ARM );
	action_arm_mgr = mgr;
	UNLOCK( ACTION_ARM );
}

void setResultArm(ResultArmMgr& mgr)
{
	WR_LOCK( RESULT_ARM );
	result_arm_mgr = mgr;
	UNLOCK( RESULT_ARM );
}

	/* For Toward Robot Attention Estimator (HAE) */
void setPerceptionHAE(PerceptionHAEMgr &mgr) {
	WR_LOCK( PERCEPTION_HAE );
	perception_HAE_mgr = mgr;
	UNLOCK( PERCEPTION_HAE );
}

void setHAE(HAEMgr &mgr) {
	WR_LOCK( HAE );
	hae_mgr = mgr;
	UNLOCK( HAE );
}

void setAttentionLevel(AttentionLevelMgr &mgr) {
	WR_LOCK( ATTENTIONLEVEL );
	attention_level_mgr = mgr;
	UNLOCK( ATTENTIONLEVEL );
}

void setRequestInference(RequestInferenceMgr &mgr) {
	WR_LOCK( REQUEST_INFERENCE );
	request_inference_mgr = mgr;
	UNLOCK( REQUEST_INFERENCE );
}

void setRobotParameter(RobotParameterMgr &mgr) {
	WR_LOCK( ROBOTPARAMETER );
	robot_parameter_mgr = mgr;
	UNLOCK( ROBOTPARAMETER );
}

	/* For grasping */
void setActionSearchGrasp( ActionSearchGrasp& msg )
{
	RD_LOCK(ACTION_SEARCH_GRASP);
	action_search_grasp = msg;
	UNLOCK(ACTION_SEARCH_GRASP);
}

void setResultSearchGrasp( ResultSearchGrasp& msg )
{
	RD_LOCK(RESULT_SEARCH_GRASP);
	result_search_grasp = msg;
	UNLOCK(RESULT_SEARCH_GRASP);
}