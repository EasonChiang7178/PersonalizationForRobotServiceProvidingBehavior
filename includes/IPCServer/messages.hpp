/******************************************************
NTU EECS Advanced Control Lab, Intelligent Robot Lab

2010 Kuo-Chen Huang 
Ming-Fang Chang 
	 add Action/Result mgrs
2013 
******************************************************/

#ifndef MESSAGE_H
#define MESSAGE_H

#include <stdio.h>
#include <cstring>

//--------------------------------------------------//
// define the message structures of each sensor data
//--------------------------------------------------//

class PoseMsg{
public:
	PoseMsg(){}
	PoseMsg( double i_x, double i_y, double i_theta ) : x(i_x), y(i_y), theta(i_theta) {}
	double x;
	double y;
	double theta;
};

typedef PoseMsg GoalMgr;
typedef PoseMsg SubgoalMgr;
typedef PoseMsg LaserPoseMgr;
typedef PoseMsg OdometryMgr;

#define LASER_POSE_NAME "LaserPoseMgr"
#define LASER_POSE_FORMAT "{double, double, double}"

#define SUBGOAL_NAME "SubgoalMgr"
#define SUBGOAL_FORMAT "{double, double, double}"

#define GOAL_NAME "GoalMgr"
#define GOAL_FORMAT "{double, double, double}"

#define ODOMETRY_NAME "OdometryMgr"
#define ODOMETRY_FORMAT "{double, double, double}"

#define URG_LASER_NUM 682
class URGLaser{
public:
	URGLaser(){memset(range,0,URG_LASER_NUM);}
	int range[URG_LASER_NUM];
};
#define URGLASER_NAME "URGLaserMgr"
#define URGLASER_FORMAT "{[int:682]}"

// Hand
enum hand_act{SHAKEHAND=1,COME,RAISEHAND,TAKEOBJ,LOOSE,GRASP_V,GRASP_D,END,PREGRASP,MDEG,WAVEHAND,STOP,TURN,POUR_WATER,STANDBY,REPOSE,LOOSE_P,PUTOBJ,TAKESCROLL, FIVE_JOINT_MOVE, SIX_JOINT_MOVE};
class HandPoseMgr{
public:
	double obj[16];
	double width;
	double deg;
	double gripv;
	double j1length;
	int done;
	int HPos;
};
#define HANDPOS_NAME "HandPoseMgr"
#define HANDPOS_FORMAT "{[double:16], double, double, double, double, int, int}"

//HeadMotor
enum actions{BY_DEG, BY_KEYBOARD, ACTION_STOP, ACTION_UP, ACTION_DOWN, ACTION_LEFT, ACTION_RIGHT,NODHEAD,SHAKEHEAD};
class HeadMotorMgr{
public:
	double RLpos;
	double DUpos;
	double RLgoal;
	double DUgoal;
	double action;

};
#define HEADMOTOR_NAME "HeadMotorMgr"
#define HEADMOTOR_FORMAT "{double, double, double, double, double}"

// facial expression
class FacialExpMgr{
public:
	int state;
};
#define FACIAlEXP_NAME "FacialExpMgr"
#define FACIAlEXP_FORMAT "{int}"

//robot state
class StateMgr{
public: 
	char status[256];
	//char webcontrol[256];

};
#define STATE_NAME "StateMgr"
#define STATE_FORMAT "{[char:256]}"

#define MAX_PEOPLE  10
class PeopleMgr{
public:
	PeopleMgr():count(0){
		memset(x,0,MAX_PEOPLE*sizeof(float));
		memset(y,0,MAX_PEOPLE*sizeof(float));
		memset(vel,0,MAX_PEOPLE*sizeof(float));
		memset(theta,0,MAX_PEOPLE*sizeof(float));
	}
	float count;
	float x[MAX_PEOPLE];
	float y[MAX_PEOPLE];
	float vel[MAX_PEOPLE];
	float theta[MAX_PEOPLE];
};
#define PEOPLE_NAME "PeopleMgr"
#define PEOPLE_FORMAT "{float, [float:10], [float:10], [float:10], [float:10]}"

class KeyWordMgr{
public:
	char keyword[256];
};
#define KEY_WORD_NAME "WordMgr"
#define KEY_WORD_FORMAT "{[char:256]}"

#define max_face  3
class FaceDetect{
public:
	FaceDetect():count(0){
		memset(x,0,max_face*sizeof(int));
		memset(y,0,max_face*sizeof(int));
		memset(width,0,max_face*sizeof(int));
		memset(height,0,max_face*sizeof(int));
	}
	int count;
	int x[max_face];
	int y[max_face];
	int width[max_face];
	int height[max_face];
};
#define FACE_DETECT_NAME "FaceDetect"
#define FACE_DETECT_FORMAT "{int, [int:3], [int:3], [int:3], [int:3]}"

class FaceRecogMgr{
public:
	char Name[128];
};
#define FACE_RECOG_NAME "FaceRecogMgr"
#define FACE_RECOG_FORMAT "{[char:128]}"
// loc state : LOC_INIT, LOC_OK, LOC_FAILURE
class LocState{
public:
	LocState(){ sprintf( state, "LOC_INIT"); }
	char state[16];
};
#define LOC_STATE_NAME "LocState"
#define LOC_STATE_FORMAT "{[char:16]}"

class Action_Speak{
public:
	char words[64];
	float voiceVolume;
};
#define ACTION_SPEAK_NAME "Action_Speak"
#define ACTION_SPEAK_FORMAT "{ [char:64], float}"

class Result_Speak{
public:
	int status;
};
#define RESULT_SPEAK_NAME "Result_Speak"
#define RESULT_SPEAK_FORMAT "{ int }"

//for Navi
typedef enum {WAITING_FOR_BEGINNING, EXECUTING, FINISHED, ERRORING, RECOVERING} NAVI_CURRENT_STATUS;
typedef enum {GENERAL, TO_POINT, HUMAN_FOLLOWING, HUMAN_GUIDING, CONTROL_BY_IPC, CONTROL_BY_KEYBOARD } NAVI_ACTIONS;

class Action_Navi{
public:
	int action_name; //NAVI_ACTIONS
};
#define ACTION_NAVI_NAME "Action_Navi"
#define ACTION_NAVI_FORMAT "{ int }"

class Result_Navi{
public:
	int current_status; //CURRENT_STATUS
	int action_name; // NAVI_ACTIONS
};

#define RESULT_NAVI_NAME "Result_Navi"
#define RESULT_NAVI_FORMAT "{int, int}"

class ServerVelMgr{
public:
	int RVel; 
	int LVel;
};

#define SERVER_VEL_NAME "Server_Vel"
#define SERVER_VEL_FORMAT "{int, int}"

//enum POSE_MODES{LASER_POSE, ODOMETRY_POSE};
class PoseModeMgr{
public:
	int pose_name;
};
#define POSE_MODE_NAME "PoseModeMgr"
#define POSE_MODE_FORMAT "{ int }"

class InvalidGoalMgr{
public:
	int invalid_goal;
};
#define INVALID_GOAL_NAME "InvalidGoalMgr"
#define INVALID_GOAL_FORMAT "{int}"

class MessageFreqMgr{
public:
	int message_freq;
};
#define MESSAGE_FREQ_NAME "MessageFreqMgr"
#define MESSAGE_FREQ_FORMAT "{int}"


typedef enum { MANUAL_MODE, AUTO_MODE } NAVI_MODE;
typedef enum { NO_OP, UP, DOWN, LEFT, RIGHT } MANUAL_DIRECTION;

class NaviParMsg{
public:
	NaviParMsg() : NaviMode(AUTO_MODE), Map_Enable(true), GoalSeeking_Enable(true), ObstacleAvoidance_Enable(true),
	PathPlanning_Enable(true), ND_Enable(true), Localization_Enable(true){}

	NAVI_MODE NaviMode;
	int dummy;

	bool Map_Enable;
	bool GoalSeeking_Enable;
	bool ObstacleAvoidance_Enable;
	bool PathPlanning_Enable;
	bool ND_Enable;
	bool Localization_Enable;

	// Manual
	double M_LinearV;
	double M_AngularV;
	//int Manual_Direction;
	// Auto
	double A_LinearV;
	double A_AngularV;
	double MaxLinearV;
	double MaxAngularV;
	double Kv;
	double Kw;
	double Ke;
	double ActiveRange;
	double DeadZoneRange;
	double LeftBalance;
	double AngleBalance;
	double TotalBalance;
};
#define NAVIPAR_NAME "NaviParMsg"
#define NAVIPAR_FORMAT "{double, byte, byte, byte, byte, byte, byte, byte, byte, double, double, double, double, double, double, double, double, double, double, double, double, double, double}"
//Navi

//For Google Calendar
typedef enum {ADD_SCHEDULE, DELETE_SCHEDULE, MODIFY_SCHEDULE, QUERY_SPECI_SCHEDULE,
	QUERY_TODAY_SCHEDULE, ERROR_GC, WAITING_FOR_COMMAND} GCALENDAR_CURRENT_STATUS;
	// Action definiton
typedef enum {FREE_ACTION_GC, QUERY_TODAY_SKD, QUERY_NEXTWEEK_SKD, ADD_SKD, DELETE_SKD, MODIFY_SKD,
				CHECK_BY_TIME, CHECK_BY_CONTENT} GCALENDAR_ACTIONS;
class Action_GCalendarMgr{
public:
	int action_name;	//GCALENDAR_ACTIONS
};
#define ACTION_GCALENDAR_NAME "Action_GCalendar"
#define ACTION_GCALENDAR_FORMAT "{ int }"

class Result_GCalendarMgr{
public:
	int current_status;	//CURRENT_STATUS
	int action_name;	//GCALENDAR_ACTIONS
};
#define RESULT_GCALENDAR_NAME "Result_GCalendar"
#define RESULT_GCALENDAR_FORMAT "{int, int}"

class ScheduleInfoMgr{
public:
	char schedule_title[32];	/*----The info of shedule want to be added, or queried----*/
	char schedule_date_time[48];/* Format: YYYY mm/dd HH:MM:SS                            */
	char schedule_location[16];	/*--------------------------------------------------------*/
	char person[32];			// whose schedule
};
#define SCHEDULE_INFO_NAME "ScheduleInfoMgr"
#define SCHEDULE_INFO_FORMAT "{[char:32], [char:48], [char:16], [char:32]}"

class CheckInfoMgr{
public:
	char who[32];				// whose schedule
	char title[32];				// the schedule want to be deleted, or check by content
	char date_time[48];			// check by time. Format: YYYY mm/dd HH:MM:SS
};
#define CHECK_INFO_NAME "CheckInfoMgr"
#define CHECK_INFO_FORMAT "{[char:32], [char:32], [char:48]}"

class QueryResultMgr{
public:
	char content[256];				// content of query result
};
#define QUERY_RESULT_NAME "QueryResultMgr"
#define QUERY_RESULT_FORMAT "{[char:256]}"
//Google Calendar

typedef enum {faceRecog, bodyDirection, socialRange, faceDetect, DBNF, bodyDetection, legDetection, robotParameter} pCom;
class PerceptionMgr{
public:
	pCom sensing;				// content of query result
};
#define PERCEPTION_NAME "PerceptionMgr"
#define PERCEPTION_FORMAT "{int}"

typedef enum{HR, HH, HHR} fForm;
class DBNMgr{
public:
	fForm socailForm;				// content of query result
};
#define DBN_NAME "DBNMgr"
#define DBN_FORMAT "{int}"

typedef enum {None_in, Single_in, Multi_in} Social_Space_type;
typedef enum {No_People, H2R, H2H, HH2R} Body_Direction_type;
class FformationMgr
{
	public:
	Social_Space_type social_space;
	Body_Direction_type body_direction;
	int face_num;
};
#define FFORMATION_NAME "FformationMgr"
#define FFORMATION_FORMAT "{int, int, int}"
// TP //
typedef enum {
	AP_READY,
	AP_RUNNING,
	AP_RECOVERING,	// not used so far
	AP_FINISH,
	AP_FAIL,		// not used so far
	AP_KILL
} ApState;

class Action_apStateMgr {
public:
	char apName[16];
	char argument[128];
	ApState state;
};
#define ACTION_APSTATE_NAME "Action_apStateMgr"
#define ACTION_APSTATE_FORMAT "{[char:16],[char:128], int}"

class Result_apStateMgr {
public:
	char apName[16];
	ApState state;
};
#define RESULT_APSTATE_NAME "Result_apStateMgr"
#define RESULT_APSTATE_FORMAT "{[char:16], int}"

/* For arm manipulating */
	//Arm Position for action planner
class ArmPositionMgr{
public:
	double x_pos;
	double y_pos;
	double z_pos;
	double move_time;
	double headDeg;
};
#define ARMPOSITION_NAME "ArmPositionMgr"
#define ARMPOSITION_FORMAT "{double, double, double, double, double}"

	// ActionArm
enum action_arm{ARM_STOP, ARM_MOVE, ARM_GRAB, ARM_PUT, ARM_HOME, ARM_PREGRASP, ARM_LIMIT_PREGRASP, ARM_BASE_MOVE, 
				ARM_ROLL_HOME, ARM_ROLL_CLOCKWISE, ARM_ROLL_COUNTERCLOCKWISE, ARM_FORWARD_A_BIT, ARM_BACKWARD_A_BIT, 
				ARM_HEAD_DOWN, ARM_HEAD_UP, ARM_HEAD_SHAKE, ARM_WAVE};class ActionArmMgr{
public:
	int armState;
};
#define ACTIONARM_NAME "ActionArmMgr"
#define ACTIONARM_FORMAT "{int}"

	// ResultArm
enum result_arm{ARM_IDLE, ARM_EXECUTING, ARM_FINISHED, ARM_ERROR, ARM_FAIL};
class ResultArmMgr{
public:
	int armState;
};
#define RESULTARM_NAME "ResultArmMgr"
#define RESULTARM_FORMAT "{int}"
/* For arm manipulating */

/*** Human Attention Estimator (HAE) ***/
	// Perception Communication
typedef enum{faceDirectionDiscrete, bodyDirectionDiscrete, voiceDetection, bodyDirectionCont, attentionContext, puMeasurement} pCom_HAE;
class PerceptionHAEMgr{
public:
	pCom_HAE sensing;			// content of query result
};
#define PERCEPTION_HAE_NAME		"PerceptionHAEMgr"
#define PERCEPTION_HAE_FORMAT	"{int}"

	// Observation of HAE
typedef enum {None_HAE_Face, Far_Center_Face, Near_Center_Face, Center_Face} Face_Direction_HAE_type;
typedef enum {None_HAE_Body, Far_Center_Body, Near_Center_Body, Center_Body} Body_Direction_HAE_type;
typedef enum {None, Low, Medium, High}										 AudioVolume_type;
typedef enum {Pleasantness, Like, Neutral, Offensive, Unpleasantness}		 PU_type;
class HAEMgr
{
public:
	Face_Direction_HAE_type		face_direction;
	Body_Direction_HAE_type		body_direction;
	AudioVolume_type			voice_detection;
	PU_type						pu;
	float						body_direction_cont;
};
#define HAE_NAME	"HAEMgr"
#define HAE_FORMAT	"{int, int, int, int, float}"

	// State of HAE, output to RL agent
typedef enum{Neglect, Contingency, AttentionHigh, InterestInRobot} AttentionLevel_HAE_type;
class AttentionLevelMgr{
public:
	AttentionLevel_HAE_type attentionLevel;		// content of query result
};
#define ATTENTIONLEVEL_NAME		"AttentionLevelMgr"
#define ATTENTIONLEVEL_FORMAT	"{int}"

	// Request HAE output result
typedef enum{AttentionLevel} highInferenceType;
class RequestInferenceMgr{
public:
	highInferenceType sensing;			// content of query result
};
#define REQUEST_INFERENCE_NAME		"RequestInferenceMgr"
#define REQUEST_INFERENCE_FORMAT	"{int}"
/*** Human Attention Estimator ***/

/*** Social Attention Model (SAM) ***/
	// Observation of robot parameters
typedef enum {Low_RSV, Medium_RSV, High_RSV} robotSpeechVolume_type;
typedef enum {Low_RMS, Medium_RMS, High_RMS} robotMotionSpeed_type;
class RobotParameterMgr {
public:
	robotSpeechVolume_type	volume;
	robotMotionSpeed_type	speed;
};
#define ROBOTPARAMETER_NAME		"RobotParameterMgr"
#define ROBOTPARAMETER_FORMAT	"{int, int}"
/*** Social Attention Model (SAM) ***/

#endif
