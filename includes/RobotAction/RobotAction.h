
#ifndef ROBOTACTION_H
#define ROBOTACTION_H

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
using namespace std;

#define M_PI       3.14159265358979323846

#include "IPCserver/client.hpp"
#include "../CoordTrans/CoordTrans.h"

class RobotAction {
	public:
		/* Constructors */
		RobotAction();
		RobotAction(const string& IPCserverAddress);
			// Copy constructor
		RobotAction(const RobotAction& copy);
		
		/* Destructor */
		~RobotAction();

		/* Overloading assignment operator */
		RobotAction& operator=(const RobotAction& copy);

		/* Primitive Robot Actions, return false if it cannot receive the feedback from component (Timeout) */
			// Idle
		const bool doNothing(const int& sleepTime);

			// For Head Shake
		const bool headShake(const int& swingRange, const int& destDegree);
			// Turn face
		const bool turningFace(const int& goalDegree);
			// Turn face to human, return the degree that head turns
		const int turnFaceToHuman();

			// Arm wave
		const bool armWave();

			// For Navigation
		const bool toPoint(const double& x, const double& y, const double& theta);
			// Rotate the body
		const bool rotation(const double& swingRange);
			// For Approach
		const bool forwardApproach(const int& speed, const double& distance);
			// For MovingToFrontOfHuman
		const bool movingToAroundOfHuman(const int& speed, const float& distance, const double& angle2FrontOfHuman);

			// For making a sound
		const bool makeSounds(const string& pathToAudioFile);
			// For robot speaking
		const bool speaking(const string& textToSpeak, const float& voiceVolume);

		/* Setting Set */
		const int setTimeout(const int& second) {timeout = second; return 1;}
		const int setKeywordListened();
		const int resetKeyword(const string& str) { keywordListened = str; return 1; }

		/* Getting Set */
		const int getTimeout() const { return timeout; }
		const int getATR() const { return curAttentionLevel; }
		const int getFaceDirection() const { return curFaceDirection; }
		const string getListenContent() { return keywordListened; }

		/* For Human Attention Estimator */
			// Query current attention toward the robot
		const bool sensingATR(const int& waitingTime);
			// Query current face direction
		const bool sensingFD(const int& waitingTime);

	private:
		/* For IPC Communication */
		void subcribeAndPublish();

		/* Human Speech Input */
		string keywordListened;

		/* Manipulating the timeout of component */
			// Time (s) for waiting the action finish
		int timeout;
			// For calculating executing time
		time_t action_start, action_current;
			// Get current time
		time_t getCurrentTime() { action_start = time(NULL); return action_start; }
			// Compute the executing time
		int executingTime(const time_t& action_start) {
			action_current = time(NULL);
			return static_cast< int >(difftime(action_current, action_start));
		}
		int executingTime() { return executingTime(action_start); }

		/* For Human Attention Estimator */
			// Current Attention Level
		int curAttentionLevel, curFaceDirection;
			// For buzy waiting for mgr receive
		bool buzyWaitForMgr(const int& delayTime);

		/* For Robot Pose Manipulation */

};

#endif