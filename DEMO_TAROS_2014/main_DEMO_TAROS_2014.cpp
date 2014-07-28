
/* Standrad Included Library */
#include <iostream>
#include <cstdio>
#include <string>
using namespace std;

/* Third-party Library */
#include "RobotAction/RobotAction.h"

//** Problem Dependent Variable Setting **//
	// The name of the audience, default is the hello
string userName = "«¢¼Ó";

/* For IPC handler using */
bool isUserNameReceived = false;

//=============================================================================
int main(int argc, char* argv[]) {
	RobotAction robot;

	/* Session: Online Face Recognition */

	/* Session: Attracting user's attention */

	/* Session: Grasping */
		// Turn to the cabinet
	robot.toPoint(0.0, 0.0, -90.0);
		// Grasping!
	robot.graspingNil(0.0, 0.0, 0.0);

	return 0;
}
