
#ifndef BODYDIRECTION_H
#define BODYDIRECTION_H

#include "NiImageCaptor.h"

class BodyDirection : public NiImageCaptor
{

public:
		// Initilization
	const int startGetBodyDirection();
	
	const int analyzeFunc();

	const vector<double> getBodyAngle(){return userBodyAngle;};

	/* For F-Formation Recognition */
	const int calculateBodyFformation();
	const int getBodyFformation(){return bodyFformation;};

	/* For Human Attention Estimator */
	const int calculateBodyDirectionToCamera();
	const int getBodyDirectionHAE() {return bodyDirectionHAE;};

	const float getBodyDirectionCont() {return bodyDirectionToCamera;};

private:

	float bodyDirectionToCamera;

	const float point3fDist(const nite::Point3f a, const nite::Point3f b);

	vector<vector<double>> thetaData;		//previous 10 frame body direction for smoothing
	vector<double> userBodyAngle;			// user body orientation
	vector<double> userLeanAngle;			// user body lean foward angle
	vector<double> userHandSpeed;			// user both and speed avg for detecting body language
	vector<nite::Point3f> leftHandTemp;		// for counting speed
	vector<nite::Point3f> rightHandTemp;	// for counting speed

	vector<int> userTouchFlag;				// only detect for user 1 and 2

	/* For F-Formation Recognition */
	int bodyFformation;

	/* For Human Attention Estimator */
	int bodyDirectionHAE;
};


#endif