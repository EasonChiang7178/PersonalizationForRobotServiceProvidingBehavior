
#ifndef BODYDIRECTION_H
#define BODYDIRECTION_H

#include "NiImageCaptor.h"
#include <ctime>
#include <conio.h>			// To use getch(), kbhit()

class BodyDirection : public NiImageCaptor
{

public:
		// Initilization
	const int startGetBodyDirection();
	
	const int analyzeFunc();

	const vector<double> getBodyAngle(){return userBodyAngle;};

		// Display the feature and image, for debug
	const int drawImg();

	/* For F-Formation Recognition */
	const int calculateBodyFformation();
	const int getBodyFformation(){return bodyFformation;};

	/* For Human Attention Estimator */
	const int calculateBodyDirectionToCamera(const int& userID, const NiImageStruct& niImageStruct);
	const vector< int > getBodyDirectionHAE() {return bodyDirectionHAE;};
	const vector< double > getBodyDirectionCont() {return bodyDirectionToCamera;};

	/* For Human's Affective State Classification */
		// HandToHead
	const int calculateHandToHead(const int& userID, const NiImageStruct& niImageStruct);
	const vector< double > getHandToHead() {return userHandToHead;}
		// ArmAsymmetry
	const int calculateArmAsymmetry(const int& userID, const NiImageStruct& niImageStruct);
	const vector< double > getArmAsymmetry() {return userArmAsymmetry;}
		// ArmAreaSpanned
	const int calculateArmAreaSpanned(const int& userID, const NiImageStruct& niImageStruct);
	const vector< double > getArmAreaSpanned() {return userArmAreaSpanned;}
		// HandSpeed
	const int calculateHandSpeed(const int& userID, const NiImageStruct& niImageStruct, const int& intervalMax);
	const vector< double > getHandSpeed() {return userHandSpeedMax;}
		// HandToBody
	const int calculateHandToBody(const int& userID, const NiImageStruct& niImageStruct);
	const vector< double > getHandToBody() {return userHandToBody;}
		// Pleasantness and Unpleasantness
	const int calculatePU(const int& userID, const double& w1, const double& w2, const double& w3, const double& w4, const double&  w5);
	const vector< double > getPU() {return userUnpleasantness;}

private:
	double fps;

		// Vector manipulation
	const float point3fDist(const nite::Point3f a, const nite::Point3f b);
	const float point3fLength(const nite::Point3f& a);
		// GaussianKernal
	float gaussianKernal3(float xDist, float yDist, float zDist, float sigma) {
		float dx = xDist / sigma;
		float dy = yDist / sigma;
		float dz = zDist / sigma;
		return std::exp(-float(0.5) * (dx * dx + dy * dy + dz * dz));
	}

	vector<vector<double>> thetaData;		// previous 7 frame body direction for smoothing
	vector<double> userBodyAngle;			// user body orientation
	vector<double> userLeanAngle;			// user body lean foward angle
	vector<double> userHandSpeed;			// user both and speed avg for detecting body language
	vector<nite::Point3f> leftHandTemp;		// for counting speed
	vector<nite::Point3f> rightHandTemp;	// for counting speed

	vector<int> userTouchFlag;				// only detect for user 1 and 2

	/* For F-Formation Recognition */
	int bodyFformation;

	/* For Human Attention Estimator */
	vector< int > bodyDirectionHAE;
		// For user body direction relative to the camera
	vector< double > bodyDirectionToCamera;

	/* For Human's Affective State Classification */
	vector< double > userHandToHead;
	vector< double > userArmAsymmetry;
	vector< double > userArmAreaSpanned;
	vector< double > userHandToBody;
	vector< double > userUnpleasantness;
		// For computing max hand speed over intervalMax
	vector< double > userHandSpeedMax;
	time_t startHS, endHS;

	double w1, w2, w3, w4, w5;
};


#endif