
/* Standrad Included Library */
#include <iostream>
#include <sstream>
#include <vector>
#include <ctime>
using namespace std;

/* Third-party Library */
	// OpenCV2
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
//	// Import Inter-Process Communication Server
//#include "IPCserver\Client.hpp"
	// Object for manipulating face detection
#include "FaceDetection\FaceDetection.h"
	// LCM core
#include "lcm\lcm-cpp.hpp"
	// LCM message data type
#include "lcm\FaceDetectionLcm.hpp"
	// LCM shared consts
#include "lcm\LcmComm.hpp"

using namespace cv;
using namespace lcm;

//** Problem Dependent Variable Setting **//
#define RECORDING
#define IMG_WIDTH	640
#define IMG_HEIGHT	480
#define BODYMINSIZE 101
#define FACEMINSIZE 20
#define EYEMINSIZE  1
#define BODYMAXSIZE 480
#define FACEMAXSIZE 300
#define EYEMAXSIZE  30
	// For front view face detection
//String facefront_cascade_name = "../models/CascadeClassifiers/haarcascade_frontalface_alt.xml";
//String facefront_cascade_name = "../models/CascadeClassifiers/haarcascade_frontalface_alt_tree.xml";
String facefront_cascade_name = "../models/CascadeClassifiers/haarcascade_frontalface_alt2.xml";
//String facefront_cascade_name = "../models/CascadeClassifiers/haarcascade_frontalface_default.xml";
//String facefront_cascade_name = "../models/CascadeClassifiers/lbpcascade_frontalface.xml";
	// For profile view face detection
String faceProfile_cascade_name = "../models/CascadeClassifiers/haarcascade_profileface.xml";
//String faceProfile_cascade_name = "../models/CascadeClassifiers/lbpcascade_profileface.xml";
	// For upper body front or back view detection
//String upperbody_cascade_name = "../models/CascadeClassifiers/haarcascade_upperbody.xml";
String upperbody_cascade_name = "../models/CascadeClassifiers/haarcascade_mcs_upperbody.xml";
	// For eyes detection
//String eyes_cascade_name = "../models/CascadeClassifiers/haarcascade_eye.xml";
String eyes_cascade_name = "../models/CascadeClassifiers/haarcascade_eye_tree_eyeglasses.xml";

/** Declration of Variables **/
String window_name = "Capture - Face detection";
Mat frame;
static int faceDirectionHAE = 0;
int imageIndex = 0;

/** Declration of Functions **/
	// Handler for receiving Request HAE message
void Perception_HAE_handler();

//=============================================================================
int main( void )
{
	/* Initialize LCM */
	LCM lcm(LCM_CTOR_PARAMS);
	if (!lcm.good())
	{
		cout << "> ERROR: Cannot initialize LCM" << endl;
		return 1;
	}
	
	/* Initialize and load the cascades */
	FaceDetection faceDetector(facefront_cascade_name, faceProfile_cascade_name, upperbody_cascade_name,
							   BODYMINSIZE, FACEMINSIZE);

	/* Read the video stream */
		// Open the default camera
	VideoCapture capture(0);
	if ( ! capture.isOpened() ) { printf("> ERROR: In opening video capture\n"); getchar(); return -1; }

	capture.set(CV_CAP_PROP_FRAME_WIDTH, IMG_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
	double dWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);	//get the width of frames of the video
	double dHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);	//get the height of frames of the video

	cout << "> Frame size : " << dWidth << " x " << dHeight << endl;
	cout << "> Press ESC to exit..." << endl;

		// To calculate frame rate
	time_t start, end;
	time(&start);
	int counter = 0;

	while ( capture.read(frame) )
	{
		if( frame.empty() ) {
			printf("> WARNING: No captured frame -- Break!");
			break;
		}

		/* Apply the classifier to the frame */
		faceDetector.detectFaceAndDirection(frame);

		/* Draw the result, face and body detection */
		faceDetector.drawAllResult(frame);

		/* For Face Direction */
		faceDetector.calculateFaceDirection();
		faceDirectionHAE = faceDetector.getFaceDirection();

		/* Show results */
			// Calculate and display frame rate in image
		time(&end); counter++;
		double fps = static_cast< double >(counter) / difftime(end, start);
		stringstream ss; string fpsStr;
		ss << fps; ss >> fpsStr;
		putText(frame, "FPS: " + fpsStr, Point(0,15), CV_FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1);
		imshow( window_name, frame );

		/* Send message through LCM */
		FaceDetectionLcm faceMgr;
		faceMgr.face_direction = faceDirectionHAE;
		lcm.publish(FACE_DETECTION, &faceMgr);

		int c = waitKey(5);
		if( (char)c == 27 ) { break; } // escape
	}
	return 0;
}

//=============================================================================
void Perception_HAE_handler()
{
//	HAEMgr percetData;
//	getHAE(percetData);
//
//	percetData.face_direction = static_cast< Face_Direction_HAE_type >(faceDirectionHAE);
//		// Send FaceDirectionHAE
//	sendHAE(percetData);
//	Sleep(sizeof(percetData));
//	printf("\n> Send Success! (FaceDirection: %d)\n", faceDirectionHAE);
//
//#ifdef RECORDING
//	/* Store the image for annotating data */
//	cv::Mat img;
//	if (frame.empty() == true)
//		return;
//	cv::resize(frame, img, cv::Size(frame.cols / 2, frame.rows / 2));
//	
//	stringstream ss;
//	string imgNumber;
//	ss << imageIndex++;
//	ss >> imgNumber;
//	
//	cv::imwrite("../models/DBN_Model/TrainingData/raw_" + imgNumber + ".png", img);
//#endif
//
//	return;
}