
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
	// Import Inter-Process Communication Server
#include "IPCserver\Client.hpp"
	// Object for manipulating face detection
#include "FaceDetection\FaceDetection.h"
using namespace cv;

//** Problem Dependent Variable Setting **//
//#define RECORDING
#define IMG_WIDTH	320
#define IMG_HEIGHT	240
#define BODYMINSIZE 90
#define FACEMINSIZE 60
#define IPCSERVER "localhost"		// Local host
//#define IPCSERVER "192.168.11.4"	// Old mac
	// For front view face detection
//String facefront_cascade_name = "../models/CascadeClassifiers/haarcascade_frontalface_alt.xml";
//String facefront_cascade_name = "../models/CascadeClassifiers/haarcascade_frontalface_alt_tree.xml";
//String facefront_cascade_name = "../models/CascadeClassifiers/haarcascade_frontalface_alt2.xml";
//String facefront_cascade_name = "../models/CascadeClassifiers/haarcascade_frontalface_default.xml";
String facefront_cascade_name = "../models/CascadeClassifiers/lbpcascade_frontalface.xml";
	// For profile view face detection
String faceProfile_cascade_name = "../models/CascadeClassifiers/haarcascade_profileface.xml";
//String faceProfile_cascade_name = "../models/CascadeClassifiers/lbpcascade_profileface.xml";
	// For eyes detection
//String eyes_cascade_name = "../models/CascadeClassifiers/haarcascade_eye_tree_eyeglasses.xml";
	// For upper body front or back view detection
//String upperbody_cascade_name = "../models/CascadeClassifiers/haarcascade_upperbody.xml";
String upperbody_cascade_name = "../models/CascadeClassifiers/haarcascade_mcs_upperbody.xml";

/** Declration of Variables **/
String window_name = "Capture - Face detection";
Mat frame;
static int faceDirectionHAE = 0;
int imageIndex = 0;

/** Declration of Functions **/
//	// Multi stage classification: Upperbody->FrontalFace->ProfileFace
//void detectAndDisplay(Mat& frame, CascadeClassifier& frontalFaceClassifier, CascadeClassifier& profileFaceClassifier, CascadeClassifier& upperBodyClassifier,
//								  vector< Rect >& resultFFace, vector< Rect >& resultPFace, vector< Rect >& resultBody);
//	// Draw the result on the image frame
//void drawImg(Mat& frame, const vector< Rect >& resultFFace, const vector< Rect >& resultPFace, const vector< Rect >& resultBody);
//	// Calculate Face Direction Discrete for HAE
//void calculateFaceDirectionDiscrete(const vector< Rect >& reslutFFace, const vector< Rect >& resultPFace);
	// Handler for receiving Request HAE message
void Perception_HAE_handler();

//=============================================================================
int main( void )
{
	/** Connect to IPC Server **/
	init_comm();
	connect_to_server(IPCSERVER);
	subscribe(PERCEPTION_HAE,TOTAL_MSG_NUM);
	publish(HAE, TOTAL_MSG_NUM);
	listen();
	
	/* Initialize and load the cascades */
	FaceDetection faceDetector(facefront_cascade_name, faceProfile_cascade_name, upperbody_cascade_name, BODYMINSIZE, FACEMINSIZE);
	//CascadeClassifier frontalFace_cascade, profileFace_cascade, upperBody_cascade;
	//if( !frontalFace_cascade.load( facefront_cascade_name ) ){ printf("> ERROR: loading frontal face cascade\n"); getchar(); return -1; };
	//if( !profileFace_cascade.load( faceProfile_cascade_name ) ){ printf("> ERROR: loading profile face cascade\n"); getchar(); return -1; };
	//if( !upperBody_cascade.load( upperbody_cascade_name ) ){ printf("> ERROR: loading upper body cascade\n"); getchar(); return -1; };
	//vector< Rect > resultFFace, resultPFace, resultBody;

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
		//detectAndDisplay(frame, frontalFace_cascade, profileFace_cascade, upperBody_cascade, resultFFace, resultPFace, resultBody);

		/* Draw the result, face and body detection */
		faceDetector.drawAllResult(frame);
		//drawImg(frame, resultFFace, resultPFace, resultBody);

		/* For Face Direction */
		faceDetector.calculateFaceDirection();
		faceDirectionHAE = faceDetector.getFaceDirection();
		//calculateFaceDirectionDiscrete(resultFFace, resultPFace);

		/* Show results */
			// Calculate and display frame rate in image
		time(&end); counter++;
		double fps = static_cast< double >(counter) / difftime(end, start);
		stringstream ss; string fpsStr;
		ss << fps; ss >> fpsStr;
		putText(frame, "FPS: " + fpsStr, Point(0,15), CV_FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1);
		imshow( window_name, frame );

		int c = waitKey(10);
		if( (char)c == 27 ) { break; } // escape
	}
	return 0;
}

//=============================================================================
	// Multi stage classification: Upperbody->FrontalFace->ProfileFace
//void detectAndDisplay(Mat& frame, CascadeClassifier& frontalFaceClassifier, CascadeClassifier& profileFaceClassifier, CascadeClassifier& upperBodyClassifier,
//								  vector< Rect >& resultFFace, vector< Rect >& resultPFace, vector< Rect >& resultBody) {
//	resultFFace.clear(); resultFFace.resize(0);
//	resultPFace.clear(); resultPFace.resize(0);
//	resultBody.clear();  resultBody.resize(0);
//	Mat frame_gray;
//
//	cvtColor (frame, frame_gray, COLOR_BGR2GRAY);
//	equalizeHist(frame_gray, frame_gray);
//
//	/* Detect Upper Human Body */
//	upperBodyClassifier.detectMultiScale(frame_gray, resultBody, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(90, 90));
//
//	for (auto it = resultBody.begin(); it != resultBody.end(); it++) {
//		Mat humanROI = frame_gray(*it);
//
//		/* Detect Frontal Face */
//		frontalFaceClassifier.detectMultiScale(humanROI, resultFFace, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30));
//
//		if (resultFFace.size() != 0) {
//				// Offset back image pixels
//			for (auto itFace = resultFFace.begin(); itFace != resultFFace.end(); itFace++) {
//				itFace->x += it->x;
//				itFace->y += it->y;
//			}
//			continue;
//		}
//
//		/* Detect Profile Face */
//		profileFaceClassifier.detectMultiScale(humanROI, resultPFace, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30));
//
//		if (resultPFace.size() == 0) {
//			flip(humanROI, humanROI, 1);
//			profileFaceClassifier.detectMultiScale(humanROI, resultPFace, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30));
//		}
//			// Offset backe image pixels
//		for (auto itFace = resultPFace.begin(); itFace != resultPFace.end(); itFace++) {
//			itFace->x += it->x;
//			itFace->y += it->y;
//		}
//	}
//}
//
//void drawImg(Mat& frame, const vector< Rect >& resultFFace, const vector< Rect >& resultPFace, const vector< Rect >& resultBody) {
//	/* Draw bodies we got */
//	if (resultBody.size() != 0) {
//		for (auto itBody = resultBody.begin(); itBody != resultBody.end(); itBody++) {
//			rectangle(frame, *itBody, Scalar(176, 104, 10), 2, 8, 0);
//			putText(frame, "Upperbody", Point(itBody->x + itBody->width - 80, itBody->y + itBody->height + 15), CV_FONT_HERSHEY_PLAIN, 1, Scalar(176, 104, 10), 2);
//		}
//	} else
//		return;
//	
//	/* Draw front face we got */
//	if (resultFFace.size() != 0)
//		for (auto itFFace = resultFFace.begin(); itFFace != resultFFace.end(); itFFace++) {
//			Point center( itFFace->x + itFFace->width/2, itFFace->y + itFFace->height/2 );
//			ellipse( frame, center, Size( itFFace->width/2, itFFace->height/2 ), 0, 0, 360, Scalar(109, 55, 255), 4, 8, 0 );
//			putText(frame, "FrontalFace", Point(itFFace->x + itFFace->width - 90, itFFace->y + itFFace->height + 15), CV_FONT_HERSHEY_PLAIN, 1, Scalar(109, 55, 255), 2);
//			return;
//		}
//
//	/* Draw front face we got */
//	if (resultPFace.size() != 0)
//		for (auto itPFace = resultPFace.begin(); itPFace != resultPFace.end(); itPFace++) {
//			Point center( itPFace->x + itPFace->width/2, itPFace->y + itPFace->height/2 );
//			ellipse( frame, center, Size( itPFace->width/2, itPFace->height/2 ), 0, 0, 360, Scalar(79, 132, 225), 4, 8, 0 );
//			putText(frame, "ProfileFace", Point(itPFace->x + itPFace->width - 90, itPFace->y + itPFace->height + 15), CV_FONT_HERSHEY_PLAIN, 1, Scalar(79, 132, 225), 2);
//		}
//}
//
//void calculateFaceDirectionDiscrete(const vector< Rect >& resultFFace, const vector< Rect >& resultPFace) {
//	if (resultFFace.size() != 0)
//		faceDirectionHAE = 2;
//	else if (resultPFace.size() != 0)
//		faceDirectionHAE = 1;
//	else
//		faceDirectionHAE = 0;
//
//		// Print information
//	cout << "> Face Direction: " << faceDirectionHAE << endl;
//}

void Perception_HAE_handler()
{
	HAEMgr percetData;
	getHAE(percetData);

	percetData.face_direction = static_cast< Face_Direction_HAE_type >(faceDirectionHAE);
		// Send FaceDirectionHAE
	sendHAE(percetData);
	Sleep(sizeof(percetData));
	printf("\n> Send Success! (FaceDirection: %d)\n", faceDirectionHAE);

#ifdef RECORDING
	/* Store the image for annotating data */
	cv::Mat img;
	//cv::resize(frame, img, cv::Size(frame.cols / 2, frame.rows / 2));
	
	stringstream ss;
	string imgNumber;
	ss << imageIndex++;
	ss >> imgNumber;
	
	cv::imwrite("../models/DBN_Model/TrainingData/raw" + imgNumber + ".png", img);
#endif

	return;
}