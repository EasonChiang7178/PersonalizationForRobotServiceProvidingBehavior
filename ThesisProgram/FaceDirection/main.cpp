///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2010, Jason Mora Saragih, all rights reserved.
//
// This file is part of FaceTracker.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * The software is provided under the terms of this licence stricly for
//       academic, non-commercial, not-for-profit purposes.
//     * Redistributions of source code must retain the above copyright notice,
//       this list of conditions (licence) and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions (licence) and the following disclaimer
//       in the documentation and/or other materials provided with the
//       distribution.
//     * The name of the author may not be used to endorse or promote products
//       derived from this software without specific prior written permission.
//     * As this software depends on other libraries, the user must adhere to
//       and keep in place any licencing terms of those libraries.
//     * Any publications arising from the use of this software, including but
//       not limited to academic journal and conference publications, technical
//       reports and manuals, must cite the following work:
//
//       J. M. Saragih, S. Lucey, and J. F. Cohn. Face Alignment through
//       Subspace Constrained Mean-Shifts. International Conference of Computer
//       Vision (ICCV), September, 2009.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
// EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

#include <sstream>
#include <iostream>
#include <vector>
#include <string>
#include <time.h>
#include <cmath>
#include <OpenNI.h>
#include <opencv/highgui.h>
#include <Tracker.h>
	// Import Inter-Process Communication Server
#include "IPCserver\Client.hpp"
using namespace std;

#define RECORDING

#define M_PI 3.1415926
//#define DEBUG

int parse_cmd(int argc, const char** argv, char* ftFile,char* conFile,char* triFile, bool &fcheck,double &scale,int &fpd);	// parsing commend argument
int findFaceDirection(cv::Mat &image,cv::Mat &shape ,cv::Mat &visi, string &faceData, cv::Point2f &nosePoint);										// function for finding face direction
void loadWithPoints(cv::Mat& ip, cv::Mat& img, string &faceData, cv::Point2f &nosePoint);										// load points to solve pnp
void resquestHandler(string data);

#ifdef DEBUG
void Draw(cv::Mat &image,cv::Mat &shape,cv::Mat &con,cv::Mat &tri,cv::Mat &visi);						// drawing face alignment for debug
#endif

static int faceDirectionHAE = 0;

cv::Mat im;
#ifdef RECORDING
	// For store image
int savedNumber = 0;
#endif

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
	cv::resize(im, img, cv::Size(im.cols / 2, im.rows / 2));
	
	stringstream ss;
	string imgNumber;
	ss << savedNumber++;
	ss >> imgNumber;
	
	cv::imwrite("../models/DBN_Model/TrainingData/raw" + imgNumber + ".png", img);
#endif

	return;
}

//=============================================================================
int main(int argc, const char** argv)
{
	/** Connect to IPC Server **/
	init_comm();
	connect_to_server();
	subscribe(PERCEPTION_HAE, TOTAL_MSG_NUM);
	publish(HAE, TOTAL_MSG_NUM);
	listen();

	/*	Create Another Thread for Response Demend	*/
	//pthread_t responseThread;
	string faceData;		// sending data
	//pthread_create( &responseThread, NULL, (void*) &resquestHandler, (void*) &faceData );

	cv::Point2f nosePoint;
	vector< double > historyData;

	using namespace std;
	// ---------------------------- //
	// Initialize Device and OpenNI //
	// ---------------------------- //
	openni::Device device;
	
	openni::OpenNI::initialize();
	cout << "Initialization..." << openni::OpenNI::getExtendedError() << endl;

	const char* deviceURI = openni::ANY_DEVICE;
	if (device.open(deviceURI) != openni::STATUS_OK)
	{
		cout << "Device open failed:" << openni::OpenNI::getExtendedError() << endl;
		openni::OpenNI::shutdown();
		return 1;
	}

	/*	Create Depth and Color Stream	*/
	openni::VideoStream depth, color;

	if (depth.create(device, openni::SENSOR_DEPTH) != openni::STATUS_OK)
	{
		cout << "Couldn't find depth stream:" << openni::OpenNI::getExtendedError() << endl;
		return 1;
	}

	if (color.create(device, openni::SENSOR_COLOR) != openni::STATUS_OK)
	{
		cout << "Couldn't find color stream:" << openni::OpenNI::getExtendedError() << endl;
		return 1;
	}

	/*	Set the Video Stream	*/
		// For depth stream
	openni::VideoMode mModeDepth;
	mModeDepth.setResolution(640, 480);
	mModeDepth.setFps(30);
	mModeDepth.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
	if (depth.setVideoMode(mModeDepth) != openni::STATUS_OK)
	{
		cout << "Couldn't apply the setting of depth stream:" << openni::OpenNI::getExtendedError() <<endl;
		return 1;
	}
		// For color stream
	openni::VideoMode mModeColor;
	mModeColor.setResolution( 640, 480 );
	mModeColor.setFps( 30 );
	mModeColor.setPixelFormat( openni::PIXEL_FORMAT_RGB888 );
	if(color.setVideoMode(mModeColor)!=openni::STATUS_OK)
	{
		cout << "Couldn't apply the setting of color stream" << endl;
		return 1;
	}

	/*	Calibration the Depth Image to Color Image	*/
	if (device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
	{
		if (device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR) == openni::STATUS_OK)
			cout << "Calibrate depth image to color image..." << endl;
		else
			cout << "Set calibration failed:" << openni::OpenNI::getExtendedError() <<endl;
	}
	else
	{
		cout << "Not support calibrate depth image to color image..." << endl;
	}

	// ------------- //
	// Start Running //
	// ------------- //
	if (depth.start() != openni::STATUS_OK)
	{
		cout << "Couldn't start depth stream:" << openni::OpenNI::getExtendedError() <<endl;
		depth.destroy();
	}
	if (color.start() != openni::STATUS_OK)
	{
		cout << "Couldn't start color stream:" << openni::OpenNI::getExtendedError() <<endl;
		color.destroy();
	}
	if (!depth.isValid() || !color.isValid())
	{
		cout << "No valid streams exiting..." << endl;
		openni::OpenNI::shutdown();
		return 2;
	}

	/*	Capture Frame of Vedio Stream 	*/
	openni::VideoFrameRef depthFrame, colorFrame;

	char ftFile[256], conFile[256], triFile[256];
	bool fcheck = false;
	double scale = 1;
	int fpd = -1;
	bool show = true;
		// parse command line arguments
	if (parse_cmd(argc,argv,ftFile,conFile,triFile,fcheck,scale,fpd) < 0)
		return 0;

	std::vector<int> wSize1(1); wSize1[0] = 7;					// set other tracking parameters
	std::vector<int> wSize2(3); wSize2[0] = 11; wSize2[1] = 9; wSize2[2] = 7;
	int nIter = 5; double clamp = 3,fTol = 0.01;
	FACETRACKER::Tracker model(ftFile);
	cv::Mat tri = FACETRACKER::IO::LoadTri(triFile);
	cv::Mat con = FACETRACKER::IO::LoadCon(conFile);

	/*	Initialize Camera and Display Window	*/
	cv::Mat frame,gray; double fps=0; char sss[256]; std::string text;
	//CvCapture* camera = cvCreateCameraCapture(CV_CAP_ANY); if(!camera)return -1;	// capture from camera
	//CvCapture* camera = cvCaptureFromFile("out.mpg"); if(!camera)return -1;	// capture from vedio file
	int64 t1,t0 = cvGetTickCount(); int fnum = 0;
	cvNamedWindow("Face Tracker",1);
	std::cout << "Hot keys: "        << std::endl
			  << "\t ESC - quit"     << std::endl
			  << "\t d   - Redetect" << std::endl;

	//loop until quit (i.e user presses ESC)
	bool failed = true;
	time_t timeBegin, timeEnd;
	timeBegin = time(NULL);
	timeEnd = time(NULL);
	
	while(1) {
		/* Read next frame */
		depth.readFrame( &depthFrame );
		color.readFrame( &colorFrame );

		/* Preprocess and show the current depth image */
		cv::Mat mImageDepth( depthFrame.getHeight(), depthFrame.getWidth(), CV_16UC1, (void*)depthFrame.getData());
		//cv::Mat mScaledDepth;
		//mImageDepth.convertTo( mScaledDepth, CV_8U, 255.0 / depth.getMaxPixelValue());
		//cv::imshow( "Depth Image", mScaledDepth );
		

		/* Preprocess and show the current color image */
		const cv::Mat mImageRGB(colorFrame.getHeight(), colorFrame.getWidth(), CV_8UC3, (void*)colorFrame.getData());
		cv::Mat cImageBGR;
		cv::cvtColor( mImageRGB, cImageBGR, CV_RGB2BGR );
		//cv::imshow( "Color Image", cImageBGR );


		//grab image, resize and flip
		//IplImage* I = cvQueryFrame(camera); if(!I)continue; frame = I;

		frame = cImageBGR;
		if(scale == 1)
			im = frame;
		else
			cv::resize(frame,im,cv::Size(scale*frame.cols,scale*frame.rows));
		cv::flip(im,im,1);
		cv::cvtColor(im,gray,CV_BGR2GRAY);

		/*	Track This Image	*/
		std::vector<int> wSize;
		if (failed)
			wSize = wSize2;
		else
			wSize = wSize1;
		if (model.Track(gray, wSize, fpd, nIter, clamp, fTol, fcheck) == 0) {
			int idx = model._clm.GetViewIdx(); failed = false;
			#ifdef DEBUG
			Draw(im,model._shape,con,tri,model._clm._visi[idx]);
			#endif
			findFaceDirection(im, model._shape, model._clm._visi[idx], faceData, nosePoint);	// find face direction

			/* Caculate FaceDirectionDiscrete */
			stringstream ss;
			double beta = 0.0;
			ss << faceData.substr(faceData.find_last_of(" ") + 1);
			ss >> beta;

			if (nosePoint.x < 320.0)
				beta = -(beta - 90.0);
			else
				beta = beta + 90;
			//cout << beta << endl;
			cv::flip(mImageDepth,mImageDepth,1);
			if (nosePoint.y > 479.0) nosePoint.y = 479;
			if (nosePoint.x > 639.0) nosePoint.x = 639;
			unsigned short nosePointZ = mImageDepth.at< unsigned short >(nosePoint.y, nosePoint.x);
			double angleCamToUser = acos(nosePointZ / (sqrt(pow((float)nosePoint.x - 320,2) + pow((float)nosePointZ,2)))) * 180.0 / M_PI;

			/* The outcome of FaceDirectionDiscrete */
			if (angleCamToUser < 89.0) {
				double focusToCamera = fabs(90 -angleCamToUser - beta);

				/* Gaussian Smoothing */
					//maintain history data
				historyData.push_back(focusToCamera);
				if (historyData.size() > 7)
					historyData.erase(historyData.begin());

					//smooth data gaussian blur version
				double g_mask[7] = {0.004429, 0.053994, 0.242038, 0.399074, 0.242038, 0.053994, 0.004429};
				for (unsigned int j(0); j < historyData.size(); j++)
					focusToCamera += historyData[j] * g_mask[j];

				cout << "angleCamToUser: " << angleCamToUser << ", focusToCamera: " << focusToCamera << endl;

				//if (focusToCamera < 0.0) focusToCamera = -focusToCamera; // Absolute value
				if (focusToCamera > 16.0)
					faceDirectionHAE = 1;
				else if (focusToCamera > 8.0)
					faceDirectionHAE = 2;
				else
					faceDirectionHAE = 3;
				cout << "> faceDirection(D): " << faceDirectionHAE << endl;
			}
		} else {
			if (show) {
				cv::Mat R(im,cvRect(0,0,150,50));
				R = cv::Scalar(0,0,255);
			}
			model.FrameReset(); failed = true;
				// No people
			faceDirectionHAE = 0;
		}

		/*	Draw Framerate on Display Image	*/
		if (fnum >= 9) {
			t1 = cvGetTickCount();
			fps = 10.0 / ((double(t1-t0) / cvGetTickFrequency()) / 1e+6);
			t0 = t1; fnum = 0;
		} else 
			fnum += 1;
		if (show) {
			sprintf(sss,"%d frames/sec",(int)floor(fps)); text = sss;
			cv::putText(im,text,cv::Point(10,20),
			CV_FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,255));
		}

		/*	Show Image and Check for Rser Input	*/
		imshow("Face Tracker",im);
		int c = cvWaitKey(10);
		if(char(c) == 'q')
			break; 
		else if(char(c) == 'd')
			model.FrameReset();
		else
		{
			timeEnd = time(NULL);
			if(timeEnd-timeBegin>3)
			{
				model.FrameReset();
				timeBegin = time(NULL);
			}
		}	
		//usleep(100000);	// release resource for others work
	}

	disconnect_to_server();
	return 0;
}

//=============================================================================
int parse_cmd(int argc, const char** argv,
		  char* ftFile,char* conFile,char* triFile,
		  bool &fcheck,double &scale,int &fpd)
{
	int i; fcheck = false; scale = 1; fpd = -1;
	for(i = 1; i < argc; i++){
	if((std::strcmp(argv[i],"-?") == 0) ||
	(std::strcmp(argv[i],"--help") == 0)){
		std::cout << "track_face:- Written by Jason Saragih 2010" << std::endl
		   << "Performs automatic face tracking" << std::endl << std::endl
		   << "#" << std::endl
		   << "# usage: ./face_tracker [options]" << std::endl
		   << "#" << std::endl << std::endl
		   << "Arguments:" << std::endl
		   << "-m <string> -> Tracker model (default: model/face2.tracker)"
		   << std::endl
		   << "-c <string> -> Connectivity (default: model/face.con)"
		   << std::endl
		   << "-t <string> -> Triangulation (default: model/face.tri)"
		   << std::endl
		   << "-s <double> -> Image scaling (default: 1)" << std::endl
		   << "-d <int>    -> Frames/detections (default: -1)" << std::endl
		   << "--check     -> Check for failure" << std::endl;
		return -1;
		}
	}
	for(i = 1; i < argc; i++){
		if(std::strcmp(argv[i],"--check") == 0){fcheck = true; break;}
		}
		if(i >= argc)fcheck = false;
		for(i = 1; i < argc; i++){
		if(std::strcmp(argv[i],"-s") == 0){
		if(argc > i+1)scale = std::atof(argv[i+1]); else scale = 1;
		break;
		}
	}
	if(i >= argc)scale = 1;
		for(i = 1; i < argc; i++){
		if(std::strcmp(argv[i],"-d") == 0){
		if(argc > i+1)fpd = std::atoi(argv[i+1]); else fpd = -1;
		break;
		}
	}
	if(i >= argc)fpd = -1;
		for(i = 1; i < argc; i++){
		if(std::strcmp(argv[i],"-m") == 0){
		if(argc > i+1)std::strcpy(ftFile,argv[i+1]);
		else strcpy(ftFile,"model/face2.tracker");
		break;
		}
	}
	if(i >= argc)std::strcpy(ftFile,"model/face2.tracker");
		for(i = 1; i < argc; i++){
		if(std::strcmp(argv[i],"-c") == 0){
		if(argc > i+1)std::strcpy(conFile,argv[i+1]);
		else strcpy(conFile,"model/face.con");
		break;
		}
	}
	if(i >= argc)std::strcpy(conFile,"model/face.con");
		for(i = 1; i < argc; i++){
		if(std::strcmp(argv[i],"-t") == 0){
		if(argc > i+1)std::strcpy(triFile,argv[i+1]);
		else strcpy(triFile,"model/face.tri");
		break;
		}
	}
	if(i >= argc)std::strcpy(triFile,"model/face.tri");
	return 0;
}

//=============================================================================
int findFaceDirection(cv::Mat &image,cv::Mat &shape ,cv::Mat &visi, string &faceData, cv::Point2f &nosePoint)
{
	std::vector<cv::Point2f> imagePoints;
	int i,n = shape.rows/2; cv::Point p1,p2; cv::Scalar c;

	/*	Parsing 7 Points for Finding Face Direction	*/
	imagePoints.resize(7);
	if(visi.at<int>(37,0) == 0)
		imagePoints[0] = cv::Point2f(0.0, 0.0);
	else
		imagePoints[0] = cv::Point2f(shape.at<double>(37,0),shape.at<double>(37+n,0));

	if(visi.at<int>(44,0) == 0)
		imagePoints[1] = cv::Point2f(0.0, 0.0);
	else
		imagePoints[1] = cv::Point2f(shape.at<double>(44,0),shape.at<double>(44+n,0));

	if(visi.at<int>(30,0) == 0)
		imagePoints[2] = cv::Point2f(0.0, 0.0);
	else
		imagePoints[2] = cv::Point2f(shape.at<double>(30,0),shape.at<double>(30+n,0));

	if(visi.at<int>(48,0) == 0)
		imagePoints[3] = cv::Point2f(0.0, 0.0);
	else
		imagePoints[3] = cv::Point2f(shape.at<double>(48,0),shape.at<double>(48+n,0));

	if(visi.at<int>(54,0) == 0)
		imagePoints[4] = cv::Point2f(0.0, 0.0);
	else
		imagePoints[4] = cv::Point2f(shape.at<double>(54,0),shape.at<double>(54+n,0));

	if(visi.at<int>(0,0) == 0)
		imagePoints[5] = cv::Point2f(0.0, 0.0);
	else
		imagePoints[5] = cv::Point2f(shape.at<double>(0,0),shape.at<double>(0+n,0));

	if(visi.at<int>(16,0) == 0)
		imagePoints[6] = cv::Point2f(0.0, 0.0);
	else
		imagePoints[6] = cv::Point2f(shape.at<double>(16,0),shape.at<double>(16+n,0));

	cv::Mat ip(imagePoints);
	loadWithPoints( ip, image, faceData, nosePoint);	// using this function to solve pnp (HeadPose.cc)

	return 0;
}

//=============================================================================
//void resquestHandler(string data)
//{
//	//  Socket to talk to clients
//	void *context = zmq_ctx_new ();
//	void *responder = zmq_socket (context, ZMQ_REP);
//	int rc = zmq_bind (responder, "tcp://*:5500");
//	assert (rc == 0);
//
//	while(1)
//	{
//		char buffer[1024];
//		zmq_recv (responder, buffer, 10, 0);
//		printf ("Received Request %s \n", buffer);
//		strcpy(buffer,data.c_str());
//		zmq_send (responder, buffer, 1024, 0);       
//		cout << data << endl;
//		usleep(100000);	//  Do some 'work'
//	}
//	return;
//}

//=============================================================================
#ifdef DEBUG
void Draw(cv::Mat &image,cv::Mat &shape,cv::Mat &con,cv::Mat &tri,cv::Mat &visi)
{
  int i,n = shape.rows/2; cv::Point p1,p2; cv::Scalar c;

  //draw triangulation
  /*c = CV_RGB(0,0,0);
  for(i = 0; i < tri.rows; i++){
	if(visi.at<int>(tri.at<int>(i,0),0) == 0 ||
	   visi.at<int>(tri.at<int>(i,1),0) == 0 ||
	   visi.at<int>(tri.at<int>(i,2),0) == 0)continue;
	p1 = cv::Point(shape.at<double>(tri.at<int>(i,0),0),
		   shape.at<double>(tri.at<int>(i,0)+n,0));
	p2 = cv::Point(shape.at<double>(tri.at<int>(i,1),0),
		   shape.at<double>(tri.at<int>(i,1)+n,0));
	cv::line(image,p1,p2,c);
	p1 = cv::Point(shape.at<double>(tri.at<int>(i,0),0),
		   shape.at<double>(tri.at<int>(i,0)+n,0));
	p2 = cv::Point(shape.at<double>(tri.at<int>(i,2),0),
		   shape.at<double>(tri.at<int>(i,2)+n,0));
	cv::line(image,p1,p2,c);
	p1 = cv::Point(shape.at<double>(tri.at<int>(i,2),0),
		   shape.at<double>(tri.at<int>(i,2)+n,0));
	p2 = cv::Point(shape.at<double>(tri.at<int>(i,1),0),
		   shape.at<double>(tri.at<int>(i,1)+n,0));
	cv::line(image,p1,p2,c);
  }*/
  //draw connections
  /*c = CV_RGB(0,0,255);
  for(i = 0; i < con.cols; i++){
	if(visi.at<int>(con.at<int>(0,i),0) == 0 ||
	   visi.at<int>(con.at<int>(1,i),0) == 0)continue;
	p1 = cv::Point(shape.at<double>(con.at<int>(0,i),0),
		   shape.at<double>(con.at<int>(0,i)+n,0));
	p2 = cv::Point(shape.at<double>(con.at<int>(1,i),0),
		   shape.at<double>(con.at<int>(1,i)+n,0));
	cv::line(image,p1,p2,c,1);
  }*/
  //draw points
  /*for(i = 0; i < n; i++){
	if(visi.at<int>(i,0) == 0)continue;
	p1 = cv::Point(shape.at<double>(i,0),shape.at<double>(i+n,0));
	c = CV_RGB(255,0,0); cv::circle(image,p1,2,c);
	//
	char sss[8]; std::string text;
	sprintf(sss,"%d",i); text = sss;
	cv::putText(image,text,p1,
		  CV_FONT_HERSHEY_SIMPLEX,0.3,CV_RGB(255,255,255));
  }*/
  return;
}
#endif

//=============================================================================


