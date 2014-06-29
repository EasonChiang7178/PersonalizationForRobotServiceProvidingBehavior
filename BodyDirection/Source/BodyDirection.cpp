/* Third-party Library */
	// Import Inter-Process Communication Server
#include "IPCserver\Client.hpp"
#include "BodyDirection.h"

#include <cmath>
#include <sstream>

//#define _USE_MATH_DEFINES

#define M_PI       3.14159265358979323846

using namespace std;

const int BodyDirection::analyzeFunc()
{
	NiImageStruct niImageStruct = getNiImageStruct();
	cout << "user size: " << niImageStruct.Users.size() << endl;

	/* Initialize	*/
	vector<double> oneUserAngle;
	nite::Point3f point3D;
	double angle;
	//int flag;
		// For smoothing the body direction
	while(thetaData.size() < niImageStruct.Users.size())
		thetaData.push_back(oneUserAngle);
		// Body direction
	while(userBodyAngle.size() < niImageStruct.Users.size())
		userBodyAngle.push_back(angle);
		// Body leaning
	while(userLeanAngle.size() < niImageStruct.Users.size())
		userLeanAngle.push_back(angle);
		// Hand Speed of both two hands
	while(userHandSpeed.size() < niImageStruct.Users.size())
		userHandSpeed.push_back(angle);
		// For counting the hand speed
	while(leftHandTemp.size() < niImageStruct.Users.size())
		leftHandTemp.push_back(point3D);
	while(rightHandTemp.size() < niImageStruct.Users.size())
		rightHandTemp.push_back(point3D);
		// Touch flag
	//while(userTouchFlag.size() < niImageStruct.Users.size())
	//	userTouchFlag.push_back(flag);

//**** WARNING: Hard coding, check for if no user exists ****//
	if (niImageStruct.Users.size() == 0)
		bodyDirectionHAE = 0;

	/*	Counting features of users	*/
	for(unsigned int i(0); i<niImageStruct.Users.size(); i++)	// for all users
	{
		/*	Calculating Body Orientation Angle	*/
		nite::Point3f leftShoulder, rightShoulder;

		leftShoulder = niImageStruct.Users[i][2].position3D;
		rightShoulder = niImageStruct.Users[i][3].position3D;

		double xdiff, zdiff, theta;
		xdiff = rightShoulder.x - leftShoulder.x;
		zdiff = rightShoulder.z - leftShoulder.z;
		theta = atan2(xdiff,zdiff) * 180.0 / M_PI;

			//maintain history data
		thetaData[i].push_back(theta);
		if(thetaData[i].size() > 7)
			thetaData[i].erase(thetaData[i].begin());

			//smooth data gaussian blur version
		double g_mask[7] = {0.004429, 0.053994, 0.242038, 0.399074, 0.242038, 0.053994, 0.004429};
		double gm_theta = 0;
		for(unsigned int j(0); j<thetaData[i].size(); j++)
			gm_theta += thetaData[i][j]*g_mask[j];

		userBodyAngle[i] = fabs(gm_theta);
		//cout << "userBodyAngle:" << userBodyAngle[i] << endl;	// for debug output

		/* Calculating DirectionToCamera */
		calculateBodyDirectionToCamera();

		/*	Calculating Body Lean Angle	*/
		nite::Point3f vecOfRightShoudlerToLeftHip;
		nite::Point3f vecOfLeftShoudlerToRightHip;

		nite::Point3f leftHip = niImageStruct.Users[i][9].position3D;
		nite::Point3f rightHip = niImageStruct.Users[i][10].position3D;

		vecOfRightShoudlerToLeftHip.x = leftHip.x - rightShoulder.x;
		vecOfRightShoudlerToLeftHip.y = leftHip.y - rightShoulder.y;
		vecOfRightShoudlerToLeftHip.z = leftHip.z - rightShoulder.z;

		vecOfLeftShoudlerToRightHip.x = rightHip.x - leftShoulder.x;
		vecOfLeftShoudlerToRightHip.y = rightHip.y - leftShoulder.y;
		vecOfLeftShoudlerToRightHip.z = rightHip.z - leftShoulder.z;

		nite::Point3f vecOfCrossProduct;
		vecOfCrossProduct.x = vecOfRightShoudlerToLeftHip.y*vecOfLeftShoudlerToRightHip.z
							 -vecOfRightShoudlerToLeftHip.z*vecOfLeftShoudlerToRightHip.y;

		vecOfCrossProduct.y = vecOfRightShoudlerToLeftHip.z*vecOfLeftShoudlerToRightHip.x
							 -vecOfRightShoudlerToLeftHip.x*vecOfLeftShoudlerToRightHip.z;
		
		vecOfCrossProduct.z = vecOfRightShoudlerToLeftHip.x*vecOfLeftShoudlerToRightHip.y
							 -vecOfRightShoudlerToLeftHip.y*vecOfLeftShoudlerToRightHip.x;

		double lenghtOfCrossProduct = sqrt(	pow(vecOfCrossProduct.x,2)+
											pow(vecOfCrossProduct.y,2)+
											pow(vecOfCrossProduct.z,2));

		userLeanAngle[i] = asin(vecOfCrossProduct.y/lenghtOfCrossProduct) * 180.0 / M_PI;
		
		//cout << "userLeanAngle:" << userLeanAngle[i] << endl;	// debug  output

		/*	Calculating Hand Speed	*/
		nite::Point3f leftHand = niImageStruct.Users[i][6].position3D;
		nite::Point3f rightHand = niImageStruct.Users[i][7].position3D;

		float leftHandDisplacement;
		float rightHandDisplacement;

		leftHandDisplacement = point3fDist( leftHand, leftHandTemp[i]);
		rightHandDisplacement = point3fDist( rightHand, rightHandTemp[i]);

		userHandSpeed[i] = (leftHandDisplacement + rightHandDisplacement) / 2000.0; // avg in unit (m/s)

		leftHandTemp[i] = leftHand;
		rightHandTemp[i] = rightHand;

		//cout << "userHandSpeed:" << userHandSpeed[i] << endl;	// debug  output

		/*	Calculating Touch Flag	*/
		// check the dist between both hand to sholders and another person's hands 
		//float distThresh(300.0f);			// threshold is setting as 30cm = 300mm
		//if(niImageStruct.Users.size()>1)	// more than two people
		//{
		//	if(i==0)	// for user 0
		//	{
		//		if(	point3fDist( leftHand, niImageStruct.Users[1][6].position3D) < distThresh ||	// both hand to left hand
		//			point3fDist( rightHand, niImageStruct.Users[1][6].position3D) < distThresh )		
		//			userTouchFlag[i] = 1;
		//		
		//		else if(	point3fDist( leftHand, niImageStruct.Users[1][7].position3D) < distThresh ||	// both hand to right hand
		//					point3fDist( rightHand, niImageStruct.Users[1][7].position3D) < distThresh )		
		//					userTouchFlag[i] = 1;

		//		else if(	point3fDist( leftHand, niImageStruct.Users[1][2].position3D) < distThresh ||	// both hand to left shoulder
		//					point3fDist( rightHand, niImageStruct.Users[1][2].position3D) < distThresh )		
		//					userTouchFlag[i] = 1;

		//		else if(	point3fDist( leftHand, niImageStruct.Users[1][3].position3D) < distThresh ||	// both hand to right shoulder
		//					point3fDist( rightHand, niImageStruct.Users[1][3].position3D) < distThresh )		
		//					userTouchFlag[i] = 1;
		//		else
		//			userTouchFlag[i] = 0;
		//	}

		//	if(i==1)	// for user 1
		//	{
		//		if(	point3fDist( leftHand, niImageStruct.Users[0][6].position3D) < distThresh ||	// both hand to left hand
		//			point3fDist( rightHand, niImageStruct.Users[0][6].position3D) < distThresh )		
		//			userTouchFlag[i] = 1;
		//		
		//		else if(	point3fDist( leftHand, niImageStruct.Users[0][7].position3D) < distThresh ||	// both hand to right hand
		//					point3fDist( rightHand, niImageStruct.Users[0][7].position3D) < distThresh )		
		//					userTouchFlag[i] = 1;

		//		else if(	point3fDist( leftHand, niImageStruct.Users[0][2].position3D) < distThresh ||	// both hand to left shoulder
		//					point3fDist( rightHand, niImageStruct.Users[0][2].position3D) < distThresh )		
		//					userTouchFlag[i] = 1;

		//		else if(	point3fDist( leftHand, niImageStruct.Users[0][3].position3D) < distThresh ||	// both hand to right shoulder
		//					point3fDist( rightHand, niImageStruct.Users[0][3].position3D) < distThresh )		
		//					userTouchFlag[i] = 1;
		//		else
		//			userTouchFlag[i] = 0;
		//	}
		//}
		//else	// only one user or on user be seen
		//	for(unsigned int j(0); j<userTouchFlag.size();j++) userTouchFlag[j]=0;

		//cout << "userTouchFlag:" << userTouchFlag[i] << endl;	// debug output
	}
	//Sleep(1000);
	//system("cls");
	return 0;
}

const float BodyDirection::point3fDist(const nite::Point3f a, const nite::Point3f b)
{
	return sqrt(pow(a.x-b.x,2) + pow(a.y-b.y,2) + pow(a.z-b.z,2));
}

const int BodyDirection::startGetBodyDirection()
{
		// Enable the thread to capture the color and depth image
	startCapture();
		/** Connect to IPC Server **/
	init_comm();
	connect_to_server();
	subscribe(PERCEPTION_HAE, TOTAL_MSG_NUM);
	publish(HAE, TOTAL_MSG_NUM);
	listen();

	extern bool bodyDirectionFlag, bodyDirectionContFlag;
	bodyDirectionHAE = 0;

	while (true)
	{
			/* Press 'Q' (81), 'q' (113) or esc (27) to leave */
		if (kbhit()) {
			int command = getch();
			if (command == 81 || command == 113 || command == 27)
				break;
		}

		analyzeFunc();
		drawImg();

		if (bodyDirectionFlag == true) {
				// Send the body direction message
			HAEMgr HAEdata;
			getHAE(HAEdata);
			HAEdata.body_direction = static_cast< Body_Direction_HAE_type >(getBodyDirectionHAE());
			sendHAE(HAEdata);
			Sleep(sizeof(HAEdata));
			printf("\n> Send Success! (BodyDirection: %d)\n", getBodyDirectionHAE());

			bodyDirectionFlag = false;
			
			/* Store the image for annotating data */
			//NiImageStruct niImageStruct = getNiImageStruct();
			//cv::Mat img;
			//cv::resize(niImageStruct.Color, img, cv::Size(niImageStruct.Color.cols / 2, niImageStruct.Color.rows / 2));
			//
			//stringstream ss;
			//string imgNumber;
			//ss << savedNumber++;
			//ss >> imgNumber;
			//
			//img = cv::imread("../models/DBN_Model/TrainingData/TEST.png");
			//cv::imshow("TEST", img);
			////cv::imwrite("raw" + imgNumber + ".jpg", img);
			//cout << cv::imwrite("../models/DBN_Model/TrainingData/raw" + imgNumber + ".jpg", img) << endl;
		}

		if (bodyDirectionContFlag == true) {
				// Send the body direction continous message
			HAEMgr HAEdata;
			getHAE(HAEdata);
			HAEdata.body_direction_cont = getBodyDirectionCont();
			sendHAE(HAEdata);
			Sleep(sizeof(HAEdata));
			printf("\n> Send Success! (BodyDirectionCont: %f)\n", getBodyDirectionHAE());

			bodyDirectionContFlag = false;
		}
	}
	
	disconnect_to_server();
	return 0;
}

const int BodyDirection::calculateBodyFformation()
{
	NiImageStruct niImageStruct = getNiImageStruct();

	if(niImageStruct.Users.size()==0)		// no people
		bodyFformation = 0;	
	else
	{
		if(	niImageStruct.Users.size()==1 &&	
			userBodyAngle[0] > 45.0 && 
			userBodyAngle[0] < 135.0)			// H2R
		{
			bodyFformation = 1;
		}
		else
		{
			if( niImageStruct.Users.size()== 2 )
			{
				if(	(userBodyAngle[0]<45.0 || userBodyAngle[0]>135.0) && 
					(userBodyAngle[1]<45.0 || userBodyAngle[1]>135.0) )	//H2H
				{
					bodyFformation = 2;
				}
				else
				{
					if(	userBodyAngle[0]>45.0 && userBodyAngle[0]<135.0 && 
						userBodyAngle[1]>45.0 && userBodyAngle[1]<135.0)		//HH2R
					{
						bodyFformation = 3;
					}
					else
						bodyFformation = 0;
				}
			}
			else
				bodyFformation = 0;
		}
	}
	return 0;
}

const int BodyDirection::calculateBodyDirectionToCamera()
{
	NiImageStruct niImageStruct = getNiImageStruct();

	if (niImageStruct.Users.size() == 0)		// None_HAE_Body
		bodyDirectionHAE = 0;
	else {
		nite::Point3f neckPoint = niImageStruct.Users[0][1].position3D;

		//double angleCamToUser = acos(neckPoint.z / point3fDist(neckPoint, nite::Point3f(0, 0, 0))) * 180.0 / M_PI; // This angle calculated in 3D
		double angleCamToUser = acos(neckPoint.z / point3fDist(nite::Point3f(neckPoint.x, 0.0, neckPoint.z), nite::Point3f(0.0, 0.0, 0.0))) * 180.0 / M_PI;
		double bodyAngle = userBodyAngle[0];
		if (neckPoint.x > 0.0)
			bodyAngle = -(bodyAngle - 180);
		//double bodyAngle = ((userBodyAngle[0] > 90.0) ? 180 - userBodyAngle[0] : userBodyAngle[0]);

		bodyDirectionToCamera = 90 - angleCamToUser - bodyAngle;
		double FocusToCamera = fabs(bodyDirectionToCamera);
		
		if (FocusToCamera > 35.0)
			bodyDirectionHAE = 1;
		else if (FocusToCamera > 20.0)
			bodyDirectionHAE = 2;
		else
			bodyDirectionHAE = 3;
			// For debug output
		//cout << "> AngleCamToUser: " << angleCamToUser << " " << neckPoint.x << endl;
		//cout << "> Body Direction Toward Camera(C): " << bodyDirectionToCamera << endl;
		//cout << "> Body Direction Toward Camera(D): " << bodyDirectionHAE << endl;
	}
	return 0;
}

const int BodyDirection::drawImg()
{
	/*	Critical Settion	*/
	NiImageStruct niImageStructTemp;

	niImageStructTemp = getShareVariable();

	if (niImageStructTemp.Color.empty() == true) {
		cout << "> WARNING: Image Captor failed!" << endl;
		return 1;
	}

	for(unsigned int i(0); i < niImageStructTemp.Users.size(); i++)
	{
		/*	Drawing Skelenton Line	*/
			// Upper body
		cv::line( niImageStructTemp.Color, niImageStructTemp.Users[i][ 0].position2D, niImageStructTemp.Users[i][ 1].position2D, cv::Scalar( 255, 0, 0 ), 3 );
		cv::line( niImageStructTemp.Color, niImageStructTemp.Users[i][ 1].position2D, niImageStructTemp.Users[i][ 2].position2D, cv::Scalar( 255, 0, 0 ), 3 );
		cv::line( niImageStructTemp.Color, niImageStructTemp.Users[i][ 1].position2D, niImageStructTemp.Users[i][ 3].position2D, cv::Scalar( 255, 0, 0 ), 3 );
		cv::line( niImageStructTemp.Color, niImageStructTemp.Users[i][ 2].position2D, niImageStructTemp.Users[i][ 4].position2D, cv::Scalar( 255, 0, 0 ), 3 );
		cv::line( niImageStructTemp.Color, niImageStructTemp.Users[i][ 3].position2D, niImageStructTemp.Users[i][ 5].position2D, cv::Scalar( 255, 0, 0 ), 3 );
		cv::line( niImageStructTemp.Color, niImageStructTemp.Users[i][ 4].position2D, niImageStructTemp.Users[i][ 6].position2D, cv::Scalar( 255, 0, 0 ), 3 );
		cv::line( niImageStructTemp.Color, niImageStructTemp.Users[i][ 5].position2D, niImageStructTemp.Users[i][ 7].position2D, cv::Scalar( 255, 0, 0 ), 3 );
		cv::line( niImageStructTemp.Color, niImageStructTemp.Users[i][ 1].position2D, niImageStructTemp.Users[i][ 8].position2D, cv::Scalar( 255, 0, 0 ), 3 );
			// Lower body
		cv::line( niImageStructTemp.Color, niImageStructTemp.Users[i][ 8].position2D, niImageStructTemp.Users[i][ 9].position2D, cv::Scalar( 255, 0, 0 ), 3 );
		cv::line( niImageStructTemp.Color, niImageStructTemp.Users[i][ 8].position2D, niImageStructTemp.Users[i][10].position2D, cv::Scalar( 255, 0, 0 ), 3 );
			// Print user ID
		stringstream ss; ss  << "USER_ID: " << i;
		cv::putText(niImageStructTemp.Color, ss.str(), niImageStructTemp.Users[i][ 8].position2D, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(128,255,128), 2);

		/*	Drawing Jonint	*/
		for( int  s = 0; s < 11; ++ s ) // To joint 8 for upper body, 15 for whole body
		{
			if( niImageStructTemp.Users[i][s].jointConfidence > 0.5 )
				cv::circle( niImageStructTemp.Color, niImageStructTemp.Users[i][s].position2D, 3, cv::Scalar( 0, 0, 255 ), 2 );
			else
				cv::circle( niImageStructTemp.Color, niImageStructTemp.Users[i][s].position2D, 3, cv::Scalar( 0, 255, 0 ), 2 );

			//ss.str("");
			//ss << s;
			//cv::putText(niImageStructTemp.Color, ss.str(), niImageStructTemp.Users[i][s].position2D, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(205,205,205), 2); // Show joint number
		}

		/* Draw Features */
			// Body Direction
		ss.str(""); ss << "BodyDirection: " << userBodyAngle[i];
		cv::Point2f bd(niImageStructTemp.Users[i][ 8].position2D.x, niImageStructTemp.Users[i][ 8].position2D.y + 20);
		cv::putText(niImageStructTemp.Color, ss.str(), bd, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(205,205,205), 2);
			// Body Lean
		ss.str(""); ss << "BodyLean: " << userLeanAngle[i];
		cv::Point2f bl(niImageStructTemp.Users[i][ 8].position2D.x, niImageStructTemp.Users[i][ 8].position2D.y + 40);
		cv::putText(niImageStructTemp.Color, ss.str(), bl, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(205,205,205), 2);
			// Hand Speed
		ss.str(""); ss << "HandSpeed: " << userHandSpeed[i];
		cv::Point2f hs(niImageStructTemp.Users[i][ 8].position2D.x, niImageStructTemp.Users[i][ 8].position2D.y + 60);
		cv::putText(niImageStructTemp.Color, ss.str(), hs, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(205,205,205), 2);
	}

	cv::imshow( "Color Image", niImageStructTemp.Color );
	cv::waitKey(10);
	return 0;
}