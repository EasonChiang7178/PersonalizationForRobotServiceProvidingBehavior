/* Third-party Library */
	// Import Inter-Process Communication Server
#include "IPCserver\Client.hpp"
#include "BodyDirection.h"

#include <cmath>
#include <sstream>

//#define _USE_MATH_DEFINES

#define M_PI       3.14159265358979323846

using namespace std;

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

	w1 = 0.8; w2 = 0.7; w3 = 2; w4 = 1.0; w5 = 2.0;

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
			/* Send the body direction message */
			HAEMgr HAEdata;
			getHAE(HAEdata);
				// Prepare the message for body directon to camera discrete
			vector< int > bdd = getBodyDirectionHAE();
			if (bdd.size() < 0) {
				cout << "> WARNING: user is not found" << endl;
				HAEdata.body_direction = static_cast< Body_Direction_HAE_type >(0);
			} else {
//** WARNING: ONLY CONSIDER THE USER 1 **//
				HAEdata.body_direction = static_cast< Body_Direction_HAE_type >(bdd[0]);
			}
			sendHAE(HAEdata);
			Sleep(sizeof(HAEdata));
			printf("\n> Send Success! (BodyDirection: %d)\n", bdd[0]);

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
			vector< double > bdc = getBodyDirectionCont();
			if (bdc.size() < 0) {
				cout << "> WARNING: user is not found" << endl;
				HAEdata.body_direction_cont = 180.0;
			} else {
//** WARNING: ONLY CONSIDER THE USER 1 **//
				HAEdata.body_direction_cont = static_cast< float >(bdc[0]);
			}
			sendHAE(HAEdata);
			Sleep(sizeof(HAEdata));
			printf("\n> Send Success! (BodyDirectionCont: %f)\n", getBodyDirectionHAE());

			bodyDirectionContFlag = false;
		}
	}
	
	disconnect_to_server();
	return 0;
}

const int BodyDirection::analyzeFunc()
{
	NiImageStruct niImageStruct = getNiImageStruct();
	cout << "user size: " << niImageStruct.Users.size() << endl;

	/* Initialize	*/
	vector< double > oneUserAngle;
	nite::Point3f point3D;
	double angle;
	int valueDummy;
	
	/* Body Direction */
		// For smoothing the body direction
	while(thetaData.size() < niImageStruct.Users.size())
		thetaData.push_back(oneUserAngle);
		// Body direction
	while(userBodyAngle.size() < niImageStruct.Users.size())
		userBodyAngle.push_back(angle);
		// For bodyDirectionToCameraDiscrete
	while(bodyDirectionHAE.size() < niImageStruct.Users.size())
		bodyDirectionHAE.push_back(valueDummy);
		// For bodyDirectionToCameraCont
	while(bodyDirectionToCamera.size() < niImageStruct.Users.size())
		bodyDirectionToCamera.push_back(angle);

	/* Pleasantness-Unpleasantness Detection */
		// For Hand to Head
	while(userHandToHead.size() < niImageStruct.Users.size())
		userHandToHead.push_back(angle);
		// For ArmAsymmetry
	while(userArmAsymmetry.size() < niImageStruct.Users.size())
		userArmAsymmetry.push_back(angle);
		// For ArmAreaSpanned
	while(userArmAreaSpanned.size() < niImageStruct.Users.size())
		userArmAreaSpanned.push_back(angle);
		// For HandToBody
	while(userHandToBody.size() < niImageStruct.Users.size())
		userHandToBody.push_back(angle);
	//	// Body leaning
	//while(userLeanAngle.size() < niImageStruct.Users.size())
	//	userLeanAngle.push_back(angle);

	/* Intimacy detection */
		// Touch flag
	//while(userTouchFlag.size() < niImageStruct.Users.size())
	//	userTouchFlag.push_back(valueDummy);

		// Clear the value of features if no user detected
	if (niImageStruct.Users.size() == 0) {
		for(auto itBDD = bodyDirectionHAE.begin(); itBDD != bodyDirectionHAE.end(); itBDD++)
			*itBDD = 0;
		for(auto itBDC = bodyDirectionToCamera.begin(); itBDC != bodyDirectionToCamera.end(); itBDC++)
			*itBDC = 90.0;
	}

	/*	Counting features of users	*/
	for(unsigned int i(0); i < niImageStruct.Users.size(); i++)	// for all users
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
		calculateBodyDirectionToCamera(i, niImageStruct);

		/*	Calculating Body Lean Angle	*/
		//nite::Point3f vecOfRightShoudlerToLeftHip;
		//nite::Point3f vecOfLeftShoudlerToRightHip;

		//nite::Point3f leftHip = niImageStruct.Users[i][9].position3D;
		//nite::Point3f rightHip = niImageStruct.Users[i][10].position3D;

		//vecOfRightShoudlerToLeftHip.x = leftHip.x - rightShoulder.x;
		//vecOfRightShoudlerToLeftHip.y = leftHip.y - rightShoulder.y;
		//vecOfRightShoudlerToLeftHip.z = leftHip.z - rightShoulder.z;

		//vecOfLeftShoudlerToRightHip.x = rightHip.x - leftShoulder.x;
		//vecOfLeftShoudlerToRightHip.y = rightHip.y - leftShoulder.y;
		//vecOfLeftShoudlerToRightHip.z = rightHip.z - leftShoulder.z;

		//nite::Point3f vecOfCrossProduct;
		//vecOfCrossProduct.x = vecOfRightShoudlerToLeftHip.y*vecOfLeftShoudlerToRightHip.z
		//					 -vecOfRightShoudlerToLeftHip.z*vecOfLeftShoudlerToRightHip.y;

		//vecOfCrossProduct.y = vecOfRightShoudlerToLeftHip.z*vecOfLeftShoudlerToRightHip.x
		//					 -vecOfRightShoudlerToLeftHip.x*vecOfLeftShoudlerToRightHip.z;
		//
		//vecOfCrossProduct.z = vecOfRightShoudlerToLeftHip.x*vecOfLeftShoudlerToRightHip.y
		//					 -vecOfRightShoudlerToLeftHip.y*vecOfLeftShoudlerToRightHip.x;

		//double lenghtOfCrossProduct = sqrt(	pow(vecOfCrossProduct.x,2)+
		//									pow(vecOfCrossProduct.y,2)+
		//									pow(vecOfCrossProduct.z,2));

		//userLeanAngle[i] = asin(vecOfCrossProduct.y/lenghtOfCrossProduct) * 180.0 / M_PI;
		
		//cout << "userLeanAngle:" << userLeanAngle[i] << endl;	// debug  output

		/*	Calculating Hand Speed	*/
		this->calculateHandSpeed(i, niImageStruct, 2);
		//cout << "userHandSpeedMax:" << userHandSpeedMax[i] << endl;	// debug output

		/* Calculating Hand To Head Feature */
		this->calculateHandToHead(i, niImageStruct);

		/* Calculating Hand To Trunk Feature */
		this->calculateHandToBody(i, niImageStruct);

		/* Calculating ArmAsymmety */
		this->calculateArmAsymmetry(i, niImageStruct);

		/* Calculating ArmAreaSpanned */
		this->calculateArmAreaSpanned(i, niImageStruct);

		if (kbhit()) {
			int command = getch();
			if (command == 'a')
				w1 += 0.1;
			if (command == 'z')
				w1 -= 0.1;
			if (command == 's')
				w2 += 0.1;
			if (command == 'x')
				w2 -= 0.1;
			if (command == 'd')
				w3 += 0.1;
			if (command == 'c')
				w3 -= 0.1;
			if (command == 'f')
				w4 += 0.1;
			if (command == 'v')
				w4 -= 0.1;
			if (command == 'g')
				w5 += 0.1;
			if (command == 'b')
				w5 -= 0.1;
			
		}

		/* Calculating Pleasantness-Unpleasantness */
		this->calculatePU(i, niImageStruct, w1, w2, w3, w4, w5);
		cout << "w1: " << w1 << ", w2: " << w2 << ", w3: " << w3 << ", w4: " << w4 << ", w5: " << w5 << endl;

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

const int BodyDirection::calculateBodyDirectionToCamera(const int& userID, const NiImageStruct& niImageStruct)
{
	if (niImageStruct.Users.size() == 0) {		// None_HAE_Body
		for(auto itBDD = bodyDirectionHAE.begin(); itBDD != bodyDirectionHAE.end(); itBDD++)
			*itBDD = 0;
		for(auto itBDC = bodyDirectionToCamera.begin(); itBDC != bodyDirectionToCamera.end(); itBDC++)
			*itBDC = 90.0;
	} else {
		nite::Point3f neckPoint = niImageStruct.Users[userID][1].position3D;

		//double angleCamToUser = acos(neckPoint.z / point3fDist(neckPoint, nite::Point3f(0, 0, 0))) * 180.0 / M_PI; // This angle calculated in 3D
		double angleCamToUser = acos(neckPoint.z / point3fDist(nite::Point3f(neckPoint.x, 0.0, neckPoint.z), nite::Point3f(0.0, 0.0, 0.0))) * 180.0 / M_PI;
		double bodyAngle = userBodyAngle[userID];
		if (neckPoint.x > 0.0)
			bodyAngle = -(bodyAngle - 180);
		//double bodyAngle = ((userBodyAngle[0] > 90.0) ? 180 - userBodyAngle[0] : userBodyAngle[0]);

		bodyDirectionToCamera[userID] = 90 - angleCamToUser - bodyAngle;
		double FocusToCamera = fabs(bodyDirectionToCamera[userID]);
		
		if (FocusToCamera > 35.0)
			bodyDirectionHAE[userID] = 1;
		else if (FocusToCamera > 20.0)
			bodyDirectionHAE[userID] = 2;
		else
			bodyDirectionHAE[userID] = 3;
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
		}

		/* Draw Features */
		int displayPos = 1;
		//	// Body Direction
		//ss.str(""); ss << "BD: " << userBodyAngle[i];
		//cv::Point2f bd(niImageStructTemp.Users[i][ 8].position2D.x + 20, niImageStructTemp.Users[i][ 8].position2D.y + 20 * displayPos++);
		//cv::putText(niImageStructTemp.Color, ss.str(), bd, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(205,205,205), 2);
		//	// Body Direction To Camera Discrete
		//ss.str(""); ss << "BD (D): " << bodyDirectionHAE[i];
		//cv::Point2f bdd(niImageStructTemp.Users[i][ 8].position2D.x + 20, niImageStructTemp.Users[i][ 8].position2D.y + 20 * displayPos++);
		//cv::putText(niImageStructTemp.Color, ss.str(), bdd, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(205,205,205), 2);
		//	// Body Direction To Camera Continous
		//if (bodyDirectionToCamera.size() == 0)
		//	return 1;
		//ss.str(""); ss << "BD (C): " << bodyDirectionToCamera[i];
		//cv::Point2f bdc(niImageStructTemp.Users[i][ 8].position2D.x + 20, niImageStructTemp.Users[i][ 8].position2D.y + 20 * displayPos++);
		//cv::putText(niImageStructTemp.Color, ss.str(), bdc, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(205,205,205), 2);
			// Body Lean
		//if (userLeanAngle.size() < niImageStructTemp.Users.size())
		//	return 1;
		//ss.str(""); ss << "BL: " << userLeanAngle[i];
		//cv::Point2f bl(niImageStructTemp.Users[i][ 8].position2D.x + 20, niImageStructTemp.Users[i][ 8].position2D.y + 20 * displayPos++);
		/*cv::putText(niImageStructTemp.Color, ss.str(), bl, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,230), 2);*/
			// Hand To Head
		if (userHandToHead.size() < niImageStructTemp.Users.size())
			return 1;
		ss.str(""); ss << "HtoH: " << userHandToHead[i];
		cv::Point2f hth(niImageStructTemp.Users[i][ 8].position2D.x + 20, niImageStructTemp.Users[i][ 8].position2D.y + 20 * displayPos++);
		cv::putText(niImageStructTemp.Color, ss.str(), hth, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,230), 2);
			// ArmAsymmetry
		if (userArmAsymmetry.size() < niImageStructTemp.Users.size())
			return 1;
		ss.str(""); ss << "AS: " << userArmAsymmetry[i];
		cv::Point2f as(niImageStructTemp.Users[i][ 8].position2D.x + 20, niImageStructTemp.Users[i][ 8].position2D.y + 20 * displayPos++);
		cv::putText(niImageStructTemp.Color, ss.str(), as, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,230), 2);
			// ArmAreaSpanned
		if (userArmAreaSpanned.size() < niImageStructTemp.Users.size())
			return 1;
		ss.str(""); ss << "AAS: " << userArmAreaSpanned[i];
		cv::Point2f aas(niImageStructTemp.Users[i][ 8].position2D.x + 20, niImageStructTemp.Users[i][ 8].position2D.y + 20 * displayPos++);
		cv::putText(niImageStructTemp.Color, ss.str(), aas, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,230), 2);
			// Hand Speed Max
		if (userHandSpeedMax.size() < niImageStructTemp.Users.size())
			return 1;
		ss.str(""); ss << "HS: " << userHandSpeedMax[i];
		cv::Point2f hs(niImageStructTemp.Users[i][ 8].position2D.x + 20, niImageStructTemp.Users[i][ 8].position2D.y + 20 * displayPos++);
		cv::putText(niImageStructTemp.Color, ss.str(), hs, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,230), 2);
			// Dynamics
		if (userDynamics.size() < niImageStructTemp.Users.size())
			return 1;
		ss.str(""); ss << "HS (D): " << userDynamics[i];
		cv::Point2f d(niImageStructTemp.Users[i][ 8].position2D.x + 20, niImageStructTemp.Users[i][ 8].position2D.y + 20 * displayPos++);
		cv::putText(niImageStructTemp.Color, ss.str(), d, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,230), 2);
			// Hand To Body
		if (userHandToBody.size() < niImageStructTemp.Users.size())
			return 1;
		ss.str(""); ss << "HtoB: " << userHandToBody[i];
		cv::Point2f htb(niImageStructTemp.Users[i][ 8].position2D.x + 20, niImageStructTemp.Users[i][ 8].position2D.y + 20 * displayPos++);
		cv::putText(niImageStructTemp.Color, ss.str(), htb, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,230), 2);
			// Pleasantness-Unpleasantness
		if (userUnpleasantness.size() < niImageStructTemp.Users.size())
			return 1;
		ss.str(""); ss << "PU: " << userUnpleasantness[i];
		cv::Point2f pu(niImageStructTemp.Users[i][ 8].position2D.x + 20, niImageStructTemp.Users[i][ 8].position2D.y + 20 * displayPos++);
		cv::putText(niImageStructTemp.Color, ss.str(), pu, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,230), 2);
			// Display Pleasantness-Unpleasantness level
		if (userUnpleasantness.size() < niImageStructTemp.Users.size())
			return 1;
		ss.str("");
		cv::Point2f puDisplay(niImageStructTemp.Users[i][ 8].position2D.x + 20, niImageStructTemp.Users[i][ 8].position2D.y + 20 * displayPos++);
		switch (static_cast< int > (userUnpleasantness[i])) {
			case 0:
				ss << "Pleasantness";
				cv::putText(niImageStructTemp.Color, ss.str(), puDisplay, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,188,0), 2);
				break;

			case 1:
				ss << "Like";
				cv::putText(niImageStructTemp.Color, ss.str(), puDisplay, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,230,0), 2);
				break;

			case 2:
				ss << "Neutral";
				cv::putText(niImageStructTemp.Color, ss.str(), puDisplay, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,230), 2);
				break;

			case 3:
				ss << "Offensivenss";
				cv::putText(niImageStructTemp.Color, ss.str(), puDisplay, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,171,255), 2);
				break;

			case 4:
			case 5:
			case 6:
				ss << "Unpleasantness";
				cv::putText(niImageStructTemp.Color, ss.str(), puDisplay, CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,43,255), 2);
				break;
		}
	}

	cv::imshow( "Color Image", niImageStructTemp.Color );
	cv::waitKey(10);
	return 0;
}

const int BodyDirection::calculateHandToHead(const int& userID, const NiImageStruct& niImageStruct) {
	/* Initialization */
	nite::Point3f head = niImageStruct.Users[userID][0].position3D;
	nite::Point3f lHand = niImageStruct.Users[userID][6].position3D;
	nite::Point3f rHand = niImageStruct.Users[userID][7].position3D;

	/* Computation */
	float lHandToHead = this->gaussianKernal3((head.x - lHand.x) / 500, (head.y - lHand.y) / 500, (head.z - lHand.z) / 500, 0.85f);
	float rHandToHead = this->gaussianKernal3((head.x - rHand.x) / 500, (head.y - rHand.y) / 500, (head.z - rHand.z) / 500, 0.85f);

	userHandToHead[userID] = (lHandToHead + rHandToHead) / 2;// * std::exp(-1.0 / 2.0);
	return 0;
}

const int BodyDirection::calculateHandToBody(const int& userID, const NiImageStruct& niImageStruct) {
	/* Initialization */
	nite::Point3f trunk = niImageStruct.Users[userID][8].position3D;
	nite::Point3f lHand = niImageStruct.Users[userID][6].position3D;
	nite::Point3f rHand = niImageStruct.Users[userID][7].position3D;

	/* Computation */
	float lHandToTrunk = this->gaussianKernal3(0, 0, (trunk.z - lHand.z) / 500, static_cast< float >(0.5));
	float rHandToTrunk = this->gaussianKernal3(0, 0, (trunk.z - rHand.z) / 500, static_cast< float >(0.5));

	//userHandToBody[userID] = lHandToTrunk * rHandToTrunk * std::exp(-1.0 / 2.0);
	userHandToBody[userID] = (lHandToTrunk + rHandToTrunk) / 2;
	return 0;
}

const int BodyDirection::calculateArmAsymmetry(const int& userID, const NiImageStruct& niImageStruct) {
	/* Initialization */
	nite::Point3f lShould = niImageStruct.Users[userID][2].position3D;
	nite::Point3f rShould = niImageStruct.Users[userID][3].position3D;
	nite::Point3f lElbow = niImageStruct.Users[userID][4].position3D;
	nite::Point3f rElbow = niImageStruct.Users[userID][5].position3D;
	nite::Point3f lHand = niImageStruct.Users[userID][6].position3D;
	nite::Point3f rHand = niImageStruct.Users[userID][7].position3D;
	nite::Point3f trunk = niImageStruct.Users[userID][8].position3D;

	/* Reflect left side of elbow and hand into right side */
		// Calculate normal vector of the reflection plane
	nite::Point3f normalVecReflection(lShould.x - rShould.x, lShould.y - rShould.y, lShould.z - rShould.z);
	float normalLength = point3fLength(normalVecReflection);
	normalVecReflection.x /= normalLength; normalVecReflection.y /= normalLength; normalVecReflection.z /= normalLength;
		// Reflect the lElbow point
	nite::Point3f lElbowVec(lElbow.x - trunk.x, lElbow.y - trunk.y, lElbow.z - trunk.z);
	float distElbowToProjTrunk = lElbowVec.x * normalVecReflection.x + lElbowVec.y * normalVecReflection.y + lElbowVec.z * normalVecReflection.z;
	nite::Point3f lElbowRef(lElbow.x - 2 * distElbowToProjTrunk * normalVecReflection.x,
							 lElbow.y - 2 * distElbowToProjTrunk * normalVecReflection.y,
							 lElbow.z - 2 * distElbowToProjTrunk * normalVecReflection.z);
		// Reflect the lHand point
	nite::Point3f lHandVec(lHand.x - trunk.x, lHand.y - trunk.y, lHand.z - trunk.z);
	float distHandToProjTrunk = lHandVec.x * normalVecReflection.x + lHandVec.y * normalVecReflection.y + lHandVec.z * normalVecReflection.z;
	nite::Point3f lHandRef(lHand.x - 2 * distHandToProjTrunk * normalVecReflection.x,
							 lHand.y - 2 * distHandToProjTrunk * normalVecReflection.y,
							 lHand.z - 2 * distHandToProjTrunk * normalVecReflection.z);
		// For debug output
	//cout << "rElbow, x: " << rElbow.x << ", y: " << rElbow.y << ", z: " << rElbow.z << endl;
	//cout << "lElbow, x: " << lElbowRef.x << ", y: " << lElbowRef.y << ", z: " << lElbowRef.z << endl;
	//cout << "rHand, x: " << rHand.x << ", y: " << rHand.y << ", z: " << rHand.z << endl;
	//cout << "lHand, x: " << lHandRef.x << ", y: " << lHandRef.y << ", z: " << lHandRef.z << endl;

	userArmAsymmetry[userID] = (point3fDist(lElbowRef, rElbow) / 1000.0 + point3fDist(lHandRef, rHand) / 1000.0) / 2.0;
	return 0;
}

const int BodyDirection::calculateArmAreaSpanned(const int& userID, const NiImageStruct& niImageStruct) {
	/* Initialization */
	nite::Point3f lShould = niImageStruct.Users[userID][2].position3D;
	nite::Point3f rShould = niImageStruct.Users[userID][3].position3D;
	nite::Point3f lElbow = niImageStruct.Users[userID][4].position3D;
	nite::Point3f rElbow = niImageStruct.Users[userID][5].position3D;
	nite::Point3f lHand = niImageStruct.Users[userID][6].position3D;
	nite::Point3f rHand = niImageStruct.Users[userID][7].position3D;

	/* Cross Product */
		// For 3-D vector
	//nite::Point3f vecOfUpperArmL(lElbow.x - lShould.x, lElbow.y - lShould.y, lElbow.z - lShould.z);
	//nite::Point3f vecOfLowerArmL(lHand.x - lElbow.x, lHand.y - lElbow.y, lHand.z - lElbow.z);
	//nite::Point3f vecOfUpperArmR(rElbow.x - rShould.x, rElbow.y - rShould.y, rElbow.z - rShould.z);
	//nite::Point3f vecOfLowerArmR(rHand.x - rElbow.x, rHand.y - rElbow.y, rHand.z - rElbow.z);
		// For 2-D vector
	nite::Point3f vecOfUpperArmL(lElbow.x - lShould.x, lElbow.y - lShould.y, 0.0);
	nite::Point3f vecOfLowerArmL(lHand.x - lElbow.x, lHand.y - lElbow.y, 0.0);
	nite::Point3f vecOfUpperArmR(rElbow.x - rShould.x, rElbow.y - rShould.y, 0.0);
	nite::Point3f vecOfLowerArmR(rHand.x - rElbow.x, rHand.y - rElbow.y, 0.0);
	nite::Point3f vecOfCrossProductL, vecOfCrossProductR;
	
	//vecOfCrossProductL.x = vecOfUpperArmL.y*vecOfLowerArmL.z
	//					  -vecOfUpperArmL.z*vecOfLowerArmL.y;
	//vecOfCrossProductL.y = vecOfUpperArmL.z*vecOfLowerArmL.x
	//					  -vecOfUpperArmL.x*vecOfLowerArmL.z;
	vecOfCrossProductL.z = vecOfUpperArmL.x*vecOfLowerArmL.y
						  -vecOfUpperArmL.y*vecOfLowerArmL.x;

	//vecOfCrossProductR.x = vecOfUpperArmR.y*vecOfLowerArmR.z
	//					  -vecOfUpperArmR.z*vecOfLowerArmR.y;
	//vecOfCrossProductR.y = vecOfUpperArmR.z*vecOfLowerArmR.x
	//					  -vecOfUpperArmR.x*vecOfLowerArmR.z;
	vecOfCrossProductR.z = vecOfUpperArmR.x*vecOfLowerArmR.y
						  -vecOfUpperArmR.y*vecOfLowerArmR.x;

	float lArmArea = this->gaussianKernal3(0.0f, 0.0f, 1 - (fabs(vecOfCrossProductL.z) / point3fLength(vecOfUpperArmL) / point3fLength(vecOfLowerArmL)), 0.375f);
	float rArmArea = this->gaussianKernal3(0.0f, 0.0f, 1 - (fabs(vecOfCrossProductR.z) / point3fLength(vecOfUpperArmR) / point3fLength(vecOfLowerArmR)), 0.375f);
		// Avoid the case when lower arm is too show, where the ArmArea may be 1
	if (point3fLength(vecOfLowerArmL) < 65.0)
		lArmArea = 0;
	if (point3fLength(vecOfLowerArmR) < 65.0)
		rArmArea = 0;

	userArmAreaSpanned[userID] = (lArmArea + rArmArea) / 2;
	//userArmAreaSpanned[userID] = (fabs(vecOfCrossProductL.z) / point3fLength(vecOfUpperArmL) / point3fLength(vecOfLowerArmL)//);
	//						   +  fabs(vecOfCrossProductR.z) /point3fLength(vecOfUpperArmR) / point3fLength(vecOfLowerArmR)) / 2;
	
	//userArmAreaSpanned[userID] = (point3fLength(vecOfCrossProductL) / point3fLength(vecOfUpperArmL) / point3fLength(vecOfLowerArmL));
							   //+ point3fLength(vecOfCrossProductR) / point3fLength(vecOfUpperArmR) / point3fLength(vecOfLowerArmR)) / 2;
	return 0;
}

const int BodyDirection::calculateHandSpeed(const int& userID, const NiImageStruct& niImageStruct, const int& intervalMax) {
	/* Initialization */
		// Hand Speed of both two hands
	while(userHandSpeed.size() < niImageStruct.Users.size())
		userHandSpeed.push_back(double(0.0));
	while(userHandSpeedHistory.size() < niImageStruct.Users.size())
		userHandSpeedHistory.push_back(vector< double >(0));
		// For counting the hand speed
	while(leftHandTemp.size() < niImageStruct.Users.size())
		leftHandTemp.push_back(nite::Point3f(0.0, 0.0, 0.0));
	while(rightHandTemp.size() < niImageStruct.Users.size())
		rightHandTemp.push_back(nite::Point3f(0.0, 0.0, 0.0));
		// For Dynamics (HandSpeedMax)
	while(userDynamics.size() < niImageStruct.Users.size())
		userDynamics.push_back(double(0.0));
		// For HandSpeedMax
	while(userHandSpeedMax.size() < niImageStruct.Users.size())
		userHandSpeedMax.push_back(double(0.0));

	/* Computation */
		// Current Hand Speed
	nite::Point3f leftHand = niImageStruct.Users[userID][6].position3D;
	nite::Point3f rightHand = niImageStruct.Users[userID][7].position3D;

	float leftHandDisplacement = point3fDist( leftHand, leftHandTemp[userID]);
	float rightHandDisplacement = point3fDist( rightHand, rightHandTemp[userID]);

	if (leftHandDisplacement > 300.0)
		leftHandDisplacement = 0.0;
	if (rightHandDisplacement > 300.0)
		rightHandDisplacement = 0.0;

	leftHandTemp[userID] = leftHand;
	rightHandTemp[userID] = rightHand;

	float movingHand = rightHandDisplacement;
	if (leftHandDisplacement > rightHandDisplacement) {
		movingHand = leftHandDisplacement;

	if (niImageStruct.Users[userID][6].jointConfidence <= 0.5)
		movingHand = 0;
	} else if (niImageStruct.Users[userID][7].jointConfidence <= 0.5)
		movingHand = 0;

	userHandSpeed[userID] = movingHand * 10.0 / 2000.0; // avg in unit (m/s)?

	/* Smoothing */
		// Maintain the history data
	userHandSpeedHistory[userID].push_back(userHandSpeed[userID]);
	if (userHandSpeedHistory[userID].size() > 8)
		userHandSpeedHistory[userID].erase(userHandSpeedHistory[userID].begin());
		// Smoothing
	double hsTempSmoothed = 0.0;
	for (unsigned int i(0); i < userHandSpeedHistory[userID].size(); i++)
		hsTempSmoothed += userHandSpeedHistory[userID][i] * 0.125; // Mean average filter for 8 frames

	/* Classify speed into light or sudden */
	int dynamicTemp = 0;
	if (hsTempSmoothed > 0.18)
		dynamicTemp = -1;
	if (hsTempSmoothed > 0.24)
		dynamicTemp = 1;

	if (userDynamics[userID] == -1 || fabs(hsTempSmoothed - userHandSpeedMax[userID]) < 0.03)
		time(&startHS);

	if (hsTempSmoothed > userHandSpeedMax[userID]) {
		userHandSpeedMax[userID] = hsTempSmoothed;
		userDynamics[userID] = dynamicTemp;
		time(&startHS);
	}
	time(&endHS);

	if (difftime(endHS, startHS) > intervalMax) {
		userHandSpeedMax[userID] = hsTempSmoothed;
		userDynamics[userID] = dynamicTemp;
	}

	return 0;
}

const int BodyDirection::calculatePU(const int& userID, const NiImageStruct& niImageStruct, const double& w1, const double& w2, const double& w3, const double& w4, const double& w5) {
	/* Initialization */
		// For Pleasantness-Unpleasantness
	while (userUnpleasantness.size() < niImageStruct.Users.size())
		userUnpleasantness.push_back(double(0.0));
		// For History Data
	while (userPUHistory.size() < niImageStruct.Users.size())
		userPUHistory.push_back(vector< double >(0));
	
	/* Computing and Smoothing the PU */
	double puTemp = w1 * userHandToHead[userID] +
					w2 * userArmAsymmetry[userID] +
					w3 * userArmAreaSpanned[userID] +
					w4 * userDynamics[userID] +
					w5 * userHandToBody[userID];
		// Maintain the history data
	userPUHistory[userID].push_back(puTemp);
	if (userPUHistory[userID].size() > 8)
		userPUHistory[userID].erase(userPUHistory[userID].begin());
		// Smoothing
	double puTempSmooth = 0.0;
	for (unsigned int j(0); j < userPUHistory[userID].size(); j++)
		puTempSmooth += userPUHistory[userID][j] * 0.125; // Mean average filter, for 8 frames

	userUnpleasantness[userID] = puTempSmooth;

	/* Unsmoothing version */
	//userUnpleasantness[userID] = w1 * userHandToHead[userID] +
	//							 w2 * userArmAsymmetry[userID] +
	//							 w3 * userArmAreaSpanned[userID] +
	//							 w4 * userHandSpeedMax[userID] +
	//							 w5 * userHandToBody[userID];
	return 0;
}