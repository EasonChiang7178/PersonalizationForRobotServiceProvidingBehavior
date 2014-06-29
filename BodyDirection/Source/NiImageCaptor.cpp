

#include "NiImageCaptor.h"
//#include <ctime>

bool NiImageCaptor::isStart = false;

/**
* Function Name:	Constructor
* Function Purpose:	Do noting
*/
NiImageCaptor::NiImageCaptor()
{
}

/**
* Function Name:	Destructor
* Function Purpose:	Stop the thread and close device if not closed
*/
NiImageCaptor::~NiImageCaptor()
{
	this->stopCapture();
}

/**
* Function Name:	Thread
* Function Purpose:	Called by startCaptrue and capture images and user infor
*/
void NiImageCaptor::thread()
{	
	/*	Initialize Device and OpenNI	*/
	openni::Status rc = openni::STATUS_OK;
	
	rc = openni::OpenNI::initialize();
	cout << "Initialization..." << openni::OpenNI::getExtendedError() << endl;

	const char* deviceURI = openni::ANY_DEVICE;
	if (device.open(deviceURI) != openni::STATUS_OK)
	{
		cout << "Device open failed:" << openni::OpenNI::getExtendedError() << endl;
		openni::OpenNI::shutdown();
		return;
	}

	/*	Create Depth and Color Stream	*/
	if (depth.create(device, openni::SENSOR_DEPTH) != openni::STATUS_OK)
	{
		cout << "Couldn't find depth stream:" << openni::OpenNI::getExtendedError() << endl;
		return;
	}

	if (color.create(device, openni::SENSOR_COLOR) != openni::STATUS_OK)
	{
		cout << "Couldn't find color stream:" << openni::OpenNI::getExtendedError() << endl;
		return;
	}

	/*	Set the Video Stream	*/
	mModeDepth.setResolution( 640, 480 );
	mModeDepth.setFps( 30 );
	mModeDepth.setPixelFormat( openni::PIXEL_FORMAT_DEPTH_1_MM );
	if(depth.setVideoMode( mModeDepth)!=openni::STATUS_OK)
	{
		cout << "Couldn't apply the setting of depth stream:" << openni::OpenNI::getExtendedError() <<endl;
		return;
	}
	 
	mModeColor.setResolution( 640, 480 );
	mModeColor.setFps( 30 );
	mModeColor.setPixelFormat( openni::PIXEL_FORMAT_RGB888 );
	if(color.setVideoMode( mModeColor)!=openni::STATUS_OK)
	{
		cout << "Couldn't apply the setting of color stream" << endl;
		return;
	}

	/*	Calibration the Depth Image to Color Image	*/
	if( device.isImageRegistrationModeSupported( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
	{
		if(device.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR )==openni::STATUS_OK)
			cout << "Calibrate depth image to color image..." << endl;
		else
			cout << "Set calibration failed:" << openni::OpenNI::getExtendedError() <<endl;
	}
	else
	{
		cout << "Not support calibrate depth image to color image..." << endl;
	}

	/*	Initial NiTE	*/
	nite::NiTE::initialize();
  
	/*	Create user tracker	*/
	nite::UserTracker mUserTracker;
	mUserTracker.create( &device );
	mUserTracker.setSkeletonSmoothingFactor( 0.5f );

	/*	Start Running	*/
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
		return;
	}

	/*	Capture Frame of Vedio Stream 	*/
	openni::VideoFrameRef depthFrame, colorFrame;
	NiImageStruct niImageStructTemp;
	
	isStart = true;	// set initialize flag

	while (isStart)
	{
		depth.readFrame( &depthFrame );
		color.readFrame( &colorFrame );
 
		const cv::Mat mImageDepth( depthFrame.getHeight(), depthFrame.getWidth(), CV_16UC1, (void*)depthFrame.getData());
		mImageDepth.convertTo( niImageStructTemp.Depth, CV_8U, 255.0 / depth.getMaxPixelValue());

		const cv::Mat mImageRGB(colorFrame.getHeight(), colorFrame.getWidth(), CV_8UC3, (void*)colorFrame.getData());
		cv::cvtColor( mImageRGB, niImageStructTemp.Color, CV_RGB2BGR );
		
		/*	Get user frame	*/
		nite::UserTrackerFrameRef  mUserFrame;
		mUserTracker.readFrame( &mUserFrame );

		/*	Get users data	*/
		const nite::Array<nite::UserData>& aUsers = mUserFrame.getUsers();
		vector<vector<JointStruct>> detectedUser;
		vector<JointStruct> oneUserData(15);

		for( int i = 0; i < aUsers.getSize(); ++ i )		// For user 1
		{
			const nite::UserData& rUser = aUsers[i];

			if( rUser.isNew() )	// check user status
				mUserTracker.startSkeletonTracking( rUser.getId() );	// start tracking for new user
 
			if( rUser.isVisible() )
			{
				const nite::Skeleton& rSkeleton = rUser.getSkeleton();	// get user skeleton
				if( rSkeleton.getState() == nite::SKELETON_TRACKED )
				{
					// p4c. build joints array
					nite::SkeletonJoint aJoints[15];
					aJoints[ 0] = rSkeleton.getJoint( nite::JOINT_HEAD );
					aJoints[ 1] = rSkeleton.getJoint( nite::JOINT_NECK );
					aJoints[ 2] = rSkeleton.getJoint( nite::JOINT_LEFT_SHOULDER );
					aJoints[ 3] = rSkeleton.getJoint( nite::JOINT_RIGHT_SHOULDER );
					aJoints[ 4] = rSkeleton.getJoint( nite::JOINT_LEFT_ELBOW );
					aJoints[ 5] = rSkeleton.getJoint( nite::JOINT_RIGHT_ELBOW );
					aJoints[ 6] = rSkeleton.getJoint( nite::JOINT_LEFT_HAND );
					aJoints[ 7] = rSkeleton.getJoint( nite::JOINT_RIGHT_HAND );
					aJoints[ 8] = rSkeleton.getJoint( nite::JOINT_TORSO );
					aJoints[ 9] = rSkeleton.getJoint( nite::JOINT_LEFT_HIP );
					aJoints[10] = rSkeleton.getJoint( nite::JOINT_RIGHT_HIP );
					aJoints[11] = rSkeleton.getJoint( nite::JOINT_LEFT_KNEE );
					aJoints[12] = rSkeleton.getJoint( nite::JOINT_RIGHT_KNEE );
					aJoints[13] = rSkeleton.getJoint( nite::JOINT_LEFT_FOOT );
					aJoints[14] = rSkeleton.getJoint( nite::JOINT_RIGHT_FOOT );
						// Low confidence, draw out this user
					if (aJoints[2].getPositionConfidence() <= 0.5 && aJoints[3].getPositionConfidence() <= 0.5 && aJoints[8].getPositionConfidence() <= 0.5) {
						mUserTracker.stopSkeletonTracking(rUser.getId());
						continue;
					}

					for( int  s = 0; s < 15; ++ s )		// change the world coordinate into image coordinate
					{
						oneUserData[s].position3D = aJoints[s].getPosition();
						mUserTracker.convertJointCoordinatesToDepth( 
										oneUserData[s].position3D.x,
										oneUserData[s].position3D.y,
										oneUserData[s].position3D.z,
										&(oneUserData[s].position2D.x), 
										&(oneUserData[s].position2D.y) );

						oneUserData[s].jointConfidence = aJoints[s].getPositionConfidence();
					}
					detectedUser.push_back(oneUserData);
				}
			}
		}
		niImageStructTemp.Users = detectedUser;

		/*	Critical Section	*/
		setShareVariable(niImageStructTemp);

		//cout << "Captured image" << endl;
		cv::waitKey(10);
	}

	/*	Close Deviec	*/
	depth.stop();
	depth.destroy();
	color.stop();
	color.destroy();
	mUserTracker.destroy();
	device.close();
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
	cout << "Stop Capture" << endl;

	return;
}

/**
* Function Name:	startCapture
* Function Purpose:	Call the thread function to start captrue images and user inform
*/
const int NiImageCaptor::startCapture()
{
	if(isStart==false)
	{
		startThread();
		while(isStart==false)
			Sleep(1000);
	}
	else 
		cout << "NiImageCaptor already initialized" << endl;
	return 0;
}

/**
* Function Name:	stopCapture
* Function Purpose:	Close the capture thread and close the device
*/
const int NiImageCaptor::stopCapture()
{
	if(isStart == true)
	{
		/*	Stop the Captured Thread	*/
		isStart = false;
		cv::waitKey(5000);
		stopThread();

		/*	Reconnect Device	*/
		//const char* deviceURI = openni::ANY_DEVICE;
		//if (device.open(deviceURI) != openni::STATUS_OK)
		//{
		//	cout << "Device open failed:" << openni::OpenNI::getExtendedError() << endl;
		//	openni::OpenNI::shutdown();
		//	return 1;
		//}

		/*	Close Deviec	*/
		//depth.stop();
		//depth.destroy();
		//color.stop();
		//color.destroy();
		//mUserTracker.destroy();
		//device.close();
		//nite::NiTE::shutdown();
		//openni::OpenNI::shutdown();
		
		//cout << "Stop Capture" << endl;
	}
	else 
		cout << "NiImageCaptor dosen't start yet" << endl;
	return 0;
}

/**
* Function Name:	drawImg
* Function Purpose:	Show the captured result and use for debugging
*/
const int NiImageCaptor::drawImg()
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
		//cv::line( niImageStructTemp.Color, niImageStructTemp.Users[i][ 9].position2D, niImageStructTemp.Users[i][11].position2D, cv::Scalar( 255, 0, 0 ), 3 );
		//cv::line( niImageStructTemp.Color, niImageStructTemp.Users[i][10].position2D, niImageStructTemp.Users[i][12].position2D, cv::Scalar( 255, 0, 0 ), 3 );
		//cv::line( niImageStructTemp.Color, niImageStructTemp.Users[i][11].position2D, niImageStructTemp.Users[i][13].position2D, cv::Scalar( 255, 0, 0 ), 3 );
		//cv::line( niImageStructTemp.Color, niImageStructTemp.Users[i][12].position2D, niImageStructTemp.Users[i][14].position2D, cv::Scalar( 255, 0, 0 ), 3 );
		
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
	}

	//cv::imshow( "Depth Image", niImageStructTemp.Depth );
	cv::imshow( "Color Image", niImageStructTemp.Color );
	cv::waitKey(10);
	return 0;
}