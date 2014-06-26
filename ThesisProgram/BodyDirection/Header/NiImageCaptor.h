/*************************************************************************//**

	@file		NiImageCaptor.h
	@authors	Ting-Sheng Chu
	@version	1.10
	@date		2014-03
	@copyright	GNU Public License.
	
	@brief		This is a class for Xtion pro Live to capture the depth image
				and color(RGB) image as well as the user information using openNI
				user tracking library. It shows the result by openCV 
				and optimized using multi-thread technic.

	@pre		Windows_API(for win thread), openNI2(x86), NITE(2.2), openCV(2.4.5) 
	@bug		Currently no bug found yet

	@section Revise_Log

	14/03/20	Create the class.	\n
	14/03/20	Modify the isStart flag to optimaize initial time.	\n

****************************************************************************/

#ifndef NIIMAGECAPTOR_H
#define NIIMAGECAPTOR_H

#pragma warning(push,0)
#include "cv.h" 
#include "cxcore.h"
#include "highgui.h"
#include "cvaux.h"
#include "ml.h"
#pragma warning(pop)

/*	STL Header	*/
#include <iostream>
#include <vector>
 
/*	OpenNI Header	*/
#include <OpenNI.h>
  
/*	NiTE Header	*/
#include <NiTE.h>

/*	User Define Header	*/
#include "ChuThread.h"

using namespace std;
using namespace chuThreadNamespace;

struct JointStruct		// struct for saving joint inforomation
{
	nite::Point3f position3D;
	cv::Point2f position2D;
	float	jointConfidence;
};

struct NiImageStruct	// struct for saving image and user informaiton form openni
{
	cv::Mat Depth;
	cv::Mat Color;
	vector<vector<JointStruct>>  Users;
};

class NiImageCaptor : public ChuThread<int, NiImageStruct>
{

public:

	NiImageCaptor();
	~NiImageCaptor();				/// Close the device if it hasn't been closed

	void thread();					/// Thread for capturing image from Xtion pro

	const int startCapture();		/// Initialize Xtion pro and start capture frame
	const int stopCapture();		/// Stop capture frame and close device

	const int drawImg();			/// Show the captured depth, color, and user image

	NiImageStruct getNiImageStruct()/// Get the getNiImage Struct form shared variable
	{return getShareVariable();};

private:

	/*	Static Flag	*/
	static bool isStart;			// the initialize flag as well as start/end flag

	/* OpenNI Object	*/
	openni::Device  device;					// xtion pro device object
	openni::VideoStream depth;				// for depth stream
	openni::VideoMode mModeDepth;			// setting depthe stream mode
	openni::VideoStream color;				// for color stream
	openni::VideoMode mModeColor;			// setting color stream mode
	nite::UserTracker mUserTracker;			// openni user tracker object

};

#endif