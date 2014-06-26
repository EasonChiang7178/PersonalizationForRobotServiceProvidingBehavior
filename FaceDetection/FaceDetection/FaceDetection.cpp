#include "FaceDetection.h"

/* Initialization */
FaceDetection::FaceDetection()
{
	setUpperBodyMinimumSize(60);
	setFaceMinimumSize(30);

	detectCount = 0;
}

FaceDetection::FaceDetection(const String pathToFrontalFaceXML, const String pathToProfileFaceXML, const String pathToUpperBodyXML)
{
	this->load(pathToFrontalFaceXML, pathToProfileFaceXML, pathToUpperBodyXML);
	setUpperBodyMinimumSize(60);
	setFaceMinimumSize(30);
	setEyeMinimumSize(20);
	setUpperbodyMaximumSize(0);
	setFaceMaximumSize(0);
	setEyeMaximumSize(0);

	detectCount = 0;
}

FaceDetection::FaceDetection(const String pathToFrontalFaceXML, const String pathToProfileFaceXML, const String pathToUpperBodyXML,
							 const int minBodySize, const int minFaceSize)
{
	this->load(pathToFrontalFaceXML, pathToProfileFaceXML, pathToUpperBodyXML);
	setUpperBodyMinimumSize(minBodySize);
	setFaceMinimumSize(minFaceSize);
	setEyeMinimumSize(0);
	setUpperbodyMaximumSize(0);
	setFaceMaximumSize(0);
	setEyeMaximumSize(0);

	detectCount = 0;
}

FaceDetection::FaceDetection(const String pathToFrontalFaceXML, const String pathToProfileFaceXML, const String pathToUpperBodyXML, const String pathToEyeXML,
							 const int minBodySize, const int maxBodySize, const int minFaceSize, const int maxFaceSize, const int minEyeSize, const int maxEyeSize)
{
	this->load(pathToFrontalFaceXML, pathToProfileFaceXML, pathToUpperBodyXML, pathToEyeXML);
	setUpperBodyMinimumSize(minBodySize);
	setUpperbodyMaximumSize(maxBodySize);
	setFaceMinimumSize(minFaceSize);
	setFaceMaximumSize(maxFaceSize);
	setEyeMinimumSize(minEyeSize);
	setEyeMaximumSize(maxEyeSize);

	detectCount = 0;
}

FaceDetection::~FaceDetection()
{
	resultFrontalFace.clear(); resultFrontalFace.resize(0);
	resultProfileFace.clear(); resultProfileFace.resize(0);
	resultUpperBody.clear();   resultUpperBody.resize(0);
	resultEye.clear();		   resultEye.resize(0);
}

const bool FaceDetection::loadFrontalFace(const String pathToFrontalFaceXML)
{
	if (!frontalFace_cascade.load(pathToFrontalFaceXML)) {
		cout << "> ERROR: loading frontal face cascade: " << pathToFrontalFaceXML
			 << "         Please press ENTER to continue...";
		getchar();
		return false;
	}
	return true;
}

const bool FaceDetection::loadProfileFace(const String pathToProfileFaceXML)
{
	if (!profileFace_cascade.load(pathToProfileFaceXML)) {
		cout << "> ERROR: loading profile face cascade: " << pathToProfileFaceXML
			 << "         Please press ENTER to continue...";
		getchar();
		return false;
	}
	return true;
}

const bool FaceDetection::loadUpperBody(const String pathToUpperBodyXML) {
	if (!upperBody_cascade.load(pathToUpperBodyXML)) {
		cout << "> ERROR: loading upper body cascade: " << pathToUpperBodyXML
			 << "         Please press ENTER to continue...";
		getchar();
		return false;
	}
	return true;
}

const bool FaceDetection::loadEye(const String pathToEyeXML) {
	if (!eye_cascade.load(pathToEyeXML)) {
		cout << "> ERROR: loading eye cascade: " << pathToEyeXML
			 << "         Please press ENTER to continue...";
		getchar();
		return false;
	}
	return true;
}

const bool FaceDetection::load(const String pathToFrontalFaceXML, const String pathToProfileFaceXML, const String pathToUpperBodyXML)
{
	bool result = true;
	result = this->loadFrontalFace(pathToFrontalFaceXML);
	result = this->loadProfileFace(pathToProfileFaceXML);
	result = this->loadUpperBody(pathToUpperBodyXML);

	return result;
}

const bool FaceDetection::load(const String pathToFrontalFaceXML, const String pathToProfileFaceXML, const String pathToUpperBodyXML, const String pathToEyeXML)
{
	bool result = true;
	result = this->loadFrontalFace(pathToFrontalFaceXML);
	result = this->loadProfileFace(pathToProfileFaceXML);
	result = this->loadUpperBody(pathToUpperBodyXML);
	result = this->loadEye(pathToEyeXML);

	return result;
}

	// Multi stage detect face
const bool FaceDetection::detectFaceAndDirection()
{
	return this->detectFaceAndDirection(imgGray);
}

const bool FaceDetection::detectFaceAndDirection(const Mat& imageToProcess)
{
	resultFrontalFace.clear(); resultFrontalFace.resize(0);
	resultProfileFace.clear(); resultProfileFace.resize(0);
	resultUpperBody.clear();   resultUpperBody.resize(0);
	resultEye.clear();		   resultEye.resize(0);

	Mat tempGray = imageToProcess;
	if (imageToProcess.channels() != 1)
		tempGray = this->Preprocessing(imageToProcess);

	/* Detect Upper Body */
	if (this->detectUpperBody(tempGray) == false) {
		cout << "> WARNING: No people detected" << endl;
		return false;
	}

	vector< Rect > tempFFace;
	vector< Rect > tempPFace;
	vector< Rect > tempEye;
	for (auto it = resultUpperBody.begin(); it != resultUpperBody.end(); it++) {
		Mat humanROI = tempGray(*it);

		/* Detect Frontal Face */
		if (this->detectFrontFace(humanROI) == true) {
				// Offset back image pixels
			for (auto itFace = resultFrontalFace.begin(); itFace != resultFrontalFace.end(); itFace++) {
				itFace->x += it->x;
				itFace->y += it->y;

				/* Detect Eye */
				Mat FrontFaceROI = tempGray(*itFace);
				if (this->detectEye(FrontFaceROI) == true) {
						// Offset back image pixels
					for (auto itEye = resultEye.begin(); itEye != resultEye.end(); itEye++) {
						itEye->x += itFace->x;
						itEye->y += itFace->y;
						tempEye.push_back(*itEye);
					}
				}
			}
			tempFFace.push_back(resultFrontalFace[0]);
			continue;
		}

		/* Detect Profile Face */
			// Right Profile Face
		if (this->detectProfileFace(humanROI) == false) {
				// Try left Profile Face
			flip(humanROI, humanROI, 1);
			this->detectProfileFace(humanROI);
		}
			// Offset backe image pixels
		for (auto itFace = resultProfileFace.begin(); itFace != resultProfileFace.end(); itFace++) {
			itFace->x += it->x;
			itFace->y += it->y;
		}
		if (resultProfileFace.size() > 0)
			tempPFace.push_back(resultProfileFace[0]);
	}
	resultFrontalFace = tempFFace;
	resultProfileFace = tempPFace;
	resultEye = tempEye;

	return (resultFrontalFace.size() + resultProfileFace.size() > 0) ? true : false;
}

/* Detect UpperBody */
const bool FaceDetection::detectUpperBody()
{
	return this->detectUpperBody(imgGray);
}

const bool FaceDetection::detectUpperBody(const Mat& imageToProcess)
{
	resultUpperBody.clear();   resultUpperBody.resize(0);

	Mat tempGray = imageToProcess;
	if (imageToProcess.channels() != 1)
		tempGray = this->Preprocessing(imageToProcess);

	upperBody_cascade.detectMultiScale(tempGray, resultUpperBody, 1.1, 1, 0|CASCADE_SCALE_IMAGE, Size(minimumUpperBodySize, minimumUpperBodySize), Size(maximumUpperBodySize, maximumUpperBodySize));

	return (resultUpperBody.size() > 0) ? true : false;
}

/* Detect FrontalFace */
const bool FaceDetection::detectFrontFace()
{
	return this->detectFrontFace(imgGray);
}

const bool FaceDetection::detectFrontFace(const Mat& imageToProcess)
{
	resultFrontalFace.clear(); resultFrontalFace.resize(0);

	Mat tempGray = imageToProcess;
	if (imageToProcess.channels() != 1)
		tempGray = this->Preprocessing(imageToProcess);

	frontalFace_cascade.detectMultiScale(tempGray, resultFrontalFace, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(minimumFaceSize, minimumFaceSize), Size(maximumFaceSize, maximumFaceSize));

	return (resultFrontalFace.size() > 0) ? true : false;
}

/* Detect ProfileFace */
const bool FaceDetection::detectProfileFace()
{
	return this->detectProfileFace(imgGray);
}

const bool FaceDetection::detectProfileFace(const Mat& imageToProcess)
{
	resultProfileFace.clear(); resultProfileFace.resize(0);

	Mat tempGray = imageToProcess;
	if (imageToProcess.channels() != 1)
		tempGray = this->Preprocessing(imageToProcess);

	profileFace_cascade.detectMultiScale(tempGray, resultProfileFace, 1.1, 1, 0|CASCADE_SCALE_IMAGE, Size(minimumFaceSize, minimumFaceSize), Size(maximumFaceSize, maximumFaceSize));

	return (resultProfileFace.size() > 0) ? true : false;
}

/* Detect Eye */
const bool FaceDetection::detectEye()
{
	return this->detectEye(imgGray);
}

const bool FaceDetection::detectEye(const Mat& imageToProcess)
{
	resultEye.clear(); resultEye.resize(0);

	Mat tempGray = imageToProcess;
	if (imageToProcess.channels() != 1)
		tempGray = this->Preprocessing(imageToProcess);

	eye_cascade.detectMultiScale(tempGray, resultEye, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(minimumEyeSize, minimumEyeSize), Size(maximumEyeSize, maximumEyeSize));

	return (resultEye.size() > 0) ? true : false;
}


/* Draw result on the image */
const int FaceDetection::drawAllResult(Mat& imageToProcess)
{
	int result = true;
	result = this->drawUpperBody(imageToProcess);
	result = this->drawFrontalFace(imageToProcess);
	result = this->drawProfileFace(imageToProcess);
	result = this->drawEye(imageToProcess);
	return result;
}
	// Draw frontal face
const int FaceDetection::drawFrontalFace(Mat& imageToProcess)
{
	if (resultFrontalFace.size() != 0) {
		for (auto itFFace = resultFrontalFace.begin(); itFFace != resultFrontalFace.end(); itFFace++) {
			Point center( itFFace->x + itFFace->width/2, itFFace->y + itFFace->height/2 );
			ellipse(imageToProcess, center, Size( itFFace->width/2, itFFace->height/2 ), 0, 0, 360, Scalar(109, 55, 255), 4, 8, 0 );
			putText(imageToProcess, "FrontalFace", Point(itFFace->x + itFFace->width - 90, itFFace->y + itFFace->height + 15), CV_FONT_HERSHEY_PLAIN, 1, Scalar(109, 55, 255), 2);
		}
		return true;
	}
	return false;
}
	// Draw Profile face
const int FaceDetection::drawProfileFace(Mat& imageToProcess)
{
	if (resultProfileFace.size() != 0) {
		for (auto itPFace = resultProfileFace.begin(); itPFace != resultProfileFace.end(); itPFace++) {
			Point center( itPFace->x + itPFace->width/2, itPFace->y + itPFace->height/2 );
			ellipse(imageToProcess, center, Size( itPFace->width/2, itPFace->height/2 ), 0, 0, 360, Scalar(79, 132, 225), 4, 8, 0 );
			putText(imageToProcess, "ProfileFace", Point(itPFace->x + itPFace->width - 90, itPFace->y + itPFace->height + 15), CV_FONT_HERSHEY_PLAIN, 1, Scalar(79, 132, 225), 2);
		}
		return true;
	}
	return false;
}
	// Draw upper body
const int FaceDetection::drawUpperBody(Mat& imageToProcess)
{
	if (resultUpperBody.size() != 0) {
		for (auto itBody = resultUpperBody.begin(); itBody != resultUpperBody.end(); itBody++) {
			rectangle(imageToProcess, *itBody, Scalar(176, 104, 10), 2, 8, 0);
			putText(imageToProcess, "Upperbody", Point(itBody->x + itBody->width - 80, itBody->y + itBody->height + 15), CV_FONT_HERSHEY_PLAIN, 1, Scalar(176, 104, 10), 2);
		}
		return true;
	}

	return false;
}
	// Draw eye
const int FaceDetection::drawEye(Mat& imageToProcess)
{
	if (resultEye.size() != 0) {
		for (auto itEye = resultEye.begin(); itEye != resultEye.end(); itEye++) {
			Point eye_center(itEye->x + itEye->width/2, itEye->y + itEye->height/2);
			int radius = cvRound( (itEye->width + itEye->height)*0.25 );
			circle(imageToProcess, eye_center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );

			putText(imageToProcess, "Eye", Point(itEye->x + itEye->width - 80, itEye->y + itEye->height + 15), CV_FONT_HERSHEY_PLAIN, 1, Scalar(255,0,0), 2);
		}
		return true;
	}

	return false;
}

	// Preprocess: Gray conversion and Normalization
Mat FaceDetection::Preprocessing(const Mat& imageToProcess)
{
	Mat imageProcessed;
	cvtColor (imageToProcess, imageProcessed, COLOR_BGR2GRAY);
	equalizeHist(imageProcessed, imageProcessed);
	return imageProcessed;
}

	// To evaluate the face direction just utilizing frontal, profile face detection
const int FaceDetection::calculateFaceDirection()
{
	if (resultFrontalFace.size() != 0)
		historyData.push_front(2);
	else if (resultProfileFace.size() != 0)
		historyData.push_front(1);
	else
		historyData.push_front(0);

	if (detectCount++ < 6) {
		return 0;
	}

	/* Perform Gaussian smoothing */
	//double g_mask[7] = {0.004429, 0.053994, 0.242038, 0.399074, 0.242038, 0.053994, 0.004429};
	double g_mask[7] = {0.142857, 0.142857, 0.142857, 0.142857, 0.142857, 0.142857, 0.142857};
	double tempDirection = 0.0;
	int i = 0;
	for (auto it = historyData.begin(); it != historyData.end(); it++)
		tempDirection += (*it) * g_mask[i++];

	if (tempDirection > 1.3)
		faceDirection = 2;
	else if (tempDirection > 0.2)
		faceDirection = 1;
	else
		faceDirection = 0;

	historyData.pop_back();

		// Print information
	cout << "> Face Direction: " << faceDirection << endl;

	return 1;
}