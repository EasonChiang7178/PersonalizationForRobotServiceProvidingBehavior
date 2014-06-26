/* Standrad Included Library */
#include <iostream>
#include <vector>
#include <list>
using namespace std;

/* Third-party Library */
	// OpenCV2
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

class FaceDetection {
	public:
		/* Initialization */
		FaceDetection();
		FaceDetection(const String pathToFrontalFaceXML, const String pathToProfileFaceXML, const String pathToUpperBodyXML);
		FaceDetection(const String pathToFrontalFaceXML, const String pathToProfileFaceXML, const String pathToUpperBodyXML, const int minBodySize, const int minFaceSize);
		FaceDetection(const String pathToFrontalFaceXML, const String pathToProfileFaceXML, const String pathToUpperBodyXML, const String pathToEyeXML,
					  const int minBodySize, const int maxBodySize, const int minFaceSize, const int maxFaceSize, const int minEyeSize, const int maxEyeSize);
		~FaceDetection();
			// Load trained XML data
		const bool loadFrontalFace(const String pathToFrontalFaceXML);
		const bool loadProfileFace(const String pathToProfileFaceXML);
		const bool loadUpperBody(const String pathToUpperBodyXML);
		const bool loadEye(const String pathToEyeXML);
		const bool load(const String pathToFrontalFaceXML, const String pathToProfileFaceXML, const String pathToUpperBodyXML);
		const bool load(const String pathToFrontalFaceXML, const String pathToProfileFaceXML, const String pathToUpperBodyXML, const String pathToEyeXML);

		/* Detectioin */
			// Multi stage detect face
		const bool detectFaceAndDirection();
		const bool detectFaceAndDirection(const Mat& imageToProcess);
			// Detect UpperBody
		const bool detectUpperBody();
		const bool detectUpperBody(const Mat& imageToProcess);
			// Detect FrontalFace
		const bool detectFrontFace();
		const bool detectFrontFace(const Mat& imageToProcess);
			// Detect ProfileFace
		const bool detectProfileFace();
		const bool detectProfileFace(const Mat& imageToProcess);
			// Detect Eye
		const bool detectEye();
		const bool detectEye(const Mat& imageToProcess);

		/* Draw result on image */
		const int drawAllResult(Mat& imageToProcess);
			// Draw frontal face
		const int drawFrontalFace(Mat& imageToProcess);
			// Draw Profile face
		const int drawProfileFace(Mat& imageToProcess);
			// Draw upper body
		const int drawUpperBody(Mat& imageToProcess);
			// Draw eye
		const int drawEye(Mat& imageToProcess);

		/* Set Parameter */
			// Pass image to data member
		const int setImage(Mat& imageToProcess) { img = imageToProcess; imgGray = Preprocessing(img); };
			// Set the minimun size of the possible upper body, face and eye in the image
		const int setUpperBodyMinimumSize(int minimumDetectedSize) { minimumUpperBodySize = minimumDetectedSize; return 1; }
		const int setFaceMinimumSize(int minimumDetectedSize) { minimumFaceSize = minimumDetectedSize; return 1; }
		const int setEyeMinimumSize(int minimumDetectedSize) { minimumEyeSize = minimumDetectedSize; return 1; }
			// Set the maximum size of the possible upper body, face or eye in the image
		const int setUpperbodyMaximumSize(int maximumDetectedSize) { maximumUpperBodySize = maximumDetectedSize; return 1; }
		const int setFaceMaximumSize(int maximumDetectedSize) { maximumFaceSize = maximumDetectedSize; return 1; }
		const int setEyeMaximumSize(int maximumDetectedSize) { maximumEyeSize = maximumDetectedSize; return 1; }

		/* Get informatioin */
			// Get number of humans, counted by UpperBody
		const int getHumanCountUpperBody(){ return static_cast< int >(resultUpperBody.size()); }
			// Get number of humans, counted by FrontalFace
		const int getHumanCountFrontalFace(){ return static_cast< int >(resultFrontalFace.size()); }
			// Get number of humans, counted by ProfileFace
		const int getHumanCountProfileFace(){ return static_cast< int >(resultProfileFace.size()); }
			// Get number of humans, counted by AllFace
		const int getHumanCountFace(){ return static_cast< int >(resultFrontalFace.size() + resultProfileFace.size()); }
			// Get number of humans, counted by Eye
		const int getCountEye() { return static_cast< int >(resultEye.size()); }
			// Get result of frontal face
		vector< Rect > getResultFrontalFace() { return resultFrontalFace; }
			// Get result of profile face
		vector< Rect > getResultProfileFace() { return resultProfileFace; }
			// Get result of upper body
		vector< Rect > getResultUpperBody() { return resultUpperBody; }
			// Get result of eye
		vector< Rect > getResultEye() { return resultEye; }
			// Get image processed
		Mat& getImage() { return img; }

		/* For face direction */
			// To evaluate the face direction just utilizing frontal, profile face detection
		const int calculateFaceDirection();
			// Output the faceDirection
		const int getFaceDirection() { return faceDirection; }

	private:
		Mat img;
		Mat imgGray;
			// Preprocess: Gray conversion and Normalization
		Mat Preprocessing(const Mat& imageToProcess);
		/* Parameters for cascade classifiers */
		int minimumUpperBodySize;
		int maximumUpperBodySize;
		int minimumFaceSize;
		int maximumFaceSize;
		int minimumEyeSize;
		int maximumEyeSize;

		/* CascadeClassifiers for face detection */
		CascadeClassifier frontalFace_cascade;
		CascadeClassifier profileFace_cascade;
		CascadeClassifier upperBody_cascade;
		CascadeClassifier eye_cascade;

		/* Vector to store the classifiers' result */
		vector< Rect > resultFrontalFace;
		vector< Rect > resultProfileFace;
		vector< Rect > resultUpperBody;
		vector< Rect > resultEye;

			// For Human Attention Estimator (HAE)
		int faceDirection;
		int detectCount;
		list< int > historyData;
};