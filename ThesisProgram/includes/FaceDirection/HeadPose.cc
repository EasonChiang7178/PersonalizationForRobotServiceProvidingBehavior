// HeadPose.cpp : Defines the entry point for the console application.

#include <vector>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

#define PI 3.1415926

using namespace std;
using namespace cv;

float calculateFaceAngle(float cosBetaCosGama,float sinBeta, float cosAlphaCosBeta, string &faceData)
{
	stringstream ss;

	float alpha, gamma, beta, cosGama, cosAlpha;

	beta = asin(sinBeta);		// beta = yaw

	cosGama = cosBetaCosGama/cos(beta);
	gamma = acos(cosGama);		// gamma = roll

	cosAlpha = cosAlphaCosBeta/cos(beta);
	alpha = acos(cosAlpha);		// alpha = pitch

	alpha = alpha*180.0/PI;		// rad to degree
	beta = beta*180.0/PI;
	gamma = gamma*180.0/PI;

	//printf("alpha:%f\n", alpha);
	printf("> Beta: %f\n", beta);
	//printf("gamma:%f\n", gamma);

	ss << gamma << " " << alpha << " " << beta << endl;	// set to string faceData
	faceData = ss.str();

	return 0;
}

void loadWithPoints(cv::Mat& ip, cv::Mat& img, string &faceData, Point2f &nosePoint) {

	
	double rot[9] = {0};
	vector<double> rv(3), tv(3);
	vector< Point3f > modelPoints;
	Mat op;
	Mat camMatrix;
	Mat rvec(rv),tvec(tv);

	//new model points:
	modelPoints.push_back(Point3f(2.37427,110.322,21.7776));	// l eye (v 314)
	modelPoints.push_back(Point3f(70.0602,109.898,20.8234));	// r eye (v 0)
	modelPoints.push_back(Point3f(36.8301,78.3185,52.0345));	// nose (v 1879)
	modelPoints.push_back(Point3f(14.8498,51.0115,30.2378));	// l mouth (v 1502)
	modelPoints.push_back(Point3f(58.1825,51.0115,29.6224));	// r mouth (v 695)
	modelPoints.push_back(Point3f(-61.8886,127.797,-89.4523));	// l ear (v 2011)
	modelPoints.push_back(Point3f(127.603,126.9,-83.9129));		// r ear (v 1138)+
	
	op = Mat(modelPoints);

	int max_d = MAX(img.rows,img.cols);
	camMatrix = (Mat_<double>(3,3) << max_d, 0, img.cols/2.0,
					0,	max_d, img.rows/2.0,
					0,	0,	1.0);
	//cout << "using cam matrix " << endl << camMatrix << endl;
	
	double _dc[] = {0,0,0,0};
	//debug
	//int i;
	//for (i = 0; i < 7; i++)
	//{
	//	std::cout << ip.at<cv::Point2f>(i,0).x << " " << ip.at<cv::Point2f>(i,0).y << endl;
	//}

	solvePnP(op, ip, camMatrix, Mat(1,4,CV_64FC1,_dc), rvec, tvec, false, CV_ITERATIVE);

	Mat rotM(3,3,CV_64FC1,rot);
	Rodrigues(rvec,rotM);
	double* _r = rotM.ptr<double>();
	//printf("rotation mat: \n %.3f %.3f %.3f\n%.3f %.3f %.3f\n%.3f %.3f %.3f\n",
	//	_r[0],_r[1],_r[2],_r[3],_r[4],_r[5],_r[6],_r[7],_r[8]);

	//printf("trans vec: \n %.3f %.3f %.3f\n",tv[0],tv[1],tv[2]);

	calculateFaceAngle(_r[0], _r[2], _r[8], faceData);

	double _pm[12] = {_r[0],_r[1],_r[2],tv[0],
		  	  _r[3],_r[4],_r[5],tv[1],
			  _r[6],_r[7],_r[8],tv[2]};

	/*	Drawing Check Point	*/
	Matx34d P(_pm);
	Mat KP = camMatrix * Mat(P);
	// reproject object points - check validity of found projection matrix
	for (int i=0; i<op.rows; i++) {
		Mat_<double> X = (Mat_<double>(4,1) << op.at<float>(i,0),op.at<float>(i,1),op.at<float>(i,2),1.0);
		// cout << "object point " << X << endl;
		Mat_<double> opt_p = KP * X;
		Point2f opt_p_img(opt_p(0)/opt_p(2),opt_p(1)/opt_p(2));
		// cout << "object point reproj " << opt_p_img << endl; 
		
		if (i == 2)
			nosePoint = opt_p_img;

		circle(img, opt_p_img, 4, Scalar(0,0,255), 1);
	}
	//rotM = rotM.t();// transpose to conform with majorness of opengl matrix

	return;
}



