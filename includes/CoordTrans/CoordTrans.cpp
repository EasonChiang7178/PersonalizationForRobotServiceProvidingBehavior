
#include "CoordTrans.h"

#include <iostream>
#include <sstream>
#include <map>

using namespace std;

int CoordTrans::trans2D(const double x, const double y, const double theta,
						const double transX, const double transY, const double transTheta,
						double &newX, double &newY, double &newTheta)
{
	float xTemp, yTemp;
	float rad = static_cast<float>(transTheta * PI / 180.0);
	xTemp = x - transX;
	yTemp = y - transY;
	newX = (xTemp * cos(rad) + yTemp * sin(rad));
	newY = (-xTemp * sin(rad) + yTemp * cos(rad));
	newTheta = theta - transTheta;
	return 0;
}

#ifdef VISUALIZE

int CoordTrans::plane2Img(const double x, const double y, int &imgX, int &imgY) {
	imgX = 320 - static_cast< int >(y * 100);
	imgY = 480 - static_cast< int >(x * 100);
	return 0;
}

int CoordTrans::xtion2Img(const float xtionX, const float xtionZ, const float xtionTheta,
						int &imgX, int &imgY, float &imgTheta)
{
	imgX = 320 - static_cast<int>(xtionX/10.0);
	if(imgX<10) imgX = 10;
	if(imgX>629) imgX = 629;

	imgY = 480 - static_cast<int>(xtionZ/10.0);
	if(imgY<10) imgY = 10;
	if(imgY>469) imgY = 469;
	
	imgTheta = 180.0f - xtionTheta;

	if(imgTheta< 0.0)
		imgTheta += 360.0f;
	if(imgTheta> 359.0)
		imgTheta -= 360.0f;
	return 0;
}


int CoordTrans::drawMatchResult(const vector<vector<float>> &a,
						const vector<vector<float>> &b,
						const map<int,int> &m)
{
	cv::Mat image(480, 640, CV_8UC3, cv::Scalar(255,255,255));
	
	cv::line(image, cv::Point(320,479), cv::Point(300,479), cv::Scalar(0,0,0), 4);
	cv::line(image, cv::Point(320,479), cv::Point(320,459), cv::Scalar(0,0,0), 4);

	if(m.size()<1)
	{
		cv::imshow("image", image);
		cv::waitKey(100);
		return -1;
	}

	stringstream ss;
	int imgX, imgY;
	int imgX2, imgY2;
	float imgTheta;
	cv::Point centerP, thetaP;

	for(unsigned int i(0); i<a.size(); i++)
	{
		xtion2Img(a[i][0], a[i][1], a[i][2], imgX, imgY, imgTheta);
		circle(image, cv::Point(imgX, imgY), 10, cv::Scalar(255, 0, 0),2);
		// draw body direction line
		imgX2 = imgX + static_cast<int>(cos(imgTheta*PI/180.0)*10.0);
		imgY2 = imgY + static_cast<int>(sin(imgTheta*PI/180.0)*10.0);
		centerP = cv::Point(imgX, imgY);
		thetaP = cv::Point(imgX2, imgY2);
		cv::line(image, centerP, thetaP, cv::Scalar(255,0,0), 2);
	}

	for(unsigned int i(0); i<b.size(); i++)
	{
		xtion2Img(b[i][0], b[i][1], b[i][2] ,imgX, imgY, imgTheta);
		circle(image, cv::Point(imgX, imgY), 10, cv::Scalar(0, 255, 0),2);
		// draw body direction line
		imgX2 = imgX + static_cast<int>(cos(imgTheta*PI/180.0)*10.0);
		imgY2 = imgY + static_cast<int>(sin(imgTheta*PI/180.0)*10.0);
		centerP = cv::Point(imgX, imgY);
		thetaP = cv::Point(imgX2, imgY2);
		cv::line(image, centerP, thetaP, cv::Scalar(0,255,0), 2);
	}

	for(map<int,int>::const_iterator it = m.begin(); it != m.end() ; it++)
	{
		xtion2Img(a[it->first][0], a[it->first][1], a[it->first][2], imgX, imgY, imgTheta );
		xtion2Img(b[it->second][0], b[it->second][1], b[it->second][2], imgX2, imgY2, imgTheta );
		cv::line(image, cv::Point(imgX, imgY), cv::Point(imgX2, imgY2), cv::Scalar(0,0,255), 2);
	}

	cv::imshow("image", image);
	cv::waitKey(100);

	return 0;
}


int CoordTrans::drawPoint()
{
	cv::Mat image(480, 640, CV_8UC3, cv::Scalar(255,255,255));

	cv::line(image, cv::Point(320,479), cv::Point(300,479), cv::Scalar(0,0,0), 4);
	cv::line(image, cv::Point(320,479), cv::Point(320,459), cv::Scalar(0,0,0), 4);

	for(vector<cv::Point>::iterator it=pointVect.begin(); it!=pointVect.end(); it++)
		circle(image, *it, 10, cv::Scalar(255, 0, 0),5);

	cv::imshow("image", image);
	cv::waitKey(100);

	return 0;
}


vector<vector<double>> CoordTrans::trans2Drobot2host(const vector<vector<float>> &basedHumanPos,
							const float transX, const float transY, const float transTheta)
{

	vector<vector<double>> transformed;
	vector<double> onLine(3);

	if(basedHumanPos.size()<1)
	{
		cout << "No people" << endl;
		return transformed;
	}

	for(unsigned int i(0); i<basedHumanPos.size(); i++)
	{
		//trans2D(float x, float y, float transX, float transY, float rotate, float &xBar, float &yBar);
		trans2D(basedHumanPos[i][0], basedHumanPos[i][1], basedHumanPos[i][2],
				transX, transY, transTheta,
				onLine[0], onLine[1],  onLine[2]);

		transformed.push_back(onLine);
	}

	return transformed;
}

#endif