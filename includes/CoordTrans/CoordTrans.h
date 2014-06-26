
#ifndef COORDTRANS_H
#define COORDTRANS_H

#include <cmath>
#include <vector>
#include <map>

#define VISUALIZE

#ifdef VISUALIZE
#pragma warning(push,0)
#include "cv.h" 
#include "cxcore.h"
#include "highgui.h"
#include "cvaux.h"
#include "ml.h"
#pragma warning(pop)
#endif

#define PI 3.1415926

using namespace std;

class CoordTrans
{
	public:
			// Coordinate Transformation between 2-D plane
		int trans2D(const double x, const double y, const double theta,
					const double transX, const double transY, const double transRotate,
					double &newX, double &newY, double &newTheta);

#ifdef VISUALIZE
		/* Visualize the coordinate transformation */
			// 2-D plane to Image coordinate, unit of 2-D plane: meter (m)
		int plane2Img(const double x, const double y, int &imgX, int &imgY);
			// Xtion pro coordinate to Image coordinate
		int xtion2Img(const float xtionX, const float xtionZ, const float xtionTheta,
					  int &imgX, int &imgY, float &imgTheta);
			// Insert the point that wants to visualize
		int insertPoint(int x, int y){pointVect.push_back(cv::Point(x,y)); return 0;};
			// Draw the points stored in pointVect
		int drawPoint();
		
		/* Codes for Chu's thesis problem */
		int drawMatchResult(const vector<vector<float>> &a,
							const vector<vector<float>> &b,
							const map<int,int> &m);

		vector<vector<double>> trans2Drobot2host(const vector<vector<float>> &robotHumanPos, 
												const float transX, const float transY, const float transTheta);

	private:
			// For points to draw
		vector<cv::Point> pointVect;
#endif

};

#endif