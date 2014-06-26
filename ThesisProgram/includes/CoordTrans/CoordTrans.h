
#ifndef COORDTRANS_H
#define COORDTRANS_H

#include <cmath>
#include <vector>
#include <map>

#pragma warning(push,0)
#include "cv.h" 
#include "cxcore.h"
#include "highgui.h"
#include "cvaux.h"
#include "ml.h"
#pragma warning(pop)

#define PI 3.1415926

using namespace std;

class CoordTrans
{

public:

	int trans2D(const float x, const float y, const float theta,
				const float transX, const float transY, const float transRotate,
				float &newX, float &newY, float &newTheta);
	
	int xtion2Img(const float xtionX, const float xtionZ, const float xtionTheta,
					int &imgX, int &imgY, float &imgTheta);
	int insertPoint(int x, int y){pointVect.push_back(cv::Point(x,y)); return 0;};
	int drawPoint();
	
	int drawMatchResult(const vector<vector<float>> &a,
						const vector<vector<float>> &b,
						const map<int,int> &m);


	vector<vector<float>> trans2Drobot2host(const vector<vector<float>> &robotHumanPos, 
											const float transX, const float transY, const float transTheta);

private:

	vector<cv::Point> pointVect;


};



#endif