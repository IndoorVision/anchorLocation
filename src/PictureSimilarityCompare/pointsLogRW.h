#pragma once


#include "fstream"
#include "iostream"
#include "opencv2\core.hpp"

struct POINT_PAIR
{
	double x1;
	double y1;
	double x2;
	double y2;
};

class pointsLogRW
{
public:
	pointsLogRW();
	~pointsLogRW();
	
	static bool writePly(char* filename, std::vector<cv::Point3d>& pt3dArray);
	static bool writePts(char* filename, std::vector<cv::Point2d>& pt2dArray, std::vector<cv::Point3d>& pt3dArray);
	static bool readPTO(const char* ptoFileName, std::vector<POINT_PAIR>& ptPairArray);
	static bool writePTO(const char* ptoFileName, std::vector<POINT_PAIR>& ptPairArray, std::string imgFileName1, std::string imgFileName2 );
};

