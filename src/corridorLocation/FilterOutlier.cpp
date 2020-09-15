//#include "stdafx.h"
#include "FilterOutlier.h"


CFilterOutlier::CFilterOutlier()
{
}

CFilterOutlier::~CFilterOutlier()
{
}
bool CFilterOutlier::removeByX(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, double orientation, cv::Point3d centerPoint)
{
	std::vector<cv::Point2d> rigmatchPt2dArray;
	std::vector<cv::Point3d> rigMatchPt3dArray;

	for (int i = 0; i < matchPt3dArray.size(); i++)
	{
		double X = matchPt3dArray.at(i).x;
		double Y = matchPt3dArray.at(i).y;
		if(fabs(X + 7) < 0.5 || fabs(X - 7) < 0.5 
			|| fabs(Y + 3.5) < 0.5 || fabs(Y - 3.5) < 0.5)		
		{
			rigmatchPt2dArray.push_back(matchPt2dArray.at(i));
			rigMatchPt3dArray.push_back(matchPt3dArray.at(i));
		}

	}
	matchPt2dArray.swap(rigmatchPt2dArray);
	matchPt3dArray.swap(rigMatchPt3dArray);

	return true;
}


double  CFilterOutlier::estimateOri(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, double orientation, cv::Point3d& centerPoint)
{
	double sumAngle = 0.0;
	for (int i = 0; i < matchPt3dArray.size(); i++)
	{
		double delX = matchPt3dArray.at(i).x - centerPoint.x;
		double delY = matchPt3dArray.at(i).y - centerPoint.y;
		double angleLine = atan2(delY, delX) / 3.1415 * 180;
		sumAngle += angleLine;
	}
	double curAngle = sumAngle / matchPt3dArray.size();
	sumAngle = 0.0;
	int count = 0;
	for (int i = 0; i < matchPt3dArray.size(); i++)
	{
		double delX = matchPt3dArray.at(i).x - centerPoint.x;
		double delY = matchPt3dArray.at(i).y - centerPoint.y;
		double angleLine = atan2(delY, delX) / 3.1415 * 180;
		
		double diffAngle = angleLine - curAngle;
		if (diffAngle < -180)		diffAngle += 360;
		if (diffAngle > 180)    	diffAngle -= 360;
		if (abs(diffAngle) < 70) //hard code
		{
			sumAngle += angleLine;
			count++;
		}
	}
	curAngle = sumAngle / count;
	return curAngle;
}
bool CFilterOutlier::run(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray,double& orientation, cv::Point3d& centerPoint)
{
	std::vector<cv::Point2d> rigmatchPt2dArray;
	std::vector<cv::Point3d> rigMatchPt3dArray;
	//double estAngle = estimateOri(matchPt2dArray, matchPt3dArray, orientation, centerPoint);
	//orientation = estAngle - 90;
	double estAngle = orientation + 90;
	for (int i = 0; i < matchPt3dArray.size(); i++)
	{
		double delX = matchPt3dArray.at(i).x - centerPoint.x;
		double delY = matchPt3dArray.at(i).y - centerPoint.y;
		double angleLine = atan2(delY, delX) / 3.1415 * 180;
		double diffAngle = angleLine - estAngle;
		if (diffAngle < -180)		diffAngle += 360;		
		if (diffAngle > 180)    	diffAngle -= 360;
	

		if ( abs(diffAngle)<45  )
		{
			rigmatchPt2dArray.push_back(matchPt2dArray.at(i));
			rigMatchPt3dArray.push_back(matchPt3dArray.at(i));
		}
	
	}
	matchPt2dArray.swap(rigmatchPt2dArray);
	matchPt3dArray.swap(rigMatchPt3dArray);

	return true;
}
