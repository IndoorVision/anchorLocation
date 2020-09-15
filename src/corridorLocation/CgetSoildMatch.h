#pragma once
#include <opencv2/opencv.hpp>
#include "opencv2/opencv_modules.hpp"
#include <set>
#include <algorithm>
#include <iterator> 
#include<numeric>

#include <vector>
#include <fstream>
#include <string>


struct HcullAnchorworkGroup
{
	std::string anchorName;
	std::vector<cv::Point2d> matchPt2dArray_work;
	std::vector<cv::Point3d> matchPt3dArray_work;
	std::vector<cv::Point2d> matchPt2dArray_StandardPicture_work;
	std::vector<float> distanceArray;

	cv::Point3d centerPoint;
};

typedef std::pair<std::string, int> PAIR;

class CgetSoildMatch
{
public:
	CgetSoildMatch(int compareNum, int minMeanNum);
	CgetSoildMatch();
	~CgetSoildMatch();

	 void getWorkGroup(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, std::vector<cv::Point2d>& matchPt2dArray_StandardPicture, std::vector<std::string> &matchanchorNameArray, std::vector<float>& distanceArray);
	 void DistanceMeanSort_getSoildMatch(std::vector<cv::Point2d>& out_matchPt2dArray, std::vector<cv::Point3d>& out_matchPt3dArray);
	 void smallDisatanceCntSort_getSoildMatch(std::vector<cv::Point2d>& out_matchPt2dArray, std::vector<cv::Point3d>& out_matchPt3dArray);
	 void smallDisatanceCntSort_moreAnchor_getSoildMatch(std::vector<cv::Point2d>& out_matchPt2dArray, std::vector<cv::Point3d>& out_matchPt3dArray);

private:
	int m_compareNum = 20;
	int m_minMeanNum = 2;
	std::vector<HcullAnchorworkGroup> m_workGroupArray;


	void DistanceMean(std::vector<HcullAnchorworkGroup>& workGroupArray, std::vector<float>& meanDistanceArray, std::vector<size_t>& Sortindex);
	bool maxNumString(std::vector<std::string> testList, std::vector<std::string>& maxStringArray, std::vector<int>& maxStringCountArray);
	static bool cmp_val(const PAIR &left, const PAIR &right);
	int isInNameArray(std::string in_anchorName, std::vector<HcullAnchorworkGroup> workGroupArray);
	int findVerySmallDistanceMatch(std::vector<float> distanceArray);
	void smallEnoughDisatanceCount(std::vector<HcullAnchorworkGroup>& workGroupArray, float smallEnoughDistace, std::vector<int>& smallEnoughDisatanceCountArray, std::vector<size_t>& Sortindex);
	bool Homography_cullError(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, std::vector<cv::Point2d>& matchPt2dArray_OnstandardPicutre);
	double getDistance(cv::Point3d pointO, cv::Point3d pointA);
};

