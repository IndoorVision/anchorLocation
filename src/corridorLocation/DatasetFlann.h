#pragma once

#include <opencv2/opencv.hpp>
//#include <opencv2/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include "opencv2/opencv_modules.hpp"
#include <set>
#include <algorithm>
#include <iterator> 
#include <cmath>
#include<numeric>

#include <vector>
#include <fstream>
#include <string>

#include "CgetSoildMatch.h"

struct H_cull_AnchorworkGroup
{
	std::string anchorName;
	std::vector<cv::Point2d> matchPt2dArray_work;
	std::vector<cv::Point3d> matchPt3dArray_work;
	std::vector<cv::Point2d> matchPt2dArray_StandardPicture_work;
	std::vector<float> distanceArray;
};


class CDatasetFlann
{
	typedef std::pair<std::string, int> PAIR;
public:
	CDatasetFlann();
	~CDatasetFlann();

	
	bool load(char* filename);
	void query(cv::Mat& desc, std::vector<cv::Point2d>& pt2dArray, double ratio_threshold,
		std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray);

	void query_corridor(cv::Mat& desc, std::vector<cv::Point2d>& pt2dArray, double ratio_threshold,
		std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, std::vector<cv::Point2d>& matchPt2dArray_StandardPicture, std::vector<std::string> &matchanchorNameArray);

	bool Homography_cullError(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, std::vector<cv::Point2d>& matchPt2dArray_OnstandardPicutre);

	int getPlaceType();

	bool maxNumString(std::vector<std::string> testList, std::vector<std::string>& maxStringArray, std::vector<int>& maxStringCountArray);

	static bool cmp_val(const PAIR &left, const PAIR &right);

	bool getSoildMatch(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, std::vector<cv::Point2d>& matchPt2dArray_StandardPicture, std::vector<std::string> &matchanchorNameArray, std::vector<float>& distanceArray);


	

protected:
	//cv::flann::IndexParams* m_indexPara;
	cv::flann::Index m_flannIndex;
	std::vector<cv::Point3d> m_pt3dArray;
	std::vector<cv::Point2d> m_pt2dArray;
	std::vector<std::string> m_anchorNameArray;
	cv::Mat m_datasetDesc;
	void buildIndex(const cv::Mat&describ1, int imethod);
	int m_placeType = 0;  // 0 ·¿¼ä£¬ 1 ×ßÀÈ

	bool match(cv::Mat& queryDesc, std::vector<cv::DMatch>& matches, double ratio_threshold);
	bool match_corridor(cv::Mat& queryDesc, std::vector<cv::DMatch>& matches, double ratio_threshold);
	bool match_corridor(cv::Mat& queryDesc, std::vector<cv::DMatch>& matches, double ratio_threshold, std::vector<cv::Point2d>& pt2dArray,
		std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, std::vector<cv::Point2d>& matchPt2dArray_StandardPicture, std::vector<std::string> &matchanchorNameArray);
	bool is_element_in_vector(std::vector<cv::Point3d> v, cv::Point3d element);
	bool is_element_in_vector(std::vector<cv::Point2d> v, cv::Point2d element);

	int isInNameArray(std::string in_anchorName, std::vector<H_cull_AnchorworkGroup> workGroupArray);
	void DistanceMean(std::vector<H_cull_AnchorworkGroup>& workGroupArray, std::vector<float>& meanDistanceArray, std::vector<size_t>& Sortindex);
	int findVerySmallDistanceMatch(std::vector<float> distanceArray);

	void smallEnoughDisatanceCount(std::vector<H_cull_AnchorworkGroup>& workGroupArray, float smallEnoughDistace, std::vector<int>& smallEnoughDisatanceCountArray, std::vector<size_t>& Sortindex);

};

