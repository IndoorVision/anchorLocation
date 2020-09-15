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


#include <fstream>
#include <string>

//定义了CDatasetRW这个类，其中有函数CDatasetRW::run
//作用：读取库
//输入：库的名称
//输出：库中三维点pt3dArray、其对应描述子descriptors
class CDatasetRW
{
public:
	CDatasetRW();
	~CDatasetRW();

	bool run(char* filename, cv::Mat& descriptors, std::vector<cv::Point3d>& pt3dArray);
	bool run_corridor(char* filename, cv::Mat& descriptors, std::vector<cv::Point3d>& pt3dArray, std::vector<cv::Point2d> &pt2dArray, std::vector<std::string>& anchorNameArray);
	void loadOBJ(std::string filename);
	void loadOBJ_corridor(std::string filename);
	static bool writePly(char* filename, std::vector<cv::Point3d>& pt3dArray);
	static bool writePts(char* filename, std::vector<cv::Point2d>& pt2dArray, std::vector<cv::Point3d>& pt3dArray);
protected:

	struct OBJ {
		int id = 0;
		typedef int IMG_ID;
		typedef int KEY_ID;

		struct IMG_KEY {
			IMG_ID imgID;
			KEY_ID keyID;
			cv::KeyPoint pt;
		};
		std::vector<IMG_KEY> marks;
		cv::Mat des;//描述子
		cv::Point3d pt;
		cv::Point3_<uchar> colors;
		cv::Point2d imgpoints;
		std::string anchorName;
	};
	std::map<int, CDatasetRW::OBJ> m_objs;

	//void loadOBJ(std::string filename);



};

