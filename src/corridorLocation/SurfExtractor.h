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

#include "features2d_surf.hpp"
//������CSurfExtractor����࣬�������������ͬ����Ա����run 
//�������ã��Զ�λӰ�������������ȡ
//���룺��λӰ������
//�������ȡ������������Ķ�ά����pt2dArray�������Ӧ��������Mat desc
class CSurfExtractor
{
public:
	CSurfExtractor();
	~CSurfExtractor();

	bool run(char* filename, cv::Mat&desc, std::vector<cv::Point2d>& pt2dArray, cv::Mat &mK);
	bool run_corridor(char* filename, cv::Mat&desc, std::vector<cv::Point2d>& pt2dArray, cv::Mat &mK);
	bool run_corridor(char* filename, cv::Mat& desc, std::vector<cv::Point2d>& pt2dArray, double& resizeRatio);
	bool runImg(cv::Mat img, cv::Mat &desc, std::vector<cv::Point2d>& pt2dArray);
};

