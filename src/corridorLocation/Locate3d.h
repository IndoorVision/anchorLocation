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
//������CLocate3d�࣬����������������run��sloverPnP_cv,run��Ƕ����ʹ����sloverPnP_cv
//run�����ã����þ���ƥ��ģ���λӰ���ϵĶ�ά���꣨matchPt2dArray���Ϳ��е���ά�����꣨matchPt3dArray�����󷽽���õ������λ��
//run�����룺��ά���꣨matchPt2dArray���Ϳ��е���ά�����꣨matchPt3dArray����
//run��������󷽽���õ���R��C
//sloverPnP_cv�����룺��ά���꣨matchPt2dArray���Ϳ��е���ά�����꣨matchPt3dArray�����ڲξ���
//sloverPnP_cv�������R��C���ڵ�����

class CLocate3d
{
public:
	CLocate3d();
	~CLocate3d();

	bool run(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray,
		cv::Mat&R, cv::Point3d&C, cv::Mat mK);

	bool sloverPnP_cv(const std::vector<cv::Point2d> &MarkArray_,
		const std::vector<cv::Point3d> &MapArray_, cv::Mat distCoeffs,
		const cv::Mat K, cv::Mat& R, cv::Mat& T, std::vector<int> &inliers);

	bool run_corridor(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray,
		cv::Mat&R, cv::Point3d&C, cv::Mat mK, cv::Mat distort);

	bool sloverPnP_cv_corridor(const std::vector<cv::Point2d> &MarkArray_,
		const std::vector<cv::Point3d> &MapArray_, cv::Mat distCoeffs,
		const cv::Mat K, cv::Mat& R, cv::Mat& T, std::vector<int> &inliers);

	
};

