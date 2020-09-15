//#include "stdafx.h"
#include "Locate3d.h"
#include "Runtime.h"
#include <iostream>
#include <iomanip>

CLocate3d::CLocate3d()
{
}

CLocate3d::~CLocate3d()
{
}

bool CLocate3d::sloverPnP_cv(const std::vector<cv::Point2d> &MarkArray_,
	const std::vector<cv::Point3d> &MapArray_, cv::Mat distCoeffs,
	const cv::Mat K, cv::Mat& R, cv::Mat& T, std::vector<int> &inliers) 
{
	if (MapArray_.size() < 10 || MapArray_.size() != MarkArray_.size())
	{
		return false;
	}
	cv::Mat r;
	bool b = 1;
	int minInliersCount = std::max(MapArray_.size() / 3, (size_t)10);
	//std::vector<int> inliers;

	float reprojectionError = std::min(K.at<double>(0, 2), K.at<double>(1, 2)) / 80;
	reprojectionError = std::max(reprojectionError, 3.0f);;
	int iterationsCount = MarkArray_.size() * 5;

		//cv::solvePnPRansac(MapArray_, MarkArray_, K, distCoeffs, r, T, false,
		//iterationsCount, reprojectionError, 0.8, inliers);
	cv::solvePnPRansac(MapArray_, MarkArray_, K, distCoeffs, r, T, false,
		iterationsCount, reprojectionError, 0.99, inliers);

	//cv::solvePnP(MapArray_3, MarkArray_3, K, distCoeffs, r, T, false, CV_EPNP);
	if (r.empty() || inliers.empty()) {
		return false;
	}
	float rr = inliers.size()*1.0 / MapArray_.size();
	if (rr<0.2) {
		return false;
	}
	std::cout << "inliers count = " << inliers.size() << "\n";
	cv::Rodrigues(r, R);
	return b;
}

bool CLocate3d::run(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray,
	cv::Mat&R, cv::Point3d&C, cv::Mat mK)
{
	double CenterXYZ[3] = { 0,0,0 };
	for (size_t k = 0; k < matchPt3dArray.size(); k++)
	{
		CenterXYZ[0] += matchPt3dArray[k].x;
		CenterXYZ[1] += matchPt3dArray[k].y;
		CenterXYZ[2] += matchPt3dArray[k].z;
	}
	CenterXYZ[0] = CenterXYZ[0] / matchPt3dArray.size();
	CenterXYZ[1] = CenterXYZ[1] / matchPt3dArray.size();
	CenterXYZ[2] = CenterXYZ[2] / matchPt3dArray.size();

	for (size_t k = 0; k < matchPt3dArray.size(); k++) {
		matchPt3dArray[k].x -= CenterXYZ[0];
		matchPt3dArray[k].y -= CenterXYZ[1];
		matchPt3dArray[k].z -= CenterXYZ[2];
	}

	cv::Mat mRcw;
	cv::Mat mtcw;
	std::vector<int> inliers;
	{
		int pointNum = matchPt2dArray.size();
		int pointNum3 = matchPt3dArray.size();
		bool b = sloverPnP_cv(matchPt2dArray, matchPt3dArray, cv::Mat(), mK, mRcw, mtcw, inliers);
		//bool b = false;
		if (!b) {
			std::cout << "location failed " << "\n";
			return false;
		}
	}

	R = mRcw;
	cv::Mat C_ = -mRcw.inv()*mtcw;
	memcpy(&C, C_.data, sizeof(C));

	C.x += CenterXYZ[0];
	C.y += CenterXYZ[1];
	C.z += CenterXYZ[2];

	std::cout << std::fixed << std::setprecision(10);
	std::cout << "R=" << R << std::endl;
	std::cout << "C=" << C << std::endl;
	return true;
}




bool CLocate3d::sloverPnP_cv_corridor(const std::vector<cv::Point2d> &MarkArray_,
	const std::vector<cv::Point3d> &MapArray_, cv::Mat distCoeffs,
	const cv::Mat K, cv::Mat& R, cv::Mat& T, std::vector<int> &inliers)
{
	if (MapArray_.size() < 4 || MapArray_.size() != MarkArray_.size())
	{
		return false;
	}
	cv::Mat r;
	bool b = 1;
	int minInliersCount = std::max(MapArray_.size() / 3, (size_t)10);
	//std::vector<int> inliers;

	float reprojectionError = std::min(K.at<double>(0, 2), K.at<double>(1, 2)) / 80;
	reprojectionError = std::max(reprojectionError, 3.0f);;
	int iterationsCount = MarkArray_.size() * 5;

	//cv::solvePnPRansac(MapArray_, MarkArray_, K, distCoeffs, r, T, false,
	//iterationsCount, reprojectionError, 0.8, inliers);
	cv::solvePnPRansac(MapArray_, MarkArray_, K, distCoeffs, r, T, false,
		100, reprojectionError, 0.8, inliers);

	//cv::solvePnP(MapArray_, MarkArray_, K, distCoeffs, r, T, false, CV_EPNP);

	if (r.empty() || inliers.empty()) {
		return false;
	}
	float rr = inliers.size()*1.0 / MapArray_.size();
	if (rr<0.2) {
		return false;
	}
	std::cout << "inliers count = " << inliers.size() << "\n";
	cv::Rodrigues(r, R);
	return b;
}

bool CLocate3d::run_corridor(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray,
	cv::Mat&R, cv::Point3d&C, cv::Mat mK, cv::Mat distort)
{
	double CenterXYZ[3] = { 0,0,0 };
	for (size_t k = 0; k < matchPt3dArray.size(); k++)
	{
		CenterXYZ[0] += matchPt3dArray[k].x;
		CenterXYZ[1] += matchPt3dArray[k].y;
		CenterXYZ[2] += matchPt3dArray[k].z;
	}
	CenterXYZ[0] = CenterXYZ[0] / matchPt3dArray.size();
	CenterXYZ[1] = CenterXYZ[1] / matchPt3dArray.size();
	CenterXYZ[2] = CenterXYZ[2] / matchPt3dArray.size();

	for (size_t k = 0; k < matchPt3dArray.size(); k++) {
		matchPt3dArray[k].x -= CenterXYZ[0];
		matchPt3dArray[k].y -= CenterXYZ[1];
		matchPt3dArray[k].z -= CenterXYZ[2];
	}

	cv::Mat mRcw;
	cv::Mat mtcw;
	std::vector<int> inliers;
	{
		int pointNum = matchPt2dArray.size();
		int pointNum3 = matchPt3dArray.size();
		bool b = sloverPnP_cv_corridor(matchPt2dArray, matchPt3dArray, distort, mK, mRcw, mtcw, inliers);
		//bool b = false;
		if (!b) {
			std::cout << "location failed " << "\n";
			return false;
		}
	}

	R = mRcw;
	cv::Mat C_ = -mRcw.inv()*mtcw;
	memcpy(&C, C_.data, sizeof(C));

	C.x += CenterXYZ[0];
	C.y += CenterXYZ[1];
	C.z += CenterXYZ[2];

	std::cout << std::fixed << std::setprecision(10);
	std::cout << "R=" << R << std::endl;
	std::cout << "C=" << C << std::endl;
	return true;
}