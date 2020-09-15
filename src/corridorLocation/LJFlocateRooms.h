#pragma once
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
#include <vector>
#include <fstream>
#include <string>
#include <random>
#include"DataStruction.h"
#include"PMatrixTools.h"
#include "pointsLogRW.h"

class CDatasetFlann;
struct DATASET_INFO
{
	char roomName[64];
	char datasetFileName[1024];
	cv::Point3d centerPoint;
	CDatasetFlann* pDatasetFlann;
	cv::Point3d shiftXYZ;

};

struct Camera_Index
{
	double fx,fy =0;
	double cx,cy = 0;
	double s = 0;
	double k1, k2, k3, p1, p2 =0;

};
class CLJFlocateRooms
{
public:
	CLJFlocateRooms();
	~CLJFlocateRooms();

	void init(std::string pathName);
	bool locate(char* roomName, char* imageFilename, double& X, double& Y,double& oritation);
	bool locate_erroranalysis(char* roomName, char* imageFilename, double& X, double& Y, double& orientation, double &mx, double &my, double &moritation);
	bool locate(cv::Point3d& initPoint, char* imageFilename,double& X, double& Y,double& oritation);
	Camera_Index m_cameraIndex;


protected:
	std::vector<DATASET_INFO> m_datasetArray;
	

	void init(std::vector<DATASET_INFO> &datasetArray);
	bool locateImpl(char* roomName,CDatasetFlann* pDatasetFlann, char* imageFilename, double& X, double& Y, double& oritation);
	bool locateImpl_erroranalysis(char* roomName, CDatasetFlann* pDatasetFlann, char* imageFilename, double& X, double& Y, double& orientation, double &mx, double &my, double &moritation);
	bool calErrorBygaussianNoise(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, double X, double Y, double orientation, double &mx, double &my, double &moritation);
	double myproject2d(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, double orientation, double Xs, double Ys, double threshold);

	bool calErrorBygaussianNoise3D(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, cv::Mat R, cv::Point3d C, double &mx, double &my, double &moritation);
	bool KmatrixAnddistortMatrixMake(double resizeRatio, cv::Mat& mk, cv::Mat& distort);

};

