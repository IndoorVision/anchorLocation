#pragma once
#include<iostream>
#include<fstream>
#include<opencv2\opencv.hpp>
#include<iomanip>
#include<vector>
#include<string>
#include <numeric>      // std::iota
#include"direct.h"

#include"SurfExtractor.h"
#include"pointsLogRW.h"

struct OutputStruct
{
	std::string anchorName;  //设定为和标准片名字一致
	std::vector<cv::Point3d> outputPoint3d;
	std::vector<cv::Point2d> outputPoint2d;
	cv::Mat outputDescriptor;
};

struct Anchor_extra
{
	std::string anchor_type;  //  normal代表正射影像， side 代表侧视影像 。 初始值默认设置为空
	cv::Mat homography_sideToStandard = cv::Mat::eye(3, 3, CV_64F);
	int normal_standardPicure_Width = 0;
	int normal_standardPicure_Height = 0;
};

struct ShootPictureInfo
{
	

	std::string imgName;
	std::string standardImagName;
	cv::Mat descriptors;
	cv::Mat homography = cv::Mat::zeros(3, 3, CV_64F);
	std::vector<int> descriptorsScore;
	

	std::vector<cv::Point3d> objectPoints;
	std::vector<cv::Point2d> imagePoints;
	std::vector<cv::Point3d> controalPoints;

	// sideShootAnchor extra
	Anchor_extra anchor_extra;
};


class CshootPictureProcessor
{

public:
	CshootPictureProcessor();
	~CshootPictureProcessor();


	bool readInfo(std::string filePath, std::vector<ShootPictureInfo>& myshootPictrueInfoVec);

	bool shootPictiresMatch_getScore(std::vector<ShootPictureInfo> &shootPictureArray);
	bool calculate_HomographyAndObejcPoints(std::string imgPath, std::vector<ShootPictureInfo> &shootPictureArray);
	bool selectGoodDescriptors(std::vector<ShootPictureInfo> &shootPictureArray, OutputStruct& outputstruct);
	bool createDatabase(string outfileName, OutputStruct &outputstruct);
	bool readStandPictureDescriptors(std::string imgPath, std::vector<ShootPictureInfo> &shootPictureArray, OutputStruct &ouputstruct);

	static void SplitString(const string& s, vector<string>& v, const string& c);
	bool is_element_in_vector(vector<cv::Point2d> v, cv::Point2d element);

	template <typename T>
	vector<size_t> sort_indexes_e(vector<T> &v)
	{
		vector<size_t> idx(v.size());
		iota(idx.begin(), idx.end(), 0);
		sort(idx.begin(), idx.end(),
			[&v](size_t i1, size_t i2) {return v[i1] < v[i2]; });
		return idx;
	}

	template<typename T>
	T SumVector(vector<T>& vec)
	{
		T res = 0;
		for (size_t i = 0; i<vec.size(); i++)
		{
			res += vec[i];
		}
		return res;
	};

private:
	bool readconfigfile(std::string configfilePath, std::vector<string> &filenameVec, std::vector<cv::Point3d> &ControalPoints, Anchor_extra& anchor_extra);
	bool readshootPictrue(std::string imgPath, std::string imgName, ShootPictureInfo& myshootPictrueInfo);
	cv::Point2d sideToNormal(cv::Point2d pt_onSideStandard, cv::Mat H);
	vector<string> split(string s, char delim);


};

