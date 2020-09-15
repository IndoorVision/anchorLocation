// PictureSimilarityCompare.cpp : 定义控制台应用程序的入口点。
//

#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

#include "pointsLogRW.h"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;


void getSingleFileName(std::string& fileNamePath) {
	string::size_type iPos = fileNamePath.find_last_of('\\') + 1;
	fileNamePath = fileNamePath.substr(iPos, fileNamePath.length() - iPos);
}



int main(int argc, char* argv[])
{
	Mat imageL1 = imread(argv[1]);
	Mat imageR1 = imread(argv[2]);

	//找出特征点
	Ptr<SURF>f2d = xfeatures2d::SURF::create();
	f2d->setHessianThreshold(1000);
	vector<KeyPoint> keyPoint1, keyPoint2;
	f2d->detect(imageL1, keyPoint1);
	f2d->detect(imageR1, keyPoint2);

	//特征点匹配
	Mat descriptors_1, descriptors_2;
	f2d->compute(imageL1, keyPoint1, descriptors_1);
	f2d->compute(imageR1, keyPoint2, descriptors_2);

	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
	std::vector< std::vector<cv::DMatch> > knn_matches;
	matcher->knnMatch(descriptors_1, descriptors_2, knn_matches, 2);

	const float ratio_thresh = 0.7f;
	std::vector<cv::DMatch> good_matches;
	for (size_t k = 0; k < knn_matches.size(); k++)
	{
		if (knn_matches[k][0].distance < ratio_thresh * knn_matches[k][1].distance)
		{
			good_matches.push_back(knn_matches[k][0]);
		}
	}

	std::vector<cv::Point2d> srcMatchPt;
	std::vector<cv::Point2d> dstMatchPt;
	for (size_t k = 0; k < good_matches.size(); k++)
	{
		srcMatchPt.push_back(keyPoint1[good_matches[k].queryIdx].pt);
		dstMatchPt.push_back(keyPoint2[good_matches[k].trainIdx].pt);
	}


	//输出
	string fileName1 = argv[1];     string fileName2 = argv[2];
	getSingleFileName(fileName1);
	getSingleFileName(fileName2);
	string outPutName = fileName1.substr(0, fileName1.rfind(".")) +"_" +fileName2.substr(0, fileName2.rfind(".")) + ".pto";

	std::vector<POINT_PAIR> ptPairArray;
	ptPairArray.reserve(srcMatchPt.size());
	for (size_t i = 0; i < srcMatchPt.size(); i++)
	{
		POINT_PAIR point_pair;
		point_pair.x1 = srcMatchPt[i].x;
		point_pair.y1 = srcMatchPt[i].y;

		point_pair.x2 = dstMatchPt[i].x;
		point_pair.y2 = dstMatchPt[i].y;

		ptPairArray.push_back(point_pair);
	}
	outPutName = "data\\pto\\" + outPutName;
	pointsLogRW::writePTO(outPutName.c_str(), ptPairArray, fileName1+".jpg", fileName2 +".jpg");
	std::cout << "共匹配上" << srcMatchPt.size() << "个特征" << std::endl;
	return 0;


 
}






