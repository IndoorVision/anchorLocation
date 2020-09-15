// HwarpPerspective.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <fstream>
#include <string>

#include <opencv2\opencv.hpp>
#include "core/core.hpp"
#include "highgui/highgui.hpp"
#include "imgproc/imgproc.hpp"


using namespace cv;

void onMouse(int event, int x, int y, int flags, void *utsc);
Point2f srcTri[4], dstTri[4];
int clickTimes = 0;  //在图像上单击次数
Mat image;
Mat imageWarp;
double width ; double height;
std::string imgoutPath;


vector<string> split(string s, char delim) {
	vector<string> v;
	std::stringstream stringstream1(s);
	string tmp;
	while (std::getline(stringstream1, tmp, delim)) {
		v.push_back(tmp);
	}
	return v;
}

void getOutputPath(std::string& srcImagePath) {

	std::string::size_type ipos = srcImagePath.find_last_of("\\");
	srcImagePath.erase(ipos);
	srcImagePath += "standard.jpg";
}

void getimageWidthAndHeight(std::string controlPointsPath, double& width, double& height)
{
	std::vector<cv::Point3d> controlPoints;
	std::ifstream infile;
	infile.open(controlPointsPath);
	if (!infile.is_open())
	{
		std::cout << "read the control points failed!" << std::endl;
	}

	for (size_t i = 0; i < 4; i++)
	{
		std::string s;
		std::getline(infile, s);
		std::vector<std::string> v = split(s, ',');

		cv::Point3d pt;
		if (v.size() >= 3) {
			pt.x = stod(v[0]);   pt.y = stod(v[1]);   pt.z = stod(v[2]);
		}
		controlPoints.push_back(pt);
	}

	cv::Point3d widthVector = controlPoints[2] - controlPoints[1];
	cv::Point3d heightVector = controlPoints[1] - controlPoints[0];

	width = sqrt(widthVector.x * widthVector.x + widthVector.y * widthVector.y + widthVector.z * widthVector.z);
	height = sqrt(heightVector.x * heightVector.x + heightVector.y * heightVector.y + heightVector.z * heightVector.z);
}

void onMouse(int event, int x, int y, int flags, void *utsc)
{
	if (event == CV_EVENT_LBUTTONUP)   //响应鼠标左键抬起事件
	{
		circle(image, Point(x, y), 2.5, Scalar(0, 0, 255), 2.5);  //标记选中点

		cv::namedWindow("Source Image", CV_WINDOW_NORMAL);
		imshow("Source Image", image);
		srcTri[clickTimes].x = x;
		srcTri[clickTimes].y = y;
		clickTimes++;
	}
	if (clickTimes == 4)
	{
		dstTri[0].x = 0;
		dstTri[0].y = 0;
		dstTri[1].x = 0;
		dstTri[1].y = height;
		dstTri[2].x = width;
		dstTri[2].y = height;
		dstTri[3].x = width ;
		dstTri[3].y = 0;
		Mat transform = Mat::zeros(3, 3, CV_32FC1); //透视变换矩阵
		transform = getPerspectiveTransform(srcTri, dstTri);  //获取透视变换矩阵		
		warpPerspective(image, imageWarp, transform, Size(width, height ));  //透视变换

		cv::namedWindow("After WarpPerspecttive", CV_WINDOW_NORMAL);
		imshow("After WarpPerspecttive", imageWarp);
		imwrite(imgoutPath, imageWarp);
	}
}

void runMain(std::string imagePath, std::string controlPointsPath)
{
	imgoutPath = imagePath;
	getOutputPath(imgoutPath);
	image = imread(imagePath);
	getimageWidthAndHeight(controlPointsPath, width, height);
	width = width * 2000;
	height = height * 2000;
	cv::namedWindow("Source Image", CV_WINDOW_NORMAL);
	imshow("Source Image", image);
	setMouseCallback("Source Image", onMouse);
	waitKey();

}



int main(int argc, char *argv[])
{
	runMain(argv[1], argv[2]);
	waitKey();
	return 0;
}


