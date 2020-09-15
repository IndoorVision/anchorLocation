// sideStandardPictureCreate.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include<opencv2\opencv.hpp>
#include <opencv/cv.h>  
#include <opencv/highgui.h>  
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>


IplImage* src = 0;
IplImage* dst = 0;
int src_width = 0;
int src_height = 0;
std::vector<cv::Point2d> dstPoints;

void on_mouse(int event, int x, int y, int flags, void* ustc)
{
	static CvPoint pre_pt = { -1,-1 };
	static CvPoint cur_pt = { -1,-1 };
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);
	char temp[16];

	if (event == CV_EVENT_LBUTTONDOWN)
	{
		cvCopy(dst, src);
		sprintf(temp, "(%d,%d)", x, y);
		pre_pt = cvPoint(x, y);
		cvPutText(src, temp, pre_pt, &font, cvScalar(0, 0, 0, 255));
		cvCircle(src, pre_pt, 3, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
		cvShowImage("src", src);
		cvCopy(src, dst);
		dstPoints.push_back(pre_pt);
	}
	else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))
	{
		cvCopy(dst, src);
		sprintf(temp, "(%d,%d)", x, y);
		cur_pt = cvPoint(x, y);
		cvPutText(src, temp, cur_pt, &font, cvScalar(0, 0, 0, 255));
		cvShowImage("src", src);
	}
	else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))
	{
		cvCopy(dst, src);
		sprintf(temp, "(%d,%d)", x, y);
		cur_pt = cvPoint(x, y);
		cvPutText(src, temp, cur_pt, &font, cvScalar(0, 0, 0, 255));
		cvRectangle(src, pre_pt, cur_pt, cvScalar(0, 255, 0, 0), 1, 8, 0);
		cvShowImage("src", src);
	}
	else if (event == CV_EVENT_LBUTTONUP)
	{
		sprintf(temp, "(%d,%d)", x, y);
		cur_pt = cvPoint(x, y);
		cvPutText(src, temp, cur_pt, &font, cvScalar(0, 0, 0, 255));
		cvCircle(src, cur_pt, 3, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
		cvRectangle(src, pre_pt, cur_pt, cvScalar(0, 255, 0, 0), 1, 8, 0);
		cvShowImage("src", src);
		cvCopy(src, dst);
	}
}

std::string getAnchorName(std::string pathName)
{
	std::string::size_type iPos = pathName.find_last_of("\\");
	std::string Out_anchorName = pathName.substr(iPos+1, pathName.length() - iPos);
	return Out_anchorName;
}

int getAnchorNum(std::string pathName)
{
	//获取anchor信息
	std::string::size_type iPos = pathName.find_last_of("\\");
	std::string anchorName = pathName.substr(iPos + 1, pathName.length() - iPos);

	//获取编号
	std::string::size_type iPosBegin = anchorName.find('or');
	std::string::size_type iPosend = anchorName.find('_');
	std::string outNum = anchorName.substr(iPosBegin+1 , iPosend-iPosBegin-1);

	return atoi(outNum.c_str());

}

std::string getAnchorType(std::string pathName)
{
	//获取anchor信息
	std::string::size_type iPos = pathName.find_last_of("\\");
	std::string anchorName = pathName.substr(iPos + 1, pathName.length() - iPos);

	//获取类型
	iPos = anchorName.find_last_of('_');
	std::string out_type = anchorName.substr(iPos+1, anchorName.length() - iPos);
	return out_type;
}


int main(int arvc, char* argv[])
{
	std::string PackagePath = argv[1];
	// resize the sideImage
	{
		cv::Mat sideImage = cv::imread(PackagePath +"\\shoot.jpg");
		double max_wh = std::max(sideImage.rows, sideImage.cols);
		if (max_wh>1920)
		{
			double t = std::max(max_wh / 1920, 1.0);;
			cv::resize(sideImage, sideImage, cv::Size(sideImage.cols / t, sideImage.rows / t));
		}
		cv::imwrite(PackagePath+"\\shoot.jpg", sideImage);
	}
	


	// step1
	src = cvLoadImage((PackagePath +"\\shoot.jpg").c_str(), 1);
	dst = cvCloneImage(src);
	cvNamedWindow("src", CV_WINDOW_NORMAL);
	cvSetMouseCallback("src", on_mouse, 0);

	src_height = src->height;  src_width = src->width;

	cvShowImage("src", src);
	cvWaitKey(0);
	cvDestroyAllWindows();
	cvReleaseImage(&src);
	cvReleaseImage(&dst);

	std::cout << "获取四角点完毕" << std::endl;

	// step2 
	cv::Mat standardPicimg = cv::imread(PackagePath+"\\standard.jpg");
	double max_wh = std::max(standardPicimg.rows, standardPicimg.cols);
	if (max_wh>1920)
	{
		double t = std::max(max_wh / 1920, 1.0);;
		cv::resize(standardPicimg, standardPicimg, cv::Size(standardPicimg.cols / t, standardPicimg.rows / t));
	}
	cv::imwrite(PackagePath+"\\standard.jpg", standardPicimg);
	std::cout << "读取标准图像完毕" << std::endl;


	// step3
	std::vector<cv::Point2d> srcPoints;
	double width = standardPicimg.cols;   double height = standardPicimg.rows;
	cv::Point2d srcPt1 = { 0,0 };             
	cv::Point2d srcPt2 = { 0, height };
	cv::Point2d srcPt3 = { width, height };
	cv::Point2d srcPt4 = { width, 0 };
	srcPoints.push_back(srcPt1); srcPoints.push_back(srcPt2); srcPoints.push_back(srcPt3); srcPoints.push_back(srcPt4);
	cv::Mat H = findHomography(srcPoints, dstPoints, CV_RANSAC);

	// step4 
	cv::Mat outPutImag = cv::Mat(src_height, src_width, CV_8UC3, cv::Scalar(255, 255, 255));
	for (int i = 0; i < standardPicimg.rows; i++)
	{
		for (int j = 0; j < standardPicimg.cols; j++)
		{
			cv::Mat src = (cv::Mat_<double>(3, 1) << j, i, 1);
			cv::Mat calDstPoints = H * src;
			cv::Point2d H_convertion = { calDstPoints.at<double>(0, 0) / calDstPoints.at<double>(2, 0) ,calDstPoints.at<double>(1, 0) / calDstPoints.at<double>(2, 0) };
			outPutImag.at<cv::Vec3b>(H_convertion.y, H_convertion.x)[0]= standardPicimg.at<cv::Vec3b>(i, j)[0];
			outPutImag.at<cv::Vec3b>(H_convertion.y, H_convertion.x)[1] = standardPicimg.at<cv::Vec3b>(i, j)[1];
			outPutImag.at<cv::Vec3b>(H_convertion.y, H_convertion.x)[2] = standardPicimg.at<cv::Vec3b>(i, j)[2];
		}
	}

	// out img
	int anchorNum = getAnchorNum(PackagePath);   std::string anchorType = getAnchorType(PackagePath);
	if (anchorNum <= 9)
	{
		std::string IMGpath =  "standard_" + anchorType + ".jpg";
		bool b = cv::imwrite(PackagePath+"\\" +IMGpath, outPutImag);
	}
	else
	{
		std::string IMGpath = "standard_" + anchorType + ".jpg";
		bool b = cv::imwrite(PackagePath + "\\" + IMGpath, outPutImag);
	}
		
	
	// out txt
	H = H.inv();
	std::string out_string = std::to_string(H.at<double>(0, 0)) + ',' + std::to_string(H.at<double>(0, 1)) + ',' + std::to_string(H.at<double>(0, 2)) + ',' +
		std::to_string(H.at<double>(1, 0)) + ',' + std::to_string(H.at<double>(1, 1)) + ',' + std::to_string(H.at<double>(1, 2)) + ',' +
		std::to_string(H.at<double>(2, 0)) + ',' + std::to_string(H.at<double>(2, 1)) + ',' + std::to_string(H.at<double>(2, 2));

	out_string = out_string + ','+std::to_string(standardPicimg.cols) +','+std::to_string(standardPicimg.rows);
	
	std::string anchorName = getAnchorName(PackagePath);
	std::ofstream myout(PackagePath+"\\"+anchorName+".txt");
	myout << out_string;
	myout.close();

	//删除正射标准片
	if (remove((PackagePath + "\\standard.jpg").c_str())) {
		std::cout << "Delete  failed" << std::endl;
	}

	
	return 0;
}



