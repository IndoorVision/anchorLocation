#include "stdafx.h"
#include <fstream>
#include <opencv2/opencv.hpp>



void loadOB(std::string combineDBfileName, std::string partDBfilename) {

	std::ifstream fin(partDBfilename, std::ios::binary);
	if (!fin || fin.bad()) {
		return;
	}

	std::ofstream fou(combineDBfileName, std::ios::app | std::ios::binary);

	while (fin.good()) {
		cv::Point3d tmpPoint3d;
		cv::Point2d tmpPoint2d;
		char tmpAnchorName[256];
		cv::Mat tmpDesc(1, 64, CV_32FC1);

		fin.read(tmpAnchorName, 100);
		fin.read((char*)&tmpPoint2d.x, sizeof(double));
		fin.read((char*)&tmpPoint2d.y, sizeof(double));
		fin.read((char*)&tmpPoint3d.x, sizeof(double));
		fin.read((char*)&tmpPoint3d.y, sizeof(double));
		fin.read((char*)&tmpPoint3d.z, sizeof(double));
		fin.read((char*)tmpDesc.data, tmpDesc.step[0]);

		fou.write(reinterpret_cast<char*>(&tmpAnchorName), 100);
		fou.write(reinterpret_cast<char*>(&tmpPoint2d), sizeof(double) * 2);
		fou.write(reinterpret_cast<char*>(&tmpPoint3d), sizeof(double) * 3);
		fou.write(reinterpret_cast<char*>(tmpDesc.ptr<uchar>(0)), sizeof(float) * tmpDesc.cols);
	}

	fin.close();
	fou.close();
}


int main(int argc, char* argv[])
{
	std::string combineDBfileName = argv[1];
	std::string partDBfileName = argv[2];
	loadOB(combineDBfileName, partDBfileName);
	return 0;
}

