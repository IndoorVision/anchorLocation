// changeName.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <string>
#include <stdio.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <cv.h>
#include<io.h>
#include"direct.h"

#include <vector>
#include <fstream>
#include <windows.h>

using namespace std;
using namespace cv;

#include <iostream>
#include <vector>
#include <io.h>
#include <string>
#include <cstdio>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;


bool isJPG(std::string src_name)
{
	int pos = src_name.find_last_of('\\');
	std::string name = src_name.erase(0, pos + 1);

	std::string::size_type idx;
	idx = name.find("jpg");
	if (idx == string::npos)
		return false;
	else
		return true;
}

bool isStandardPic(std::string src_name)
{
	int pos = src_name.find_last_of('\\');
	std::string name = src_name.erase(0, pos + 1);

	std::string::size_type idx;
	idx = name.find("standard");
	if (idx == string::npos)
		return false;
	else
		return true;
}

bool isshoot(std::string src_name)
{
	int pos = src_name.find_last_of('\\');
	std::string name = src_name.erase(0, pos + 1);

	std::string::size_type idx;
	idx = name.find("shoot");
	if (idx == string::npos)
		return false;
	else
		return true;
}
	



void runMain(std::string path, std::string write_path) {
	//mkdir(write_path.c_str());
	vector<String> output_name;
	vector<String> src_name;
	bool haveTxt = false;
	std::string txtName;
	glob(path, src_name, false);
	if (src_name.size() == 0)
	{
		cerr << "That's no file in " << path << endl;
		exit(1);
	}

	int imgCnt = 0;
	for (int i = 0; i < src_name.size(); ++i) {

		std::string new_name ;
		if (!isJPG(src_name[i]))
		{
			haveTxt = true;
			txtName = src_name[i];
			continue;
		}

		Mat image = imread(src_name[i]);
		if (image.empty()) {
			cerr << "Read image " << src_name[i] << " failed!";
			exit(1);
		}


		if (!isStandardPic(src_name[i]) && !isshoot( src_name[i]))
		{ 
			imgCnt++;
			output_name.push_back(to_string(imgCnt) + ".jpg");
			new_name = write_path+ "\\"+ to_string(imgCnt) + ".jpg";
			
			rename(src_name[i].c_str(), new_name.c_str());
		}
		else {
			int pos = src_name[i].find_last_of('\\');
			std::string name = src_name[i].erase(0, pos+1 );
			output_name.insert(output_name.begin(), name);
		}
	}

	cout << "Totally rename " << (output_name.size()-2) << " pictures!" << endl;

	//output txt
	ofstream write;
	write.open(write_path+"\\createdbConfig.txt");
	int PicSize = output_name.size();   write << PicSize << std::endl;
	std::string type; 
	if (haveTxt == true)
	{
		type = "side";
		write << type << std::endl;

		ifstream in;  
		string line;
		in.open(txtName);   
		getline(in, line);
		in.close();
		write << line << std::endl;
	}
	else
	{
		type = "normal";
		write << type << std::endl;
		write << "1,0,0,0,1,0,0,0,1,1920,1279" << std::endl;
	}

	for (size_t i = 0; i < output_name.size() ; i++)
	{
		write << output_name[i] << std::endl;
	}
	
	//write controlPoints
	ifstream getControlPoints;
	getControlPoints.open( write_path+"\\..\\control.txt");
	assert(getControlPoints.is_open());   //若失败,则输出错误消息,并终止程序运行 
	std::string line;

	while (getline(getControlPoints, line))
	{
		write << line << std::endl;
	}

	write.close();
	getControlPoints.close();
	
}



int main(int argc, char *argv[]) {
	runMain(argv[1], argv[1]);
}

