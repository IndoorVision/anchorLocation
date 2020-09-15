
#include <string>
#include<opencv2/opencv.hpp>

#include<iostream>
#include<time.h>
#include "features2d_surf.hpp"
#include "SurfExtractor.h"

#include "LJFlocateRooms.h"

using namespace cv;
using namespace std;

CLJFlocateRooms* g_pLJFlocateRooms = nullptr;

vector<string> split(string s, char delim) {
	vector<string> v;
	stringstream stringstream1(s);
	string tmp;
	while (getline(stringstream1, tmp, delim)) {
		v.push_back(tmp);
	}
	return v;
}

void initLocate(std::string pathName)
{
	
	if (g_pLJFlocateRooms == nullptr) {
		g_pLJFlocateRooms = new CLJFlocateRooms;
		g_pLJFlocateRooms->init(pathName);
	}
}

bool readCameraConfig(std::string pathName, double cameraIndex[10]) {
	ifstream ifs;
	ifs.open(pathName, ios::in);

	if (!ifs.is_open()){
		std::cout << "打开文件失败！" << std::endl;
		return false;
	}

	std::string s;
	getline(ifs, s);

	std::vector<string> v = split(s, ',');
	if (v.size() ==10)
	{
		cameraIndex[0] = stod(v[0]);
		cameraIndex[1] = stod(v[1]);
		cameraIndex[2] = stod(v[2]);
		cameraIndex[3] = stod(v[3]);
		cameraIndex[4] = stod(v[4]);
		cameraIndex[5] = stod(v[5]);
		cameraIndex[6] = stod(v[6]);
		cameraIndex[7] = stod(v[7]);
		cameraIndex[8] = stod(v[8]);
		cameraIndex[9] = stod(v[9]);
	}
}



std::string LocateByRoom(std::string roomName, std::string imgFileName, double orientation, double cameraIndex[10])
{
	//get roomeName and imgFileNme
	clock_t t1 = clock();
	char image_filename[1024];
	strcpy(image_filename, imgFileName.c_str());
	char roomName2[1024];
	strcpy(roomName2, roomName.c_str());
	double X1 = -1.0;    double Y1 = -1.0;     double ori = -9999.0;
	double mx = 0.2; double my = 0.2; double moritation = 0;
	ori = orientation;
	
	//get CameraIndex
	g_pLJFlocateRooms->m_cameraIndex.fx = cameraIndex[0];
	g_pLJFlocateRooms->m_cameraIndex.fy = cameraIndex[1];
	g_pLJFlocateRooms->m_cameraIndex.cx = cameraIndex[2];
	g_pLJFlocateRooms->m_cameraIndex.cy = cameraIndex[3];
	g_pLJFlocateRooms->m_cameraIndex.s = cameraIndex[4];
	g_pLJFlocateRooms->m_cameraIndex.k1 = cameraIndex[5];
	g_pLJFlocateRooms->m_cameraIndex.k2 = cameraIndex[6];
	g_pLJFlocateRooms->m_cameraIndex.k3 = cameraIndex[7];
	g_pLJFlocateRooms->m_cameraIndex.p1 = cameraIndex[8];
	g_pLJFlocateRooms->m_cameraIndex.p2 = cameraIndex[9];

	//Locate
	bool bret = g_pLJFlocateRooms->locate_erroranalysis(roomName2, image_filename, X1, Y1, ori, mx, my, moritation);
	
	std::cout << "mx =" << mx << "  " << "my =" << my << std::endl;
	
	clock_t t2 = clock();
	double t12 = double(t2 - t1);
	t12 = t12 / CLOCKS_PER_SEC ;
	std::cout << "总耗时"<<t12 <<"s" << std::endl;

	char  buffer[1024];
	// sprintf(buffer,"The LocationResult is X=\"+%lf +\"Y=\"+ %lf",X1, Y1);
	sprintf(buffer, "   %.4f(%.4f  %.4f  %0.2f)", t12, X1, Y1, ori);
	return buffer;
}


void runMain(double ori, char* datasetPathName, char* roomName, char* imageFileName)
{
	ori = - ori;
	if (ori < 0)		ori += 360;
	if (ori > 360)		ori -= 360;

	initLocate(datasetPathName);
	
	double cameraIndex[10];
	readCameraConfig("cvconfig.txt", cameraIndex);
	std::string result = LocateByRoom(roomName, imageFileName, ori, cameraIndex);
	double X = 0.0;
	double Y = 0.0;
	double T = 0.0;
	double orientation = 0.0;
	sscanf(result.c_str(), "%lf(%lf %lf %lf)", &T, &X, &Y, &orientation);


	//X -= 550516.58;
	//Y -= 3372315.30;
	printf("X =%lf\tY = %lf ori = %lf\n", X, Y, orientation);
}


int main(int argc, char** argv)
{
//	char datasetPathName[] = ".\\";	
//	char roomName[] = "214";
//	char imageFileName[1024] = ".\\20190909pai\\IMG_3434.JPG";

	//strcpy(imageFileName, "locate2d.JPG");

	

	char* datasetPathName = argv[1];
	char *imageFileName = argv[2];
	char *roomName = argv[3];

	double ori = 310;
	runMain(ori, datasetPathName, roomName, imageFileName);
    return 0;
}


