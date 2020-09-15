//#include "stdafx.h"
#include "DatasetFlann.h"
#include "SurfExtractor.h"
//#include "LJFResect.h"
#include "vote_locate2.h"
#include "DatasetRW.h"
#include "DatasetFlann.h"
#include "Locate3d.h"
#include "LJFlocateRooms.h"
#include "FilterOutlier.h"
#include "Locate2d.h"
#include "MonteCarlo2d.h"	

//#define OUTPUT_MATCH_PTS
//#define OUTPUT_PLY
//#define OUTPUT_PTO

CLJFlocateRooms::CLJFlocateRooms()
{
}


CLJFlocateRooms::~CLJFlocateRooms()
{
}

void CLJFlocateRooms::init(std::string pathName)
{
    std::string objname214 = pathName + "SLT-214-obj.dat";
	//objname214 = pathName + "extractorType-OBJ-214-18-12-05.dat";
    std::string objname212 = pathName + "SLT-212-obj.dat";
	std::string objnamecorridor = pathName + "SLT-corridor.dat";
	std::vector<DATASET_INFO> datasetArray;
	DATASET_INFO info;

	strcpy(info.datasetFileName, objname212.data());
	strcpy(info.roomName, "4209276001_2_212/210");
	info.centerPoint.x = 550549.70;
	info.centerPoint.y = 3372318.60;
	info.centerPoint.z = 36.54;
	datasetArray.push_back(info); 

	strcpy(info.datasetFileName, objname214.data());
	strcpy(info.roomName, "4209276001_2_216/214");
	info.centerPoint.x = 550523.57;
	info.centerPoint.y = 3372318.60; 
	info.centerPoint.z = 36.54;
	datasetArray.push_back(info);


	strcpy(info.datasetFileName, objnamecorridor.data());
	strcpy(info.roomName, "4209276001_2_11");
	info.centerPoint.x = 1;
	info.centerPoint.y = 1;
	info.centerPoint.z = 1;
	datasetArray.push_back(info);

	/*
	std::string lmars4floor = pathName + "4floor.dat";
	strcpy(info.datasetFileName, lmars4floor.data());
	strcpy(info.roomName, "lmars4floor");
	info.centerPoint.x = 3.96;
	info.centerPoint.y = 7.65;
	info.centerPoint.z = 1.55;
	datasetArray.push_back(info);
	*/

	//datasetArray.push_back(info);
	init(datasetArray);
}

void CLJFlocateRooms::init(std::vector<DATASET_INFO> &datasetArray)
{
	m_datasetArray = datasetArray; 

	for (int i = 0; i < m_datasetArray.size(); i++)
	{
		CDatasetFlann* pDatasetFlann = new CDatasetFlann;
		m_datasetArray[i].pDatasetFlann = pDatasetFlann;
		pDatasetFlann->load(m_datasetArray[i].datasetFileName);
	}
}

bool CLJFlocateRooms::locate_erroranalysis(char* roomName, char* imageFilename, double& X, double& Y, double& orientation, double &mx, double &my, double &moritation)
{
	double ratio_threshold = 0.6;
	CDatasetFlann* pDatasetFlann = nullptr;

	for (int i = 0; i < m_datasetArray.size(); i++)
	{
		if (strcmp(roomName, m_datasetArray[i].roomName) == 0)
		{
			pDatasetFlann = m_datasetArray[i].pDatasetFlann;
			break;
		}
	}

	locateImpl_erroranalysis(roomName, pDatasetFlann, imageFilename, X, Y, orientation, mx, my, moritation);

	return true;
}

bool CLJFlocateRooms::locateImpl_erroranalysis(char* roomName, CDatasetFlann* pDatasetFlann, char* imageFilename, double& X, double& Y, double& orientation, double &mx, double &my, double &moritation) {
	
	int placeType = pDatasetFlann->getPlaceType();
	if (placeType == 1)
	{
		double ratio_threshold = 0.6;

		std::vector<std::string> matchanchorNameArray;
		std::vector<cv::Point2d> matchPt2dArray;
		std::vector<cv::Point3d> matchPt3dArray;
		std::vector<cv::Point2d> matchPt2dArray_OnstandardPitcure;

		//SURF
		double resizeRatio;
		cv::Mat mK;
		cv::Mat queryDesc;
		cv::Mat distortMatrix;
		std::vector<cv::Point2d> pt2dArray;             //对定位影像进行特征点提取： 输入：定位影像 输出：坐标及描述子
		CSurfExtractor surf;
	
		//m_cameraIndex.cx = 0;
		if (m_cameraIndex.cx != 0 && m_cameraIndex.cy != 0)
		{
			surf.run_corridor(imageFilename, queryDesc, pt2dArray, resizeRatio);
			KmatrixAnddistortMatrixMake(resizeRatio, mK, distortMatrix);
		}
		else
			surf.run_corridor(imageFilename, queryDesc, pt2dArray, mK);
		
		if (pt2dArray.size() < 10)
		{
			X = -9999; Y = -9999;
			return false;
		}

		//Query
		clock_t t5 = clock();
		pDatasetFlann->query_corridor(queryDesc, pt2dArray, ratio_threshold,matchPt2dArray, matchPt3dArray, matchPt2dArray_OnstandardPitcure, matchanchorNameArray);
		clock_t t6 = clock();  double t56 = double(t6 - t5) / CLOCKS_PER_SEC;    std::cout << "Flann 耗时" << t56 << "s" << std::endl;

#ifdef OUTPUT_PLY
		char plyFileName[1024];
		strcpy(plyFileName, imageFilename);
		strcat(plyFileName, ".ply");                                 //打印匹配点
		CDatasetRW::writePly(plyFileName, matchPt3dArray);
#endif // OUTPUT_PLY

#ifdef OUTPUT_PTO
		std::vector<POINT_PAIR> ptPairArray;
		for (size_t k = 0; k < matchPt2dArray.size(); k++) {
			POINT_PAIR myPoint_parir;
			myPoint_parir.x1 = matchPt2dArray[k].x;     myPoint_parir.y1 = matchPt2dArray[k].y;
			myPoint_parir.x2 = matchPt2dArray_OnstandardPitcure[k].x;  myPoint_parir.y2 = matchPt2dArray_OnstandardPitcure[k].y;
			ptPairArray.push_back(myPoint_parir);
		}
		string string_ptoFileName = roomName;   string_ptoFileName += ".pto";
		pointsLogRW::writePTO(string_ptoFileName.c_str(), ptPairArray, imageFilename, "indoor_002_standard.jpg");
#endif // OUTPUT_PTO

		CLocate3d ljfresect;
		cv::Mat R;
		cv::Point3d C;
		double Z;
		clock_t t7 = clock();
		bool b= ljfresect.run_corridor(matchPt2dArray, matchPt3dArray, R, C, mK, distortMatrix);
		clock_t t8 = clock(); double t78 = double(t8 - t7) / CLOCKS_PER_SEC;    std::cout << "Locate3d 耗时" << t78 << "s" << std::endl;
		if (b) {
			X = C.x;  Y = C.y;
		}
		else {
			X = -9999; Y = -9999;
		}

	}

	else {
		double ratio_threshold = 0.6;

		std::vector<cv::Point2d> matchPt2dArray;
		std::vector<cv::Point3d> matchPt3dArray;

		cv::Mat mK;
		cv::Mat queryDesc;
		std::vector<cv::Point2d> pt2dArray;             //对定位影像进行特征点提取： 输入：定位影像 输出：坐标及描述子
		CSurfExtractor surf;
		surf.run(imageFilename, queryDesc, pt2dArray, mK);

		pDatasetFlann->query(queryDesc, pt2dArray, ratio_threshold,
			matchPt2dArray, matchPt3dArray);

		std::vector<cv::Point2d> orign_matchPt2dArray;
		std::vector<cv::Point3d> orign_matchPt3dArray;
		orign_matchPt2dArray = matchPt2dArray;
		orign_matchPt3dArray = matchPt3dArray;

		cv::Point3d centerPoint;
		for (int i = 0; i < m_datasetArray.size(); i++)
		{
			if (strcmp(roomName, m_datasetArray[i].roomName) == 0)
			{
				centerPoint = m_datasetArray[i].centerPoint;
			}
		}

		for (int i = 0; i < matchPt3dArray.size(); i++)
		{
			matchPt3dArray[i].x -= centerPoint.x;
			matchPt3dArray[i].y -= centerPoint.y;
		}

		cv::Point3d newCenterPoint(0.0, 0.0, 0.0);

		CMonteCarlo2d monteCarlo2d;
		X = 0; Y = 0;
		monteCarlo2d.run(mK, matchPt2dArray, matchPt3dArray, newCenterPoint, orientation,
			X, Y, 500, 1.0);
		newCenterPoint.x = X;
		newCenterPoint.y = Y;

		CFilterOutlier  filterOutlier;
		CLocate2d  locate2d;

		// filter orign Points 
		filterOutlier.run(matchPt2dArray, matchPt3dArray, orientation, newCenterPoint);
		filterOutlier.removeByX(matchPt2dArray, matchPt3dArray, orientation, newCenterPoint);

		CLocate3d ljfresect_first;
		cv::Mat R;
		cv::Point3d C;
		bool b = ljfresect_first.run(matchPt2dArray, matchPt3dArray, R, C, mK);
		if (b) {
			X = C.x;  Y = C.y;
		}
		else {
			X = -9999; Y = -9999;
		}

		if (fabs(X) > 7 || fabs(Y) > 4) {
			X = -9999; Y = -9999;
		}

		if (X != -9999) {
			X += centerPoint.x;
			Y += centerPoint.y;

			// cal error
			C.x = X; C.y = Y;
			calErrorBygaussianNoise3D(mK, orign_matchPt2dArray, orign_matchPt3dArray, R, C, mx, my, moritation);
		}
		

	}

	return true;
}

bool CLJFlocateRooms::calErrorBygaussianNoise3D(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, cv::Mat R, cv::Point3d C, double &mx, double &my, double &moritation) {
	
	double pointNum = matchPt2dArray.size();
	double MaxNoiseX = mx;
	double MaxNoiseY = my;

	double small_residual = 0.1;

	// cal sd, mean
	cv::Mat tmp_m, tmp_sd;
	cv::meanStdDev(matchPt2dArray, tmp_m, tmp_sd);
	double std_x = tmp_sd.at<double>(0, 0);
	double std_y = tmp_sd.at<double>(1, 0);

	//逐渐放大标准差，扩大误差，进行试探，标准差按步长0.5进行增长，总共试探10次
	for (int i = 1; i < 8; i++) {

		// Define random generator with Gaussian distribution
		double mean = 0.0;//均值
		double stddev = 0 + i*0.1;//标准差
		std::default_random_engine generator;
		std::normal_distribution<double> dist(mean, stddev);

		// Add Gaussian noise
		double noiseX = dist(generator);
		double noiseY = dist(generator);

		double X_noise = C.x + noiseX;
		double Y_noise = C.y + noiseY;

		//获取P矩阵
		ImageEo imageEo;  imageEo.R = R;  imageEo.Xs = X_noise; imageEo.Ys = Y_noise; imageEo.Zs = C.z;
		ImageIo imageIo;  imageIo.f = mK.at<double>(0, 0);  imageIo.x0 = mK.at<double>(0, 2); imageIo.y0 = mK.at<double>(1, 2);
		cv::Mat P;
		PMatrixTools::createPMatrix(imageEo, imageIo, P);


		//重投影
		double residual = 0;
		double passCNT = 0;
		if (matchPt2dArray.size() >= 3) {
			// reprojection
			std::vector<cv::Point2d> imagePointArray;
			PMatrixTools::projectFrom3D_T_2D_P(P, matchPt3dArray, imagePointArray);

			//cal residual
			std::vector<double> residualArray;
			for (int j = 0; j < pointNum; j++) {
				double tmpResidual = (fabs(imagePointArray[j].x - matchPt2dArray[j].x) / std_x + fabs(imagePointArray[j].y - matchPt2dArray[j].y) / std_y) / 2;
				residualArray.push_back(tmpResidual);

				if (tmpResidual < small_residual) {
					residual += (tmpResidual / std_x);  passCNT++;
				}
			}

			if (passCNT > pointNum / 3 * 2) {
				residual = residual / passCNT;
			}

		}

		if (residual < 10 && residual != 0)
		{
			if (MaxNoiseX < abs(noiseX))
				MaxNoiseX = abs(noiseX);

			if (MaxNoiseY < abs(noiseY))
				MaxNoiseY = abs(noiseY);
		}
	}

	mx = MaxNoiseX;
	my = MaxNoiseY;

	return true;

}


/*----------------------------------------------------------------- not use -----------------------------------------------------------------------*/
bool CLJFlocateRooms::locateImpl(char* roomName, CDatasetFlann* pDatasetFlann, char* imageFilename, double& X, double& Y, double& orientation)
{
	double ratio_threshold = 0.6;

	std::vector<cv::Point2d> matchPt2dArray;
	std::vector<cv::Point3d> matchPt3dArray;

	cv::Mat mK;
	cv::Mat queryDesc;
	std::vector<cv::Point2d> pt2dArray;             //对定位影像进行特征点提取： 输入：定位影像 输出：坐标及描述子
	CSurfExtractor surf;
	surf.run(imageFilename, queryDesc, pt2dArray, mK);


	pDatasetFlann->query(queryDesc, pt2dArray, ratio_threshold,
		matchPt2dArray, matchPt3dArray);

	//if (1)
	//{
	//	char plyFileName[1024];
	//	strcpy(plyFileName, imageFilename);
	//	strcat(plyFileName, ".ply");                                 //打印匹配点
	//	CDatasetRW::writePly(plyFileName, matchPt3dArray);
	//}
	//进行匹配 得到匹配好的matchPt2dArray matchPt3dArray

#ifdef OUPUT_MATCH_PTS 
	char pointFileName[1024];
	strcpy(pointFileName, imageFilename);
	strcat(pointFileName, "c++.txt");                                 //打印匹配点
	CDatasetRW::writePts(pointFileName, matchPt2dArray, matchPt3dArray);
#endif
	//orientation = -9999.0f;
	if (orientation == -9999.0f)
	{
		CLocate3d ljfresect;
		cv::Mat R;
		cv::Point3d C;
		ljfresect.run(matchPt2dArray, matchPt3dArray, R, C, mK);
		X = C.x;
		Y = C.y;

	}
	else {
		cv::Point3d centerPoint;
		for (int i = 0; i < m_datasetArray.size(); i++)
		{
			if (strcmp(roomName, m_datasetArray[i].roomName) == 0)
			{
				centerPoint = m_datasetArray[i].centerPoint;
			}
		}

		for (int i = 0; i < matchPt3dArray.size(); i++)
		{
			matchPt3dArray[i].x -= centerPoint.x;
			matchPt3dArray[i].y -= centerPoint.y;
		}

		cv::Point3d newCenterPoint(0.0, 0.0, 0.0);

		CMonteCarlo2d monteCarlo2d;
		X = 0; Y = 0;
		monteCarlo2d.run(mK, matchPt2dArray, matchPt3dArray, newCenterPoint, orientation,
			X, Y, 500, 1.0);
		newCenterPoint.x = X;
		newCenterPoint.y = Y;

		std::vector<cv::Point2d> orign_matchPt2dArray;
		std::vector<cv::Point3d> orign_matchPt3dArray;
		orign_matchPt2dArray = matchPt2dArray;
		orign_matchPt3dArray = matchPt3dArray;


		//outputPly(imageFilename, "zero_.ply", matchPt3dArray);

		CFilterOutlier  filterOutlier;
		CLocate2d  locate2d;


		filterOutlier.run(matchPt2dArray, matchPt3dArray, orientation, newCenterPoint);

		//filterOutlier.removeByX(matchPt2dArray, matchPt3dArray, orientation, newCenterPoint);

		//outputPly(imageFilename, "MonteCarlo.ply", matchPt3dArray);


		//locate2d.run(mK,matchPt2dArray,matchPt3dArray, newCenterPoint, orientation, X, Y);
		double mx = 0; double my = 0; double moritation = 0;
		locate2d.run_erroranalysis(mK, matchPt2dArray, matchPt3dArray, newCenterPoint, orientation, X, Y, mx, my, moritation);

		matchPt2dArray = orign_matchPt2dArray;
		matchPt3dArray = orign_matchPt3dArray;

		double small_threshold = 10;
		double orientation1 = orientation;


		filterOutlier.run(matchPt2dArray, matchPt3dArray, orientation1, newCenterPoint);
		filterOutlier.removeByX(matchPt2dArray, matchPt3dArray, orientation1, newCenterPoint);
		double residual = monteCarlo2d.project2d(mK, matchPt2dArray, matchPt3dArray, orientation*3.1415926 / 180.0, X, Y, small_threshold);

		if (residual > 20) //failure ljf:失败后，只利用了matchPt2dArray, matchPt3dArray（filterout后）， newCenterPoint（由第一次motacarlod得出），进行下次计算。
		{

			monteCarlo2d.run(mK, matchPt2dArray, matchPt3dArray, newCenterPoint, orientation, X, Y, 200, 0.5);
			locate2d.run(mK, matchPt2dArray, matchPt3dArray, newCenterPoint, orientation, X, Y);
			double residual_secondTime = monteCarlo2d.project2d(mK, matchPt2dArray, matchPt3dArray, orientation*3.1415926 / 180.0, X, Y, small_threshold);
			if (residual_secondTime > 20) {
				X = -9999;
				Y = -9999;
			}

			//使用三维后方交会
			//CLocate3d ljfresect;
			//cv::Mat R;
			//cv::Point3d C;
			//bool b = ljfresect.run(orign_matchPt2dArray, orign_matchPt3dArray, R, C, mK);
			//if (b) {
			//	X = C.x;  Y = C.y;
			//}
			//else {
			//	X = -9999; Y = -9999;
			//}
		}

		//outputPly(imageFilename, "locate2d_.ply", matchPt3dArray);
		if (X != -9999) {
			X += centerPoint.x;
			Y += centerPoint.y;
		}

	}

	return true;
}

bool CLJFlocateRooms::locate(cv::Point3d& initPoint, char* imageFilename, double& X, double& Y, double& orientation)
{

	CDatasetFlann* pDatasetFlann = NULL;

	std::map<int, double > distance;
	int minIndex = 0;
	double  mindis = 99999;
	for (int i = 0; i< m_datasetArray.size(); i++)
	{
		double  tmp;
		tmp = abs(m_datasetArray[i].centerPoint.x - X) + abs(m_datasetArray[i].centerPoint.y - Y);
		if (tmp<mindis)
		{
			mindis = tmp;
			minIndex = i;
		}
	}
	pDatasetFlann = m_datasetArray[minIndex].pDatasetFlann;
	char * roomname = m_datasetArray[minIndex].roomName;

	locateImpl(roomname, pDatasetFlann, imageFilename, X, Y, orientation);

	return true;
}

bool CLJFlocateRooms::locate(char* roomName, char* imageFilename, double& X, double& Y, double& orientation)
{
	double ratio_threshold = 0.6;
	CDatasetFlann* pDatasetFlann = nullptr;

	for (int i = 0; i < m_datasetArray.size(); i++)
	{
		if (strcmp(roomName, m_datasetArray[i].roomName) == 0)
		{
			pDatasetFlann = m_datasetArray[i].pDatasetFlann;
			break;
		}
	}

	locateImpl(roomName, pDatasetFlann, imageFilename, X, Y, orientation);

	return true;
}

bool CLJFlocateRooms::calErrorBygaussianNoise(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, double X, double Y, double orientation, double &mx, double &my, double &moritation) {

	double MaxNoiseX = mx;
	double MaxNoiseY = my;

	//逐渐放大标准差，扩大误差，进行试探，标准差按步长0.5进行增长，总共试探10次
	for (int i = 0; i < 10; i++) {

		// Define random generator with Gaussian distribution
		double mean = 0.0;//均值
		double stddev = 0 + i*0.5;//标准差
		std::default_random_engine generator;
		std::normal_distribution<double> dist(mean, stddev);

		// Add Gaussian noise
		double noiseX = dist(generator);
		double noiseY = dist(generator);

		double X_noise = X + noiseX;
		double Y_noise = Y + noiseY;


		//重投影
		double small_threshold = 1;
		double residual = 0;
		if (matchPt2dArray.size() >= 3) {
			residual = CLJFlocateRooms::myproject2d(mK, matchPt2dArray, matchPt3dArray, orientation*3.1415926 / 180.0, X_noise, Y_noise, small_threshold);
		}


		if (residual < 10)
		{
			if (MaxNoiseX < abs(noiseX))
				MaxNoiseX = abs(noiseX);

			if (MaxNoiseY < abs(noiseY))
				MaxNoiseY = abs(noiseY);
		}
}

	mx = MaxNoiseX;
	my = MaxNoiseY;

	return true;
}

double CLJFlocateRooms::myproject2d(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, double orientation, double Xs, double Ys, double threshold)
{
	double f = mK.at<double>(0, 0);
	double cx = mK.at<double>(0, 2);

	cv::Mat tmp_m, tmp_sd;
	cv::meanStdDev(matchPt2dArray, tmp_m, tmp_sd);
	double std_x = tmp_sd.at<double>(0, 0);

	std::vector<double> resArray;
	double c = cos(orientation);
	double s = sin(orientation);
	int count = 0;
	double sumResidual = 0.0;
	for (size_t i = 0; i < matchPt2dArray.size(); i++) {
		double x = matchPt2dArray[i].x - cx;
		double Xi = matchPt3dArray[i].x;
		double Yi = matchPt3dArray[i].y;


		double den = c*(Xi - Xs) + s*(Yi - Ys);
		double men = -s*(Xi - Xs) + c*(Yi - Ys);
		double xv = den / men*f;
		xv -= x;
		resArray.push_back(xv);
		xv = fabs(xv);

		if (xv / std_x < threshold)  //hard code，Mahalanobis distance
		{
			count++;
			sumResidual = sumResidual + (xv*xv / pow(std_x, 2));
		}
	}
	if (count < matchPt2dArray.size() / 3)
		sumResidual = 9999.0;
	else
		sumResidual = sqrt(sumResidual / count);
	return sumResidual;
}

void outputPly(char* imageFilename, char* postName, std::vector<cv::Point3d>& matchPt3dArray)
{
#ifdef OUPUT_ZERO_PLY
	char plyFileName[1024];
	strcpy(plyFileName, imageFilename);
	strcat(plyFileName, postName);
	CDatasetRW::writePly(plyFileName, matchPt3dArray);
#endif
}

bool CLJFlocateRooms::KmatrixAnddistortMatrixMake(double resizeRatio,cv::Mat& mK, cv::Mat& distort) {

	mK = (cv::Mat_<double>(3, 3) << m_cameraIndex.fx/ resizeRatio, m_cameraIndex.s/resizeRatio, m_cameraIndex.cx / resizeRatio
		, 0, m_cameraIndex.fy/ resizeRatio, m_cameraIndex.cy / resizeRatio
		, 0, 0, 1);

	distort = (cv::Mat_<double>(5, 1) << m_cameraIndex.k1, m_cameraIndex.k2, m_cameraIndex.p1, m_cameraIndex.p2, m_cameraIndex.k3);

	return true;
}