
#include "DatasetRW.h"



CDatasetRW::CDatasetRW()
{
}


CDatasetRW::~CDatasetRW()
{
}

void CDatasetRW::loadOBJ(std::string filename)
{
		std::ifstream fin(filename, std::ios::binary);
		if (!fin || fin.bad()) {
			return;
		}
		m_objs.clear();
		int cnt = 0;
		fin.read((char*)&cnt, sizeof(cnt));
		for (int i = 0; i < cnt; i++)
		{
			CDatasetRW::OBJ t;
			fin.read((char*)&(t.id), sizeof(t.id));
			fin.read((char*)&(t.pt.x), sizeof(t.pt.x));
			fin.read((char*)&(t.pt.y), sizeof(t.pt.y));
			fin.read((char*)&(t.pt.z), sizeof(t.pt.z));

			int sucnt = 0;
			fin.read((char*)&(sucnt), sizeof(sucnt));
			t.marks.resize(sucnt);
			for (size_t j = 0; j < sucnt; j++) {
				fin.read((char*)&(t.marks[j]), sizeof(t.marks[j]));
			}
			int rows = 0;
			int cols = 0;
			int type_ = 0;

			fin.read((char*)&(rows), sizeof(rows));
			fin.read((char*)&(cols), sizeof(cols));
			fin.read((char*)&(type_), sizeof(type_));
			t.des = cv::Mat(rows, cols, type_);
			int total_ = rows*t.des.step[0];
			fin.read((char*)(t.des.data), total_);

			fin.read((char*)&(t.colors.x), sizeof(t.colors.x));
			fin.read((char*)&(t.colors.y), sizeof(t.colors.y));
			fin.read((char*)&(t.colors.z), sizeof(t.colors.z));

			m_objs[t.id] = t;
		}
		fin.close();
}

void CDatasetRW::loadOBJ_corridor(std::string filename) {
	std::ifstream fin(filename, std::ios::binary);
	if (!fin || fin.bad()) {
		return;
	}
	m_objs.clear();
	
	int id = 0;
	while(fin.good()) {
		cv::Point3d tmpPoint3d;
		cv::Point2d tmpPoint2d;
		char tmpAnchorName[256];
		cv::Mat tmpDesc(1, 64, CV_32FC1);

		CDatasetRW::OBJ t;
		t.id = id;
		fin.read(tmpAnchorName, 100);
		fin.read((char*)&tmpPoint2d.x, sizeof(double));
		fin.read((char*)&tmpPoint2d.y, sizeof(double));
		fin.read((char*)&tmpPoint3d.x, sizeof(double));
		fin.read((char*)&tmpPoint3d.y, sizeof(double));
		fin.read((char*)&tmpPoint3d.z, sizeof(double));
		fin.read((char*)tmpDesc.data, tmpDesc.step[0]);

		t.imgpoints = tmpPoint2d; t.pt = tmpPoint3d;  t.des = tmpDesc;   t.anchorName = std::string(tmpAnchorName);

		m_objs[t.id] = t;
		id++;
	}
}

bool CDatasetRW::writePly(char* filename, std::vector<cv::Point3d>& pt3dArray)
{

	FILE* fp = fopen(filename, "w");
	if (fp == NULL)
	{
		printf("error: create file %s failed\n", filename);
		return false;
	}

	int pointNum = pt3dArray.size();
	fprintf(fp, "ply\r\n");
	fprintf(fp, "format ascii 1.0\r\n");
	fprintf(fp, "comment VCGLIB generated\r\n");
	fprintf(fp, "element vertex %d\r\n", pointNum);
	fprintf(fp, "property float x\r\n");
	fprintf(fp, "property float y\r\n");
	fprintf(fp, "property float z\r\n");
	fprintf(fp, "end_header\r\n");

	for (int i = 0; i < pointNum; i++)
	{
		fprintf(fp, "%f %f %f\r\n", pt3dArray[i].x, pt3dArray[i].y, pt3dArray[i].z);
	}
	fclose(fp);
	printf("create ply file %s completed\n", filename);
	return false;
}

bool CDatasetRW::writePts(char* filename, std::vector<cv::Point2d>& pt2dArray, std::vector<cv::Point3d>& pt3dArray)
{
	std::ofstream out(filename);

	for (size_t i = 0; i < pt2dArray.size(); i++)//得到正确匹配的点对和正确匹配的角度
	{
		cv::Point2d p2d = pt2dArray[i];
		cv::Point3d p3d = pt3dArray[i];
		out <<  i << "\t" << p2d.x << "\t" << p2d.y << "\t\t" 
			<< p3d.x << "\t" << p3d.y << "\t" << p3d.z << "\r\n";//输出像点和物点	
		
	}
	return true;
}

bool CDatasetRW::run(char* filename, cv::Mat& descriptors, std::vector<cv::Point3d>& pt3dArray)
{
	int featureNum = m_objs.size();
	descriptors = cv::Mat(featureNum, 64, CV_32F);
	pt3dArray.reserve(featureNum);
	
	//cv::Mat M1(3, 3, CV_8UC4, cv::Scalar(0, 0, 0, 255));
	descriptors = cv::Mat();
	int iRow = 0;
	for (auto &it : m_objs) {
		descriptors.push_back(it.second.des);
		//descriptors.row(iRow) = it.second.des;
		pt3dArray.push_back(it.second.pt);
		iRow++;
	}

	return true;
}

bool CDatasetRW::run_corridor(char* filename, cv::Mat& descriptors, std::vector<cv::Point3d>& pt3dArray,std::vector<cv::Point2d> &pt2dArray, std::vector<std::string>& anchorNameArray)
{
	int featureNum = m_objs.size();
	descriptors = cv::Mat(featureNum, 64, CV_32F);
	pt3dArray.reserve(featureNum);
	pt2dArray.reserve(featureNum);
	anchorNameArray.reserve(featureNum);

	//cv::Mat M1(3, 3, CV_8UC4, cv::Scalar(0, 0, 0, 255));
	descriptors = cv::Mat();
	int iRow = 0;
	for (auto &it : m_objs) {
		descriptors.push_back(it.second.des);
		//descriptors.row(iRow) = it.second.des;
		pt3dArray.push_back(it.second.pt);
		pt2dArray.push_back(it.second.imgpoints);
		anchorNameArray.push_back(it.second.anchorName);
		iRow++;
		//	if (iRow > 10000)
		//		break;
	}

	//writePly("points214.ply", pt3dArray);
	return true;
}