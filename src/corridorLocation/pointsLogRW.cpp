
#include "pointsLogRW.h"


pointsLogRW::pointsLogRW()
{
}


pointsLogRW::~pointsLogRW()
{
}

bool pointsLogRW::writePly(char* filename, std::vector<cv::Point3d>& pt3dArray)
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

bool pointsLogRW::writePts(char* filename, std::vector<cv::Point2d>& pt2dArray, std::vector<cv::Point3d>& pt3dArray)
{
	std::ofstream out(filename);

	for (size_t i = 0; i < pt2dArray.size(); i++)//得到正确匹配的点对和正确匹配的角度
	{
		cv::Point2d p2d = pt2dArray[i];
		cv::Point3d p3d = pt3dArray[i];
		out << i << "\t" << p2d.x << "\t" << p2d.y << "\t\t"
			<< p3d.x << "\t" << p3d.y << "\t" << p3d.z << "\r\n";//输出像点和物点	

	}
	return true;
}

bool pointsLogRW::readPTO(const char* ptoFileName, std::vector<POINT_PAIR>& ptPairArray)
{
	FILE* fp = fopen(ptoFileName, "r");
	if (fp == NULL)
	{
		printf("error: open file %s failed\n", ptoFileName);
		return false;
	}

	char buffer[1024];
	memset(buffer, 0, 1024);

	POINT_PAIR ptPair;
	char s1[320];
	char s2[320];
	char s3[320];

	while (!feof(fp))
	{
		fgets(buffer, 1024, fp);
		//isPointLine(buffer);
		int num = sscanf_s(buffer, "%s%s%s x%lf y%lf X%lf Y%lf", s1, s2, s3, &ptPair.x1, &ptPair.y1, &ptPair.x2, &ptPair.y2);
		if (7 == num)
			ptPairArray.push_back(ptPair);
		memset(buffer, 0, 1024);
	}
	return true;

}

bool pointsLogRW::writePTO(const char* ptoFileName, std::vector<POINT_PAIR>& ptPairArray, std::string imgFileName1, std::string imgFileName2)
{
	if (ptPairArray.size() < 1)
	{
		//gwprintf("warnning: match file is empty!\n");
		return false;
	}
	FILE* fp = fopen(ptoFileName, "w");
	if (fp == NULL)
	{
		printf("error: open file %s failed\n", ptoFileName);
		return false;
	}
	fprintf(fp, "# Hugin project file\n");
	fprintf(fp, "#hugin_ptoversion 2\n");
	fprintf(fp, "p f2 w9664 h7345 v50  E14.9366 R0 S0,9664,267,7078 n\"TIFF_m c:LZW r:CROP\"\n");
	fprintf(fp, "m g1 i0 f0 m2 p0.00784314\n\n");

	fprintf(fp, "# image lines\n");
	fprintf(fp, "#-hugin  cropFactor=1\n");
	fprintf(fp, "i w10328 h7760 f0 v50 Ra0 Rb0 Rc0 Rd0 Re0 Eev14.9366336120219 Er1 Eb1 r0 p0 y0 TrX0 TrY0 TrZ0 j0 a0 b0 c0 d0 e0 g0 t0 Va1 Vb0 Vc0 Vd0 Vx0 Vy0  Vm5 n\"%s\"\n", imgFileName1);
	fprintf(fp, "#-hugin  cropFactor=1\n");
	fprintf(fp, "i w10328 h7760 f0 v=0 Ra=0 Rb=0 Rc=0 Rd=0 Re=0 Eev14.9366336120219 Er1 Eb1 r0 p0 y0 TrX0 TrY0 TrZ0 j0 a=0 b=0 c=0 d=0 e=0 g=0 t=0 Va=0 Vb=0 Vc=0 Vd=0 Vx=0 Vy=0  Vm5 n\"%s\"\n", imgFileName2);

	fprintf(fp, "\n\nv p1 r1 y1\n\n");
	fprintf(fp, "# automatically generated control points\n");
	for (int i = 0; i <ptPairArray.size(); i++)
	{
		POINT_PAIR& ptPair = ptPairArray[i];
		fprintf(fp, "c n0 N%d x%lf y%lf X%lf Y%lf t0\n",
			1, ptPair.x1, ptPair.y1, ptPair.x2, ptPair.y2);
	}
	fprintf(fp, "\n# :-)");
	fclose(fp);
	return true;
}
