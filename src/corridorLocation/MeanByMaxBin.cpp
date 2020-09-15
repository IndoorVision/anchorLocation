
#include "MeanByMaxBin.h"


CMeanByMaxBin::CMeanByMaxBin()
{
}


CMeanByMaxBin::~CMeanByMaxBin()
{
}

void CMeanByMaxBin::buildBin(std::vector<cv::Point3d>& matchPt3dArray, cv::Point2d& centerPoint, double interval)
{
	int binNum = int (360 / interval +0.5);
	m_binCountArray.resize(binNum,0);

	for (int i = 0; i < matchPt3dArray.size(); i++)
	{
		double delX = matchPt3dArray.at(i).x - centerPoint.x;
		double delY = matchPt3dArray.at(i).y - centerPoint.y;
		double angleLine = atan2(delY, delX) / 3.1415 * 180;
		if (angleLine > 360) angleLine -= 360;
		if (angleLine < 0) angleLine += 360;

		int binIndex = int(angleLine / interval + 0.5);
		if (binIndex >= binNum) binIndex -= binNum;
		if (binIndex < 0) binIndex += binNum;

		m_binCountArray[binIndex]++;		
	}


}
double CMeanByMaxBin::selectBin(double interval)
{
	
	int maxBinIndex = 0;
	int maxCount = 0;
	for (int i = 0; i < m_binCountArray.size(); i++)
	{
		if (m_binCountArray[i] > maxCount)
		{
			maxBinIndex = i;
			maxCount = m_binCountArray[i];
		}
	}
	double angle = -1.0;
	angle = (maxBinIndex + 0.5)*interval;
	return angle;

}


double  CMeanByMaxBin::estimateOri(std::vector<cv::Point3d>& matchPt3dArray, cv::Point2d& centerPoint)
{
	double interval = 20.0;
	buildBin(matchPt3dArray, centerPoint, interval);
	double angle = selectBin(interval);
	return angle;
}