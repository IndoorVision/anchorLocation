#include "MonteCarlo2d.h"

#include "MeanByMaxBin.h"
CMonteCarlo2d::CMonteCarlo2d()
{
}


CMonteCarlo2d::~CMonteCarlo2d()
{
}


double CMonteCarlo2d::project2d(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, double orientation, double Xs, double Ys, double threshold)
{
	double f = mK.at<double>(0, 0);
	double cx = mK.at<double>(0, 2);
	
	//orientation = 90 *3.1415926 / 180;
	std::vector<double> resArray;
	double c = cos(orientation);
	double s = sin(orientation);
	int count  = 0;
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

		if (xv < threshold)  //hard code
		{
			count++;
			sumResidual = sumResidual+ (xv*xv);
		}
	}
	if (count < matchPt2dArray.size() / 3)
		sumResidual = 9999.0;
	else
		sumResidual = sqrt(sumResidual / count);
	return sumResidual;
}

double  CMonteCarlo2d::estimateOri(std::vector<cv::Point3d>& matchPt3dArray, cv::Point2d& centerPoint)
{
	double sumAngle = 0.0;
 	for (int i = 0; i < matchPt3dArray.size(); i++)
	{
		double delX = matchPt3dArray.at(i).x - centerPoint.x;
		double delY = matchPt3dArray.at(i).y - centerPoint.y;
		double angleLine = atan2(delY, delX) / 3.1415 * 180;
		sumAngle += angleLine;
	}
	double meanAngle = sumAngle / matchPt3dArray.size();

	CMeanByMaxBin meanByMaxBin;
	meanAngle = meanByMaxBin.estimateOri(matchPt3dArray, centerPoint);
	

	sumAngle = 0.0;
	int count = 0;
	for (int i = 0; i < matchPt3dArray.size(); i++)
	{
		double delX = matchPt3dArray.at(i).x - centerPoint.x;
		double delY = matchPt3dArray.at(i).y - centerPoint.y;
		double angleLine = atan2(delY, delX) / 3.1415 * 180;

		double diffAngle = angleLine - meanAngle;
		if (diffAngle < -180)		diffAngle += 360;
		if (diffAngle > 180)    	diffAngle -= 360;
		if (fabs(diffAngle) < 60) //hard code
		{
			sumAngle += diffAngle;
			count++;
		}
	}
	double meanAngle1 = sumAngle / count;
	meanAngle1 += meanAngle;
	return meanAngle1;
}

struct XYOR
{
	double X;
	double Y;
	
	double ori;
	double residual;
	

};
bool CMonteCarlo2d::run(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, cv::Point3d centerPoint, double& orientation, double& X, double& Y, double threshold, double dStep)
{
	std::vector<XYOR> xyorArray;

	double Xs = centerPoint.x;
	double Ys = centerPoint.y;
//	double threshold = 50;
	 dStep = 0.5;

	std::vector<cv::Point2d> XsYsArray;
	cv::Point2d pt;
	pt.x = 3;	pt.y = 2; XsYsArray.push_back(pt);
	pt.x = -6;	pt.y = -3.5; XsYsArray.push_back(pt);
	pt.x = 2.0;	pt.y = 1.5; XsYsArray.push_back(pt);

	for (double x = -7; x <= 7; x += dStep)
	for (double y = -4; y <= 4; y += dStep)
	{
		pt.x = Xs + x;
		pt.y = Ys + y;
		XsYsArray.push_back(pt);
	}
	

	//XsYsArray.clear();
	//pt.x = 3;	pt.y = 2; XsYsArray.push_back(pt);
	//pt.x = -2;	pt.y = 2; XsYsArray.push_back(pt);
	//pt.x = -2;	pt.y = -2; XsYsArray.push_back(pt);

	cv::Point2d goodPt;
	double  goodOri;
	double minResidual = 9999.0;
	double degree2rad = 3.1415926 / 180;

	double compassOrientation = 90;
	for (int i = 0; i < XsYsArray.size(); i++)
	{
		
		double  estOri = estimateOri(matchPt3dArray, XsYsArray[i]);
		estOri = estOri - 90;

	//	double diffAngle = estOri - compassOrientation;
	//	if (diffAngle < -180)		diffAngle += 360;
	//	if (diffAngle > 180)    	diffAngle -= 360;
	//	if (abs(diffAngle) > 30)
	//		continue;
		double residual = project2d(mK, matchPt2dArray, matchPt3dArray, estOri*degree2rad, XsYsArray[i].x, XsYsArray[i].y, threshold);
		if (residual < minResidual)
		{
			minResidual = residual;
			goodPt = XsYsArray[i];
			goodOri = estOri;
		}

		if (residual < 400)
		{
			XYOR xyor;
			xyor.X = XsYsArray[i].x;	xyor.Y = XsYsArray[i].y;
			xyor.ori = estOri;	xyor.residual = residual;
			xyorArray.push_back(xyor);
		}

	}
	printf("the goodPt: x = %lf, y =%lf ; the goodOri is %lf\n", goodPt.x, goodPt.y, goodOri);

	orientation = goodOri;
	X = goodPt.x;
	Y = goodPt.y;
	return true;
}