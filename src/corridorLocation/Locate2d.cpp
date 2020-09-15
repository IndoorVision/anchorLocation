#include "Locate2d.h"
#include "DatasetRW.h"


CLocate2d::CLocate2d()
{
}

CLocate2d::~CLocate2d()
{
}


void CLocate2d::removeBigResidual(cv::Mat mK,std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, float orientation, double Xs, double Ys, double threshold)
{
	std::vector<cv::Point2d> rigmatchPt2dArray;
	std::vector<cv::Point3d> rigMatchPt3dArray;
	std::vector<double> resudualArray;

	double f = mK.at<double>(0, 0);
	double cx = mK.at<double>(0, 2);
	
	//float project2d(const std::vector<Data> &datas, float f, float cx, float sita, float Xs, float Ys)
	
		double c = cos(orientation);
		double s = sin(orientation);
		double sum_xv = 0;
		for (size_t i = 0; i < matchPt2dArray.size(); i++) {
			double x = matchPt2dArray[i].x - cx;
			double Xi = matchPt3dArray[i].x;
			double Yi = matchPt3dArray[i].y;


			double den = c*(Xi - Xs) + s*(Yi - Ys);
			double men = -s*(Xi - Xs) + c*(Yi - Ys);
			double xv = den / men*f;
			xv -= x;
			resudualArray.push_back(xv);
			xv = fabs(xv);			
			if (xv < threshold)  //hard code
			{
				rigmatchPt2dArray.push_back(matchPt2dArray[i]);
				rigMatchPt3dArray.push_back(matchPt3dArray[i]);
			}
		}	
	matchPt2dArray.swap(rigmatchPt2dArray);
	matchPt3dArray.swap(rigMatchPt3dArray);
}

//cv::Mat loactionCollinear2D_iter(const std::vector<Data> &datas, float f, float cx, double Xs_0, double Ys_0, double sita_0, double &vx)
cv::Mat CLocate2d::loactionCollinear2D_iter(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, double orientation_0, double Xs_0, double Ys_0, double &vx)
{
	double f = mK.at<double>(0, 0);
	double cx = mK.at<double>(0, 2);
	int n = matchPt2dArray.size();
	double Xs = Xs_0;
	double Ys = Ys_0;
	double sita = orientation_0;
	cv::Mat A(n, 3, CV_64F, cv::Scalar(0));
	cv::Mat L(n, 1, CV_64F, cv::Scalar(0));
	cv::Mat P = cv::Mat(n, n, CV_64F, cv::Scalar(0));
	cv::setIdentity(P);
	cv::Mat X;

	for (size_t m = 0; m < 5; m++) {
		//填充数据
		for (int i = 0; i < n; i++) {
			double x =  matchPt2dArray[i].x - cx;
			double Xi = matchPt3dArray[i].x;
			double Yi = matchPt3dArray[i].y;

			double s = sin(sita);
			double c = cos(sita);

			double dXs = s*x + f*c;
			double dYs = -c*x + f*s;
			double dsita = x*(-c*(Xi - Xs) - s*(Yi - Ys)) - f*(-s*(Xi - Xs) + c*(Yi - Ys));

			A.at<double>(i, 0) = dXs;
			A.at<double>(i, 1) = dYs;
			A.at<double>(i, 2) = dsita;

			double F_ = x*(-s*(Xi - Xs) + c*(Yi - Ys)) - f*(c*(Xi - Xs) + s*(Yi - Ys));
			L.at<double>(i, 0) = -F_;

		}
		//求解
		X = (A.t()*P*A).inv()*A.t()*P*L;
		cv::Mat V = A*X - L;
		vx = cv::norm(V);
		V = V.mul(V);
		V /= cv::norm(V);
		double* dv = (double*)V.data;
		for (int i = 0; i < n; i++) {
			P.at<double>(i, i) = 1.0 / (fabs(dv[i]) + 0.00001);
		}
	}

	return X;

}

cv::Mat CLocate2d::loactionCollinear2D_iter_erroranalysis(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, double orientation_0, double Xs_0, double Ys_0, double &vx, double &mx, double &my, double &moritation)
{
	double f = mK.at<double>(0, 0);
	double cx = mK.at<double>(0, 2);
	int n = matchPt2dArray.size();
	double Xs = Xs_0;
	double Ys = Ys_0;
	double sita = orientation_0;
	cv::Mat A(n, 3, CV_64F, cv::Scalar(0));
	cv::Mat L(n, 1, CV_64F, cv::Scalar(0));
	cv::Mat P = cv::Mat(n, n, CV_64F, cv::Scalar(0));
	cv::setIdentity(P);
	cv::Mat X;

	for (size_t m = 0; m < 5; m++) {
		//填充数据
		for (int i = 0; i < n; i++) {
			double x = matchPt2dArray[i].x - cx;
			double Xi = matchPt3dArray[i].x;
			double Yi = matchPt3dArray[i].y;

			double s = sin(sita);
			double c = cos(sita);

			double dXs = s*x + f*c;
			double dYs = -c*x + f*s;
			double dsita = x*(-c*(Xi - Xs) - s*(Yi - Ys)) - f*(-s*(Xi - Xs) + c*(Yi - Ys));

			A.at<double>(i, 0) = dXs;
			A.at<double>(i, 1) = dYs;
			A.at<double>(i, 2) = dsita;

			double F_ = x*(-s*(Xi - Xs) + c*(Yi - Ys)) - f*(c*(Xi - Xs) + s*(Yi - Ys));
			L.at<double>(i, 0) = -F_;

		}
		//求解
		X = (A.t()*P*A).inv()*A.t()*P*L;
		cv::Mat V = A*X - L;
		vx = cv::norm(V);
		V = V.mul(V);
		V /= cv::norm(V);
		double* dv = (double*)V.data;
		for (int i = 0; i < n; i++) {
			P.at<double>(i, i) = 1.0 / (fabs(dv[i]) + 0.00001);
		}

		//求解误差mx,my,moritation
		cv::Mat Q(3, 3, CV_64F, cv::Scalar(0));
		Q = (A.t()*A).inv();
		double m0 = sqrt( abs(vx / ( n - 3)));
		mx = m0 * sqrt(Q.at<double>(0, 0));
		my = m0 * sqrt(Q.at<double>(1, 1));
		moritation = m0 * sqrt(Q.at<double>(2, 2));
	}

	return X;

}

void fillThresholdArray(std::vector<double>& thresholdArray)
{
	
	thresholdArray.push_back(300);
	thresholdArray.push_back(250);
	thresholdArray.push_back(200);
	thresholdArray.push_back(150);
	thresholdArray.push_back(100);
	thresholdArray.push_back(90);
	thresholdArray.push_back(80);
	thresholdArray.push_back(70);
	thresholdArray.push_back(60);
	thresholdArray.push_back(50);
	thresholdArray.push_back(50);
	thresholdArray.push_back(50);
	thresholdArray.push_back(50);
}
void fillThresholdArray1(std::vector<double>& thresholdArray)
{
	
	thresholdArray.push_back(10000);
	thresholdArray.push_back(10000);
	thresholdArray.push_back(10000);
	thresholdArray.push_back(10000);
	thresholdArray.push_back(10000);
	thresholdArray.push_back(10000);
	thresholdArray.push_back(10000);
	thresholdArray.push_back(10000);
	thresholdArray.push_back(10000);
	thresholdArray.push_back(10000);
	thresholdArray.push_back(300);
	thresholdArray.push_back(250);
	thresholdArray.push_back(200);
	thresholdArray.push_back(150);
	thresholdArray.push_back(100);
	thresholdArray.push_back(50);
	thresholdArray.push_back(50);
}
bool CLocate2d::run(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, cv::Point3d centerPoint, double& orientation, double& X, double& Y)
{

	//取均值
	double Xs = 0;
	double Ys = 0;
	orientation = orientation * CV_PI / 180.0;
	double sita = orientation;

	Xs = centerPoint.x;
	Ys = centerPoint.y;
	std::vector<double> thresholdArray;
	fillThresholdArray1(thresholdArray);

	cv::Mat XX;
	int iterCount = 0;
	double vx = 0;
	
	do
	{
		XX = loactionCollinear2D_iter(mK, matchPt2dArray, matchPt3dArray, sita, Xs, Ys, vx);
		Xs += XX.at<double>(0, 0);
		Ys += XX.at<double>(1, 0);
		sita += XX.at<double>(2, 0);

		while (sita>2 * CV_PI) 			sita -= 2 * CV_PI;
		while (sita <0) 			sita += 2 * CV_PI;		

		removeBigResidual(mK, matchPt2dArray, matchPt3dArray, sita, Xs, Ys, thresholdArray[iterCount]);

		if (matchPt2dArray.size() <= 1)
			break;


	//	char filename[32];
	//	sprintf(filename, "3dPt%d.ply", iterCount);
	//	CDatasetRW::writePly(filename, matchPt3dArray);

		if (++iterCount > 10) {
			break;
		}
	} while (cv::norm(XX, cv::NORM_L2)>0.001);
	Result res;
	res.C.x = Xs;
	res.C.y = Ys;
	res.angle = sita / CV_PI * 180;
	res.res = vx;

	X = Xs;
	Y = Ys;
	orientation = sita / CV_PI * 180;

	return true;


}

bool CLocate2d::run_erroranalysis(cv::Mat mK, std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, cv::Point3d centerPoint, double& orientation, double& X, double& Y, double &mx, double &my, double &moritation)
{

	//取均值
	double Xs = 0;
	double Ys = 0;
	orientation = orientation * CV_PI / 180.0;
	double sita = orientation;

	Xs = centerPoint.x;
	Ys = centerPoint.y;
	std::vector<double> thresholdArray;
	fillThresholdArray1(thresholdArray);

	cv::Mat XX;
	int iterCount = 0;
	double vx = 0;


	do
	{
		XX = loactionCollinear2D_iter_erroranalysis(mK, matchPt2dArray, matchPt3dArray, sita, Xs, Ys, vx, mx, my, moritation);
		Xs += XX.at<double>(0, 0);
		Ys += XX.at<double>(1, 0);
		sita += XX.at<double>(2, 0);

		while (sita>2 * CV_PI) 			sita -= 2 * CV_PI;
		while (sita <0) 			sita += 2 * CV_PI;

		removeBigResidual(mK, matchPt2dArray, matchPt3dArray, sita, Xs, Ys, thresholdArray[iterCount]);

		if (matchPt2dArray.size() <= 3)
			break;


		//	char filename[32];
		//	sprintf(filename, "3dPt%d.ply", iterCount);
		//	CDatasetRW::writePly(filename, matchPt3dArray);

		if (++iterCount > 10) {
			break;
		}
	} while (cv::norm(XX, cv::NORM_L2)>0.001);
	Result res;
	res.C.x = Xs;
	res.C.y = Ys;
	res.angle = sita / CV_PI * 180;
	res.res = vx;

	X = Xs;
	Y = Ys;
	orientation = sita / CV_PI * 180;

	return true;


}

