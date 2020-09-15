#include "PMatrixTools.h"
#include "MatrixTools.h"


PMatrixTools::PMatrixTools()
{
}


PMatrixTools::~PMatrixTools()
{
}

//将DP旋转矩阵改为CV旋转矩阵
void PMatrixTools::RMatrixFromDP2CV(ImageEo &imageEo)
{
	cv::Mat_<double>  r(3, 3);

	//计算旋转矩阵
	r = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, -1, 0, 0, 0, -1);
	imageEo.R = r*imageEo.R.t();
}

//p矩阵生成
void PMatrixTools::createPMatrix(ImageEo imageEo, ImageIo imageIo, cv::Mat &P)
{
	cv::Mat_<double> R(3, 3), r(3,3);
	cv::Mat_<double> K(3, 3);
	cv::Mat_<double> C(3, 3);

	//计算旋转矩阵
	//r = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, -1, 0, 0, 0, -1);
	//R = r*imageEo.R.t();
	R = imageEo.R;
	
	//计算K矩阵
	K = (cv::Mat_<double>(3, 3) << imageIo.f, 0, imageIo.x0, 0, imageIo.f, imageIo.y0, 0, 0, 1);
	//计算P矩阵
	P = (cv::Mat_<double>(3, 4) << 1, 0, 0, -imageEo.Xs,
		0, 1, 0, -imageEo.Ys,
		0, 0, 1, -imageEo.Zs);
	P = K*R*P;
}
//p矩阵分解
void PMatrixTools::splitPmatrix(cv::Mat P, ImageIo& imageIo, ImageEo& imageEo)
{
	MatrixTools matrixTools;
	cv::Mat_<double> C(3, 1);
	cv::Mat_<double> K(3, 3), R(3, 3);

	matrixTools.RQSplit(P.colRange(0,3), K, R);

	C = ((K*R).inv())*P.col(3);
	imageEo.Xs = -C(0, 0);
	imageEo.Ys = -C(1, 0);
	imageEo.Zs = -C(2, 0);

	return;
}


//p矩阵前方交会
void PMatrixTools::parameterCompute(cv::Mat P, double x, double y, cv::Mat& parameter)
{
	cv::Mat_<double> mid(2, 4);

	mid(0, 0) = P.at<double>(1, 0) - y * P.at<double>(2, 0);
	mid(0, 1) = P.at<double>(1, 1) - y * P.at<double>(2, 1);
	mid(0, 2) = P.at<double>(1, 2) - y * P.at<double>(2, 2);
	mid(0, 3) = P.at<double>(1, 3) - y * P.at<double>(2, 3);

	mid(1, 0) = x*P.at<double>(2, 0) - P.at<double>(0, 0);
	mid(1, 1) = x*P.at<double>(2, 1) - P.at<double>(0, 1);
	mid(1, 2) = x*P.at<double>(2, 2) - P.at<double>(0, 2);
	mid(1, 3) = x*P.at<double>(2, 3) - P.at<double>(0, 3);

	parameter = mid;
}

void PMatrixTools::ForwardIntersection_P_MultiImg(std::vector<cv::Mat> P,
	VecVecPoint2D &Point2DArray, 
	std::vector<cv::Point3d> & point3DArray)
{
	cv::SVD svd;
	cv::Point3d point3D;
	
	//读取了多张影像的P矩阵，提取特征点中只用到了两张：选取第二张[1]和第五张[4]
	std::vector<cv::Mat> partOfP;
	if(P.size()>=5)
	{
		partOfP.push_back(P[1]);
		partOfP.push_back(P[4]);
	}
	else if (P.size()>=2)
	{
		partOfP.push_back(P[0]);
		partOfP.push_back(P[1]);
	}
	else
	{
		std::cout << "can not read true P" << std::endl;
	}

	for (int i = 0; i < Point2DArray.size(); i++)
	{
		cv::Mat_<double> A(partOfP.size() * 2, 4);
		cv::Mat_<double> XYZ(4, 1);
		if (partOfP.size() != Point2DArray[i].size())
		{
			std::cout << "P矩阵数量与同名点数量不匹配" << '\n';
			std::cout << "P矩阵数量: " << partOfP.size() << '\n';
			std::cout << "同名点数量: " << Point2DArray[i].size() << '\n';
			return;
		}
		for (int j = 0; j < partOfP.size(); j++)
		{
			cv::Mat_<double> midParameters(2, 4);
			parameterCompute(partOfP[j], Point2DArray[i][j].x, Point2DArray[i][j].y, midParameters);
			for (int k = 0; k < 2; k++)
			{
				A(j * 2 + k, 0) = midParameters(k, 0);
				A(j * 2 + k, 1) = midParameters(k, 1);
				A(j * 2 + k, 2) = midParameters(k, 2);
				A(j * 2 + k, 3) = midParameters(k, 3);
			}
		}

		XYZ = svd(A).vt.t().col(3);
		cv::Mat axx = A*XYZ;
		point3D.x = XYZ(0, 0) / XYZ(3, 0);
		point3D.y = XYZ(1, 0) / XYZ(3, 0);
		point3D.z = XYZ(2, 0) / XYZ(3, 0);
		point3DArray.push_back(point3D);

		/*
		for (int i = 0; i < Point2DArray.size(); i++)
		{
		cv::Mat_<double> A(P.size()*2, 4);
		cv::Mat_<double> XYZ(4, 1);
		if (P.size() != Point2DArray[i].size())
		{
		std::cout << "P矩阵数量与同名点数量不匹配" << '\n';
		std::cout << "P矩阵数量: " << P.size() << '\n';
		std::cout << "同名点数量: " << Point2DArray[i].size() << '\n';
		return;
		}
		for (int j = 0; j < P.size(); j++)
		{
		cv::Mat_<double> midParameters(2,4);
		parameterCompute(P[j], Point2DArray[i][j].x, Point2DArray[i][j].y, midParameters);
		for (int k = 0; k < 2; k++)
		{
		A(j * 2 + k, 0) = midParameters(k, 0);
		A(j * 2 + k, 1) = midParameters(k, 1);
		A(j * 2 + k, 2) = midParameters(k, 2);
		A(j * 2 + k, 3) = midParameters(k, 3);
		}
		}
		*/
	}
}


void PMatrixTools::ForwardIntersection_P_MultiImg_MaxDistanceIndex(std::vector<cv::Mat> P,
	VecVecPoint2D &Point2DArray,
	std::vector<cv::Point3d> & point3DArray, int MaxDistanceImgIndex)
{
	cv::SVD svd;
	cv::Point3d point3D;

	//读取了多张影像的P矩阵，提取特征点中只用到了两张：选取第二张[1]和第五张[4]
	std::vector<cv::Mat> partOfP;
	if (P.size() >= 5)
	{
		partOfP.push_back(P[1]);
		partOfP.push_back(P[MaxDistanceImgIndex]);
	}
	else
	{
		std::cout << "can not read true P" << std::endl;
	}

	for (int i = 0; i < Point2DArray.size(); i++)
	{
		cv::Mat_<double> A(partOfP.size() * 2, 4);
		cv::Mat_<double> XYZ(4, 1);
		if (partOfP.size() != Point2DArray[i].size())
		{
			std::cout << "P矩阵数量与同名点数量不匹配" << '\n';
			std::cout << "P矩阵数量: " << partOfP.size() << '\n';
			std::cout << "同名点数量: " << Point2DArray[i].size() << '\n';
			return;
		}
		for (int j = 0; j < partOfP.size(); j++)
		{
			cv::Mat_<double> midParameters(2, 4);
			parameterCompute(partOfP[j], Point2DArray[i][j].x, Point2DArray[i][j].y, midParameters);
			for (int k = 0; k < 2; k++)
			{
				A(j * 2 + k, 0) = midParameters(k, 0);
				A(j * 2 + k, 1) = midParameters(k, 1);
				A(j * 2 + k, 2) = midParameters(k, 2);
				A(j * 2 + k, 3) = midParameters(k, 3);
			}
		}

		XYZ = svd(A).vt.t().col(3);
		cv::Mat axx = A*XYZ;
		point3D.x = XYZ(0, 0) / XYZ(3, 0);
		point3D.y = XYZ(1, 0) / XYZ(3, 0);
		point3D.z = XYZ(2, 0) / XYZ(3, 0);
		point3DArray.push_back(point3D);

		/*
		for (int i = 0; i < Point2DArray.size(); i++)
		{
		cv::Mat_<double> A(P.size()*2, 4);
		cv::Mat_<double> XYZ(4, 1);
		if (P.size() != Point2DArray[i].size())
		{
		std::cout << "P矩阵数量与同名点数量不匹配" << '\n';
		std::cout << "P矩阵数量: " << P.size() << '\n';
		std::cout << "同名点数量: " << Point2DArray[i].size() << '\n';
		return;
		}
		for (int j = 0; j < P.size(); j++)
		{
		cv::Mat_<double> midParameters(2,4);
		parameterCompute(P[j], Point2DArray[i][j].x, Point2DArray[i][j].y, midParameters);
		for (int k = 0; k < 2; k++)
		{
		A(j * 2 + k, 0) = midParameters(k, 0);
		A(j * 2 + k, 1) = midParameters(k, 1);
		A(j * 2 + k, 2) = midParameters(k, 2);
		A(j * 2 + k, 3) = midParameters(k, 3);
		}
		}
		*/
	}
}




//p矩阵投影
void PMatrixTools::projectFrom3D_T_2D_P(cv::Mat P,
	std::vector<cv::Point3d> & point3DArray,
	std::vector<cv::Point2d> &imagePointArray)
{
	for (int i = 0; i < point3DArray.size(); i++)
	{
		cv::Point2d mid;
		cv::Mat point3D = (cv::Mat_<double>(4, 1) << point3DArray[i].x, point3DArray[i].y
			, point3DArray[i].z, 1 );
		cv::Mat_<double> point2D = P*point3D;
		mid.x = point2D(0, 0) / point2D(2, 0);
		mid.y = point2D(1, 0) / point2D(2, 0);
		imagePointArray.push_back(mid);
	}
}

void PMatrixTools::projectFrom3D_T_2D_P(cv::Mat P, cv::Point3d & point3D, cv::Point2d &imagePoint)
{
	cv::Mat midPoint3D = (cv::Mat_<double>(4, 1) << point3D.x, point3D.y
		, point3D.z, 1);
	cv::Mat_<double> point2D = P*midPoint3D;
	imagePoint.x = point2D(0, 0) / point2D(2, 0);
	imagePoint.y = point2D(1, 0) / point2D(2, 0);
}