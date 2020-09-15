#include "CgetSoildMatch.h"


CgetSoildMatch::CgetSoildMatch()
{
}

CgetSoildMatch::CgetSoildMatch(int compareNum, int minMeanNum)
{
	m_compareNum = compareNum;
	m_minMeanNum = minMeanNum;
}


CgetSoildMatch::~CgetSoildMatch()
{
}




void CgetSoildMatch::getWorkGroup(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, std::vector<cv::Point2d>& matchPt2dArray_StandardPicture, std::vector<std::string> &matchanchorNameArray, std::vector<float>& distanceArray) {
	std::vector<std::string> maxStringArray;
	std::vector<int> maxStringCountArrary;
	maxNumString(matchanchorNameArray, maxStringArray, maxStringCountArrary);

	if (m_compareNum > maxStringArray.size()) m_compareNum = maxStringArray.size();
	if (m_minMeanNum > maxStringArray.size()) m_minMeanNum = maxStringArray.size();

	std::vector<int> cntVector;

	m_workGroupArray.resize(m_compareNum);
	cntVector.resize(m_compareNum);
	for (size_t i = 0; i < m_compareNum; i++)
	{
		m_workGroupArray[i].anchorName = maxStringArray[i];
		m_workGroupArray[i].matchPt2dArray_work.resize(maxStringCountArrary[i]);
		m_workGroupArray[i].matchPt3dArray_work.resize(maxStringCountArrary[i]);
		m_workGroupArray[i].matchPt2dArray_StandardPicture_work.resize(maxStringCountArrary[i]);
		m_workGroupArray[i].distanceArray.resize(maxStringCountArrary[i]);
	}

	// 循环matchanchorNameArray, 将各个锚点的数据放入workGroupArray中
	for (size_t i = 0; i < matchPt2dArray.size(); i++)
	{
		int isIn = isInNameArray(matchanchorNameArray[i], m_workGroupArray);
		if (isIn != -9999)
		{
			m_workGroupArray[isIn].matchPt2dArray_work[cntVector[isIn]] = matchPt2dArray[i];
			m_workGroupArray[isIn].matchPt3dArray_work[cntVector[isIn]] = matchPt3dArray[i];
			m_workGroupArray[isIn].matchPt2dArray_StandardPicture_work[cntVector[isIn]] = matchPt2dArray_StandardPicture[i];
			m_workGroupArray[isIn].distanceArray[cntVector[isIn]] = distanceArray[i];
			cntVector[isIn]++;
		}
	}
}

void CgetSoildMatch::DistanceMeanSort_getSoildMatch(std::vector<cv::Point2d>& out_matchPt2dArray, std::vector<cv::Point3d>& out_matchPt3dArray) {

	int maxPssNum = 0;
	int maxPassPicOrder = 0;

	// 对特征描述子的距离求平均
	std::vector<float> meanDistanceArray;
	std::vector<size_t> Sortindex_smallMean;
	DistanceMean(m_workGroupArray, meanDistanceArray, Sortindex_smallMean);
	// 筛去特征描述子极少，但平均距离排名靠前的锚点。
	for (size_t i = 0; i < Sortindex_smallMean.size(); )
	{
		if (m_workGroupArray[Sortindex_smallMean[i]].distanceArray.size() <= 10 && (findVerySmallDistanceMatch(m_workGroupArray[Sortindex_smallMean[i]].distanceArray)<2))
			std::vector<size_t>(Sortindex_smallMean.begin() + 1, Sortindex_smallMean.end()).swap(Sortindex_smallMean);
		else
			break;
	}

	for (size_t i = 0; i < m_compareNum; i++)
	{
		int nPosition = 9999;
		std::vector<size_t>::iterator iElement = find(Sortindex_smallMean.begin(), Sortindex_smallMean.end(), i);
		if (iElement != Sortindex_smallMean.end()) nPosition = distance(Sortindex_smallMean.begin(), iElement);

		if (m_workGroupArray[i].matchPt2dArray_work.size() >= 4 && nPosition < m_minMeanNum)
			Homography_cullError(m_workGroupArray[i].matchPt2dArray_work, m_workGroupArray[i].matchPt3dArray_work, m_workGroupArray[i].matchPt2dArray_StandardPicture_work);
		else
			continue;



		if (m_workGroupArray[i].matchPt2dArray_work.size() > maxPssNum) {
			maxPssNum = m_workGroupArray[i].matchPt2dArray_work.size();
			maxPassPicOrder = i;

			out_matchPt2dArray = m_workGroupArray[i].matchPt2dArray_work;
			out_matchPt3dArray = m_workGroupArray[i].matchPt3dArray_work;
		}
	}

	std::cout << "此张照片属于" << m_workGroupArray[maxPassPicOrder].anchorName << "锚点" << std::endl;
}

void CgetSoildMatch::smallDisatanceCntSort_getSoildMatch(std::vector<cv::Point2d>& out_matchPt2dArray, std::vector<cv::Point3d>& out_matchPt3dArray)
{
	int maxPssNum = 0;
	int maxPassPicOrder = 0;

	std::vector<int> smallEnoughDisatanceCountArray;
	std::vector<size_t> Sortindex_smallCnt;
	float smallEnoughDistace = 0.02;
	smallEnoughDisatanceCount(m_workGroupArray, smallEnoughDistace, smallEnoughDisatanceCountArray, Sortindex_smallCnt);

	for (size_t i = 0; i < m_compareNum; i++)
	{
		int nPosition = 9999;
		std::vector<size_t>::iterator iElement = find(Sortindex_smallCnt.begin(), Sortindex_smallCnt.end(), i);
		if (iElement != Sortindex_smallCnt.end()) nPosition = distance(Sortindex_smallCnt.begin(), iElement);

		if (m_workGroupArray[i].matchPt2dArray_work.size() >= 4 && nPosition < m_minMeanNum)
			Homography_cullError(m_workGroupArray[i].matchPt2dArray_work, m_workGroupArray[i].matchPt3dArray_work, m_workGroupArray[i].matchPt2dArray_StandardPicture_work);
		else
			continue;

		if (m_workGroupArray[i].matchPt2dArray_work.size() > maxPssNum) {
			maxPssNum = m_workGroupArray[i].matchPt2dArray_work.size();
			maxPassPicOrder = i;

			out_matchPt2dArray = m_workGroupArray[i].matchPt2dArray_work;
			out_matchPt3dArray = m_workGroupArray[i].matchPt3dArray_work;
		}
	}
	std::cout << "此张照片属于" << m_workGroupArray[maxPassPicOrder].anchorName << "锚点" << std::endl;
}

void CgetSoildMatch::smallDisatanceCntSort_moreAnchor_getSoildMatch(std::vector<cv::Point2d>& out_matchPt2dArray, std::vector<cv::Point3d>& out_matchPt3dArray)
{
	int maxPssNum = 0;
	int maxPassPicOrder = 0;

	std::vector<int> smallEnoughDisatanceCountArray;
	std::vector<size_t> Sortindex_smallCnt;
	float smallEnoughDistace = 0.02;
	smallEnoughDisatanceCount(m_workGroupArray, smallEnoughDistace, smallEnoughDisatanceCountArray, Sortindex_smallCnt);

	for (size_t i = 0; i < m_compareNum; i++)
	{
		int nPosition = 9999;
		std::vector<size_t>::iterator iElement = find(Sortindex_smallCnt.begin(), Sortindex_smallCnt.end(), i);
		if (iElement != Sortindex_smallCnt.end()) nPosition = distance(Sortindex_smallCnt.begin(), iElement);

		if (m_workGroupArray[i].matchPt2dArray_work.size() >= 4 && nPosition < m_minMeanNum)
			Homography_cullError(m_workGroupArray[i].matchPt2dArray_work, m_workGroupArray[i].matchPt3dArray_work, m_workGroupArray[i].matchPt2dArray_StandardPicture_work);
		else
			continue;

		if (m_workGroupArray[i].matchPt2dArray_work.size() > maxPssNum) {
			maxPssNum = m_workGroupArray[i].matchPt2dArray_work.size();
			maxPassPicOrder = i;

			out_matchPt2dArray = m_workGroupArray[i].matchPt2dArray_work;
			out_matchPt3dArray = m_workGroupArray[i].matchPt3dArray_work;
		}
	}

	// 增加空间距离的判断
	cv::Point3d matchedBestAnchorCenter = m_workGroupArray[maxPassPicOrder].centerPoint;
	int smallEnoughDistanceCnt_MatchedBest = smallEnoughDisatanceCountArray[maxPassPicOrder];
	for (size_t i = 0; i < m_compareNum; i++)
	{
		if (i == maxPassPicOrder)
			continue;

		double distance = getDistance(m_workGroupArray[i].centerPoint, matchedBestAnchorCenter);
		double ratio_compareWithBest = double(smallEnoughDisatanceCountArray[i]) / double(smallEnoughDistanceCnt_MatchedBest);

		
		if (distance < 2 && ratio_compareWithBest > 0.3 )
		{
			//相等,代表没有经过H矩阵变换；不相等,代表经过了H矩阵变换
			if(m_workGroupArray[i].matchPt2dArray_work.size() == m_workGroupArray[i].matchPt2dArray_StandardPicture_work.size() && m_workGroupArray[i].matchPt2dArray_work.size() >= 4)
				Homography_cullError(m_workGroupArray[i].matchPt2dArray_work, m_workGroupArray[i].matchPt3dArray_work, m_workGroupArray[i].matchPt2dArray_StandardPicture_work);

			if (m_workGroupArray[i].matchPt2dArray_work.size() <= 4)
				continue;

				
			int allPointNum = out_matchPt2dArray.size() + m_workGroupArray[i].matchPt2dArray_work.size();
			out_matchPt2dArray.reserve(allPointNum);
			out_matchPt3dArray.reserve(allPointNum);
			
			for (size_t j = 0; j < m_workGroupArray[i].matchPt2dArray_work.size(); j++)
			{
				out_matchPt2dArray.push_back(m_workGroupArray[i].matchPt2dArray_work[j]);
				out_matchPt3dArray.push_back(m_workGroupArray[i].matchPt3dArray_work[j]);
			}

			std::cout << "此张照片附加" << m_workGroupArray[i].anchorName << "锚点" << std::endl;
			
		}
	}


	std::cout << "此张照片属于" << m_workGroupArray[maxPassPicOrder].anchorName << "锚点" << std::endl;
	std::cout << "所有正确匹配的特征个数为 " << out_matchPt2dArray.size() << std::endl;
}






template <typename T>
std::vector<size_t> sort_indexes_e(std::vector<T> &v)
{
	std::vector<size_t> idx(v.size());
	iota(idx.begin(), idx.end(), 0);
	sort(idx.begin(), idx.end(),
		[&v](size_t i1, size_t i2) {return v[i1] < v[i2]; });
	return idx;
};

template <typename T>
std::vector<size_t> sort_indexes_i(std::vector<T> &v)
{
	std::vector<size_t> idx(v.size());
	iota(idx.begin(), idx.end(), 0);
	sort(idx.begin(), idx.end(),
		[&v](size_t i1, size_t i2) {return v[i1] > v[i2]; });
	return idx;
};

void CgetSoildMatch::DistanceMean(std::vector<HcullAnchorworkGroup>& workGroupArray, std::vector<float>& meanDistanceArray, std::vector<size_t>& Sortindex) {

	float sum = 0;
	meanDistanceArray.resize(workGroupArray.size());
	for (int i = 0; i < workGroupArray.size(); i++)
	{
		if (workGroupArray[i].distanceArray.size() >= 5)
		{
			sum = std::accumulate(std::begin(workGroupArray[i].distanceArray), std::end(workGroupArray[i].distanceArray), 0.0);
			meanDistanceArray[i] = sum / workGroupArray[i].distanceArray.size();
		}
		else
			meanDistanceArray[i] = 9999;

	}
	Sortindex = sort_indexes_e(meanDistanceArray);
}

void CgetSoildMatch::smallEnoughDisatanceCount(std::vector<HcullAnchorworkGroup>& workGroupArray, float smallEnoughDistace, std::vector<int>& smallEnoughDisatanceCountArray, std::vector<size_t>& Sortindex) {

	smallEnoughDisatanceCountArray.resize(workGroupArray.size());
	int eachWorkGroupNum = 0;
	for (int i = 0; i < workGroupArray.size(); i++)
	{
		int accumulationCnt = 0;
		eachWorkGroupNum = workGroupArray[i].distanceArray.size();
		for (int j = 0; j < eachWorkGroupNum; j++)
		{
			if (workGroupArray[i].distanceArray[j] < smallEnoughDistace)
				smallEnoughDisatanceCountArray[i]++;

			if (j % 5 == 0) {
				workGroupArray[i].centerPoint.x += (workGroupArray[i].matchPt3dArray_work[j].x );
				workGroupArray[i].centerPoint.y += (workGroupArray[i].matchPt3dArray_work[j].y );
				workGroupArray[i].centerPoint.z += workGroupArray[i].matchPt3dArray_work[j].z;
				accumulationCnt++;
			}
		}
		workGroupArray[i].centerPoint.x = workGroupArray[i].centerPoint.x / accumulationCnt ;
		workGroupArray[i].centerPoint.y = workGroupArray[i].centerPoint.y / accumulationCnt ;
		workGroupArray[i].centerPoint.z = workGroupArray[i].centerPoint.z / accumulationCnt;
	}

	Sortindex = sort_indexes_i(smallEnoughDisatanceCountArray);

}

int CgetSoildMatch::isInNameArray(std::string in_anchorName, std::vector<HcullAnchorworkGroup> workGroupArray) {
	int isINameArrayCnt = -9999;
	for (int i = 0; i < workGroupArray.size(); i++)
	{
		if (in_anchorName == workGroupArray[i].anchorName)
		{
			isINameArrayCnt = i;
			break;
		}
	}
	return isINameArrayCnt;
}

bool CgetSoildMatch::maxNumString(std::vector<std::string> testList, std::vector<std::string>& maxStringArray, std::vector<int>& maxStringCountArray) {

	std::map<std::string, int> keyList; //以元素值作为key,计数作为value
	for (std::vector<std::string>::iterator it = testList.begin(); it != testList.end(); it++)
	{
		keyList[*it]++;
	}

	std::vector<PAIR> vec(keyList.begin(), keyList.end());
	std::sort(vec.begin(), vec.end(), cmp_val);
	maxStringArray.resize(vec.size());    maxStringCountArray.resize(vec.size());
	int i = 0;
	for (std::vector<PAIR>::iterator ite = vec.begin(); ite != vec.end(); ++ite)
	{
		maxStringArray[i] = ite->first;
		maxStringCountArray[i] = ite->second;
		i++;
	}

	return true;
}

bool CgetSoildMatch::cmp_val(const PAIR &left, const PAIR &right)
{
	return left.second > right.second;
}

int CgetSoildMatch::findVerySmallDistanceMatch(std::vector<float> distanceArray)
{
	int verySmallDistanceMatch = 0;
	for (size_t i = 0; i < distanceArray.size(); i++)
	{
		if (distanceArray[i] <= 0.01)
			verySmallDistanceMatch++;
	}
	return verySmallDistanceMatch;
}

bool CgetSoildMatch::Homography_cullError(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, std::vector<cv::Point2d>& matchPt2dArray_OnstandardPicutre) {
	//cal Homography
	cv::Mat H = cv::findHomography(matchPt2dArray, matchPt2dArray_OnstandardPicutre, CV_RANSAC);
	H.convertTo(H, CV_64F);

	//use the Homography to cull error points
	int pass_HomographyNum = 0;
	double residual;
	if (H.cols == 3) {

		for (size_t k = 0; k < matchPt2dArray.size(); k++) {
			cv::Mat srcPoints = cv::Mat::ones(3, 1, CV_64F);
			srcPoints.at<double>(0, 0) = matchPt2dArray[k].x;
			srcPoints.at<double>(1, 0) = matchPt2dArray[k].y;
			srcPoints.at<double>(2, 0) = 1.0;

			cv::Mat calDstPoints = H * srcPoints;
			cv::Point2d H_convertion = {
				calDstPoints.at<double>(0, 0) / calDstPoints.at<double>(2, 0),
				calDstPoints.at<double>(1, 0) / calDstPoints.at<double>(2, 0) };

			residual = std::fabs(H_convertion.x - matchPt2dArray_OnstandardPicutre[k].x) +
				std::fabs(H_convertion.y - matchPt2dArray_OnstandardPicutre[k].y);

			if (residual < 5) {
				matchPt2dArray[pass_HomographyNum] = matchPt2dArray[k];
				matchPt3dArray[pass_HomographyNum] = matchPt3dArray[k];
				pass_HomographyNum++;
			}
		}

		matchPt2dArray.resize(pass_HomographyNum);
		matchPt3dArray.resize(pass_HomographyNum);
	}
	else {
		matchPt2dArray.resize(0);
		matchPt3dArray.resize(0);
	}

	return true;
}

double CgetSoildMatch::getDistance(cv::Point3d pointO, cv::Point3d pointA)

{

	double distance;

	distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2) ;

	distance = sqrtf(distance);

	return distance;

}


