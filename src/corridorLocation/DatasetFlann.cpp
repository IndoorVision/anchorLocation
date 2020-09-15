
#include "DatasetFlann.h"
#include "DatasetRW.h"
#include "Runtime.h"
#include "SurfExtractor.h"
#include <omp.h>



CDatasetFlann::CDatasetFlann()
{
}


CDatasetFlann::~CDatasetFlann()
{
}

//#define ANDROID_STUDIO 
void CDatasetFlann::buildIndex(const cv::Mat& describ1,	int imethod)
{
	//CRuntime t("cv::flann::Index init");
#ifdef ANDROID_STUDIO
	if(imethod == 0){
		m_flannIndex.build( describ1, cv::flann::KDTreeIndexParams(6));
	}
	else if (imethod == 1)
	{
		m_flannIndex.build(describ1, cv::flann::KMeansIndexParams());
	}
	else if (imethod == 2)
	{	
		m_flannIndex.build(describ1, cv::flann::HierarchicalClusteringIndexParams(32));
	}	
#else
	if (imethod == 0) {
		m_flannIndex.build(cv::Mat(), describ1, cv::flann::KDTreeIndexParams(6));
	}
	else if (imethod == 1)
	{
		m_flannIndex.build(cv::Mat(), describ1, cv::flann::KMeansIndexParams());
	}
	else if (imethod == 2)
	{
		m_flannIndex.build(cv::Mat(), describ1, cv::flann::HierarchicalClusteringIndexParams(32));
	}
#endif 	
}
bool CDatasetFlann::load(char* filename)
{
	const char *current;;
	char const *corridor = "corridor";
	current = strstr(filename, corridor);

	if (current == NULL) {
		CDatasetRW rw;
		std::string str;
		str = filename;
		rw.loadOBJ(str);
		rw.run(filename, m_datasetDesc, m_pt3dArray);   
		m_placeType = 0; // 设置为房间

		int imethod = 0;
		buildIndex(m_datasetDesc, imethod);
		return true;
	}


	if (current != NULL) {

		CDatasetRW rw;
		std::string str;
		str = filename;
		rw.loadOBJ_corridor(str);
		rw.run_corridor(filename, m_datasetDesc, m_pt3dArray, m_pt2dArray, m_anchorNameArray);   
		m_placeType = 1;  //设置为走廊

		int imethod = 0;
		buildIndex(m_datasetDesc, imethod);
		return true;

	}
}

void CDatasetFlann::query(cv::Mat& queryDesc, std::vector<cv::Point2d>& pt2dArray, double ratio_threshold,
	std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray)
{
	std::vector<cv::DMatch> matches;
	match(queryDesc, matches, ratio_threshold);
	
	//m_pt3dArray
	//pt2dArray
	int num = matches.size();
	if (num > 200)
		num = 200;
	for (int i = 0; i < num; i++)
	{
		matchPt2dArray.push_back(pt2dArray[matches[i].queryIdx]);
		matchPt3dArray.push_back(m_pt3dArray[matches[i].trainIdx]);
	}
}

void CDatasetFlann::query_corridor(cv::Mat& queryDesc, std::vector<cv::Point2d>& pt2dArray, double ratio_threshold,
	std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, std::vector<cv::Point2d>& matchPt2dArray_StandardPicture, std::vector<std::string> &matchanchorNameArray)
{
	
	std::vector<cv::DMatch> matches;
	match_corridor(queryDesc, matches, ratio_threshold);

	std::vector<cv::Point2d> oriMatched_matchPt2dArray;  oriMatched_matchPt2dArray.reserve(matches.size());
	std::vector<cv::Point3d> oriMatched_matchPt3dArray;  oriMatched_matchPt3dArray.reserve(matches.size());
	std::vector<cv::Point2d> oriMatched_matchPt2dArray_StandardPicture;   oriMatched_matchPt2dArray_StandardPicture.reserve(matches.size());
	matchanchorNameArray.reserve(matches.size());
	std::vector<float> oriMatched_distance; oriMatched_distance.reserve(matches.size());

	int num = matches.size();
	for (int i = 0; i < num; i++)
	{
		// step1 -- 查找有无重复点
		bool is_3d_element_in_vector = is_element_in_vector(oriMatched_matchPt3dArray, m_pt3dArray[matches[i].trainIdx]);
		bool is_2d_element_in_vector = is_element_in_vector(oriMatched_matchPt2dArray, pt2dArray[matches[i].queryIdx]);

		// step2 -- 若无重复点，则保存
		if (!is_3d_element_in_vector && !is_2d_element_in_vector) {
			oriMatched_matchPt2dArray.push_back(pt2dArray[matches[i].queryIdx]);
			oriMatched_matchPt3dArray.push_back(m_pt3dArray[matches[i].trainIdx]);
			oriMatched_matchPt2dArray_StandardPicture.push_back(m_pt2dArray[matches[i].trainIdx]);
			matchanchorNameArray.push_back(m_anchorNameArray[matches[i].trainIdx]);

			oriMatched_distance.push_back(matches[i].distance);
		}
	}
	
	CgetSoildMatch mygetSoildMatch;
	mygetSoildMatch.getWorkGroup(oriMatched_matchPt2dArray, oriMatched_matchPt3dArray, oriMatched_matchPt2dArray_StandardPicture, matchanchorNameArray, oriMatched_distance);
	//mygetSoildMatch.smallDisatanceCntSort_getSoildMatch(matchPt2dArray, matchPt3dArray);
	mygetSoildMatch.smallDisatanceCntSort_moreAnchor_getSoildMatch(matchPt2dArray, matchPt3dArray);
	

	//getSoildMatch(oriMatched_matchPt2dArray, oriMatched_matchPt3dArray, oriMatched_matchPt2dArray_StandardPicture, matchanchorNameArray, oriMatched_distance);
	//matchPt2dArray = oriMatched_matchPt2dArray;
	//matchPt3dArray = oriMatched_matchPt3dArray;
	//matchPt2dArray_StandardPicture = oriMatched_matchPt2dArray_StandardPicture;

}

bool CDatasetFlann::match(cv::Mat& queryDesc, std::vector<cv::DMatch>& matches, double ratio_threshold)
{

	int max_id = -1;

	int knn = 2;
	cv::Mat indices = cv::Mat(queryDesc.rows, knn, CV_32S);
	cv::Mat dists = cv::Mat_<float>(queryDesc.rows, knn);
	m_flannIndex.knnSearch(queryDesc, indices, dists, knn);
	//参数knn  searchParams的含义
	matches.clear();
	matches.reserve(indices.rows);

	for (int i = 0; i < indices.rows; i++) {
		int *pindex = (int*)indices.row(i).data;                   // *pindex中存储的是在库中的特征点的索引，i代表定位图像中的特征点索引
		float *pdis = (float*)dists.row(i).data;

		if ((pdis[0] / pdis[1]) <= ratio_threshold) {
			cv::DMatch dmatch;
			dmatch.distance = pdis[0];
			dmatch.queryIdx = i;
			dmatch.trainIdx = pindex[0];
			//dmatch.ratio_ = pdis[0] / double(pdis[1]);
			matches.push_back(dmatch);
		}
	}
	return true;
}

bool CDatasetFlann::match_corridor(cv::Mat& queryDesc, std::vector<cv::DMatch>& matches, double ratio_threshold)
{
	int max_id = -1;

	int knn = 2;
	cv::Mat indices = cv::Mat(queryDesc.rows, knn, CV_32S);
	cv::Mat dists = cv::Mat_<float>(queryDesc.rows, knn);

	{
		CRuntime tt("m_flannIndex.knnSearch");
		m_flannIndex.knnSearch(queryDesc, indices, dists, knn);
	}

	matches.clear();
	matches.reserve(indices.rows);

	for (int i = 0; i < indices.rows; i++) {
		int *pindex = (int*)indices.row(i).data;                   // *pindex中存储的是在库中的特征点的索引，i代表定位图像中的特征点索引
		float *pdis = (float*)dists.row(i).data;

		cv::Point3d diffPt3d = (m_pt3dArray[pindex[0]] - m_pt3dArray[pindex[1]]);
		double Diffdistance = sqrt(diffPt3d.x *diffPt3d.x + diffPt3d.y *diffPt3d.y + diffPt3d.z *diffPt3d.z);

		if ((pdis[0] / pdis[1]) <= ratio_threshold || Diffdistance<0.05) {
			cv::DMatch dmatch;
			dmatch.distance = pdis[0];
			dmatch.queryIdx = i;
			dmatch.trainIdx = pindex[0];
			matches.push_back(dmatch);
		}
	}
	return true;
}

bool CDatasetFlann::match_corridor(cv::Mat& queryDesc, std::vector<cv::DMatch>& matches, double ratio_threshold, std::vector<cv::Point2d>& pt2dArray,
	std::vector<cv::Point2d>& matchPt2dArray,std::vector<cv::Point3d>& matchPt3dArray, std::vector<cv::Point2d>& matchPt2dArray_StandardPicture, std::vector<std::string> &matchanchorNameArray)
{
	int max_id = -1;

	int knn = 2;
	cv::Mat indices = cv::Mat(queryDesc.rows, knn, CV_32S);
	cv::Mat dists = cv::Mat_<float>(queryDesc.rows, knn);
	m_flannIndex.knnSearch(queryDesc, indices, dists, knn);
	//参数knn  searchParams的含义
	matches.clear();
	matches.reserve(indices.rows);

	for (int i = 0; i < indices.rows; i++) {
		int *pindex = (int*)indices.row(i).data;                   // *pindex中存储的是在库中的特征点的索引，i代表定位图像中的特征点索引
		float *pdis = (float*)dists.row(i).data;

		cv::Point3d diffPt3d = (m_pt3dArray[pindex[0]] - m_pt3dArray[pindex[1]]);
		double Diffdistance = sqrt(diffPt3d.x *diffPt3d.x + diffPt3d.y *diffPt3d.y + diffPt3d.z *diffPt3d.z);

		if ((pdis[0] / pdis[1]) <= ratio_threshold || Diffdistance<0.05) {
			bool is_3d_element_in_vector = is_element_in_vector(matchPt3dArray, m_pt3dArray[matches[i].trainIdx]);
			bool is_2d_element_in_vector = is_element_in_vector(matchPt2dArray, pt2dArray[matches[i].queryIdx]);

			if (!is_3d_element_in_vector && !is_2d_element_in_vector) {
				matchPt2dArray.push_back(pt2dArray[i]);
				matchPt3dArray.push_back(m_pt3dArray[pindex[0]]);
				matchPt2dArray_StandardPicture.push_back(m_pt2dArray[pindex[0]]);
				matchanchorNameArray.push_back(m_anchorNameArray[pindex[0]]);
			}
		}
	}
	return true;
}

bool CDatasetFlann::Homography_cullError(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, std::vector<cv::Point2d>& matchPt2dArray_OnstandardPicutre) {
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
				matchPt2dArray_OnstandardPicutre[pass_HomographyNum] = matchPt2dArray_OnstandardPicutre[k];
				pass_HomographyNum++;
			}
		}

		matchPt2dArray.resize(pass_HomographyNum);
		matchPt3dArray.resize(pass_HomographyNum);
		matchPt2dArray_OnstandardPicutre.resize(pass_HomographyNum);
	}
	else {
		matchPt2dArray.resize(0);
		matchPt3dArray.resize(0);
		matchPt2dArray_OnstandardPicutre.resize(0);
	}

	return true;
}

bool CDatasetFlann::getSoildMatch(std::vector<cv::Point2d>& matchPt2dArray, std::vector<cv::Point3d>& matchPt3dArray, std::vector<cv::Point2d>& matchPt2dArray_StandardPicture, std::vector<std::string> &matchanchorNameArray, std::vector<float>& distanceArray)
{
//#define smallMean
#define smallCnt
	
	std::vector<std::string> maxStringArray;
	std::vector<int> maxStringCountArrary;
	maxNumString(matchanchorNameArray, maxStringArray, maxStringCountArrary);

	int maxPssNum = 0;
	int maxPassPicOrder = 0;

	int compareNum = 20;
	if (compareNum > maxStringArray.size()) compareNum = maxStringArray.size();

	int minMeanNum = 2;
	if (minMeanNum > maxStringArray.size()) minMeanNum = maxStringArray.size();

	std::vector<H_cull_AnchorworkGroup> workGroupArray;
	std::vector<int> cntVector;
	
	workGroupArray.resize(compareNum);
	cntVector.resize(compareNum);
	for (size_t i = 0; i < compareNum; i++)
	{
		workGroupArray[i].anchorName = maxStringArray[i];
		workGroupArray[i].matchPt2dArray_work.resize(maxStringCountArrary[i]);
		workGroupArray[i].matchPt3dArray_work.resize(maxStringCountArrary[i]);
		workGroupArray[i].matchPt2dArray_StandardPicture_work.resize(maxStringCountArrary[i]);
		workGroupArray[i].distanceArray.resize(maxStringCountArrary[i]);
	}

	// 循环matchanchorNameArray, 将各个锚点的数据放入workGroupArray中
	for (size_t i = 0; i < matchPt2dArray.size(); i++)
	{
		int isIn = isInNameArray(matchanchorNameArray[i], workGroupArray);
		if (isIn != -9999)
		{
			workGroupArray[isIn].matchPt2dArray_work[cntVector[isIn]] = matchPt2dArray[i];
			workGroupArray[isIn].matchPt3dArray_work[cntVector[isIn]] = matchPt3dArray[i];
			workGroupArray[isIn].matchPt2dArray_StandardPicture_work[cntVector[isIn]] = matchPt2dArray_StandardPicture[i];
			workGroupArray[isIn].distanceArray[cntVector[isIn]] = distanceArray[i];
			cntVector[isIn]++;
		}
	}



#ifdef smallMean
	// 对特征描述子的距离求平均
	std::vector<float> meanDistanceArray;
	std::vector<size_t> Sortindex_smallMean;
	DistanceMean(workGroupArray, meanDistanceArray, Sortindex_smallMean);
	// 筛去特征描述子极少，但平均距离排名靠前的锚点。
	for (size_t i = 0; i < Sortindex_smallMean.size(); )
	{
		if (workGroupArray[Sortindex_smallMean[i]].distanceArray.size() <= 10 && (findVerySmallDistanceMatch(workGroupArray[Sortindex_smallMean[i]].distanceArray)<2))
			vector<size_t>(Sortindex_smallMean.begin() + 1, Sortindex_smallMean.end()).swap(Sortindex_smallMean);
		else
			break;
	}

	for (size_t i = 0; i < compareNum; i++)
	{
		int nPosition = 9999;
		std::vector<size_t>::iterator iElement = find(Sortindex_smallMean.begin(), Sortindex_smallMean.end(), i);
		if (iElement != Sortindex_smallMean.end()) nPosition = distance(Sortindex_smallMean.begin(), iElement);

		if (workGroupArray[i].matchPt2dArray_work.size() >= 4 && nPosition < minMeanNum)
			Homography_cullError(workGroupArray[i].matchPt2dArray_work, workGroupArray[i].matchPt3dArray_work, workGroupArray[i].matchPt2dArray_StandardPicture_work);
		else
			continue;



		if (workGroupArray[i].matchPt2dArray_work.size() > maxPssNum) {
			maxPssNum = workGroupArray[i].matchPt2dArray_work.size();
			maxPassPicOrder = i;

			matchPt2dArray = workGroupArray[i].matchPt2dArray_work;
			matchPt3dArray = workGroupArray[i].matchPt3dArray_work;
		}
	}
#endif // smallMean

#ifdef smallCnt
	//对特征描述子的距离进行统计
	std::vector<int> smallEnoughDisatanceCountArray;
	std::vector<size_t> Sortindex_smallCnt;
	float smallEnoughDistace = 0.02;
	smallEnoughDisatanceCount(workGroupArray, smallEnoughDistace, smallEnoughDisatanceCountArray, Sortindex_smallCnt);

	for (size_t i = 0; i < compareNum; i++)
	{
		int nPosition = 9999;
		std::vector<size_t>::iterator iElement = find(Sortindex_smallCnt.begin(), Sortindex_smallCnt.end(), i);
		if (iElement != Sortindex_smallCnt.end()) nPosition = distance(Sortindex_smallCnt.begin(), iElement);

		if (workGroupArray[i].matchPt2dArray_work.size() >= 4 && nPosition < minMeanNum)
			Homography_cullError(workGroupArray[i].matchPt2dArray_work, workGroupArray[i].matchPt3dArray_work, workGroupArray[i].matchPt2dArray_StandardPicture_work);
		else
			continue;

		if (workGroupArray[i].matchPt2dArray_work.size() > maxPssNum) {
			maxPssNum = workGroupArray[i].matchPt2dArray_work.size();
			maxPassPicOrder = i;

			matchPt2dArray = workGroupArray[i].matchPt2dArray_work;
			matchPt3dArray = workGroupArray[i].matchPt3dArray_work;
			matchPt2dArray_StandardPicture = workGroupArray[i].matchPt2dArray_StandardPicture_work;
		}
	}
#endif // smallCnt


	
	std::cout << "此张照片属于" << maxStringArray[maxPassPicOrder] << "锚点" << std::endl;
	return true;
}



/*----------------------------------------------------------------------tools--------------------------------------------------------------------------------*/
int CDatasetFlann::getPlaceType() {
	return m_placeType;
}

bool CDatasetFlann::is_element_in_vector(vector<cv::Point3d> v, cv::Point3d element) {
	vector<cv::Point3d>::iterator it;
	it = find(v.begin(), v.end(), element);
	if (it != v.end()) {
		return true;
	}
	else {
		return false;
	}
}

bool CDatasetFlann::is_element_in_vector(vector<cv::Point2d> v, cv::Point2d element) {
	vector<cv::Point2d>::iterator it;
	it = find(v.begin(), v.end(), element);
	if (it != v.end()) {
		return true;
	}
	else {
		return false;
	}
}

bool CDatasetFlann::maxNumString(std::vector<std::string> testList, std::vector<std::string>& maxStringArray, std::vector<int>& maxStringCountArray) {

	std::map<std::string, int> keyList; //以元素值作为key,计数作为value
	for (std::vector<std::string>::iterator it = testList.begin(); it != testList.end(); it++)
	{
		keyList[*it]++;
	}

	std::vector<PAIR> vec(keyList.begin(), keyList.end());
	std::sort(vec.begin(), vec.end(), cmp_val);
	maxStringArray.resize(vec.size());    maxStringCountArray.resize(vec.size());
	int i = 0;
	for (vector<PAIR>::iterator ite = vec.begin(); ite != vec.end(); ++ite)
	{
		maxStringArray[i] = ite->first;
		maxStringCountArray[i] = ite->second;
		i++;
	}

	return true;
}

bool CDatasetFlann::cmp_val(const PAIR &left, const PAIR &right)
{
	return left.second > right.second;
}

int CDatasetFlann::isInNameArray(std::string in_anchorName, std::vector<H_cull_AnchorworkGroup> workGroupArray) {
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


template <typename T>
vector<size_t> sort_indexes_e(vector<T> &v)
{
	vector<size_t> idx(v.size());
	iota(idx.begin(), idx.end(), 0);
	sort(idx.begin(), idx.end(),
		[&v](size_t i1, size_t i2) {return v[i1] < v[i2]; });
	return idx;
};

template <typename T>
vector<size_t> sort_indexes_i(vector<T> &v)
{
	vector<size_t> idx(v.size());
	iota(idx.begin(), idx.end(), 0);
	sort(idx.begin(), idx.end(),
		[&v](size_t i1, size_t i2) {return v[i1] > v[i2]; });
	return idx;
};

void CDatasetFlann::DistanceMean(std::vector<H_cull_AnchorworkGroup>& workGroupArray, std::vector<float>& meanDistanceArray, std::vector<size_t>& Sortindex ) {

	float sum=0;
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

int CDatasetFlann::findVerySmallDistanceMatch(std::vector<float> distanceArray)
{
	int verySmallDistanceMatch = 0;
	for (size_t i = 0; i < distanceArray.size(); i++)
	{
		if (distanceArray[i] <= 0.01)
			verySmallDistanceMatch++;
	}
	return verySmallDistanceMatch;
}

void CDatasetFlann::smallEnoughDisatanceCount(std::vector<H_cull_AnchorworkGroup>& workGroupArray, float smallEnoughDistace, std::vector<int>& smallEnoughDisatanceCountArray, std::vector<size_t>& Sortindex) {

	smallEnoughDisatanceCountArray.resize(workGroupArray.size());
	int eachWorkGroupNum = 0;
	for (int i = 0; i < workGroupArray.size(); i++)
	{
		eachWorkGroupNum = workGroupArray[i].distanceArray.size();
		for (int j = 0; j < eachWorkGroupNum; j++)
		{
			if (workGroupArray[i].distanceArray[j] < smallEnoughDistace)
				smallEnoughDisatanceCountArray[i]++;
		}
	}

	Sortindex = sort_indexes_i(smallEnoughDisatanceCountArray);
	
}


