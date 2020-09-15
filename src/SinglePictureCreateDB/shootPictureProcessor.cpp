#include "stdafx.h"
#include "shootPictureProcessor.h"

#define pto_output
#define standardPic_Compressedsize 1920.0
#define shootPic_Compressedsize 1920.0

CshootPictureProcessor::CshootPictureProcessor()
{
}


CshootPictureProcessor::~CshootPictureProcessor()
{
}


bool CshootPictureProcessor::readInfo(std::string filePath, std::vector<ShootPictureInfo>& myshootPictrueInfoVec) {
	std::vector<string> filenameVec;
	std::vector<cv::Point3d> ControalPoints;
	Anchor_extra anchor_extra;
	

	readconfigfile(filePath, filenameVec, ControalPoints, anchor_extra);

	myshootPictrueInfoVec.resize(filenameVec.size() - 1);
	for (size_t i = 1; i < filenameVec.size(); i++) {
		myshootPictrueInfoVec[i - 1].standardImagName = filenameVec[0];
		myshootPictrueInfoVec[i - 1].imgName = filenameVec[i];
		myshootPictrueInfoVec[i - 1].controalPoints = ControalPoints;

		//extra
		myshootPictrueInfoVec[i - 1].anchor_extra = anchor_extra;
	}

	for (size_t i = 0; i < myshootPictrueInfoVec.size(); i++) {
		readshootPictrue(filePath, myshootPictrueInfoVec[i].imgName, myshootPictrueInfoVec[i]);
	}

	std::cout << "以成功读取" << myshootPictrueInfoVec.size() << "张拍摄图，并提取特征" << std::endl;
	return true;
}

bool CshootPictureProcessor::readconfigfile(string configfilePath, std::vector<string> &filenameVec, std::vector<cv::Point3d> &ControalPoints, Anchor_extra& anchor_extra) {

	ifstream infile;
	string filefullPath = configfilePath + "\\createdbConfig.txt";
	infile.open(filefullPath.data());
	assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行 

	string s;
	std::vector<string> allin;
	int lineNum = 0;
	int imgNum = 0;
	while (getline(infile, s)) {
		if (s.empty()) continue;

		if (lineNum == 0) {
			imgNum = stoi(s);
			std::cout << "一共有" << imgNum << "张照片" << std::endl;
			lineNum++;
			continue;
		}
		if (lineNum == 1) {
			anchor_extra.anchor_type = s;
			std::cout << "此锚点类型为" << anchor_extra.anchor_type << std::endl;
			lineNum++;
			continue;
		}
		if (lineNum == 2) {
			vector<string> v = split(s, ',');
			anchor_extra.homography_sideToStandard = (cv::Mat_<double>(3, 3) << stod(v[0]) , stod(v[1]) ,stod(v[2]),
		          stod(v[3]) , stod(v[4]) ,stod(v[5]),
				  stod(v[6]) , stod(v[7]) , stod(v[8]));
			anchor_extra.normal_standardPicure_Width = stod(v[9]);
			anchor_extra.normal_standardPicure_Height= stod(v[10]);
			lineNum++;
			continue;
		}

		allin.push_back(s);
		lineNum++;
	}
	infile.close();             //关闭文件输入流 

	// 写入照片名称
	for (size_t i = 0; i < imgNum; i++) {
		filenameVec.push_back(allin[i]);
	}

	// 写入坐标信息
	for (size_t i = imgNum; i < allin.size(); i++) {

		std::vector<string> splitlineVec;
		SplitString(allin[i], splitlineVec, ",");

		if (splitlineVec.size() == 3) {
			cv::Point3d tmp_Point3d;
			tmp_Point3d.x = stod(splitlineVec[0]);
			tmp_Point3d.y = stod(splitlineVec[1]);
			tmp_Point3d.z = stod(splitlineVec[2]);
			ControalPoints.push_back(tmp_Point3d);
		}
	}
	return true;
}

bool CshootPictureProcessor::readshootPictrue(std::string imgPath, std::string imgName, ShootPictureInfo& myshootPictrueInfo) {

	cv::Mat img = cv::imread(imgPath + "/" + imgName);

	double max_wh = std::max(img.rows, img.cols);
	if (max_wh>shootPic_Compressedsize)
	{
		double t = std::max(max_wh / shootPic_Compressedsize, 1.0);;
		cv::resize(img, img, cv::Size(img.cols / t, img.rows / t));
	}

	string folderPath = imgPath + "\\" + "resizeIMG";
	mkdir(folderPath.c_str());
	cv::imwrite(folderPath + "\\" + "resize_" + imgName, img);


	//surf
	cv::Ptr<cv::SURF> surf = new cv::SURF;
	std::vector<cv::KeyPoint> keypoints;
	surf->detect(img, keypoints);
	surf->compute(img, keypoints, myshootPictrueInfo.descriptors);

	for (int i = 0; i < keypoints.size(); i++) {
		myshootPictrueInfo.imagePoints.push_back(keypoints[i].pt);
	}

	 
	return true;
}

bool CshootPictureProcessor::calculate_HomographyAndObejcPoints(std::string imgPath, std::vector<ShootPictureInfo> &shootPictureArray) {

	std::string standardPicutreName = shootPictureArray[0].standardImagName;
	std::vector<cv::Point3d> ControlPoint = shootPictureArray[0].controalPoints;

// step1 -- read standard Picture , and surf it.
	cv::Mat img = cv::imread(imgPath + "\\" + standardPicutreName);


	double max_wh = std::max(img.rows, img.cols);
	if (max_wh>standardPic_Compressedsize)
	{
		double t = std::max(max_wh / standardPic_Compressedsize, 1.0);;
		cv::resize(img, img, cv::Size(img.cols / t, img.rows / t));
	}

	string folderPath = imgPath + "\\" + "resizeIMG";
	mkdir(folderPath.c_str());
	cv::imwrite(folderPath + "\\" + "resize_" + "standard.jpg", img);

	cv::Ptr<cv::SURF> surf = new cv::SURF;
	std::vector<cv::KeyPoint> StandPictutrekeypoints;
	cv::Mat standardPicutredescriptors;
	surf->detect(img, StandPictutrekeypoints);
	surf->compute(img, StandPictutrekeypoints, standardPicutredescriptors);


// step2 -- match the standardPicture with each shootPicutre in shootPicutureArray, calculate the H and cull the error points
	for (size_t i = 0; i < shootPictureArray.size(); i++) {

		ShootPictureInfo *currentShootPicture = &shootPictureArray[i];

		//match
		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
		std::vector< std::vector<cv::DMatch> > knn_matches;
		matcher->knnMatch(currentShootPicture->descriptors, standardPicutredescriptors, knn_matches, 2);


		const float ratio_thresh = 0.7f;
		std::vector<cv::DMatch> good_matches;
		for (size_t k = 0; k < knn_matches.size(); k++)
		{
			if (knn_matches[k][0].distance < ratio_thresh * knn_matches[k][1].distance)
			{
				good_matches.push_back(knn_matches[k][0]);
			}
		}

		std::vector<cv::Point2d> matched_shootPicturePoints;
		std::vector<cv::Point2d> matched_standardPicturePoints;
		std::vector<cv::Point2d> matched_Hconverstion_shootPicturePoints;
		cv::Mat matched_descriptors = cv::Mat::zeros(good_matches.size(), currentShootPicture->descriptors.cols, CV_32F);


		int descNum = 0;
		for (size_t k = 0; k < good_matches.size(); k++)
		{
			// step1 -- 查找有无重复点
			bool is_2d_shootPicture_element_in_vector = is_element_in_vector(matched_shootPicturePoints, currentShootPicture->imagePoints[good_matches[k].queryIdx]);
			bool is_2d_standardPicutre_element_in_vector = is_element_in_vector(matched_standardPicturePoints, StandPictutrekeypoints[good_matches[k].trainIdx].pt);

			if (!is_2d_shootPicture_element_in_vector && !is_2d_standardPicutre_element_in_vector)
			{
				//-- Get the keypoints from the good matches
				currentShootPicture->descriptors.row(good_matches[k].queryIdx).copyTo(matched_descriptors.row(descNum));
				matched_shootPicturePoints.push_back(currentShootPicture->imagePoints[good_matches[k].queryIdx]);
				matched_standardPicturePoints.push_back(StandPictutrekeypoints[good_matches[k].trainIdx].pt);
				descNum++;
			}

		}

		//cal Homography
		if (matched_shootPicturePoints.size() < 4)
		{
			return false;
		}

		cv::Mat H = findHomography(matched_shootPicturePoints, matched_standardPicturePoints, CV_RANSAC);

		int pass_HomographyNum = matched_shootPicturePoints.size();
		//use the Homography to cull error points
		std::vector<double> residualVec;
		for (size_t k = 0; k < matched_shootPicturePoints.size(); k++) {

			cv::Mat srcPoints = (cv::Mat_<double>(3, 1) << matched_shootPicturePoints[k].x, matched_shootPicturePoints[k].y, 1);
			cv::Mat calDstPoints = H * srcPoints;
			cv::Point2d H_convertion = { calDstPoints.at<double>(0, 0) / calDstPoints.at<double>(2, 0) ,calDstPoints.at<double>(1, 0) / calDstPoints.at<double>(2, 0) };
			matched_Hconverstion_shootPicturePoints.push_back(H_convertion);

			double residual = sqrt(pow(H_convertion.x - matched_standardPicturePoints[k].x, 2) + pow(H_convertion.y - matched_standardPicturePoints[k].y, 2));
			residualVec.push_back(residual);

			if (residual > 1) {
				matched_shootPicturePoints[k].x = 0; matched_shootPicturePoints[k].y = 0;
				pass_HomographyNum--;
			}

		}
		residualVec.clear();


//  step3 -- cal the objectPoints in shootPicture
		cv::Mat output_descriptors = cv::Mat::zeros(pass_HomographyNum, currentShootPicture->descriptors.cols, CV_32F);
		std::vector<cv::Point2d> output_imagePoints;

#ifdef pto_output
		std::vector<cv::Point2d> imagePoints_inShootPicture;  //ptoUse
		std::vector<cv::Point2d> imagePoints_inStandardPicuture;  //ptoUse
#endif // pto_output

		int cnt = 0;
		cv::Point3d X1 = ControlPoint[0];    cv::Point3d X2 = ControlPoint[1];     cv::Point3d X3 = ControlPoint[2];    cv::Point3d X4 = ControlPoint[3];

		// side_standard TO normal_standard 将斜视标准片的像方点，利用H矩阵，转换到正射标准片上
		for (size_t k = 0; k < matched_Hconverstion_shootPicturePoints.size(); k++)
		{
			cv::Mat srcPoints_sideTOnormal = (cv::Mat_<double>(3, 1) << matched_Hconverstion_shootPicturePoints[k].x, matched_Hconverstion_shootPicturePoints[k].y, 1);
			cv::Mat DstPoints_sideTOnormal = shootPictureArray[0].anchor_extra.homography_sideToStandard * srcPoints_sideTOnormal;
			cv::Point2d H_convertion = { DstPoints_sideTOnormal.at<double>(0, 0) / DstPoints_sideTOnormal.at<double>(2, 0) ,DstPoints_sideTOnormal.at<double>(1, 0) / DstPoints_sideTOnormal.at<double>(2, 0) };
			matched_Hconverstion_shootPicturePoints[k].x = H_convertion.x;
			matched_Hconverstion_shootPicturePoints[k].y = H_convertion.y;
		}
		// end

		// 读取标准片的长和宽：若为normal标准片，则直接读取img的宽高；若为side，则要读取txt文本中，转换后的标准片的宽高；直接读取img宽高不为normal标准片的宽高
		int width;       int height;
		if (shootPictureArray[0].anchor_extra.anchor_type == "normal")
		{
			width = img.cols;    height = img.rows;
		}
		if (shootPictureArray[0].anchor_extra.anchor_type == "side")
		{
			width = shootPictureArray[0].anchor_extra.normal_standardPicure_Width;
			height = shootPictureArray[0].anchor_extra.normal_standardPicure_Height;
		}

		for (size_t k = 0; k < matched_shootPicturePoints.size(); k++) {
			if (matched_shootPicturePoints[k].x == 0)  continue;

			cv::Point3d objPoints;
			objPoints.x = X1.x + (X4 - X1).x / width * matched_Hconverstion_shootPicturePoints[k].x + (X2 - X1).x / height * matched_Hconverstion_shootPicturePoints[k].y;
			objPoints.y = X1.y + (X4 - X1).y / width * matched_Hconverstion_shootPicturePoints[k].x + (X2 - X1).y / height * matched_Hconverstion_shootPicturePoints[k].y;
			objPoints.z = X1.z + (X4 - X1).z / width * matched_Hconverstion_shootPicturePoints[k].x + (X2 - X1).z / height * matched_Hconverstion_shootPicturePoints[k].y;

			currentShootPicture->objectPoints.push_back(objPoints);
			output_imagePoints.push_back(matched_Hconverstion_shootPicturePoints[k]);
			matched_descriptors.row(k).copyTo(output_descriptors.row(cnt));

#ifdef pto_output
			imagePoints_inShootPicture.push_back(matched_shootPicturePoints[k]);   //matched_H_culled_Point2d_onShootPicture        ptoUse
			imagePoints_inStandardPicuture.push_back(sideToNormal(matched_standardPicturePoints[k], shootPictureArray[0].anchor_extra.homography_sideToStandard)); //matched_H_culled_Point2d_onStandardPicture  ptoUse
#endif // pto_output

			cnt++;
		}

		currentShootPicture->descriptors = output_descriptors;
		currentShootPicture->imagePoints = output_imagePoints;

#ifdef pto_output

		string folderPath = imgPath + "\\" + "shoot_standard";
		mkdir(folderPath.c_str());

		string string_ptoFileName = folderPath + "\\" + shootPictureArray[i].imgName + "_" + shootPictureArray[i].standardImagName + ".pto";
		const char* ptoFileName = string_ptoFileName.c_str();
		std::string imgFileName1 = shootPictureArray[i].imgName;
		std::string imgFileName2 = shootPictureArray[i].standardImagName;

		std::vector<POINT_PAIR> ptPairArray;
		for (size_t k = 0; k < imagePoints_inShootPicture.size(); k++) {
			POINT_PAIR myPoint_parir;
			myPoint_parir.x1 = imagePoints_inShootPicture[k].x;     myPoint_parir.y1 = imagePoints_inShootPicture[k].y;
			myPoint_parir.x2 = imagePoints_inStandardPicuture[k].x;  myPoint_parir.y2 = imagePoints_inStandardPicuture[k].y;
			ptPairArray.push_back(myPoint_parir);
		}
		pointsLogRW::writePTO(ptoFileName, ptPairArray, imgFileName1, imgFileName2);
#endif // pto_output


	}

	std::cout << "每张拍摄图与标准图匹配完毕，并获得物方点" << std::endl;
	return true;
}

bool CshootPictureProcessor::shootPictiresMatch_getScore(std::vector<ShootPictureInfo> &shootPictureArray) {

	int shootPicutreNum = shootPictureArray.size();

	// 遍历循环所有拍摄片，和其他片进行比较
	for (int i = 0; i < shootPicutreNum; i++) {
		ShootPictureInfo *currentShootPicture = &shootPictureArray[i];
		if (currentShootPicture->descriptors.rows <= 2) continue;

		int descriptorNum = currentShootPicture->descriptors.rows;
		currentShootPicture->descriptorsScore.resize(descriptorNum);

		for (int j = 0; j < shootPicutreNum; j++) {
			if (i == j) continue;
			ShootPictureInfo comparedShootPicture = shootPictureArray[j];
			if (comparedShootPicture.descriptors.rows <= 2) continue;

			//-- Matching descriptor vectors with a FLANN based matcher
			cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
			std::vector< std::vector<cv::DMatch> > knn_matches;
			
			matcher->knnMatch(currentShootPicture->descriptors, comparedShootPicture.descriptors, knn_matches, 2);

			//-- Filter matches using the Lowe's ratio test
			const float ratio_thresh = 0.7f;
			std::vector<cv::DMatch> good_matches;
			for (size_t k = 0; k < knn_matches.size(); k++)
			{
				if (knn_matches[k][0].distance < ratio_thresh * knn_matches[k][1].distance)
				{
					good_matches.push_back(knn_matches[k][0]);
				}
			}

			//-- use the  good_matches to calculate currentShootPicutre scores;
			for (size_t k = 0; k < good_matches.size(); k++)
			{
				int goodIndex = good_matches[k].queryIdx;
				currentShootPicture->descriptorsScore[goodIndex]++;
			}
		}
	}

	std::cout << "成功获得每张照片的特征得分表" << std::endl;
	return true;
}

bool CshootPictureProcessor::selectGoodDescriptors(std::vector<ShootPictureInfo> &shootPictureArray, OutputStruct& outputstruct) {

	
	int descriptorCols = shootPictureArray[0].descriptors.cols;   //特征描述子的维度
	int outputcnt = 0;  //用来记录总输出的特征描述子个数。
	std::vector<cv::Mat> tmp_outputDescriptorVec;


	// step1 -- 从所有拍摄片中选择得分最高的，作为建库片
	int pictureSize = shootPictureArray.size();
	std::vector<int> sumScoreArray;

	for (size_t i = 0; i < pictureSize; i++) {

		ShootPictureInfo currentShootPicutureInfo = shootPictureArray[i];
		int single_pic_score_sum = SumVector(currentShootPicutureInfo.descriptorsScore);
		sumScoreArray.push_back(single_pic_score_sum);
	}

		
	std::vector<size_t> sortArray = sort_indexes_e(sumScoreArray);  // 排序, 得分最高像片序号最前
	reverse(begin(sortArray), end(sortArray));


	// step2 -- 选择前10张照片，输出特征; 若不足10张，则全部保存；
	if (pictureSize > 10) pictureSize = 10;


	// step3 --  根据分数选出n张影像后，正向逐张进行输出信息
	for (size_t i = 0; i < pictureSize; i++) {
		cv::Mat currentDescriptor = shootPictureArray[sortArray[i]].descriptors;
		std::vector<int> currentDescriptorScore = shootPictureArray[sortArray[i]].descriptorsScore;
		std::vector<cv::Point2d> currentPoint2d = shootPictureArray[sortArray[i]].imagePoints;
		std::vector<cv::Point3d> currentPoint3d = shootPictureArray[sortArray[i]].objectPoints;

		if (currentDescriptorScore.size() <=2) continue;
		//对 Score进行排序，选择前500个进行输出
		std::vector<size_t> Single_Picutre_Score_sortArray = sort_indexes_e(currentDescriptorScore);
		reverse(begin(Single_Picutre_Score_sortArray), end(Single_Picutre_Score_sortArray));

		int outputDescriptorNum = 500;
		if (currentDescriptor.rows < 500)   outputDescriptorNum = currentDescriptor.rows;

		cv::Mat tmp_outputDescriptor =cv::Mat::zeros(outputDescriptorNum, descriptorCols, CV_32F); 
		for (size_t j = 0; j < outputDescriptorNum; j++) {
			currentDescriptor.row(Single_Picutre_Score_sortArray[j]).copyTo(tmp_outputDescriptor.row(j));
			outputstruct.outputPoint2d.push_back(currentPoint2d[Single_Picutre_Score_sortArray[j]]);
			outputstruct.outputPoint3d.push_back(currentPoint3d[Single_Picutre_Score_sortArray[j]]);
			outputcnt++;
		}

		tmp_outputDescriptorVec.push_back(tmp_outputDescriptor);
	}
	// step3 -- end


	// step4 --  根据分数选出n张影像后，反向逐张进行输出信息
	//int reverseOutPicNum = 3;
	//if (reverseOutPicNum > shootPictureArray.size())  reverseOutPicNum = shootPictureArray.size();
	//reverse(begin(sortArray), end(sortArray));

	//for (size_t i = 0; i < reverseOutPicNum; i++) {
	//	cv::Mat currentDescriptor = shootPictureArray[sortArray[i]].descriptors;
	//	std::vector<cv::Point2d> currentPoint2d = shootPictureArray[sortArray[i]].imagePoints;
	//	std::vector<cv::Point3d> currentPoint3d = shootPictureArray[sortArray[i]].objectPoints;

	//	cv::Mat tmp_outputDescriptor = cv::Mat::zeros(currentPoint2d.size(), descriptorCols, CV_32F);

	//	for (size_t j = 0; j < currentPoint2d.size(); j++) {
	//		outputstruct.outputPoint2d.push_back(currentPoint2d[j]);
	//		outputstruct.outputPoint3d.push_back(currentPoint3d[j]);
	//		outputcnt++;
	//	}
	//	tmp_outputDescriptorVec.push_back(currentDescriptor);
	//}
	// step4 -- end


	for (size_t i = 0; i < tmp_outputDescriptorVec.size(); i++) {
		outputstruct.outputDescriptor.push_back(tmp_outputDescriptorVec[i]);
	}

	

	return true;

}

bool CshootPictureProcessor::readStandPictureDescriptors(std::string imgPath, std::vector<ShootPictureInfo> &shootPictureArray, OutputStruct &ouputstruct) {

	//step1 -- read info from shootPictureArray
	std::string standardPictureName = shootPictureArray[0].standardImagName;
	std::vector<cv::Point3d> controlPoint3d = shootPictureArray[0].controalPoints;


	//step2 -- read IMG and surf it
	cv::Mat img = cv::imread(imgPath + "\\" + standardPictureName);

	double max_wh = std::max(img.rows, img.cols);
	if (max_wh>standardPic_Compressedsize)
	{
		double t = std::max(max_wh / standardPic_Compressedsize, 1.0);;
		cv::resize(img, img, cv::Size(img.cols / t, img.rows / t));
	}

	cv::Ptr<cv::SURF> surf = new cv::SURF;
	std::vector<cv::KeyPoint> StandPictutrekeypoints;
	cv::Mat standardPicutredescriptors;
	surf->hessianThreshold = 5000;
	surf->detect(img, StandPictutrekeypoints);
	surf->compute(img, StandPictutrekeypoints, standardPicutredescriptors);


	//step3 -- get the keypoints' 3dPoints;
	int width;       int height;
	if(shootPictureArray[0].anchor_extra.anchor_type =="normal")
	{
		 width = img.cols;    height = img.rows;
	}
	if (shootPictureArray[0].anchor_extra.anchor_type == "side")
	{
		width = shootPictureArray[0].anchor_extra.normal_standardPicure_Width;
		height = shootPictureArray[0].anchor_extra.normal_standardPicure_Height;
	}
	cv::Point3d X1 = controlPoint3d[0];    cv::Point3d X2 = controlPoint3d[1];     cv::Point3d X3 = controlPoint3d[2];    cv::Point3d X4 = controlPoint3d[3];

	for (size_t k = 0; k < StandPictutrekeypoints.size(); k++) {

		cv::Point3d objPoints;
		objPoints.x = X1.x + (X4 - X1).x / width * StandPictutrekeypoints[k].pt.x + (X2 - X1).x / height * StandPictutrekeypoints[k].pt.y;
		objPoints.y = X1.y + (X4 - X1).y / width * StandPictutrekeypoints[k].pt.x + (X2 - X1).y / height * StandPictutrekeypoints[k].pt.y;
		objPoints.z = X1.z + (X4 - X1).z / width * StandPictutrekeypoints[k].pt.x + (X2 - X1).z / height * StandPictutrekeypoints[k].pt.y;

		ouputstruct.outputPoint3d.push_back(objPoints);
		ouputstruct.outputPoint2d.push_back(StandPictutrekeypoints[k].pt);
		ouputstruct.outputDescriptor.push_back(standardPicutredescriptors.row(k));

	}

	return true;

}

bool CshootPictureProcessor::createDatabase(string outfileName, OutputStruct& outputstruct) {

	std::ofstream outfile(outfileName, ios::app | ios::binary);

	std::string anchorName = outputstruct.anchorName;
	std::vector<cv::Point3d> vecPoint3d = outputstruct.outputPoint3d;
	std::vector<cv::Point2d> vecPoint2d = outputstruct.outputPoint2d;
	cv::Mat matchedDesc = outputstruct.outputDescriptor;

	int outputNum = outputstruct.outputPoint2d.size();
	

	for (size_t i = 0; i < vecPoint3d.size(); i++)
	{

		outfile.write(reinterpret_cast<char*>(&anchorName), 100);
		outfile.write(reinterpret_cast<char*>(&vecPoint2d[i]), sizeof(double) * 2);
		outfile.write(reinterpret_cast<char*>(&vecPoint3d[i]), sizeof(double) * 3);
		outfile.write(reinterpret_cast<char*>(matchedDesc.ptr<uchar>(i)), sizeof(float) * matchedDesc.cols);
	}

	std::cout << "一共输出保存" << outputNum << "个特征点" << std::endl;

	return true;
}




void CshootPictureProcessor::SplitString(const string& s, vector<string>& v, const string& c)
{
	string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while (string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2 - pos1));

		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length())
		v.push_back(s.substr(pos1));
}

bool CshootPictureProcessor::is_element_in_vector(vector<cv::Point2d> v, cv::Point2d element) {
	vector<cv::Point2d>::iterator it;
	it = find(v.begin(), v.end(), element);
	if (it != v.end()) {
		return true;
	}
	else {
		return false;
	}
}

vector<string> CshootPictureProcessor::split(string s, char delim) {
	vector<string> v;
	stringstream stringstream1(s);
	string tmp;
	while (getline(stringstream1, tmp, delim)) {
		v.push_back(tmp);
	}
	return v;
}


cv::Point2d CshootPictureProcessor::sideToNormal(cv::Point2d pt_onSideStandard, cv::Mat H) {
	cv::Mat srcPoints_sideTOnormal = (cv::Mat_<double>(3, 1) << pt_onSideStandard.x, pt_onSideStandard.y, 1);
	cv::Mat DstPoints_sideTOnormal = H * srcPoints_sideTOnormal;
	cv::Point2d H_convertion = { DstPoints_sideTOnormal.at<double>(0, 0) / DstPoints_sideTOnormal.at<double>(2, 0) ,DstPoints_sideTOnormal.at<double>(1, 0) / DstPoints_sideTOnormal.at<double>(2, 0) };

	cv::Point2d ptOnNormal;
	ptOnNormal.x = H_convertion.x;
	ptOnNormal.y = H_convertion.y;
	
	return ptOnNormal;
}
