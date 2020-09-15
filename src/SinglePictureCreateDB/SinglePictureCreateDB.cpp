// SinglePictureCreateDB.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include<fstream>
#include<iostream>
#include "shootPictureProcessor.h"




bool runMain(string filePath, std::string anchorName) {

	CshootPictureProcessor myshootPictureProcessor;
	std::vector<ShootPictureInfo> myshootPictureInfoVec;
	OutputStruct outputStruct;
	outputStruct.anchorName = anchorName;

	// step-1 read configInfo, and surf all the shoot picture
	// IN: configFile  OUT: myshootPictureInfoVec[i].imgName,standardImagName,controalPoints, orign_descriptors,orign_imagePoints
	myshootPictureProcessor.readInfo(filePath, myshootPictureInfoVec);

	// step-2 match with the standardPicture, calculate the H, finally update the matched_descriptors, matched_imagePoints, objectPoints
	// IN: standardPicutre  OUT: matched_matched_descriptors, matched_imagePoints(On the StandardPicture), objectPoints
	myshootPictureProcessor.calculate_HomographyAndObejcPoints(filePath, myshootPictureInfoVec);


	// step-3 make the shootPicutre matching with each other , and get the ScoreVec
	// IN: each shootPicutre descriptor  OUT: each shootPicture ScoreVec
	myshootPictureProcessor.shootPictiresMatch_getScore(myshootPictureInfoVec);

	// step-4 depending on the shootPicture score,  select the good Descriptors;
	// IN: myshootPictureInfoVec   OUT:select outputPoint2d, outputPoint3d, outputDescriptor. 
	myshootPictureProcessor.selectGoodDescriptors(myshootPictureInfoVec, outputStruct);


	// step-4.1 add the standardPicture Descriptor and 3dPoints
	if(myshootPictureInfoVec[0].anchor_extra.anchor_type == "normal")
	{
		myshootPictureProcessor.readStandPictureDescriptors(filePath, myshootPictureInfoVec, outputStruct);
	}
	
	//step-5 createDB
	string dbFilePath = filePath + "\\..\\"+anchorName+".dat";
	myshootPictureProcessor.createDatabase(dbFilePath, outputStruct);


	return true;
}

int main(int argc, char* argv[])
{	
	runMain(argv[1], argv[2]);
    return 0;
}




