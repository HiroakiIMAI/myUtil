#include "stdafx.h"

#include "cvUtil.hpp"

void imaiUtil::multiScaleTM(
	cv::Mat& src,
	cv::Mat& tmpl,
	cv::Mat& result,
	cv::Point& mPos, 
	cv::Size2i& sTmpl )
{
	// Binaraize
	cv::Mat src_8UC1;
	src.convertTo( src_8UC1, CV_8UC1, -255.0f / 4500.0f, 255.0f );
	//std::cout<< "type: "<< img_depth.type()<< ","<< src_8UC1.type()<< std::endl; 
	cv::threshold(src_8UC1, src_8UC1, 205	, 255, CV_THRESH_TOZERO    );
	cv::threshold(src_8UC1, src_8UC1, 254	, 255, CV_THRESH_TOZERO_INV);
	
	
	cv::imshow("ppp",src_8UC1);
	
	// Load template patch
	cv::Mat tmpl_8UC1;
	cvtColor(tmpl, tmpl_8UC1, CV_RGB2GRAY);

	// Multi Scale Template Matching
	cv::Mat img_templates[10];
	cv::Mat matchedResult[10];
	double templateMatch_maxVal = 0;
	int    templateMatch_matchedScale = 0;
	cv::Point2i matchedPos;
	for(int i=0 ; i<10 ; i++)
	{
		// resize template scale
		cv::resize(tmpl_8UC1, img_templates[i], cv::Size(), 0.4 + 0.1*i, 0.4 + 0.1*i);
	
		// Template Matching
		cv::matchTemplate( src_8UC1, img_templates[i], matchedResult[i], CV_TM_CCORR_NORMED );

		
		// result criteria
		double tmp_maxVal = 0;
		cv::Point2i tmp_matchedPos;
		cv::minMaxLoc(matchedResult[i], NULL, &tmp_maxVal, NULL, &tmp_matchedPos);
		if(tmp_maxVal > templateMatch_maxVal)
		{
			templateMatch_maxVal = tmp_maxVal;
			matchedPos = tmp_matchedPos;
			templateMatch_matchedScale = i;
		}
		std::cout<<"maxVal_"<< i << ": "<< tmp_maxVal<< std::endl;
	}
	
	std::cout<<"matchedScale: "<< templateMatch_matchedScale<< std::endl;

	//char caWindowName[128];
	//sprintf_s(caWindowName, "matchedResult_%d", i);
	
	/*
	if(1)
	{
		cv::Mat roi(src_8UC1,
			cv::Rect(
				toolPos.x, 
				toolPos.y,
				img_templates[templateMatch_matchedScale].cols, 
				img_templates[templateMatch_matchedScale].rows
			)
		);
		//roi += 128;
		//roi = cv::Scalar(255);
	}
	*/

	mPos.x = matchedPos.x;
	mPos.y = matchedPos.y;
	sTmpl.width  = img_templates[templateMatch_matchedScale].cols;
	sTmpl.height = img_templates[templateMatch_matchedScale].rows;
	//cv::imshow("CV_window",src_8UC1);
	//src_8UC1.copyTo(result);
	matchedResult[templateMatch_matchedScale].copyTo(result);

}