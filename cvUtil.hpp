
#ifndef __CV_UTIL_HPP__
#define __CV_UTIL_HPP__


//-- OpenCV --//
#include <opencv2/opencv.hpp>
// バージョン取得
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)
// ビルドモード
#ifdef _DEBUG
#define CV_EXT_STR "d.lib"
#else
#define CV_EXT_STR ".lib"
#endif
// ライブラリのリンク（不要な物はコメントアウト）
#pragma comment(lib, "opencv_core"			CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib, "opencv_highgui"		CV_VERSION_STR CV_EXT_STR)

namespace imaiUtil{

	void multiScaleTM( 
		cv::Mat& src,
		cv::Mat& tmpl,
		cv::Mat& result,
		cv::Point& mPos,
		cv::Size2i& sTmpl);

	void drawCoordinate_fromXml( std::string fileName_int, std::string fileName_ext, cv::Mat img_out );

}



#endif