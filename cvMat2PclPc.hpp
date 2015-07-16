#define __CVMAT_TO_PCLPC_HPP__
#ifdef  __CVMAT_TO_PCLPC_HPP__

//-- Kinect --//
//#include <Kinect.h>

//-- PCL --//
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/cloud_viewer.h>  

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

	HRESULT makePointCloudXYZRGB_fromPc3DMat( 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_dst,
		cv::Mat &pc3DMat,
		cv::Mat &colorIndicsMapped2Depth,
		cv::Mat &colorMat,
		cv::Mat &depthMat
		);
	
	HRESULT makePointCloudXYZRGB_fromPc3DMatMasked( 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_dst,
		cv::Mat &pc3DMat,
		cv::Mat &colorIndicsMapped2Depth,
		cv::Mat &colorMat,
		cv::Mat &depthMat,
		cv::Mat &maskMat,
		unsigned int th_far  = 0,
		unsigned int th_near = 0
		);

}

#endif