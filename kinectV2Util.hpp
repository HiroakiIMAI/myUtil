#define __KINECT_TO_CVMAT_HPP__
#ifdef  __KINECT_TO_CVMAT_HPP__

//-- Kinect --//
#include <Kinect.h>

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
	template<class Interface>
	inline void SafeRelease( Interface *& pInterfaceToRelease )
	{
		if( pInterfaceToRelease != NULL ){
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}

	class KinectV2Class
	{
	protected:
		//------ Interfaces ------//
		// Sensor
		IKinectSensor* pSensor;
		// Color
		IColorFrameSource* pColorSource;
		IColorFrameReader* pColorReader;
		IFrameDescription* pColorFrameDescription;
		IColorFrame* pColorFrame;
		// Depth
		IDepthFrameSource* pDepthSource;
		IDepthFrameReader* pDepthReader;
		IFrameDescription* pDepthFrameDescription;
		IDepthFrame* pDepthFrame;
		// IR
		IInfraredFrameSource* pInfraredSource;
		IInfraredFrameReader* pInfraredReader;
		IFrameDescription*    pInfraredFrameDescription;
		IInfraredFrame* pInfraredFrame;
		// CoordinateMapper
		ICoordinateMapper* pCoordMapper;


		cv::Mat int_color;
		cv::Mat int_color_half;
		cv::Mat int_depth;
		cv::Mat int_depth_8bit;
		cv::Mat int_IR;

	public:
		cv::Mat color;
		cv::Mat color_half;
		int width_c;
		int height_c;
		unsigned int bufferSize_c;

		cv::Mat depth;
		cv::Mat depth_8bit;
		int width_d;
		int height_d;
		unsigned int bufferSize_d; 

		cv::Mat IR;
		int width_ir;
		int height_ir;
		unsigned int bufferSize_ir; 



		KinectV2Class();
		~KinectV2Class();
		void getColor();
		void getDepth();
		void getIR();

	};

};

#endif