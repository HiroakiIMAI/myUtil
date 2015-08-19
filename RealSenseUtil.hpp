#ifndef __REAL_SENSE_UTIL_HPP__
#define __REAL_SENSE_UTIL_HPP__

#include <libuvc/libuvc.h>

#include <stdio.h>
#include <iostream>
#include <unistd.h>

#include <opencv2/opencv.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/calib3d/calib3d.hpp>

#include "libuvc_internal.h"

// device seralNumber
#define REALSENSE01 "044140031001"
#define REALSENSE02 "047140013306"
#define REALSENSE03 "043140088609"
#define REALSENSE04 "039140071403" /*(broken?)*/

#define HAYASHI	"043140028008"


namespace imaiUtil{

	typedef struct
	{
		unsigned char b;
		unsigned char g;
		unsigned char r;
	}st_bgr;

	class RealSenseClass
	{
	protected:
	  uvc_context_t *ctx;
	  uvc_device_t  *dev;
	  uvc_device_handle_t *devh_d;
	  uvc_device_handle_t *devh_rgb;
	  uvc_stream_ctrl_t ctrl_rgb;
	  uvc_stream_ctrl_t ctrl_d;

	  cv::Mat color_buff;
	  cv::Mat depth_buff;

	  enum uvc_frame_format depthFormat;

	  bool calibrated;

	  struct{
		  cv::Mat camMtx;
		  cv::Mat distCoeff;
		  cv::Mat undistMap1;
		  cv::Mat undistMap2;
	  }intrinsic_color;

	  struct{
		  cv::Mat camMtx;
		  cv::Mat distCoeff;
		  cv::Mat undistMap1;
		  cv::Mat undistMap2;
	  }intrinsic_depth;

	  cv::Mat R_mtx;
	  cv::Mat R_vec;
	  cv::Mat T_vec;


	public:
	  RealSenseClass(
			  enum uvc_frame_format colorFrameFormat,
			  cv::Size colorSize,
			  enum uvc_frame_format depthFrameFormat,
			  cv::Size depthSize,
			  const char* deviceSerialNumber = NULL,
			  bool verbose = false);

	  int openRealSense(
			  enum uvc_frame_format colorFrameFormat,
			  cv::Size colorSize,
			  enum uvc_frame_format depthFrameFormat,
			  cv::Size depthSize,
			  const char* deviceSerialNumber = NULL,
			  bool verbose = false);

	  int openDevice(
			  const char* deviceSerialNumber = NULL,
			  bool verbose = false);

	  int openColor(
			  enum uvc_frame_format colorFrameFormat,
			  cv::Size imageSize,
			  bool verbose = false);

	  int openDepth(
			  enum uvc_frame_format depthFrameFormat,
			  cv::Size imageSize,
			  bool verbose = false);

	  ~RealSenseClass();


	  unsigned char* getColorBufferPointer();
	  unsigned char* getDepthBufferPointer();

	  cv::Mat* getColorImage();
	  cv::Mat* getDepthImage();
	  cv::Mat* getPc3DMat();
	  cv::Mat* getColorIndicesMapped2Depth();
	  bool isCalibrated();

	  int loadCalibData();

	  pthread_mutex_t mtx_c;
	  pthread_mutex_t mtx_d;

	  cv::Mat color;
	  cv::Mat depth;
	  cv::Mat ir;
	  cv::Mat pc3DMat;
	  cv::Mat colorIndicsMapped2pc3DMat;

	  cv::Mat color_undist;
	  cv::Mat depth_undist;

	  std::string serialNumber;


	};
};

#endif
