#include "RealSenseUtil.hpp"

using namespace imaiUtil;

void cb_color(uvc_frame_t *frame, void *ptr) {
  uvc_frame_t *bgr;
  uvc_error_t ret;

  /* We'll convert the image from YUV/JPEG to BGR, so allocate space */
  bgr = uvc_allocate_frame(frame->width * frame->height * 3);
  if (!bgr) {
    printf("unable to allocate bgr frame!");
    return;
  }

  /* Do the BGR conversion */
  ret = uvc_any2bgr(frame, bgr);
  if (ret) {
    uvc_perror(ret, "uvc_any2bgr");
    uvc_free_frame(bgr);
    return;
  }

  RealSenseClass* p_rs = (RealSenseClass*)ptr;
  pthread_mutex_lock( &p_rs->mtx_c );
  memcpy(
		  p_rs->getColorBufferPointer(),
		  bgr->data,
		  bgr->data_bytes
		);
  pthread_mutex_unlock( &p_rs->mtx_c );

  uvc_free_frame(bgr);
}

void cb_depth(uvc_frame_t *frame, void *ptr) {
  uvc_error_t ret;
  RealSenseClass* p_rs = (RealSenseClass*)ptr;

  pthread_mutex_lock( &p_rs->mtx_d );
  memcpy(
		  p_rs->getDepthBufferPointer(),
		  frame->data,
		  frame->data_bytes
		);
  pthread_mutex_unlock( &p_rs->mtx_d );
  //cv::imshow( "depth test2", p_rs->depth_buff);

  //cv::waitKey(1);
}

RealSenseClass::RealSenseClass(
			  enum uvc_frame_format colorFrameFormat,
			  cv::Size colorSize,
			  enum uvc_frame_format depthFrameFormat,
			  cv::Size depthSize,
			  const char* deviceSerialNumber,
			  bool verbose):
					  calibrated(false)
{
	openRealSense(
			colorFrameFormat, colorSize,
			depthFrameFormat, depthSize,
			deviceSerialNumber,
			verbose
			);

	this->pc3DMat 				    = cv::Mat( depthSize, CV_32FC3 );
	this->colorIndicsMapped2pc3DMat = cv::Mat( depthSize, CV_32SC2 );
}

int RealSenseClass::openDevice(const char* deviceSerialNumber, bool verbose)
{
	uvc_error_t res;

	// Initialize a UVC service context. Libuvc will set up its own libusb
	// context. Replace NULL with a libusb_context pointer to run libuvc
	// from an existing libusb context.
	res = uvc_init(&ctx, NULL);

	if (res < 0)
	{
		uvc_perror(res, "uvc_init");
		return res;
	}
	//puts("UVC initialized");

	// Locates the first attached UVC device, stores in dev
	res = uvc_find_device(
			ctx, &dev,
			0x8086,
			0x0a66,
			deviceSerialNumber); // filter devices: vendor_id, product_id, "serial_num"

	if (res < 0)
	{
		uvc_perror(res, "uvc_find_device"); // no devices found
		return -1;
	}

	usleep(5000);

	puts("Device found");
	// get and print device discription
	uvc_device_descriptor* desc;
	if (uvc_get_device_descriptor(dev, &desc) != UVC_SUCCESS)
		std::cout<< "device seralNumber : "<< "faled to get serial num"<< std::endl;
	else
		std::cout<< "device seralNumber : "<< desc->serialNumber<< std::endl;
	this->serialNumber = desc->serialNumber;
	uvc_free_device_descriptor(desc);

	return(0);
}

int RealSenseClass::openColor(
		enum uvc_frame_format colorFrameFormat,
		cv::Size imageSize,
		bool verbose )
{
	uvc_error_t res;

	//---- Color Camera ----//
	if( colorFrameFormat != 0 )
	{
		printf("Opening RGB camera\n");
		res = uvc_open2(dev, &devh_rgb, 0); //Try to open camera 0 (RGB)
		if (res < 0)
		{
			uvc_perror(res, "uvc_open"); /* unable to open device */
			return -1;
		}
		if(verbose)
			// Print out a message containing all the information that libuvc
			// knows about the device
			uvc_print_diag(devh_rgb, stderr);
		puts("RGB camera opened");

		usleep(5000);


		// Try to negotiate a 640x480 30 fps YUYV stream profile
		res = uvc_get_stream_ctrl_format_size(
		  devh_rgb, &ctrl_rgb, // result stored in ctrl
		  colorFrameFormat,
		  imageSize.width, imageSize.height, 30 // width, height, fps
		);
		if (res < 0)
		{
			uvc_perror(res, "get_mode"); // device doesn't provide a matching stream
			return -1;
		}
		this->color      = cv::Mat(imageSize, CV_8UC3);
		this->color_buff = cv::Mat(imageSize, CV_8UC3);
		if(verbose)
		{
			// Print out the result
			std::cout<< "following format is supported:"<< std::endl;
			uvc_print_stream_ctrl(&ctrl_rgb, stderr);
		}

		usleep(200000);
		//sleep(1);

		pthread_mutex_init( &this->mtx_c, NULL );
		res = uvc_start_streaming(devh_rgb, &ctrl_rgb, cb_color, (void*)this, 0);
		if (res < 0)
		{
			uvc_perror(res, "start_streaming"); // unable to start stream
			return -1;
		}
		std::cout<< "start streaming RGB"<< std::endl;
	} // end if( colorFrameFormat != 0 )
	return(0);
}

int RealSenseClass::openDepth(
		enum uvc_frame_format depthFrameFormat,
		cv::Size imageSize,
		bool verbose)
{
	uvc_error_t res;

	if( depthFrameFormat != 0 )
	{
		usleep(10000);
		printf("Opening Depth camera\n");
		std::cout<< "following format is supported:"<< std::endl;
		res = uvc_open2(dev, &devh_d, 1); //Try to open camera 1  (depth)
		if (res < 0)
		{
			uvc_perror(res, "uvc_open"); // unable to open device
			return -1;
		}
		if(verbose)
			uvc_print_diag(devh_d, stderr);
		puts("Depth camera opened");

		usleep(10000);

		// Try to negotiate a 640x480 30 fps YUYV stream profile
		res = uvc_get_stream_ctrl_format_size(
				devh_d, &ctrl_d, // result stored in ctrl
				depthFrameFormat,
				//UVC_FRAME_FORMAT_INVI,
				//UVC_FRAME_FORMAT_RELI,
				//UVC_FRAME_FORMAT_INVR,
				//UVC_FRAME_FORMAT_INVZ,
				//UVC_FRAME_FORMAT_INRI,
				imageSize.width, imageSize.height, 30 // width, height, fps
		);
		if (res < 0)
		{
			uvc_perror(res, "get_mode"); // device doesn't provide a matching stream
			return -1;
		}
		std::cout<< "get mode ok"<<std::endl;
		switch( depthFrameFormat )
		{
		case UVC_FRAME_FORMAT_INRI:
			this->depth      = cv::Mat( imageSize, CV_8UC3 );
			this->depth_buff = cv::Mat( imageSize, CV_8UC3 );
			break;
		case UVC_FRAME_FORMAT_INVI:
			this->depth      = cv::Mat( imageSize, CV_8UC1 );
			this->depth_buff = cv::Mat( imageSize, CV_8UC1 );
			break;
		default:
			this->depth      = cv::Mat( imageSize, CV_16U );
			this->depth_buff = cv::Mat( imageSize, CV_16U );
		}

		if(verbose)
		{
			// Print out the result
			std::cout<< "following format is supported:"<< std::endl;
			uvc_print_stream_ctrl(&ctrl_d, stderr);
			//uvc_print_diag(devh_d, NULL);
		}

		usleep(200000);
		//sleep(1);

		pthread_mutex_init( &this->mtx_d, NULL );
		res = uvc_start_streaming(devh_d, &ctrl_d, cb_depth, (void*)this, 0);
		if (res < 0)
		{
			uvc_perror(res, "start_streaming"); // unable to start stream
			return -1;
		}
		std::cout<< "start streaming Depth"<< std::endl;
		this->depthFormat = depthFrameFormat;
	} // end if( depthFrameFormat != 0 )
	return(0);
}

int RealSenseClass::openRealSense(
			  enum uvc_frame_format colorFrameFormat,
			  cv::Size colorSize,
			  enum uvc_frame_format depthFrameFormat,
			  cv::Size depthSize,
			  const char* deviceSerialNumber,
			  bool verbose)
{
	uvc_error_t res;

	if( this->openDevice(deviceSerialNumber) != 0  )
		return(-1);

	if( this->openColor( colorFrameFormat, colorSize, verbose ) != 0 )
		return(-1);

	if( this->openDepth( depthFrameFormat, depthSize, verbose ) != 0 )
		return(-1);

	return 0;
}

RealSenseClass::~RealSenseClass()
{
//	uvc_stop_streaming( this->devh_rgb );
//	uvc_stop_streaming( this->devh_d );
//	std::cout<< "steaming is stopped"<< std::endl;
//	usleep(200000);
//	uvc_close( this->devh_rgb );
//	uvc_close( this->devh_d );
//	std::cout<< "device handle is closed"<< std::endl;
//	usleep(200000);
//	uvc_unref_device( this->dev );
}

unsigned char* RealSenseClass::getColorBufferPointer()
{
	return this->color_buff.data;
}

unsigned char* RealSenseClass::getDepthBufferPointer()
{
	return this->depth_buff.data;
}

int RealSenseClass::loadCalibData()
{
	std::string str_path;
	if( this->serialNumber == REALSENSE01 )	{
		str_path = "data/RealSense-01/";
		std::cout<< "RealSense-01 is opened"<< std::endl;
	}
	else if(this->serialNumber == REALSENSE02)	{
		str_path = "data/RealSense-02/";
		std::cout<< "RealSense-02 is opened"<< std::endl;
	}
	else if(this->serialNumber == REALSENSE03)	{
		str_path = "data/RealSense-03/";
		std::cout<< "RealSense-03 is opened"<< std::endl;
	}
	else if(this->serialNumber == HAYASHI){
		str_path = "data/hayashi/";
		std::cout<< "RealSense-hayashi is opened"<< std::endl;
	}
	else{
		std::cout<< "load calib data faled."<< std::endl;
		return -1;
	}

	cv::Mat rotation;
	cv::FileStorage fs;
	fs.open( str_path + "intrinsic_Color.xml", cv::FileStorage::READ );
	fs["camera_matrix"]          >> this->intrinsic_color.camMtx;
	fs["distortion_coefficients"]>> this->intrinsic_color.distCoeff;
	fs.open( str_path + "intrinsic_IR.xml", cv::FileStorage::READ );
	fs["camera_matrix"]          >> this->intrinsic_depth.camMtx;
	fs["distortion_coefficients"]>> this->intrinsic_depth.distCoeff;
	fs.open( str_path + "stereoParam.xml", cv::FileStorage::READ );
	fs["translation"]          >> this->T_vec;
	fs["rotation"]>> rotation;
	if( ( (rotation.cols==1) && (rotation.rows==3) ) ||
		( (rotation.cols==3) && (rotation.rows==1) ) )
	{
		this->R_vec = rotation;
		cv::Rodrigues( rotation, this->R_mtx);
	}
	else if( (rotation.cols==3) || (rotation.rows==3) )
	{
		this->R_mtx = rotation;
		cv::Rodrigues(rotation, this->R_vec);
	}
	else
	{ std::cout<< "ERROR, invaled matrix type : readExtrinsic()"<< std::endl; return -1;}


	/*
	std::cout<< "intrinsic_color:"<<std::endl;
	std::cout<< this->intrinsic_color.camMtx<< std::endl;
	std::cout<< this->intrinsic_color.distCoeff<< std::endl;
	std::cout<< "intrinsic_depth:"<<std::endl;
	std::cout<< this->intrinsic_depth.camMtx<< std::endl;
	std::cout<< this->intrinsic_depth.distCoeff<< std::endl;
	 */
	std::cout<< "stereoParam:"<<std::endl;
	std::cout<< this->T_vec<< std::endl;
	std::cout<< this->R_vec<< std::endl;


	if(   (    (this->intrinsic_color.camMtx.rows    == 0)
			|| (this->intrinsic_color.distCoeff.rows == 0)
			|| (this->intrinsic_depth.camMtx.rows    == 0)
			|| (this->intrinsic_depth.distCoeff.rows == 0) ) )
	{
		std::cout<< "load calib data faled."<< std::endl;
		return -1;
	}

	// adjust intrinsic ( if color cam image size is defferent from calibrated [= not 640x480].  )
	this->intrinsic_color.camMtx.at<double>(0,2) += (this->color.cols - 640)/2; // cx
	this->intrinsic_color.camMtx.at<double>(1,2) += (this->color.rows - 480)/2; // cy
	this->intrinsic_color.camMtx.at<double>(0,0) *= (this->color.rows/480.f); // fx
	this->intrinsic_color.camMtx.at<double>(1,1) *= (this->color.rows/480.f); // fy

	// make undistort color map
	cv::initUndistortRectifyMap(
			this->intrinsic_color.camMtx,
			this->intrinsic_color.distCoeff,
			cv::Mat(),
			this->intrinsic_color.camMtx,
			cv::Size(this->color.cols, this->color.rows),
			CV_16SC2,
			this->intrinsic_color.undistMap1,
			this->intrinsic_color.undistMap2
			);

	// make undistort depth map
	cv::initUndistortRectifyMap(
			this->intrinsic_depth.camMtx,
			this->intrinsic_depth.distCoeff,
			cv::Mat(),
			this->intrinsic_depth.camMtx,
			cv::Size(this->depth.cols, this->depth.rows),
			CV_16SC2,
			this->intrinsic_depth.undistMap1,
			this->intrinsic_depth.undistMap2
			);


	this->calibrated = true;
	return 0;

}

cv::Mat* RealSenseClass::getColorImage()
{
	pthread_mutex_lock( &this->mtx_c );
	this->color_buff.copyTo( this->color );
	pthread_mutex_unlock( &this->mtx_c );

	if(calibrated)
	{
		cv::remap(
			this->color,
			this->color_undist,
			this->intrinsic_color.undistMap1,
			this->intrinsic_color.undistMap2,
			cv::INTER_LINEAR
			);
	}
	return &this->color;
}

cv::Mat* RealSenseClass::getDepthImage()
{
	pthread_mutex_lock( &this->mtx_d );
	this->depth_buff.copyTo( this->depth );
	pthread_mutex_unlock( &this->mtx_d );

	if(calibrated)
	{
		cv::remap(
			this->depth,
			this->depth_undist,
			this->intrinsic_depth.undistMap1,
			this->intrinsic_depth.undistMap2,
			cv::INTER_NEAREST
			);
	}

	return &this->depth;
}

cv::Mat* RealSenseClass::getPc3DMat()
{
	if(!this->calibrated)
	{
		std::cout<< "this sensor is not calibrated!"<< std::endl;
		return NULL;
	}

	const int width_d  = this->depth.cols;
	const int height_d = this->depth.rows;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud( new pcl::PointCloud<pcl::PointXYZRGB> );
	//pcl::PointXYZRGB point;

	/*
	double fx_c = this->intrinsic_color.camMtx.at<double>(0,0);
	double fy_c = this->intrinsic_color.camMtx.at<double>(1,1);
	double cx_c = this->intrinsic_color.camMtx.at<double>(0,2);
	double cy_c = this->intrinsic_color.camMtx.at<double>(1,2);
	cx_c += (this->color.cols - 640)/2;
	cy_c += (this->color.rows - 480)/2;
	 */

	double fx_d = this->intrinsic_depth.camMtx.at<double>(0,0);
	double fy_d = this->intrinsic_depth.camMtx.at<double>(1,1);
	double cx_d = this->intrinsic_depth.camMtx.at<double>(0,2);
	double cy_d = this->intrinsic_depth.camMtx.at<double>(1,2);

	if( this->depthFormat == UVC_FRAME_FORMAT_INVR )
	{
		for(int v=0; v<height_d; v++ )
		{
			for(int u=0; u<width_d; u++)
			{
				double r2 = (u-cx_d)*(u-cx_d) + (v-cy_d)*(v-cy_d);
				double r  = sqrt(r2);
				double d2 =  r2 + (fx_d*fx_d);
				double D  = ((unsigned short*)this->depth_undist.data)[v*width_d+u] / 30.5/*[depth/mm]*/ /1000.0; // [m]
				double R2 = D*D * ( r2 / d2 );
				double R  = sqrt(R2);

				((cv::Point3f*)this->pc3DMat.data)[v*width_d+u].x = R * ((u-cx_d) /r);
				((cv::Point3f*)this->pc3DMat.data)[v*width_d+u].y = R * ((v-cy_d) /r);
				((cv::Point3f*)this->pc3DMat.data)[v*width_d+u].z = sqrt( D*D - R2 );
			}
		}
	}
	else
	{
		std::cout<< "Error! invaled depth format."<< std::endl;
		return NULL;
	}

	return &this->pc3DMat;
}

cv::Mat* RealSenseClass::getColorIndicesMapped2Depth()
{
	if(!this->calibrated)
	{
		std::cout<< "this sensor is not calibrated!"<< std::endl;
		return NULL;
	}

	std::vector<cv::Point2f> indicies;
	std::vector<cv::Point3f> points;
	this->colorIndicsMapped2pc3DMat = cv::Mat::zeros(this->pc3DMat.rows, this->pc3DMat.cols, CV_32SC2);

	cv::Point3f point;
	for(int v=0; v<this->pc3DMat.rows; v++)
		for(int u=0; u<this->pc3DMat.cols; u++)
		{
			point = ((cv::Point3f*)this->pc3DMat.data)[ v*this->pc3DMat.cols + u];
			point.x *= 1000; // now use scale [mm]
			point.y *= 1000;
			point.z *= 1000;
			points.push_back(point);
		}

	cv::projectPoints(
			points,
			-this->R_vec,
			-this->T_vec, // [mm]
			this->intrinsic_color.camMtx,
			this->intrinsic_color.distCoeff,
			indicies);

	float cx = this->intrinsic_color.camMtx.at<double>(0,2);
	float cy = this->intrinsic_color.camMtx.at<double>(1,2);
	for(int v=0; v<this->pc3DMat.rows; v++)
		for(int u=0; u<this->pc3DMat.cols; u++)
		{
			if( (0<=indicies[v*this->pc3DMat.cols+u].x) && ( indicies[v*this->pc3DMat.cols+u].x < this->color.cols) &&
				(0<=indicies[v*this->pc3DMat.cols+u].y) && ( indicies[v*this->pc3DMat.cols+u].y < this->color.rows) )
			{
				((cv::Point2i*)this->colorIndicsMapped2pc3DMat.data)[v*this->pc3DMat.cols + u] = indicies[v*this->pc3DMat.cols+u];
			}
			else
			{
				((cv::Point2i*)this->colorIndicsMapped2pc3DMat.data)[v*this->pc3DMat.cols + u] = cv::Point2i(-1,-1);
			}
		}
	return &this->colorIndicsMapped2pc3DMat;
}

bool RealSenseClass::isCalibrated()
{
	return this->calibrated;
}
