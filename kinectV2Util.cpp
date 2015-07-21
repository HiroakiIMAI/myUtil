#include "kinectV2Util.hpp"

imaiUtil::KinectV2Class::KinectV2Class():
	pColorFrame(nullptr),
	pDepthFrame(nullptr),
	pInfraredFrame(nullptr),
	width_c(0), width_d(0), width_ir(0),
	height_c(0), height_d(0), height_ir(0)
{
	//---- Sensor open ----//
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor( &pSensor );
	if( FAILED( hResult ) )
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
	hResult = pSensor->Open();
	if( FAILED( hResult ) )
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;


	//---- set Color reader ----//
	hResult = pSensor->get_ColorFrameSource( &pColorSource );
	if( FAILED( hResult ) )
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
	hResult = pColorSource->OpenReader( &pColorReader );
	if( FAILED( hResult ) )
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
	hResult = pColorSource->get_FrameDescription( &pColorFrameDescription );
	if( FAILED( hResult ) )
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;

	pColorFrameDescription->get_Width( &width_c ); // 1920
	pColorFrameDescription->get_Height( &height_c ); // 1080
	bufferSize_c = width_c * height_c * 4 * sizeof( unsigned char );
	int_color      = cv::Mat( height_c  , width_c  , CV_8UC4 );
	int_color_half = cv::Mat( height_c/2, width_c/2, CV_8UC4 );

	//---- set Depth reader ----//
	hResult = pSensor->get_DepthFrameSource( &pDepthSource );
	if( FAILED( hResult ) )
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;

	hResult = pDepthSource->OpenReader( &pDepthReader );
	if( FAILED( hResult ) )
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;

	hResult = pDepthSource->get_FrameDescription( &pDepthFrameDescription );
	if( FAILED( hResult ) )
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;

	pDepthFrameDescription->get_Width( &width_d );		// 512
	pDepthFrameDescription->get_Height( &height_d );	// 424
	bufferSize_d = width_d * height_d * sizeof( unsigned short );
	int_depth      = cv::Mat( height_d  , width_d  , CV_16UC1 );
	int_depth_8bit = cv::Mat( height_d, width_d, CV_8UC1 );
	depth_8bit = cv::Mat( height_d, width_d, CV_8UC1 );

	//---- set IR reader ----//
	hResult = pSensor->get_InfraredFrameSource( &pInfraredSource );
	if( FAILED( hResult ) )
		std::cerr << "Error : IKinectSensor::get_InfraredFrameSource()" << std::endl;

	hResult = pInfraredSource->OpenReader( &pInfraredReader );
	if( FAILED( hResult ) )
		std::cerr << "Error : IInfraredFrameSource::OpenReader()" << std::endl;

	hResult = pInfraredSource->get_FrameDescription( &pInfraredFrameDescription );
	if( FAILED( hResult ) )
		std::cerr << "Error : IIInfraredFrameSource::get_FrameDescription()" << std::endl;

	pInfraredFrameDescription->get_Width( &width_ir );		// 512
	pInfraredFrameDescription->get_Height( &height_ir );	// 424
	bufferSize_ir = width_ir * height_ir * sizeof( unsigned short );
	int_IR      = cv::Mat( height_ir  , width_ir  , CV_16UC1 );

	cv::waitKey( 3000 ); // This delay is necessary. (over 3[sec])
}

imaiUtil::KinectV2Class::~KinectV2Class()
{
	SafeRelease( pColorSource );
	SafeRelease( pColorReader );
	SafeRelease( pColorFrameDescription );

	SafeRelease( pDepthSource );
	SafeRelease( pDepthReader );
	SafeRelease( pDepthFrameDescription );

	SafeRelease( pInfraredSource );
	SafeRelease( pInfraredReader );
	SafeRelease( pInfraredFrameDescription );

	pSensor->Close();
	SafeRelease(pSensor);
}

void imaiUtil::KinectV2Class::getColor()
{
	HRESULT hResult = pColorReader->AcquireLatestFrame( &pColorFrame );
	if( SUCCEEDED( hResult ) ){
		hResult = pColorFrame->CopyConvertedFrameDataToArray( bufferSize_c, reinterpret_cast<BYTE*>( int_color.data ), ColorImageFormat_Bgra );
		if( SUCCEEDED( hResult ) ){
			int_color.copyTo( color );
			cv::resize( color, color_half, cv::Size(), 0.5, 0.5 );
			cv::flip( color		, color		, 1 );
			cv::flip( color_half, color_half, 1 );
			
		}
	}
	SafeRelease( pColorFrame );
}

void imaiUtil::KinectV2Class::getDepth()
{
	HRESULT hResult = pDepthReader->AcquireLatestFrame( &pDepthFrame );
	if( SUCCEEDED( hResult ) ){
		hResult = pDepthFrame->AccessUnderlyingBuffer( &bufferSize_d, reinterpret_cast<UINT16**>( &int_depth.data ) );
		if( SUCCEEDED( hResult ) ){
			int_depth.copyTo( depth );
			depth.convertTo( depth_8bit, CV_8U, -255.0f / 4500.0f, 255.0f );
			cv::flip( depth_8bit, depth_8bit, 1 );
			cv::flip( depth		, depth		, 1 );
		}
	}
	SafeRelease( pDepthFrame );
}

void imaiUtil::KinectV2Class::getIR()
{
	HRESULT hResult = pInfraredReader->AcquireLatestFrame( &pInfraredFrame );

	if( SUCCEEDED( hResult ) ){
		hResult = pInfraredFrame->AccessUnderlyingBuffer( &bufferSize_ir, reinterpret_cast<UINT16**>( &int_IR.data ) );
		if( SUCCEEDED( hResult ) ){
			int_IR.copyTo(IR);
			cv::flip( IR, IR, 1 );
		}
	}
	SafeRelease( pInfraredFrame );
}