#include "stdafx.h"

#define __CVMAT_TO_PCLPC_CPP__
#ifdef  __CVMAT_TO_PCLPC_CPP__

#include "cvMat2PclPc.hpp"

HRESULT imaiUtil::makePointCloudXYZRGB_fromPc3DMat( 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_dst,
		cv::Mat &pc3DMat,
		cv::Mat &colorIndicsMapped2Depth,
		cv::Mat &colorMat,
		cv::Mat &depthMat
		)
{
	
	int width_d  = depthMat.cols;
	int height_d = depthMat.rows;

	int width_c  = colorMat.cols;
	int height_c = colorMat.rows;

	/*
	for( int y = 0; y < height_d; y++ ){
		for( int x = 0; x < width_d; x++ ){
			pcl::PointXYZRGB point;
 
			//DepthSpace depthSpacePoint = { static_cast<float>( x ), static_cast<float>( y ) };
			UINT16 depth = ((UINT16*)depthMat.data)[y * width_d + x];
 
			// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
			int colorX = static_cast<int>( std::floor( ((float*)(colorIndicsMapped2Depth.data))[2*(y * width_d + x)+0] + 0.5f ) );
			int colorY = static_cast<int>( std::floor( ((float*)(colorIndicsMapped2Depth.data))[2*(y * width_d + x)+1] + 0.5f ) );
			if( ( 0 <= colorX ) && ( colorX < width_c ) && ( 0 <= colorY ) && ( colorY < height_c ) ){
				//RGBQUAD color = colorMat.data[colorY * width_c + colorX];
				//point.b = color.rgbBlue;
				//point.g = color.rgbGreen;
				//point.r = color.rgbRed;
				int dataIndex = 4*(colorY * width_c + colorX);
				point.b = colorMat.data[dataIndex];
				point.g = colorMat.data[dataIndex+1];
				point.r = colorMat.data[dataIndex+2];
			}
 
			// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
			//CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
			//pCoordMapper->MapDepthPointToCameraSpace( depthSpacePoint, depth, &cameraSpacePoint );
			if( ( 0 <= colorX ) && ( colorX < width_c ) && ( 0 <= colorY ) && ( colorY < height_c ) ){
				//point.x = cameraSpacePoint.X;
				//point.y = cameraSpacePoint.Y;
				//point.z = cameraSpacePoint.Z;
				point.x = ((cv::Point3f*)(pc3DMat.data))[y * width_d + x].x;
				point.y = ((cv::Point3f*)(pc3DMat.data))[y * width_d + x].y;
				point.z = ((cv::Point3f*)(pc3DMat.data))[y * width_d + x].z;
			}
 
			pc_dst->push_back( point );
		}
	}
	*/
		
	for( int y = 0; y < height_d; y++ )
	{
		for( int x = 0; x < width_d; x++ )
		{
			pcl::PointXYZRGB point;
			int i = y * width_d + x;
			
			// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
			int colorX = static_cast<int>( std::floor( ((float*)(colorIndicsMapped2Depth.data))[2*i + 0] + 0.5f ) );
			int colorY = static_cast<int>( std::floor( ((float*)(colorIndicsMapped2Depth.data))[2*i + 1] + 0.5f ) );
			if( ( 0 <= colorX ) && ( colorX < width_c ) && ( 0 <= colorY ) && ( colorY < height_c ) ){
				int dataIndex = 4*(colorY * width_c + colorX);
				point.b = colorMat.data[dataIndex];
				point.g = colorMat.data[dataIndex+1];
				point.r = colorMat.data[dataIndex+2];
			}
			else
			{
				point.b = 255;
				point.g = 255;
				point.r = 255;
			}
 
			// Setting PointCloud XYZ
			point.x = ((cv::Point3f*)(pc3DMat.data))[i].x;
			point.y = ((cv::Point3f*)(pc3DMat.data))[i].y;
			point.z = ((cv::Point3f*)(pc3DMat.data))[i].z;
			pc_dst->push_back( point );
		}
	}
	return S_OK;
}


HRESULT imaiUtil::makePointCloudXYZRGB_fromPc3DMatMasked( 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_dst,
		cv::Mat &pc3DMat,
		cv::Mat &colorIndicsMapped2Depth,
		cv::Mat &colorMat,
		cv::Mat &depthMat,
		cv::Mat &maskMat,
		unsigned int th_far,
		unsigned int th_near
		)
{
	
	int width_d  = depthMat.cols;
	int height_d = depthMat.rows;

	int width_c  = colorMat.cols;
	int height_c = colorMat.rows;

	for( int y = 0; y < height_d; y++ )
	{
		for( int x = 0; x < width_d; x++ )
		{
			pcl::PointXYZRGB point;
			int i = y * width_d + x;
 
			if( ( ((unsigned char* )maskMat .data)[i] == 255     ) &&
				( ((unsigned short*)depthMat.data)[i] <  th_far  ) &&
				( ((unsigned short*)depthMat.data)[i] >  th_near ) )
			{
				// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
				int colorX = static_cast<int>( std::floor( ((float*)(colorIndicsMapped2Depth.data))[2*i + 0] + 0.5f ) );
				int colorY = static_cast<int>( std::floor( ((float*)(colorIndicsMapped2Depth.data))[2*i + 1] + 0.5f ) );
				if( ( 0 <= colorX ) && ( colorX < width_c ) && ( 0 <= colorY ) && ( colorY < height_c ) ){
					int dataIndex = 4*(colorY * width_c + colorX);
					point.b = colorMat.data[dataIndex];
					point.g = colorMat.data[dataIndex+1];
					point.r = colorMat.data[dataIndex+2];
				}
				else
				{
					int dataIndex = 4*(colorY * width_c + colorX);
					point.b = 255;
					point.g = 255;
					point.r = 255;
				}
 
				// Setting PointCloud XYZ
				point.x = ((cv::Point3f*)(pc3DMat.data))[i].x;
				point.y = ((cv::Point3f*)(pc3DMat.data))[i].y;
				point.z = ((cv::Point3f*)(pc3DMat.data))[i].z;
				pc_dst->push_back( point );
			}
		}
	}

	return S_OK;
}


#endif