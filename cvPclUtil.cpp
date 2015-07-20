//#include "stdafx.h"

#include <list>
#include <pcl/visualization/common/io.h>
#include <pcl/visualization/interactor_style.h>
#include <vtkVersion.h>
#include <vtkLODActor.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkCellArray.h>
#include <vtkTextProperty.h>
#include <vtkAbstractPropPicker.h>
#include <vtkCamera.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkScalarBarActor.h>
#include <vtkPNGWriter.h>
#include <vtkWindowToImageFilter.h>
#include <vtkRendererCollection.h>
#include <vtkActorCollection.h>
#include <vtkLegendScaleActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkObjectFactory.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkAssemblyPath.h>
#include <vtkAbstractPicker.h>
#include <vtkPointPicker.h>
#include <vtkAreaPicker.h>

#include <pcl/visualization/vtk/vtkVertexBufferObjectMapper.h>

#include "cvPclUtil.hpp"

Eigen::Affine3f 
imaiUtil::readExtrinsic_EigenAffine3f( std::string fileName, bool roteAroundZAxis )
{
	Eigen::Affine3f ret;
	cv::FileStorage fs( fileName, cv::FileStorage::READ );
	cv::Mat t;
	cv::Mat r_vec;
	cv::Mat r_mtx;
	fs["translation"] >> t;
	fs["rotation"]    >> r_vec;
	cv::Rodrigues(r_vec, r_mtx);
	//std::cout<<"t_vec"<< t<< std::endl;
	//std::cout<<"r_vec"<< r_vec<< std::endl;
	//std::cout<<"r_mtx"<< r_mtx<< std::endl;

	ret.matrix() <<
		r_mtx.at<double>(0,0), r_mtx.at<double>(0,1), r_mtx.at<double>(0,2), t.at<double>(0,0)*0.001,
		r_mtx.at<double>(1,0), r_mtx.at<double>(1,1), r_mtx.at<double>(1,2), t.at<double>(0,1)*0.001,
		r_mtx.at<double>(2,0), r_mtx.at<double>(2,1), r_mtx.at<double>(2,2), t.at<double>(0,2)*0.001,
		0,0,0,1;

	if(roteAroundZAxis)
		ret = Eigen::AngleAxisf( M_PI, Eigen::Vector3f::UnitZ() ) * ret;

	//std::cout<< "tf" << ret.matrix() << std::endl;
	
	//cv::Mat r_vec_r;
	//cv::Rodrigues(r_mtx , r_vec_r);
	//std::cout<<"r_vec_r"<< r_vec_r<< std::endl;
	
	return ret;
}

Eigen::Vector3f imaiUtil::rodriguess_EigenMatrix3f( Eigen::Matrix3f roteMtx )
{
	cv::Mat roteVec(1, 3, CV_32F);
	cv::Mat roteMat(3, 3, CV_32F);
	roteMat.at<float>(0,0) = roteMtx(0,0);
	roteMat.at<float>(0,1) = roteMtx(0,1);
	roteMat.at<float>(0,2) = roteMtx(0,2);
	roteMat.at<float>(1,0) = roteMtx(1,0);
	roteMat.at<float>(1,1) = roteMtx(1,1);
	roteMat.at<float>(1,2) = roteMtx(1,2);
	roteMat.at<float>(2,0) = roteMtx(2,0);
	roteMat.at<float>(2,1) = roteMtx(2,1);
	roteMat.at<float>(2,2) = roteMtx(2,2);

	cv::Rodrigues(roteMat, roteVec);

	Eigen:: Vector3f roteVec3f(
		roteVec.at<float>(0,0),
		roteVec.at<float>(1,0),
		roteVec.at<float>(2,0));

	return roteVec3f;
}