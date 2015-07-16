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

#include "pclUtil.hpp"
	
imaiUtil::customPCLVisualizerInteractorStyle::customPCLVisualizerInteractorStyle() :
	pcl::visualization::PCLVisualizerInteractorStyle(),
	key(0),
	keySym(0)
{
	
}

// Keyboard events
//void imaiUtil::customPCLVisualizerInteractorStyle::OnChar(){}
		
void imaiUtil::customPCLVisualizerInteractorStyle::OnKeyDown ()
{
	this->key = this->Interactor->GetKeyCode();
	this->keySym = this->Interactor->GetKeySym();
}

//void imaiUtil::customPCLVisualizerInteractorStyle::OnKeyUp (){}

imaiUtil::CoordinatedObjectClass::CoordinatedObjectClass():
	pc( new pcl::PointCloud<pcl::PointXYZRGB>() ),
	pc_local( new pcl::PointCloud<pcl::PointXYZRGB>() ),
	flag_drawFirstTime(true)
{
	//this->pc = (new pcl::PointCloud<pcl::PointXYZRGB>() );
	
	this->tf.matrix() = Eigen::Matrix4f::Identity();
	this->pointPix = 3;
	this->strID = "_";
}


void imaiUtil::CoordinatedObjectClass::draw(pcl::visualization::PCLVisualizer& viewer, int& viewPort)
{
	if(this->flag_drawFirstTime)
	{
		this->flag_drawFirstTime = false;

		this->colorHandler.setInputCloud(this->pc);
		viewer.addPointCloud<pcl::PointXYZRGB> ( this->pc, this->colorHandler, this->strID, viewPort);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, this->pointPix, this->strID, viewPort);
	
		/*
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_temp( new pcl::PointCloud<pcl::PointXYZRGB>() );
		this->colorHandler.setInputCloud(this->pc);

		pcl::transformPointCloud( *this->pc, *pc_temp, this->tf.matrix() );

		viewer.addPointCloud<pcl::PointXYZRGB> ( pc_temp, this->colorHandler, this->strID, viewPort);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, this->pointPix, this->strID, viewPort);
		*/
		this->int_viewer   = &viewer  ;
		this->int_viewPort = &viewPort;
	}
	else
	{
		/*
		if( &viewer != this->int_viewer )
		{
			// this is challenge for switch view_port or viewer.
		}
		*/
		this->updateDrawing();
	}
}

void imaiUtil::CoordinatedObjectClass::updateDrawing()
{
	/*
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_temp( new pcl::PointCloud<pcl::PointXYZRGB>() );
	this->colorHandler.setInputCloud(this->pc);

	pcl::transformPointCloud( *this->pc, *pc_temp, this->tf.matrix() );

	int_viewer->updatePointCloud( pc_temp, this->strID );
	*/
	
	this->int_viewer->updatePointCloud( pc, this->strID );
}

void imaiUtil::CoordinatedObjectClass::updateDrawing_withLocalPC()
{
	int_viewer->updatePointCloud( pc_local, this->strID );	
}

void imaiUtil::CoordinatedObjectClass::updateDrawing_withTransformedLocalPC()
{
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_temp( new pcl::PointCloud<pcl::PointXYZRGB>() );
	//this->colorHandler.setInputCloud(this->pc);
	pcl::transformPointCloud( *this->pc_local, *pc_temp, this->tf.matrix() );

	int_viewer->updatePointCloud( pc_temp, this->strID );
	
}


/*
template<class T_point>
	void setCropBoxCoefficients(
		pcl::CropBox<T_point>& cropBox ,
		pcl::ModelCoefficients& box 
		)
	{
		cropBox.setMax
	}
*/


