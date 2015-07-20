#ifndef __CV_PCL_UTIL_HPP__
#define __CV_PCL_UTIL_HPP__

#include <opencv2/opencv.hpp>


//-- PCL --//
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/features/normal_3d.h>  
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>  
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/search.h>
#include <pcl/registration/icp.h>

#include <pcl/visualization/cloud_viewer.h>  


namespace imaiUtil
{
	Eigen::Affine3f readExtrinsic_EigenAffine3f( std::string fileName , bool roteAroundZAxis = false );
	Eigen::Vector3f rodriguess_EigenMatrix3f( Eigen::Matrix3f roteMtx );
};

#endif