#ifndef __PCL_UTIL_HPP__
#define __PCL_UTIL_HPP__

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

#define MACRO_USE_PCL_UTIL \
	std::vector<imaiUtil::CoordinatedObjectClass*> imaiUtil::CoordinatedObjectClass::vct_cobjAll;


	class customPCLVisualizer : public pcl::visualization::PCLVisualizer 
	{
		
	};

	class customPCLVisualizerInteractorStyle : public pcl::visualization::PCLVisualizerInteractorStyle 
	{
	public:
		customPCLVisualizerInteractorStyle();

		int key;
		char* keySym;

	protected:
		//-- Override Inherit Functions --//

		//virtual void OnChar();
		
		// Keyboard events
        virtual void OnKeyDown ();
        //virtual void OnKeyUp ();
	};

	class CoordinatedObjectClass
	{
		//-- properties --//
	public:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_local;
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorHandler;
		std::string strID;
		int pointPix;

		Eigen::Affine3f tf;

		static std::vector<imaiUtil::CoordinatedObjectClass *> vct_cobjAll;

	private:
		pcl::visualization::PCLVisualizer* int_viewer;
		int* int_viewPort;
		bool flag_drawFirstTime;

		
		//-- methods --//
	public:
		CoordinatedObjectClass( std::string stringID = "_" );
		~CoordinatedObjectClass();
		void draw(pcl::visualization::PCLVisualizer& viewer, int& viewPort);
		void updateDrawing();
		void updateDrawing_withLocalPC();
		void updateDrawing_withTransformedLocalPC();
	};
	
#ifndef __CoordinatedCropBoxClass
#define __CoordinatedCropBoxClass

	template<class T_point>
	class CoordinatedCropBoxClass : public pcl::CropBox<T_point>
	{
	public:
		pcl::ModelCoefficients coeff;
		Eigen::Affine3f tf;
		std::string str_id;
		
	private:
		pcl::visualization::PCLVisualizer* int_viewer;
		int* int_viewPort;

	public:
		CoordinatedCropBoxClass();

		void setCropBoxParam( 
			Eigen::Vector3f translation,
			Eigen::Vector3f roteVec,
			Eigen::Vector3f size
			);

		void updateTf( Eigen::Affine3f* p_tfIn = (Eigen::Affine3f*)NULL );

		/*
		void setCropBoxParam( 
			Eigen::Vector3f translation,
			Eigen::Vector3f roteVec ,
			Eigen::Vector3f size
			);
		*/
		//void setCropBoxParam_fromCoeff();

		void draw(
			pcl::visualization::PCLVisualizer& viewer,
			int& viewPort
			);

		void updateDrawing();

	};



	template<class T_point>
	imaiUtil::CoordinatedCropBoxClass<T_point>::CoordinatedCropBoxClass()
		: pcl::CropBox<T_point>()
	{}


	template<class T_point>
	void imaiUtil::CoordinatedCropBoxClass<T_point>::setCropBoxParam(
				Eigen::Vector3f translation,
				Eigen::Vector3f roteVec,
				Eigen::Vector3f size
		)
	{
		//	pcl::ModelCoefficients box;
		//	box.values[0] = 0  ; box.values[1] = 0  ; box.values[2] = 0  ;						// translate
		//	box.values[3] = 0  ; box.values[4] = 0  ; box.values[5] = 0  ; box.values[6] = 0;	// rotation (quaternion)
		//	box.values[7] = 0.2; box.values[8] = 0.2; box.values[9] = 0.2;						// size
		
		// make quaternion from roteMat
		Eigen::Quaternionf quat(
			Eigen::AngleAxisf( roteVec.x(), Eigen::Vector3f::UnitX() ) *
			Eigen::AngleAxisf( roteVec.y(), Eigen::Vector3f::UnitY() ) *
			Eigen::AngleAxisf( roteVec.z(), Eigen::Vector3f::UnitZ() )
			);

		// init coeff for visual
		coeff.values.resize(10);
		coeff.values[0] = translation.x(); // x
		coeff.values[1] = translation.y(); // y
		coeff.values[2] = translation.z(); // z of inner translate
		coeff.values[3] = quat.x(); // i
		coeff.values[4] = quat.y(); // j
		coeff.values[5] = quat.z(); // k
		coeff.values[6] = quat.w(); // w of inner rotation (quaternion)
		coeff.values[7] = size.x();
		coeff.values[8] = size.y();
		coeff.values[9] = size.z(); // size

		// set box param
		this->setMax( Eigen::Vector4f( size.x()/2.0,  size.y()/2.0,  size.z()/2.0, 1.0f ) );
		this->setMin( Eigen::Vector4f(-size.x()/2.0, -size.y()/2.0, -size.z()/2.0, 1.0f ) );
		this->setTranslation(translation);	
		this->setRotation(roteVec);
	}


	template<class T_point>
	void imaiUtil::CoordinatedCropBoxClass<T_point>::updateTf( Eigen::Affine3f* p_tfIn )
	{
		if( p_tfIn != NULL )
			this->tf = *p_tfIn;

		//-- set filter param --//
		// this function set transform for inputCloud before filtering.
		// so neither coefficents for visual nor box param are not influenced.
		this->setTransform( this->tf.inverse() );
				
	}

	template<class T_point>
	void imaiUtil::CoordinatedCropBoxClass<T_point>::draw(
		pcl::visualization::PCLVisualizer& viewer,
		int& viewPort
		)
	{
		this->int_viewer   = &viewer  ;
		this->int_viewPort = &viewPort;
		viewer.addCube( this->coeff, this->str_id, *this->int_viewPort);
	}

	

	template<class T_point>
	void imaiUtil::CoordinatedCropBoxClass<T_point>::updateDrawing()
	{
		this->updateTf();
		//this->int_viewer->removeShape(this->str_id);
		//this->int_viewer->addCube( this->coeff, this->str_id, *this->int_viewPort);
		this->int_viewer->updateShapePose(this->str_id, this->tf);
	}

#endif

	class ChainedObjectBaseClass
	{
	public:
		Eigen::Affine3f tf_self;
		ChainedObjectBaseClass* parent;
		std::vector<ChainedObjectBaseClass*> chirdren;
	
	public:
		ChainedObjectBaseClass():
			parent(NULL)
		{}

		Eigen::Affine3f getTf_Base2Self()
		{
			if( this->parent == NULL )
				return this->tf_self;
			else
				return this->tf_self * parent->getTf_Base2Self();
		}

 
	};

	template< class T_point >
	int ransac_plane( 
		boost::shared_ptr<pcl::PointCloud<T_point>> cloud_in, 
		pcl::PointCloud<T_point>& cloud_out_inliers,
		pcl::PointCloud<T_point>& cloud_out_outliers,
		pcl::ModelCoefficients::Ptr coefficients,
		bool flag_coloring4cloudIn = false
		)
	{
		//-- RANSAC segmentation --//
		coefficients.reset( new pcl::ModelCoefficients );
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGB> objRANSAC;
		// Optional
		objRANSAC.setOptimizeCoefficients (true);
		// Mandatory
		objRANSAC.setModelType (pcl::SACMODEL_PLANE);
		objRANSAC.setMethodType (pcl::SAC_RANSAC);
		objRANSAC.setDistanceThreshold (0.01);
		objRANSAC.setInputCloud ( cloud_in );
		// execute RANSAC segmentation
		objRANSAC.segment (*inliers, *coefficients);
 

		//-- Extruct inlier/outlier of detected plane --//
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud ( cloud_in );
		extract.setIndices(inliers);
		extract.setNegative( false );
		extract.filter( cloud_out_inliers);
		extract.setNegative( true );
		extract.filter( cloud_out_outliers);

		//-- Coloring --//
		if(flag_coloring4cloudIn)
		{
			int size_inliers = inliers->indices.size();
			for(int i=0 ; i<size_inliers ; i++)
			{
				cloud_in->points[ inliers->indices[i] ].r = 255;
				cloud_in->points[ inliers->indices[i] ].g = 127;
				cloud_in->points[ inliers->indices[i] ].b = 127; 
			}
		}


		if (inliers->indices.size () == 0)
		{
			PCL_ERROR ("Could not estimate a planar model for the given dataset.");
			return -1;
		}
		else
		{
			std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
									<< coefficients->values[1] << " "
									<< coefficients->values[2] << " " 
									<< coefficients->values[3] << std::endl;
			return 0;
		}

	}



		/*
		template<class T_point>
			void setCropBoxCoefficients( 
				pcl::CropBox<T_point>& cropBox ,
				pcl::ModelCoefficients& box 
				);
		*/
};

#endif