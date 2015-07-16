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

	private:
		pcl::visualization::PCLVisualizer* int_viewer;
		int* int_viewPort;
		bool flag_drawFirstTime;

		//-- methods --//
	public:
		CoordinatedObjectClass();
		void draw(pcl::visualization::PCLVisualizer& viewer, int& viewPort);
		void updateDrawing();
		void updateDrawing_withLocalPC();
		void updateDrawing_withTransformedLocalPC();
	};
	
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

		void updateCoeff_byTf();

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

		Eigen::Matrix3f roteMat;
		roteMat = Eigen::Matrix3f::Identity()
			* Eigen::AngleAxisf( roteVec.x() , Eigen::Vector3f::UnitX() )
			* Eigen::AngleAxisf( roteVec.y() , Eigen::Vector3f::UnitY() )
			* Eigen::AngleAxisf( roteVec.z() , Eigen::Vector3f::UnitZ() );
		Eigen::Quaternionf quaternion(roteMat);
	
		std::cout<< "roteMat is"<< std::endl;
		std::cout<< roteMat<< std::endl;

		// set transform 
		this->tf.translation() = translation;
		this->tf.matrix()(0,0)= roteMat(0,0);
		this->tf.matrix()(0,1)= roteMat(0,1);
		this->tf.matrix()(0,2)= roteMat(0,2);
		this->tf.matrix()(1,0)= roteMat(1,0);
		this->tf.matrix()(1,1)= roteMat(1,1);
		this->tf.matrix()(1,2)= roteMat(1,2);
		this->tf.matrix()(2,0)= roteMat(2,0);
		this->tf.matrix()(2,1)= roteMat(2,1);
		this->tf.matrix()(2,2)= roteMat(2,2);

		std::cout<< "initial transform of cropBox is"<<std::endl;
		std::cout<< this->tf.matrix() << std::endl;
		
		// set coeff
		coeff.values.resize(10);
		coeff.values[0] = 0; // x
		coeff.values[1] = 0; // y
		coeff.values[2] = 0; // z of inner translate
		coeff.values[3] = 0; // w
		coeff.values[4] = 0; // i
		coeff.values[5] = 0; // j
		coeff.values[6] = 0; // k of inner rotation (quaternion)
		coeff.values[7] = size.x();
		coeff.values[8] = size.y();
		coeff.values[9] = size.z(); // size

		// set filter param
		this->setMax( Eigen::Vector4f( size.x()/2.0,  size.y()/2.0,  size.z()/2.0, 1.0f ) );
		this->setMin( Eigen::Vector4f(-size.x()/2.0, -size.y()/2.0, -size.z()/2.0, 1.0f ) );
		this->setTranslation(translation);	
		this->setRotation(roteVec);
	}



	/*
	template<class T_point>
	void imaiUtil::CoordinatedCropBoxClass<T_point>::setCropBoxParam_fromCoeff()
	{
		//	pcl::ModelCoefficients box;
		//	box.values[0] = 0  ; box.values[1] = 0  ; box.values[2] = 0  ;						// translate
		//	box.values[3] = 0  ; box.values[4] = 0  ; box.values[5] = 0  ; box.values[6] = 0;	// rotation (quaternion)
		//	box.values[7] = 0.2; box.values[8] = 0.2; box.values[9] = 0.2;						// size

		// set filter param
		this->setMax(  
			this->coeff.values[7]/2.0,
			this->coeff.values[8]/2.0,
			this->coeff.values[9]/2.0, 1.0f );
		this->setMin( 
			-this->coeff.values[7]/2.0,
			-this->coeff.values[8]/2.0,
			-this->coeff.values[9]/2.0, 1.0f );
		this->setTranslation(
			Eigen::Vector3f( 
				this->coeff.values[0], 
				this->coeff.values[1],
				this->coeff.values[2], 
				)
			);
		//param[in] rotation the (rx,ry,rz) values that the box should be rotated by
		Eigen::Quaternionf quaternion(
			box.values[3],
			box.values[4],
			box.values[5],
			box.values[6],
			);

		Eigen::Vector3f roteVec = 
		this->setRotation(roteVec);

		// set coeff
		//Eigen::Vector4f quaternion(roteVec);
		coeff.values.resize(10);
		coeff.values[0] = translation.x; 
		coeff.values[1] = translation.y;
		coeff.values[2] = translation.z; // translate
		coeff.values[3] = quaternion.w;
		coeff.values[4] = quaternion.x;
		coeff.values[5] = quaternion.y;
		coeff.values[6] = quaternion.z;	// rotation (quaternion)
		coeff.values[7] = size.x;
		coeff.values[8] = size.y;
		coeff.values[9] = size.z;// size

		// set transform 
		this->tf.pretranslate(translation);
		this->tf.prerotate(roteVec);
	}
	*/

	template<class T_point>
	void imaiUtil::CoordinatedCropBoxClass<T_point>::updateCoeff_byTf()
	{
		// set filter param	
		//this->setTransform(this->tf);
		this->setTranslation(this->tf.translation());
		Eigen::AngleAxisf aa(this->tf.rotation()); 
		this->setRotation( aa.angle()*aa.axis() );
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
		this->updateCoeff_byTf();
		//this->int_viewer->removeShape(this->str_id);
		//this->int_viewer->addCube( this->coeff, this->str_id, *this->int_viewPort);
		this->int_viewer->updateShapePose(this->str_id, this->tf);
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