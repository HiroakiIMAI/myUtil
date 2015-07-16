#ifndef __COORDINATED_OBJECT_CLASS_HPP__
#define __COORDINATED_OBJECT_CLASS_HPP__

#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gl
{
#include <GL/freeglut.h>
};

class ChainedObjectBaseClass
	{
	public:
		Eigen::Affine3f tf;	// transform with respect to local origin(pairent or base) to self 
		ChainedObjectBaseClass* parent;
		std::vector<ChainedObjectBaseClass*> chirdren;
	
	public:
		ChainedObjectBaseClass():
			parent(NULL)
		{
			this->tf.matrix() = Eigen::Matrix4f::Identity();
		}

		Eigen::Affine3f getTf_Base2Self()
		{
			if( this->parent == NULL )
				return this->tf;
			else
				return parent->getTf_Base2Self() * this->tf;
		}

#ifdef __GL_H__
		void draw_treeDiagram()
		{
			Eigen::Affine3f tf_b2self = this->getTf_Base2Self();
			/*
			if(this->parent == NULL)
			{
				gl::glBegin( GL_LINES );
				gl::glColor3d(1.0 ,0.5 ,0.5);

				gl::glVertex3f(0,0,0);

				gl::glVertex3f( 
					tf_b2self.translation().x(),
					tf_b2self.translation().y(),
					tf_b2self.translation().z() );
			
				gl::glEnd();
			}
			else
			{
				gl::glBegin( GL_LINES );
				gl::glColor3d(1.0 ,0.5 ,0.5);

				Eigen::Affine3f tf_b2parent = this->parent->getTf_Base2Self();
				gl::glVertex3f(
				tf_b2parent.translation().x(),
				tf_b2parent.translation().y(),
				tf_b2parent.translation().z() );

				gl::glVertex3f( 
					tf_b2self.translation().x(),
					tf_b2self.translation().y(),
					tf_b2self.translation().z() );
			
				gl::glEnd();
			}
			*/

			Eigen::AngleAxisf aa_self(tf_b2self.rotation());

			////////////////////////////////////////////////
			//
			gl::glPushMatrix();
			
			gl::glTranslated( tf_b2self.translation().x(), tf_b2self.translation().y(), tf_b2self.translation().z() );
			gl::glRotated(aa_self.angle() * 180.0 / M_PI , aa_self.axis().x(), aa_self.axis().y(), aa_self.axis().z());
			//std::cout << aa_self.angle()<<std::endl;

			// point
			gl::glPointSize( 10 );// [pixel]
			gl::glBegin(GL_POINTS);
			gl::glColor3d(1.0, 0, 0);

			gl::glVertex3f(0,0,0);
			gl::glEnd();

			// line from pairent to self
			if(this->parent == NULL)
			{
				Eigen::Affine3f tf_b2self_inv( this->tf.inverse() );
				gl::glBegin( GL_LINES );
				gl::glColor3d(1.0 ,0.5 ,0.5);

				gl::glVertex3f(0,0,0);

				gl::glVertex3f( 
					tf_b2self_inv.translation().x(),
					tf_b2self_inv.translation().y(),
					tf_b2self_inv.translation().z() );
			
				gl::glEnd();
			}
			else
			{
				gl::glBegin( GL_LINES );
				gl::glColor3d(1.0 ,0.5 ,0.5);

				Eigen::Affine3f tf_self2parent = this->tf.inverse(); 
				gl::glVertex3f(
				tf_self2parent.translation().x(),
				tf_self2parent.translation().y(),
				tf_self2parent.translation().z() );
				gl::glVertex3f( 0, 0, 0);
			
				gl::glEnd();
			}

			gl::glPopMatrix();
			//
			////////////////////////////////////////////////
		}

		void draw_axes()
		{
			Eigen::Affine3f tf_b2self = this->getTf_Base2Self();
			Eigen::AngleAxisf aa_self(tf_b2self.rotation());

			////////////////////////////////////////////////
			//

			// set self matrix
			gl::glPushMatrix();
			
			gl::glTranslated( tf_b2self.translation().x(), tf_b2self.translation().y(), tf_b2self.translation().z() );
			gl::glRotated(aa_self.angle() * 180.0 / M_PI , aa_self.axis().x(), aa_self.axis().y(), aa_self.axis().z());

			gl::glLineWidth(3);

			// point
			gl::glPointSize( 10 );// [pixel]
			gl::glBegin(GL_POINTS);
			gl::glColor3d(1.0, 1.0, 1.0);

			gl::glVertex3f(0,0,0);
			gl::glEnd();

			// draw axes
			gl::glBegin( GL_LINES );

			// red line as x axis
			gl::glColor3d(1.0 ,0.0 ,0.0);
			gl::glVertex3f(0,0,0);
			gl::glVertex3f(0.1, 0, 0);
			
			// green line as x axis
			gl::glColor3d(0.0 ,1.0 ,0.0);
			gl::glVertex3f(0,0,0);
			gl::glVertex3f(0, 0.1, 0);
			
			// blue line as x axis
			gl::glColor3d(0.0 ,0.0 ,1.0);
			gl::glVertex3f(0,0,0);
			gl::glVertex3f(0, 0, 0.1);
			
			gl::glEnd();

			gl::glLineWidth(1);

			gl::glPopMatrix();
			//
			////////////////////////////////////////////////
		}

#endif
};

#endif