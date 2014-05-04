/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2013, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */


/**
 * @ingroup dataflow_components
 * @file
 * Projection component.
 * This file contains a projectin component which computes a 3x4 projection matrix for OST-HMDs 
 * given an eye position and a previous intrinsic matrix.
 * More detail can be found in 
 * @article{itoh2014-3dui
 *  author    = {Itoh, Yuta and
 *               Klinker, Gudrun},
 *  title     = {Interaction-Free Calibration for Optical See-Through Head-Mounted Displays based on 3D Eye Localization},
 *  booktitle = {{Proceedings of the 9th IEEE Symposium on 3D User Interfaces (3D UI)}},
 *  month     = {march},
 *  pages     = {xxx-xxx},
 *  year      = {2014}
 * }
 * @author Yuta Itoh <yuta.itoh@in.tum.de>
 * @date 2013
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
namespace ublas = boost::numeric::ublas;

namespace Ubitrack { namespace Components {


/**
 * @ingroup dataflow_components
 * Multiplication component.
 * This class contains a multiplication of two inputs implemented as a \c TriggerComponent.
 *
 * The component multiplies requested/incoming events using operator*( A, B )
 */
class ProjectionWithEyePositionRecycle
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	ProjectionWithEyePositionRecycle( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPortIntrinsicE( "InputIntrinsicsEye", *this )
		, m_inPortPoseWT(     "InputPoseWorld2EyeTracker", *this )
		, m_inPortRotationWE0( "InputRotationWorld2EyeOld", *this )
		, m_inPortPositionWE0( "InputPositionWorld2EyeOld", *this )
		, m_inPortPositionET( "InputPositionEye2EyeTracker", *this )
		, m_outPort( "OutputProjection", *this )
		, t_WS_z(1.0)
	{
		pConfig->m_DataflowAttributes.getAttributeData( "t_WS_z", t_WS_z );
	}

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		/// ///////////////////
		/// Prepare poses
		/// ///////////////////

		/// Intrinsic parameter & pose from the wolrd to the old eye pose(display) 
		const Ubitrack::Math::Matrix3x3d  &K_E0 = m_inPortIntrinsicE.get()->content();
		Ubitrack::Math::Matrix3x3d  R_WE;
		m_inPortRotationWE0.get()->toMatrix(R_WE);
		const Ubitrack::Math::Vector3d &t_WE0 = m_inPortPositionWE0.get()->content();
		
		/// Translation from the eye to the eye tracker: t_ET
		const Ubitrack::Math::Vector3d &t_ET = m_inPortPositionET.get()->content();
		
		/// Translation from the eye tracker to the world: (R_WT,t_WT)
		const Measurement::Pose &P_WT = m_inPortPoseWT.get();
		Ubitrack::Math::Matrix3x3d  R_WT;
		P_WT->rotation().toMatrix(R_WT); /// This operation accumrates numerical error about 0.5 %
		const Ubitrack::Math::Vector3d &t_WT = P_WT->translation();
		
		///  -R_WS*(-R_WT'*t_WT+ R_WT'*t_ET);
		/// =-R_WS*R_WT'*(t_ET-t_WT);
		const Ubitrack::Math::Matrix3x3d &R_WS = R_WE;
		const Ubitrack::Math::Vector3d &t_ET_min_t_WT = t_ET-t_WT;
		const Ubitrack::Math::Vector3d &tmpVec = ublas::prod( ublas::trans(R_WT), t_ET_min_t_WT);
		const Ubitrack::Math::Vector3d t_WE = - ublas::prod( R_WS, tmpVec ); 


		/// ///////////////////
		/// Prepare necessary variables
		/// ///////////////////
		/*
		t_WE = -R_WS*(-R_WT'*t_WT+ R_WT'*t_ET);
		t_SE0_z =(t_WE0(3)- t_WS_z );
		t_E0E = t_WE-t_WE0;% eye2 position
		K2=[ (t_SE0_z+t_E0E(3))  0    -t_E0E(1)
				0  (t_SE0_z+t_E0E(3)) -t_E0E(2)
				0           0          t_SE0_z ];
		*/
		const double t_SE0_z =t_WE0(2)-t_WS_z;
		const Ubitrack::Math::Vector3d t_E0E = t_WE-t_WE0;

// K2=[ (t_SE0_z+t_E0E(3))  0    -t_E0E(1)
//          0 (t_SE0_z+t_E0E(3)) -t_E0E(2)
//          0          0          t_SE0_z ];
// K2=K2/t_SE0_z;
		Ubitrack::Math::Matrix3x3d S;
		const double t_SE0_z_inv = 1.0/t_SE0_z;
		const double tmp = ( t_SE0_z+t_E0E(2) )*t_SE0_z_inv;
		S(0,0) = tmp; S(0,1) = 0.0; S(0,2) = -t_E0E[0]*t_SE0_z_inv;
		S(1,0) = 0.0; S(1,1) = tmp; S(1,2) = -t_E0E[1]*t_SE0_z_inv;
		S(2,0) = 0.0; S(2,1) = 0.0; S(2,2) = 1.0;
		
		/// ///////////////////
		/// Calculate the projection matrix
		/// ///////////////////
		/// P_WE = K_E0*K2*[R_WS t_WE];
		Ubitrack::Math::Matrix3x4d P;
		const Ubitrack::Math::Matrix3x3d KS = ublas::prod( K_E0, S );
		/// ublas::subrange( P, start_x, x_num, start_y, y_num )
		ublas::subrange( P, 0,3, 0,3 )     = ublas::prod( KS, R_WE );
		Ubitrack::Math::Vector3d ttmp = ublas::prod( KS, t_WE);
		P(0,3) = ttmp[0];
		P(1,3) = ttmp[1];
		P(2,3) = ttmp[2];

#if 0
		std::cout.precision(15);
		std::cout<< "tmp " <<  tmp <<std::endl;
		std::cout<< "P_WE " << P <<std::endl;
		std::cout<< "t_ET " << t_ET <<std::endl;
		std::cout<< "t_WT " << t_WT <<std::endl;
		std::cout<< "tmpVec " << tmpVec<<std::endl;
		std::cout<< "R_WE " << R_WE <<std::endl;
		std::cout<< "R_WS " << R_WS <<std::endl;
		std::cout<< "R_WT " << R_WT <<std::endl;
		std::cout<< "t_WE " << t_WE <<std::endl;
		std::cout<< "t_WE0 " << t_WE0 <<std::endl;
		std::cout<< "t_E0E " << t_E0E <<std::endl;
		std::cout<< "t_WT "  << t_WT  <<std::endl;
		std::cout<< "t_SE0_z " << t_SE0_z <<std::endl;
		std::cout<< "K_E0 " << K_E0 <<std::endl;
		std::cout<< "K2 " << S <<std::endl;
		std::cout<< "K_E0*K2 " << KS <<std::endl;
#endif
		// Flip signs if necessary
		if( P(2,3)<0.0 ){
			for( int r = 0; r<3; r++ ){
				for( int c = 0; c<4; c++ ){
					P(r,c) = -P(r,c); 
				}
			}
		}

		m_outPort.send( Measurement::Matrix3x4( t, P ) );

	}

protected:
	/** Input port A of the component. */
	Dataflow::TriggerInPort< Measurement::Matrix3x3 >  m_inPortIntrinsicE;

	/** Input port B of the component. */
	Dataflow::TriggerInPort< Measurement::Pose >       m_inPortPoseWT;
	
	Dataflow::TriggerInPort< Measurement::Rotation >   m_inPortRotationWE0;
	Dataflow::TriggerInPort< Measurement::Position >   m_inPortPositionWE0;

	/** Input port B of the component. */
	Dataflow::TriggerInPort< Measurement::Position >   m_inPortPositionET;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Matrix3x4 > m_outPort;

	double t_WS_z;
};



UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	// Pose * Pose = Pose
	cf->registerComponent< ProjectionWithEyePositionRecycle > ( "ProjectionWithEyePositionRecycle" );

}

} } // namespace Ubitrack::Components
