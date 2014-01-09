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
 * This file contains a projectin component which computes a 3x4 projection matrix for OST-HMDs given an eye position .
 *
 * @author Yuta Itoh <yuta.itoh@in.tum.de>
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
class ProjectionWithEyePosition
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	ProjectionWithEyePosition( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPortIntrinsicE( "InputIntrinsicsEye", *this )
		, m_inPortPoseWC( "InputPoseWorld2EyeCam", *this )
		, m_inPortRotationWE( "InputRotationWorld2Eye", *this )
		, m_inPortPositionCE( "InputPositionEyeCam2Eye", *this )
		, m_outPort( "OutputProjection", *this )
	{
	}

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		/// ///////////////////
		/// Prepare poses
		/// ///////////////////

		/// Intrinsic parameter & rotation from the wolrd to the eye(display) 
		const Measurement::Matrix3x3  &K_WE = m_inPortIntrinsicE.get();
		Ubitrack::Math::Matrix3x3d  R_WE;
		m_inPortRotationWE.get()->toMatrix(R_WE);
		
		/// Translation from the eye camera to the eye
		const Measurement::Position  &t_CE =  m_inPortPositionCE.get();
		
		/// Translation from the eye camera to the world
		const Measurement::Pose &P_WC = m_inPortPoseWC.get();
		Ubitrack::Math::Matrix3x3d  R_WC;
		P_WC->rotation().toMatrix(R_WC);
		const Ubitrack::Math::Vector< double, 3 > &t_WC = P_WC->translation();
		/// ///////////////////
		/// Calculate the projection matrix
		/// ///////////////////
		/// t = R_WC * t_CE +t_WC
		/// P = K* [ R_WE t ]
		Ubitrack::Math::Vector< double, 3 > ttmp = ublas::prod( R_WC, (*t_CE.get()) ) + t_WC;

		Ubitrack::Math::Matrix3x4d P;
		ublas::subrange( P, 0,3, 0,3 ) = ublas::prod( (*K_WE.get()), R_WE );
		ttmp = ublas::prod( (*K_WE.get()), ttmp);
		P(0,3) = ttmp[0];
		P(1,3) = ttmp[1];
		P(2,3) = ttmp[2];

		m_outPort.send( Measurement::Matrix3x4( t, P ) );

	}

protected:
	/** Input port A of the component. */
	Dataflow::TriggerInPort< Measurement::Matrix3x3 >  m_inPortIntrinsicE;

	/** Input port B of the component. */
	Dataflow::TriggerInPort< Measurement::Pose >       m_inPortPoseWC;

	/** Input port A of the component. */
	Dataflow::TriggerInPort< Measurement::Rotation >   m_inPortRotationWE;

	/** Input port B of the component. */
	Dataflow::TriggerInPort< Measurement::Position >   m_inPortPositionCE;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Matrix3x4 > m_outPort;
};



UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	// Pose * Pose = Pose
	cf->registerComponent< ProjectionWithEyePosition > ( "ProjectionWithEyePosition" );

}

} } // namespace Ubitrack::Components
