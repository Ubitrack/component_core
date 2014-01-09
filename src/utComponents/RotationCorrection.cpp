/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
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
 * Component for correction of orientation to be used in indirect 
 * tracking setups (satellite tracking).
 *
 * @author Peter Keitler <keitler@in.tum.de>
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

// get a logger
#include <log4cpp/Category.hh>
static log4cpp::Category& eventsLogger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.RotationCorrection" ) );

namespace Ubitrack { namespace Components {


class RotationCorrectionComponentOrth
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	RotationCorrectionComponentOrth( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_refPort( "RefMeasurement", *this )
		, m_errPort( "ErrMeasurement", *this  )
		, m_rotCorrPort( "OrientationCorrection", *this )
	{
	}

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
	  // Fetch data
	  Measurement::Position refPos = m_refPort.get();
	  Measurement::Position errPos = m_errPort.get();
	  
	  // 1. Calc normalized correction axis
	  Math::Vector< double, 3 > axis = cross_prod( *errPos, *refPos );

	  // 2. Calc correction angle. It always is between 0 and 180°
	  double angle = acos( inner_prod(*errPos, *refPos) / ( norm_2(*errPos) * norm_2(*refPos) ) );

	  LOG4CPP_TRACE( eventsLogger, "axis: " << axis << ", angle: " << angle << ", timestamp: " << t );

	  // 3. Calc orientational correction. The axis will be normed by the constructor
	  Math::Quaternion corrRot( axis, angle );

	  m_rotCorrPort.send( Measurement::Rotation( t, ~corrRot ) );
	}

protected:
	/** Reference measurement of reference point correspondence */
	Dataflow::TriggerInPort< Measurement::Position > m_refPort;

	/** Erroneous measurement of reference point correspondence */
	Dataflow::TriggerInPort< Measurement::Position > m_errPort;

	/** Corrected pose output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Rotation > m_rotCorrPort;
};


class RotationCorrectionComponentFull
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	RotationCorrectionComponentFull( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_refPortA( "RefMeasurementA", *this )
		, m_errPortA( "ErrMeasurementA", *this  )
		, m_refPortB( "RefMeasurementB", *this )
		, m_errPortB( "ErrMeasurementB", *this  )
		, m_rotCorrPort( "OrientationCorrection", *this )
	{
	}

	/** 
	 * Method that computes the result. 
	 * More details about the algorithm cbe found in Horn 86 - Closed-Form Solution of Absolute Orientation Using Unit Quaternions
	 */
	void compute( Measurement::Timestamp t )
	{
	  // Fetch data
	  Math::Vector< double, 3 > refPointA = *(m_refPortA.get());
	  Math::Vector< double, 3 > errPointA = *(m_errPortA.get());
	  Math::Vector< double, 3 > refPointB = *(m_refPortB.get());
	  Math::Vector< double, 3 > errPointB = *(m_errPortB.get());

	  /* 
	   * Compute normal for reference plane defined by center of
	   * gravity (COG) and reference measurements as well as for
	   * erroneous plane defined by COG and erroneous
	   * measurements. Then compute axis/angle for mapping the
	   * erroneous plane to the reference plane
	   */
	  Math::Vector< double, 3 > refAxis = cross_prod( refPointB, refPointA );
	  refAxis /= norm_2( refAxis );
	  Math::Vector< double, 3 > errAxis = cross_prod( errPointB, errPointA );
	  errAxis /= norm_2( errAxis );
	  Math::Vector< double, 3 > planeAxis = cross_prod( errAxis, refAxis );
	  planeAxis /= norm_2( planeAxis );
	  double planeAngle = acos( inner_prod(errAxis, refAxis) / ( norm_2(errAxis) * norm_2(refAxis) ) );
	  LOG4CPP_TRACE( eventsLogger, "plane mapping: axis: " << planeAxis << ", angle: " << planeAngle << ", timestamp: " << t );

	  /*
	   * Step 1: Rotation which maps erroneous plane to reference
	   * plane. Apply transform to both erroneous points since we
	   * need them in the reference plane.
	   */
	  Math::Quaternion planeCorrRot( planeAxis, planeAngle );
	  errPointA = planeCorrRot * errPointA;
	  errPointB = planeCorrRot * errPointB;

	  /*
	   * Now, points have to be mapped in the reference plane by a
	   * rotation about the normal of the reference plane. Horn
	   * treats this as a general least-squares problem. In the
	   * special case of only two points on a unit sphere around a
	   * common origin, the mean of the two rotation angles for
	   * the two point correspondences yields the least-squares
	   * solution! The choice of vectors for computation of the
	   * axis is arbitrary. It is not sufficient to use the plane
	   * normal refAxis from above due to sign ambiguity!
	   */
	  Math::Vector< double, 3 > ptAxis = cross_prod( errPointA, refPointA );
	  ptAxis /= norm_2( ptAxis );
	  double ptAngleA = acos( inner_prod(errPointA, refPointA) / ( norm_2(errPointA) * norm_2(refPointA) ) );
	  double ptAngleB = acos( inner_prod(errPointB, refPointB) / ( norm_2(errPointB) * norm_2(refPointB) ) );
	  double ptAngle = ( ptAngleA + ptAngleB ) / 2;
	  LOG4CPP_TRACE( eventsLogger, "point mapping angle: " << ptAngle << " (resulting from angles A/B: " << ptAngleA << ", " << ptAngleB << ")" );

	  /* 
	   * Step 2: Rotation which maps erroneous points to reference
	   * points (in reference plane)
	   */
	  Math::Quaternion pointCorrRot( ptAxis, ptAngle );

	  /*
	   * Calc correction pose using identity translation. The two
	   * rotational correction steps above are concatenated and
	   * prepended to the initial transformation. 
	   */
	  Math::Quaternion corrRot = (planeCorrRot * pointCorrRot);
	  
	  m_rotCorrPort.send( Measurement::Rotation( t, ~corrRot ) );
	}

protected:
	/** Reference measurement of reference point  correspondence A */
	Dataflow::TriggerInPort< Measurement::Position > m_refPortA;

	/** Erroneous measurement of reference point correspondence A */
	Dataflow::TriggerInPort< Measurement::Position > m_errPortA;

	/** Reference measurement of reference point correspondence B */
	Dataflow::TriggerInPort< Measurement::Position > m_refPortB;

	/** Erroneous measurement of reference point correspondence B */
	Dataflow::TriggerInPort< Measurement::Position > m_errPortB;

	/** Corrected pose output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Rotation > m_rotCorrPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
  cf->registerComponent< RotationCorrectionComponentOrth > ( "RotationCorrectionOrthogonal" );
  cf->registerComponent< RotationCorrectionComponentFull > ( "RotationCorrectionFull" );
}

} } // namespace Ubitrack::Components
