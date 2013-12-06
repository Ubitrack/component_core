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
 * Absolute orientation using RANSAC component.
 * This file contains a component to compute the Absolute Orientation problem using RANSAC.
 *
 * @author Christian Waechter <christian.waechter@in.tum.de>
 */
 
#include <vector> 
#include <math.h> // wegen Optimization.h
#include <log4cpp/Category.hh>

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.AbsoluteOrientationRANSAC" ) );


#include <utMath/Matrix.h>
#include <utMath/Ransac.h>
#include <utMath/Optimization.h>

#include <utDataflow/TriggerComponent.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utCalibration/AbsoluteOrientation.h>


namespace Ubitrack { namespace Components {

using namespace Ubitrack::Math;


/**
 * @ingroup dataflow_components
 * Absolute orientation component.
 * This class contains a component to compute the Absolute Orientation (3D-3D pose estimation) problem.
 *
 * @par Input Ports
 * ExpansionInPort<Position> with name "InputA".
 * ExpansionInPort<Position> with name "InputB".
 *
 * @par Output Ports
 * TriggerOutPort<Pose> with name "Output".
 *
 * @par Configuration
 * DataflowConfiguration: expansion="space" or "time" for time/space expansion
 *
 * @par Operation
 * The component computes the transformation from a coordinate system A to a coordinate system B,
 * given corresponding points in A (InputA) and B (InputB). For details see
 * \c Ubitrack::Calibration::calculateAbsoluteOrientation.
 */
 
 
class RANSACAbsoluteOrientationComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	RANSACAbsoluteOrientationComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Dataflow::TriggerComponent( sName, subgraph )
		, m_inPortA( "InputA", *this )
		, m_inPortB( "InputB", *this )
		, m_outPort( "Output", *this )
		, m_threshold( 0.0 )
		, m_nSetSize( 3 )
		, m_nMinInliers( 3 )
		, m_nMinRuns( 1 )
		, m_nMaxRuns( 100 )
    {
		subgraph->m_DataflowAttributes.getAttributeData( "threshold", m_threshold );
		subgraph->m_DataflowAttributes.getAttributeData( "setSize", m_nSetSize );
		subgraph->m_DataflowAttributes.getAttributeData( "minInliers", m_nMinInliers );
		subgraph->m_DataflowAttributes.getAttributeData( "minRuns", m_nMinRuns );
		subgraph->m_DataflowAttributes.getAttributeData( "maxRuns", m_nMaxRuns );
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		if ( m_inPortA.get()->size() != m_inPortB.get()->size() || m_inPortA.get()->size() < 3 )
			UBITRACK_THROW( "Illegal number of correspondences" );

		boost::shared_ptr< Math::Pose > p( new Math::Pose() );
		unsigned number = 
			Ransac( *p
			, *m_inPortA.get(), *m_inPortB.get()
			, m_threshold,  m_nSetSize, m_nMinInliers
			, m_nMinRuns, m_nMaxRuns
			, Calibration::EstimateAbsoluteOrientation< double >()
			, Calibration::EvaluateAbsoluteOrientation< double >() );

		LOG4CPP_INFO( logger, "Robust absolute orientation performed with " << number << " iterations" );

		m_outPort.send( Measurement::Pose( t, p ) );
    }

protected:

	/** Input port A of the component. */
	Dataflow::ExpansionInPort< Math::Vector< double, 3 > > m_inPortA;

	/** Input port B of the component. */
	Dataflow::ExpansionInPort< Math::Vector< double, 3 > > m_inPortB;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Pose > m_outPort;
	
	/** Output port of the component. */
	double m_threshold;
	
	/** Output port of the component. */
	unsigned m_nSetSize;
	
	/** Output port of the component. */
	unsigned m_nMinInliers;
	
	/** Output port of the component. */
	unsigned m_nMinRuns;
	
	/** Output port of the component. */
	unsigned m_nMaxRuns;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< RANSACAbsoluteOrientationComponent > ( "AbsoluteOrientationRANSAC" );
}

} } // namespace Ubitrack::Components
