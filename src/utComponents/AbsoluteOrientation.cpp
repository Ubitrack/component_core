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
 * Absolute orientation component.
 * This file contains a component to compute the Absolute Orientation problem.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utCalibration/AbsoluteOrientation.h>

namespace Ubitrack { namespace Components {

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
template< class ResultType >
class AbsoluteOrientationComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	AbsoluteOrientationComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPortA( "InputA", *this )
		, m_inPortB( "InputB", *this )
		, m_outPort( "Output", *this )
    {
		generateSpaceExpansionPorts( pConfig );

		for(int i = 0; i < 6; i++)
			for(int j = 0; j < 6; j++)
				m_covariance(i,j)=0;
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		if ( m_inPortA.get()->size() != m_inPortB.get()->size() )
			UBITRACK_THROW( "Illegal number of correspondences" );
		if ( m_inPortA.get()->size() < 3 )
			UBITRACK_THROW( "Insufficient correspondences" );

		const std::vector< Math::Vector< double, 3 > >& left = *m_inPortA.get();
		const std::vector< Math::Vector< double, 3 > >& right = *m_inPortB.get();	
			
		Math::Pose pose = Calibration::calculateAbsoluteOrientation( left, right );
				
		Calibration::EvaluateAbsoluteOrientation< double > evaluator;
		double m_error_distance = 0;
		for( unsigned int i = 0; i < left.size(); i++ ) {
			m_error_distance += evaluator( pose, left[i], right[i] );
		}
			
		m_error_distance /= left.size(); //avg error
		m_covariance (0,0) = m_covariance (1,1) = m_covariance (2,2) = pow(m_error_distance,2.0) / 3.0; //RMS²/3 in trace

		boost::shared_ptr< Math::ErrorPose > ep( new Math::ErrorPose( pose, m_covariance ) );

		sendResult( Measurement::ErrorPose( t, ep ) );
    }

protected:
	void sendResult( Measurement::ErrorPose ep );
	
	/** Input port A of the component. */
	Dataflow::ExpansionInPort< Math::Vector< double, 3 > > m_inPortA;

	/** Input port B of the component. */
	Dataflow::ExpansionInPort< Math::Vector< double, 3 > > m_inPortB;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< ResultType > m_outPort;

	Math::Matrix< double, 6, 6 > m_covariance;
};


template <>
void AbsoluteOrientationComponent< Measurement::ErrorPose >::sendResult( Measurement::ErrorPose ep )
{
	m_outPort.send( ep );
}

template <>
void AbsoluteOrientationComponent< Measurement::Pose >::sendResult( Measurement::ErrorPose ep )
{
	m_outPort.send( Measurement::Pose( ep.time(), *ep ) );
}


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< AbsoluteOrientationComponent< Measurement::Pose > > ( "AbsoluteOrientation" );
	cf->registerComponent< AbsoluteOrientationComponent< Measurement::ErrorPose > > ( "AbsoluteOrientationCovar" );
}

} } // namespace Ubitrack::Components
