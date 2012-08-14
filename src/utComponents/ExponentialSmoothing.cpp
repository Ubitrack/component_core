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
 * Exponential Smoothing component.
 * This class calculates a moving average of the given measurements by exponential smoothing.
 *
 * @author Christian Waechter <christian.waechter@in.tum.de>
 */
 
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>


// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.ExponentialSmoothing" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Exponential Smoothing component.
 * This class contains the smoothing of a measurement implemented as a \c TriggerComponent.
 *
 */
template< class EventType >
class ExponentialSmoothingComponent
	: public Dataflow::TriggerComponent
{
public:

	ExponentialSmoothingComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_alpha( .5 )
		, m_init ( false )
		, m_inPort( "Input", *this )
		, m_outPort( "Output", *this )
   {
		pConfig -> m_DataflowAttributes.getAttributeData( "alpha", m_alpha );
		// schlegem: should work without that 
		// m_mean = EventType::value_type();
   }
   
   /** Method that computes the result for the smoothing. */
	void compute( Measurement::Timestamp t )
	{
		if( !m_init ){
			m_mean = *( m_inPort.get() );
			m_init = true;
		}
		else
			m_mean = (  m_alpha * ( *( m_inPort.get() ) )  ) + ( ( 1. - m_alpha ) * m_mean );
			
		LOG4CPP_TRACE( logger, "exponential smoothing: " << m_mean );
		m_outPort.send( EventType( t, m_mean ) );
	}

protected:

	/** smoothing factor */
	double m_alpha;

	/** flag if first measurement was received */
	bool m_init;

	/** mean at last timestep */
	typename EventType::value_type m_mean;
   
	/** Input ports of the component. */
	Dataflow::TriggerInPort< EventType > m_inPort;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< EventType > m_outPort;
	

};

/** addition operator for adding two poses */
Math::Pose operator+( const Math::Pose& pose1, const Math::Pose& pose2 )
{
	Math::Pose result = Math::Pose ( pose1.rotation() + pose2.rotation(),  pose1.translation() + pose2.translation() );;
	return result;
};

/** multiplication operator for scaling a pose by a scalar value */
Math::Pose operator*( const double& alpha, const Math::Pose &pose )
{
	return Math::Pose ( alpha * pose.rotation(),  alpha * pose.translation() );
}

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf )
{
	cf->registerComponent< ExponentialSmoothingComponent< Measurement::Distance > > ( "ExponentialSmoothingDistance" );
	cf->registerComponent< ExponentialSmoothingComponent< Measurement::Position2D > > ( "ExponentialSmoothingPosition2D" );
	cf->registerComponent< ExponentialSmoothingComponent< Measurement::Position > > ( "ExponentialSmoothingPosition" );
	cf->registerComponent< ExponentialSmoothingComponent< Measurement::Rotation > > ( "ExponentialSmoothingRotation" );
	cf->registerComponent< ExponentialSmoothingComponent< Measurement::Pose > > ( "ExponentialSmoothingPose" );
}

} } // namespace Ubitrack::Components
