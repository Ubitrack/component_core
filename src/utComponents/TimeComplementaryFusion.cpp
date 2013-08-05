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
 * Time complementary fusion component.
 *
 * @author Manuel Huber <huberma@in.tum.de>
 */
#include <string>
#include <iostream>

#include <boost/bind.hpp>

#include <utDataflow/PushConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Time complementary fusion component.
 *
 * @par Input Ports
 * PushConsumer<EventType> port with name "InputA".
 * PushConsumer<EventType> port with name "InputB".
 *
 * @par Output Ports
 * PushSupplier<EventType> port with name "Output".
 *
 * @par Configuration
 * None.
 *
 * @par Operation
 * TBD
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 * - Ubitrack::Measurement::Pose : PoseTimeComplementaryFusion
 * - Ubitrack::Measurement::Rotation : RotationTimeComplementaryFusion
 */
template< class EventType > class TimeComplementaryFusion
	: public Dataflow::Component
{
public:
	TimeComplementaryFusion( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
      : Dataflow::Component( sName )
	  , m_eventATimestamp ( -1 )
      , m_delayTime( 500 )
      , m_portA( "InputA", *this, boost::bind( &TimeComplementaryFusion::poseInA, this, _1 ) )
      , m_portB( "InputB", *this, boost::bind( &TimeComplementaryFusion::poseInB, this, _1 ) )
      , m_outPort( "Output", *this )
      , m_logger( log4cpp::Category::getInstance( "Ubitrack.Components.Delay" ) )	  
	{
      	std::string tempValue;
		double ms = 0;
		std::string delayAttrName = "delayTime";

		if ( subgraph->m_DataflowAttributes.hasAttribute( delayAttrName  ) ){
			// read value as a string
			tempValue = subgraph->m_DataflowAttributes.getAttributeString( delayAttrName );
			LOG4CPP_DEBUG( m_logger, "Setting delay time (string) " << tempValue );

			// converting the string into a double value
			std::istringstream inStream( tempValue ); 
			inStream >> ms;
			LOG4CPP_DEBUG( m_logger, "Setting delay time (double) " << ms );

			// the input is expected in ms, but we need ns
			ms *= 1000000;

			// finally store the casted value
			m_delayTime = static_cast<long int>(ms);
		}
		
		LOG4CPP_INFO( m_logger, "Setting delay time" << m_delayTime );		
    }

protected:
	// receives events from InputA
    void poseInA( const EventType& m )
    {
	  m_outPort.send( m ); // EventType( m.time(), m.get() ) );
	  m_eventATimestamp = m.time();
    }

	// receives events from InputB
    void poseInB( const EventType& m )
    {
		Measurement::Timestamp timeDiff = m.time() - m_eventATimestamp;

		if ((m_eventATimestamp < 0) || (timeDiff > m_delayTime * 1000000))
			m_outPort.send( m ); // EventType( m.time(), m.get() ) );
    }

	// the last event from each port
	Measurement::Timestamp m_eventATimestamp;

    // holds the delay time 
    long int m_delayTime;

	// the two input ports
	Dataflow::PushConsumer< EventType > m_portA;
	Dataflow::PushConsumer< EventType > m_portB;
	Dataflow::PushSupplier< EventType > m_outPort;
	
    /** log4cpp logger reference */
    log4cpp::Category& m_logger;	
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< TimeComplementaryFusion< Measurement::Pose > > ( "PoseTimeComplementaryFusion" );
	cf->registerComponent< TimeComplementaryFusion< Measurement::ErrorPose > > ( "ErrorPoseTimeComplementaryFusion" );
	cf->registerComponent< TimeComplementaryFusion< Measurement::Rotation > > ( "RotationTimeComplementaryFusion" );
}

} } // namespace Ubitrack::Components
