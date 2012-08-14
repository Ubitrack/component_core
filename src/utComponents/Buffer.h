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
 * Buffer component
 * This file contains a buffer component which is the
 * most simple push-pull adapter.
 * The component accepts an event via a push input port
 * and sends the last received event for any request via
 * the pull output port.
 *
 * This may be useful for static spatial relationships which are
 * can be calibrated at runtime.
 *
 * @author Manuel Huber <huberma@in.tum.de>
 */

#include <string>
#include <iostream>

#include <log4cpp/Category.hh>
#include <boost/bind.hpp>

#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Buffer component
 * This class contains a buffer component which is the
 * most simple push-pull adapter.
 * The component accepts an event via a push input port
 * and sends the last received event for any request via
 * the pull output port.
 *
 * This may be useful for static spatial relationships which are
 * can be calibrated at runtime.
 *
 * @par Input Ports
 * PushConsumer<EventType> port with name "Input".
 *
 * @par Output Ports
 * PullSupplier<EventType> port with name "Output".
 *
 * @par Configuration
 * @verbatim <Attribute name="maxAge" value="..."/>@endverbatim
 * where \c maxAge is the maximum age of measurements in ms
 *
 * @par Operation
 * Whenever an event is received via the input port it is
 * buffered in an internal member variable.
 * Whenever an event is requested via the output port,
 * the last received event is replayed with an adapted
 * timestamp. If no event has been received so far
 * the output port cannot deliver.
 */
template< class EventType >
class Buffer
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param nm Unique name of the component.
	 * @param cfg UTQL subgraph
	 */
	Buffer( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
      : Dataflow::Component( sName )
      , m_inPort( "Input", *this, boost::bind( &Buffer< EventType >::eventIn, this, _1 ) )
      , m_outPort( "Output", *this,boost::bind( &Buffer< EventType >::eventOut, this, _1 ) )
      , m_eventCounter( 0 )
	  , m_maxAge( 0 )
	  , m_eventsLogger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.Buffer" ) )
    {
		unsigned long ms = 0;
		subgraph->m_DataflowAttributes.getAttributeData( "maxAge", ms );
		ms *= 1000000;
		m_maxAge = ms;
	}

protected:
	/**
	 * Handler method for input port
	 * Receives an event and stores it.
	 * @param m the received event
	 */
    void eventIn( const EventType& m )
    {
        m_event = m;
        if ( m_eventCounter < 1 )
            m_eventCounter++;
    }

	/**
	 * Handler method for the output port.
	 * Sends the last received event as the requested one.
	 * @param t the Timestamp of the requested event.
	 * @return the requested event.
	 * @throws Ubitrack::Util::Exception if buffer is still empty
	 */
    EventType eventOut( Ubitrack::Measurement::Timestamp t )
    {
        if ( m_eventCounter < 1 )
        {
			LOG4CPP_DEBUG( m_eventsLogger, getName() << " not enough data in buffer" );
            UBITRACK_THROW( "not enough data in buffer" );
        }
		if ( m_maxAge && m_event.time() + m_maxAge < t )
		{
			LOG4CPP_DEBUG( m_eventsLogger, getName() << " buffered measurement too old" );
			UBITRACK_THROW( "Buffered measurement too old" );
		}
        return EventType( t, m_event );
    }

	/** Input port of the component. */
	Dataflow::PushConsumer< EventType > m_inPort;
	/** Output port of the component. */
	Dataflow::PullSupplier< EventType > m_outPort;

	/** Buffer variable containing the last received event. */
	EventType m_event;

	/** Counter to check if an event was already received. */
    int m_eventCounter;

	/** maximum age of events in ns */
	Measurement::Timestamp m_maxAge;

	/** logger */
	log4cpp::Category& m_eventsLogger; 
};

typedef Buffer< Measurement::Pose > PoseBuffer;
typedef Buffer< Measurement::ErrorPose > ErrorPoseBuffer;
typedef Buffer< Measurement::ErrorPosition > ErrorPositionBuffer;
typedef Buffer< Measurement::Rotation > RotationBuffer;
typedef Buffer< Measurement::Position > PositionBuffer;
typedef Buffer< Measurement::Position2D > Position2Buffer;
typedef Buffer< Measurement::PositionList > PositionListBuffer;
typedef Buffer< Measurement::PositionList2 > PositionList2Buffer;
typedef Buffer< Measurement::RotationVelocity > RotationVelocityBuffer;
typedef Buffer< Measurement::Button > SkalarBuffer;

} } // namespace Ubitrack::Components
