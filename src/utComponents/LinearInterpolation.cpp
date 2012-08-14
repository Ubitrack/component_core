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
 * Linear interpolating component
 * This file contains a linear interpolating push-pull
 * adapter component.
 * The component accepts events via an push input port
 * and answers queries for events by linear interpolating
 * between the last two received events.
 *
 * The calculation is done by calling the linearInterpolate
 * method on the corresponding mathematical objects.
 *
 * @author Manuel Huber <huberma@in.tum.de>
 */

#include <string>
#include <iostream>
#include <deque>

#include <boost/bind.hpp>
#include <log4cpp/Category.hh>

#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

// get a logger
static log4cpp::Category& eventsLogger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.LinearInterpolation" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Linear interpolating component
 * This class contains a linear interpolating push-pull
 * adapter component.
 * The component accepts events via an push input port
 * and answers queries for events by linear interpolating
 * between the last two received events.
 *
 * The calculation is done by calling the linearInterpolate
 * method on the corresponding mathematical objects.
 *
 * @par Input Ports
 * PushConsumer<EventType> port with name "AB".
 *
 * @par Output Ports
 * PullSupplier<EventType> port with name "AB-Interpolated".
 *
 * @par Configuration
 * The "maxAge" (ms) dataflow attribute limits the amount of
 * extrapolation.
 *
 * @par Operation
 * Whenever an event is requested via the pull output port
 * the component either interpolates or extrapolates
 * from the last two events received via the input port
 * using linear interpolation.
 *
 * If the time difference is larger than some configured value
 * or of less than two events have been received so far,
 * the component cannot deliver.
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 * - Ubitrack::Measurement::Pose : PoseLinearInterpolation
 */
template< class EventType > class LinearInterpolation
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param nm Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	LinearInterpolation( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Dataflow::Component( sName )
		, m_inPort( "AB", *this, boost::bind( &LinearInterpolation::eventIn, this, _1 ) )
		, m_outPort( "AB-Interpolated", *this,boost::bind( &LinearInterpolation< EventType >::eventOut, this, _1 ) )
		, m_ringBufferSize( 50 )
		, m_timeout( 0 )
	{
		unsigned long timeout = 0;
		subgraph->m_DataflowAttributes.getAttributeData( "maxAge", timeout );
		timeout *= 1000000;
		m_timeout = timeout;
	}

protected:

	/**
	 * Handler method for input port
	 * Receives an event and stores it.
	 * The last two received events are kept.
	 * @param m the received event
	 */
    void eventIn( const EventType& m )
    {
		LOG4CPP_DEBUG( eventsLogger, getName() << " received push event with timestamp " << m.time() );

		m_ringBuffer.push_back( m );
		if ( m_ringBuffer.size() > m_ringBufferSize )
			m_ringBuffer.pop_front();
	}

	/**
	 * Handler method for the output port.
	 * Sends the last received event as the requested one.
	 * @param t the Timestamp of the requested event.
	 * @return the requested event.
	 * @throws Ubitrack::Util::Exception if not enough events are buffered or the timestamp is out of range
	 */
    EventType eventOut( Ubitrack::Measurement::Timestamp t )
    {
        if ( m_ringBuffer.size() < 2 )
        {
            // the linear interpolation requires at least 2 previous events
			LOG4CPP_NOTICE( eventsLogger, getName() << " has not enough data" );
            UBITRACK_THROW( "not enough data to start interpolation" );
        }

		// search for the two closest events in ring buffer
		typename std::deque< EventType >::iterator it2 = m_ringBuffer.end();
		it2--;
		typename std::deque< EventType >::iterator it1 = it2;
		it1--;

		while ( it1 != m_ringBuffer.begin() && it1->time() > t )
			{ it1--; it2--; }

		// compute event differences
		long long int eventDifference = it2->time() - it1->time();
		long long int timeDiff = t - it1->time();

		// check for timeout
		if ( m_timeout && ( timeDiff > m_timeout || -timeDiff > m_timeout ) )
		{
			// the time difference is too large so the t lies out of range
			LOG4CPP_NOTICE( eventsLogger, getName() << ": data too old, timeout is: " << m_timeout / 1000000 << ", measurement age: " << timeDiff / 1000000 << ", requested for " <<  Measurement::timestampToShortString( t ) );
			UBITRACK_THROW( "data is too old to do extrapolation" );
		}

		double h;
		if ( eventDifference )
			h = double( timeDiff ) / double( eventDifference );
		else
			h = 1.0;

		LOG4CPP_TRACE( eventsLogger, "prev: " << *it1 );
		LOG4CPP_TRACE( eventsLogger, "current: " << *it2 );
		LOG4CPP_TRACE( eventsLogger, "h: " << h );

        return EventType ( t, linearInterpolate( **it1, **it2, h ) );
    }

	/** Input port of the component. */
	Dataflow::PushConsumer< EventType > m_inPort;
	/** Output port of the component. */
	Dataflow::PullSupplier< EventType > m_outPort;

	/** the last n received events */
	std::deque< EventType > m_ringBuffer;

	/** (maximum) size of the ring buffer */
	unsigned m_ringBufferSize;

	/** Configuration value for timeout parameter */
	long long int m_timeout;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< LinearInterpolation< Measurement::Pose > > ( "PoseLinearInterpolation" );
	cf->registerComponent< LinearInterpolation< Measurement::Rotation > > ( "RotationLinearInterpolation" );
	cf->registerComponent< LinearInterpolation< Measurement::Position > > ( "PositionLinearInterpolation" );
	cf->registerComponent< LinearInterpolation< Measurement::ErrorPose > > ( "ErrorPoseLinearInterpolation" );
}

} } // namespace Ubitrack::Components
