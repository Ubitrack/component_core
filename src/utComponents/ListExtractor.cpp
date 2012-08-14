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
 * List extractor component
 * This class extracts the next measurement from a list of measurements after receiving a pull event on its output.
 *
 * @author Peter Keitler <keitler@in.tum.de>
 */

#include <boost/bind.hpp>
#include <log4cpp/Category.hh>

#include <utDataflow/PullConsumer.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.ListExtractor" ) );

namespace Ubitrack { namespace Components {

using namespace Dataflow;

/**
 * @ingroup dataflow_components
 * List extractor component
 * This class extracts the next measurement from a list of measurements after receiving a pull event on its output.
 *
 * @par Input Ports
 * PullConsumer<EventType> port with name "Input".
 *
 * @par Output Ports
 * PullSupplier<EventType> port with name "Output".
 *
 * @par Operation
 * Whenever a pull event occurs on this component's output, the next measurement is retrieved from the lists on the input.
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 * - Ubitrack::Measurement::PositionList : 3DPointCloudExtractor
 * - Ubitrack::Measurement::PositionList2 : 2DPointCloudExtractor
 */		

template< class EventType > class ListExtractor : public Component
{
public:
	/**
	 * Standard component constructor.
	 *
	 * @param nm Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	ListExtractor( const std::string& nm, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
        : Ubitrack::Dataflow::Component( nm )
		, m_inCount( 0 )
		, m_listPort( "Coordinates", *this )
		, m_nextEventPort( "NextCoordinate", *this, boost::bind( &ListExtractor::getNextEvent, this, _1 ) )
	{}

protected:

	EventType getNextEvent( Measurement::Timestamp t )
	{
		Measurement::Measurement< std::vector< typename EventType::value_type > > list = m_listPort.get( t );

		LOG4CPP_DEBUG( logger, getName() << " Current counter: " << m_inCount << ", wrap around at: " << list->size() );

		// Wrap around
		if ( m_inCount == list->size() )
			m_inCount = 0;
			
		m_inCount++;
		return EventType( t, list->at( m_inCount - 1 ) );
	}
	
	/** Properties of the gate */
	unsigned int m_inCount;

	/** Ports of the component */
	PullConsumer< Measurement::Measurement< typename std::vector< typename EventType::value_type > > > m_listPort;
	PullSupplier< EventType > m_nextEventPort;
};

UBITRACK_REGISTER_COMPONENT( ComponentFactory* const cf ) 
{
	cf->registerComponent< ListExtractor< Measurement::Position > > ( "3DPointCloudExtractor" );
	cf->registerComponent< ListExtractor< Measurement::Position2D > > ( "2DPointCloudExtractor" );
	cf->registerComponent< ListExtractor< Measurement::Pose > > ( "PoseCloudExtractor" );
}

} } // namespace Ubitrack::Components

