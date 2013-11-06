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
 * ring buffer component.
 * This class accumulates single (time-expanded) measurements into a (space-expanded) list of measurements.
 * The aggregation is performed using a ring buffer structure
 * 
 * @author Christian Waechter <christian.waechter@in.tum.de>
 */

#include <vector>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>


static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.RingBuffer" ) );

namespace Ubitrack { namespace Components {

template< class EventType >
class RingBuffer
	: public Dataflow::TriggerComponent
{

public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	RingBuffer( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > cfg )
		: Dataflow::TriggerComponent( sName, cfg )
		, m_inPort( "Input", *this )
		, m_outPort( "Output", *this )
		, m_size( 0 ) // number of elements
		, m_position_buffer( 0 )
	{
		m_list.clear();
		if( cfg->m_DataflowAttributes.hasAttribute( "size" ) )
		{
			cfg->m_DataflowAttributes.getAttributeData( "size", m_size );
			m_list.reserve( m_size );
			LOG4CPP_TRACE( logger, "desired list size: " << m_size );
		}
	}

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		
		if( m_list.size() < m_list.capacity() )
		{
			m_list.push_back( *m_inPort.get() );
			if( m_list.size() < m_list.capacity())
			{
				LOG4CPP_TRACE( logger, "Ring Buffer not yet full, reached " << m_list.size() << " of " << m_list.capacity() << " measurements." );
				UBITRACK_THROW( "Ring buffer not full. need to add more measurements." );
				return;
			}
			m_outPort.send( Measurement::Measurement< typename std::vector< typename EventType::value_type > >( t, m_list ) );
			return;
		}
		
		m_list[ m_position_buffer++ ] = *m_inPort.get();
		m_position_buffer = m_position_buffer % m_size;
 		m_outPort.send( Measurement::Measurement< typename std::vector< typename EventType::value_type > >( t, m_list ) );
	}

	
protected:
	/** Input port of the component. */
	Dataflow::TriggerInPort< EventType > m_inPort;
	
	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Measurement< typename std::vector< typename EventType::value_type > > > m_outPort;
	
	/** size of list to aggregate */
	std::size_t m_size;
	
	/** points to next element to override */
	std::size_t m_position_buffer;

	/** ring buffer */
	std::vector< typename EventType::value_type > m_list;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) 
{
	cf->registerComponent< RingBuffer< Measurement::Pose > > ( "PoseRingBuffer" );
	cf->registerComponent< RingBuffer< Measurement::Position > > ( "PositionRingBuffer" );
	cf->registerComponent< RingBuffer< Measurement::Distance > > ( "DistanceRingBuffer" );
}

} } // namespace Ubitrack::Components

