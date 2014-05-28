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
 * Accumulator component.
 * This class accumulates Position(2) measurements into PositionList(2) mms.
 *
 * Any number of input edges can be used, as long as they all supply the correct
 * measurement type. A dataflow attribute named maxLength specifies the maximum
 * number of list elements. If this amount is reached, the accumulator behaves
 * as a FIFO.
 * 
 * @author Florian Echtler <echtler@in.tum.de>
 */

// Ubitrack
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

// Boost
#include <boost/bind.hpp>

// log4cpp
#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.Accumulator" ) );

namespace Ubitrack { namespace Components {

using namespace Dataflow;

/// @todo the implemented behaviour is similar to a ring buffer, which is implemented in a different component "ringbuffer", delete one component?
template< class EventType >
class Accumulator
	: public Component
{
protected:

	/** short-cut to the underlying mathematical construct encapsulated in the \c EventType measurement */
	typedef typename EventType::value_type MathType;
	
	/** short-cut to a container that stores the underlying elements of the incoming \c EventTypes measurements */
	typedef typename std::vector< MathType > EventTypeList;
	
	/** short-cut to a measurement encapsulating the container of \c EventTypes */
	typedef typename Measurement::Measurement< EventTypeList > MeasurementEventTypeList;
	
	/** Maximal size of container where the elements are stored */
	std::size_t m_maxLength;
	
	/** an index pointing to the next free position in container that can be overwritten */
	std::size_t m_index;
	
	/** container storing all the elements pushed to this component */
	EventTypeList m_dataStorage;

	/** Outgoing port of this component */
	PushSupplier< MeasurementEventTypeList > m_outPort;
	
	/** Incoming ports of this component */
	std::vector< boost::shared_ptr< Dataflow::PushConsumer< EventType > > > m_inPorts;

public:
	/**
	 * Standard component constructor.
	 *
	 * @param nm Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	Accumulator( const std::string& nm, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
		: Ubitrack::Dataflow::Component( nm )
		, m_maxLength( 100 )
		, m_index( 0 )
		, m_dataStorage()
		, m_outPort( "Output", *this )
	{
		pCfg->m_DataflowAttributes.getAttributeData( "maxLength", m_maxLength );
		
		// check for maximal storage size of container
		const std::size_t maxSize = m_dataStorage.max_size();
		if( maxSize < m_maxLength )
			LOG4CPP_WARN( logger, "Cannot allocate storage space for " << m_maxLength << " elements of \"" << typeid( MathType ).name() << "\", there is capacity for only " << maxSize << " of them." );
			
		// reserve the expected space in advance
		// reduces reallocation to zero -> expectation:faster		
		m_maxLength = std::min( maxSize, m_maxLength );
		m_dataStorage.reserve( m_maxLength );
		
		// adjust size once more to available storage capacity
		m_maxLength = std::min( m_dataStorage.capacity(), m_maxLength );
		LOG4CPP_INFO( logger, "Allocated space for storing " << m_maxLength << " elements of \"" << typeid( MathType ).name() << "\".");

		for ( Graph::UTQLSubgraph::EdgeMap::iterator it = pCfg->m_Edges.begin(); it != pCfg->m_Edges.end(); it++ )
			if ( it->second->isInput() )
					m_inPorts.push_back(
						boost::shared_ptr< Dataflow::PushConsumer< EventType > >( 
							new Dataflow::PushConsumer< EventType >(
								it->first, *this, boost::bind( &Accumulator::receive, this, _1 )
							)
						)
					);
	}

protected:

	/** called when a new item arrives */
	void receive( const EventType& event )
	{
		const std::size_t n = std::distance( m_dataStorage.begin(), m_dataStorage.end() );
		
		// storage is not full, yet, insert elements at the end
		if( n < m_maxLength )
			m_dataStorage.push_back( *event );
		else // storage is full
		{
			// overwrite elements from the beginning
			m_dataStorage[ m_index++ ] = (*event);
			// reset the index if at end
			m_index = m_index % m_maxLength;
		}

		m_outPort.send( MeasurementEventTypeList( event.time(), m_dataStorage ) );
	}
};

UBITRACK_REGISTER_COMPONENT( ComponentFactory* const cf ) 
{
	cf->registerComponent< Accumulator< Measurement::Pose > > ( "PoseAccumulator" );
	cf->registerComponent< Accumulator< Measurement::Position > > ( "PositionAccumulator" );
	cf->registerComponent< Accumulator< Measurement::Position2D > > ( "Position2DAccumulator" );
	
}

} } // namespace Ubitrack::Components

