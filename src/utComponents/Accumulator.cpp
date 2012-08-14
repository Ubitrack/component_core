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

#include <boost/bind.hpp>
//#include <log4cpp/Category.hh>

#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

// currently unused
// static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.Accumulator" ) );

namespace Ubitrack { namespace Components {

using namespace Dataflow;

template< class EventType > class Accumulator : public Component
{
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
		, m_size( 0 )
		, m_data()
		, m_outPort( "Output", *this )
	{
		pCfg->m_DataflowAttributes.getAttributeData( "maxLength", m_maxLength );

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
		m_data.push_back( *event );
		if (++m_size > m_maxLength) {
			m_data.erase( m_data.begin() );
			m_size--;
		}
		m_outPort.send( Measurement::Measurement< typename std::vector< typename EventType::value_type > >( event.time(), m_data ) );
	}

	/** Properties of the accumulator*/
	unsigned int m_maxLength,m_size;
	std::vector< typename EventType::value_type > m_data;

	/** Ports of the component */
	PushSupplier< Measurement::Measurement< typename std::vector< typename EventType::value_type > > > m_outPort;
	std::vector< boost::shared_ptr< Dataflow::PushConsumer< EventType > > > m_inPorts;

};

UBITRACK_REGISTER_COMPONENT( ComponentFactory* const cf ) 
{
	cf->registerComponent< Accumulator< Measurement::Position > > ( "PositionAccumulator" );
	cf->registerComponent< Accumulator< Measurement::Position2D > > ( "Position2DAccumulator" );
	cf->registerComponent< Accumulator< Measurement::Pose > > ( "PoseAccumulator" );
}

} } // namespace Ubitrack::Components

