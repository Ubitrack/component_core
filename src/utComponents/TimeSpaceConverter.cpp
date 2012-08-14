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
 * Time-to-space converter component.
 * This class accumulates single (time-expanded) measurements into a (space-expanded) list of measurements.
 * 
 * @author Peter Keitler <keitler@in.tum.de>
 */

#include <boost/bind.hpp>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>


static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.TimeSpaceConverter" ) );

namespace Ubitrack { namespace Components {

template< class EventType >
class TimeSpaceConverter
	: public Dataflow::TriggerComponent
{

public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	TimeSpaceConverter( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > cfg )
		: Dataflow::TriggerComponent( sName, cfg )
		, m_inPort( "Input", *this )
		, m_outPort( "Output", *this )
		, m_size( 30 )
	{
		m_list.clear();
		cfg->m_DataflowAttributes.getAttributeData( "size", m_size );
	}

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		LOG4CPP_TRACE( logger, "desired list size: " << m_size );

		// Retrieve and store measurement
		m_list.push_back( *m_inPort.get() );

		LOG4CPP_TRACE( logger, "current size: " << m_list.size() );

		if ( m_list.size() < m_size ) 
		{
			if ( isPortPush ( "Input" ) ) 
			{
				LOG4CPP_TRACE( logger, "push input port, wait for more measurements..." );
				return;
			}

			LOG4CPP_TRACE( logger, "pull inport port, retrieve missing measurements..." );
			while ( m_list.size() < m_size )
			{
				LOG4CPP_TRACE( logger, "items in list: " << m_list.size() << ", pulling next measurement" );
				
				m_inPort.pull( t );
				m_list.push_back( *m_inPort.get() );
			};
		}

		LOG4CPP_TRACE( logger, "desired list size reached" );
 		m_outPort.send( Measurement::Measurement< typename std::vector< typename EventType::value_type > >( t, m_list ) );
		m_list.clear();
	}

	
protected:
	/** Input port of the component. */
	Dataflow::TriggerInPort< EventType > m_inPort;
	
	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Measurement< typename std::vector< typename EventType::value_type > > > m_outPort;
	
	/** Size of list to aggregate */
	unsigned int m_size;

	std::vector< typename EventType::value_type > m_list;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) 
{
	cf->registerComponent< TimeSpaceConverter< Measurement::Pose > > ( "PoseTimeToSpaceConverter" );
	cf->registerComponent< TimeSpaceConverter< Measurement::Position > > ( "PositionTimeToSpaceConverter" );
	cf->registerComponent< TimeSpaceConverter< Measurement::Distance > > ( "DistanceTimeToSpaceConverter" );
}

} } // namespace Ubitrack::Components

