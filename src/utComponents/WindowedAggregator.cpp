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
 * Aggregator that uses a time based queue.
 * This class accumulates single (time-expanded) measurements into a (space-expanded) queue of measurements.
 * The aggregation is based on a certain time window given by the user.
 * 
 * @author Christian Waechter <christian.waechter@in.tum.de>
 */

#include <deque>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>


static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.WindowedAggregator" ) );

namespace Ubitrack { namespace Components {

template< class EventType >
class WindowedAggregator
	: public Dataflow::TriggerComponent
{

public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	WindowedAggregator( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > cfg )
		: Dataflow::TriggerComponent( sName, cfg )
		, m_inPort( "Input", *this )
		, m_outPort( "Output", *this )
		, m_time( 0.0 ) // duration in nanoseconds
	{
		m_mList.clear();
		m_tList.clear();
		if( cfg->m_DataflowAttributes.hasAttribute( "time" ) )
		{
			cfg->m_DataflowAttributes.getAttributeData( "time", m_time );
			LOG4CPP_TRACE( logger, "desired list duration[ms] : " << m_time );
			m_time *= 1e+06; //ms to ns
		}
	}

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		//add newest elements to queues
		m_mList.push_back( *m_inPort.get() );
		m_tList.push_back( t );
		
		//delete oldest elements from queues
		while( ( m_tList.front() + m_time ) < t )
		{
			m_mList.pop_front();
			m_tList.pop_front();
		}	

		//copy all elements from queue to vector
		std::vector< typename EventType::value_type > output;
		output.reserve( m_mList.size() );
		output.assign( m_mList.begin(), m_mList.end() );
		LOG4CPP_TRACE( logger, "items in queue: " << m_mList.size() );
 		m_outPort.send( Measurement::Measurement< typename std::vector< typename EventType::value_type > >( t, output ) );
	}

	
protected:
	/** Input port of the component. */
	Dataflow::TriggerInPort< EventType > m_inPort;
	
	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Measurement< typename std::vector< typename EventType::value_type > > > m_outPort;
	
	/** max event age to be allowed */
	double m_time; 

	/** queue containing all measurements */
	std::deque< typename EventType::value_type > m_mList;
	
	/** queue containing all timestamps */
	std::deque< Measurement::Timestamp > m_tList;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) 
{
	cf->registerComponent< WindowedAggregator< Measurement::Pose > > ( "PoseWindowedAggregator" );
	cf->registerComponent< WindowedAggregator< Measurement::Position > > ( "PositionWindowedAggregator" );
	cf->registerComponent< WindowedAggregator< Measurement::Distance > > ( "DistanceWindowedAggregator" );
}

} } // namespace Ubitrack::Components

