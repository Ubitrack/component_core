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
 * Collector component.
 * This file collects a single measrurement and adds it to the list
 * Implemented this way to comply with the Trackman concepts
 * If trackman supports ways of modeling optional edges
 * this component could be changed.
 *
 * @author Christian Waechter <christian.waechter@in.tum.de>
 */
 
// Boost
#include <boost/bind.hpp>
 
// Ubitrack
#include <utDataflow/PullSupplier.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utGraph/UTQLSubgraph.h>
#include <utUtil/Exception.h>

// LOG4CPP
#include <log4cpp/Category.hh>
static log4cpp::Category& eventLogger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.Collector" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Collector component.
 * This class collects single measurments and adds it to a list
 *
 * The component collects requested events
 */
template< class EventTypeA, class EventTypeB >
class CollectorComponent
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	CollectorComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::Component( sName )
		, m_inPortA( "Input", *this )
		, m_inPortB( "List", *this )
		, m_outPort( "Output", *this, boost::bind( &CollectorComponent::sendOutput, this, _1 ) )
    {
    }

	EventTypeB sendOutput( Measurement::Timestamp t )
	{
		EventTypeB List;
		try
		{
			List = m_inPortB.get( t );
		}
		catch ( const Util::Exception& e )
		{
		        List = EventTypeB( typename EventTypeB::value_type() );
        		LOG4CPP_WARN( eventLogger, "Got exception: " << e );
		}
		catch ( const std::exception& e )
		{
			LOG4CPP_WARN( eventLogger, "Got unknown exception: " << e.what() );
		}
		
		try
		{
			EventTypeA Meas = m_inPortA.get( t );
			List->push_back( *Meas );
		}
		catch ( const Util::Exception& e )
		{
			LOG4CPP_WARN( eventLogger, "Got exception: " << e );
		}
		catch ( const std::exception& e )
		{
			LOG4CPP_WARN( eventLogger, "Got unknown exception: " << e.what() );
		}
		return EventTypeB( t, List );
	}

protected:
	
	/** Input port A of the component. */
	Dataflow::PullConsumer< EventTypeA > m_inPortA;
	
	/** Input port B of the component. */
	Dataflow::PullConsumer< EventTypeB > m_inPortB;

	/** Output port of the component. */	
	Dataflow::PullSupplier< EventTypeB > m_outPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {

	cf->registerComponent< CollectorComponent< Measurement::Pose, Measurement::PoseList > > ( "PoseCollector" );
	cf->registerComponent< CollectorComponent< Measurement::Position, Measurement::PositionList > > ( "PositionCollector" );
	cf->registerComponent< CollectorComponent< Measurement::Position2D, Measurement::PositionList2 > > ( "PositionCollector2D" );
	cf->registerComponent< CollectorComponent< Measurement::Distance, Measurement::DistanceList > > ( "DistanceCollector" );
	
}

} } // namespace Ubitrack::Components
