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
 * Aggregator component.
 * This class bundles several push inputs into one output.
 *
 * Any number of input edges can be used, as long as they all supply the correct
 * measurement type.
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

template< class EventType > class Aggregator: public Component
{
public:
	/**
	 * Standard component constructor.
	 *
	 * @param nm Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	Aggregator( const std::string& nm, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
		: Ubitrack::Dataflow::Component( nm )
		, m_outPort( "Output", *this )
	{
		for ( Graph::UTQLSubgraph::EdgeMap::iterator it = pCfg->m_Edges.begin(); it != pCfg->m_Edges.end(); it++ )
			if ( it->second->isInput() )
					m_inPorts.push_back(
						boost::shared_ptr< Dataflow::PushConsumer< EventType > >( 
							new Dataflow::PushConsumer< EventType >(
								it->first, *this, boost::bind( &Aggregator::receive, this, _1 )
							)
						)
					);
	}

protected:

	/** called when a new item arrives */
	void receive( const EventType& event )
	{
		m_outPort.send( event );
	}

	/** Ports of the component */
	PushSupplier< EventType > m_outPort;
	std::vector< boost::shared_ptr< Dataflow::PushConsumer< EventType > > > m_inPorts;

};

UBITRACK_REGISTER_COMPONENT( ComponentFactory* const cf ) 
{
	cf->registerComponent< Aggregator< Measurement::Pose> > ( "PoseAggregator" );
	cf->registerComponent< Aggregator< Measurement::Rotation > > ( "RotationAggregator" );
	cf->registerComponent< Aggregator< Measurement::Position > > ( "PositionAggregator" );
}

} } // namespace Ubitrack::Components

