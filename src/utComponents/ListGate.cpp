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
 * List gate component
 * This class pushes the next measurement from a list of measurements only after receiving a signal.
 *
 * @author Peter Keitler <keitler@in.tum.de>
 */

#include <boost/bind.hpp>
#include <log4cpp/Category.hh>

#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.Gate" ) );

namespace Ubitrack { namespace Components {

using namespace Dataflow;

/**
 * @ingroup dataflow_components
 * Gate component
 * This class pushes a given Event only after receiving a signal.
 *
 * @par Input Ports
 * PullConsumer<EventType> port with name "Input".
 * PushConsumer<Measurement::Button> port with name "Signal".
 *
 * @par Output Ports
 * PushSupplier<EventType> port with name "Output".
 *
 * @par Configuration
 * \c event: the button event on wich the gate should open.
 *
 * @par Operation
 * Whenever a signal is received, an incoming Event is allowed to pass.
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 * - Ubitrack::Measurement::PositionList : PositionListGate
 *
 * @par Example Configuration
 * \verbatim 
<Pattern name="ListGate" id="someId">
	<Input>
		<Node name="CoordSystem" id="someId"/>
		<Node name="PointCloud" id="someId2"/>
		<Node nmae="EventSpace" id="someId3"/>
		<Node name="Event" id="someId4"/>
		<Edge name="Coordinates" source="CoordSystem" destination="PointCloud" pattern-ref="..." edge-ref="...">
			<Predicate>type=="PositionList"&amp;&amp;mode=="pull"</Predicate>
		</Edge>
		<Edge name="ButtonEvent" source="EventSpace" destination="Event" pattern-ref="..." edge-ref="...">
			<Predicate>type=="Button"&amp;&amp;mode=="push"</Predicate>
		</Edge>
	</Input>
	<Output>
		<Node name="Point" id="someId5"/>
		<Edge name="Coordinate" source="CoordSystem" destination="Point">
		    <Attribute name="type" value="3DPosition"/>
			<Attribute name="mode" value="push"/>
		</Edge>
	</Output>
	<DataflowConfiguration>
		<UbitrackLib class="PositionListGate"/>
		<Attribute name="event" value="42"/>
	</DataflowConfiguration>
</Pattern> 
\endverbatim
 */		

template< class EventType > class ListGate : public Component
{
public:
	/**
	 * Standard component constructor.
	 *
	 * @param nm Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	ListGate( const std::string& nm, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
        : Ubitrack::Dataflow::Component( nm )
		, m_inCount( 0 )
		, m_event( ' ' )
		, m_listPort( "Coordinates", *this )
		, m_buttonPort( "ButtonEvent", *this, boost::bind( &ListGate::openGate, this, _1 ) )
		, m_outPort( "Coordinate", *this )
		, m_nextEventPort( "NextCoordinate", *this, boost::bind( &ListGate::getNextEvent, this, _1 ) )
	{
		// default button key
		std::string button( " " );
		if ( pCfg->m_DataflowAttributes.hasAttribute( "event" ) )
			button = pCfg->m_DataflowAttributes.getAttributeString( "event" );
			
		if ( button.empty() )
			m_event = Math::Scalar< int >( -1 );
		else
			m_event = Math::Scalar< int >( button[ 0 ] );
	}

protected:

	/**
	 * Retrieves next list element from listPort and pushes it on outPort
	 */
	void openGate( const Measurement::Button& b )
	{
		LOG4CPP_DEBUG( logger, "List gate received button event " << b );

		if ( m_event < 0 || *b == m_event )
		{
			Measurement::Measurement< std::vector< typename EventType::value_type > > list = m_listPort.get( b.time() );

			LOG4CPP_DEBUG( logger, "List gate sending next list element " << list->at( m_inCount ) );
			m_outPort.send( EventType( b.time(), list->at( m_inCount ) ) );

			m_inCount++;
			if ( m_inCount == list->size() )
			{
				m_inCount = 0;
				LOG4CPP_DEBUG( logger, "List gate wrapping around" );
			}
		}
	}

	/**
	 * returns the next coordinate as a pull port.
	 * Required e.g. for HMD calibration, where the cursor position needs to be displayed before the actual alignment.
	 */
	EventType getNextEvent( Measurement::Timestamp t )
	{
		Measurement::Measurement< std::vector< typename EventType::value_type > > list = m_listPort.get( t );
		if ( m_inCount >= list->size() )
			return EventType();
			
		return EventType( t, list->at( m_inCount ) );
	}
	
	/** Properties of the gate */
	unsigned int m_inCount;
	Math::Scalar< int > m_event;

	/** Ports of the component */
	PullConsumer< Measurement::Measurement< typename std::vector< typename EventType::value_type > > > m_listPort;
	PushConsumer< Measurement::Button > m_buttonPort;
	PushSupplier< EventType > m_outPort;
	PullSupplier< EventType > m_nextEventPort;
};

UBITRACK_REGISTER_COMPONENT( ComponentFactory* const cf ) 
{
	cf->registerComponent< ListGate< Measurement::Position > > ( "3DPointCloudGate" );
	cf->registerComponent< ListGate< Measurement::Position2D > > ( "2DPointCloudGate" );
	cf->registerComponent< ListGate< Measurement::Pose > > ( "PoseCloudGate" );
}

} } // namespace Ubitrack::Components

