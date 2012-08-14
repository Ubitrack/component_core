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
 *
 * @file Component for pull-push conversion that reacts upon
 * pushed signal events. Upon a pushed event, a measurement is pulled
 * an pushed onward with the timestamp of the signal event.
 *
 * @author Michael Schlegel <schlegem@in.tum.de>
 */

#include <utGraph/UTQLSubgraph.h>
#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utMeasurement/Measurement.h>

#include <log4cpp/Category.hh>

namespace Ubitrack { 
namespace Components {

/**
 * @ingroup dataflow_components This component does a push pull
 * conversion. If a butten event is arriving a measurement will be
 * pulled. The measurement is then sent via push
 *
 * @par Input Ports
 * PullConsumer<Measurement> with name "Input".
 * PushConsumer<Measurement::Button> with name "Trigger".
 *
 * @par Output Ports
 * PushSupplier<Measurement> with name "Output".
 *
 * @par Configuration
 * \c button: the button event which shall trigger a push on the \c Output port.
 *
 * @par Operation
 *
 * @par Instances
 */
template< class EventType >
class Trigger
    : public Dataflow::Component
{
public:
    /**
     * UTQL component constructor.
     *
     * @param sName Unique name of the component.
     * @param subgraph UTQL subgraph
     */
    Trigger( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Dataflow::Component( sName )
		, m_button( ' ' )
		, m_inPort("Input", *this)
		, m_inTriggerPort( "Trigger", *this, boost::bind( &Trigger::receiveEvent, this, _1 ) )
		, m_outPort( "Output", *this )
		, m_logger( log4cpp::Category::getInstance( "Ubitrack.Components.Trigger" ) )
    {
		// read button key
		std::string button( " " );

		if ( subgraph->m_DataflowAttributes.hasAttribute( "event" ) )
			button = subgraph->m_DataflowAttributes.getAttributeString( "event" );
			
		if ( button.empty() )
			m_button = Math::Scalar< int >( -1 );
		else 
		{
			LOG4CPP_INFO( m_logger, "Configure Trigger to react on button event '" << button[0] << "' (ID " << (int)(button[0]) );
			
			m_button = Math::Scalar< int >( button[ 0 ] );
		}
    }

    /** Method that computes the result. */
    void receiveEvent( const Measurement::Button& event )
    {
		if ( m_button < 0 || *event == m_button )
		{
			m_outPort.send( m_inPort.get( event.time() ) );
		}
    }

protected:
	Math::Scalar< int > m_button;
     
    /** Input port of the component. */
    Dataflow::PullConsumer< EventType > m_inPort;
    Dataflow::PushConsumer< Measurement::Button > m_inTriggerPort;

    /// Output port of the component
    Dataflow::PushSupplier< EventType > m_outPort;

    /** log4cpp logger reference */
    log4cpp::Category& m_logger;	
};


} } // namespace Ubitrack::Components
