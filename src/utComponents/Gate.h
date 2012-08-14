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

#ifndef __UBITRACK_COMPONENTS_GATE_H_INCLUDED__
#define __UBITRACK_COMPONENTS_GATE_H_INCLUDED__

/**
 * @ingroup dataflow_components
 * @file
 * Gate component
 * This class pushes a given Event only after receiving a signal.
 *
 * @author Daniel Muhra <muhra@in.tum.de>
 */

#include <boost/bind.hpp>
#include <log4cpp/Category.hh>

#include <utDataflow/PushConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/Component.h>
#include <utMeasurement/Measurement.h>
#include <utGraph/UTQLSubgraph.h>

namespace Ubitrack { namespace Components {

using namespace Dataflow;

/**
 * @ingroup dataflow_components
 * Gate component
 * This class pushes a given Event only after receiving a signal.
 *
 * @par Configuration
 * \c gateType: wich event should be sent. 1 for next incoming Event, 0 for last received event.
 * \c button: the button, on wich the gate should open.
 *
 * @par Operation
 * Whenever a signal is received, an incoming Event is allowed to pass.
 *
 */		

template< class EventType > 
class Gate : public Component
{
public:
	/**
	 * Standard component constructor.
	 *
	 * @param nm Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	Gate( const std::string& nm, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
        : Ubitrack::Dataflow::Component( nm )
		, m_type( 0 )
		, m_inCount( 0 )
		, m_button( ' ' )
        , m_inPort( "Input", *this, boost::bind( &Gate::manageGate, this, _1 ) )
		, m_SignalPort( "Signal", *this, boost::bind( &Gate::openGate, this, _1 ) )
		, m_outPort( "Output", *this )
		, logger( log4cpp::Category::getInstance( "Ubitrack.Components.Gate" ) )
	{
		pCfg->m_DataflowAttributes.getAttributeData( "gateType", m_type );

		// read button key
		std::string button( " " );

		if ( pCfg->m_DataflowAttributes.hasAttribute( "button" ) )
			button = pCfg->m_DataflowAttributes.getAttributeString( "button" );
			
		if ( button.empty() )
			m_button = Math::Scalar< int >( -1 );
		else 
		{
			LOG4CPP_INFO( logger, "Configure gate to react on button '" << button[0] << "' (ID " << (int)(button[0]) );
			
			m_button = Math::Scalar< int >( button[ 0 ] );
		}

		m_open = false;
	}

protected:

	/**
	 * Receives an event and stores it.
	 * @param m the received event
	 */
	void manageGate( const EventType& data )
	{
		m_inCount = 1;
		old = data;
		if ( m_open && m_type == 1 )
		{
			m_outPort.send( old );
			LOG4CPP_DEBUG( logger, "Gate sending current measurement" );
			m_open = false;
		}
	}

	void openGate( const Measurement::Button& b )
	{
		LOG4CPP_DEBUG( logger, "Received button event with ID " << *b << ", shall we open the gate configured for event ID" << m_button << "?" );
		
		if ( m_button < 0 || *b == m_button )
		{
			if ( m_type == 0 )
				if ( m_inCount )
				{
					m_outPort.send( old );
					LOG4CPP_DEBUG( logger, "Gate sending last measurement" );
				}
				else
				{
					LOG4CPP_WARN( logger, "Gate has not enough data" );
				}
			else if ( m_type == 2 )
				if ( m_inCount )
				{
				        old.time( b.time() );
					m_outPort.send( old );
					LOG4CPP_DEBUG( logger, "Gate synchronizing last measurement on button timestamp" );
				}
				else
				{
					LOG4CPP_WARN( logger, "Gate has not enough data" );
				}
			else 
				m_open = true;
		}
	}

	/** properties of the gate*/
	int m_type;
	bool m_open;
	int m_inCount;
	Math::Scalar< int > m_button;

	/**last received event*/
	EventType old;

	/** Ports of the component. */
	PushConsumer< EventType > m_inPort;
	PushConsumer< Measurement::Button > m_SignalPort;
	PushSupplier< EventType > m_outPort;
	
	/** log4cpp logger reference */
	log4cpp::Category& logger;	
};

} } // namespace Ubitrack::Components

#endif
