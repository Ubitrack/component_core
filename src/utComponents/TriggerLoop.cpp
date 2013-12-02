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
 * Components that pulls its input at a given frequency and pushes the result
 *
 * @author Peter Keitler <keitler@in.tum.de>
 */

#include <string>

#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <utDataflow/Component.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/Exception.h>
#include <utUtil/OS.h>


using namespace Ubitrack;

// get a logger
#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.TriggerLoop" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Trigger loop component
 *
 * @par Input Ports
 * PushConsumer< Button > with name "IterationDone".
 * PushConsumer< Button > with name "ExternalTrigger".
 *
 * @par Output Ports
 * PushSupplier< Button > with name "LoopDone".
 * PushSupplier< Button > with name "IterationTrigger".
 *
 * @par Configuration
 *
 * @par Operation
 *
 * Implements a loop in the Ubitrack dataflow. The component has to be
 * triggered via its {@code ExternalTrigger}. It then generates many
 * trigger events on its {@code IterationTrigger} output. Those should
 * lead to the same amount of events being pushed on the {@code
 * IterationDone} input. Finally, one event is pushed onward on the
 * {@code LoopDone} output. An endless loop can be constructed by
 * setting {@link #m_counter} to 0. An event on the {@code
 * ExternalTrigger} resets the component and starts the loop anew.

 *
 * @par Instances
 *
 */
class TriggerLoop
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	TriggerLoop( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Dataflow::Component( sName )
		, m_inExtTrigger( "LoopTrigger", *this, boost::bind( &TriggerLoop::externalTrigger, this, _1 ) )
		, m_inIterationDone( "IterationDone", *this, boost::bind( &TriggerLoop::iterationDone, this, _1 ) )
		, m_outIterationTrigger( "IterationTrigger", *this )
		, m_outLoopDone ( "LoopDone", *this )
		, m_bStopped( true )
		, m_counter ( 0 )
		, m_size( 100 )
		, m_button( ' ' )
		, m_inButton( ' ' )
    {
		LOG4CPP_DEBUG( logger, "Setup TriggerLoopcomponent" );

		// read configuration for amount of samples to use
		subgraph->m_DataflowAttributes.getAttributeData( "size", m_size );

		// read button key
		std::string button( " " );
		std::string inButton( " " );

		if ( subgraph->m_DataflowAttributes.hasAttribute( "button" ) )
			button = subgraph->m_DataflowAttributes.getAttributeString( "button" );
		if ( subgraph->m_DataflowAttributes.hasAttribute( "inButton" ) )
			inButton = subgraph->m_DataflowAttributes.getAttributeString( "inButton" );
			
		m_button = Math::Scalar< int >( button[ 0 ] );
		m_inButton = Math::Scalar< int >( inButton[ 0 ] );
    }


	~TriggerLoop()
	{}


    void externalTrigger( const Measurement::Button& e )
    {
		if ( *e == m_inButton )
		{
			boost::mutex::scoped_lock l( m_mutex );

			if ( m_bStopped ) 
			{
				LOG4CPP_DEBUG( logger, getName() << " Received trigger event with timestamp " << e.time() << ". Trigger first loop iteration..." );
				m_outIterationTrigger.send( Measurement::Button( e.time(), m_button ) );
				m_bStopped = false;

				// Reset internal state
				m_counter = 0;
			}
			else 
			{
				LOG4CPP_ERROR( logger, getName() << " received trigger signal while computation was already running. Ignored." );
			}
		}
	}


    void iterationDone( const Measurement::Button& e )
    {
		LOG4CPP_TRACE( logger, getName() << " Received loop iteration done event with timestamp " << e.time() );

		m_counter ++;
		LOG4CPP_TRACE( logger, getName() << " Current counter: " << m_counter << ", go on until: " << m_size );

		// Check list size
		if ( m_size > 0 && m_counter == m_size )
		{
			// push onward loop done event
			LOG4CPP_DEBUG( logger, getName() << " Terminate and push loop done event" );
			m_outLoopDone.send( Measurement::Button ( e.time(), m_button ) );

			boost::mutex::scoped_lock l( m_mutex );
			m_bStopped = true;

			return;
		}
		
		// If not reached, send next trigger event
		LOG4CPP_TRACE( logger, getName() << " Triggering next loop iteration..." );
		m_outIterationTrigger.send( Measurement::Button( Measurement::now(), m_button ) );
	}
	

protected:
	/** Input port of the component. */
	Dataflow::PushConsumer< Measurement::Button > m_inExtTrigger;
	Dataflow::PushConsumer< Measurement::Button > m_inIterationDone;

	// Output ports of the component
	Dataflow::PushSupplier< Measurement::Button > m_outIterationTrigger;
	Dataflow::PushSupplier< Measurement::Button > m_outLoopDone;

	// stop
	bool m_bStopped;

	int m_counter;
	int m_size;

	Math::Scalar< int > m_button;
	Math::Scalar< int > m_inButton;

	boost::mutex m_mutex;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Components::TriggerLoop >( "TriggerLoop" );
}

} } // Namespace Ubitrack::Components
