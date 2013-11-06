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
 * @ingroup driver_components
 * @file
 * Generating a button event using the activated console.
 *
 * @author Christian Waechter <christian.waechter@cs.tum.edu>
 */
 
// std
#ifdef WIN32
#include <conio.h>
#endif

// Boost
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>

// Ubitrack
#include <utUtil/OS.h>
#include <utMath/Scalar.h>
#include <utMeasurement/Measurement.h>
#include <utDataflow/Component.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>


namespace Ubitrack { namespace Drivers {

class KeyboardEvent
	: public Dataflow::Component
{
protected:
	
	/** Output ports of the component. */
	Dataflow::PushSupplier< Measurement::Button > m_outPort;
	
	/** signs if the component should be stoppped. */
	bool m_bStop;
	
	/** thread that checks for keyboard input. */
	boost::scoped_ptr< boost::thread > m_pThread;
	
public:
	/**
	 * Standard component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	KeyboardEvent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
		: Dataflow::Component( sName )
		, m_outPort( "Output" , *this )
    {
	}
	
	/** Destructor */
	~KeyboardEvent()
	{
		stop();
	}
	
	/** component start method, starts thread */
	virtual void start()
	{
		if ( !m_running )
		{
			m_bStop = false;
			m_running = true;
			m_pThread.reset( new boost::thread( boost::bind( &KeyboardEvent::mainLoop, this ) ) );
		}
	}

	/** component stop method, stops thread */
	virtual void stop()
	{
		if ( m_running )
		{
			m_bStop = true;
			m_running = false;
			if ( m_pThread )
				m_pThread->join();
		}
	}
	
	void mainLoop()
	{
		while( !m_bStop )
		{
			Util::sleep( 2 );
			///@todo make an unix alternative, since this is Windows only at the moment
#ifdef WIN32
			if(kbhit())
			{
				int c = getch();
				Measurement::Timestamp t = Measurement::now();
				m_outPort.send( Measurement::Button( t, Math::Scalar< int >( c ) ) );
			}
#endif
		}
	}

};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< KeyboardEvent > ( "KeyboardEvent" );
}

} } // namespace Ubitrack::Drivers

