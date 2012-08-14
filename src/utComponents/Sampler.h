
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
 * @author Adnane Jadid <jadid@in.tum.de>
 */

#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <log4cpp/Category.hh>

#include <utDataflow/Component.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/Exception.h>
#include <utUtil/OS.h>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.Sampler" ) );
static log4cpp::Category& eventLogger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.Sampler" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Components that pulls its input at a given frequency and pushes the result
 *
 * @par Input Ports
 * PullConsumer< EventType > with name "Input".
 *
 * @par Output Ports
 * PushSupplier< EventType > with name "Output".
 *
 * @par Configuration
 * @verbatim <Configuration frequency="100"/> @endverbatim
 * \c frequency: floating point number giving the sampling frequency in Hz
 *
 * @par Operation
 *
 * @par Instances
 *
 */
template< class EventType >
class Sampler
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	Sampler< EventType >( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Dataflow::Component( sName )
		, m_inPort( "Input", *this )
		, m_outPort( "Output", *this )
		, m_fFrequency( 1.0 )
		, m_nOffset( 0 )
		, m_bStop( true )
    {
		subgraph->m_DataflowAttributes.getAttributeData( "frequency", m_fFrequency );
		
		double offset( 0 );
		subgraph->m_DataflowAttributes.getAttributeData( "offset", offset );
		m_nOffset = (long long int)( 1e9 * offset );
		
		stop();
    }

	~Sampler()
	{
		stop();
	}

	/**
	 * component start method, starts thread
	 */
	virtual void start()
	{
		if ( !m_running )
		{
			m_running = true;
			m_bStop = false;
			m_pThread.reset( new boost::thread( boost::bind( &Sampler< EventType >::threadMethod, this ) ) );
		}
	}

	/**
	 * component stop method, stops thread
	 */
	virtual void stop()
	{
		// stop thread
		// mutex?
		if ( m_running )
		{
			m_running = false;
			m_bStop = true;
			if ( m_pThread )
			{
				m_pThread->join();
			}
		}
	}


protected:
	/** Method that computes the result. */
	void threadMethod();

	/** Input port of the component. */
	Dataflow::PullConsumer< EventType > m_inPort;

	// Output ports of the component
	Dataflow::PushSupplier< EventType > m_outPort;

	// sampling frequency
	double m_fFrequency;

	// offset to add to sampled timestamps
	long long int m_nOffset;
	
	// stop?
	bool m_bStop;

	// pointer to the thread
	boost::scoped_ptr< boost::thread > m_pThread;
};

template< class EventType >
void Sampler< EventType >::threadMethod()
{
	Measurement::Timestamp step = Measurement::Timestamp( 1 / m_fFrequency * 1e9 );
	Measurement::Timestamp now( Measurement::now() );
	Measurement::Timestamp nextTime( now + step );

	while ( !m_bStop )
	{
		// compute sleep time and sleep
		now = Measurement::now();
		if ( now < nextTime )
		{
			// sampling took less than the sleep duration
			long long int sleepdur = nextTime - now;
			nextTime += step;
			Util::sleep( int( sleepdur / 1000000 ), int( sleepdur % 1000000 ) );
		}
		else
		{
			// sampling took more than the sleep duration -> yield some processing time to other threads
			nextTime = now + step;
			boost::thread::yield();
		}

		// do sampling
		now = Measurement::now() + m_nOffset;
		try
		{
			m_outPort.send( m_inPort.get( now ) );
		}
		catch ( const Util::Exception& e )
		{
			LOG4CPP_WARN( eventLogger, "Got exception: " << e );
		}
		catch ( const std::exception& e )
		{
			LOG4CPP_WARN( eventLogger, "Got unknown exception: " << e.what() );
		}
	}
}

} } // namespace Ubitrack::Components


