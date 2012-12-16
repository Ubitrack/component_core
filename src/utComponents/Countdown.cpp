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
 * Components that sends a true signal after a chosen time
 *
 * @author Daniel Muhra <muhra@in.tum.de>
 */

#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <log4cpp/Category.hh>

#include <utDataflow/Component.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/Exception.h>
#include <utUtil/OS.h>

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Components that sends a true signal after a chosen time
 *
 * @par Output Ports
 * PushSupplier< bool > with name "Output".
 *
 * @par Configuration
 * \c time: countdown time in seconds
 * \c number: how often to send the signal; -1 for infinite
 *
 * @par Operation
 *
 * @par Instances
 *
 */

class Countdown
	: public Dataflow::Component
{
public:
	/**
	 * Standard component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	Countdown( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
		: Dataflow::Component( sName )
		, m_outPort( "Output", *this )
    {
		pCfg->m_DataflowAttributes.getAttributeData( "time", m_seconds );
		pCfg->m_DataflowAttributes.getAttributeData( "number", m_number );

		boost::thread* thread;
		if(m_number != -1)
			thread = new boost::thread(boost::bind( &Countdown::down, this ));
		else thread = new boost::thread(boost::bind( &Countdown::downInfinite, this ));
    }

protected:
	/** Methods that sends the signal. */
	void down();
	void downInfinite();

	// Output ports of the component
	Dataflow::PushSupplier< Measurement::Button > m_outPort;
	
	// waiting time
	int m_seconds;
	
	// how often to do?
	int m_number;	
};

void Countdown::down()
{
	while(m_number > 0)
	{
		Util::sleep(m_seconds*1000, 0);
		m_outPort.send( Measurement::Button(Measurement::now(), Math::Scalar<int>(1)));
		m_number--;		
	}
}

void Countdown::downInfinite()
{
	while(true)
	{
		Util::sleep(m_seconds*1000, 0);
		m_outPort.send(Measurement::Button(Measurement::now(), Math::Scalar<int>(1)));		
	}
}

} } // namespace Ubitrack::Components

UBITRACK_REGISTER_COMPONENT( Ubitrack::Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Components::Countdown > ( "Countdown" );
}
