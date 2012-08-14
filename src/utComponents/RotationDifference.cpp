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
 * Components that differenciates incoming rotation measurements
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */
#include <log4cpp/Category.hh>
#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Events.RotationDifference" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Components that differenciates incoming rotation measurements
 *
 * @par Input Ports
 * PushConsumer<Rotation> with name "Input".
 *
 * @par Output Ports
 * PushSupplier<RotationVelocity> with name "Ouput".
 *
 * @par Configuration
 * @verbatim <Configuration maxTime="..."/>@endverbatim
 * where \c maxTime is the maximum time between measurements in ms
 *
 * @par Operation
 * Computes the RotationVelocity from the previous and the current measurement, divided by the time:
 * vel = ( a^-1 * b ) / dt
 *
 * @par Instances
 */
class RotationDifferenceComponent
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	RotationDifferenceComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Dataflow::Component( sName )
		, m_inPort( "Input", *this, boost::bind( &RotationDifferenceComponent::receive, this, _1 ) )
		, m_outPort( "Output", *this )
		, m_nReceived( 0 )
		, m_maxTime( 1000000000L )
    {
		const TiXmlElement* node = subgraph->m_DataflowConfiguration.getXML();
		if ( node )
		{
			unsigned long ms = 1000;
			subgraph->m_DataflowAttributes.getAttributeData( "maxTime", ms );
			m_maxTime = ms * 1000000L;
		}
    }

	/** Method that computes the result. */
	void receive( const Measurement::Rotation& m )
	{
		if ( m_nReceived )
			if ( m_lastMeasurement.time() + m_maxTime > m.time() && m_lastMeasurement.time() < m.time() )
			{
				Measurement::RotationVelocity vel( m.time(),
					Math::RotationVelocity( *m_lastMeasurement, *m, ( m.time() - m_lastMeasurement.time() ) / 1e9 ) );
				m_outPort.send( vel );
			}
			else
				LOG4CPP_NOTICE( logger, "Measurements too old for RotationDifference" );

		m_lastMeasurement = m;
		m_nReceived++;
    }

protected:
	/** Input port of the component. */
	Dataflow::PushConsumer< Measurement::Rotation > m_inPort;

	/** Output port of the component */
	Dataflow::PushSupplier< Measurement::RotationVelocity > m_outPort;

	/** number of measurements received so far */
	unsigned m_nReceived;

	/** maximum time between two measurements */
	Measurement::Timestamp m_maxTime;

	/** last measurement received */
	Measurement::Rotation m_lastMeasurement;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< RotationDifferenceComponent > ( "RotationDifference" );
}

} } // namespace Ubitrack::Components
