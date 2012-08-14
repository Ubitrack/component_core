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
 * Time complementary fusion component.
 *
 * @author Manuel Huber <huberma@in.tum.de>
 */
#include <string>
#include <iostream>

#include <boost/bind.hpp>

#include <utDataflow/PushConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Time complementary fusion component.
 *
 * @par Input Ports
 * PushConsumer<EventType> port with name "InputA".
 * PushConsumer<EventType> port with name "InputB".
 *
 * @par Output Ports
 * PushSupplier<EventType> port with name "Output".
 *
 * @par Configuration
 * None.
 *
 * @par Operation
 * TBD
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 * - Ubitrack::Measurement::Pose : PoseTimeComplementaryFusion
 * - Ubitrack::Measurement::Rotation : RotationTimeComplementaryFusion
 */
template< class EventType > class TimeComplementaryFusionManual
	: public Dataflow::Component
{
public:
	TimeComplementaryFusionManual( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  )
      : Dataflow::Component( sName )
      , m_portA( "InputA", *this, boost::bind( &TimeComplementaryFusionManual::poseInA, this, _1 ) )
      , m_portB( "InputB", *this, boost::bind( &TimeComplementaryFusionManual::poseInB, this, _1 ) )
      , m_portC( "InputC", *this, boost::bind( &TimeComplementaryFusionManual::poseInC, this, _1 ) )
	  , m_SignalPort( "ButtonEvent", *this, boost::bind( &TimeComplementaryFusionManual::signalIn, this, _1 ) )
      , m_outPort( "Output", *this )
	{
		m_button = Math::Scalar< int >( -1 );
    }

protected:
	// receives events from InputA
    void poseInA( const EventType& m )
    {
		if ( m_button == '1' )
			m_outPort.send( m );
    }

	// receives events from InputB
    void poseInB( const EventType& m )
    {
		if ( m_button == '2' )
			m_outPort.send( m );
    }

	// receives events from InputC
    void poseInC( const EventType& m )
    {
		if ( m_button == '3' )
			m_outPort.send( m );
    }

	
	void signalIn( const Measurement::Button& b ) 
	{
		m_button = *b;
	}
	
	Math::Scalar< int > m_button;

	Dataflow::PushConsumer< EventType > m_portA;
	Dataflow::PushConsumer< EventType > m_portB;
	Dataflow::PushConsumer< EventType > m_portC;
	Dataflow::PushConsumer< Measurement::Button > m_SignalPort;

	Dataflow::PushSupplier< EventType > m_outPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< TimeComplementaryFusionManual< Measurement::Pose > > ( "PoseTimeComplementaryFusionManual" );
}

} } // namespace Ubitrack::Components
