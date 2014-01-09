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
 * Inversion component.
 * This file contains a datatype conversion component.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/Exception.h>

namespace Ubitrack { namespace Components {


// generic measurement cast function
template< class A, class B >
A convert( const B& b )
{ return A( b.time(), *b ); }

// list extraction
template< class A >
A convert( const Measurement::Measurement< std::vector< typename A::value_type > >& b )
{
	if ( b->size() != 1 )
		UBITRACK_THROW( "list size not 1" );
	return A( b.time(), b->at( 0 ) );
}


/**
 * @ingroup dataflow_components
 * Cast component. Converts one measurement type to another (provided this makes any sense).
 *
 * @par Input Ports
 * TriggerInPort<EventTypeIn> with name "Input".
 *
 * @par Output Ports
 * TriggerOutPort<EventTypeOut> with name "Output".
 *
 * @par Operation
 * The component converts requested/incoming events
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 */
template< class EventTypeIn, class EventTypeOut >
class CastComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	CastComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPort( "Input", *this )
		, m_outPort( "Output", *this )
    	{
    	}

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t );

protected:
	/** Input port of the component. */
	Dataflow::TriggerInPort< EventTypeIn > m_inPort;
	/** Output port of the component. */
	Dataflow::TriggerOutPort< EventTypeOut > m_outPort;
};



template< >
void CastComponent< Measurement::ErrorPose, Measurement::Pose >::compute( Measurement::Timestamp t )
{
	m_outPort.send( Measurement::Pose( t, *m_inPort.get() ) );
}

template<>
void CastComponent< Measurement::Pose, Measurement::Position >::compute( Measurement::Timestamp t )
{
	m_outPort.send( Measurement::Position( t, m_inPort.get()->translation() ) );
}

template<>
void CastComponent< Measurement::Pose, Measurement::Rotation >::compute( Measurement::Timestamp t )
{
	m_outPort.send( Measurement::Rotation( t, m_inPort.get()->rotation() ) );
}

template<>
void CastComponent< Measurement::PositionList, Measurement::Position >::compute( Measurement::Timestamp t )
{
	if ( m_inPort.get()->size() != 1 )
		UBITRACK_THROW( "list size not 1" );
	m_outPort.send( Measurement::Position( t, m_inPort.get()->at( 0 ) ) );
}

template<>
void CastComponent< Measurement::PositionList2, Measurement::Position2D >::compute( Measurement::Timestamp t )
{
	if ( m_inPort.get()->size() != 1 )
		UBITRACK_THROW( "list size not 1" );
	m_outPort.send( Measurement::Position2D( t, m_inPort.get()->at( 0 ) ) );
}

template<>
void CastComponent< Measurement::Pose, Measurement::Matrix3x4 >::compute( Measurement::Timestamp t )
{
	Math::Matrix< double, 3, 4 > tmpMatrix(*m_inPort.get());
	m_outPort.send( Measurement::Matrix3x4( t, tmpMatrix ) );
}


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< CastComponent< Measurement::ErrorPose, Measurement::Pose > > ( "CastErrorPose2Pose" );
	cf->registerComponent< CastComponent< Measurement::Pose, Measurement::Position > > ( "CastPose2Position" );
	cf->registerComponent< CastComponent< Measurement::Pose, Measurement::Rotation > > ( "CastPose2Rotation" );
	cf->registerComponent< CastComponent< Measurement::PositionList, Measurement::Position > > ( "CastPositionList2Position" );
	cf->registerComponent< CastComponent< Measurement::PositionList2, Measurement::Position2D > > ( "Cast2DPositionList22DPosition" );
	cf->registerComponent< CastComponent< Measurement::Pose, Measurement::Matrix3x4 > > ( "CastPose2Matrix3x4" );
}

} } // namespace Ubitrack::Components
