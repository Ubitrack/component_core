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
 * Distance component.
 * This file contains the computation of distance between two inputs,
 * implemented as a \c TriggerComponent.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

namespace ublas = boost::numeric::ublas;

namespace Ubitrack { namespace Components {

// computes the distance for vectors
template< std::size_t N, typename T >
double doDistance( const Math::Vector< T, N >& a, const Math::Vector< T, N >& b )
{
	return ublas::norm_2( a - b );
}


// computes the distance for poses
double doDistance( const Math::Pose& a, const Math::Pose& b )
{
	return ublas::norm_2( a.translation() - b.translation() );
}
	

/**
 * @ingroup dataflow_components
 * Distance component.
 * This class contains the computation of distance between two inputs,
 * implemented as a \c TriggerComponent.
 *
 * @par Input Ports
 * TriggerInPort<EventType> with name "InputA".
 * TriggerInPort<EventType> with name "InputB".
 *
 * @par Output Ports
 * TriggerOutPort<EventType> with name "Output".
 *
 * @par Configuration
 * None.
 *
 * @par Operation
 * The component the distance between incoming/requested events using
 * len( A - B ). Only the position is taken into account for poses.
 */
template< class EventType >
class DistanceComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	DistanceComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPortA( "InputA", *this )
		, m_inPortB( "InputB", *this )
		, m_outPort( "Output", *this )
	{
	}

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		double d = doDistance( *m_inPortA.get(), *m_inPortB.get() );
		Measurement::Distance dist( t, d );
		m_outPort.send( dist );
	}

protected:
	/** Input port A of the component. */
	Dataflow::TriggerInPort< EventType > m_inPortA;

	/** Input port B of the component. */
	Dataflow::TriggerInPort< EventType > m_inPortB;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Distance > m_outPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< DistanceComponent< Measurement::Pose > > ( "PoseDistance" );
	cf->registerComponent< DistanceComponent< Measurement::Position > > ( "PositionDistance" );
}

} } // namespace Ubitrack::Components
