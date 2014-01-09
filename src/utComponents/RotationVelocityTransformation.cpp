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
 * Multiplication component.
 * This file contains a multiplication of two inputs implemented as a \c TriggerComponent.
 *
 * @author Michael Schlegel <schlegem@in.tum.de>
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Multiplication component.
 * This class contains a multiplication of a rotation velocity and a rotation implemented as a \c TriggerComponent.
 *
 * The component multiplies requested/incoming events using operator*( A, B )
 */
template< class EventTypeA, class EventTypeB, class EventTypeOut >
class MultiplicationComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	MultiplicationComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPortA( "AB", *this )
		, m_inPortB( "BC", *this )
		, m_outPort( "AC", *this )
    {
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		m_outPort.send( EventTypeOut( t, *m_inPortA.get() * *m_inPortB.get() ) );
	}

protected:
	/** Input port A of the component. */
	Dataflow::TriggerInPort< EventTypeA > m_inPortA;

	/** Input port B of the component. */
	Dataflow::TriggerInPort< EventTypeB > m_inPortB;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< EventTypeOut > m_outPort;
};


/** multiplication operator for batch multiplication of many vectors with a pose */
std::vector< Math::Vector< double, 3 > > operator*( const Math::Pose& pose, const std::vector< Math::Vector< double, 3 > >& p3d )
{
	std::vector< Math::Vector< double, 3 > > result( p3d.size() );
	for ( unsigned i = 0; i < p3d.size(); i++ )
		result[ i ] = pose * p3d[ i ];
	return result;
}

/** multiplication operator for position "multiplication" (= addition) */
Math::Vector< double, 3 > operator*( const Math::Vector< double, 3 >& pos1, const Math::Vector< double, 3 >& pos2 )
{
	Math::Vector< double, 3 > result;
	result = pos1 + pos2;
	return result;
}


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {

	// Rotation * RotationVelocity = RotationVelocity
	cf->registerComponent< MultiplicationComponent< Measurement::Rotation, Measurement::RotationVelocity, Measurement::RotationVelocity > > ( "RotationVelocityTransformation" );


}

} } // namespace Ubitrack::Components
