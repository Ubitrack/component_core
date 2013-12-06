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
 * This file contains an inversion implemented as a \c TriggerComponent.
 *
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
 * Inversion component.
 * This class contains an inversion implemented as a \c TriggerComponent.
 *
 * The component inverts requested/incoming events using operator~
 */
template< class EventType >
class InversionComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	InversionComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPort( "AB", *this )
		, m_outPort( "BA", *this )
    {
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		m_outPort.send( EventType( t, ~*m_inPort.get() ) );
	}

protected:
	/** Input port of the component. */
	Dataflow::TriggerInPort< EventType > m_inPort;
	/** Output port of the component. */
	Dataflow::TriggerOutPort< EventType > m_outPort;
};


/** inversion operator for position = negation */
Math::Vector< double, 3 > operator~( const Math::Vector< double, 3 >& op )
{
	return Math::Vector< double, 3 >( -op );
}

/** inversion operator for batch inversion of many pose vectors  */
std::vector< Math::Vector< double, 3 > > operator~( const std::vector< Math::Vector< double, 3 > >& p3d )
{
	std::vector< Math::Vector< double, 3 > > result( p3d.size() );
	for ( unsigned i = 0; i < p3d.size(); i++ )
		result[ i ] = ~p3d[ i ];
	return result;
}

/** inversion operator for batch inversion of many pose vectors  */
std::vector< Math::Pose > operator~( const std::vector< Math::Pose >& p6d )
{
	std::vector< Math::Pose > result( p6d.size() );
	for ( unsigned i = 0; i < p6d.size(); i++ )
		result[ i ] = ~p6d[ i ];
	return result;
}



UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< InversionComponent< Measurement::Pose > > ( "PoseInversion" );
	cf->registerComponent< InversionComponent< Measurement::ErrorPose > > ( "ErrorPoseInversion" );
	cf->registerComponent< InversionComponent< Measurement::Rotation > > ( "RotationInversion" );
	cf->registerComponent< InversionComponent< Measurement::Position > > ( "PositionInversion" );
	cf->registerComponent< InversionComponent< Measurement::PoseList > > ( "PoseListInversion" );
	cf->registerComponent< InversionComponent< Measurement::PositionList > > ( "PositionListInversion" );
}

} } // namespace Ubitrack::Components
