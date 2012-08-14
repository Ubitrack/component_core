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
 * Components that does nothing, but pass events unmodified.
 * This component is only needed in some server-based scenarios and should not be instantiated manually.
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
 * Identity component.
 * This class contains an inversion implemented as a \c TriggerComponent.
 *
 * The component passes on requested/incoming events unmodified.
 */
template< class EventType >
class IdentityComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	IdentityComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPort( "Input", *this )
		, m_outPort( "Output", *this )
    {
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp )
	{
		m_outPort.send( m_inPort.get() );
	}

protected:
	/** Input port of the component. */
	Dataflow::TriggerInPort< EventType > m_inPort;
	/** Output port of the component. */
	Dataflow::TriggerOutPort< EventType > m_outPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< IdentityComponent< Measurement::Pose > > ( "PoseIdentity" );
	cf->registerComponent< IdentityComponent< Measurement::Rotation > > ( "RotationIdentity" );
	cf->registerComponent< IdentityComponent< Measurement::Position > > ( "PositionIdentity" );
}

} } // namespace Ubitrack::Components
