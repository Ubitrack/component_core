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
 * Component for functional complementary fusion.
 * This file contains a combination of Position and Rotation into Pose as a \c TriggerComponent.
 *
 * @author Florian Echtler <echtler@in.tum.de>
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Component for functional complementary fusion.
 * This file contains a combination of Position and Rotation into Pose as a \c TriggerComponent.
 *
 * @par Input Ports
 * TriggerInPort<Position> with name "PositionInput".
 * TriggerInPort<Rotation> with name "RotationInput".
 *
 * @par Output Ports
 * TriggerOutPort<Pose> with name "Output".
 *
 * @par Configuration
 * None.
 *
 * @par Operation
 * The component combines requested/incoming Position and Rotation events into a Pose.
 */
class FunctionalFusionComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	FunctionalFusionComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_posPort( "PositionInput", *this )
		, m_rotPort( "RotationInput", *this )
		, m_outPort( "Output", *this )
	{
	}

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		m_outPort.send( Measurement::Pose( t, Math::Pose( *m_rotPort.get(), *m_posPort.get() ) ) );
	}

protected:
	/** Position input port of the component. */
	Dataflow::TriggerInPort< Measurement::Position > m_posPort;

	/** Rotation input port of the component. */
	Dataflow::TriggerInPort< Measurement::Rotation > m_rotPort;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Pose > m_outPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< FunctionalFusionComponent > ( "FunctionalFusion" );
}

} } // namespace Ubitrack::Components
