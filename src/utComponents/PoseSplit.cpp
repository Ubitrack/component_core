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
 * Components that splits a pose into rotation and translation components
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */
#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.PoseSplit" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Components that splits a pose into rotation and translation components
 *
 * @par Input Ports
 * PushConsumer< Pose > with name "Input".
 *
 * @par Output Ports
 * PushSupplier<Rotation> with name "Rotation".
 * PushSupplier<Position> with name "Translation".
 *
 * @par Configuration
 * None.
 *
 * @par Operation
 *
 * @par Instances
 */
class PoseSplitComponent
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	PoseSplitComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  )
		: Dataflow::Component( sName )
		, m_inPort( "Input", *this, boost::bind( &PoseSplitComponent::receivePose, this, _1 ) )
		, m_outRotation( "Rotation", *this )
		, m_outTranslation( "Translation", *this )
    {
		LOG4CPP_ERROR( logger, "This Component is deprecated. Please use \"CastPose2Position\" and \"CastPose2Rotation\" instead." );
    }

	/** Method that computes the result. */
	void receivePose( const Measurement::Pose& p )
	{
		m_outRotation.send( Measurement::Rotation( p.time(), p->rotation() ) );
		m_outTranslation.send( Measurement::Position( p.time(), p->translation() ) );
    }

protected:
	/** Input port of the component. */
	Dataflow::PushConsumer< Measurement::Pose > m_inPort;

	// Output ports of the component
	Dataflow::PushSupplier< Measurement::Rotation > m_outRotation;
	Dataflow::PushSupplier< Measurement::Position > m_outTranslation;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< PoseSplitComponent > ( "PoseSplit" );
}

} } // namespace Ubitrack::Components
