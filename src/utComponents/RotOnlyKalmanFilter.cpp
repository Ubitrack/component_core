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
 * Component for kalman filtering of orientations
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#include <boost/bind.hpp>
#include <log4cpp/Category.hh>

#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utTracking/RotationOnlyKF.h>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.RotOnlyKalmanFilter" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Component for kalman filtering of orientations
 *
 * @par Input Ports
 * PushConsumer< Rotation > with name "InAbsolute".
 * PushConsumer< RotationVelocity > with name "InVelocity".
 *
 * @par Output Ports
 * PullSupplier< Rotation > with name "Output".
 *
 * @par Configuration
 * None.
 *
 * @par Operation
 * integrates absolute and relative measurements. relative measurements must be calibrated before!
 * Make sure timestamps are reasonably correct!
 */
class RotOnlyKalmanFilterComponent
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	RotOnlyKalmanFilterComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  )
		: Dataflow::Component( sName )
		, m_inAbsolute( "InAbsolute", *this, boost::bind( &RotOnlyKalmanFilterComponent::receiveAbsolute, this, _1 ) )
		, m_inVelocity( "InVelocity", *this, boost::bind( &RotOnlyKalmanFilterComponent::receiveVelocity, this, _1 ) )
		, m_out( "Output", *this, boost::bind( &RotOnlyKalmanFilterComponent::sendOut, this, _1 ) )
    {
    }

	/** integrates an absolute measurement. */
	void receiveAbsolute( const Measurement::Rotation& m )
	{
		LOG4CPP_DEBUG( logger, "Received absolute measurement: " << m );
		LOG4CPP_TRACE( logger, "state before: " << m_kf.getState() );

		m_kf.addRotationMeasurement( m );

		LOG4CPP_DEBUG( logger, "computed state: " << m_kf.getState() );
    }

	/** integrates an absolute measurement. */
	void receiveVelocity( const Measurement::RotationVelocity& m )
	{
		LOG4CPP_DEBUG( logger, "Received velocity measurement: " << m );
		LOG4CPP_TRACE( logger, "state before: " << m_kf.getState() );

		m_kf.addVelocityMeasurement( m );

		LOG4CPP_DEBUG( logger, "computed state: " << m_kf.getState() );
    }

	/** Method that returns a predicted measurement. */
	Measurement::Rotation sendOut( Measurement::Timestamp t )
	{
		LOG4CPP_DEBUG( logger, "Computing rotation for t=" << t );
		LOG4CPP_TRACE( logger, "state: " << m_kf.getState() );

		return m_kf.predict( t );
	}

protected:
	// Input ports of the component.
	Dataflow::PushConsumer< Measurement::Rotation > m_inAbsolute;
	Dataflow::PushConsumer< Measurement::RotationVelocity > m_inVelocity;

	// Output ports of the component
	Dataflow::PullSupplier< Measurement::Rotation > m_out;

	// the kalman filter
	Tracking::RotationOnlyKF m_kf;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< RotOnlyKalmanFilterComponent > ( "RotOnlyKalmanFilter" );
}

} } // namespace Ubitrack::Components
