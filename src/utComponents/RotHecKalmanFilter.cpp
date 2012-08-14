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
 * Component for online compuation of a rotation-only Hand-Eye Calibration
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#include <log4cpp/Category.hh>
#include <boost/bind.hpp>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utCalibration/RotationHecKalmanFilter.h>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.RotHecKalmanFilter" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Component for online compuation of a rotation-only Hand-Eye Calibration
 *
 * @par Input Ports
 * TriggerInPort< Rotation > with name "InA".
 * TriggerInPort< Rotation > with name "InB".
 *
 * Both inputs expect absolute orientations!
 *
 * @par Output Ports
 * PullSupplier< Rotation > with name "Output".
 *
 * @par Configuration
 * None.
 *
 * @par Operation
 * Computes the quaternion X, such that AX = XB, if A and B are relative orientations
 *
 * @par Instances
 */
class RotHecKalmanFilterComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	RotHecKalmanFilterComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_lastTime( 0 )
		, m_maxTime( 5000000000LL )
		, m_minTime( 500000000LL )
		, m_inA( "InA", *this )
		, m_inB( "InB", *this )
		, m_out( "Output", *this, boost::bind( &RotHecKalmanFilterComponent::sendOut, this, _1 ) )
    {
		// make it call compute() for every measurement
		addTriggerOutput( true );
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		Math::Quaternion qa( *m_inA.get() );
		Math::Quaternion qb( *m_inB.get() );

		Math::Quaternion deltaA( (~qa) * m_lastA );
		Math::Quaternion deltaB( (~qb) * m_lastB );

		// 0.99 is about 10 deg
		bool bValidAngle = ( ( fabs( deltaA.w() ) < 0.99 ) || ( fabs( deltaB.w() ) < 0.99 ) );

		if ( t < m_lastTime + m_minTime )
		{
			// time too short between measurements
			LOG4CPP_TRACE( logger, "time between measurements too short" );
			return;
		}

		LOG4CPP_TRACE( logger, "Received measurements: " << qa << ", " << qb );

		if ( t < m_lastTime + m_maxTime )
		{
			// time between minTime and maxTime => do computation

			if ( bValidAngle )
			{
				m_kf.addMeasurement( deltaA, deltaB );
				LOG4CPP_DEBUG( logger, "Computed transformation " << m_kf.getResult() );
			}
			else
			{
				LOG4CPP_TRACE( logger, "angle between measurements too small" );
			}
		}
		else
		{ LOG4CPP_TRACE( logger, "time between measurements too long" ); }

		// store measurements
		m_lastA = qa;
		m_lastB = qb;
		m_lastTime = t;
    }

	Measurement::Rotation sendOut( Measurement::Timestamp t )
	{
		return Measurement::Rotation( t, m_kf.getResult() );
	}

protected:
	// store the last measurements
	Math::Quaternion m_lastA, m_lastB;

	// last measurement time
	Measurement::Timestamp m_lastTime;

	// maximum time between two measurements
	Measurement::Timestamp m_maxTime;

	// minimum time between two measurements
	Measurement::Timestamp m_minTime;

	// Input ports of the component.
	Dataflow::TriggerInPort< Measurement::Rotation > m_inA;
	Dataflow::TriggerInPort< Measurement::Rotation > m_inB;

	// Output ports of the component
	Dataflow::PullSupplier< Measurement::Rotation > m_out;

	// the kalman filter
	Calibration::RotationHecKalmanFilter m_kf;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< RotHecKalmanFilterComponent > ( "RotHecKalmanFilter" );
}

} } // namespace Ubitrack::Components

