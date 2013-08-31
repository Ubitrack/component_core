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
 * Component for kalman filtering
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <log4cpp/Category.hh>
#include <boost/numeric/ublas/io.hpp>
#include <sstream>

#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utTracking/PoseKalmanFilter.h>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.PoseKalmanFilter" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Component for kalman filtering
 *
 * @par Input Ports
 * - PushConsumer< ErrorPose > with name "InPose".
 * - PushConsumer< Rotation > with name "InRotation".
 * - PushConsumer< RotationVelocity > with name "InRotationVelocity".
 * - PushConsumer< RotationVelocity > with name "InInverseRotationVelocity".
 *
 * Note: Additional input ports can be generated using arbitrary edge names 
 * starting with "InPose", "InRotation", ...
 *
 * @par Output Ports
 * PullSupplier< ErrorPose > with name "Output".
 *
 * @par Configuration
 * - DataflowConfiguration Attribute "posPN": sequence of floats
 * - DataflowConfiguration Attribute "oriPN": sequence of floats
 * - DataflowConfiguration Attribute "insideOut": "true"/"false"
 *
 * @par Operation
 * integrates absolute and relative measurements. relative measurements must be calibrated before!
 * Make sure timestamps are reasonably correct!
 */
class PoseKalmanFilterComponent
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	PoseKalmanFilterComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Dataflow::Component( sName )
		, m_out( "OutPose", *this, boost::bind( &PoseKalmanFilterComponent::sendOut, this, _1 ) )
		, m_outPush( "OutPosePush", *this )
    {
		// dynamically generate input ports
		for ( Graph::UTQLSubgraph::EdgeMap::iterator it = subgraph->m_Edges.begin(); it != subgraph->m_Edges.end(); it++ ) 
		{
			if ( it->second->isInput() ) 
			{
				if ( 0 == it->first.compare( 0, 6, "InPose" ) )
					m_inPosePorts.push_back( boost::shared_ptr< Dataflow::PushConsumer< Measurement::ErrorPose > >( 
						new Dataflow::PushConsumer< Measurement::ErrorPose >( it->first, *this, 
							boost::bind( &PoseKalmanFilterComponent::receivePose, this, _1 ) ) ) );
				else if ( 0 == it->first.compare( 0, 18, "InRotationVelocity" ) )
					m_inRotationVelocityPorts.push_back( boost::shared_ptr< Dataflow::PushConsumer< Measurement::RotationVelocity > >( 
						new Dataflow::PushConsumer< Measurement::RotationVelocity >( it->first, *this, 
							boost::bind( &PoseKalmanFilterComponent::receiveRotationVelocity, this, _1 ) ) ) );
				else if ( 0 == it->first.compare( 0, 10, "InRotation" ) )
					m_inRotationPorts.push_back( boost::shared_ptr< Dataflow::PushConsumer< Measurement::Rotation > >( 
						new Dataflow::PushConsumer< Measurement::Rotation >( it->first, *this, 
							boost::bind( &PoseKalmanFilterComponent::receiveRotation, this, _1 ) ) ) );
				else if ( 0 == it->first.compare( 0, 25, "InInverseRotationVelocity" ) )
					m_inInverseRotationVelocityPorts.push_back( boost::shared_ptr< Dataflow::PushConsumer< Measurement::RotationVelocity > >( 
						new Dataflow::PushConsumer< Measurement::RotationVelocity >( it->first, *this, 
							boost::bind( &PoseKalmanFilterComponent::receiveInverseRotationVelocity, this, _1 ) ) ) );
			}
		}
	
		// generate motion model
		std::vector< double > posPN;
		std::vector< double > oriPN;
		double d;
		std::string sPN;

		// read pos process noise
		if ( subgraph->m_DataflowAttributes.hasAttribute( "posPN" ) )
			sPN = subgraph->m_DataflowAttributes.getAttributeString( "posPN" );
		else
			sPN = "0.6";

		{
			std::istringstream inStream( sPN );
			while ( inStream >> d )
				posPN.push_back( d );
		}

		// read rot process noise
		if ( subgraph->m_DataflowAttributes.hasAttribute( "oriPN" ) )
			sPN = subgraph->m_DataflowAttributes.getAttributeString( "oriPN" );
		else
			sPN = "0.07 3.6";

		{
			std::istringstream inStream( sPN );
			while ( inStream >> d )
				oriPN.push_back( d );
		}

		bool bInsideOut = subgraph->m_DataflowAttributes.getAttributeString( "insideOut" ) == "true";

		// create the motion model
		Tracking::LinearPoseMotionModel motionModel( posPN.size() - 1, oriPN.size() - 1 );
		for ( std::size_t i( 0 ); i < posPN.size(); i++ )
			motionModel.setPosPN( i, posPN[ i ] );
		for ( std::size_t i( 0 ); i < oriPN.size(); i++ )
			motionModel.setOriPN( i, oriPN[ i ] );

		// initialize kalman filter with motion model
		m_pKF.reset( new Tracking::PoseKalmanFilter( motionModel, bInsideOut ) );
    }

	/** integrates a pose measurement. */
	void receivePose( const Measurement::ErrorPose& m )
	{
		LOG4CPP_DEBUG( logger, "Received pose measurement: " << m );
		LOG4CPP_TRACE( logger, "state before: " << m_pKF->getState() << std::endl << m_pKF->getCovariance() );

		m_pKF->addPoseMeasurement( m );

		checkSend( m.time() );

		LOG4CPP_TRACE( logger, "state after: " << m_pKF->getState() << std::endl << m_pKF->getCovariance() );
    }

	/** integrates a rotation measurement. */
	void receiveRotation( const Measurement::Rotation& m )
	{
		LOG4CPP_DEBUG( logger, "Received rotation measurement: " << m );
		LOG4CPP_TRACE( logger, "state before: " << m_pKF->getState() << std::endl << m_pKF->getCovariance() );

		m_pKF->addRotationMeasurement( m );

		checkSend( m.time() );

		LOG4CPP_TRACE( logger, "state after: " << m_pKF->getState() << std::endl << m_pKF->getCovariance() );
    }

	/** integrates a rotation velocity measurement. */
	void receiveRotationVelocity( const Measurement::RotationVelocity& m )
	{
		LOG4CPP_DEBUG( logger, "Received rotation velocity measurement: " << m );
		LOG4CPP_TRACE( logger, "state before: " << m_pKF->getState() << std::endl << m_pKF->getCovariance() );

		m_pKF->addRotationVelocityMeasurement( m );

		checkSend( m.time() );

		LOG4CPP_TRACE( logger, "state after: " << m_pKF->getState() << std::endl << m_pKF->getCovariance() );
    }

	/** integrates an inverse rotation velocity measurement. */
	void receiveInverseRotationVelocity( const Measurement::RotationVelocity& m )
	{
		LOG4CPP_DEBUG( logger, "Received inverse rotation velocity measurement: " << m );
		LOG4CPP_TRACE( logger, "state before: " << m_pKF->getState() << std::endl << m_pKF->getCovariance() );

		m_pKF->addInverseRotationVelocityMeasurement( m );

		checkSend( m.time() );

		LOG4CPP_TRACE( logger, "state after: " << m_pKF->getState() << std::endl << m_pKF->getCovariance() );
    }

	/** Method that returns a predicted measurement. */
	Measurement::ErrorPose sendOut( Measurement::Timestamp t )
	{
		LOG4CPP_DEBUG( logger, "Computing pose for t=" << t );
		LOG4CPP_TRACE( logger, "state: " << m_pKF->getState() << std::endl << m_pKF->getCovariance() );

		return m_pKF->predictPose( t );
	}

protected:
	/** sends a measurement to connected push consumers */
	void checkSend( Measurement::Timestamp t )
	{
		if ( m_outPush.isConnected() )
		{
			LOG4CPP_TRACE( logger, "Checking whether  to send pose" );
			
			// only send when there are no more queued events
			for ( std::size_t i( 0 ); i < m_inPosePorts.size(); i++ )
				if ( m_inPosePorts[ i ]->getQueuedEvents() > 0 )
					return;
			for ( std::size_t i( 0 ); i < m_inRotationPorts.size(); i++ )
				if ( m_inRotationPorts[ i ]->getQueuedEvents() > 0 )
					return;
			for ( std::size_t i( 0 ); i < m_inRotationVelocityPorts.size(); i++ )
				if ( m_inRotationVelocityPorts[ i ]->getQueuedEvents() > 0 )
					return;
			for ( std::size_t i( 0 ); i < m_inInverseRotationVelocityPorts.size(); i++ )
				if ( m_inInverseRotationVelocityPorts[ i ]->getQueuedEvents() > 0 )
					return;

			LOG4CPP_TRACE( logger, "Sending pose" );
			m_outPush.send( m_pKF->predictPose( t ) );
		}
	}

	// Input ports of the component.
	std::vector< boost::shared_ptr< Dataflow::PushConsumer< Measurement::ErrorPose > > > m_inPosePorts;
	std::vector< boost::shared_ptr< Dataflow::PushConsumer< Measurement::Rotation > > > m_inRotationPorts;
	std::vector< boost::shared_ptr< Dataflow::PushConsumer< Measurement::RotationVelocity > > > m_inRotationVelocityPorts;
	std::vector< boost::shared_ptr< Dataflow::PushConsumer< Measurement::RotationVelocity > > > m_inInverseRotationVelocityPorts;

	// Output ports of the component
	Dataflow::PullSupplier< Measurement::ErrorPose > m_out;
	Dataflow::PushSupplier< Measurement::ErrorPose > m_outPush;


	// the kalman filter
	boost::scoped_ptr< Tracking::PoseKalmanFilter > m_pKF;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< PoseKalmanFilterComponent > ( "PoseKalmanFilter" );
}

} } // namespace Ubitrack::Components
