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
 * Component for kalman filtering with events arriving out of order through network
 *
 * @author Frieder Pankratz <pankratz@in.tum.de>
 */

#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <log4cpp/Category.hh>
#include <boost/numeric/ublas/io.hpp>
#include <sstream>
#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/circular_buffer.hpp>

#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utTracking/OutOfOrderPoseKalmanFilter.h>

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
class OutOfOrderPoseKalmanFilterComponent
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	OutOfOrderPoseKalmanFilterComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Dataflow::Component( sName )
		, m_out( "OutPose", *this, boost::bind( &OutOfOrderPoseKalmanFilterComponent::sendOut, this, _1 ) )
		, m_outPush( "OutPosePush", *this )
    {
		// dynamically generate input ports
		int index = 0;
		for ( Graph::UTQLSubgraph::EdgeMap::iterator it = subgraph->m_Edges.begin(); it != subgraph->m_Edges.end(); it++ ) 
		{
			if ( it->second->isInput() ) 
			{
				if ( 0 == it->first.compare( 0, 6, "InPose" ) )
					m_inPosePorts.push_back( boost::shared_ptr< Dataflow::PushConsumer< Measurement::ErrorPose > >( 
						new Dataflow::PushConsumer< Measurement::ErrorPose >( it->first, *this, 
							boost::bind( &OutOfOrderPoseKalmanFilterComponent::receivePose, this, _1, index++) ) ) );
				/*else if ( 0 == it->first.compare( 0, 18, "InRotationVelocity" ) )
					m_inRotationVelocityPorts.push_back( boost::shared_ptr< Dataflow::PushConsumer< Measurement::RotationVelocity > >( 
						new Dataflow::PushConsumer< Measurement::RotationVelocity >( it->first, *this, 
							boost::bind( &OutOfOrderPoseKalmanFilterComponent::receiveRotationVelocity, this, _1 ) ) ) );
				else if ( 0 == it->first.compare( 0, 10, "InRotation" ) )
					m_inRotationPorts.push_back( boost::shared_ptr< Dataflow::PushConsumer< Measurement::Rotation > >( 
						new Dataflow::PushConsumer< Measurement::Rotation >( it->first, *this, 
							boost::bind( &OutOfOrderPoseKalmanFilterComponent::receiveRotation, this, _1 ) ) ) );
				else if ( 0 == it->first.compare( 0, 25, "InInverseRotationVelocity" ) )
					m_inInverseRotationVelocityPorts.push_back( boost::shared_ptr< Dataflow::PushConsumer< Measurement::RotationVelocity > >( 
						new Dataflow::PushConsumer< Measurement::RotationVelocity >( it->first, *this, 
							boost::bind( &OutOfOrderPoseKalmanFilterComponent::receiveInverseRotationVelocity, this, _1 ) ) ) );*/
			}
		}
	
		// generate motion model
		
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
				m_posPN.push_back( d );
		}

		// read rot process noise
		if ( subgraph->m_DataflowAttributes.hasAttribute( "oriPN" ) )
			sPN = subgraph->m_DataflowAttributes.getAttributeString( "oriPN" );
		else
			sPN = "0.07 3.6";

		{
			std::istringstream inStream( sPN );
			while ( inStream >> d )
				m_oriPN.push_back( d );
		}

		m_bInsideOut = subgraph->m_DataflowAttributes.getAttributeString( "insideOut" ) == "true";
	

		// initialize kalman filter 
		m_pKF.reset( new Tracking::OutOfOrderPoseKalmanFilter(30, m_posPN, m_oriPN,m_bInsideOut  ) );



		// create ofstream
		std::string sFilename = "KalmanLog.txt";



		m_pStream.reset(new std::ofstream(sFilename.c_str()));
		if (!m_pStream->good())
			UBITRACK_THROW("Could not open file " + sFilename + " for writing");

		// create oarchive
		std::string sFilenameOut = "KalmanOutputLog";
		for (int i = 0; i < m_posPN.size(); i++) {
			sFilenameOut += "_" + std::to_string(m_posPN[i]);
		}
		sFilenameOut += ".txt";

		m_pStreamPose.reset(new std::ofstream(sFilenameOut.c_str()));
		if (!m_pStreamPose->good())
			UBITRACK_THROW("Could not open file " + sFilenameOut + " for writing");
		m_pArchive.reset(new boost::archive::text_oarchive(*m_pStreamPose));


		std::string sFilenameInOut = "KalmanInputLog.txt";
		m_pStreamPoseInput.reset(new std::ofstream(sFilenameInOut.c_str()));
		if (!m_pStreamPoseInput->good())
			UBITRACK_THROW("Could not open file " + sFilenameInOut + " for writing");
		m_pArchiveInput.reset(new boost::archive::text_oarchive(*m_pStreamPoseInput));

    }

	/** integrates a pose measurement. */
	void receivePose( const Measurement::ErrorPose& m, int index)
	{				
		Measurement::Timestamp tsNow = Measurement::now();
		Measurement::Timestamp delay = tsNow - m.time();
		delay = delay / 1000000l;

		
		std::string linesep("\n");
		(*m_pStream) << linesep;
		(*m_pStream) << index;
		(*m_pStream) << " ";
		(*m_pStream) << tsNow;
		(*m_pStream) << " ";
		(*m_pStream) << m.time();

		(*m_pArchiveInput) << linesep;
		(*m_pArchiveInput) << m;

		/*int maxDelay[4] = { 40,30,30,30 };
		if (delay > maxDelay[index]) {
			LOG4CPP_WARN(logger, "delay of input " << index << " too big, reject " << delay);
			return;
		}*/
		

		m_pKF->addPoseMeasurement(m, index);
		//checkSend(m.time());				
    }

	/** Method that returns a predicted measurement. */
	Measurement::ErrorPose sendOut( Measurement::Timestamp t )
	{
		
		Measurement::Timestamp tsNow = Measurement::now();


		Measurement::ErrorPose result = m_pKF->predictPose(t);

		std::string linesep("\n");
		(*m_pStream) << linesep;
		(*m_pStream) << -1;
		(*m_pStream) << " ";
		(*m_pStream) << tsNow;
		(*m_pStream) << " ";
		(*m_pStream) << t;


		(*m_pArchive) << linesep;
		(*m_pArchive) << result;

		return result;
	}

	virtual void stop()
	{
		m_running = false;		
	}

protected:
	/** sends a measurement to connected push consumers */
	void checkSend( Measurement::Timestamp t )
	{
		if ( m_outPush.isConnected() )
		{
			
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
	boost::scoped_ptr< Tracking::OutOfOrderPoseKalmanFilter > m_pKF;
	

	std::vector< double > m_posPN;
	std::vector< double > m_oriPN;
	bool m_bInsideOut;



	/** output stream */
	boost::scoped_ptr< std::ofstream > m_pStream;

	boost::scoped_ptr< std::ofstream > m_pStreamPose;
	/** output archive */
	boost::scoped_ptr< boost::archive::text_oarchive > m_pArchive;

	boost::scoped_ptr< std::ofstream > m_pStreamPoseInput;
	boost::scoped_ptr< boost::archive::text_oarchive > m_pArchiveInput;
	
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< OutOfOrderPoseKalmanFilterComponent > ( "OutOfOrderPoseKalmanFilter" );
}

} } // namespace Ubitrack::Components
