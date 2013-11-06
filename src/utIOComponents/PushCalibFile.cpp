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
 * @ingroup driver_components
 * @file
 * Components that write/read a static measurement to/from a file. Supplies push event when dataflow starts
 *
 * @author Frieder Pankratz <pankratz@in.tum.de>
 */
 
#include <fstream>
#include <boost/bind.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/thread.hpp>
#include <utMeasurement/Measurement.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/PushSupplier.h>
#include <utUtil/CalibFile.h>

using namespace Ubitrack::Dataflow;

namespace Ubitrack { namespace Drivers {

/**
 * @ingroup driver_components
 * PushCalibReaderWriter components, writes a static measurement to a file.
 * Only the last incoming measurement is stored!
 *
 * @par Input Ports
 * PushConsumer< EventType > with name "Input".
 *
 * @par Output Ports
 * None.
 *
 * @par Configuration
@verbatim
<Configuration file="<filename>"/>
@endverbatim
 *
 */
template< class EventType >
class PushCalibReaderWriter
	: public Component
{
public:
	/** open the file */
	PushCalibReaderWriter( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Component( name )
		, m_inPort( "Input", *this, boost::bind( &PushCalibReaderWriter::eventIn, this, _1 ) )
		, m_lastTS( 0 )
		, m_lastMeasurement( typename EventType::value_type() )
		, m_outPort( "Output", *this )
		, m_outPortPull( "OutputPull", *this, boost::bind( &PushCalibReaderWriter::request, this, _1 ) )
	{
		// read filename from configuration
		m_sFilename = subgraph->m_DataflowAttributes.getAttributeString( "file" );
		if ( m_sFilename.empty() )
			UBITRACK_THROW( "No \"file\" attribute for PushCalibReaderWriter component " + name );


		// read measurement
		try{
			Util::readCalibFile( m_sFilename, m_lastMeasurement );
			m_lastTS = m_lastMeasurement.time();
		}catch(Ubitrack::Util::Exception e) {
			// file invalid or does not exist
		}
	}

	/** Saves the last received measurement in case it was not yet written to disk */
	~PushCalibReaderWriter()
	{
		// if the last measurement was not written to disk, do it now
		boost::mutex::scoped_lock l( m_fileMutex );
		if ( m_lastMeasurement )
			Util::writeCalibFile( m_sFilename, m_lastMeasurement );
	}

	virtual void start(){
		Component::start();
		if(m_lastTS > 0)
			m_outPort.send(EventType( Measurement::now(), *m_lastMeasurement ));
	}
	
protected:
	/** output port */
	Dataflow::PushConsumer< EventType > m_inPort;

	/** file name of configuration */
	std::string m_sFilename;

	/** saves the timestamp of the last saved measurement */
	Measurement::Timestamp m_lastTS;

	/** saves the last measurement in case it was not written to disk */
	EventType m_lastMeasurement;

	/** mutex to protect file from simultaneous writes */
	boost::mutex m_fileMutex;

	/** handler method for incoming events */
	void eventIn( const EventType& event );
	
	/** minimum time between disk writes */
	static const Measurement::Timestamp s_minWriteDistance = 975000000LL;

	/** output port */	
	Dataflow::PushSupplier< EventType > m_outPort;
	Dataflow::PullSupplier< EventType > m_outPortPull;

	/** handler method for incoming pull requests */
	EventType request( Measurement::Timestamp t )
	{ return EventType( t, *m_lastMeasurement ); }
};


template< class EventType >
void PushCalibReaderWriter< EventType >::eventIn( const EventType& n )
{
	// lock the file to prevent other threads from writing simultaneously
	boost::mutex::scoped_lock l( m_fileMutex );
	m_lastMeasurement = n;
	m_outPort.send(m_lastMeasurement);
	if ( m_lastTS + s_minWriteDistance < n.time() )		{

		Util::writeCalibFile( m_sFilename, n );	
		m_lastTS = n.time();
	}
	
}




} } // namespace Ubitrack::Drivers


UBITRACK_REGISTER_COMPONENT( Ubitrack::Dataflow::ComponentFactory* const cf )
{
	// // cf->registerComponent< Ubitrack::Drivers::CalibWriterXML< Ubitrack::Measurement::Pose > > ( "PoseCalibWriterXML" );

	cf->registerComponent< Ubitrack::Drivers::PushCalibReaderWriter< Ubitrack::Measurement::Distance > > ( "DistancePushCalibReaderWriter" );
	cf->registerComponent< Ubitrack::Drivers::PushCalibReaderWriter< Ubitrack::Measurement::Pose > > ( "PosePushCalibReaderWriter" );
	cf->registerComponent< Ubitrack::Drivers::PushCalibReaderWriter< Ubitrack::Measurement::ErrorPose > > ( "ErrorPosePushCalibReaderWriter" );
	cf->registerComponent< Ubitrack::Drivers::PushCalibReaderWriter< Ubitrack::Measurement::ErrorPosition > > ( "ErrorPositionPushCalibReaderWriter" );
	cf->registerComponent< Ubitrack::Drivers::PushCalibReaderWriter< Ubitrack::Measurement::Position > > ( "PositionPushCalibReaderWriter" );
	cf->registerComponent< Ubitrack::Drivers::PushCalibReaderWriter< Ubitrack::Measurement::Rotation > > ( "RotationPushCalibReaderWriter" );
	cf->registerComponent< Ubitrack::Drivers::PushCalibReaderWriter< Ubitrack::Measurement::Matrix3x3 > > ( "Matrix3x3PushCalibReaderWriter" );
	cf->registerComponent< Ubitrack::Drivers::PushCalibReaderWriter< Ubitrack::Measurement::Matrix3x4 > > ( "Matrix3x4PushCalibReaderWriter" );
	cf->registerComponent< Ubitrack::Drivers::PushCalibReaderWriter< Ubitrack::Measurement::Matrix4x4 > > ( "Matrix4x4PushCalibReaderWriter" );
	cf->registerComponent< Ubitrack::Drivers::PushCalibReaderWriter< Ubitrack::Measurement::Vector4D > > ( "Vector4PushCalibReaderWriter" );
	cf->registerComponent< Ubitrack::Drivers::PushCalibReaderWriter< Ubitrack::Measurement::PositionList > > ( "PositionListPushCalibReaderWriter" );
	cf->registerComponent< Ubitrack::Drivers::PushCalibReaderWriter< Ubitrack::Measurement::PositionList2 > > ( "PositionList2DPushCalibReaderWriter" );
	cf->registerComponent< Ubitrack::Drivers::PushCalibReaderWriter< Ubitrack::Measurement::PoseList > > ( "PoseListPushCalibReaderWriter" );
	cf->registerComponent< Ubitrack::Drivers::PushCalibReaderWriter< Ubitrack::Measurement::DistanceList > > ( "DistanceListPushCalibReaderWriter" );
	
}
