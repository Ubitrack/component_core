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
 * Components that write/read a static measurement to/from a file.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
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
#include <utUtil/CalibFile.h>

using namespace Ubitrack::Dataflow;

namespace Ubitrack { namespace Drivers {

/**
 * @ingroup driver_components
 * CalibWriter components, writes a static measurement to a file.
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
 * @par Instances
 * Registered for the following EventTypes and names:
 * - \c Measurement::Pose (PoseCalibWriter)
 * - \c Measurement::Rotation (RotationCalibWriter)
 * - \c Measurement::Position (PositionCalibWriter)
 * - \c Measurement::RotationVelocity (RotationVelocityCalibWriter)
 */
template< class EventType >
class CalibWriter
	: public Component
{
public:
	/** open the file */
	CalibWriter( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Component( name )
		, m_inPort( "Input", *this, boost::bind( &CalibWriter::eventIn, this, _1 ) )
		, m_lastTS( 0 )
	{
		// read filename from configuration
		m_sFilename = subgraph->m_DataflowAttributes.getAttributeString( "file" );
		if ( m_sFilename.empty() )
			UBITRACK_THROW( "No \"file\" attribute for CalibWriter component " + name );
	}

	/** Saves the last received measurement in case it was not yet written to disk */
	~CalibWriter()
	{
		// if the last measurement was not written to disk, do it now
		boost::mutex::scoped_lock l( m_fileMutex );
		if ( m_lastMeasurement )
			Util::writeCalibFile( m_sFilename, m_lastMeasurement );
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
};


template< class EventType >
void CalibWriter< EventType >::eventIn( const EventType& n )
{
	// lock the file to prevent other threads from writing simultaneously
	boost::mutex::scoped_lock l( m_fileMutex );

	if ( m_lastTS + s_minWriteDistance > n.time() )
		m_lastMeasurement = n;
	else
	{
		Util::writeCalibFile( m_sFilename, n );
		m_lastMeasurement.reset();
		m_lastTS = n.time();
	}
}


/**
 * @ingroup driver_components
 * CalibReader components, works like static measurement, but reads the configuration from a file.
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * PullSupplier< EventType > with name "Output".
 *
 * @par Configuration
@verbatim
<DataflowConfiguration>
	<Attribute name="file" value="<filename>"/>
</DataflowConfiguration>
@endverbatim
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 * - \c Measurement::Pose (PoseCalibReader)
 * - \c Measurement::Rotation (RotationCalibReader)
 * - \c Measurement::Position (PositionCalibReader)
 * - \c Measurement::RotationVelocity (RotationVelocityCalibReader)
 */
template< class EventType >
class CalibReader
	: public Component
{
public:
	/** open the file */
	CalibReader( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Component( name )
		, m_outPort( "Output", *this, boost::bind( &CalibReader::request, this, _1 ) )
		, m_Measurement( typename EventType::value_type() )
	{
		// read filename from configuration
		std::string m_sFilename = subgraph->m_DataflowAttributes.getAttributeString( "file" );
		if ( m_sFilename.empty() )
			UBITRACK_THROW( "No \"file\" attribute for CalibReader component " + name );

		// read measurement
		Util::readCalibFile( m_sFilename, m_Measurement );
	}

protected:
	/** output port */
	Dataflow::PullSupplier< EventType > m_outPort;

	/** mutex to protect file from simultaneous writes */
	EventType m_Measurement;

	/** handler method for incoming pull requests */
	EventType request( Measurement::Timestamp t )
	{ return EventType( t, *m_Measurement ); }
};


} } // namespace Ubitrack::Drivers


UBITRACK_REGISTER_COMPONENT( Ubitrack::Dataflow::ComponentFactory* const cf )
{
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::Distance > > ( "DistanceCalibWriter" );
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::Pose > > ( "PoseCalibWriter" );
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::ErrorPose > > ( "ErrorPoseCalibWriter" );
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::ErrorPosition > > ( "ErrorPositionCalibWriter" );
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::Position > > ( "PositionCalibWriter" );
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::Rotation > > ( "RotationCalibWriter" );
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::Matrix3x3 > > ( "Matrix3x3CalibWriter" );
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::Matrix3x4 > > ( "Matrix3x4CalibWriter" );
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::Matrix4x4 > > ( "Matrix4x4CalibWriter" );
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::Vector4D > > ( "Vector4CalibWriter" );
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::Vector8D > > ( "Vector8CalibWriter" );
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::PositionList > > ( "PositionListCalibWriter" );
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::PositionList2 > > ( "PositionList2DCalibWriter" );
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::PoseList > > ( "PoseListCalibWriter" );
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::DistanceList > > ( "DistanceListCalibWriter" );

	// MH: CameraIntrinsics serialization does not work
	// CW: added again, removed the serialization error (2013-07-08)
	cf->registerComponent< Ubitrack::Drivers::CalibWriter< Ubitrack::Measurement::CameraIntrinsics > > ( "CameraIntrinsicsCalibWriter" );
	
	
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::Distance > > ( "DistanceCalibReader" );
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::Pose > > ( "PoseCalibReader" );
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::ErrorPose > > ( "ErrorPoseCalibReader" );
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::ErrorPosition > > ( "ErrorPositionCalibReader" );
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::Position > > ( "PositionCalibReader" );
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::Rotation > > ( "RotationCalibReader" );
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::Matrix3x3 > > ( "Matrix3x3CalibReader" );
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::Matrix3x4 > > ( "Matrix3x4CalibReader" );
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::Matrix4x4 > > ( "Matrix4x4CalibReader" );
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::Vector4D > > ( "Vector4CalibReader" );
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::Vector8D > > ( "Vector8CalibReader" );
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::PositionList > > ( "PositionListCalibReader" );
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::PositionList2 > > ( "PositionList2DCalibReader" );
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::PoseList > > ( "PoseListCalibReader" );
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::DistanceList > > ( "DistanceListCalibReader" );

	// MH: CameraIntrinsics serialization does not work
	// CW: added again, removed the serialization error (2013-07-08)
	cf->registerComponent< Ubitrack::Drivers::CalibReader< Ubitrack::Measurement::CameraIntrinsics > > ( "CameraIntrinsicsCalibReader" );
}
