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
 * Implements a sink that writes incoming events to a file
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#include <fstream>
#include <boost/bind.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <utMeasurement/Measurement.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utDataflow/PushConsumer.h>

using namespace Ubitrack::Dataflow;

namespace Ubitrack { namespace Drivers {

/**
 * @ingroup driver_components
 * Recorder components, writes incoming events to a file.
 *
 * @par Input Ports
 * PushConsumer< EventType > with name "Input".
 *
 * @par Output Ports
 * None.
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
 * - \c Measurement::Pose (PoseRecorder)
 * - \c Measurement::Rotation (RotationRecorder)
 * - \c Measurement::Position (PositionRecorder)
 * - \c Measurement::RotationVelocity (RotationVelocityRecorder)
 */
template< class EventType >
class Recorder
	: public Component
{
public:
	/** open the file */
	Recorder( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Component( name )
		, m_inPort( "Input", *this, boost::bind( &Recorder::eventIn, this, _1 ) )
	{
		// read filename from configuration
		std::string sFilename = subgraph->m_DataflowAttributes.getAttributeString( "file" );
		if ( sFilename.empty() )
			UBITRACK_THROW( "No file attribute for Recorder component " + name );

		// create ofstream
		m_pStream.reset( new std::ofstream( sFilename.c_str() ) );
		if ( !m_pStream->good() )
			UBITRACK_THROW( "Could not open file " + sFilename + " for writing" );

		// create oarchive
		m_pArchive.reset( new boost::archive::text_oarchive( *m_pStream ) );
	}

	/** Close the file */
	~Recorder()
	{
		// destroy archive first
		m_pArchive.reset();
	}

protected:
	/** output port */
	Dataflow::PushConsumer< EventType > m_inPort;

	/** output stream */
	boost::scoped_ptr< std::ofstream > m_pStream;

	/** output archive */
	boost::scoped_ptr< boost::archive::text_oarchive > m_pArchive;

	/** handler method for incoming events */
	void eventIn( const EventType& event );
};


template< class EventType >
void Recorder< EventType >::eventIn( const EventType& n )
{
	// inserts a string "\n" before each measurement to make the resulting file more readable
	std::string linesep( "\n" );
	(*m_pArchive) << linesep;
	(*m_pArchive) << n;
}


} } // namespace Ubitrack::Drivers


UBITRACK_REGISTER_COMPONENT( Ubitrack::Dataflow::ComponentFactory* const cf )
{
	cf->registerComponent< Ubitrack::Drivers::Recorder< Ubitrack::Measurement::Pose > > ( "PoseRecorder" );
	cf->registerComponent< Ubitrack::Drivers::Recorder< Ubitrack::Measurement::ErrorPose > > ( "ErrorPoseRecorder" );
	cf->registerComponent< Ubitrack::Drivers::Recorder< Ubitrack::Measurement::ErrorPosition > > ( "ErrorPositionRecorder" );
	cf->registerComponent< Ubitrack::Drivers::Recorder< Ubitrack::Measurement::Position > > ( "PositionRecorder" );
	cf->registerComponent< Ubitrack::Drivers::Recorder< Ubitrack::Measurement::Position2D > > ( "Position2Recorder" );
	cf->registerComponent< Ubitrack::Drivers::Recorder< Ubitrack::Measurement::Rotation > > ( "RotationRecorder" );
	cf->registerComponent< Ubitrack::Drivers::Recorder< Ubitrack::Measurement::RotationVelocity > > ( "RotationVelocityRecorder" );
	cf->registerComponent< Ubitrack::Drivers::Recorder< Ubitrack::Measurement::PositionList > > ( "PositionListRecorder" );
	cf->registerComponent< Ubitrack::Drivers::Recorder< Ubitrack::Measurement::PositionList2 > > ( "PositionList2Recorder" );
	cf->registerComponent< Ubitrack::Drivers::Recorder< Ubitrack::Measurement::PoseList > > ( "PoseListRecorder" );
	cf->registerComponent< Ubitrack::Drivers::Recorder< Ubitrack::Measurement::Matrix3x4 > > ( "Matrix3x4Recorder" );
}
