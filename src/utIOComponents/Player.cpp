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
 * The player component for playback of recorded events
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp>

#include <utUtil/OS.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/Timestamp.h>
#include <utDataflow/Module.h>
#include <utDataflow/PushSupplier.h>

#ifdef HAVE_OPENCV
	#include <utVision/Image.h>
	#include <opencv/highgui.h>
#endif
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Drivers.Player" ) );

namespace Ubitrack { namespace Drivers {

// forward decls
class PlayerComponentBase;


/**
 * Component key for PlayerProducer/Consumer components.
 */
class PlayerComponentKey
	: public Dataflow::EdgeAttributeKey< std::string >
{
public:
	/** extract the "file" parameter of the edge config. */
	PlayerComponentKey( boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::EdgeAttributeKey< std::string >( pConfig, "Output", "file" )
	{}
};


/**
 * Module used by player components, maintains a single main loop for all player components
 */
class PlayerModule
	: public Dataflow::Module< Dataflow::SingleModuleKey, PlayerComponentKey, PlayerModule, PlayerComponentBase >
{
public:
	/** simplifies our life afterwards */
	typedef Dataflow::Module< Dataflow::SingleModuleKey, PlayerComponentKey, PlayerModule, PlayerComponentBase > BaseClass;

	PlayerModule( const Dataflow::SingleModuleKey& key, boost::shared_ptr< Graph::UTQLSubgraph >, FactoryHelper* fh )
		: BaseClass( key, fh )
		, m_bStop( false )
	{
		LOG4CPP_INFO( logger, "created PlayerModule" );
	}

	~PlayerModule()
	{
		// stop main loop
		m_bStop = true;

		// wait for thread
		if ( m_pMainLoopThread )
			m_pMainLoopThread->join();

		LOG4CPP_INFO( logger, "destroyed PlayerModule" );
	}

	void startThread()
	{
		LOG4CPP_DEBUG( logger, "starting thread" );

		// start mainloop
		if ( !m_pMainLoopThread )
			m_pMainLoopThread.reset( new boost::thread( boost::bind( &PlayerModule::mainloop, this ) ) );
	}

protected:

	/** the main loop thread */
	boost::shared_ptr< boost::thread > m_pMainLoopThread;

	/** stop the main loop? */
	bool m_bStop;

	/** method that runs the main loop */
	void mainloop();

	/** create new components */
	boost::shared_ptr< PlayerComponentBase > createComponent( const std::string& type, const std::string& name,
		boost::shared_ptr< Graph::UTQLSubgraph > pConfig, const ComponentKey& key, PlayerModule* pModule );
};


/**
 * Base class for all player components.
 */
class PlayerComponentBase
	: public PlayerModule::Component
{
public:
	PlayerComponentBase( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph >, const PlayerComponentKey& key,
		PlayerModule* module )
		: PlayerModule::Component( name, key, module )
	{}

	virtual ~PlayerComponentBase()
	{}

	/** returns the timestamp of the first event */
	virtual Measurement::Timestamp getFirstTime() const
	{ assert( false ); return 0; }

	/** return real time of the next measurement to be played or 0 if no events */
	virtual Measurement::Timestamp getNextTime( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{ assert( false ); return 0; }

	/** send the next event with the given offset */
	virtual void sendNext( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{ assert( false ); }

	virtual void start()
	{
		// for some reason, the default startModule mechanism does not work here...
		PlayerModule::Component::start();
		getModule().startThread();
	}
};


/**
 * @ingroup dataflow_components
 * Player component, loads recorded events from files and plays them at the
 * original speed.
 *
 * Multiple player components are synchronized, keeping the relative time between
 * the events. Playback starts with the event that has the lowest timestamp.
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * PushSupplier< EventType > with name "Output".
 *
 * @par Configuration
@verbatim
<Configuration file="<filename>"/>
@endverbatim
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 * - \c Measurement::Pose (PlayerPose)
 * - \c Measurement::Rotation (PlayerRotation)
 * - \c Measurement::Position (PlayerPosition)
*/
template< class EventType >
class PlayerComponent
	: public PlayerComponentBase
{
public:
	/** loads the file */
	PlayerComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > pConfig, const PlayerComponentKey& key,
		PlayerModule* module )
		: PlayerComponentBase( name, pConfig, key, module )
		, m_offset( 0 )
		, m_speedup( 1.0 )
		, m_outPort( "Output", *this )
	{
		LOG4CPP_INFO( logger, "Created PlayerComponent for file=" << key.get() );

		// read configuration
		pConfig->getEdge( "Output" )->getAttributeData( "offset", m_offset );
		pConfig->getEdge( "Output" )->getAttributeData( "speedup", m_speedup );

		// open file
		std::ifstream file( key.get().c_str() );
		if ( !file.good() )
			UBITRACK_THROW( "Could not open file " + key.get() );

		boost::archive::text_iarchive archive( file );

		// read contents until end-of-file exception
		try
		{
			while ( true )
			{
				EventType e( boost::shared_ptr< typename EventType::value_type >( new typename EventType::value_type() ) );
				std::string dummy; // for newline character in archive
				archive >> dummy >> e;
				m_events.push_back( e );
			}
		}
		catch( ... )
		{}

		m_iterator = m_events.begin();
	}

	Measurement::Timestamp getFirstTime() const
	{
		if ( !m_events.empty() )
			return m_events.front().time() + 1000000LL * m_offset;
		else
			return 0;
	}

	/** return time of the next measurement to be played or 0 if no events */
	Measurement::Timestamp getNextTime( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{
		if ( m_iterator != m_events.end() )
			return recordTimeToReal( m_iterator->time(), recordStart, playbackStart );
		else
			return 0;
	}

	/** send the next event */
	void sendNext( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{
		m_outPort.send( EventType( recordTimeToReal( m_iterator->time(), recordStart, playbackStart ), *m_iterator ) );
		m_iterator++;
	}

protected:

	/**
	 * converts a recorded time to a real time.
	 * @param t timestamp to convert
	 * @param recordStart time the recording was started
	 * @param realStart time the playback was started
	 */
	Measurement::Timestamp recordTimeToReal( Measurement::Timestamp t, Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{ return static_cast< Measurement::Timestamp >( ( t - recordStart + m_offset * 1000000LL ) / m_speedup + playbackStart ); }

	/** offset if the event should be sent at some other time than its timestamp */
	int m_offset;

	/** speedup factor */
	double m_speedup;

	/** output port */
	Dataflow::PushSupplier< EventType > m_outPort;

	/** list of all events */
	std::vector< EventType > m_events;

	/** iterator to next event */
	typename std::vector< EventType >::iterator m_iterator;
};

/**
 * @ingroup dataflow_components
 * Player component, loads recorded events from files and plays them at the
 * original speed.
 *
 * Multiple player components are synchronized, keeping the relative time between
 * the events. Playback starts with the event that has the lowest timestamp.
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * PushSupplier< EventType > with name "Output".
 *
 * @par Configuration
@verbatim
<Configuration file="<filename>"/>
@endverbatim
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 * - \c Measurement::Pose (PlayerPose)
 * - \c Measurement::Rotation (PlayerRotation)
 * - \c Measurement::Position (PlayerPosition)
*/

#ifdef HAVE_OPENCV
class PlayerComponentImage
	: public PlayerComponentBase
{
public:
	/** loads the file */
	PlayerComponentImage( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > pConfig, const PlayerComponentKey& key,
		PlayerModule* module )
		: PlayerComponentBase( name, pConfig, key, module )
		, m_tsFile( "" )
		, m_offset( 0 )
		, m_speedup( 1.0 )
		, m_outPort( "Output", *this )
	{
		LOG4CPP_INFO( logger, "Created PlayerComponentImage for file = " << key.get() );

		// read configuration
		pConfig->getEdge( "Output" )->getAttributeData( "offset", m_offset );
		pConfig->getEdge( "Output" )->getAttributeData( "speedup", m_speedup );

		// get the file which describes the timestamps and filenames
		pConfig->getEdge( "Output" )->getAttributeData( "file", m_tsFile );

		boost::filesystem::path tsFile( m_tsFile );
		if( !boost::filesystem::exists( tsFile ) )
			UBITRACK_THROW( "file with timestamps does not exist, please check the path: " + m_tsFile );
		
		// read file with timestamps and filenames
		std::ifstream ifs( m_tsFile.c_str() );

		if( !ifs.is_open ( ) )
			UBITRACK_THROW( "Could not open file \"" + m_tsFile  + "\". This file should contain the timestamps and filenames of the images." );
	

		// declare and initialize the variable for the timestamp and the file name string of the pictures
		Measurement::Timestamp timeStamp = 0;
		std::string fileName("");


		// read contents line by line
		// read the image file and add the image and the timestamp to an event queue
		std::string temp;
		LOG4CPP_INFO( logger, "Starting loading images for file = " << key.get() );

		while( getline( ifs, temp ) )
		{

		  LOG4CPP_NOTICE( logger, "Loading image file for log line " <<  temp );

		  // parsing line with stringstreams
		  // we have the fileformat "1272966027407 CameraRaw01249.jpg"
		  std::stringstream ss (std::stringstream::in | std::stringstream::out);
		  ss << temp; // parse the line
		  ss >> timeStamp;
		  ss >> fileName;

		  if ( fileName.begin() == fileName.end() )
		  	continue;

		  // mh: uses c++11 standard
		  // if ( fileName.front() == '"' ) {

		  if ( *(fileName.begin()) == '"' ) {
			fileName.erase( 0, 1 ); // erase the first character
			fileName.erase( fileName.size() - 1 ); // erase the last character
		}
  		  
		 	// check for length of timestamp to deside if timestamp is ms or ns
			// (we just need ms here, no need to be more accurate)
			timeStamp = ( timeStamp > 1e13 ) ? timeStamp * 1e-06 : timeStamp;
			
			boost::filesystem::path file( fileName );
			boost::filesystem::file_status fstatus( boost::filesystem::status( file ) );
			
			//check if path to file is given absolute
			if( !boost::filesystem::exists( fstatus ) )
			{
				//no absolute path, check reltive path to timestampfile:
				file = boost::filesystem::path( tsFile.parent_path().string() + "/" + fileName );
				if( !boost::filesystem::exists( file ) )
					continue;
			}
			
			//logging
			LOG4CPP_TRACE( logger, "loading image file " <<  file.string() << " for frame " <<  timeStamp );

			// Load the image
			// assigning the NULL avoids memory leaks
			IplImage* pIpl = NULL;
			try
			{
				pIpl = cvLoadImage( file.string().c_str(), CV_LOAD_IMAGE_UNCHANGED );
			}
			catch( std::exception& e )
			{
				LOG4CPP_ERROR( logger, "loading image file \"" << file.string() << "\" failed: " << e.what() );
				LOG4CPP_ERROR( logger, "loading image file \"" << file.string() << "\" failed: " << e.what() );
				continue;
			}
			
			if( !pIpl )
			{
				LOG4CPP_ERROR( logger, "loading image file \"" <<  file.string() << "\" failed." );
				continue;
			}

			// convert loaded image into the required pImage class
			boost::shared_ptr< Vision::Image > pImage( new Vision::Image( pIpl->width, pIpl->height, 3 ) );
			cvConvertImage( pIpl, *pImage );
			pImage->origin = pIpl->origin;
			pImage->channelSeq[0]='B';
			pImage->channelSeq[1]='G';
			pImage->channelSeq[2]='R';


			// Building the event and packing timestamp and image into it
			Measurement::ImageMeasurement e( (Measurement::Timestamp) ( 1e6 * timeStamp), pImage );

			// Store it
			m_events.push_back(e);

			// releasing memory
			cvReleaseImage(&pIpl);
		}

		m_iterator = m_events.begin();
	}

	Measurement::Timestamp getFirstTime() const
	{
		if ( !m_events.empty() )
			return m_events.front().time() + 1000000LL * m_offset;
		else
			return 0;
	}

	/** return time of the next measurement to be played or 0 if no events */
	Measurement::Timestamp getNextTime( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{
		if ( m_iterator != m_events.end() )
			return recordTimeToReal( m_iterator->time(), recordStart, playbackStart );
		else
			return 0;
	}

	/** send the next event */
	void sendNext( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{
		m_outPort.send( Measurement::ImageMeasurement( recordTimeToReal( m_iterator->time(), recordStart, playbackStart ), *m_iterator ) );
		m_iterator++;
	}

protected:

	/**
	 * converts a recorded time to a real time.
	 * @param t timestamp to convert
	 * @param recordStart time the recording was started
	 * @param realStart time the playback was started
	 */
	Measurement::Timestamp recordTimeToReal( Measurement::Timestamp t, Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{ return static_cast< Measurement::Timestamp >( ( t - recordStart + m_offset * 1000000LL ) / m_speedup + playbackStart ); }

	/** file which defines timestamps and images */
	std::string m_tsFile;
	
	/** offset if the event should be sent at some other time than its timestamp */
	int m_offset;

	/** speedup factor */
	double m_speedup;

	/** output port */
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;

	/** list of all events */
	std::vector< Measurement::ImageMeasurement > m_events;

	/** iterator to next event */
	std::vector< Measurement::ImageMeasurement >::iterator m_iterator;
};
#endif


void PlayerModule::mainloop()
{
	// find time of first recorded event in queue
	Measurement::Timestamp recordStart( 0 );
	{
		ComponentList l( getAllComponents() );
		for ( ComponentList::iterator it = l.begin(); it != l.end(); it++ )
		{
			Measurement::Timestamp t = (*it)->getFirstTime();
			if ( t && ( recordStart == 0 || t < recordStart ) )
				recordStart = t;
		}
	}
	LOG4CPP_DEBUG( logger, "recordStart = " << recordStart );

	// delay start for 3s to allow other components to start
	Measurement::Timestamp playbackStart( Measurement::now() + 2000000000LL );
	LOG4CPP_DEBUG( logger, "playbackStart = " << playbackStart );

	// find playback time of first event in queue
	Measurement::Timestamp nextEventTime( 0 );
	{
		ComponentList l( getAllComponents() );
		for ( ComponentList::iterator it = l.begin(); it != l.end(); it++ )
		{
			Measurement::Timestamp t = (*it)->getNextTime( recordStart, playbackStart );
			if ( t && ( nextEventTime == 0 || t < nextEventTime ) )
				nextEventTime = t;
		}
	}
	LOG4CPP_DEBUG( logger, "Starting main loop" );

	// main loop
	while ( !m_bStop && nextEventTime )
	{
		LOG4CPP_DEBUG( logger, "nextEventTime = " << nextEventTime );

		// sleep until next event
		Measurement::Timestamp now( Measurement::now() );
		long long int sleepdur( nextEventTime - now );
		if ( sleepdur > 0 )
		{
			LOG4CPP_DEBUG( logger, "sleeping " << sleepdur / 1000000 << "ms" );
			Util::sleep( int( sleepdur / 1000000 ), int( sleepdur % 1000000 ) );
		}

		now = Measurement::now();
		nextEventTime = 0;

		// iterate all components
		ComponentList l( getAllComponents() );
		for ( ComponentList::iterator it = l.begin(); it != l.end(); it++ )
		{
			// send all events due until now
			Measurement::Timestamp t = (*it)->getNextTime( recordStart, playbackStart );
			for ( ; t && t <= now; )
			{
				(*it)->sendNext( recordStart, playbackStart );
				t = (*it)->getNextTime( recordStart, playbackStart );
				if ( !t )
					LOG4CPP_NOTICE( logger, (*it)->getName() << " reached end of recording" );
			}

			// update next event time
			if ( t && ( nextEventTime == 0 || t < nextEventTime ) )
				nextEventTime = t;
		}
	}
}


// has to be here, after all class declarations
boost::shared_ptr< PlayerComponentBase > PlayerModule::createComponent( const std::string& type, const std::string& name,
	boost::shared_ptr< Graph::UTQLSubgraph > pConfig, const PlayerModule::ComponentKey& key, PlayerModule* pModule )
{
	if ( type == "PlayerPose" )
		return boost::shared_ptr< PlayerComponentBase >( new PlayerComponent< Measurement::Pose >( name, pConfig, key, pModule ) );
#ifdef HAVE_OPENCV		
	else if ( type == "PlayerImage" )
		return boost::shared_ptr< PlayerComponentBase >( new PlayerComponentImage( name, pConfig, key, pModule ) );
#endif
	else if ( type == "PlayerRotation" )
		return boost::shared_ptr< PlayerComponentBase >( new PlayerComponent< Measurement::Rotation >( name, pConfig, key, pModule ) );
	else if ( type == "PlayerPosition" )
		return boost::shared_ptr< PlayerComponentBase >( new PlayerComponent< Measurement::Position >( name, pConfig, key, pModule ) );
	else if ( type == "PlayerPosition2" )
		return boost::shared_ptr< PlayerComponentBase >( new PlayerComponent< Measurement::Position2D >( name, pConfig, key, pModule ) );
	else if ( type == "PlayerPositionList" )
		return boost::shared_ptr< PlayerComponentBase >( new PlayerComponent< Measurement::PositionList >( name, pConfig, key, pModule ) );
	else if ( type == "PlayerPositionList2" )
		return boost::shared_ptr< PlayerComponentBase >( new PlayerComponent< Measurement::PositionList2 >( name, pConfig, key, pModule ) );
	else if ( type == "PlayerErrorPose" )
		return boost::shared_ptr< PlayerComponentBase >( new PlayerComponent< Measurement::ErrorPose >( name, pConfig, key, pModule ) );
	else if ( type == "PlayerErrorPosition" )
		return boost::shared_ptr< PlayerComponentBase >( new PlayerComponent< Measurement::ErrorPosition >( name, pConfig, key, pModule ) );
	else if ( type == "PlayerPoseList" )
		return boost::shared_ptr< PlayerComponentBase >( new PlayerComponent< Measurement::PoseList >( name, pConfig, key, pModule ) );
	else if ( type == "PlayerRotationVelocity" )
		return boost::shared_ptr< PlayerComponentBase >( new PlayerComponent< Measurement::RotationVelocity >( name, pConfig, key, pModule ) );

	UBITRACK_THROW( "Class " + type + " not supported by player module" );
}


} } // namespace Ubitrack::Drivers


UBITRACK_REGISTER_COMPONENT( Ubitrack::Dataflow::ComponentFactory* const cf ) {
	// create list of supported types
	std::vector< std::string > playerComponents;
	playerComponents.push_back( "PlayerPose" );
	playerComponents.push_back( "PlayerErrorPose" );
	playerComponents.push_back( "PlayerErrorPosition" );
	playerComponents.push_back( "PlayerRotation" );
	playerComponents.push_back( "PlayerPosition" );
	playerComponents.push_back( "PlayerPosition2" );
	playerComponents.push_back( "PlayerPositionList" );
	playerComponents.push_back( "PlayerPositionList2" );
	playerComponents.push_back( "PlayerPoseList" );
	playerComponents.push_back( "PlayerRotationVelocity" );
#ifdef HAVE_OPENCV
	playerComponents.push_back( "PlayerImage" );
#endif

	cf->registerModule< Ubitrack::Drivers::PlayerModule > ( playerComponents );
}
