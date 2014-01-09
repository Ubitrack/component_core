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
 * TestSource component.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#include <string>

#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/OS.h>

#include <log4cpp/Category.hh>

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.TestSource" ) );

using namespace Ubitrack;

// anonymous namespace for private functions
// TODO: Move random data generation to ubitracklib. Tests need it, too
namespace {

// returns a random number between -1 and +1
double random()
{
	return double( rand() ) / RAND_MAX * 2.0 - 1.0;
}

Math::Vector< double, 3 > randomPosition( const Math::Vector< double, 3 >& ref, double noise )
{
	return ref + Math::Vector< double, 3 >( random() * noise, random() * noise, random() * noise );
}

Math::Quaternion randomRotation( const Math::Quaternion& ref, double noise )
{
	Math::Quaternion qRand( random() * noise, random() * noise, random() * noise, 0 );
	double qNorm = boost::math::norm( qRand );
	if ( qNorm > 1.0 )
	{
		// too large
		double newNorm = ( random() + 1.0 ) / 2.0;
		qRand *= newNorm / qNorm;
		qNorm = newNorm;
	}
	qRand = Math::Quaternion( qRand.x(), qRand.y(), qRand.z(), sqrt( 1.0 - qNorm * qNorm ) );
	return ref * qRand;
}

}


namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Pushes events at a given rate with optional noise.
 *
 * This is primarily useful for generating test events when no tracker is available
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * PushSupplier<EventType> port with name "Output".
 *
 * @par Configuration
 * - Edge configuration:
 * @verbatim
 * <Configuration position="..." rotation="..." frequency="..." posnoise="..." rotnoise="..."/>
 * @endverbatim
 *   - \c position : the 3D position component of the event (defaults to "0 0 0")
 *   - \c rotation : the 3D rotation component of the event (defaults to "0 0 0 1")
 *   - \c frequency : float describing number of events to generate per second (defaults to "30")
 *   - \c posnoise : float giving the max radius around \c position in which to move (defaults to "0")
 *   - \c rotnoise : float giving the max sine of the angle around \c rotation by which to rotate (defaults to "0")
 *   .
 * Depending on the instantiated type of the component
 * either the position, the rotation or both parts
 * are mandatory.
 *
 * @par Operation
 * Creates an event \c frequency times per second with optional noise
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 * - Ubitrack::Measurement::Position: TestSourcePosition
 * - Ubitrack::Measurement::Rotation: TestSourceRotation
 * - Ubitrack::Measurement::Pose : TestSourcePose
 */
template< class EventType > class TestSource
    : public Dataflow::Component
{
public:

	/**
	 * UTQL component constructor.
	 *
	 * @param nm Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	TestSource( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Dataflow::Component( sName )
		, m_outPort( "Output", *this )
		, m_bStop( true )
		, m_frequency( 30.0 )
		, m_jerkTime( 3000 )
		, m_posNoise( 0.1 )
		, m_rotNoise( 0.1 )
	{
		// read configuration
		readStatic( *subgraph );

		subgraph->m_DataflowAttributes.getAttributeData( "posnoise", m_posNoise );
		subgraph->m_DataflowAttributes.getAttributeData( "rotnoise", m_rotNoise );
		subgraph->m_DataflowAttributes.getAttributeData( "frequency", m_frequency );
		subgraph->m_DataflowAttributes.getAttributeData( "jerktime", m_jerkTime );
		

		// start thread
		LOG4CPP_INFO( logger, "starting TestSource with frequency " << m_frequency );
		LOG4CPP_DEBUG( logger, "noise: " << m_posNoise << " " << m_rotNoise );
		stop();
	}

	/** component start method, starts thread */
	virtual void start()
	{
		if ( !m_running )
		{
			m_bStop = false;
			m_running = true;
			if ( m_frequency )
				m_pThread.reset( new boost::thread( boost::bind( &TestSource< EventType >::mainLoop, this ) ) );
		}
	}

	/** component stop method, stops thread */
	virtual void stop()
	{
		if ( m_running )
		{
			LOG4CPP_INFO( logger, "stopping TestSource" );
			m_bStop = true;
			m_running = false;
			if ( m_pThread )
				m_pThread->join();
		}
	}

	/** destructor, stops thread */
	~TestSource()
	{
		stop();
	}

protected:

	/**
	 * reads the static measurement from the configuration
	 */
	void readStatic( const Graph::UTQLSubgraph& subgraph );

	/**
	 * The main loop, running in a separate thread
	 */
	void mainLoop()
	{
		// initialize
		Measurement::Timestamp jerkInterval = Measurement::Timestamp( 1000000 ) * m_jerkTime;
		m_prevMeasurement = randomEvent( m_staticMeasurement, m_posNoise, m_rotNoise );
		m_nextMeasurement = randomEvent( m_staticMeasurement, m_posNoise, m_rotNoise );

		Measurement::Timestamp lastTime = Measurement::now();
		while ( !m_bStop )
		{
			// sleep some time
			Util::sleep( unsigned( 1000.0 / m_frequency ) );

			// compute new random position?
			Measurement::Timestamp now = Measurement::now();
			if ( now / jerkInterval > lastTime / jerkInterval )
			{
				m_prevMeasurement = m_nextMeasurement;
				m_nextMeasurement = randomEvent( m_staticMeasurement, m_posNoise, m_rotNoise );
			}

			// interpolate
			EventType event( now,
				linearInterpolate( m_prevMeasurement, m_nextMeasurement, double( now % jerkInterval ) / jerkInterval ) );

			// send
			m_outPort.send( event );

			lastTime = now;
		}
	}

	/// helper function to check and parse position attribute
	Math::Vector< double, 3 > readPosition( const std::string& s )
	{
		double position[3];
		sscanf( s.c_str(), "%lf %lf %lf", &position[0], &position[1], &position[2] );
		return Math::Vector< double, 3 >( position[0], position[1], position[2] );
	}

	/// helper function to check and parse rotation attribute
	Math::Quaternion readRotation( const std::string& s )
	{
		double rotation[4];
		sscanf( s.c_str(), "%lf %lf %lf %lf", &rotation[0], &rotation[1], &rotation[2], &rotation[3] );
		return Math::Quaternion( rotation[0], rotation[1], rotation[2], rotation[3] );
	}

	// creates a random event
	static typename EventType::value_type randomEvent( const typename EventType::value_type& ref, double posNoise, double rotNoise );

	// the output port
	Dataflow::PushSupplier< EventType > m_outPort;

	// pointer to thread
	boost::scoped_ptr< boost::thread > m_pThread;

	// should the thread stop?
	bool m_bStop;

	// event generation frequency
	double m_frequency;

	// time between direction changes
	int m_jerkTime;

	// the noise values
	double m_posNoise;
	double m_rotNoise;

	// the static measurement as specified in the configuration
	typename EventType::value_type m_staticMeasurement;

	// the two measurements to interpolate between
	typename EventType::value_type m_prevMeasurement;
	typename EventType::value_type m_nextMeasurement;
};


template<>
void TestSource< Measurement::Position >::readStatic( const Graph::UTQLSubgraph& subgraph )
{
	m_staticMeasurement = Math::Vector< double, 3 >( 0, 0, 0 );

	std::string sAttr = subgraph.m_DataflowAttributes.getAttributeString( "position" );
	if ( !sAttr.empty() )
		m_staticMeasurement = readPosition( sAttr );
}


template<>
void TestSource< Measurement::Rotation >::readStatic( const Graph::UTQLSubgraph& subgraph )
{
	m_staticMeasurement = Math::Quaternion( 0, 0, 0, 1 );

	std::string sAttr = subgraph.m_DataflowAttributes.getAttributeString( "rotation" );
	if ( !sAttr.empty() )
		m_staticMeasurement = readRotation( sAttr );
}


template<>
void TestSource< Measurement::Pose >::readStatic( const Graph::UTQLSubgraph& subgraph )
{
	Math::Quaternion rot( 0, 0, 0, 1 );
	Math::Vector< double, 3 > pos( 0, 0, 0 );

	std::string sAttr = subgraph.m_DataflowAttributes.getAttributeString( "position" );
	if ( !sAttr.empty() )
		pos = readPosition( sAttr );

	sAttr = subgraph.m_DataflowAttributes.getAttributeString( "rotation" );
	if ( !sAttr.empty() )
		rot = readRotation( sAttr );

	m_staticMeasurement = Math::Pose( rot, pos );
}


template<>
Math::Vector< double, 3 > TestSource< Measurement::Position >::randomEvent(
	const Math::Vector< double, 3 >& ref, double posNoise, double )
{
	return randomPosition( ref, posNoise );
}


template<>
Math::Quaternion TestSource< Measurement::Rotation >::randomEvent(
	const Math::Quaternion& ref, double, double rotNoise )
{
	return randomRotation( ref, rotNoise );
}


template<>
Math::Pose TestSource< Measurement::Pose >::randomEvent(
	const Math::Pose& ref, double posNoise, double rotNoise )
{
	return Math::Pose( randomRotation( ref.rotation(), rotNoise ), randomPosition( ref.translation(), posNoise ) );
}


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< TestSource< Measurement::Position > > ( "TestSourcePosition" );
	cf->registerComponent< TestSource< Measurement::Rotation > > ( "TestSourceRotation" );
	cf->registerComponent< TestSource< Measurement::Pose > > ( "TestSourcePose" );
}

} } // namespace Ubitrack::Components
