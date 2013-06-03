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


#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/binary_object.hpp>

#include "NetworkSource.h"

#include <iostream>
#include <sstream>

#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

#include <boost/array.hpp>

#include <log4cpp/Category.hh>


namespace Ubitrack { namespace Drivers {

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Drivers.NetworkSource" ) );

SourceModule::SourceModule( const SourceModuleKey& moduleKey, boost::shared_ptr< Graph::UTQLSubgraph >, FactoryHelper* pFactory )
	: Module< SourceModuleKey, SourceComponentKey, SourceModule, SourceComponentBase >( moduleKey, pFactory )
{
	stopModule();
}


SourceModule::~SourceModule()
{
	stopModule();
}

void SourceModule::startModule()
{
	if ( !m_running )
	{
		m_running = true;

		using boost::asio::ip::udp;
		

		LOG4CPP_DEBUG( logger, "Starting Network Source service" );
		LOG4CPP_DEBUG( logger, "Creating receiver on port " << m_moduleKey );

		m_IoService.reset( new boost::asio::io_service );

		m_Socket.reset( new udp::socket( *m_IoService ) );

		m_Socket->open( udp::v4() );
		boost::asio::socket_base::reuse_address option( true );
		m_Socket->set_option( option );
		m_Socket->bind( udp::endpoint( udp::v4(), m_moduleKey ) );

		m_Socket->async_receive_from(
			boost::asio::buffer( receive_data, max_receive_length ),
		sender_endpoint,
		boost::bind( &SourceModule::HandleReceive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );

		// network thread runs until io_service is interrupted
		LOG4CPP_DEBUG( logger, "Starting network receiver thread" );
		m_NetworkThread = boost::shared_ptr< boost::thread >( new boost::thread( boost::bind( &boost::asio::io_service::run, m_IoService.get() ) ) );

		LOG4CPP_DEBUG( logger, "Network Source service started" );
	}
}

void SourceModule::stopModule()
{

	if ( m_running )
	{
		m_running = false;
		LOG4CPP_NOTICE( logger, "Stopping Network Source Module" );

		if ( m_IoService )
		{
			LOG4CPP_TRACE( logger, "Stopping IO service" );
			m_IoService->stop();

			LOG4CPP_TRACE( logger, "Joining network thread" );
			if ( m_NetworkThread )
				m_NetworkThread->join();

			LOG4CPP_TRACE( logger, "Detroying network thread" );
			m_NetworkThread.reset();

			LOG4CPP_TRACE( logger, "Closing socket" );
			if ( m_Socket )
			{
				m_Socket->close();
				m_Socket.reset();
			}

			LOG4CPP_TRACE( logger, "Detroying IO service" );
			m_IoService.reset();
		}

	}
	LOG4CPP_DEBUG( logger, "Network Source Stopped" );
}




void SourceModule::HandleReceive( const boost::system::error_code err, size_t length )
{
	Measurement::Timestamp recvtime = Measurement::now();

	LOG4CPP_DEBUG( logger, "Received " << length << " bytes" );

	// some error checking
	if ( err && err != boost::asio::error::message_size )
	{
		std::ostringstream msg;
		msg << "Error receiving from socket: \"" << err << "\"";
		LOG4CPP_ERROR( logger, msg.str() );
		UBITRACK_THROW( msg.str() );
	}

	if (length >= max_receive_length)
	{
		LOG4CPP_ERROR( logger, "Too many bytes received" );
		UBITRACK_THROW( "FIXME: received more than max_receive_length bytes." );
	}

	try
	{
		// make receive data null terminated and create a string stream
		receive_data[length] = 0;
		std::string data( receive_data );
		LOG4CPP_TRACE( logger, "data: " << data );
		std::istringstream stream( data );
		boost::archive::text_iarchive message( stream );

		// parse packet
		std::string name;
		message >> name;
		LOG4CPP_DEBUG( logger, "Message for component " << name );

		SourceComponentKey key( name );

		if ( hasComponent( key ) ) {
			boost::shared_ptr< SourceComponentBase > comp = getComponent( key );
			comp->parse( message, recvtime );
		}
		else
			LOG4CPP_WARN( logger, "NetworkSink is sending with id=\"" << name << "\", found no corresponding NetworkSource pattern with same id."  );
	}
	catch ( const std::exception& e )
	{
		LOG4CPP_ERROR( logger, "Caught exception " << e.what() );
	}

	// restart receiving new packet
	m_Socket->async_receive_from(
		boost::asio::buffer( receive_data, max_receive_length ),
		sender_endpoint,
		boost::bind( &SourceModule::HandleReceive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred )
	);
}


boost::shared_ptr< SourceComponentBase > SourceModule::createComponent( const std::string& type, const std::string& name,
	boost::shared_ptr< Graph::UTQLSubgraph > config, const SourceModule::ComponentKey& key, SourceModule* pModule )
{
	if ( type == "NetworkSourcePose" )
		return boost::shared_ptr< SourceComponentBase >( new SourceComponent< Measurement::Pose >( name, config, key, pModule ) );
	else if ( type == "NetworkSourceErrorPose" )
		return boost::shared_ptr< SourceComponentBase >( new SourceComponent< Measurement::ErrorPose >( name, config, key, pModule ) );
	else if ( type == "NetworkSourceRotation" )
		return boost::shared_ptr< SourceComponentBase >( new SourceComponent< Measurement::Rotation >( name, config, key, pModule ) );
	else if ( type == "NetworkSourcePosition" )
		return boost::shared_ptr< SourceComponentBase >( new SourceComponent< Measurement::Position >( name, config, key, pModule ) );
	else if ( type == "NetworkSourcePosition2D" )
		return boost::shared_ptr< SourceComponentBase >( new SourceComponent< Measurement::Position2D >( name, config, key, pModule ) );
	else if ( type == "NetworkSourcePoseList" )
		return boost::shared_ptr< SourceComponentBase >( new SourceComponent< Measurement::PoseList >( name, config, key, pModule ) );
	else if ( type == "NetworkSourcePositionList" )
		return boost::shared_ptr< SourceComponentBase >( new SourceComponent< Measurement::PositionList >( name, config, key, pModule ) );
	else if ( type == "NetworkSourcePositionList2" )
		return boost::shared_ptr< SourceComponentBase >( new SourceComponent< Measurement::PositionList2 >( name, config, key, pModule ) );
	else if ( type == "NetworkSourceEvent" )
		return boost::shared_ptr< SourceComponentBase >( new SourceComponent< Measurement::Button >( name, config, key, pModule ) );
	else if ( type == "NetworkSourceMatrix3x3" )
		return boost::shared_ptr< SourceComponentBase >( new SourceComponent< Measurement::Matrix3x3 >( name, config, key, pModule ) );
	else if ( type == "NetworkSourceMatrix3x4" )
		return boost::shared_ptr< SourceComponentBase >( new SourceComponent< Measurement::Matrix3x4 >( name, config, key, pModule ) );
	else if ( type == "NetworkSourceMatrix4x4" )
		return boost::shared_ptr< SourceComponentBase >( new SourceComponent< Measurement::Matrix4x4 >( name, config, key, pModule ) );

	UBITRACK_THROW( "Class " + type + " not supported by network source module." );
}


// register module at factory
UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {

	// create list of supported types
	std::vector< std::string > sourceComponents;

	sourceComponents.push_back( "NetworkSourcePose" );
	sourceComponents.push_back( "NetworkSourceErrorPose" );
	sourceComponents.push_back( "NetworkSourceRotation" );
	sourceComponents.push_back( "NetworkSourcePosition" );
	sourceComponents.push_back( "NetworkSourcePosition2D" );
	sourceComponents.push_back( "NetworkSourcePoseList" );
	sourceComponents.push_back( "NetworkSourcePositionList" );
	sourceComponents.push_back( "NetworkSourcePositionList2" );
	sourceComponents.push_back( "NetworkSourceEvent" );
	sourceComponents.push_back( "NetworkSourceMatrix3x3" );
	sourceComponents.push_back( "NetworkSourceMatrix3x4" );
	sourceComponents.push_back( "NetworkSourceMatrix4x4" );

	cf->registerModule< SourceModule >( sourceComponents );
}

} } // namespace Ubitrack::Drivers
