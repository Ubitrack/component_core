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
 * Print component
 * This class is a print component that prints
 * event received via a push port to the console.
 *
 * This is primary useful for debugging.
 *
 * @author Manuel Huber <huberma@in.tum.de>
 */

#include <string>
#include <iostream>

#include <boost/bind.hpp>

#include <utDataflow/PushConsumer.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

namespace Ubitrack { namespace Components {

using namespace Dataflow;

/**
 * @ingroup dataflow_components
 * Print component
 * This class is a print component that prints
 * event received via a push port to the console.
 *
 * This is primary useful for debugging.
 *
 * @par Input Ports
 * PushConsumer<EventType> port with name "Input".
 *
 * @par Output Ports
 * None.
 *
 * @par Configuration
 * None.
 *
 * @par Operation
 * Whenever an event is received via the input port it is
 * print out to the console.
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 * - Ubitrack::Measurement::Pose : PosePrintSink
 */
template< class EventType > 
class PrintSink 
	: public Component
{
public:
	/**
	 * Standard component constructor.
	 *
	 * @param nm Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	PrintSink( const std::string& nm, boost::shared_ptr< Graph::UTQLSubgraph > )
        : Ubitrack::Dataflow::Component( nm )
        , m_Port( "Input", *this, boost::bind( &PrintSink::printFunc, this, _1 ) )
	{
	}

protected:
	/**
	 * Handler method for input port
	 * Receives an event and prints it on cout.
	 * @param m the received event
	 */
	void printFunc ( const EventType& data )
	{
		std::cout << "Received Measurement for "
		  << this->getName() << ": "
		  << data << std::endl;
	}

	/** Input port of the component. */
	PushConsumer< EventType > m_Port;
};

//template< class EventType > void PrintSink::printFunc( const EventType& data )


template<> void PrintSink< Measurement::ErrorPosition >::printFunc( const Ubitrack::Measurement::ErrorPosition& data )
{
	std::cout << "Received Measurement for "
			  << getName() << ": ";
	std::cout << (*data).value;
	std::cout << (*data).covariance;
	std::cout << Measurement::timestampToShortString(data.time());
	std::cout << std::endl;
}

template<> void PrintSink< Measurement::PositionList >::printFunc( const Ubitrack::Measurement::PositionList& data )
{
	std::cout << "Received Measurement for "
			  << getName() << ": ";
	for (std::vector< Math::Vector< double, 3 > >::iterator it = data->begin();
		 it != data.get()->end(); ++it)
	{
	  std::cout << *it << " ";
	}
	std::cout << Measurement::timestampToShortString(data.time());
	std::cout << std::endl;
}

template<> void PrintSink< Measurement::PoseList >::printFunc( const Ubitrack::Measurement::PoseList& data )
{
	std::cout << "Received Measurement for "
			  << getName() << ": ";
	for (std::vector< Math::Pose >::iterator it = data->begin();
		 it != data.get()->end(); ++it)
	{
	  std::cout << *it << " ";
	}
	std::cout << Measurement::timestampToShortString(data.time());
	std::cout << std::endl;
}

template<> void PrintSink< Measurement::DistanceList >::printFunc( const Ubitrack::Measurement::DistanceList& data )
{
	std::cout << "Received Measurement for "
			  << getName() << ": ";
	for (std::vector< Math::Scalar< double > >::iterator it = data->begin();
		 it != data.get()->end(); ++it)
	{
	  std::cout << *it << " ";
	}
	std::cout << Measurement::timestampToShortString(data.time());
	std::cout << std::endl;
}

template<> void PrintSink< Measurement::IDList >::printFunc( const Ubitrack::Measurement::IDList& data )
{
	std::cout << "Received Measurement for "
			  << getName() << ": ";
	for (std::vector< Math::Scalar< unsigned long > >::iterator it = data->begin();
		 it != data.get()->end(); ++it)
	{
	  std::cout << *it << " ";
	}
	std::cout << Measurement::timestampToShortString(data.time());
	std::cout << std::endl;
}

template<> void PrintSink< Measurement::PositionList2 >::printFunc( const Ubitrack::Measurement::PositionList2& data )
{
	std::cout << "Received Measurement for "
			  << getName() << ": ";
	for (std::vector< Math::Vector< double, 2 > >::iterator it = data->begin();
 it != data.get()->end(); ++it)
	{
	  std::cout << *it << " ";
	}
	std::cout << Measurement::timestampToShortString(data.time());
	std::cout << std::endl;
}

UBITRACK_REGISTER_COMPONENT( ComponentFactory* const cf ) {
	cf->registerComponent< PrintSink< Measurement::Pose > > ( "PosePrintSink" );
	cf->registerComponent< PrintSink< Measurement::ErrorPose > > ( "ErrorPosePrintSink" );
	cf->registerComponent< PrintSink< Measurement::Position > > ( "PositionPrintSink" );
	cf->registerComponent< PrintSink< Measurement::ErrorPosition > > ( "ErrorPositionPrintSink" );
	cf->registerComponent< PrintSink< Measurement::Position2D > > ( "Position2PrintSink" );
	cf->registerComponent< PrintSink< Measurement::Vector4D > > ( "Vector4PrintSink" );
	cf->registerComponent< PrintSink< Measurement::Rotation > > ( "RotationPrintSink" );
	cf->registerComponent< PrintSink< Measurement::RotationVelocity > > ( "RotationVelocityPrintSink" );
	cf->registerComponent< PrintSink< Measurement::Distance > > ( "DistancePrintSink" );
	cf->registerComponent< PrintSink< Measurement::IDList > > ( "IDListPrintSink" );
	cf->registerComponent< PrintSink< Measurement::PositionList > > ( "PositionListPrintSink" );
	cf->registerComponent< PrintSink< Measurement::PositionList2 > > ( "PositionList2PrintSink" );
	cf->registerComponent< PrintSink< Measurement::PoseList > > ( "PoseListPrintSink" );
	cf->registerComponent< PrintSink< Measurement::DistanceList > > ( "DistanceListPrintSink" );
	cf->registerComponent< PrintSink< Measurement::Button > > ( "ButtonPrintSink" );
	cf->registerComponent< PrintSink< Measurement::Matrix4x4 > > ( "Matrix4x4PrintSink" );
	cf->registerComponent< PrintSink< Measurement::Matrix3x3 > > ( "Matrix3x3PrintSink" );
	cf->registerComponent< PrintSink< Measurement::Matrix3x4 > > ( "Matrix3x4PrintSink" );
	cf->registerComponent< PrintSink< Measurement::CameraIntrinsics > > ( "CameraIntrinsicsPrintSink" );
}

} } // namespace Ubitrack::Components

