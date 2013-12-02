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
 * Average component.
 * This class calculates the average of a List measurement.
 *
 * @author Florian Echtler <echtler@in.tum.de>
 * @author Christian Waechter <christian.waechter@in.tum.de> (modified)
 */

// Ubitrack
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/Exception.h>
#include <utTracking/Average.h>

// Boost
#include <boost/math/constants/constants.hpp>


//LOG4CPP
//#include <log4cpp/Category.hh>
//static log4cpp::Category& eventLogger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.Average" ) );
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.Average" ) );

namespace Ubitrack { namespace Components {

using namespace Dataflow;
namespace ublas = boost::numeric::ublas;

template< class EventType, class ResultType >
class Average
	: public TriggerComponent
{
public:
	/**
	 * Standard component constructor.
	 *
	 * @param nm Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	Average( const std::string& nm, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
		: Dataflow::TriggerComponent( nm, pCfg )
		, m_inPort( "Input", *this )
		, m_outPort( "Output", *this )
	{
	}

protected:
	Tracking::Average<typename EventType::value_type,typename ResultType::value_type> m_average;
	
	Dataflow::ExpansionInPort< typename EventType::value_type > m_inPort;	
	Dataflow::TriggerOutPort< ResultType > m_outPort;
	/** called when a new item arrives */
	void compute( Measurement::Timestamp t )
	{			
		std::vector<typename EventType::value_type> eventList = *m_inPort.get();	
		typename ResultType::value_type tmp = m_average.mean( eventList );
		m_outPort.send( ResultType( t, tmp ) );		
	}

};


UBITRACK_REGISTER_COMPONENT( ComponentFactory* const cf ) 
{
	cf->registerComponent< Average< Measurement::Distance, Measurement::Distance > > ( "DistanceListAverage" );
	cf->registerComponent< Average< Measurement::Position2D, Measurement::Position2D > > ( "PositionList2DAverage" );
	cf->registerComponent< Average< Measurement::Position, Measurement::Position > > ( "PositionListAverage" );
	cf->registerComponent< Average< Measurement::Pose, Measurement::Pose > > ( "PoseListAverage" );
	cf->registerComponent< Average< Measurement::Rotation, Measurement::Rotation > > ( "RotationListAverage" );
	
	// with Covariance
	cf->registerComponent< Average< Measurement::Position, Measurement::ErrorPosition > > ( "PositionListAverageError" );
	cf->registerComponent< Average< Measurement::Pose, Measurement::ErrorPose > > ( "PoseListAverageError" );
}

} } // namespace Ubitrack::Components

