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
 * This class calculates the average of a various List measurements.
 *
 * @author Florian Echtler <echtler@in.tum.de>
 * @author Christian Waechter <christian.waechter@in.tum.de> (modified)
 */

// Ubitrack
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/ComponentFactory.h>

#include <utMeasurement/Measurement.h>
#include <utMath/Stochastic/Average.h>

//std
#include <algorithm> // std::for_each

// currently not used
#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.Average" ) );

namespace Ubitrack { namespace Components {

using namespace Dataflow;


template< class EventType, class ResultType >
class Average
	: public TriggerComponent
{
	/** abbreviation of sequence container for the incoming measurements. */
	typedef typename std::vector< typename EventType::value_type > EventList;
protected:

	/** Input port of the component supporting various spatial measurements. */
	Dataflow::ExpansionInPort< typename EventType::value_type > m_inPort;
	
	/** Output port of the component supporting corresponding measurements representing a mean value. */
	Dataflow::TriggerOutPort< ResultType > m_outPort;
	
	/** an index that points to the latest position of the previous averaging step,
	 * only new incoming measurement values after this step will be added to the average. */
	std::size_t m_lastIndex;

	/** Class that maintains a running average such that only the update needs to be calculated. */
	Math::Stochastic::Average< typename ResultType::value_type > m_averager;
	
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
		, m_lastIndex( 0 )
	{
		LOG4CPP_DEBUG( logger, "Started to average measurements of type \"" << typeid( typename EventType::value_type ).name() 
			<< "\", will result in a measurement of type \"" << typeid( typename ResultType::value_type ).name() << "\"." );
	}

	/** called when a new item arrives */
	void compute( Measurement::Timestamp t )
	{	
		typename EventList::const_iterator itBegin = m_inPort.get()->begin();	
		const typename EventList::const_iterator itEnd = m_inPort.get()->end();
		
		// calculate some distances for later use
		const std::size_t n = std::distance( itBegin, itEnd );
		const std::size_t nDiff ( n-m_lastIndex );
		
		// move iterator to first position that has not been used for the average so far
		std::advance( itBegin, m_lastIndex );
		m_lastIndex+= nDiff;
		
		m_averager = std::for_each( itBegin, itEnd, m_averager );
		const typename ResultType::value_type result = m_averager.getAverage();
		
		m_outPort.send( ResultType( t, result ) );
		
		LOG4CPP_TRACE( logger, "Updated average with " << nDiff << " new measurement(s) of type \"" << typeid( typename EventType::value_type ).name()
			<< "\", \naverage estimated to " << result << " from " << m_lastIndex << " measurement(s)." );
	}

};


UBITRACK_REGISTER_COMPONENT( ComponentFactory* const cf ) 
{
	cf->registerComponent< Average< Measurement::Pose, Measurement::Pose > > ( "PoseListAverage" );
	cf->registerComponent< Average< Measurement::Distance, Measurement::Distance > > ( "DistanceListAverage" );
	cf->registerComponent< Average< Measurement::Rotation, Measurement::Rotation > > ( "RotationListAverage" );
	cf->registerComponent< Average< Measurement::Position, Measurement::Position > > ( "PositionListAverage" );
	cf->registerComponent< Average< Measurement::Position2D, Measurement::Position2D > > ( "PositionList2DAverage" );

	// with Covariance output
	cf->registerComponent< Average< Measurement::Position, Measurement::ErrorPosition > > ( "PositionListAverageError" );
	cf->registerComponent< Average< Measurement::Pose, Measurement::ErrorPose > > ( "PoseListAverageError" );
}

} } // namespace Ubitrack::Components

