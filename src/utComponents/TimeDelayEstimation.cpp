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
 * TimeDelayEstimation component.
 * This file contains a TimeDelayEstimation of two asynchonious input ports
 *
 * @author Frieder Pankratz <pankratz@in.tum.de>
 * @author Christian Waechter <christian.waechter@in.tum.de>
 */
 
#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>

#include <utMath/Blas1.h>  // norm_2
#include <utMath/Stochastic/Average.h>
#include <utMath/Stochastic/Correlation.h>
#include <utMeasurement/Measurement.h>


// std
#include <map> // std::map
#include <vector> // std::vector
#include <string> // std::string
#include <algorithm> // std::transform

// log4cpp
#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.TimeDelayEstimation" ) );

namespace Ubitrack { namespace Components {

/**
 * This class supports the estimation of time delays between two different sensors with similar types of measurement data.
 */
template< typename EventType >
class TimeDelayEstimation
	: public Dataflow::Component
{
	
	typedef std::vector<double>::iterator vec_iter;
	
	/** underlying math type that is encapsulated from measurement */
	typedef typename EventType::value_type math_type;
	
	/** underlying built-in type that specifies the precision (e.g. \c double or \c float )  */
	typedef typename math_type::value_type precision_type;
	
	/** type of the sequence container that stores all events which are pushed to the component*/
	typedef typename std::vector< EventType > EventList;
	
	/** defines the sequence container of the mathematical datatype of the corresponding \c EventType */
	typedef typename std::vector< math_type > TypeList;
	
	/** type used to map time offsets [ms] to correlation . */
	typedef typename std::map< std::ptrdiff_t, std::vector< precision_type > > CorrelationMap;
	
protected:
	/** Reference port, time difference is calculated in respect to this port. */
	Dataflow::PushConsumer< EventType > m_inPortA;

	/** Input port B of the component. */
	Dataflow::PushConsumer< EventType > m_inPortB;
	
	/** Provides the estimated time difference */
	Dataflow::PushSupplier< Measurement::Distance > m_outPort;

	/// stores all measurements from 1st sensor being pushed to the component.
	EventList m_dataIn1;
	/// stores all measurements from 2nd sensor being pushed to the component.
	EventList m_dataIn2;
	
	/// stores all measurements from 1st sensor being pushed to the component.
	EventList m_threadData1;
	/// stores all measurements from 2nd sensor being pushed to the component.
	EventList m_threadData2;
	
	/// Stores interpolated data from 1st sensor.
	TypeList m_interpolatedData1;
	
	/// Stores interpolated data from 2nd sensor.
	TypeList m_interpolatedData2;
	
	/// remembers last common timestamp such that not every information needs to be recalculated.
	Measurement::Timestamp m_tNextInterpolation;
	
	/// stores the one-dimensional data from projected measurements of the 1st sensor.
	std::vector< precision_type > m_projectedData1;
	
	/// stores the one-dimensional data from projected measurements of the 2nd sensor.
	std::vector< precision_type > m_projectedData2;
	
	/// maps time differences [ms] to correlation factor
	CorrelationMap m_correlationData;
	
	// Mutex to check if storage is being filled (not used at the moment)
	// boost::mutex m_mutexStorage;
	
	/// Mutex to check if thread is already running
	boost::mutex m_mutexThread;
	
	/// Thread that handles calculation of cross-correlation
	boost::scoped_ptr< boost::thread > m_pThread;

	/// defines the time interval the time delay can maximaly reach.
	std::size_t m_maxTimeOffsetInMs;
	
	/// defines the length of the data chunks used for time delay estimation.
	std::size_t m_sliceSizeInMs;
	
	
public:

	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	TimeDelayEstimation( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::Component( sName )
		, m_inPortA( "AB1", *this, boost::bind( &TimeDelayEstimation::receiveSensor1, this, _1 )  )
		, m_inPortB( "AB2", *this, boost::bind( &TimeDelayEstimation::receiveSensor2, this, _1 )  )		
		, m_outPort( "Output", *this )
		, m_tNextInterpolation( 0 )
		, m_maxTimeOffsetInMs( 200 ) // <- size of window that will be moved across the data
		, m_sliceSizeInMs ( 2000 ) // <- size of a data chunk for comparison
    {
		LOG4CPP_INFO( logger, "Cross-correlation will use chunks of " <<  m_sliceSizeInMs << " ms and moving them across a " << m_maxTimeOffsetInMs << " ms sliding window." );
    }

	~TimeDelayEstimation()
	{
	}
	
	void receiveSensor1( const EventType& p )
	{
		m_dataIn1.push_back( p );
		startThread();
	}

	void receiveSensor2( const EventType& p )
	{
		m_dataIn2.push_back( p );
		startThread();
	}
	
	/// this functions starts the thread if not already running
	void startThread()
	{

		// do not interpolate if no data is available
		if( m_dataIn1.empty() || m_dataIn2.empty() )
			return;
	
		// do not interpolate data that is too short
		if( m_dataIn1.size() < 100 || m_dataIn2.size() < 100 )
			return;

		if( boost::mutex::scoped_try_lock ( m_mutexThread ) ) {
			m_threadData1 = m_dataIn1;
			m_threadData2 = m_dataIn2;
			m_pThread.reset( new boost::thread( boost::bind( &TimeDelayEstimation::computeTimedelay, this ) ) );
		} 
		//else
		//{
		//	LOG4CPP_WARN( logger, "Cannot perform cross-correlation, thread is still busy, sending old calibration data instead." );
		//	//std::cerr << "Cannot perform cross-correlation, thread is still busy, sending old calibration data instead.\n";
		//}
	}
	
	/// main function that performs all single steps after the other, should be called whenever a new measurement is added to the queues
	void computeTimedelay();
	
	/// function that interpolates data from the measurements to have a measurement at each single millisecond.
	Measurement::Timestamp updateInterpolatedData( const Measurement::Timestamp tStart, const Measurement::Timestamp tEnd, const EventList& eList, TypeList& interpolatedData ) const;
	
	/// function that projects all these single measurements to a single dimensional value
	void updateProjectedData( const TypeList& interpolatedSequence, std::vector< precision_type > &projected ) const;
	
	/**
	 * Function that projects the multi-dimensional data to a single dimension.
	 *
	 * Several template specialization for the various measurement types exist.
	 * @tparam InputIterator
	 * @tparam OutputIterator an iterator pointing to the container where the single dimensional data from the projection is stored.
	 * @param itBegin An iterator pointing to the beginning of a sequence, that stores the values that are projected.
	 * @param itEnd  An iterator pointing to the end of a sequence, that stores the values that are projected.
	 * @param itOut An iterator pointing to the sequence container, that stores the projected data, where new values will be inserted.
	 * @return An iterator pointing to the element that follows the last element written in the result sequence.
	 */
	template< typename InputIterator, typename OutputIterator >
	OutputIterator projectData( const InputIterator itBegin, const InputIterator itEnd, OutputIterator iOut ) const;

	/// performs the cross-correlation several times on different data chunks
	void updateDelayMap( std::vector< precision_type >& projected1, std::vector< precision_type >& projected2, CorrelationMap &mapping ) const;

	/**
	 * Function that performs the cross correlation by shifting chunks of measurements from one sensor along the other sensor's measurements
	 * within a time window of a certain length.
	 *
	 * @tparam InputIterator an iterator pointing to the sequence container with the single dimensional data.
	 * @param itBegin1 An iterator pointing to the beginning of the sequence that contains the 1st sensor's measurements.
	 * @param itEnd1 An iterator pointing to the end of the sequence that contains the 1st sensor's measurements.
	 * @param itBegin2 An iterator pointing to the beginning of the sequence that contains the 2nd sensor's measurements.
	 * @param itEnd2 An iterator pointing to the end of the sequence that contains the 2nd sensor's measurements.
	 * @param timeOffsetInMs 
	 * @param timeSliceInMs
	 * @param mapping
	 */
	template< typename InputIterator >
	void correlateBySlidingWindow( const InputIterator itBegin1, const InputIterator itEnd1
		, const InputIterator itBegin2
		, const std::size_t timeOffsetInMs
		, CorrelationMap& mapping ) const;
		
	/// selects the best matching offset, by comparing all so far estimated correlation factors.
	std::ptrdiff_t selectBestMatch( const CorrelationMap& mapping ) const;
};

template< typename EventType >
Measurement::Timestamp TimeDelayEstimation< EventType >::updateInterpolatedData( const Measurement::Timestamp tStart, const Measurement::Timestamp tEnd, const EventList& eList, TypeList& interpolatedData ) const
{
	const Measurement::Timestamp incOneMs = 1000000LL; // <- timestamp for 1 ms, used for incrementation and determination of steps

	const std::size_t numTimeSteps = ( tEnd - tStart ) / incOneMs;
	LOG4CPP_DEBUG( logger, "Starting to interpolate at " << numTimeSteps << " steps, each of one ms time in between." );

	// reserve some space for adding new elements
	interpolatedData.reserve( interpolatedData.size() + numTimeSteps );
	
	typename EventList::const_iterator it1;
	typename EventList::const_iterator it2 = eList.begin();
	typename EventList::const_iterator itLast = eList.begin();
	
	for( Measurement::Timestamp t_n = tStart; t_n<tEnd; t_n += incOneMs )
	{
		it1 = itLast;
		
		while( it1->time() < t_n )
			itLast = it1++;
		
		it2 = itLast;
		//or alternatively (but not really better):
		//std::advance( it2, std::distance( eList.begin(), itLast ) );
		
		while( it2->time() < t_n )
			++it2;
			
		assert( it2 != eList.end() );
		assert( itLast->time() <= it2->time() );
		
		const Measurement::Timestamp eventTimeDifference = it2->time() - itLast->time();
		const Measurement::Timestamp timeDiff = t_n - itLast->time();
		const double factor = (eventTimeDifference) ? static_cast< double >( timeDiff ) / static_cast< double >( eventTimeDifference ) : 1;
		
		LOG4CPP_TRACE( logger, "interpolating between " << (*(*it1)) << " and " << (*(*it2)) << " with following factor t=" << factor << " [0,1]." );
		const math_type value = (factor == 1) ? *( *it1 ) : linearInterpolate( *(*it1), *(*it2), factor );
		interpolatedData.push_back( value );
	}
	
	return tEnd;// + incOneMs;
}

template< typename EventType >
void TimeDelayEstimation< EventType >::updateProjectedData( const TypeList& interpolatedSequence, std::vector< precision_type > &projected ) const
{
	// project the interpolated data to one-dimensional values to perform the cross-correlation
	const std::size_t ni = interpolatedSequence.size();
	const std::size_t np = projected.size();
	
	// reserve some more space
	projected.reserve( ni );
	
	// set the iterator to the correct position within the sequence interpolation data sequence
	typename TypeList::const_iterator itBegin = interpolatedSequence.begin();
	std::advance( itBegin, np );
	
	// perform the projection using the iterators (==pointers)
	projectData( itBegin, interpolatedSequence.end(), std::back_inserter( projected ) );
}

template<>
template< typename InputIterator, typename OutputIterator >
OutputIterator TimeDelayEstimation< Measurement::Position >::projectData( const InputIterator itBegin, const InputIterator itEnd, OutputIterator iOut ) const
{
	// choose one of the following alternatives (should all do the same)
	// return std::transform( itBegin, itEnd, iOut, &Math::norm_2< math_type > );
	return std::transform( itBegin, itEnd, iOut, std::ptr_fun( &Math::norm_2< math_type > ) );
	// return std::transform( itBegin, itEnd, iOut, boost::numeric::ublas::norm_2< typename math_type::base_type > );
}

template< typename EventType >
void TimeDelayEstimation< EventType >::updateDelayMap( std::vector< precision_type >& projected1, std::vector< precision_type >& projected2, CorrelationMap& mapping ) const
{
	const std::size_t n = projected1.size();
	// this should be validated
	assert( n>0 );
	assert( n == projected2.size() );
	
	if( n < (m_maxTimeOffsetInMs*2) )
		return;
	
	LOG4CPP_DEBUG( logger, "Using " << n << " data values at one ms timestamps for cross-correlation." );
	
	
	// take chunks that are twice as big as the maximal offset is moved along the data
	const std::size_t numChunks = (n - m_maxTimeOffsetInMs*2) / m_sliceSizeInMs;

	LOG4CPP_INFO( logger, "Estimated " << numChunks << " chunks to perform cross-correlation");
	
	typename std::vector< precision_type >::iterator itBegin1 = projected1.begin();
	typename std::vector< precision_type >::iterator itBegin2 = projected2.begin();

	for( std::size_t i = 0; i<numChunks; ++i )
	{
		// shift the iterators of the first measurements to the starting position, slightly behind the second ones
		std::advance( itBegin1, m_maxTimeOffsetInMs );
		typename std::vector< precision_type >::iterator itEnd1 ( itBegin1 );
		std::advance( itEnd1, m_sliceSizeInMs );
		
		// do not shift the second ones right now, first estimate the offset
		correlateBySlidingWindow( itBegin1, itEnd1, itBegin2, m_maxTimeOffsetInMs, mapping );
		
		// and now shift them to a later position
		std::advance( itBegin2, m_maxTimeOffsetInMs );
		typename std::vector< precision_type >::iterator itEnd2 ( itBegin2 );
		std::advance( itEnd2, m_sliceSizeInMs );
	}
}

template< typename EventType >
template< typename InputIterator >
void TimeDelayEstimation< EventType >::correlateBySlidingWindow( const InputIterator itBegin1, const InputIterator itEnd1
	, const InputIterator itBegin2
	, const std::size_t timeOffsetInMs, CorrelationMap& mapping ) const
{
	typedef typename InputIterator::value_type value_type;
	
	InputIterator itBegin = itBegin2;
	InputIterator itEnd = itBegin;
	std::advance( itEnd, std::distance( itBegin1, itEnd1 ) );
	
	for( std::ptrdiff_t j = - static_cast< ptrdiff_t > ( timeOffsetInMs ); j< static_cast< ptrdiff_t > ( timeOffsetInMs ); ++j, ++itBegin, ++itEnd )
	{
		// use carefully, works only with random access iterator at the moment:
		const value_type corr = Math::Stochastic::correlation( itBegin1, itEnd1, itBegin, itEnd );
		mapping[ j ].push_back( corr );
	}
}

template< typename EventType >
void TimeDelayEstimation< EventType >::computeTimedelay( )
{
	boost::mutex::scoped_lock lock( m_mutexThread );
	
	
	// check if there is a change in internal storage that can be used and perform the change
		
	const Measurement::Timestamp startTime = m_tNextInterpolation ? m_tNextInterpolation : std::max( m_threadData1.front().time(), m_threadData2.front().time() );
	const Measurement::Timestamp endTime = std::min( m_threadData1.back().time(), m_threadData2.back().time() );
	if( !(startTime < endTime) )
		return;

	{	// starting to interpolate data for further processing
		updateInterpolatedData( startTime, endTime, m_threadData1, m_interpolatedData1 );
		m_tNextInterpolation = updateInterpolatedData( startTime, endTime, m_threadData2, m_interpolatedData2 );
		
		LOG4CPP_DEBUG( logger, "Storing now " << m_interpolatedData1.size() << " values of interpolated \"" << typeid( math_type ).name() << "\" values." );
	}
	
	{	//project the interpolated data to one dimensional values
		const std::size_t nDiff = m_interpolatedData1.size() - m_projectedData1.size();
		
		updateProjectedData( m_interpolatedData1, m_projectedData1 );
		updateProjectedData( m_interpolatedData2, m_projectedData2 );
		LOG4CPP_DEBUG( logger, nDiff << " values have been projected to one-dimensional data and added to internal storage for cross-correlation." );
	}
	
	m_correlationData.clear();
	{
		updateDelayMap( m_projectedData1, m_projectedData2, m_correlationData );
	}
	{
		const std::ptrdiff_t offset = selectBestMatch( m_correlationData );
		double result = static_cast< double >( offset );
		m_outPort.send( Measurement::Distance( Measurement::now(), Math::Scalar<double>(result) ) );
		LOG4CPP_INFO( logger, "[" << getName() << "]The delay between the sensor measurements has been estimated to " << offset << " ms, or briefly : time(1st sensor) == time(2nd sensor) + (result=" << result << ") [ms]." );
	}
}

template< typename EventType >
std::ptrdiff_t TimeDelayEstimation< EventType >::selectBestMatch( const CorrelationMap& mapping ) const
{
	std::ptrdiff_t bestOffset = 0;
	precision_type bestCorrelation = 0;
	
	const typename CorrelationMap::const_iterator itEnd = mapping.end();
	for( typename CorrelationMap::const_iterator it = mapping.begin(); it != itEnd; ++it )
	{
		Math::Stochastic::Average< double > averager;
		averager = std::for_each( it->second.begin(), it->second.end(), averager );
		const precision_type value = averager.getAverage();
		
		if( bestCorrelation < value )
		{
			bestCorrelation = value;
			bestOffset = it->first;
		}
	}
	LOG4CPP_DEBUG( logger, "The highest correlation factor " << bestCorrelation << " was estimated for a time offset of " << bestOffset << " [ms]." );
	return bestOffset;
}


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf )
{
	// the following two components first need a corresponding projectData to compile
	// cf->registerComponent< TimeDelayEstimation< Measurement::Pose > > ( "TimeDelayEstimationPose" );
	// cf->registerComponent< TimeDelayEstimation< Measurement::Rotation > > ( "TimeDelayEstimationRotation3D" );
	cf->registerComponent< TimeDelayEstimation< Measurement::Position > > ( "TimeDelayEstimationPosition3D" );
	
}

} } // namespace Ubitrack::Components