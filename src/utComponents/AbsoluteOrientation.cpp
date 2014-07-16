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
 * Absolute orientation component.
 * This file contains a component to compute the Absolute Orientation problem.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 * @author Christian Waechter <christian.waechter@in.tum.de>
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>

#include <utMeasurement/Measurement.h>

#include <utAlgorithm/PoseEstimation3D3D/Ransac.h>
#include <utAlgorithm/PoseEstimation3D3D/AbsoluteOrientation.h>
#include <utAlgorithm/PoseEstimation3D3D/CovarianceEstimation.h>




// //#define DO_TIMING
// #ifdef DO_TIMING
// #include <utUtil/BlockTimer.h>
// #endif

// get a logger
// log4cpp
#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.3D3DPoseEstimation" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Absolute orientation component.
 * This class contains a component to compute the Absolute Orientation (3D-3D pose estimation) problem.
 *
 * @par Input Ports
 * ExpansionInPort<Position> with name "InputA".
 * ExpansionInPort<Position> with name "InputB".
 *
 * @par Output Ports
 * TriggerOutPort<Pose> with name "Output".
 *
 * @par Configuration
 * DataflowConfiguration: expansion="space" or "time" for time/space expansion
 *
 * @par Operation
 * The component computes the transformation from a coordinate system A to a coordinate system B,
 * given corresponding points in A (InputA) and B (InputB). For details see
 * \c Ubitrack::Algorithm::calculateAbsoluteOrientation.
 */
template< class ResultType >
class AbsoluteOrientationComponent
	: public Dataflow::TriggerComponent
{
	/// type of the measurements expected as input, some kind of 3-vector.
	typedef Math::Vector< double, 3 > value_type;
	
	/// type of sequence container of the measurements.
	typedef std::vector< Math::Vector< double, 3 > > EventType;
	
	/// definition of built-in datatype that specifies the precision ( e.g. \c double or \c float ).
	typedef typename ResultType::value_type::value_type precision_type;
	
protected:
		
	/** Input port A of the component. */
	Dataflow::ExpansionInPort< value_type > m_inPortA;

	/** Input port B of the component. */
	Dataflow::ExpansionInPort< value_type > m_inPortB;

	/** Output port of the component supporting pose measurements . */
	Dataflow::TriggerOutPort< ResultType > m_outPort;
	
	/** Output port of the component supporting error measurements. @todo make this a TriggerPort as well*/
	Dataflow::PushSupplier< Measurement::ErrorPose > m_outErrorPort;
	
	/// signs if robust (==RANSAC)  calibration is applied.
	bool useRobust;
	
	/// defines the acceptance (tolerance level) of when a measurement is accepted as an inlier.
	precision_type threshold;
	
	/// defines the size of the minimal consensus set of corresponding measurements.
	std::size_t minSetSize;
	
	/// percent of outlier that are expected to appear in the set of corresponding measurements.
	precision_type percentOutlier;
	
	/// desired probability of a valid result when RANSAC returns a solution of the pose estimation.
	precision_type successProbability;
	
	/// a function that is specialized to either send pose information with or without covariance.
	void sendResult( const EventType& eventA, const Measurement::Pose& pose, const EventType& eventB );
	
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	AbsoluteOrientationComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Dataflow::TriggerComponent( sName, subgraph )
		, m_inPortA( "InputA", *this )
		, m_inPortB( "InputB", *this )
		, m_outPort( "Output", *this )
		, m_outErrorPort( "ErrorPose", *this )
		, useRobust( false )
		, threshold( 0.025 )  // <- this works quite well here, you also might want to try something up to 0.05
		, minSetSize( 3 )  // <- the algorithm needs at least 3 poses to estimate a result (minimum consensus set)
		, percentOutlier( 0.4 ) // <- should be a reasonable number, maybe derived empirically
		, successProbability( 0.99 ) // <- usually you want to have this parameter quite high (~95% or higher)
    {
		generateSpaceExpansionPorts( subgraph );
		
		if( !subgraph->m_DataflowAttributes.hasAttribute( "enableRANSAC" ) )
			return;
	
		useRobust = subgraph->m_DataflowAttributes.getAttributeString( "enableRANSAC" ) == "true";
		LOG4CPP_INFO( logger, "Starting as " << ( useRobust ? "robust component using RANSAC" : "standard component" ) << ".");
		
		if( !useRobust) 
			return;
		
		if( subgraph->m_DataflowAttributes.hasAttribute( "threshold" ) )
			subgraph->m_DataflowAttributes.getAttributeData( "threshold", threshold );
		
		if( subgraph->m_DataflowAttributes.hasAttribute( "minSetSize" ) )
			subgraph->m_DataflowAttributes.getAttributeData( "minSetSize", minSetSize );
		
		if( subgraph->m_DataflowAttributes.hasAttribute( "percentOutlier" ) )
			subgraph->m_DataflowAttributes.getAttributeData( "percentOutlier", percentOutlier );
		
		if( subgraph->m_DataflowAttributes.hasAttribute( "successProbability" ) )
			subgraph->m_DataflowAttributes.getAttributeData( "successProbability", successProbability );
		
		
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		const std::size_t n_3Dpts = m_inPortA.get()->size();
		const std::size_t n = m_inPortB.get()->size();
		
		if ( n_3Dpts != n )
			UBITRACK_THROW( "Illegal number of correspondences" );
		if ( n_3Dpts < 3 )
			UBITRACK_THROW( "Insufficient correspondences" );


		Math::Pose resultPoseBA;
		{	// determine the pose
	
			bool success = false;
			if( !useRobust )
				success = Algorithm::PoseEstimation3D3D::estimatePose6D_3D3D( *m_inPortB.get(), resultPoseBA , *m_inPortA.get() );
			else
			{
				const Ubitrack::Math::Optimization::RansacParameter< precision_type > params( threshold, minSetSize, n_3Dpts, percentOutlier, successProbability );
				success = Algorithm::PoseEstimation3D3D::estimatePose6D_3D3D( *m_inPortB.get(), resultPoseBA , *m_inPortA.get() , params );	
			}
			
			if( !success )
				UBITRACK_THROW( "Cannot estimate a result, some problem occurred while performing a 3d-3d pose estimation (absolute orientation)." );
		}
		
		LOG4CPP_DEBUG( logger, "Estimated a pose=" << resultPoseBA << ", starting to push to outgoing ports." );
		sendResult( *m_inPortB.get(), Measurement::Pose( t, resultPoseBA ), *m_inPortA.get() );
    }
};


template <>
void AbsoluteOrientationComponent< Measurement::ErrorPose >::sendResult( const EventType& eventA, const Measurement::Pose& pose, const EventType& eventB )
{
	{
		//estimate the covariance using covariance backward propagation
	
		Ubitrack::Math::Matrix< precision_type, 6, 6 > cov = Ubitrack::Math::Matrix< precision_type, 6, 6 >::zeros();
		const bool success = Ubitrack::Algorithm::PoseEstimation3D3D::estimatePose6DCovariance( eventA.begin(), eventA.end(), *pose, eventB.begin(), eventB.end(), cov );
		if( success )
		{
			const Math::ErrorPose ep( *pose, cov );
			m_outPort.send( Measurement::ErrorPose( pose.time(), ep ) );
			m_outErrorPort.send( Measurement::ErrorPose( pose.time(), ep ) );
		}
		else
			UBITRACK_THROW( "Cannot estimate a result, some problem occurred while calculating the covariance." );
	}
}

template <>
void AbsoluteOrientationComponent< Measurement::Pose >::sendResult( const EventType& eventA, const Measurement::Pose& pose, const EventType& eventB )
{
	m_outPort.send( pose );
	
	if( !m_outErrorPort.isConnected() )
		return;
		
	{	//estimate the covariance using covariance backward propagation
	
		Ubitrack::Math::Matrix< precision_type, 6, 6 > cov = Ubitrack::Math::Matrix< precision_type, 6, 6 >::zeros();
		const bool success = Ubitrack::Algorithm::PoseEstimation3D3D::estimatePose6DCovariance( eventA.begin(), eventA.end(), *pose, eventB.begin(), eventB.end(), cov );
		if( success )
		{
			const Math::ErrorPose ep( *pose, cov );
			m_outErrorPort.send( Measurement::ErrorPose( pose.time(), ep ) );
		}
		else
			UBITRACK_THROW( "Cannot estimate a result, some problem occurred while calculating the covariance." );
	}
}

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf )
{
	// for the old pattern
	cf->registerComponent< AbsoluteOrientationComponent< Measurement::Pose > > ( "AbsoluteOrientation" );
	cf->registerComponent< AbsoluteOrientationComponent< Measurement::ErrorPose > > ( "AbsoluteOrientationError" );
	// while the new component should support this
	cf->registerComponent< AbsoluteOrientationComponent< Measurement::Pose > > ( "3D3DPoseEstimation" );
	
}

} } // namespace Ubitrack::Components
