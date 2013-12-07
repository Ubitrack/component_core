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
 * Components that pulls its input at a given frequency and pushes the result
 *
 * @author Peter Keitler <keitler@in.tum.de>
 */

#include <string>

#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>


#include <boost/math/constants/constants.hpp> // PI

#include <utDataflow/Component.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/Exception.h>
#include <utUtil/OS.h>


using namespace Ubitrack;
namespace ublas = boost::numeric::ublas;

// get a logger
#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.CovarianceEstimation" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Covariance estimation component
 *
 * @par Input Ports
 * PushConsumer< EventType > with name "PerturbedInput".
 * PushConsumer< Button > with name "TriggerInput".
 *
 * @par Output Ports
 * PushSupplier< EventType > with name "Distribution".
 * PushSupplier< Button > with name "Sync".
 *
 * @par Configuration
 *
 * @par Operation
 *
 * Determines the covariance of a given stream of poses. The component
 * has to be triggered once via its {@code TriggerInput}. It then
 * generates many trigger events on its {@code Sync} output. Those
 * should lead to the same amount of measurements pushed on the {@code
 * PerturbedInput} input. Finally, one result is pushed onward on the
 * {@code Distribution} output.
 *
 * @par Instances
 *
 */
template< class EventType, class ResultType >
class CovarianceEstimation
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	CovarianceEstimation( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Dataflow::Component( sName )
		, m_inPortPerturbed( "PerturbedInput", *this, boost::bind( &CovarianceEstimation::dataIn, this, _1 ) )
		, m_inPortTrigger( "TriggerInput", *this, boost::bind( &CovarianceEstimation::triggerIn, this, _1 ) )
		, m_outPortSync( "Sync", *this )
		, m_outPortDist ( "Distribution", *this )
		, m_bStopped( true )
		, m_counter ( 0 )
		, m_size( 100 )
		, m_button( ' ' )
		, m_inButton( ' ' )
    {
		LOG4CPP_DEBUG( logger, "Setup CovarianceEstimation component" );

		// read configuration for amount of samples to use
		subgraph->m_DataflowAttributes.getAttributeData( "size", m_size );

		// read button key
		std::string button( " " );
		std::string inButton( " " );

		if ( subgraph->m_DataflowAttributes.hasAttribute( "button" ) )
			button = subgraph->m_DataflowAttributes.getAttributeString( "button" );
		if ( subgraph->m_DataflowAttributes.hasAttribute( "inButton" ) )
			inButton = subgraph->m_DataflowAttributes.getAttributeString( "inButton" );
			
		m_button = Math::Scalar< int >( button[ 0 ] );
		m_inButton = Math::Scalar< int >( inButton[ 0 ] );

		mean = Math::Vector< double, 7 >::zeros();
		outProd = Math::Matrix< double, 7, 7 >::zeros( );
    }


	~CovarianceEstimation()
	{}


    void triggerIn( const Measurement::Button& e )
    {
		if ( *e == m_inButton )
		{
			// lock the file to prevent other threads from writing simultaneously
			boost::mutex::scoped_lock l( m_mutex );

			if ( m_bStopped ) 
			{
				LOG4CPP_DEBUG( logger, getName() << " Received trigger event with timestamp " << e.time() << ". Invoke computation by sending first sync signal..." );
				m_outPortSync.send( Measurement::Button( e.time(), m_button ) );
				m_bStopped = false;

				// Reset internal state
				m_counter = 0;
				mean = Math::Vector< double, 7 >::zeros();
				outProd = Math::Matrix< double, 7, 7 >::zeros();
			}
			else 
			{
				LOG4CPP_ERROR( logger, getName() << " received trigger signal while computation was already running. Ignored." );
			}
		}
	}


	/**
	 * Handler method for input of perturbed measurement data. All
	 * measurements are collected and will contribute to the finally
	 * generated distribution which will be computed and pushed onward
	 * as soon as the configured amount of trials is reached.
	 *
	 * It is not necessary to trigger the compoent before via the
	 * {@code TriggerInput} input and noisy data may be pushed
	 * asynchronously. However, each push will result in an event to
	 * be issued on the {@code Sync} output.
	 */
    void dataIn( const EventType& e )
    {
		LOG4CPP_TRACE( logger, getName() << " Received perturbed measurement with timestamp " << e.time() );

		if ( m_bStopped ) 
		{
			LOG4CPP_TRACE( logger, getName() << " Covariance estimation has not been triggered yet, ignore measurement" );
			return;
		}
			
		m_counter ++;
		LOG4CPP_TRACE( logger, getName() << " Current counter: " << m_counter << ", go on until: " << m_size );

		// Compute incremental covariance
		typename ResultType::value_type estimate = incrementalEstimate ( *e );

		// Check list size
		if ( m_counter == m_size )
		{
			// push onward final result
			LOG4CPP_DEBUG( logger, getName() << " Terminate and push final result" );
			m_outPortDist.send( ResultType ( e.time(), estimate ) );

			boost::mutex::scoped_lock l( m_mutex );
			m_bStopped = true;

			return;
		}
		
		// If not reached, send next trigger event
		LOG4CPP_TRACE( logger, getName() << " Triggering computation..." );
		m_outPortSync.send( Measurement::Button( Measurement::now(), m_button ) );
	}
	

protected:
	typename ResultType::value_type incrementalEstimate( typename EventType::value_type& perturbed );

	/** Input port of the component. */
	Dataflow::PushConsumer< EventType > m_inPortPerturbed;
	Dataflow::PushConsumer< Measurement::Button > m_inPortTrigger;

	// Output ports of the component
	Dataflow::PushSupplier< Measurement::Button > m_outPortSync;
	Dataflow::PushSupplier< ResultType > m_outPortDist;

	// stop
	bool m_bStopped;

	Math::Vector< double > mean;
	Math::Matrix< double, 0, 0 > outProd;

	int m_counter;
	int m_size;

	Math::Scalar< int > m_button;
	Math::Scalar< int > m_inButton;

	boost::mutex m_mutex;
};


template<>
Math::ErrorVector< double, 3 > CovarianceEstimation< Measurement::Position, Measurement::ErrorPosition >::incrementalEstimate( Math::Vector< double, 3 >& posNew )
{
	ublas::vector_range< typename Math::Vector< double >::base_type > posMean ( mean, ublas::range( 0, 3 ) );
	ublas::matrix_range< typename Math::Matrix< double, 0, 0 >::base_type > outProd3 ( outProd, ublas::range ( 0, 3 ), ublas::range ( 0, 3 ) );

 	// Running mean value of position random variable
 	posMean = ( ( ((double)m_counter - 1) / (double)m_counter ) * posMean ) + ( ( 1 / (double)m_counter ) * posNew );

	// Running outer product of position random variable (not yet normalized by number of measurements)
 	outProd3 = outProd3 + ublas::outer_prod( posNew, posNew );

 	// Compute covariance matrix
 	if ( m_counter == 1 ) 
 	{
 		LOG4CPP_TRACE( logger, "Not enough data to compute covariance matrix" );
 		return Math::ErrorVector< double, 3 >();
 	}

 	Math::ErrorVector< double, 3 > ev ( posMean, outProd3 / ((double)m_counter) - ublas::outer_prod ( posMean, posMean ) );

	LOG4CPP_TRACE( logger, "Running (empirical) mean / covariance: " << std::endl << ev );

 	return ev;
}


template<>
Math::ErrorPose CovarianceEstimation< Measurement::Pose, Measurement::ErrorPose >::incrementalEstimate( Math::Pose& poseNew )
{
	ublas::vector_range< typename Math::Vector< double >::base_type > posMean( mean, ublas::range( 0, 3 ) );
	ublas::vector_range< typename Math::Vector< double >::base_type > rotMean( mean, ublas::range( 3, 7 ) );

	LOG4CPP_TRACE ( logger, "Update pose event: " << poseNew );

	// The order is tx, ty, tz, qx, qy, qz, qw.
	Math::Vector< double > poseNewVec( 7 );
	poseNew.toVector( poseNewVec );
	ublas::vector_range< typename Math::Vector< double >::base_type > posNew( poseNewVec, ublas::range( 0, 3 ) );
	ublas::vector_range< typename Math::Vector< double >::base_type > rotNew( poseNewVec, ublas::range( 3, 7 ) );

	// Take care of quaternion ambiguity
 	if ( ublas::inner_prod( rotNew, rotMean ) < 0 )
 		rotNew *= -1;

	// Update running mean value
	mean = ( ( ((double)m_counter - 1) / (double)m_counter ) * mean ) + ( ( 1 / (double)m_counter ) * poseNewVec );

	// Running outer product of pose random variable (not yet normalized by number of measurements)
	outProd = outProd + ublas::outer_prod( poseNewVec, poseNewVec );

	/*
	 * Use inverted mean value to transform the additive 7x7
	 * covariance to the 6x6 multiplicative format The conversion is
	 * conducted according to the following formulas:
	 * 
	 * q_m = q_0 * ( q_id + q_e )
	 * 
	 * where q_id is the identity quaternion and q_e is a quaternion
	 * with expectation ((0,0,0),0) and a covariance covering only the
	 * imaginary part. Together ( q_id + q_e ) represent a small
	 * quaternion ((e_rx, e_ry, e_rz), 1). If mean and covariance of
	 * the quaternion are estimated according to the usual formulas,
	 * however, one gets the following instead:
	 * 
	 * q_m = q_0 + q'_e
	 * 
	 * Together with the first formula, this yields
	 * 
	 * q_0 * ( q_id + q_e ) = q_0 + q'_e
	 * ( q_id + q_e )       = ~q_0 * q_0 + ~q_0 * q'_e
	 * q_e                  = q_id + ~q_0 * q'_e - q_id
	 * q_e                  = ~q_0 * q'_e
	 *
	 * Thus, one has to rotate the distribution by ~q_0. The variance
	 * of the real part can then be discarded, it should be ~0.
	 */

	Math::Vector< double, 7 > invMean;
	(~(Math::Pose::fromVector( mean ) ) ).toVector( invMean );
	Math::ErrorVector< double, 7 > ev ( invMean, outProd / ( (double)m_counter ) - ublas::outer_prod ( mean, mean ) );
	Math::ErrorPose invEp = Math::ErrorPose::fromAdditiveErrorVector( ev );
	
	// We created the error pose from the inverted mean value above, to obtain the transformed 6x6 covariance
	// Now, we recreate the error pose with the computed mean value.
	Math::ErrorPose ep( Math::Pose::fromVector( mean ), invEp.covariance() );

	LOG4CPP_TRACE( logger, "Running (empirical) mean / covariance: " << std::endl << ep );

	// For debug purposes, compute positional and angular error...
	Math::Matrix< double, 6, 6 > covar = ep.covariance();
	double posRms = sqrt ( covar (0,0) + covar (1,1) + covar (2,2) );
	LOG4CPP_INFO( logger, "RMS positional error [mm]: " << posRms );
	Math::Vector< double, 3 > axis;
	axis (0) = sqrt ( covar (3,3) );
	axis (1) = sqrt ( covar (4,4) );
	axis (2) = sqrt ( covar (5,5) );
	double norm = norm_2 (axis);
	double phi = asin ( norm ) * 2;
	phi = phi * 180 / boost::math::constants::pi<double>();
	LOG4CPP_INFO( logger, "Standard deviation of rotational error [deg]: " << phi );
	
	return ep;
}



UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< CovarianceEstimation< Measurement::Pose, Measurement::ErrorPose > > ( "PoseCovarianceEstimation" );
	cf->registerComponent< CovarianceEstimation< Measurement::Position, Measurement::ErrorPosition > > ( "3DPositionCovarianceEstimation" );
}

} } // Namespace Ubitrack::Components
