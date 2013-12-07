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
 * 2D-3D pose optimization component for multiple-camera systems.
 * Requires an initial pose!
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */
#include <log4cpp/Category.hh>

#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

//#include <utCalibration/Function/MultipleCameraProjectionErrorART.h>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.2D3DPoseEstimation" ) );
//static log4cpp::Category& optLogger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.2D3DPoseEstimation.LM" ) );

//#define OPTIMIZATION_LOGGING
#include <utMath/LevenbergMarquardt.h>
#include <utMath/BackwardPropagation.h>

#include <utMath/NewFunction/Function.h>
#include <utMath/NewFunction/Addition.h>
#include <utMath/NewFunction/Dehomogenization.h>
#include <utMath/NewFunction/LieRotation.h>
#include <utMath/NewFunction/LinearTransformation.h>


namespace Ubitrack { namespace Components {

/**
 * Function to minimize. Input is a 6-vector containing translation and exponential map rotation.
 */ 
template< class VType = double >
class ObjectiveFunction
{
public:
	ObjectiveFunction( const std::vector< Math::Vector< VType, 3 > >& p3D, 
		const std::vector< Math::Matrix< double, 3, 3 > >& cameraRotations, 
		const std::vector< Math::Vector< double, 3 > >& cameraTranslations, 
		const std::vector< Math::Matrix< VType, 3, 3 > >& cameraIntrinsics, 
		const std::vector< std::pair< unsigned, unsigned > > visibilities )
		: m_p3D( p3D )
		, m_camR( cameraRotations )
		, m_camT( cameraTranslations )
		, m_camI( cameraIntrinsics )
		, m_vis( visibilities )
	{}

	/**
	 * return the size of the result vector
	 */
	unsigned size() const
	{ return 2 * m_vis.size(); }


	/**
	 * @param result vector to store the result in
	 * @param input containing the parameters (target pose as 7-vector)
	 * @param J matrix to store the jacobian (evaluated for input) in
	 */
	template< class VT1, class VT2, class MT > 
	void evaluateWithJacobian( VT1& result, const VT2& input, MT& J ) const
	{
		namespace NF = Math::Function;
		namespace ublas = boost::numeric::ublas;
		
		for ( unsigned i = 0; i < m_vis.size(); i++ )
		{
				ublas::vector_range< VT1 > subResult( result, ublas::range( i * 2, ( i + 1 ) * 2 ) );
				ublas::matrix_range< MT > subJ( J, ublas::range( i * 2, ( i + 1 ) * 2 ), ublas::range( 0, 6 ) );
		
				( NF::Dehomogenization< 3 >() <<
				( NF::LinearTransformation< 3, 3 >( m_camI[ m_vis[ i ].second ] ) <<
					( NF::Addition< 3 >() <<
						( NF::fixedParameterRef< 3 >( m_camT[ m_vis[ i ].second ] ) ) <<
						( NF::LinearTransformation< 3, 3 >( m_camR[ m_vis[ i ].second ] ) <<
							( NF::Addition< 3 >() <<
								( NF::parameter< 3 >( 0 ) ) <<
								( NF::LieRotation() <<
									( NF::parameter< 3 >( 3  ) ) <<
									( NF::fixedParameterRef< 3 >( m_p3D[ m_vis[ i ].first ] ) )
								)
							)
						)
					)
				)
			).evaluateWithJacobian( input, subResult, subJ );
		}
	}
	
protected:
	const std::vector< Math::Vector< VType, 3 > >& m_p3D;
	const std::vector< Math::Matrix< double, 3, 3 > >& m_camR;
	const std::vector< Math::Vector< double, 3 > >& m_camT;
	const std::vector< Math::Matrix< VType, 3, 3 > >& m_camI;
	const std::vector< std::pair< unsigned, unsigned > > m_vis;
};


/**
 * @ingroup dataflow_components
 * 2D-3D pose optimization component for multiple-camera systems.
 * 
 * Given pairs of corresponding 2D and 3D points in multiple cameras, the component computes the 
 * pose. An initial pose estimation is required.
 *
 * @par Operation
 * The component refines a 6D pose from corresponding 2d and 3d points by nonlinear optimization.
 * The compoent can also output the covariance matrix. Note, however, that the covariance matrix is in A.R.T. format, not Ubitrack!
 */
class MultipleCameraPoseOptimization
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	MultipleCameraPoseOptimization( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > config )
		: Dataflow::TriggerComponent( sName, config )
		, m_in3d( "Input3d", *this )
		, m_in2d( "Input2d", *this )
		, m_inWeights( "Weights", *this )
		, m_inCameraPoses( "CameraPose", *this )
		, m_inCameraMatrices( "Intrinsics", *this )
		, m_inInitialPose( "InitialPose", *this )
		// , m_inCenterOfGravity( "CenterOfGravity", *this )
		, m_outPort( "PoseEstimate", *this )
		, m_outPortError( "PoseEstimateError", *this )
    {
		generateSpaceExpansionPorts( config );
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp ts )
	{
		namespace ublas = boost::numeric::ublas;

		const std::vector< Math::Vector< double, 3 > >& p3d = *m_in3d.get();
		const std::vector< std::vector< Math::Vector< double, 2 > > >& p2d = *m_in2d.get();
		const std::vector< std::vector< Math::Scalar< double > > >& weights = *m_inWeights.get();
		const std::vector< Math::Pose >& camPoses = *m_inCameraPoses.get();
		const std::vector< Math::Matrix< double, 3, 3 > >& camMatrices = *m_inCameraMatrices.get();
		const Math::Pose& initialPose = *m_inInitialPose.get();

		if ( p3d.size() < 3 )
			UBITRACK_THROW( "2D3D pose estimation requires at least 3 points" );

		//###	
		if ( p2d.size() < 2 ) //###1 )
			UBITRACK_THROW( "2D3D pose estimation requires at least 1 camera" );
			
		if ( p2d.size() != weights.size() || p2d.size() != camPoses.size() || p2d.size() != camMatrices.size() )
			UBITRACK_THROW( "All ports must have the same number of cameras" );
			
		// count number of observations
		unsigned nObservations = 0;
		for ( unsigned iCam = 0; iCam < weights.size(); iCam++ )
		{
			if ( p3d.size() != weights[ iCam ].size() || p3d.size() != p2d[ iCam ].size() )
				UBITRACK_THROW( "All cameras must have same number of measurements as 3D points" );
				
			for ( unsigned iM = 0; iM < p3d.size(); iM++ )
				if ( weights[ iCam ][ iM ] != 0.0 )
					nObservations++;
		}
		
		// create measurement and observations vectors
		std::vector< std::pair< unsigned, unsigned > > observations;
		Math::Vector< double > measurements( 2 * nObservations );

		int iObs = 0;
		for ( unsigned iCam = 0; iCam < weights.size(); iCam++ )
			for ( unsigned iM = 0; iM < p3d.size(); iM++ )
				if ( weights[ iCam ][ iM ] != 0.0 )
				{
					LOG4CPP_TRACE( logger, "Observation: marker " << iM << " -> camera " << iCam << ", weight=" << weights[ iCam ][ iM ] << ", m=" << p2d[ iCam ][ iM ] );
					
					observations.push_back( std::make_pair( iM, iCam ) );
					ublas::subrange( measurements, 2 * iObs, 2 * (iObs+1) ) = p2d[ iCam ][ iM ];
					iObs++;
				}

		// create camera matrices and rotations
		std::vector< Math::Matrix< double, 3, 3 > > camRotations( camPoses.size() );
		std::vector< Math::Vector< double, 3 > > camTranslations( camPoses.size() );
		for ( unsigned iCam = 0; iCam < weights.size(); iCam++ )
		{
			camRotations[ iCam ] = Math::Matrix< double, 3, 3 >( camPoses[ iCam ].rotation() );
			camTranslations[ iCam ] = camPoses[ iCam ].translation();
		}

		// starting optimization
		LOG4CPP_DEBUG( logger, "Optimizing pose over " << p2d.size() << " cameras using " << nObservations << " observations" );
		
		ObjectiveFunction< double > f( p3d, camRotations, camTranslations, camMatrices, observations );
		Math::Vector< double, 6 > param;
		ublas::subrange( param, 0, 3 ) = initialPose.translation();
		ublas::subrange( param, 3, 6 ) = initialPose.rotation().toLogarithm();
		
		double res = Math::levenbergMarquardt( f, param, measurements, Math::OptTerminate( 10, 1e-6 ), Math::OptNoNormalize() );
		
		Math::Pose finalPose( Math::Quaternion::fromLogarithm( ublas::subrange( param, 3, 6 ) ), ublas::subrange( param, 0, 3 ) );
		LOG4CPP_DEBUG( logger, "pose: " << finalPose << ", residual: " << res );
		
		// send result
		m_outPort.send( Measurement::Pose( ts, finalPose ) );

		// compute covariance
		#if 0 // 
		{
			LOG4CPP_DEBUG( logger, "Computing covariance" );
			
			// pose in exponential map representation
			Math::Vector< double, 6 > expParam;
			ublas::subrange( expParam, 0, 3 ) = ublas::subrange( param, 0, 3 );
			ublas::subrange( expParam, 3, 6 ) = finalPose.rotation().toLogarithm();
			
			// initialize matrices
			Math::Matrix< double, 6, 6 > cov;
			
			// make 3x4 projection matrices
			std::vector< Math::Matrix< double, 3, 4 > > camPs( camPoses.size() );
			for ( unsigned i = 0; i < camPoses.size(); i++ )
				camPs[ i ] = ublas::prod( camMatrices[ i ], Math::Matrix< double, 3, 4 >( camPoses[ i ] ) );
				
			// backward propagation
			Calibration::Function::MultipleCameraProjectionErrorART< double > fe( p3d, camPs, observations ); //, *m_inCenterOfGravity.get() );
			Math::backwardPropagationIdentity( cov, 100.0, fe, expParam );
			LOG4CPP_DEBUG( logger, "covariance: " << cov );
			
			m_outPortError.send( Measurement::ErrorPose( ts, Math::ErrorPose( finalPose, cov ) ) );
		}
		#else
			m_outPortError.send( Measurement::ErrorPose( ts, Math::ErrorPose( finalPose, Math::Matrix< double, 6, 6 >() ) ) );
		#endif
		
    }

protected:
	
	/** Input port: set of 3D points in target coordinates. May be NaN if no measurement is given. */
	Dataflow::TriggerInPort< Measurement::PositionList > m_in3d;

	/** Input port: list of corresponding 2D points for each 3D point. */
	Dataflow::ExpansionInPort< std::vector< Math::Vector< double, 2 > > > m_in2d;

	/** 
	 * Input port: list of weights (inverse variance) for each corresponding 2D-3D measurement of 
	 * each camera. Can be set to 0 for no observation. 
	 */
	Dataflow::ExpansionInPort< std::vector< Math::Scalar< double > > > m_inWeights;

	/** Input port: Poses for each camera. Describes transformation from world to camera */
	Dataflow::ExpansionInPort< Math::Pose > m_inCameraPoses;

	/** Input port: Intrinsic matrices for each camera. */
	Dataflow::ExpansionInPort< Math::Matrix< double, 3, 3 > > m_inCameraMatrices;

	/** Input port: Initial pose for optimization */
	Dataflow::TriggerInPort< Measurement::Pose > m_inInitialPose;

	/** Input port: Center of gravity for error estimation */
	//Dataflow::TriggerInPort< Measurement::Position > m_inCenterOfGravity;

	/** Output port: Optimized result. */
	Dataflow::TriggerOutPort< Measurement::Pose > m_outPort;
	
	/** Output port: pose with covariance. Note: Covariance is in A.R.T. format, which is not the same as standard Ubitrack. */
	Dataflow::TriggerOutPort< Measurement::ErrorPose > m_outPortError;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< MultipleCameraPoseOptimization > ( "2D6DPoseEstimation" );
}

} } // namespace Ubitrack::Components
