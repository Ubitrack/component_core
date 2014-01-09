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
 * Perturbation component.
 * This file contains perturbation functionality for common measurement types.
 * It is implemented as a \c TriggerComponent.
 *
 * @author Peter Keitler <keitler@in.tum.de>
 */
#include <string>

#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

#include <log4cpp/Category.hh>
#include <boost/scoped_ptr.hpp>
#include <boost/random.hpp>
#include <boost/math/constants/constants.hpp>

using namespace Ubitrack;

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.Perturbation" ) );
static log4cpp::Category& eventLogger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.Perturbation" ) );

namespace Ubitrack { namespace Components {


enum Distribution { GAUSSIAN, UNIFORM };


template< class EventType >
class PerturbationComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
    PerturbationComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Dataflow::TriggerComponent( sName, subgraph )
		, m_inPort( "AB", *this )
		, m_outPort( "AB-Perturbed", *this )
		, enableNormalize( false )
		, dist ( GAUSSIAN )
	{
		double posStdDev( 0.01 );
		subgraph->m_DataflowAttributes.getAttributeData( "posStdDev", posStdDev );
		double rotStdDev( 0.0 );
		subgraph->m_DataflowAttributes.getAttributeData( "rotStdDev", rotStdDev );
		std::string normalize( "false" );
		subgraph->m_DataflowAttributes.getAttributeData( "enableNormalize", normalize );
		if ( normalize.compare( "true" ) == 0 )
			enableNormalize = true;
		std::string distribution( "gaussian" );
		subgraph->m_DataflowAttributes.getAttributeData( "distribution", distribution );
		if ( distribution.compare ( "uniform" ) == 0 )
			dist = UNIFORM;

		LOG4CPP_DEBUG( logger, "Setup perturbation component " << sName << ". pos. std. dev: " << posStdDev << ", rot. std. dev: " << rotStdDev << ", normalization: " << enableNormalize << ", distribution type: " << distribution );

		/* Simple random number generator. */
		rng.reset ( new boost::mt19937( (unsigned int )Measurement::now() ) );
		
		/* Position error */
		distPosNorm.reset ( new boost::normal_distribution<double>( 0.0, posStdDev ) );
		distSamplerPosNorm.reset ( new boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> >( *rng, *distPosNorm ) );
		distPosUni.reset ( new boost::uniform_real<double>( -posStdDev * sqrt(double(3.0) ), posStdDev * sqrt(double(3.0) ) ) );
		distSamplerPosUni.reset ( new boost::variate_generator<boost::mt19937&, boost::uniform_real<double> >( *rng, *distPosUni ) );

		/* Direction error */
		distDir.reset( new boost::uniform_on_sphere<double>( 3 ) );
		distSamplerDir.reset ( new boost::variate_generator<boost::mt19937&, boost::uniform_on_sphere<double> >( *rng, *distDir ) );

		/* Angle error */
		distRotNorm.reset ( new boost::normal_distribution<double>( 0.0, rotStdDev * (boost::math::constants::pi<double>() / 180.0) ) );
		distSamplerRotNorm.reset ( new boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> >( *rng, *distRotNorm ) );
		distRotUni.reset ( new boost::uniform_real<double>( -rotStdDev * sqrt(double(3.0)) * (boost::math::constants::pi<double>() / 180.0), rotStdDev * sqrt(double(3.0)) * (boost::math::constants::pi<double>() / 180.0) ) );
		distSamplerRotUni.reset ( new boost::variate_generator<boost::mt19937&, boost::uniform_real<double> >( *rng, *distRotUni ) );
	}

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		EventType event( t, perturb ( *(m_inPort.get()) ) );
	    m_outPort.send( event );
	}	

protected:
	Math::Vector< double, 2 > perturbPosition2D( Math::Vector< double, 2 > pos ) 
	{
		if ( dist == GAUSSIAN )
			return pos + Math::Vector< double, 2 >( (*distSamplerPosNorm)(), (*distSamplerPosNorm)() );
		else
			return pos + Math::Vector< double, 2 >( (*distSamplerPosUni)() / sqrt((double)2.0), (*distSamplerPosUni)() / sqrt((double)2.0) );
	}

	Math::Vector< double, 3 > perturbPosition( Math::Vector< double, 3 > pos ) 
	{
		if ( dist == GAUSSIAN )
			return pos + Math::Vector< double, 3 >( (*distSamplerPosNorm)(), (*distSamplerPosNorm)(), (*distSamplerPosNorm)() );
		else
			return pos + Math::Vector< double, 3 >( (*distSamplerPosUni)() / sqrt((double)3.0), (*distSamplerPosUni)() / sqrt((double)3.0), (*distSamplerPosUni)() / sqrt(double(3.0)) );
	}
	
	Math::Quaternion perturbOrientation( Math::Quaternion rot ) 
	{
		/*
		 * Construct arbitrary rotation axis using spherical
		 * coordinate system.  The returned axis is already normalized
		 * (!)
		 */
		std::vector<double> stdAxis = (*distSamplerDir)();
		Math::Vector< double, 3 > axis ( stdAxis[0], stdAxis[1], stdAxis[2] );
		double angle;
		if ( dist == GAUSSIAN )
			angle = (*distSamplerRotNorm)();
		else
			angle = (*distSamplerRotUni)();
		axis *= sin( angle / 2 );
		
		/*
		 * Construct either a regular or a small quaternion
		 * representing the perturbation by encoding angular noise in
		 * axis computed above and setting the quaternion real part
		 * accordingly.
		 *
		 * Note that for small rotation angles, the sine (quaternion
		 * imaginary part) changes rapidly whereas the cosine
		 * (quaternion real part) remains one.
		 */
		Math::Quaternion errQuat;
		if ( enableNormalize )
			errQuat = Math::Quaternion( axis(0), axis(1), axis(2), sqrt( (double)1.0 - norm_2(axis) ) );
		else
			errQuat = Math::Quaternion( axis(0), axis(1), axis(2), 1.0 );

		// This corresponds to applying the error first, applying the
		// actual rotation afterwards. See also ErrorPose.h
		rot *= errQuat;
		// To be on the safe side, remove numeric deviations
		if ( enableNormalize )
			rot.normalize();
		
		return rot;
	}

	typename EventType::value_type perturb( const typename EventType::value_type& ref );
	
	/** Input port of the component. */
	Dataflow::TriggerInPort< EventType > m_inPort;
	/** Output port of the component. */
	Dataflow::TriggerOutPort< EventType > m_outPort;
	/** Shall the perturbed orientation quaternion be normalized? */
	bool enableNormalize;
	/** Determines which distribution type to use for random sampling */
	Distribution dist;

	/** Default random number generator used for all variate generators. */
	boost::scoped_ptr< boost::mt19937 > rng;

	/** Position error distributions. */
	//@{
	boost::scoped_ptr< boost::normal_distribution<double> > distPosNorm;
	boost::scoped_ptr< boost::uniform_real<double> > distPosUni;
	//@}
	
	/** Position variate generators */
	//@{
	boost::scoped_ptr< boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > > distSamplerPosNorm;
	boost::scoped_ptr< boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > > distSamplerPosUni;
	//@}

	/** Angle error distributions. */
	//{@
	boost::scoped_ptr< boost::normal_distribution<double> > distRotNorm;
	boost::scoped_ptr< boost::uniform_real<double> > distRotUni;
	//@}

	/** Angle variate generators */
	//@{
	boost::scoped_ptr< boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > > distSamplerRotNorm;
	boost::scoped_ptr< boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > > distSamplerRotUni;
	//@}

	/** Direction uniform error distribution. */
	boost::scoped_ptr< boost::uniform_on_sphere<double> > distDir;
	/** Direction random number generator */
	boost::scoped_ptr< boost::variate_generator<boost::mt19937&, boost::uniform_on_sphere<double> > > distSamplerDir;
};


template<>
Math::Vector< double, 2 > PerturbationComponent< Measurement::Position2D >::perturb(
	const Math::Vector< double, 2 >& ref )
{
	return perturbPosition ( ref );
}


template<>
Math::Vector< double, 3 > PerturbationComponent< Measurement::Position >::perturb(
	const Math::Vector< double, 3 >& ref )
{
	return perturbPosition ( ref );
}


template<>
std::vector< Math::Vector< double, 2 > > PerturbationComponent< Measurement::PositionList2 >::perturb( const std::vector< Math::Vector< double, 2 > >& refList )
{
	std::vector< Math::Vector< double, 2 > > result( refList.size() );
	for ( unsigned i = 0; i < refList.size(); i++ ) 
	{
		result[ i ] = perturbPosition2D ( refList[ i ] );
		LOG4CPP_TRACE( logger, "Perturbed 2D list point: " << result[i] );
	}
	
	return result;
}


template<>
std::vector< Math::Vector< double, 3 > > PerturbationComponent< Measurement::PositionList >::perturb( const std::vector< Math::Vector< double, 3 > >& refList )
{
	std::vector< Math::Vector< double, 3 > > result( refList.size() );
	for ( unsigned i = 0; i < refList.size(); i++ ) 
	{
		result[ i ] = perturbPosition ( refList[ i ] );
		LOG4CPP_TRACE( logger, "Perturbed 3D list point: " << result[i] );
	}
	
	return result;
}


template<>
std::vector< Math::Pose > PerturbationComponent< Measurement::PoseList >::perturb( const std::vector< Math::Pose >& refList )
{
	std::vector< Math::Pose > result( refList.size() );
	for ( unsigned i = 0; i < refList.size(); i++ ) 
	{
		result[ i ] = Math::Pose( perturbOrientation ( refList[ i ].rotation() ), perturbPosition ( refList[ i ].translation() ) );
		LOG4CPP_TRACE( logger, "Perturbed list pose: " << result[i] );
	}
	
	return result;
}


template<>
Math::Quaternion PerturbationComponent< Measurement::Rotation >::perturb(
	const Math::Quaternion& ref )
{
	return perturbOrientation ( ref );
}


template<>
Math::Pose PerturbationComponent< Measurement::Pose >::perturb(
	const Math::Pose& ref )
{
	return Math::Pose( perturbOrientation ( ref.rotation() ), perturbPosition ( ref.translation() )  );
}


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< PerturbationComponent< Measurement::Position2D    > > ( "2DPositionPerturbation" );
	cf->registerComponent< PerturbationComponent< Measurement::Position      > > ( "3DPositionPerturbation" );
	cf->registerComponent< PerturbationComponent< Measurement::PositionList2 > > ( "2DPositionListPerturbation" );
	cf->registerComponent< PerturbationComponent< Measurement::PositionList  > > ( "3DPositionListPerturbation" );
	cf->registerComponent< PerturbationComponent< Measurement::Rotation      > > ( "RotationPerturbation" );
	cf->registerComponent< PerturbationComponent< Measurement::Pose          > > ( "PosePerturbation" );
	cf->registerComponent< PerturbationComponent< Measurement::PoseList      > > ( "PoseListPerturbation" );
}

} } // namespace Ubitrack::Components
