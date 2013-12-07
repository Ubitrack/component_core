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
 * Multiplication component.
 * This file contains a multiplication of two inputs implemented as a \c TriggerComponent.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Multiplication component.
 * This class contains a multiplication of two inputs implemented as a \c TriggerComponent.
 *
 * The component multiplies requested/incoming events using operator*( A, B )
 */
template< class EventTypeA, class EventTypeB, class EventTypeOut >
class MultiplicationComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	MultiplicationComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPortA( "AB", *this )
		, m_inPortB( "BC", *this )
		, m_outPort( "AC", *this )
    {
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		m_outPort.send( EventTypeOut( t, *m_inPortA.get() * *m_inPortB.get() ) );
	}

protected:
	/** Input port A of the component. */
	Dataflow::TriggerInPort< EventTypeA > m_inPortA;

	/** Input port B of the component. */
	Dataflow::TriggerInPort< EventTypeB > m_inPortB;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< EventTypeOut > m_outPort;
};


/** multiplication operator for batch multiplication of many 3D position vectors with a pose */
std::vector< Math::Vector< double, 3 > > operator*( const Math::Pose& pose, const std::vector< Math::Vector< double, 3 > >& p3d )
{
	std::vector< Math::Vector< double, 3 > > result( p3d.size() );
	for ( unsigned i = 0; i < p3d.size(); i++ )
		result[ i ] = pose * p3d[ i ];
	return result;
}

/** multiplication operator for batch multiplication of many pose vectors with a pose */
std::vector< Math::Pose > operator*( const Math::Pose& pose, const std::vector< Math::Pose >& p6d )
{
	std::vector< Math::Pose > result( p6d.size() );
	for ( unsigned i = 0; i < p6d.size(); i++ )
		result[ i ] = pose * p6d[ i ];
	return result;
}

/** multiplication operator for batch multiplication of a pose with many poses  */
std::vector< Math::Pose > operator*( const std::vector< Math::Pose >& p6d, const Math::Pose& pose )
{
	std::vector< Math::Pose > result( p6d.size() );
	for ( unsigned i = 0; i < p6d.size(); i++ )
		result[ i ] = p6d[ i ] * pose;
	return result;
}

/** multiplication operator for position "multiplication" (= addition) */
Math::Vector< double, 3 > operator*( const Math::Vector< double, 3 >& pos1, const Math::Vector< double, 3 >& pos2 )
{
	Math::Vector< double, 3 > result;
	result = pos1 + pos2;
	return result;
}

/** multiplication operator for batch multiplication of many 3D position vectors with a pose */
std::vector< Math::ErrorVector< double, 3 > > operator*( const Math::Pose& pose, const std::vector< Math::ErrorVector< double, 3 > >& p3d )
{
	std::vector< Math::ErrorVector< double, 3 > > result( p3d.size() );
	for ( unsigned i = 0; i < p3d.size(); i++ )
		result[ i ] =  Math::ErrorVector< double, 3 >(pose * p3d[ i ].value, p3d[ i ].covariance);
	return result;
}


/*
Math::ErrorVector< double, 3 > operator*( const Math::ErrorPose& a, const Math::ErrorVector< double, 3 >& b )
{
	
	// covariance transform
	Matrix< double, 3, 6 > jacobian;
	errorPoseTimesVectorJacobian( jacobian, a, b.value );

	Matrix< double, 3, 6 > tmp;
	Matrix< double, 3, 3 > newCovariance;

	noalias( tmp ) = ublas::prod( jacobian, a.covariance() );
	noalias( newCovariance ) = ublas::prod( tmp, ublas::trans( jacobian ) );

	return ErrorVector< double, 3 >( static_cast< const Pose& >( a ) * b, newCovariance );
}
*/
UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	// Pose * Pose = Pose
	cf->registerComponent< MultiplicationComponent< Measurement::Pose, Measurement::Pose, Measurement::Pose > > ( "PoseMultiplication" );

	// ErrorPose * ErrorPose = ErrorPose
	cf->registerComponent< MultiplicationComponent< Measurement::ErrorPose, Measurement::ErrorPose, Measurement::ErrorPose > > ( "ErrorPoseMultiplication" );

	// Pose * ErrorPose = ErrorPose
	cf->registerComponent< MultiplicationComponent< Measurement::Pose, Measurement::ErrorPose, Measurement::ErrorPose > > ( "PoseErrorPoseMultiplication" );

	// ErrorPose * Pose = ErrorPose
	cf->registerComponent< MultiplicationComponent< Measurement::ErrorPose, Measurement::Pose, Measurement::ErrorPose > > ( "ErrorPosePoseMultiplication" );

	// Rotation * Rotation = Rotation
	cf->registerComponent< MultiplicationComponent< Measurement::Rotation, Measurement::Rotation, Measurement::Rotation > > ( "RotationMultiplication" );

	// Pose * Position = Position
	cf->registerComponent< MultiplicationComponent< Measurement::Pose, Measurement::Position, Measurement::Position > > ( "PosePositionMultiplication" );

	// Position * Position = Position
	cf->registerComponent< MultiplicationComponent< Measurement::Position, Measurement::Position, Measurement::Position > > ( "PositionMultiplication" );

	// ErrorPose * Position = ErrorPosition
	cf->registerComponent< MultiplicationComponent< Measurement::ErrorPose, Measurement::Position, Measurement::ErrorPosition > > ( "ErrorPosePositionMultiplication" );

	// Pose * PositionList = PositionList
	cf->registerComponent< MultiplicationComponent< Measurement::Pose, Measurement::PositionList, Measurement::PositionList > > ( "PosePositionListMultiplication" );

	// Pose * PoseList = PoseList
	cf->registerComponent< MultiplicationComponent< Measurement::Pose, Measurement::PoseList, Measurement::PoseList > > ( "PosePoseListMultiplication" );

	// PoseList * Pose = PoseList
	cf->registerComponent< MultiplicationComponent< Measurement::PoseList, Measurement::Pose, Measurement::PoseList > > ( "PoseListPoseMultiplication" );

	// Rotation * RotationVelocity = RotationVelocity
	cf->registerComponent< MultiplicationComponent< Measurement::Rotation, Measurement::RotationVelocity, Measurement::RotationVelocity > > ( "RotationVelocityMultiplication" );
	
	// Rotation * RotationVelocity = RotationVelocity
	cf->registerComponent< MultiplicationComponent< Measurement::Pose, Measurement::ErrorPositionList, Measurement::ErrorPositionList > > ( "PoseErrorPositionListMultiplication" );

	cf->registerComponent< MultiplicationComponent< Measurement::ErrorPose, Measurement::ErrorPosition, Measurement::ErrorPosition > > ( "ErrorPoseErrorPositionMultiplication" );

}

} } // namespace Ubitrack::Components
