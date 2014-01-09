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
 * 2D-3D pose estimation component.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */
#include <log4cpp/Category.hh>

#include <utMath/MatrixOperations.h>
#include <utCalibration/2D3DPoseEstimation.h>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utCalibration/2D3DPoseEstimation.h>

#include <boost/lexical_cast.hpp>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.2D3DPoseEstimation" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * 2D-3D pose estimation component.
 * Given pairs of corresponding 2D and 3D points, the component computes the pose.
 *
 * @par Operation
 * The component computes the 6D pose from corresponding 2d and 3d points. It first computes
 * a rough initial pose which is refined by nonlinear optimization.
 *
 * Currently, the component has one drawback: for the first step (the initialization), the 
 * component assumes that the first 4 3D points lie on a plane, which in general is NOT the 
 * case! It is however for square markers...
 */
class PoseEstimation2D3D
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	PoseEstimation2D3D( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > config )
		: Dataflow::TriggerComponent( sName, config )
		, m_in2d( "Input2d", *this )
		, m_in3d( "Input3d", *this )
		, m_inCam( "Intrinsics", *this )
		, m_errOutPort( "Output", *this )
		, m_iMinCorrespondences( 4 )
		, m_method( (enum Calibration::InitializationMethod)Calibration::PLANAR_HOMOGRAPHY )
    {
		config->m_DataflowAttributes.getAttributeData( "initPoseMethod", (unsigned int &)m_method );
		config->m_DataflowAttributes.getAttributeData( "min2d3dCorresp", m_iMinCorrespondences );
		
		if ( m_iMinCorrespondences < 4 ) {
			LOG4CPP_ERROR( logger, "2D3D pose estimation cannot be performed with less than 4 points. Falling back to a minimum of 4 points." );
			m_iMinCorrespondences = 4;
		}
		
		generateSpaceExpansionPorts( config );
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp ts )
	{
		const std::vector< Math::Vector< double, 2 > >& p2d( *m_in2d.get() );
		const std::vector< Math::Vector< double, 3 > >& p3d( *m_in3d.get() );
		const Math::Matrix< double, 3, 3 > cam = *m_inCam.get();

		if ( p2d.size() < 4 ) {
			UBITRACK_THROW( "2D3D pose estimation configured to use at least " + boost::lexical_cast<std::string>( m_iMinCorrespondences ) + " points" );
		}
		
		Math::ErrorPose errPose = Calibration::computePose( p2d, p3d, cam, m_method );
		
		m_errOutPort.send( Measurement::ErrorPose( ts, errPose ) );		
    }

protected:
	/** Input port Input2d of the component. */
	Dataflow::ExpansionInPort< Math::Vector< double, 2 > > m_in2d;

	/** Input port Input3d of the component. */
	Dataflow::ExpansionInPort< Math::Vector< double, 3 > > m_in3d;

	/** Input port Intrinsics of the component. */
	Dataflow::TriggerInPort< Measurement::Matrix3x3 > m_inCam;

	/** Optional error output port of the component. */
	Dataflow::TriggerOutPort< Measurement::ErrorPose > m_errOutPort;

	/** Minimum number of correspondences */
	unsigned int m_iMinCorrespondences;

	/** Method used for computation of initial pose */
	enum Calibration::InitializationMethod m_method;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< PoseEstimation2D3D > ( "2D3DPoseEstimation" );
}

} } // namespace Ubitrack::Components
