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
 * 3x3HomogeneousMatrixEstimation component.
 * This file contains a component to compute a 3x3 HomogeneousMatrix Estimation 
 *
 * @author Adnane Jadid <jadid@in.tum.de>
 */

#include <log4cpp/Category.hh>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utCalibration/Homography.h>

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.HomogeneousMatrixEstimation" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * 3x3HomogeneousMatrixEstimation component.
 * This file contains a component to compute a 2D homography H in terms of a homogeneous 3x3 matrix.
 *
 * @par Input Ports
 * ExpansionInPort<Position2D> with name "InputA".
 * ExpansionInPort<Position2D> with name "InputB".
 *
 * @par Output Ports
 * TriggerOutPort<Matrix3x3> with name "Output".
 *
 * @par Configuration
 * DataflowConfiguration: "expansion" = "time" or "space"
 * None.
 *
 * @par Operation
 * The component computes 3x3 Homogeneous Matrix Estimation from 2D to 2D,
 * given corresponding points in Input2D and 1_Input2D. For details see 
 * \c Ubitrack::Calibration::projectionDLT.
 *
 * @par Instances
 * Registered for the following expansions and push/pull configurations:
 * <table>
 * <tr><th> Name </th><th> Expansion </th><th> Push/Pull </th></tr>
 * <tr><td> \c 3x3HomogeneousMatrixEstimationPushPullPull </td><td> Time </td><td> Push+Pull=Pull </td></tr>
 * <tr><td> \c 3x3HomogeneousMatrixEstimationPullPushPull </td><td> Time </td><td> Pull+Push=Pull </td></tr>
 * <tr><td> \c 3x3HomogeneousMatrixEstimationPushPullPush </td><td> Time </td><td> Push+Pull=Push </td></tr>
 * <tr><td> \c 3x3HomogeneousMatrixEstimationPullPushPush </td><td> Time </td><td> Pull+Push=Push </td></tr>
 * </table>
 */

class HomogeneousMatrixEstimation 
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * Standard component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	HomogeneousMatrixEstimation( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPortA( "InputA", *this )
		, m_inPortB( "InputB",*this)
		, m_outPort( "Output", *this )
    {
		generateSpaceExpansionPorts( pConfig );
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		LOG4CPP_DEBUG( logger, "3x3HomogeneousMatrixEstimation using " << m_inPortB.get()->size() << " points" );

		if ( m_inPortA.get()->size() != m_inPortB.get()->size() || m_inPortA.get()->size() < 4  )
			UBITRACK_THROW( "Illegal number of correspondences" );
		
		Math::Matrix< double, 3, 3 > mat = Calibration::homographyDLT( *m_inPortB.get(),*m_inPortA.get() );
	
		m_outPort.send( Measurement::Matrix3x3( t, mat ) );
	}

protected:

	/** 2D Input port of the component. */
	Dataflow::ExpansionInPort< Math::Vector< double, 2 > > m_inPortA;

	/** 3D Input port of the component. */
	Dataflow::ExpansionInPort< Math::Vector< double, 2 > > m_inPortB;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Matrix3x3 > m_outPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< HomogeneousMatrixEstimation >( "3x3HomogeneousMatrixEstimation" );
}
 
} } // namespace Ubitrack::Components

