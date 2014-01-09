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
 * SPAAM component.
 * This file contains a component to compute a SPAAM calibration matrix.
 *
 * @author Florian Echtler <echtler@in.tum.de>
 */

#include <log4cpp/Category.hh>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utCalibration/Projection.h>

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.SPAAM" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * SPAAM component.
 * This file contains a component to compute a SPAAM calibration matrix.
 *
 * @par Input Ports
 * ExpansionInPort<Position2D> with name "Input2D".
 * ExpansionInPort<Position> with name "Input3D".
 *
 * @par Output Ports
 * TriggerOutPort<Matrix3x4> with name "Output".
 *
 * @par Configuration
 * DataflowConfiguration: "expansion" = "time" or "space"
 * None.
 *
 * @par Operation
 * The component computes the projection matrix from 3D to 2D,
 * given corresponding points in Input2D and Input3D. For details see 
 * \c Ubitrack::Calibration::projectionDLT.
 *
 * @par Instances
 * Registered for the following expansions and push/pull configurations:
 * <table>
 * <tr><th> Name </th><th> Expansion </th><th> Push/Pull </th></tr>
 * <tr><td> \c SpaamPushPullPull </td><td> Time </td><td> Push+Pull=Pull </td></tr>
 * <tr><td> \c SpaamPullPushPull </td><td> Time </td><td> Pull+Push=Pull </td></tr>
 * <tr><td> \c SpaamPushPullPush </td><td> Time </td><td> Push+Pull=Push </td></tr>
 * <tr><td> \c SpaamPullPushPush </td><td> Time </td><td> Pull+Push=Push </td></tr>
 * </table>
 */

class SPAAM 
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * Standard component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	SPAAM( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPort2D( "Input2D", *this )
		, m_inPort3D( "Input3D", *this )
		, m_outPort( "Output", *this )
    {
		generateSpaceExpansionPorts( pConfig );
	}

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		LOG4CPP_DEBUG( logger, "SPAAM using " << m_inPort3D.get()->size() << " points" );

		if ( m_inPort2D.get()->size() != m_inPort3D.get()->size() || m_inPort2D.get()->size() < 6 )
			UBITRACK_THROW( "Illegal number of correspondences" );

		Math::Matrix< double, 3, 4 > mat = Calibration::projectionDLT( *m_inPort3D.get(), *m_inPort2D.get() );

		// print decomposed matrix to console if logging is enabled
		if ( logger.isDebugEnabled() )
		{
			Math::Matrix< double, 3, 3 > K;
			Math::Matrix< double, 3, 3 > R;
			Math::Vector< double, 3 > t;
			Calibration::decomposeProjection( K, R, t, mat );
			LOG4CPP_DEBUG( logger, "K: " << K );
			LOG4CPP_DEBUG( logger, "R: " << R );
			LOG4CPP_DEBUG( logger, "t: " << t );
		}

		m_outPort.send( Measurement::Matrix3x4( t, mat ) );
	}

protected:

	/** 2D Input port of the component. */
	Dataflow::ExpansionInPort< Math::Vector< double, 2 > > m_inPort2D;

	/** 3D Input port of the component. */
	Dataflow::ExpansionInPort< Math::Vector< double, 3 > > m_inPort3D;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Matrix3x4 > m_outPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< SPAAM >( "Spaam" );
}
 
} } // namespace Ubitrack::Components

