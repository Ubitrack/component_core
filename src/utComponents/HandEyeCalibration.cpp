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
 * @ingroup ubitrack_components
 * @file
 * Computes the transformation pose between a hand and an eye component
 *
 * @author Daniel Muhra <muhra@in.tum.de>
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMath/Vector.h>

#include <utCalibration/HandEyeCalibration.h>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.HandEyeCalibration" ) );
static log4cpp::Category& eventLogger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.HandEyeCalibration" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup ubitrack_components
 * hand-eye calibration component.
 * This class contains a hand-eye calibration implemented as a \c TriggerComponent.
 *
 * @par Input Ports
 * ExpansionInPort< Math::Pose > of name "HandPose", hand pose in the robot coordinate system
 * ExpansionInPort< Math::Pose > of name "ObjectPose", eye pose in the eye coordinate system
 *
 * @par Output Ports
 * TriggerOutPort<Measurement::Pose>> with name "Output".
 * 
 * @par Operation
 * The component returns the transformation as an Measurement::Pose
 *
 */

class HandEyeCalibrationComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * Standard component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	HandEyeCalibrationComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
		: Dataflow::TriggerComponent( sName, pCfg )
		, m_handPort( "HandPose", *this )
		, m_objectPort( "ObjectPose", *this )
		, m_transfPort( "Output", *this )
    {
    }

	/** Method that computes the result. */	
	void compute( Measurement::Timestamp t )
	{
		if ( m_handPort.get()->size() != m_objectPort.get()->size() || m_handPort.get()->size() < 2 )
			UBITRACK_THROW( "Illegal number of correspondences" );

		boost::shared_ptr< Math::Pose > p( new Math::Pose( Calibration::performHandEyeCalibration( *m_handPort.get(), *m_objectPort.get() ) ) );
		
		m_transfPort.send( Measurement::Pose ( t, p ) );
	}

protected:
	Math::Pose m_transformation;
	/** Input port of the component. */
	Dataflow::ExpansionInPort< Math::Pose > m_handPort;
	Dataflow::ExpansionInPort< Math::Pose > m_objectPort;
	/** Output ports of the component. */
	Dataflow::TriggerOutPort< Measurement::Pose > m_transfPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< HandEyeCalibrationComponent > ( "HECalibration" );
}

} } // namespace Ubitrack::Components

