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
 * @ingroup vision_components
 * @file
 * Implements a component that unwarps images.
 *
 * @author Christian Waechter <christian.waechter@in.tum.de>
 */

// Ubitrack Math
#include <utMath/Vector.h>
#include <utMath/CameraIntrinsics.h>
#include <utMath/Geometry/PointUndistortion.h>

// Ubitrack Dataflow
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>

// std
#include <string>

// log4cpp
#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.Undistortion" ) );

using namespace Ubitrack;
using namespace Ubitrack::Dataflow;

namespace Ubitrack { namespace Components {


class Undistortion
	: public TriggerComponent
{
	typedef double value_type;
	
protected:
	
	/** intrinsic camera parameters: 3x3matrix + distortion parameters */
	PullConsumer< Measurement::CameraIntrinsics > m_intrinsicsPort;
	
	/** distorted points */
	TriggerInPort< Measurement::PositionList2 > m_pointsIn;
	
	/** undistorted points */
	TriggerOutPort<  Measurement::PositionList2 > m_pointsOut;

	
public:
	Undistortion( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: TriggerComponent( sName, pConfig )
		, m_intrinsicsPort( "CameraIntrinsics", *this ) //new intrinsics port
		, m_pointsIn( "Input", *this )
		, m_pointsOut( "Output", *this )
	{
	}

		
	void compute( Measurement::Timestamp t )
	{
		
		if( !m_intrinsicsPort.isConnected() )
			UBITRACK_THROW( "Cannot start undistortion of point(s), no camera intrinsic parameters are available." );
			
		//support for new camera intrinsics measurement
		Math::CameraIntrinsics< value_type > camIntrinsics = *m_intrinsicsPort.get( t );
		
		try
		{
			const std::vector< Math::Vector< value_type, 2 > > &vectorIn = *m_pointsIn.get( );
			std::vector< Math::Vector< value_type, 2 > > vectorOut;
			vectorOut.reserve( vectorIn.size() );
			
			Math::Geometry::undistort_points( camIntrinsics, vectorIn.begin(), vectorIn.end(), std::back_inserter( vectorOut ) );
			m_pointsOut.send( Measurement::PositionList2( t, vectorOut ) );
		}
		catch( const std::exception &e )
		{
			LOG4CPP_WARN( logger, "Could not undistort point(s), no point(s) available.\n" << e.what() );
			return;
		}
	}
};

} } // namespace Ubitrack::Components


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) 
{
	cf->registerComponent< Ubitrack::Components::Undistortion > ( "UndistortPositionList2D" );
}
