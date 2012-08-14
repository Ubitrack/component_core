
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
 * @author Adnane Jadid <jadid@in.tum.de>
 */

#include <utDataflow/ComponentFactory.h>
#include "Sampler.h"

// get a logger
//static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.Sampler" ) );
//static log4cpp::Category& eventLogger( log4cpp::Category::getInstance( "Ubitrack.Events.Components.Sampler" ) );

namespace Ubitrack { namespace Components {

UBITRACK_REGISTER_COMPONENT( Ubitrack::Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Components::Sampler< Ubitrack::Measurement::Pose > > ( "PoseSampler" );
	cf->registerComponent< Ubitrack::Components::Sampler< Ubitrack::Measurement::ErrorPose > > ( "ErrorPoseSampler" );
	cf->registerComponent< Ubitrack::Components::Sampler< Ubitrack::Measurement::ErrorPosition > > ( "ErrorPositionSampler" );
	cf->registerComponent< Ubitrack::Components::Sampler< Ubitrack::Measurement::Rotation > > ( "RotationSampler" );
	cf->registerComponent< Ubitrack::Components::Sampler< Ubitrack::Measurement::Position > > ( "PositionSampler" );
	cf->registerComponent< Ubitrack::Components::Sampler< Ubitrack::Measurement::Matrix3x3 > > ( "Matrix3x3Sampler" );
	cf->registerComponent< Ubitrack::Components::Sampler< Ubitrack::Measurement::Matrix3x4 > > ( "Matrix3x4Sampler" );
	cf->registerComponent< Ubitrack::Components::Sampler< Ubitrack::Measurement::Button > > ( "ButtonSampler" );
	cf->registerComponent< Ubitrack::Components::Sampler< Ubitrack::Measurement::PositionList2 > > ( "PositionList2Sampler" );
	cf->registerComponent< Ubitrack::Components::Sampler< Ubitrack::Measurement::PositionList > > ( "PositionListSampler" );
	cf->registerComponent< Ubitrack::Components::Sampler< Ubitrack::Measurement::PoseList > > ( "PoseListSampler" );
	cf->registerComponent< Ubitrack::Components::Sampler< Ubitrack::Measurement::Vector4D > > ( "Vector4DSampler" );
	
}


} } // namespace Ubitrack::Components


