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
 * Gate component
 * This class pushes a given Event only after receiving a signal.
 *
 * @author Daniel Muhra <muhra@in.tum.de>
 */

#include "Gate.h"
#include <utDataflow/ComponentFactory.h>

namespace Ubitrack { namespace Components {

using namespace Dataflow;

UBITRACK_REGISTER_COMPONENT( ComponentFactory* const cf ) 
{
	cf->registerComponent< Gate< Measurement::Pose > > ( "PoseGate" );
	cf->registerComponent< Gate< Measurement::ErrorPose > > ( "ErrorPoseGate" );
	cf->registerComponent< Gate< Measurement::Position > > ( "PositionGate" );
	cf->registerComponent< Gate< Measurement::Position2D > > ( "Position2DGate" );
	cf->registerComponent< Gate< Measurement::Rotation > > ( "RotationGate" );
	cf->registerComponent< Gate< Measurement::Button > > ( "ButtonGate" );
	cf->registerComponent< Gate< Measurement::Distance > > ( "DistanceGate" );
	cf->registerComponent< Gate< Measurement::PoseList > > ( "PoseListGate" );
	cf->registerComponent< Gate< Measurement::PositionList > > ( "CloudGate" );
	cf->registerComponent< Gate< Measurement::PositionList2 > > ( "PositionList2Gate" );
	cf->registerComponent< Gate< Measurement::Matrix4x4 > > ( "MatrixGate" );
}

} } // namespace Ubitrack::Components
