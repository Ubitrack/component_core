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


#include "Trigger.h"
#include <utDataflow/ComponentFactory.h>

namespace Ubitrack { namespace Components {

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
    cf->registerComponent< Trigger< Measurement::Rotation > > ( "ButtonTriggerRotation" );
    cf->registerComponent< Trigger< Measurement::RotationVelocity > > ( "ButtonTriggerRotationVelocity" );
    cf->registerComponent< Trigger< Measurement::Position > > ( "ButtonTriggerPosition" );
    cf->registerComponent< Trigger< Measurement::Position2D > > ( "ButtonTriggerPosition2" );
    cf->registerComponent< Trigger< Measurement::Position2D > > ( "ButtonTriggerDistance" );
    cf->registerComponent< Trigger< Measurement::Pose > > ( "ButtonTriggerPose" );
    cf->registerComponent< Trigger< Measurement::PositionList > > ( "ButtonTriggerPositionList" );
    cf->registerComponent< Trigger< Measurement::PositionList2 > > ( "ButtonTriggerPositionList2" );
    cf->registerComponent< Trigger< Measurement::DistanceList > > ( "ButtonTriggerDistanceList" );
    cf->registerComponent< Trigger< Measurement::PoseList > > ( "ButtonTriggerPoseList" );
    cf->registerComponent< Trigger< Measurement::Matrix3x3 > > ( "ButtonTriggerMatrix3x3" );
    cf->registerComponent< Trigger< Measurement::Matrix3x4 > > ( "ButtonTriggerMatrix3x4" );
    cf->registerComponent< Trigger< Measurement::Matrix4x4 > > ( "ButtonTriggerMatrix4x4" );
}

} } // namespace Ubitrack::Components
