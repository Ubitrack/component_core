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
 * Buffer component
 * This file contains a buffer component which is the
 * most simple push-pull adapter.
 * The component accepts an event via a push input port
 * and sends the last received event for any request via
 * the pull output port.
 *
 * This may be useful for static spatial relationships which are
 * can be calibrated at runtime.
 *
 * @author Manuel Huber <huberma@in.tum.de>
 */

#include <utDataflow/ComponentFactory.h>
#include "Buffer.h"

namespace Ubitrack { namespace Components {

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {

	cf->registerComponent< DistanceBuffer > ( "DistanceBuffer" );	
	cf->registerComponent< SkalarBuffer > ( "SkalarBuffer" );	

	cf->registerComponent< Position2Buffer > ( "Position2Buffer" );
	cf->registerComponent< PositionBuffer > ( "PositionBuffer" );
	cf->registerComponent< PoseBuffer > ( "PoseBuffer" );

	cf->registerComponent< ErrorPosition2Buffer > ( "ErrorPosition2Buffer" );	
	cf->registerComponent< ErrorPositionBuffer > ( "ErrorPositionBuffer" );	
	cf->registerComponent< ErrorPoseBuffer > ( "ErrorPoseBuffer" );	

	cf->registerComponent< RotationBuffer > ( "RotationBuffer" );

	cf->registerComponent< Matrix3x3Buffer > ( "Matrix3x3Buffer" );	
	cf->registerComponent< Matrix3x4Buffer > ( "Matrix3x4Buffer" );	
	cf->registerComponent< Matrix4x4Buffer > ( "Matrix4x4Buffer" );	

	cf->registerComponent< Vector4DBuffer > ( "Vector4DBuffer" );

	cf->registerComponent< PositionList2Buffer > ( "PositionList2Buffer" );
	cf->registerComponent< PositionListBuffer > ( "PositionListBuffer" );
	cf->registerComponent< PositionListBuffer > ( "PoseListBuffer" );

	cf->registerComponent< CameraIntrinsicsBuffer > ( "CameraIntrinsicsBuffer" );

	cf->registerComponent< RotationVelocityBuffer > ( "RotationVelocityBuffer" );
	
}


} } // namespace Ubitrack::Components

