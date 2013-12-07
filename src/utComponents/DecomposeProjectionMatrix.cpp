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
 * Components that splits a Projection Matrix in intrinsic and extrinsic components
 *
 * @author Adnane Jadid <jadid@in.tum.de>
 */
#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utCalibration/Projection.h>
#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.DecomposeProjectionMatrix" ) );

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Components that splits a Matrix into intrinsic and pose components
 *
 * @par Input Ports
 * PushConsumer< ProjectionMatrix > with name "Input".
 *
 * @par Output Ports
 * PushSupplier<intrinsic > with name "intrinsic".
 *PushSupplier<extrinsic > with name "pose".
 *
 * @par Configuration
 * None.
 *
 * @par Operation
 *
 * @par Instances
 */
class DecomposeProjectionMatrix
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	DecomposeProjectionMatrix( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  )
		: Dataflow::Component( sName )
		, m_inPort( "Input", *this, boost::bind( &DecomposeProjectionMatrix::receiveProjectionMatrix, this, _1 ) )
		, m_outIntrinsic( "Intrinsic", *this )
		, m_outExtrinsic( "Extrinsic", *this )
    {
		LOG4CPP_ERROR( logger, "This Component is deprecated. Please ask Adnane Jadid" );
    }

	/** Method that computes the result. */
	void receiveProjectionMatrix( const Measurement::Matrix3x4 p )
	{
			Math::Matrix< double, 3, 3 > K;
			Math::Matrix< double, 3, 3 > R;
			Math::Vector< double, 3 > t;
			
			 	
			Calibration::decomposeProjection( K, R, t,*p);
			Math::Quaternion rq=Math::Quaternion(R);
			Math::Pose aPose= Math::Pose(rq,t);
		m_outIntrinsic.send(  Measurement::Matrix3x3(p.time(),K));
		m_outExtrinsic.send(  Measurement::Pose(p.time(), aPose ) );
    }

protected:
	/** Input port of the component. */
	Dataflow::PushConsumer< Measurement::Matrix3x4 > m_inPort;

	// Output ports of the component
	Dataflow::PushSupplier<Measurement::Matrix3x3 > m_outIntrinsic;
	Dataflow::PushSupplier<Measurement::Pose > m_outExtrinsic;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< DecomposeProjectionMatrix > ( "DecomposeProjectionMatrix" );
}

} } // namespace Ubitrack::Components
