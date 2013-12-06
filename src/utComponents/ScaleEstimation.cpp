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
 * ScaleEstimation component.
 * 
 * This component estimtes the scale between two corresponding
 * translational movements. Therefore, two inputs are provided. The
 * component is implemented as a \c TriggerComponent.
 *
 * @author Peter Keitler <keitler@in.tum.de>
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * ScaleEstimator component.
 * 
 * This component estimtes the scale between two corresponding
 * relative translational movements. Therefore, two inputs are
 * provided. The component is implemented as a \c TriggerComponent.
 */
template< class EventType >
class ScaleEstimation
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	ScaleEstimation( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPortA( "RelA", *this )
		, m_inPortB( "RelB", *this )
		, m_outPort( "Scale", *this )
    {
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		m_outPort.send( Measurement::Distance( t, estimateScale ( *m_inPortA.get(), *m_inPortB.get() ) ) );
	}

protected:
	Math::Scalar< double > estimateScale( typename EventType::value_type&, typename EventType::value_type& );

	/** Input port A of the component. */
	Dataflow::TriggerInPort< EventType > m_inPortA;

	/** Input port B of the component. */
	Dataflow::TriggerInPort< EventType > m_inPortB;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Distance > m_outPort;
};



template<>
Math::Scalar< double > ScaleEstimation< Measurement::Pose >::estimateScale( Math::Pose& poseA, Math::Pose& poseB )
{
	return Math::Scalar< double >( norm_2 ( poseA.translation() ) / norm_2 ( poseB.translation() ) );
};

template<>
Math::Scalar< double > ScaleEstimation< Measurement::Position >::estimateScale( Math::Vector< double, 3 >& posA, Math::Vector< double, 3 >& posB )
{
	return Math::Scalar< double >( norm_2 ( posA ) / norm_2 ( posB ) );
};

template<>
Math::Scalar< double > ScaleEstimation< Measurement::Distance >::estimateScale( Math::Scalar< double >& distA, Math::Scalar< double >& distB )

{
	return Math::Scalar< double >( distA / distB );
};



UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< ScaleEstimation< Measurement::Pose > > ( "PoseScaleEstimation" );
	cf->registerComponent< ScaleEstimation< Measurement::Position > > ( "PositionScaleEstimation" );
	cf->registerComponent< ScaleEstimation< Measurement::Distance > > ( "DistanceScaleEstimation" );
}

} } // namespace Ubitrack::Components
