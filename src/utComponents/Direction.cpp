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
 * Direction component.
 * This file contains a direction calculation between two poses implemented as a \c TriggerComponent.
 *
 * @author Florian Echtler <echtler@in.tum.de>
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

namespace ublas = boost::numeric::ublas;

namespace Ubitrack { namespace Components {


/** Method to calculate rotation from unit vector to other pose */
template< class Type > Math::Quaternion calculate( Math::Vector< double, 3 >& orig, Math::Pose& pose1, Type& t2 )
{
	return Math::Quaternion();
}

template<> Math::Quaternion calculate< Math::Vector< double, 3 > >( Math::Vector< double, 3 >& orig, Math::Pose& pose1, Math::Vector< double, 3 >& pos2 )
{
	Math::Vector< double, 3 > dir = (~pose1) * pos2;
	dir = dir / ublas::norm_2( dir );

	Math::Vector< double, 3 > axis = cross_prod( orig, dir );
	
	Math::Quaternion q( axis[0], axis[1], axis[2], 1 + ublas::inner_prod( orig, dir ) );
	q.normalize();

	return pose1.rotation() * q;
}

template<> Math::Quaternion calculate< Math::Pose >( Math::Vector< double, 3 >& orig, Math::Pose& pose1, Math::Pose& pose2 )
{
	Math::Vector< double, 3 > pos2 = pose2.translation();
	return calculate( orig, pose1, pos2 );
}


template < class EventType > 
class DirectionComponent
	: public Dataflow::TriggerComponent
{
	public:
		/**
		 * UTQL component constructor.
		 *
		 * @param sName Unique name of the component.
		 * @param subgraph UTQL subgraph
		 */
		DirectionComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > cfg )
			: Dataflow::TriggerComponent( sName, cfg )
			, m_inPortA( "AB", *this )
			, m_inPortB( "BC", *this )
			, m_tipInPort ( "AD", *this )
			, m_outPort( "AE", *this )
		{
		}

		/** Method that computes the result. */
		void compute( Measurement::Timestamp t )
		{
			m_outPort.send( Measurement::Rotation( t, calculate( *m_tipInPort.get(t), *m_inPortA.get(), *m_inPortB.get() ) ) );
		}

	protected:

		/** Input port A of the component. */
		Dataflow::TriggerInPort< Measurement::Pose > m_inPortA;

		/** Input port B of the component. */
		Dataflow::TriggerInPort< EventType > m_inPortB;

	    /** Input port of the component. */
	    Dataflow::PullConsumer< Measurement::Position > m_tipInPort;

	    /** Output port of the component. */
		Dataflow::TriggerOutPort< Measurement::Rotation > m_outPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {

	cf->registerComponent< DirectionComponent< Measurement::Pose > > ( "PoseDirection" );
	cf->registerComponent< DirectionComponent< Measurement::Position > > ( "PositionDirection" );

}

} } // namespace Ubitrack::Components

