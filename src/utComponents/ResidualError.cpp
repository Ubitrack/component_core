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
 * Residual error component.
 * This file contains a component to compute the residual error between two lists of points.
 *
 * @author Peter Keitler <keitler@in.tum.de>
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Residual error component.
 * This file contains a component to compute the residual error between two lists of points.
 *
 * @par Input Ports
 * ExpansionInPort<Position> with name "InputA".
 * ExpansionInPort<Position> with name "InputB".
 *
 * @par Output Ports
 * TriggerOutPort<Distance> with name "Residual".
 *
 * @par Configuration
 * DataflowConfiguration: expansion="space" or "time" for time/space expansion
 *
 * @par Operation
 * The component computes the residual error between two lists of points having equal length.
 * The computed value is the sum of the sqaured euclidean distances of the point correspondences, normalized (divided) by the amount of correspondences.
 */
 template< class EventType >
class ResidualErrorComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	ResidualErrorComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPortA( "InputA", *this )
		, m_inPortB( "InputB", *this )
		, m_outPort( "Residual", *this )
    {
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		if ( m_inPortA.get()->size() != m_inPortB.get()->size() || m_inPortA.get()->size() < 1 )
			UBITRACK_THROW( "Illegal number of correspondences" );
		
		// Get measurement payload (*) for both lists
		std::vector< EventType >& left = *m_inPortA.get();
		std::vector< EventType >& right = *m_inPortB.get();

		// Sum squared euclidean distances over all point correspondences
		double residual = 0;
		for ( unsigned int i = 0; i < left.size(); i++ ) {
		    EventType l = left[i];
		    EventType r = right[i];
		    
		    residual += boost::numeric::ublas::norm_2( l - r );
		}

 		boost::shared_ptr< Math::Scalar<double> > res( new Math::Scalar<double>( residual ) );

		m_outPort.send( Measurement::Distance( t, res ) );
    }

protected:
	/** Input port A of the component. */
	Dataflow::ExpansionInPort< EventType > m_inPortA;

	/** Input port B of the component. */
	Dataflow::ExpansionInPort< EventType > m_inPortB;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Distance > m_outPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< ResidualErrorComponent< Math::Vector< double, 3 > > > ( "ResidualError" );//kept due to conformity
	cf->registerComponent< ResidualErrorComponent< Math::Vector< double, 3 > > > ( "Position3DResidualError" );
	cf->registerComponent< ResidualErrorComponent< Math::Vector< double, 2 > > > ( "Position2DResidualError" );
	

}

} } // namespace Ubitrack::Components
