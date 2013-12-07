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
 * Adds a static error to a measurement.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

namespace Ubitrack { namespace Components {

namespace {

/** reads the covariance from attributes */
Math::Matrix< double, 6, 6 > readConfiguration( boost::shared_ptr< Graph::UTQLSubgraph > subgraph
	, const Measurement::ErrorPose& )
{
	double posSigma = 1e-3;
	double rotSigma = 1e-3;
	subgraph->m_DataflowAttributes.getAttributeData( "posSigma", posSigma );
	subgraph->m_DataflowAttributes.getAttributeData( "rotSigma", rotSigma );

	Math::Matrix< double, 6, 6 > cov( Math::Matrix< double, 6, 6 >::zeros( ) );
	cov( 0, 0 ) = cov( 1, 1 ) = cov( 2, 2 ) = posSigma * posSigma;
	cov( 3, 3 ) = cov( 4, 4 ) = cov( 5, 5 ) = rotSigma * rotSigma;

	return cov;
}

}

/**
 * @ingroup dataflow_components
 * Component that adds a static error to a measurement.
 * This class contains an inversion implemented as a \c TriggerComponent.
 *
 * @par Input Ports
 * TriggerInPort<EventType> with name "Input".
 *
 * @par Output Ports
 * TriggerOutPort<EventType> with name "Output".
 *
 * @par The Pattern
 * This describes just one of several push/pull combinations
 * @verbatim
<Pattern name="StaticErrorAdd">
	<Input>
		<Node name="A"/>
		<Node name="B"/>
		<Edge name="AB" source="A" destination="B">
			<Predicate>type=="6D"&amp;&amp;mode=="push"</Predicate>
		</Edge>
	</Input>
	<Output>
		<Edge name="BA" source="B" destination="A">
			<Attribute name="type" value="6DError"/>
			<Attribute name="mode" value="push"/>
		</Edge>
	</Output>
	<DataflowConfiguration>
		<UbitrackLib class="StaticErrorAddPose"/>
	</DataflowConfiguration>
</Pattern>
@endverbatim
 *
 */
template< class EventTypeIn, class EventTypeOut, unsigned nErrSize >
class StaticErrorAddComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	StaticErrorAddComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Dataflow::TriggerComponent( sName, subgraph )
		, m_inPort( "Input", *this )
		, m_outPort( "Output", *this )
	{
		m_staticError = readConfiguration( subgraph, EventTypeOut() );
	}

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		m_outPort.send( EventTypeOut( t, typename EventTypeOut::value_type( *m_inPort.get(), m_staticError ) ) );
	}

protected:
	/** static covariance */
	Math::Matrix< double, nErrSize, nErrSize > m_staticError;

	/** Input port of the component. */
	Dataflow::TriggerInPort< EventTypeIn > m_inPort;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< EventTypeOut > m_outPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< StaticErrorAddComponent< Measurement::Pose, Measurement::ErrorPose, 6 > > ( "StaticErrorAddPose" );
}

} } // namespace Ubitrack::Components
