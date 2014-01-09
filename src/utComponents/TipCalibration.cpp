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
 * Tip/HotSpot calibration component.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utCalibration/TipCalibration.h>

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Tip/HotSpot calibration component.
 *
 * @par Input Ports
 * ExpansionInPort< Pose > with name "Input".
 *
 * @par Output Ports
 * TriggerOutPort< Position > with name "Output".
 *
 * @par Configuration
 * DataflowConfiguration: "expansion" = "time" or "space"
 *
 * @par Operation
 * The component computes the location of a tip in the coordinate frame of a marker,
 * given a list of at least three marker poses. For details see
 * \c Ubitrack::Calibration::tipCalibration.
 *
 * @par Instances
 * Registered for the following expansions and push/pull configurations:
 * <tr><th> Name </th><th> Expansion </th><th> Push/Pull </th></tr>
 * <tr><td> \c PullTipCalibration </td><td> Time </td><td> Push in, Pull out </td></tr>
 * </table>
  */
class TipCalibrationComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	TipCalibrationComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::TriggerComponent( sName, pConfig )
		, m_inPort( "Input", *this )
		, m_outPort( "Output", *this )
    {
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		if ( m_inPort.get()->size() < 3 )
			UBITRACK_THROW( "not enough poses" );

		Math::Vector< double, 3 > pm;
		Math::Vector< double, 3 > pw;
		Calibration::tipCalibration( *m_inPort.get(), pm, pw );

		m_outPort.send( Measurement::Position( t, pm ) );
    }

protected:
	/** Input port of the component. */
	Dataflow::ExpansionInPort< Math::Pose > m_inPort;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Position > m_outPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< TipCalibrationComponent > ( "TipCalibration" );
}

} } // namespace Ubitrack::Components
