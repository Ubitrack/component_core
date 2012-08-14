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
 *
 * @file Component which increments the timestamp of the input event
 * before pushing the event onwards. This functionality is sometimes
 * needed to guarantee that some parts of the dataflow are triggered
 * after others or, more generally, in a specified sequence. The
 * component(s) to be triggered first is connected directly to the
 * event source. Components to be triggered afterwards are connected
 * to the event source via this delay component. The newer timestamp
 * guarantees that the Ubitrack event queue processes the event after
 * the original event.

 * @author Peter Keitler <keitler@in.tum.de>
 */

#include <log4cpp/Category.hh>

#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>


namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components This component increments the
 * timestamp of the input event before pushing the event onwards. This
 * functionality is sometimes needed to guarantee that some parts of
 * the dataflow are triggered after others or, more generally, in a
 * specified sequence. The component(s) to be triggered first is
 * connected directly to the event source. Components to be triggered
 * afterwards are connected to the event source via this delay
 * component. The newer timestamp guarantees that the Ubitrack event
 * queue processes the event after the original event.
 *
 * @par Input Ports PullConsumer<Measurement> with name "Input".
 * PushConsumer<Measurement::Button> with name "Trigger".
 *
 * @par Output Ports
 * PushSupplier<Measurement::Button> with name "DelayedTrigger".
 *
 * @par Configuration
 *
 * @par Operation
 *
 * @par Instances
 */
class Synchronizer
    : public Dataflow::Component
{
public:
    /**
     * UTQL component constructor.
     *
     * @param sName Unique name of the component.
     * @param subgraph UTQL subgraph
     */
    Synchronizer( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Dataflow::Component( sName )
		, m_inPort( "Trigger", *this, boost::bind( &Synchronizer::receiveEvent, this, _1 ) )
		, m_outPort( "DelayedTrigger", *this )
		, m_logger( log4cpp::Category::getInstance( "Ubitrack.Components.Synchronizer" ) )
    {
    }

    /** Method that computes the result. */
    void receiveEvent( const Measurement::Button& event )
    {
		m_outPort.send( Measurement::Button( Measurement::now(), *event ) );
    }

protected:
    /** Input port of the component. */
    Dataflow::PushConsumer< Measurement::Button > m_inPort;

    /// Output port of the component
    Dataflow::PushSupplier< Measurement::Button > m_outPort;

    /** log4cpp logger reference */
    log4cpp::Category& m_logger;	
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
    cf->registerComponent< Synchronizer > ( "Synchronizer" );
}

} } // namespace Ubitrack::Components
