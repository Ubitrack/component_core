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
 * Demo/Template null component
 * This files contains the null component that does (almost) nothing.
 *
 * @author Manuel Huber <huberma@in.tum.de>
 */

#include <string>
#include <iostream>

#include <boost/bind.hpp>

#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>

namespace Ubitrack { namespace Components {

using namespace Dataflow;

/**
 * @ingroup dataflow_components
 * The NullComponent dataflow component.
 * This component does nothing and servers only as an example
 * and programming template.
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * PullSupplier<int> port with name "Output"
 *
 * @par Configuration
 * None.
 *
 * @par Operation
 * Does nothing really..
 * If the port is pulled, it always supplies "0".
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 * - int: NullComponent
 */
class NullComponent : public Component
{
public:

	/**
	 * Standard component constructor.
	 *
	 * @param nm Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	NullComponent( const std::string& nm, boost::shared_ptr< Graph::UTQLSubgraph > );

protected:
	/**
	 * Function bound to pull port.
	 * Always returns "0".
	 * @param t Timestamp for which the data is requested.
	 */
    int nullFunction ( Ubitrack::Measurement::Timestamp t )
    {
        return 0;
    }

	/**
	 * Output port of the component.
	 */
	PullSupplier< int > m_Port;
};


NullComponent::NullComponent( const std::string& nm, boost::shared_ptr< Graph::UTQLSubgraph > )
	: Ubitrack::Dataflow::Component( nm )
	, m_Port( "Output", *this, boost::bind( &NullComponent::nullFunction, this, _1 ) )
{
}

UBITRACK_REGISTER_COMPONENT( ComponentFactory* const cf ) {
	cf->registerComponent< NullComponent > ( "NullComponent" );
}

} } // namespace Ubitrack::Components
