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
 * Components that modifies timestamp
 *
 * @author Michael Schlegel <schlegem@in.tum.de>
 */

#include <log4cpp/Category.hh>

#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>


namespace Ubitrack { namespace Components {

  /**
   * @ingroup dataflow_components
   * This component adds a temporal offset to pushed measurements. 
   * It can be used to delay a measurement by the specified amount of time.
   *
   * @par Input Ports
   * PushConsumer<Measurement> with name "Input".
   *
   * @par Output Ports
   * PushSupplier<Measurement> with name "Output".
   *
   */
  template< class EventType >
  class DelayComponent
    : public Dataflow::Component
  {
  public:
    /**
     * UTQL component constructor.
     *
     * @param sName Unique name of the component.
     * @param subgraph UTQL subgraph
     */
    DelayComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph  )
      : Dataflow::Component( sName )
      , m_delayTime(0)
      , m_inPort( "Input", *this, boost::bind( &DelayComponent::receivePose, this, _1 ) )
      , m_outPort( "Output", *this )
      , m_logger( log4cpp::Category::getInstance( "Ubitrack.Components.Delay" ) )
    {
      		std::string tempValue;
		double ms = 0;
		std::string delayAttrName = "delayTime";

		if ( subgraph->m_DataflowAttributes.hasAttribute( delayAttrName  ) ){
		  // read value as a string
		  tempValue = subgraph->m_DataflowAttributes.getAttributeString( delayAttrName );
		  LOG4CPP_INFO( m_logger, "Setting delay time (string) " << tempValue );

		  // converting the string into a double value
		  std::istringstream inStream( tempValue ); 
		  inStream >> ms;
		  LOG4CPP_INFO( m_logger, "Setting delay time (double) " << ms );
		  

		  // the input is expected in ms, but we need ns
		  ms *= 1000000;

		  // finally store the casted value
		  m_delayTime = static_cast<long int>(ms);
		}

		LOG4CPP_INFO( m_logger, "Setting delay time" << m_delayTime );
    }

    /** Method that computes the result. */
    void receivePose( const EventType& event )
    {
      // a negative m_delayTime would delay a signal. 
      // for example a measurement arrives too early, a positive value
      // would delay that measurement
      m_outPort.send(EventType(event.time() +  m_delayTime, event));
    }

  protected:
    
    // holds the delay time 
    long int m_delayTime;

    /** Input port of the component. */
    Dataflow::PushConsumer< EventType > m_inPort;

    // Output port of the component
    Dataflow::PushSupplier< EventType > m_outPort;

    /** log4cpp logger reference */
    log4cpp::Category& m_logger;	

  };


  UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
    cf->registerComponent< DelayComponent< Measurement::Rotation > >   ( "DelayRotation" );
    cf->registerComponent< DelayComponent< Measurement::Position > >   ( "DelayPosition" );
    cf->registerComponent< DelayComponent< Measurement::Position2D > > ( "DelayPosition2" );
    cf->registerComponent< DelayComponent< Measurement::Pose > >       ( "DelayPose" );
	cf->registerComponent< DelayComponent< Measurement::Button > >       ( "DelayButton" );
  }

} } // namespace Ubitrack::Components
