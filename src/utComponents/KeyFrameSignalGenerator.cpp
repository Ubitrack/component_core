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
 * MovementSignalGenerator
 *
 * @author Frieder Pankratz <pankratz@in.tum.de>
 */

#include <log4cpp/Category.hh>

#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>


namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * This component generates a signal event if a measurement arrives and the movement was bit enough
 *
 * @par Input Ports
 * PushConsumer<Measurement> with name "Input".
 *
 * @par Output Ports
 * PushSupplier<Button> with name "Output".
 *
 * @par Configuration
 * none
 *
 * @par Operation
 *
 * @par Instances
 */
template< class EventType >
class KeyFrameSignalGenerator
    : public Dataflow::Component
{
public:
    /**
     * UTQL component constructor.
     *
     * @param sName Unique name of the component.
     * @param subgraph UTQL subgraph
     */
    KeyFrameSignalGenerator( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph  )
		: Dataflow::Component( sName )
		, m_inPort( "Input", *this, boost::bind( &KeyFrameSignalGenerator::receiveMeasurement, this, _1 ) )
		, m_outPort( "Output", *this )
		, m_button( ' ' )
		, m_minDistance(0.1)
		, m_logger( log4cpp::Category::getInstance( "Ubitrack.Components.KeyFrameSignalGenerator" ) )
    {
		// read button key
		std::string button( " " );

		if ( subgraph->m_DataflowAttributes.hasAttribute( "button" ) )
			button = subgraph->m_DataflowAttributes.getAttributeString( "button" );
		
		
		subgraph->m_DataflowAttributes.getAttributeData<double>( "minDistance", m_minDistance );
			
			
			
		if ( button.empty() )
			m_button = Math::Scalar< int >( -1 );
		else
			m_button = Math::Scalar< int >( button[ 0 ] );
    }

    /** Method that computes the result. */
    void receiveMeasurement( const EventType& event )
    {
		if(sufficientMovement(event))
			m_outPort.send( Measurement::Button( event.time(), m_button ) );
    }

protected:
    /** Input port of the component. */
    Dataflow::PushConsumer< EventType > m_inPort;

    /// Output port of the component
	Dataflow::PushSupplier< Measurement::Button > m_outPort;

	Math::Scalar< int > m_button;
	
	EventType m_lastMeasurement;

    /** log4cpp logger reference */
    log4cpp::Category& m_logger;	
	
	double m_minDistance;
	
	bool sufficientMovement( const typename EventType& );
};

template<>
bool KeyFrameSignalGenerator< Measurement::Pose >::sufficientMovement( const Measurement::Pose& pose )
{
	if(m_lastMeasurement.time() == 0)
	{
		m_lastMeasurement = pose;
		return true;
		
	}
	
	double distance = norm_2 ( m_lastMeasurement->translation() - pose->translation() );
	
	if(distance > m_minDistance){
		m_lastMeasurement = pose;
		return true;
	} else 
		return false;	 	
};

template<>
bool KeyFrameSignalGenerator< Measurement::Position >::sufficientMovement( const Measurement::Position& pos )
{
	if(m_lastMeasurement.time() == 0)
	{
		m_lastMeasurement = pos;
		return true;
		
	}
	
	double distance = norm_2 ( *m_lastMeasurement - *pos );
	
	if(distance > m_minDistance){
		m_lastMeasurement = pos;
		return true;
	} else 
		return false;	 	
};

template<>
bool KeyFrameSignalGenerator< Measurement::Rotation >::sufficientMovement( const Measurement::Rotation& rot )
{
	if(m_lastMeasurement.time() == 0)
	{
		m_lastMeasurement = rot;
		return true;
		
	}
	Math::Quaternion qdiff = ((*m_lastMeasurement) * ~(*rot));
	double distance = std::abs(qdiff.angle());
	
	if(distance > m_minDistance){
		m_lastMeasurement = rot;
		return true;
	} else 
		return false;	 	
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
    cf->registerComponent< KeyFrameSignalGenerator< Measurement::Pose > > ( "PoseKeyFrameSignalGenerator" );
    cf->registerComponent< KeyFrameSignalGenerator< Measurement::Position > > ( "PositionKeyFrameSignalGenerator" );
    cf->registerComponent< KeyFrameSignalGenerator< Measurement::Rotation > > ( "RotationKeyFrameSignalGenerator" );
}

} } // namespace Ubitrack::Components
