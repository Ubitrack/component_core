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
 * Static measurement component.
 * This file contains the static measurement
 * component. The component always returns the
 * event as configured in the XML configuration on
 * its pull port.
 *
 * This is primarily useful for static offline calibration
 * of certain spatial relationships or constant projecton matrices
 * and the like.
 *
 * @author Manuel Huber <huberma@in.tum.de>
 */

#include <string>
#include <sstream>

#include <boost/bind.hpp>

#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utGraph/UTQLSubgraph.h>

namespace Ubitrack { namespace Components {

	/**
	 * @ingroup dataflow_components
	 * Static measurement component.
	 * Static measurement component.
	 * This file contains the static measurement
	 * component. The component always returns the
	 * event as configured in the XML configuration on
	 * its pull port.
	 *
	 * This is primarily useful for static offline calibration
	 * of certain spatial relationships or constant projecton matrices
	 * and the like.
	 *
	 * @par Input Ports
	 * None.
	 *
	 * @par Output Ports
	 * PullSupplier<EventType> port with name "Output".

	 * @par Operation
	 * Whenever an event is requested on the pull output port
	 * the component returns the event as configured.
	 */
	template< class EventType > class StaticMeasurement
		: public Dataflow::Component
	{
	public:

		/**
		 * Standard component constructor.
		 * Standard component constructor. Also parses XML configuration.
		 *
		 * @param nm Unique name of the component.
		 * @param cfg ComponentConfiguration containing all configuration.
		 */

		StaticMeasurement( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
			: Dataflow::Component( sName )
			, m_outPort( "AB", *this, boost::bind( &StaticMeasurement::sendOutput, this, _1 ) )
		{
			if ( !subgraph->hasEdge( "AB" ) )
			{
 				UBITRACK_THROW( "Static measurement pattern without \"AB\"-Edge" );
			}
			Graph::UTQLSubgraph::EdgePtr edge = subgraph->getEdge( "AB" );

 			initMeasurement( *edge );
		}

	protected:

		/**
		 * Handler method for the output port.
		 * Sends the event as configured.
		 * @param t the Timestamp of the requested event.
		 * @return the requested event.
		 */
		EventType sendOutput( Measurement::Timestamp t )
		{
			return EventType( t, m_Data );
		}

		void initMeasurement( const Ubitrack::Graph::UTQLSubgraph::GraphEdgeAttributes& config );

		Dataflow::PullSupplier< EventType > m_outPort;

		EventType m_Data;
	};

	template<> void StaticMeasurement< Measurement::Matrix4x4 >::initMeasurement( const Ubitrack::Graph::UTQLSubgraph::GraphEdgeAttributes& config )
	{
		if ( !config.hasAttribute( "staticMatrix4x4" ) )
		{
			UBITRACK_THROW( "Static 4x4Matrix configuration without staticMatrix4x4 attribute" );
		}

		std::string matrixChars = config.getAttribute( "staticMatrix4x4" ).getText();

		double m[16];
		std::istringstream matrixString( matrixChars );
		for (int i=0; i < 16; ++i)
		{
			matrixString >> m[i];
		}

		m_Data = Measurement::Matrix4x4 ( Math::Matrix< double, 4, 4 > (m) );
	}

	template<> void StaticMeasurement< Measurement::Matrix3x3 >::initMeasurement( const Ubitrack::Graph::UTQLSubgraph::GraphEdgeAttributes& config )
	{
		if ( !config.hasAttribute( "staticMatrix3x3" ) )
		{
			UBITRACK_THROW( "Static 3x3Matrix configuration without staticMatrix3x3 attribute" );
		}

		std::string matrixChars = config.getAttribute( "staticMatrix3x3" ).getText();

		double m[9];
		std::istringstream matrixString( matrixChars );
		for (int i=0; i < 9; ++i)
		{
			matrixString >> m[i];
		}

		m_Data = Measurement::Matrix3x3 ( Math::Matrix< double, 3, 3 > (m) );
	}
	
	template<> void StaticMeasurement< Measurement::Matrix3x4 >::initMeasurement( const Ubitrack::Graph::UTQLSubgraph::GraphEdgeAttributes& config )
	{
		if ( !config.hasAttribute( "staticMatrix3x4" ) )
		{
			UBITRACK_THROW( "Static 3x4Matrix configuration without staticMatrix3x4 attribute" );
		}

		std::string matrixChars = config.getAttribute( "staticMatrix3x4" ).getText();

		double m[12];
		std::istringstream matrixString( matrixChars );
		for (int i=0; i < 12; ++i)
		{
			matrixString >> m[i];
		}

		m_Data = Measurement::Matrix3x4 ( Math::Matrix< double, 3, 4 > (m) );
	}


	// TODO: replace with measurement serializsations, could use getAttributeData then also and implement this generic..
	template<> void StaticMeasurement< Measurement::Position >::initMeasurement( const Ubitrack::Graph::UTQLSubgraph::GraphEdgeAttributes& config )
	{
		if ( !config.hasAttribute( "staticPosition" ) )
		{
			UBITRACK_THROW( "Static position configuration without staticPosition attribute" );
		}

		std::string positionChars = config.getAttribute( "staticPosition" ).getText();

		double p[3];
		std::istringstream positionString( positionChars );
		for (int i=0; i < 3; ++i)
		{
			positionString >> p[i];
		}

		m_Data = Measurement::Position ( Math::Vector< double, 3 > (p) );
	}
	
	// TODO: replace with measurement serializsations, could use getAttributeData then also and implement this generic..
	template<> void StaticMeasurement< Measurement::Position2D >::initMeasurement( const Ubitrack::Graph::UTQLSubgraph::GraphEdgeAttributes& config )
	{
		if ( !config.hasAttribute( "staticPosition2D" ) )
		{
			UBITRACK_THROW( "Static position configuration without staticPosition2D attribute" );
		}

		std::string positionChars = config.getAttribute( "staticPosition2D" ).getText();

		double p[2];
		std::istringstream positionString( positionChars );
		for (int i=0; i < 2; ++i)
		{
			positionString >> p[i];
		}

		m_Data = Measurement::Position2D ( Math::Vector< double, 2 > (p) );
	}

	// TODO: replace with measurement serializsations, could use getAttributeData then also and implement this generic..
	template<> void StaticMeasurement< Measurement::Distance >::initMeasurement( const Ubitrack::Graph::UTQLSubgraph::GraphEdgeAttributes& config )
	{
		if ( !config.hasAttribute( "staticDistance" ) )
		{
			UBITRACK_THROW( "Static scalar distance configuration without staticDistance attribute" );
		}

		std::string positionChars = config.getAttribute( "staticDistance" ).getText();

		double d;
		std::istringstream positionString( positionChars );
		positionString >> d;

		m_Data = Measurement::Distance ( Math::Scalar< double > ( d ) );
	}

	// TODO: replace with measurement serializsations, could use getAttributeData then also and implement this generic..
	template<> void StaticMeasurement< Measurement::Button >::initMeasurement( const Ubitrack::Graph::UTQLSubgraph::GraphEdgeAttributes& config )
	{
		if ( !config.hasAttribute( "button" ) )
		{
			UBITRACK_THROW( "Static scalar distance configuration without staticDistance attribute" );
		}

		std::string eventString = config.getAttribute( "button" ).getText();

		char event;
		std::istringstream eventStream( eventString );
		eventStream >> event;	
		
		m_Data = Measurement::Button ( event );
	}

	// TODO: replace with measurement serializsations, could use getAttributeData then also and implement this generic..
	template<> void StaticMeasurement< Measurement::Vector4D >::initMeasurement( const Ubitrack::Graph::UTQLSubgraph::GraphEdgeAttributes& config )
	{
		if ( !config.hasAttribute( "staticVector" ) )
		{
			UBITRACK_THROW( "Static vector configuration without staticVector attribute" );
		}

		std::string positionChars = config.getAttribute( "staticVector" ).getText();

		double p[4];
		std::istringstream positionString( positionChars );
		for (int i=0; i < 4; ++i)
		{
			positionString >> p[i];
		}

		m_Data = Measurement::Vector4D( Math::Vector< double, 4 >( p ) );
	}

	// yes yes.. rotation / position again.. will fix this all at once..
	template<> void StaticMeasurement< Measurement::Rotation >::initMeasurement( const Ubitrack::Graph::UTQLSubgraph::GraphEdgeAttributes& config )
	{
		if ( !config.hasAttribute( "staticRotation" ) )
		{
			UBITRACK_THROW( "Static rotation configuration without staticRotation attribute" );
		}

		std::string rotationChars = config.getAttribute( "staticRotation" ).getText();

		double r[4];
		std::istringstream rotationString( rotationChars );
		for (int i=0; i < 4; ++i)
		{
			rotationString >> r[i];
		}

		Math::Quaternion result( r[0], r[1], r[2], r[3] ); 
		m_Data = Measurement::Rotation ( result.normalize() );
	}

	// this seems like an opportunity to refactor rotation and position out of this
	// method.. but rather implement serialisation >> and << operators for measurements instead.
	template<> void StaticMeasurement< Measurement::Pose >::initMeasurement( const Ubitrack::Graph::UTQLSubgraph::GraphEdgeAttributes& config )
	{
		if ( !config.hasAttribute( "staticRotation" ) )
		{
			UBITRACK_THROW( "Static pose configuration without staticRotation attribute" );
		}

		std::string rotationChars = config.getAttribute( "staticRotation" ).getText();

		double r[4];
		std::istringstream rotationString( rotationChars );
		for (int i=0; i < 4; ++i)
		{
			rotationString >> r[i];
		}

		if ( !config.hasAttribute( "staticPosition" ) )
		{
			UBITRACK_THROW( "Static pose configuration without staticPosition attribute" );
		}

		std::string positionChars = config.getAttribute( "staticPosition" ).getText();

		double p[3];
		std::istringstream positionString( positionChars );
		for (int i=0; i < 3; ++i)
		{
			positionString >> p[i];
		}

		Math::Vector< double, 3 > trans( p );
		Math::Quaternion q( r[0], r[1], r[2], r[3] );

		m_Data = Measurement::Pose ( Math::Pose( q.normalize(), trans ) );
	}
	
	template<> void StaticMeasurement< Measurement::PoseList >::initMeasurement( const Ubitrack::Graph::UTQLSubgraph::GraphEdgeAttributes& config )
	{
		m_Data.reset( new Measurement::PoseList::value_type() );

		if ( !config.hasAttribute( "staticPoseList" ) )
			UBITRACK_THROW( "static pose list without \"staticPoseList\" attribute" );

		const TiXmlElement* pAttrib = config.getAttribute( "staticPoseList" ).getXML();
		if ( !pAttrib )
				UBITRACK_THROW( "Edge does not have a staticPoseList attribute element" );
		const TiXmlElement* pValElem = pAttrib->FirstChildElement( "Value" );
		if ( !pValElem )
				UBITRACK_THROW( "staticPoseList has no Value element" );
		const TiXmlElement* pPosElem = pValElem->FirstChildElement( "Attribute" );
		if ( !pPosElem )
				UBITRACK_THROW( "Value has no Attribute element" );
		if ( std::strcmp( pPosElem->Attribute( "name" ), "staticPose" ) )
				UBITRACK_THROW( "Value has no Attribute element named staticPose" );

		for( ; pPosElem; pPosElem = pPosElem->NextSiblingElement( "Attribute" ) )
		{
			const char* sValue = pPosElem->Attribute( "value" );
			if ( !sValue )
				UBITRACK_THROW( "staticPose has no value attribute" );


			Math::Vector< double, 7 > p;
			std::istringstream poseString( sValue );
			for ( unsigned i = 0; i < 7; i++ )
				poseString >> p( i );

				// Math::Pose::fromVector  ( p );
			m_Data->push_back( Math::Pose::fromVector ( p ) );
		}

	}

	template<> void StaticMeasurement< Measurement::PositionList >::initMeasurement( const Ubitrack::Graph::UTQLSubgraph::GraphEdgeAttributes& config )
	{
		m_Data.reset( new Measurement::PositionList::value_type() );

		if ( !config.hasAttribute( "staticPositionList" ) )
			UBITRACK_THROW( "static position list without \"staticPositionList\" attribute" );

		const TiXmlElement* pAttrib = config.getAttribute( "staticPositionList" ).getXML();
		if ( !pAttrib )
				UBITRACK_THROW( "Edge does not have a staticPositionList attribute element" );
		const TiXmlElement* pValElem = pAttrib->FirstChildElement( "Value" );
		if ( !pValElem )
				UBITRACK_THROW( "staticPositionList has no Value element" );
		const TiXmlElement* pPosElem = pValElem->FirstChildElement( "Attribute" );
		if ( !pPosElem )
				UBITRACK_THROW( "Value has no Attribute element" );
		if ( std::strcmp( pPosElem->Attribute( "name" ), "staticPosition" ) )
				UBITRACK_THROW( "Value has no Attribute element named staticPosition" );

		for( ; pPosElem; pPosElem = pPosElem->NextSiblingElement( "Attribute" ) )
		{
			const char* sValue = pPosElem->Attribute( "value" );
			if ( !sValue )
				UBITRACK_THROW( "staticPosition has no value attribute" );

			double p[3];
			std::istringstream positionString( sValue );
			for ( unsigned i = 0; i < 3; i++ )
				positionString >> p[i];

			m_Data->push_back( Math::Vector< double, 3 >( p ) );
		}

	}

	template<> void StaticMeasurement< Measurement::PositionList2 >::initMeasurement( const Ubitrack::Graph::UTQLSubgraph::GraphEdgeAttributes& config )
	{
		m_Data.reset( new Measurement::PositionList2::value_type() );

		if ( !config.hasAttribute( "staticPositionList" ) )
			UBITRACK_THROW( "static position list without \"staticPositionList\" attribute" );

		const TiXmlElement* pAttrib = config.getAttribute( "staticPositionList" ).getXML();
		if ( !pAttrib )
				UBITRACK_THROW( "Edge does not have a staticPositionList attribute element" );
		const TiXmlElement* pValElem = pAttrib->FirstChildElement( "Value" );
		if ( !pValElem )
				UBITRACK_THROW( "staticPositionList has no Value element" );
		const TiXmlElement* pPosElem = pValElem->FirstChildElement( "Attribute" );
		if ( !pPosElem )
				UBITRACK_THROW( "Value has no Attribute element" );
		//if ( std::strcmp( pPosElem->Attribute( "name" ), "staticPosition" ) )
		//		UBITRACK_THROW( "Value has no Attribute element named staticPosition" );

		for( ; pPosElem; pPosElem = pPosElem->NextSiblingElement( "Attribute" ) )
		{
			const char* sValue = pPosElem->Attribute( "value" );
			if ( !sValue )
				UBITRACK_THROW( "staticPosition has no value attribute" );

			double p[2];
			std::istringstream positionString( sValue );
			for ( unsigned i = 0; i < 2; i++ )
				positionString >> p[i];

			m_Data->push_back( Math::Vector< double, 2 >( p ) );
		}

	}

	template<> void StaticMeasurement< Measurement::DistanceList >::initMeasurement( const Ubitrack::Graph::UTQLSubgraph::GraphEdgeAttributes& config )
	{
		m_Data.reset( new Measurement::DistanceList::value_type() );

		if ( !config.hasAttribute( "staticDistanceList" ) )
			UBITRACK_THROW( "static distance list without \"staticDistanceList\" attribute" );

		const TiXmlElement* pAttrib = config.getAttribute( "staticDistanceList" ).getXML();
		if ( !pAttrib )
				UBITRACK_THROW( "Edge does not have a staticDistanceList attribute element" );
		const TiXmlElement* pValElem = pAttrib->FirstChildElement( "Value" );
		if ( !pValElem )
				UBITRACK_THROW( "staticDistanceList has no Value element" );
		const TiXmlElement* pDistElem = pValElem->FirstChildElement( "Attribute" );
		if ( !pDistElem )
				UBITRACK_THROW( "Value has no Attribute element" );
		//if ( std::strcmp( pDistElem->Attribute( "name" ), "staticDistance" ) )
		//		UBITRACK_THROW( "Value has no Attribute element named staticDistance" );

		for( ; pDistElem; pDistElem = pDistElem->NextSiblingElement( "Attribute" ) )
		{
			const char* sValue = pDistElem->Attribute( "value" );
			if ( !sValue )
				UBITRACK_THROW( "staticDistance has no value attribute" );

			double d;
			std::istringstream distString( sValue );
			distString >> d;

			m_Data->push_back( d );
		}

	}

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< StaticMeasurement< Measurement::Matrix4x4 > > ( "StaticMatrix4x4" );
	cf->registerComponent< StaticMeasurement< Measurement::Matrix3x4 > > ( "StaticMatrix3x4" );
	cf->registerComponent< StaticMeasurement< Measurement::Matrix3x3 > > ( "StaticMatrix3x3" );
	cf->registerComponent< StaticMeasurement< Measurement::Distance > > ( "StaticDistance" );
	cf->registerComponent< StaticMeasurement< Measurement::Position2D > > ( "StaticPosition2D" );
	cf->registerComponent< StaticMeasurement< Measurement::Position > > ( "StaticPosition" );
	cf->registerComponent< StaticMeasurement< Measurement::Vector4D > > ( "StaticVector4" );
	cf->registerComponent< StaticMeasurement< Measurement::Rotation > > ( "StaticRotation" );
	cf->registerComponent< StaticMeasurement< Measurement::Pose > > ( "StaticPose" );
	cf->registerComponent< StaticMeasurement< Measurement::Button > > ( "StaticEvent" );
	cf->registerComponent< StaticMeasurement< Measurement::PoseList > > ( "StaticPoseList" );
	cf->registerComponent< StaticMeasurement< Measurement::PositionList2 > > ( "StaticPositionList2" );
	cf->registerComponent< StaticMeasurement< Measurement::PositionList > > ( "StaticPositionList" );
	cf->registerComponent< StaticMeasurement< Measurement::DistanceList > > ( "StaticDistanceList" );
}

} } // namespace Ubitrack::Components
