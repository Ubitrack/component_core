
// std
#include <fstream>
#include <string>
#include <vector>

// boost
#include <boost/bind.hpp>

// Ubitrack
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/Timestamp.h>
#include <utMath/Vector.h>
#include <utUtil/Logging.h>


//using namespace Ubitrack;
//using namespace Ubitrack::Measurement;
//using namespace Ubitrack::Components;


#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.TrackingContestFileReader" ) );

namespace Ubitrack { namespace Components {

class TrackingContestFileReader
	: public Dataflow::Component
	{
	public:
		TrackingContestFileReader( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
			: Dataflow::Component( sName )
			, m_counter( 0 )
			, m_position( -1 )
			, m_outPort( "Output", *this, boost::bind( &TrackingContestFileReader::sendOutput, this, _1 ) )
		{
			std::string filename;
			if ( subgraph -> m_DataflowAttributes.hasAttribute( "file" ) )
				filename = subgraph -> m_DataflowAttributes.getAttributeString( "file" );

			std::fstream posfile ( filename.c_str(), std::fstream::in ); 
			std::cout << "Lade Daten aus der Datei : " << filename.c_str() << std::endl;
		
			while (!posfile.eof())
			{
				double x, y, z;
				posfile >> x >> y >> z;
			
				//Ubitrack::Math::Vector < 3 >  posXYZ (x, y, z);
				//const Ubitrack::Math::Quaternion Rot( 0.0, 0.0, 0.0, 1.0 );
				Ubitrack::Math::Vector< double, 3 > position( x, y, z );

				PositionList . push_back ( position );
				m_counter++;
			
				//std::cout << Pose6D << std::endl;

			}
			posfile.close ();
		};
	protected:

		Measurement::Vector3D sendOutput( Measurement::Timestamp t )
		{
			if( (++m_position ) >= m_counter)
				m_position = 0;
			return Measurement::Vector3D( t, PositionList.at( m_position ) );
		};

		unsigned int m_counter;

		unsigned int m_position;

		std::vector<Ubitrack::Math::Vector< double, 3 > >	PositionList; //vector mit allen Punkten

		Ubitrack::Dataflow::PullSupplier< Measurement::Position > m_outPort;	
	
};

class TrackingContestPositionListReader
	: public Dataflow::Component
	{
	public:
		TrackingContestPositionListReader( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
			: Dataflow::Component( sName )		
			, m_outPort( "Output", *this, boost::bind( &TrackingContestPositionListReader::sendOutput, this, _1 ) )
		{
			std::string filename;
			if ( subgraph -> m_DataflowAttributes.hasAttribute( "file" ) )
				filename = subgraph -> m_DataflowAttributes.getAttributeString( "file" );

			std::fstream posfile ( filename.c_str(), std::fstream::in ); 
			LOG4CPP_INFO(logger, "Lade Daten aus der Datei : " << filename.c_str());
		
			while (!posfile.eof())
			{
				double x, y, z;
				posfile >> x >> y >> z;
			
				//Ubitrack::Math::Vector < 3 >  posXYZ (x, y, z);
				//const Ubitrack::Math::Quaternion Rot( 0.0, 0.0, 0.0, 1.0 );
				Ubitrack::Math::Vector< double, 3 > position( x, y, z );

				PositionList . push_back ( position );
			
					

			}
			posfile.close ();
		};
	protected:

		Measurement::PositionList sendOutput( Measurement::Timestamp t )
		{				
			return Measurement::PositionList( t, PositionList );
		};

		

		std::vector<Ubitrack::Math::Vector< double, 3 > >	PositionList; //vector mit allen Punkten

		Ubitrack::Dataflow::PullSupplier< Measurement::PositionList > m_outPort;	
	
};

UBITRACK_REGISTER_COMPONENT( Ubitrack::Dataflow::ComponentFactory* const cf )
{
	cf->registerComponent< TrackingContestFileReader > ( "TrackingContestPositionReader" );
	cf->registerComponent< TrackingContestPositionListReader > ( "TrackingContestPositionListReader" );
}

} } //namespace Ubitrack::Components

