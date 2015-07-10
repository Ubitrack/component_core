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
 * This file contains an evaluation component
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */
#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * Evaluates prediction components and relates the prediction error to pixels on the screen.
 * Results are printed on the screen.
 * 
 * @par Input Ports
 * PushConsumer<ErrorPose> with name "InRaw". Receives unprocessed measurements from the tracker.<br>
 * PullConsumer<ErrorPose> with name "InPredictor". Gets measurements from the predictor.<br>
 * PullConsumer<Matrix3x3> with name "Intrinsics". Camera intrinsics.<br>
 *
 * @par Output Ports
 * PushSupplier<ErrorPose> with name "OutRaw". Resends the measurements from InRaw AFTER the calculation. 
 * The predictor input must be connected to this port, as otherwise the data flow priorization would 
 * push the raw measurements into the predictor before the prediction error is evaluated.
 *
 * @par Operation
 * The component computes the on-screen error by applying the camera matrix to both measurements
 * and then computes the difference. Results are printed to stdout.
 *
 */
template< class MT >
class ScreenPixelError
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	ScreenPixelError( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  )
		: Dataflow::Component( sName )
		, m_nMeasurements( 0 )
		, m_totalPosError( 0 )
		, m_inPos1( "InPos1", *this, boost::bind( &ScreenPixelError::receivePos1, this, _1 ) )
		, m_inPos2( "InPos2", *this )
		, m_inIntrinsics( "Intrinsics", *this )
		, m_outPixelError( "OutPixelError", *this )
    {
    }

	void receivePos1( const MT& _pos1 )
	{
		namespace ublas = boost::numeric::ublas;
		
		try
		{
			Measurement::Matrix3x3 intrinsics( m_inIntrinsics.get( _pos1.time() ) );
			Measurement::Position _pos2( m_inPos2.get( _pos1.time() ) );
			
			// pixel position
			Math::Vector< double, 3 > pos1( ublas::prod( *intrinsics, *_pos1 ) );
			Math::Vector< double, 3 > pos2( ublas::prod( *intrinsics, *_pos2 ) );
			pos1 /= double( pos1( 2 ) );
			pos2 /= double( pos2( 2 ) );

			double posErr = ublas::norm_2( pos1 - pos2 );
			m_totalPosError += posErr;
			
			
			m_nMeasurements++;
			
			// std::cout << "Prediction error: pos=" << posErr << ", ang=" << angleErr << ", dt=" << ( pose1.time() - m_lastTime ) * 1e-9 << 
			// 	", avgPos=" << m_totalPosError / m_nMeasurements << std::endl ;

			if (m_outPixelError.isConnected()) {
				m_outPixelError.send( Measurement::Distance( _pos1.time(), posErr ) );
			}

		}
		catch ( const Util::Exception& )
		{}
		
		m_lastTime = _pos1.time();
    }

protected:
	unsigned m_nMeasurements;
	double m_totalPosError;

	Measurement::Timestamp m_lastTime;
	Dataflow::PushConsumer< MT > m_inPos1;
	Dataflow::PullConsumer< MT > m_inPos2;
	Dataflow::PullConsumer< Measurement::Matrix3x3 > m_inIntrinsics;
	Dataflow::PushSupplier< Measurement::Distance > m_outPixelError;
	
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< ScreenPixelError< Measurement::Position > > ( "ScreenPixelError" );
}

} } // namespace Ubitrack::Components
