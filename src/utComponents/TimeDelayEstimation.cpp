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
 * TimeDelayEstimation component.
 * This file contains a TimeDelayEstimation of two inputs implemented as a \c TriggerComponent.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */
#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utCalibration/Correlation.h>
#include <queue>

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.TimeDelayEstimation" ) );

namespace ublas = boost::numeric::ublas;

namespace Ubitrack { namespace Components {


class TimeDelayEstimation
	: public Dataflow::Component
{
public:
	typedef std::vector<Measurement::Position>::iterator vec_iter;

	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	TimeDelayEstimation( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::Component( sName )
		, m_inPortA( "AB1", *this, boost::bind( &TimeDelayEstimation::receiveSensor1, this, _1 )  )
		, m_inPortB( "AB2", *this, boost::bind( &TimeDelayEstimation::receiveSensor2, this, _1 )  )		
    {
		stop();
		m_data1 = new std::vector<Measurement::Position>();
		m_data2 = new std::vector<Measurement::Position>();

		m_recordSize = 10 * 1000 * 1000000LL;
		m_sliceSize = 1 * 1000 * 1000000LL;
		m_maxTimeOffset = 500;
    }

	~TimeDelayEstimation(){
		stop();
	}

	void receiveSensor1(const Measurement::Position& p){
		m_data1->push_back(p);
		//LOG4CPP_INFO(logger, m_data1->size() << " : " << m_data2->size());
		checkData();
	}

	void receiveSensor2(const Measurement::Position& p){
		m_data2->push_back(p);
		checkData();
	}

	void checkData(){
		if(m_data1->size() > 0 && m_data2->size() > 0){
			bool avail1 = (m_data1->back().time() - m_data1->front().time()) >= m_recordSize;
			bool avail2 = (m_data2->back().time() - m_data2->front().time()) >= m_recordSize;
			if(avail1 && avail2){
				addDataToThread();
			}
		}
	}

	void addDataToThread(){
		
		boost::mutex::scoped_lock l( m_mutex );		
		m_threadData1.push(m_data1);
		m_threadData2.push(m_data2);
		
		m_data1 = new std::vector<Measurement::Position>();
		m_data2 = new std::vector<Measurement::Position>();
	}

	void threadProc();

	void estimateTimeDelay(vec_iter data1First, vec_iter data1Last, vec_iter data2First, vec_iter data2Last);

	Math::Vector3d interpolate(const std::vector<Measurement::Position>& data, Measurement::Timestamp t);
	
	/** Component start method. starts the thread */
	virtual void start();

	/** Component stop method, stops thread */
	virtual void stop();
protected:
	/** Input port A of the component. */
	Dataflow::PushConsumer< Measurement::Position > m_inPortA;

	/** Input port B of the component. */
	Dataflow::PushConsumer< Measurement::Position > m_inPortB;

	std::vector<Measurement::Position>* m_data1;
	std::vector<Measurement::Position>* m_data2;

	boost::mutex m_mutex;
	boost::condition_variable m_cond;
	std::queue<std::vector<Measurement::Position>*> m_threadData1;
	std::queue<std::vector<Measurement::Position>*> m_threadData2;

	// the thread
	boost::scoped_ptr< boost::thread > m_Thread;

	// stop the thread?
	volatile bool m_bStop;

	int m_recordSize;
	int m_sliceSize;
	int m_maxTimeOffset;
	
	
	
};

void TimeDelayEstimation::stop()
{
	if ( m_running )
	{
		m_running = false;
		m_bStop = true;
		LOG4CPP_INFO( logger, "Trying to stop highgui frame grabber");
		if ( m_Thread )
		{
			m_Thread->join();
			LOG4CPP_INFO( logger, "Trying to stop highgui frame grabber, thread joined");
		}
	}
}

void TimeDelayEstimation::start()
{
	if ( !m_running )
	{
		m_running = true;
		m_bStop = false;
		m_Thread.reset( new boost::thread( boost::bind ( &TimeDelayEstimation::threadProc, this ) ) );
	}
}

void TimeDelayEstimation::threadProc(){

	while ( !m_bStop )
	{
		std::vector<Measurement::Position>* data1 = 0;
		std::vector<Measurement::Position>* data2 = 0;
		{
			boost::mutex::scoped_lock l( m_mutex );
			if(m_threadData1.size() > 0){
				data1 = m_threadData1.front();
				data2 = m_threadData2.front();
				m_threadData1.pop();
				m_threadData2.pop();
			}			
		}

		if(data1 == 0){
			Sleep(m_recordSize/4);
			continue;
		}

		//new data available

		// get first common timestamp
		Measurement::Timestamp startTime = std::max(m_data1->front().time(), m_data2->front().time());
		
		// interpolate data in ms steps
		std::vector<double>::size_type totalCount = m_recordSize/1000000LL;
		std::vector<double> corr_data1(totalCount);
		std::vector<double> corr_data2(totalCount);

		for(std::vector<double>::size_type i=0;i<totalCount;i++){
			Measurement::Timestamp t = startTime+i*1000000LL;
			Math::Vector3d p1 = interpolate(*data1, t);
			Math::Vector3d p2 = interpolate(*data2, t);

			// reduce to one dimension
			corr_data1.push_back( ublas::norm_2(p1) );
			corr_data2.push_back( ublas::norm_2(p2) );
		}

		
	}
}

Math::Vector3d TimeDelayEstimation::interpolate(const std::vector<Measurement::Position>& data, Measurement::Timestamp t){
		Measurement::Position p1,p2;
		double factor = 1.0;		
		for(int i=1; i<data.size();i++){
			if(data[i].time() > t){
				p1 = data[i-1];
				p2 = data[i];
				Measurement::Timestamp eventDifference = p2.time() - p1.time();
				Measurement::Timestamp timeDiff = t - p1.time();

				// check for timeout
								
				if ( eventDifference )
					factor = double( timeDiff ) / double( eventDifference );
				else
					factor = 1.0;

				break;

			}
		}
		//LOG4CPP_INFO(logger, p1 << " : " << p2 << " : " << factor);
		return linearInterpolate(*p1, *p2, factor );
	}

void TimeDelayEstimation::estimateTimeDelay(vec_iter data1First, vec_iter data1Last, vec_iter data2First, vec_iter data2Last){
		Measurement::Timestamp startTime = std::min(m_data1->front().time(), m_data2->front().time());

		startTime = startTime + 500*1000000LL;

		std::vector<double> corr_data1;
		std::vector<double> corr_data2;

		corr_data1.reserve(1000);
		corr_data2.reserve(1000);

		for(int i=0;i<1000;i++){
			Measurement::Timestamp t = startTime+i*1000000LL;
			Math::Vector3d p1 = interpolate(*m_data1, t);

			corr_data1.push_back( ublas::norm_2(p1) );
		}

		int timeDelay = 0;
		double correlation = 0;

		for(int j=-500;j<500;j++){
			corr_data2.clear();

			for(int i=0;i<1000;i++){
				Measurement::Timestamp t = startTime+(i+j)*1000000LL;
				Math::Vector3d p2 = interpolate(*m_data2, t);

				corr_data2.push_back( ublas::norm_2(p2) );
			}

			double corr = Ubitrack::Calibration::computeCorrelation(corr_data1, corr_data2);
			if(corr > correlation){
				correlation = corr;
				timeDelay = j;
			}

		}

		

		
		LOG4CPP_INFO(logger, "TimeDelayEstimation:" << timeDelay << " corr:" << correlation);
		
	}


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	
	cf->registerComponent< TimeDelayEstimation > ( "TimeDelayEstimationPosition3D" );

}

} } // namespace Ubitrack::Components
