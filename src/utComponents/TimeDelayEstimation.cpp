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
#include <boost/foreach.hpp>
#include <utUtil/CalibFile.h>
#include <boost/archive/text_oarchive.hpp>
#include <utMath/Stochastic/Average.h>

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.TimeDelayEstimation" ) );

namespace ublas = boost::numeric::ublas;

namespace Ubitrack { namespace Components {

typedef struct{
	double corr;
	Measurement::Timestamp offset;
} CorrData;

class TimeDelayEstimation
	: public Dataflow::Component
{
public:
	typedef std::vector<double>::iterator vec_iter;

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

		m_recordSizeInMs = 3 * 1000;
		m_sliceSizeInMs =  2000;		
		m_maxTimeOffsetInMs = 200;		
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
			bool avail1 = (m_data1->back().time() - m_data1->front().time()) >= m_recordSizeInMs*1000000LL;
			bool avail2 = (m_data2->back().time() - m_data2->front().time()) >= m_recordSizeInMs*1000000LL;
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

	std::map< int, std::vector< double > > m_correlationData;


	// the thread
	boost::scoped_ptr< boost::thread > m_Thread;

	// stop the thread?
	volatile bool m_bStop;

	long m_recordSizeInMs;	
	long m_maxTimeOffsetInMs;
	long m_sliceSizeInMs;
	
	
	
};

void TimeDelayEstimation::stop()
{
	if ( m_running )
	{
		m_running = false;
		m_bStop = true;
		LOG4CPP_INFO( logger, "Trying to stop TimeDelayEstimation");
		if ( m_Thread )
		{
			m_Thread->join();
			LOG4CPP_INFO( logger, "Trying to stop TimeDelayEstimation, thread joined");
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

	int superindex = 0;

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
			LOG4CPP_INFO(logger, "no new data");
			Sleep(m_recordSizeInMs/4);
			continue;
		}
		//new data available
		LOG4CPP_INFO(logger, "new data");
		
		// check if data is useable
		Math::Stochastic::Average<Math::Vector3d, Math::ErrorVector<double, 3 > > averageData;
		std::vector< Math::Vector3d > tmpData;
		for(int i=0;i < data1->size();i++){
			tmpData.push_back(*(data1->at(i)));
		}
		Math::ErrorVector<double, 3 > meanWithError = averageData.mean(tmpData);

		//LOG4CPP_INFO(logger, "RMS: "<< meanWithError.getRMS() << " : " << meanWithError.covariance );
		if(meanWithError.getRMS() < 0.05){
			LOG4CPP_INFO(logger, "too little movement, RMS:" << meanWithError.getRMS());
			delete data1;
			delete data2;
			continue;
		}

		
		/*double length = 0;
		for(int i=1;i < data1->size();i++){
			length += ublas::norm_2(*(data1->at(i)) - *(data1->at(i-1))) ;
		}
		if(length < 0.7){
			LOG4CPP_INFO(logger, "too short:" << length);
			delete data1;
			delete data2;
			continue;
		}
		LOG4CPP_INFO(logger, "length:" << length);
		*/

		// get first common timestamp
		Measurement::Timestamp startTime = std::max(data1->front().time(), data2->front().time());
		Measurement::Timestamp endTime = std::min(data1->back().time(), data2->back().time());
		
		// interpolate data in ms steps
		std::vector<double>::size_type totalCount = (endTime - startTime) / 1000000L;
		LOG4CPP_INFO(logger, "totalCount:"<<totalCount);
		std::vector<double> corr_data1;
		corr_data1.reserve(totalCount);
		std::vector<double> corr_data2;
		corr_data2.reserve(totalCount);

		//std::vector<Math::Scalar<double> > corr_data1Test1;
		//std::vector<Math::Scalar<double> > corr_data1Test2;

		for(std::vector<double>::size_type i=0;i<totalCount;i++){
			Measurement::Timestamp t = startTime+i*1000000LL;
			Math::Vector3d p1 = interpolate(*data1, t);
			Math::Vector3d p2 = interpolate(*data2, t);

			//LOG4CPP_INFO(logger, "time:" << t << " value" << p1 );
						
			// reduce to one dimension
			corr_data1.push_back( ublas::norm_2(p1) );
			corr_data2.push_back( ublas::norm_2(p2) );
			//corr_data1Test1.push_back( Math::Scalar<double>(ublas::norm_2(p1)));
			//corr_data1Test2.push_back( Math::Scalar<double>(ublas::norm_2(p2)));
		}
		/*
		Measurement::DistanceList n1(startTime, corr_data1Test1);
		Measurement::DistanceList n2(startTime, corr_data1Test2);
		
		std::stringstream ss1;
		ss1 << "d:\\temp\\corr1" << startTime << ".txt";
		Util::writeCalibFile( ss1.str(), n1 );

		std::stringstream ss2;
		ss2 << "d:\\temp\\corr2" << startTime << ".txt";
		Util::writeCalibFile( ss2.str(), n2 );
		*/
	

		delete data1;
		delete data2;

		

		// select slices and estimate time delay
		LOG4CPP_INFO(logger, "corr_data1:"<<corr_data1.size());

		int countSlices = (corr_data1.size() - m_maxTimeOffsetInMs*2) / m_sliceSizeInMs;

		LOG4CPP_INFO(logger, "count of slices:" << countSlices);

		 vec_iter data1First = corr_data1.begin(); 
		 vec_iter data2First = corr_data2.begin();

		for(int i=0;i<countSlices;i++){
			data1First = data1First + m_maxTimeOffsetInMs;
			data2First = data2First + m_maxTimeOffsetInMs;

			estimateTimeDelay(data1First, data1First+m_sliceSizeInMs, data2First, data2First+m_sliceSizeInMs);

			/*
			std::stringstream ss1;
			ss1 << "d:\\temp\\corrdata\\corr1_"<< superindex << "_" << i << ".txt";			

			std::stringstream ss2;
			ss2 << "d:\\temp\\corrdata\\corr2_"<< superindex << "_" << i << ".txt";

			
			std::ofstream stream1(ss1.str());		
			std::ofstream stream2(ss2.str());		
						
			std::string linesep( "\n" );

			vec_iter tmp1 = data1First;
			int index = 0;
			
			for(;tmp1 != data1First+m_sliceSizeInMs;++tmp1){
				stream1 << index;
				stream1 << ";";
				stream1 << *tmp1;
				stream1 << linesep;
				index++;
			}
			vec_iter tmp2 = data2First;
			index = 0;
			for(;tmp2 != data2First+m_sliceSizeInMs;++tmp2){
				stream2 << index;
				stream2 << ";";
				stream2 << *tmp2;
				stream2 << linesep;
				index++;
			}
			stream1.close();
			stream2.close();
			*/
		}

		std::map< int, std::vector< double > >::iterator it = m_correlationData.begin();
		int offset=0;
		double correlation=0;

		/*
		std::stringstream ss1;
		ss1 << "d:\\temp\\corrdata\\correlation.txt";		
		std::ofstream stream1(ss1.str());		
		*/
		for(;it != m_correlationData.end();++it){

		
			Math::Stochastic::Average<double, double> average;
			
			double value = average.mean(it->second);
			if(correlation < value)
			{
				correlation = value;
				offset = it->first;
			}
			//LOG4CPP_INFO(logger, "offset:" << it->first << " corr:" << value);
							
			/*			
			std::string linesep( "\n" );
			
			stream1 << it->first;
			stream1 << ";";
			stream1 << value;
			stream1 << linesep;			
			*/
		
		}
		//stream1.close();

		LOG4CPP_INFO(logger, "Result: offset:" << offset << " corr:" << correlation);

		
		superindex++;
		
		
	}
}

Math::Vector3d TimeDelayEstimation::interpolate(const std::vector<Measurement::Position>& data, Measurement::Timestamp t){
		Measurement::Position p1,p2;
		double factor = 1.0;		
		for(int i=0; i<data.size();i++){
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
	
	std::vector<double> data1(data1First, data1Last);

	Measurement::Timestamp ts = Measurement::now();
	
	for(int j=-m_maxTimeOffsetInMs;j<m_maxTimeOffsetInMs;j++){
		std::vector<double> data2(data2First+j, data2Last+j);		

		double corr = Ubitrack::Calibration::computeCorrelation(data1, data2);
		
				
		//LOG4CPP_INFO(logger, "estimateTimeDelay:"<< j << ":" << corr << " : " << output.at<float>(0,0) << " : " << corr - output.at<float>(0,0));
		/*
		std::stringstream ss1;
		ss1 << "d:\\temp\\corrdata\\tmpdata\\corr1_" << j << ".txt";			

		std::stringstream ss2;
		ss2 << "d:\\temp\\corrdata\\tmpdata\\corr2_" << j << ".txt";

			
		std::ofstream stream1(ss1.str());		
		std::ofstream stream2(ss2.str());		
						
		std::string linesep( "\n" );

		vec_iter tmp1 = data1First;			
			
		for(int i=0;i<data1.size();i++)	{
			stream1 << i;
			stream1 << ";";
			stream1 << data1[i];
			stream1 << ";";
			stream1 << corr;			
			stream1 << linesep;			
		}
		vec_iter tmp2 = data2First;
		
		for(int i=0;i<data2.size();i++)	{
			stream2 << i;
			stream2 << ";";
			stream2 << data2[i];
			stream2 << linesep;			
		}
		stream1.close();
		stream2.close();
		
		*/
		m_correlationData[j].push_back(corr);


	}

}


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	
	cf->registerComponent< TimeDelayEstimation > ( "TimeDelayEstimationPosition3D" );

}

} } // namespace Ubitrack::Components
