/*
 * Copyright 2023  DFKI GmbH
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License
*/
 
#include "arolib/components/gpssimulator.h"

namespace arolib {

std::once_flag flag_do_once;

GPSSimulator::GPSSimulator(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_stopSim(true),
    m_updateFrequency(1),
    m_timescale(1)
{
}

GPSSimulator::~GPSSimulator()
{
    //@todo ugly solution to be sure that the sim thread finishes before destroying the object
    m_stopSim = true;
    m_updateFrequency = 1e-5;
    int count = 0;
    while(m_threadRunning && count++ < 5)
        usleep(20);
}

bool GPSSimulator::setMachineIds(const std::vector<Machine> &machines){
    if( isRunning() )
        return false;
    m_machineIds.clear();
    for(auto &m : machines)
        m_machineIds.insert(m.id);
    return true;
}

bool GPSSimulator::setMachineIds(const std::vector<int> &machineIds){
    if( isRunning() )
        return false;
    for(auto &id : machineIds)
        m_machineIds.insert(id);
    return true;
}

bool GPSSimulator::setPlan(const std::vector<Route> &plan){
    if( isRunning() )
        return false;
    m_plan = plan;
    return true;
}

bool GPSSimulator::setUpdateFrequency(float updateFrequency){

    if(updateFrequency <= 0){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Disregarding invalid updateFrequency " + std::to_string(updateFrequency) );
        return false;
    }
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_updateFrequency = updateFrequency;
    }

    logger().printOut(LogLevel::INFO, __FUNCTION__, "Update frequency set to " + std::to_string(m_updateFrequency) );
    return true;
}

bool GPSSimulator::setTimeScale(float timeScale){
//    if(timeScale == 0){
//        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Disregarding invalid timeScale " + std::to_string(timeScale) );
//        return false;
//    }
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_timescale = timeScale;
    }

    logger().printOut(LogLevel::INFO, __FUNCTION__, "Time scale set to " + std::to_string(m_timescale) );
    return true;
}

bool GPSSimulator::setDistanceDeviation(float dev)
{
    m_distanceDeviation = dev;

    logger().printOut(LogLevel::INFO, __FUNCTION__, "DistanceDeviation set to " + std::to_string(m_distanceDeviation) );
    return true;
}

AroResp GPSSimulator::start(handlePointFunction sendPoint)
{
    if (isRunning()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Sim already running." );
        return AroResp(1, "Sim already running." );
    }

    if (m_plan.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Current plan has no routes." );
        return AroResp(1, "Current plan has no routes." );
    }
    logger().printOut(LogLevel::INFO, __FUNCTION__, std::string("Starting simulation with the following parameters:\n" +
                                                       std::string("   Update frequency : ") + std::to_string(m_updateFrequency) +
                                                       std::string("   Time scale       : ") + std::to_string(m_timescale) + "\n" ) );
    m_stopSim = false;  

    //@todo doest't look very safe if the GPSSimulator instance is destroyed while the thread is running
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_threadRunning = true;
        m_threadId = getRandomId();
    }
    std::thread sim(&GPSSimulator::startSimulation, this, sendPoint, m_threadId, m_logger);
    sim.detach();

    return AroResp(0, "OK" );
}

void GPSSimulator::stop(){

    std::lock_guard<std::mutex> guard(m_mutex);
    m_stopSim = true;
}

bool GPSSimulator::isRunning()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    return !m_stopSim && m_threadRunning;
}



void GPSSimulator::startSimulation(handlePointFunction sendPoint, int threadId, Logger logger)
{
    logger.printOut(LogLevel::INFO, __FUNCTION__, "Start GPS simulation..." );
    logger.printOut(LogLevel::INFO, __FUNCTION__, "Machines: " + std::to_string(m_machineIds.size()) + "  ;  " +
                                           "Plan    : " + std::to_string(m_plan.size()) );

    // simulate for all machines:
    bool planStillRunning;

    double currTime = 0.;

    double startTime, nowTime, deltaTime = 0;
    startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    std::vector<int> refInexes(m_plan.size(), -1);

    struct SampleData{
        RoutePoint rp_plan;
        Point rp_sim;
        double bearing;
    };

    std::map<MachineId_t, SampleData> prevSamples; /**< Previous points and orientations */

    do {
        bool stopSim = true;
        float updateFrequency = 1, timescale = 1, distanceDeviation;
        int threadId2;
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            stopSim = m_stopSim;
            updateFrequency = m_updateFrequency;
            timescale = m_timescale;
            distanceDeviation = m_distanceDeviation;
            threadId2 = m_threadId;
        }

        if (threadId != threadId2){ //another thread was started -> kill this one
            logger.printOut(LogLevel::WARNING, __FUNCTION__, "TheadId missmatch: another thread was started -> kill this one" );
            return;
        }

        if (stopSim) {  // stop the simulation if the stop command has been received
            planStillRunning = false;
            logger.printOut(LogLevel::INFO, __FUNCTION__, "Stopping simulation..." );
            break;
        }

        if (fabs(m_timescale-1) < 0.0001)
            logger.printOut(LogLevel::INFO, __FUNCTION__, "Current time: " + std::to_string(currTime) );
        else
            logger.printOut(LogLevel::INFO, __FUNCTION__, "Current time: " + std::to_string(currTime) + " (scaled by " + std::to_string(m_timescale) + ")" );

        planStillRunning = false;
        for (size_t i = 0 ; i < m_plan.size() ; ++i) {

            auto& route = m_plan.at(i);
            if( m_machineIds.find(route.machine_id) == m_machineIds.end() )//don't simulate for this machine
                continue;

            if(route.route_points.empty())
                continue;
            bool routeFinished = (currTime > route.route_points.back().time_stamp);

            SampleData sampleData;
            SimData data;
            data.machineId = route.machine_id;

            if(refInexes.at(i) < 0)
                sampleData.rp_plan = route.calcPoint(currTime, &refInexes.at(i));
            else
                sampleData.rp_plan = route.calcPoint2(currTime, refInexes.at(i));
            sampleData.rp_sim = sampleData.rp_plan.point();

            if(distanceDeviation > 1e-9){
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_real_distribution<> dis_ang(0.0, 2*M_PI);
                std::uniform_real_distribution<> dis_dist(0.0, distanceDeviation);

                sampleData.rp_sim.x += dis_dist(gen);
                sampleData.rp_sim.point() = geometry::rotate(sampleData.rp_plan, sampleData.rp_sim, dis_ang(gen), false);
            }

            auto it_mp = prevSamples.find(route.machine_id);
            if(it_mp != prevSamples.end()){
                SampleData & sampleDataPrev = it_mp->second;
                double dist = geometry::calc_dist(sampleDataPrev.rp_plan, sampleData.rp_plan);

                if(dist < 1e-9 || std::fabs(deltaTime) < 1e-9)
                    sampleData = sampleDataPrev;
                else{
                    dist = geometry::calc_dist(sampleDataPrev.rp_sim, sampleData.rp_sim);
                    if(dist < 1e-9 || std::fabs(deltaTime) < 1e-9)
                        sampleData.bearing = sampleDataPrev.bearing;
                    else{
                        data.speed = dist / deltaTime;
                        Point p0_geo, p1_geo;
                        CoordTransformer::GetInstance().convert_to_geodetic(sampleDataPrev.rp_sim, p0_geo);
                        CoordTransformer::GetInstance().convert_to_geodetic(sampleData.rp_sim, p1_geo);
                        sampleData.bearing = Point::bearing(p0_geo, p1_geo);
                    }
                }
            }
            data.routeInd = i;

            data.routePoint = sampleData.rp_plan;
            data.routePoint.point() = sampleData.rp_sim.point();
            data.orientation = sampleData.bearing;

            prevSamples[route.machine_id] = sampleData;

            data.refPointInd = refInexes.at(i);
            if(data.refPointInd + 1 < route.route_points.size()){
                double ts0 = route.route_points.at(data.refPointInd).time_stamp;
                double ts1 = route.route_points.at(data.refPointInd+1).time_stamp;
                if( std::fabs(currTime - ts0)  > std::fabs(currTime - ts1) )
                    ++data.refPointInd;
            }

            {
                std::lock_guard<std::mutex> guard(m_mutex);
                if(m_stopSim){
                    logger.printOut(LogLevel::INFO, __FUNCTION__, "Simulation thread stopped." );
                    m_threadRunning = false;
                    return;
                }
            }

            sendPoint(data);

            planStillRunning = planStillRunning || !routeFinished;
        }

        std::this_thread::sleep_for(std::chrono::microseconds( (int)(1e6/updateFrequency) )); // send the GPS pose every x seconds
        nowTime   = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        deltaTime = (nowTime - startTime)/1000.0;
        currTime += deltaTime * timescale;

        startTime = nowTime; // -> set the start time for the next iteration
    } while (planStillRunning);

    logger.printOut(LogLevel::INFO, __FUNCTION__, "Simulation finished." );

    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_threadRunning = false;
    }
}

int GPSSimulator::getRandomId()
{
    std::call_once( flag_do_once, [](){ srand( std::time(NULL) ); } );
    return rand();
}

}
