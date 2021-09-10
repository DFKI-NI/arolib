/*
 * Copyright 2021  DFKI GmbH
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
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Disregarding invalid updateFrequency " + std::to_string(updateFrequency) );
        return false;
    }
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_updateFrequency = updateFrequency;
    }

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Update frequency set to " + std::to_string(m_updateFrequency) );
    return true;
}

bool GPSSimulator::setTimeScale(float timeScale){
//    if(timeScale == 0){
//        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Disregarding invalid timeScale " + std::to_string(timeScale) );
//        return false;
//    }
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_timescale = timeScale;
    }

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Time scale set to " + std::to_string(m_timescale) );
    return true;
}

AroResp GPSSimulator::start(handlePointFunction sendPoint)
{
    if (isRunning()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Sim already running." );
        return AroResp(1, "Sim already running." );
    }

    if (m_plan.empty()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Current plan has no routes." );
        return AroResp(1, "Current plan has no routes." );
    }
    m_logger.printOut(LogLevel::INFO, __FUNCTION__, std::string("Starting simulation with the following parameters:\n" +
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
    return !m_stopSim && !m_threadRunning;
}



void GPSSimulator::startSimulation(handlePointFunction sendPoint, int threadId, Logger logger)
{
    logger.printOut(LogLevel::INFO, __FUNCTION__, "Start GPS simulation..." );
    logger.printOut(LogLevel::INFO, __FUNCTION__, "Machines: " + std::to_string(m_machineIds.size()) + "  ;  " +
                                           "Plan    : " + std::to_string(m_plan.size()) );

    // simulate for all machines:
    bool planStillRunning;

    double currTime = 0.;

    double startTime, nowTime;
    startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    std::vector<int> refInexes(m_plan.size(), -1);

    do {
        bool stopSim = true;
        float updateFrequency = 1, timescale = 1;
        int threadId2;
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            stopSim = m_stopSim;
            updateFrequency = m_updateFrequency;
            timescale = m_timescale;
            threadId2 = m_threadId;
        }

        if (threadId != threadId2){ //another thread was started -> kill this one
            logger.printOut(LogLevel::WARNING, __FUNCTION__, "TheadId missmatch: another thread was started -> kill this one" );
            return;
        }

        if (m_stopSim) {  // stop the simulation if the stop command has been received
            planStillRunning = false;
            logger.printOut(LogLevel::INFO, __FUNCTION__, "Stoping simulation..." );
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


            SimData data;
            data.machineId = route.machine_id;

            if(refInexes.at(i) < 0)
                data.routePoint = route.calcPoint(currTime, &refInexes.at(i));
            else
                data.routePoint = route.calcPoint2(currTime, refInexes.at(i));

            data.speed = route.calcSpeed(currTime, refInexes.at(i));
            data.orientation = route.calcBearing(currTime, refInexes.at(i));
            data.routeInd = i;

            data.refPointInd = refInexes.at(i);
            if(data.refPointInd + 1 < route.route_points.size()){
                double ts0 = route.route_points.at(data.refPointInd).time_stamp;
                double ts1 = route.route_points.at(data.refPointInd+1).time_stamp;
                if( std::fabs(currTime - ts0)  > std::fabs(currTime - ts1) )
                    ++data.refPointInd;
            }

            sendPoint(data);


            planStillRunning = planStillRunning || !routeFinished;
        }

        std::this_thread::sleep_for(std::chrono::microseconds( (int)(1e6/updateFrequency) )); // send the GPS pose every x seconds
        nowTime   = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        currTime += (nowTime - startTime)/1000.0 * timescale;

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
