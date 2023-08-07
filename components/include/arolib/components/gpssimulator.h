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
 
#ifndef AROLIB_GPSSIMULATOR_H
#define AROLIB_GPSSIMULATOR_H

#include <fstream>
#include <vector>
#include <set>
#include <thread>
#include <functional>
#include <mutex>
#include <random>
#include <boost/tokenizer.hpp>

#include "arolib/types/machine.hpp"
#include "arolib/types/route.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/basic_responses.h"

namespace arolib {

/**
 * @brief Class used to simulate machine states (location, bunker mass, etc.) based on planned routes
 */
class GPSSimulator : public LoggingComponent
{
public:

    /**
     * @brief Struct holding simulation data
     */
    struct SimData{
        RoutePoint routePoint; /**< Route point */
        MachineId_t machineId = -1; /**< Machine ID */
        double orientation = 0.0; /**< Orientation */
        double speed = 0.0; /**< Speed */
        size_t routeInd = 0; /**< Route index */
        size_t refPointInd = 0; /**< Reference point index */
    };

    typedef std::function< void(const SimData&) > handlePointFunction; /**< Callback function to handle route points */

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit GPSSimulator(const LogLevel &logLevel = LogLevel::INFO);

    /**
     * @brief Destructor.
     */
    virtual ~GPSSimulator();

    /**
     * @brief Set the ids of the machines participating in the set plan.
     * @param machines Machines
     * @return True on success
     */
    bool setMachineIds(const std::vector<Machine>& machines);

    /**
     * @brief Set the ids of the machines participating in the set plan
     * @param machineIds Machine Ids
     * @return True on success
     */
    bool setMachineIds(const std::vector<int>& machineIds);

    /**
     * @brief Set the plan / routes to be simulated.
     * @param plan Plan / routes
     * @return True on success
     */
    bool setPlan(const std::vector<Route>& plan);

    /**
     * @brief Set the simulation update (sample) frequency
     * @param updateFrequency Frequency
     * @return True on success
     */
    bool setUpdateFrequency(float updateFrequency);

    /**
     * @brief Set the simulation timescale (related to the simulation speed)
     * @param timeScale Time scale
     * @return True on success
     */
    bool setTimeScale(float timeScale);

    /**
     * @brief Set the deviation [m] from the planned location
     * @param dev Deviation [m] (if <0 -> no deviation)
     * @return True on success
     */
    bool setDistanceDeviation(float dev);

    /**
     * @brief Start the simulation
     * @param sendPoint Callback function to handle the information of the currently simulated route point
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp start(handlePointFunction sendPoint);

    /**
     * @brief Stop the simulation
     */
    void stop();

    /**
     * @brief Check if the simulation is running
     * @return True if the simulation is running
     */
    bool isRunning();
    

protected:

    /**
     * @brief Start the simulation (internal)
     * @param sendPoint Callback function to handle the information of the currently simulated route point
     * @param threadId Thread Id
     * @param logger Logger
     */
    void startSimulation(handlePointFunction sendPoint, int threadId, Logger logger);

    /**
     * @brief Generate a random ID
     * @return Id
     */
    int getRandomId();

protected:
    bool m_stopSim; /**< Flag used to stop running simulations */
    bool m_threadRunning = false; /**< Flag used to know if the thread is running */
    int m_threadId; /**< Thread Id */
    float m_updateFrequency; /**< Simulation update (samplling) frequency */
    float m_timescale; /**< Actual simulation time scale value, computed from the frequency & timescale values */
    float m_distanceDeviation = -1; /**< Deviation from planned location */
    std::vector<Route>  m_plan; /**< Plan/routes */
    std::set<MachineId_t> m_machineIds; /**< Machine Ids */
    std::mutex m_mutex; /**< Mutex */
};

}

#endif // AROLIB_GPSSIMULATOR_H
