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
 
#ifndef AROLIB_OVERLOADACTIVITIESPLANNER_H
#define AROLIB_OVERLOADACTIVITIESPLANNER_H

#include <ctime>

#include "arolib/types/route.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/planning/activitiesswitchingplanner.hpp"
#include "arolib/planning/olvPlan.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"

namespace arolib{

/**
 * @brief Class to plan overload activities
 */
class OverloadActivitiesPlanner : public LoggingComponent
{
public:

    /**
     * @brief Planner parameters/settings
     */
    struct PlannerSettings : virtual ASP_GeneralSettings{

        /**
         * @brief Parse the parameters from a string map, starting from a default PlannerSettings
         * @param [out] param Parameters
         * @param map String map containing the parameter values
         * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
         * @return True on success
         */
        static bool parseFromStringMap( PlannerSettings& params, const std::map<std::string, std::string>& map, bool strict = false);

        /**
         * @brief Parse the parameters to a string map
         * @param param Parameters
         * @return String map containing the parameter values
         */
        static std::map<std::string, std::string> parseToStringMap( const PlannerSettings& params);
    };

    /**
     * @brief Constructor
     * @param logLevel Log level
     */
    explicit OverloadActivitiesPlanner(LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Compute the overloading activities/windows based on the given switchingStrategy, harvester route, assigned OLVs, etc.
     * @param switchingStrategy Switching strategy
     * @param harvester_route Harvester route
     * @param overloadMachines OLVs assigned to the harvester route
     * @param machineCurrentStates Map containing the current states of the machines (inc. current location, bunker mass, etc.)
     * @param harvestedMassLimit If an overload activity surpaces this value, no more overload activities will be processed (i.e. overload activities will be computed for only a part of the harvester route). Disregarded if <= 0.
     * @param _logger (optional) Pointer to the parent logger (to use the parent's logLevel). If NULL, it creates a default logger.
     * @return The computed overload activities (ordered by time)
     */
    static std::vector<OLVPlan::OverloadInfo> computeOverloadActivities(PlannerSettings settings,
                                                                        const Route &harvester_route,
                                                                        const std::vector<Machine> &overloadMachines,
                                                                        const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                                        double harvestedMassLimit = -1,
                                                                        Logger *_logger = nullptr);

    /**
     * @brief Compute the overloading activities/windows following a INFIELD switchingStrategy based on the given harvester route, assigned OLVs, etc.
     * @sa OLVSwitchingStrategy
     * @param harvester_route Harvester route
     * @param _overloadMachines OLVs assigned to the harvester route
     * @param machineCurrentStates Map containing the current states of the machines (inc. current location, bunker mass, etc.)
     * @param harvestedMassLimit If an overload activity surpaces this value, no more overload activities will be processed (i.e. overload activities will be computed for only a part of the harvester route). Disregarded if <= 0.
     * @param _logger (optional) Pointer to the parent logger (to use the parent's logLevel). If NULL, it creates a default logger.
     * @return The computed overload activities (ordered by time)
     */
    static std::vector<OLVPlan::OverloadInfo> computeFieldOverloadPoints(const Route &harvester_route,
                                                                         const std::vector<Machine> &_overloadMachines,
                                                                         const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                                         double harvestedMassLimit = -1,
                                                                         Logger *_logger = nullptr);


    /**
     * @brief Compute the overloading activities/windows following a HEADLAND_ONLY switchingStrategy based on the given harvester route, assigned OLVs, etc.
     * @sa OLVSwitchingStrategy
     * @param harvester_route Harvester route
     * @param _overloadMachines OLVs assigned to the harvester route
     * @param machineCurrentStates Map containing the current states of the machines (inc. current location, bunker mass, etc.)
     * @param harvestedMassLimit If an overload activity surpaces this value, no more overload activities will be processed (i.e. overload activities will be computed for only a part of the harvester route). Disregarded if <= 0.
     * @param _logger (optional) Pointer to the parent logger (to use the parent's logLevel). If NULL, it creates a default logger.
     * @return The computed overload activities (ordered by time)
     */
    static std::vector<OLVPlan::OverloadInfo> computeTracksEndsOverloadPoints(const Route &harvester_route,
                                                                              std::vector<Machine> overloadMachines,
                                                                              const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                                              double harvestedMassLimit = -1,
                                                                              Logger *_logger = nullptr);

protected:
    /**
     * @brief Get the index of the next route-point with a given type
     * @param route Route
     * @param start Index from which to start checking
     * @param type Type to match
     * @return Index of the next route-point with the given type
     */
    static int run_to_next_type(const Route &route, int start, RoutePoint::RoutePointType type);

protected:

    static const double InitialOlvMaxCapacityMultiplier;/**< Multiplier (factor) used on the machine bunker mass to know if it should be considered as 'full' during the OL activities' computation */
};

}

#endif // AROLIB_OVERLOADACTIVITIESPLANNER_H
