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
 
#ifndef AROLIB_VISIT_PERIOD_HPP
#define AROLIB_VISIT_PERIOD_HPP

#include "arolib/types/route_point.hpp"
#include "arolib/geometry/geometry_helper.hpp"

namespace arolib{


/**
 * @brief Class holding the information of the visit (drive over) of a machine
 */
class VisitPeriod{
public:
    /**
     * @brief Class holding delay information
     */
    struct DelayInfo{
        double time_from;/**< Time(-stamp) when the delay happens */
        double delay;/**< Delay duration*/
        bool onlyTimeOut = false;/**< The delay only affects the time_out of the visit?*/


        /**
         * @brief Comparison operator < (for sorting)
         * @param other Other
         * @return this < other
         */
        bool operator <(const DelayInfo& other) const;
    };

    MachineId_t machineId;/**< Id if the machine driving over the vertex */
    double time_in;/**< Time(-stamp) the machine enters the vertex 'area' */
    double time_out;/**< Time(-stamp) the machine leaves the vertex 'area' */
    double timestamp;/**< Timestamp of the routepoint corresponding to the visit (time_in <= timestamp <= time_out) */
    std::vector< Polygon > next_area;/**< vector holding the areas to be visited afterwards */

    /**
     * @brief Constructor
     * @param _machineId Id if the machine driving over the vertex
     * @param _time_in Time(-stamp) the machine enters the vertex 'area
     * @param _time_out Time(-stamp) the machine leaves the vertex 'area'
     * @param _timestamp Timestamp of the routepoint corresponding to the visit (time_in <= timestamp <= time_out)
     * @param _next_area vector holding the areas to be visited afterwards
     */
    explicit VisitPeriod(MachineId_t _machineId = -255,
                         double _time_in = 0,
                         double _time_out = -1,
                         double _timestamp = -1,
                         std::vector< Polygon > _next_area = {});

    /**
     * @brief Checks if the vertex is busy (not available) at a certain period of time
     * It also gives the time that the machine would have to wait until the vertex is available, a list of machines occupying this vertex at the given time (period), and whether any of the machines that is occupying this vertex at the given time (period) is planned to visit a given vertex from this vertex
     * The period to be checked is given by [time_in, time_in+duration]
     * @param map Map containing the visit periods of the vertex, where the key is the time (-stamp) at which the visit started (i.e. when the machine entered the vertex 'area')
     * @param time_in Timestamp corresponding to the start of the period to be checked
     * @param duration Duration of the period to be checked
     * @param maxMachines Maximum number of machines allowed to be in the vertex at the same time
     * @param [out] waiting Time that the machine would have to wait until the vertex is available (i.e. has space for the machine)
     * @param currentVt Vertex from which the machine wants to access the vertex under consideration
     * @param [out] towardsCurrentVt True if there is a machine located in the vertex under consideration (at any point during the give period [time_in, time_in+duration]) visiting currentVt (used to avoid machines driving over the same edge in opposite directions)
     * @param machineIds Ids of the machines located in (visiting) the vertex under consideration (at any point during the give period [time_in, time_in+duration])
     * @param excludedMachines Ids of the machines to be excluded from the check (the visit periods of these machines will be disregarded)
     * @param delays Delays' information
     * @return True if the vertex is busy (not available) at the given period of time
     */
    static bool isBusy(const std::vector<VisitPeriod> &visitPeriods,
                       double time_in,
                       double duration,
                       int maxMachines,
                       double &waiting,
                       Point currentLocation,
                       bool &towardsCurrentLocation,
                       std::vector<MachineId_t> &machineIds,
                       const std::set<MachineId_t> &excludedMachines = {},
                       const std::map<MachineId_t, std::set<DelayInfo>>& delays = {});


    /**
     * @brief Get the set of machines visiting in a given period of time
     * @param visitPeriods Visit periods to be checked
     * @param time_in Timestamp corresponding to the start of the period to be checked
     * @param duration Duration of the period to be checked
     * @param excludedMachines Ids of the machines to be excluded from the check (the visit periods of these machines will be disregarded)
     * @param delays Delays' information
     * @return Set of visiting machines
     */
    static std::set<MachineId_t> getVisitingMachines(const std::vector<VisitPeriod> &visitPeriods,
                                                     double time_in,
                                                     double duration,
                                                     const std::set<MachineId_t>& excludedMachines = {},
                                                     const std::map<MachineId_t, std::set<DelayInfo>>& delays = {});

    /**
     * @brief Check if a machine is visiting in a given period of time
     * @param visitPeriods Visit periods to be checked
     * @param machineId Id of the machine
     * @param time_in Timestamp corresponding to the start of the period to be checked
     * @param time_out Timestamp corresponding to the end of the period to be checked
     * @param delays Delays' information
     * @return True if the machine is visiting
     */
    static bool isMachineVisiting(const std::vector<VisitPeriod> &visitPeriods,
                                  MachineId_t machineId,
                                  double time_in,
                                  double time_out,
                                  const std::map<MachineId_t, std::set<DelayInfo> > &delays = {});

    /**
     * @brief Get the visit periods sorted
     * @param visitPeriods Input visit periods
     * @param delays Delays' information
     * @return Sorted visit periods
     */
    static std::multimap<double, VisitPeriod> getSortedVisitPeriods(const std::vector<VisitPeriod> &visitPeriods,
                                                                     const std::map<MachineId_t, std::set<DelayInfo> > &delays);

};

}

#endif // AROLIB_VISIT_PERIOD_HPP
