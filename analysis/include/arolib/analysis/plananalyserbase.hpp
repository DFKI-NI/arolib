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
 
#ifndef AROLIB_PLAN_ANALYSER_BASE_H
#define AROLIB_PLAN_ANALYSER_BASE_H

#include <map>
#include <fstream>

#include "arolib/misc/loggingcomponent.h"
#include "arolib/types/field.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/types/route.hpp"
#include "arolib/geometry/geometry_helper.hpp"

namespace arolib {

/**
 * @brief Base class to analyze plans of vehicles
 */
class PlanAnalyserBase : public LoggingComponent
{
public:

    /**
     * @brief Index range
     */
    struct IndexRange{
        int min = -1;
        int max = -1;
        IndexRange() = default;
        IndexRange(int _min, int _max);
    };

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit PlanAnalyserBase(const LogLevel &logLevel = LogLevel::INFO, const std::string& child = "");

    /**
     * @brief Set the (general) data used for analytics and comparison
     * @param field Field
     * @return True on success
     */
    virtual bool setData(const Field &field);

    /**
     * @brief Add a couple of route to be compared (between each other)
     * @param subfieldId Id of the corresponding subfield of the plan
     * @param machineId Id of the machine driving the routes
     * @param route1 Route 1 to be analized
     * @param route1 Route 2 to be analized
     * @return True on success
     */
    virtual bool addRoute(size_t subfieldId, const MachineId_t &machineId, const Route &route1, const Route &route2);

    /**
     * @brief Run analytics on the set data
     * @return True on success
     */
    virtual bool runAnalysis() = 0;

    /**
     * @brief Remove all routes (set in previous @see addRoute calls) from internal data base
     */
    virtual void clearRoutes();

    /**
     * @brief Remove the routes from internal data base corresponding to a specified machine (from all subfields)
     * @param machineId Id of the machine whose routes will be removed
     */
    virtual void clearMachineRoutes(MachineId_t &machineId);

    /**
     * @brief Remove the routes from internal data base corresponding to a specified machine and a specified subfield
     * @param subfieldId Id of the subfield corresponding to the routes to be removed
     * @param machineId Id of the machine whose routes will be removed
     */
    virtual void clearMachineRoutes(size_t subfieldId, MachineId_t &machineId);

    /**
     * @brief Remove all the routes from internal data base corresponding to a specified subfield
     * @param subfieldId Id of the subfield corresponding to the routes to be removed
     */
    virtual void clearSubfieldRoutes(size_t subfieldId);

    /**
     * @brief Save the analytics results (after calling @see runAnalysis) in a CSV file
     * @param filename File name/path
     * @return True on success
     */
    virtual bool saveResults_CSV(const std::string &filename) const = 0;

protected:
    Field m_field;/**< Field */
    bool m_fieldReady = false;/**< Flag to know if the field was set successfully already */
    bool m_analysisReady = false;/**< Flag to know if an analysis was run successfully already and it is available */

    std::map<size_t, std::map<MachineId_t, std::pair<Route, Route> > > m_routes; /**< Added routes to be analyzed: map<subfield, map<MachineId_t, std::pair<Planned Route, Real Route> > > */

    const std::string sep = ";"; /**< Value-separator for CSV files */
};

}

#endif // AROLIB_PLAN_ANALYSER_BASE_H
