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
 
#ifndef AROLIB_BASEROUTESPROCESSOR_H
#define AROLIB_BASEROUTESPROCESSOR_H

#include <unistd.h>
#include <iostream>
#include <math.h>
#include <string>

#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/container_helper.h"
#include "arolib/geometry/field_geometry_processing.hpp"

#include "arolib/types/machine.hpp"

#include "arolib/planning/planningworkspace.h"
#include "arolib/planning/edgeMassCalculator.hpp"
#include "arolib/planning/infieldtracksconnector.hpp"
#include "arolib/misc/basic_responses.h"

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>


namespace arolib {

/**
 * @brief Class used to process the initial base routes (prior to planning); It takes the base-routes for IF and HL processing and combines (connects) them  
 */
class BaseRoutesProcessor : public LoggingComponent, protected PlanningWorkspaceAccessor
{
public:

    /**
     * @brief Settings
     */
    struct Settings{
        bool bePreciseWithMaps = true; /**< Perform map/grid operations precisely (except for remaining-area map) */
    };

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit BaseRoutesProcessor(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Processes the headland and inner-field base routes based on the remaining area map and connects them to generate a complete route for each machine.
     * @param subfield Processed subfield
     * @param baseRoutes_headland Headland base routes (initial ones)
     * @param baseRoutes_infield Inner-field base routes (initial ones)
     * @param machines Machines
     * @param settings Settings
     * @param [out] connectedRoutes Connected processed routes
     * @param [out] processedBaseRoutes_headland Headland processed base routes
     * @param [out] processedBaseRoutes_infield Inner-field processed base routes
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp processRoutes(const Subfield& subfield,
                                const std::vector<arolib::HeadlandRoute> &baseRoutes_headland,
                                const std::vector<arolib::Route> &baseRoutes_infield,
                                const std::vector<arolib::Machine>& machines,
                                const Settings& settings,
                                std::vector<arolib::Route> &connectedRoutes,
                                std::vector<HeadlandRoute> *pProcessedBaseRoutes_headland = nullptr,
                                std::vector<Route> *pProcessedBaseRoutes_infield = nullptr) const;


    /**
     * @brief Processes the headland and inner-field base routes based on the remaining area map and connects them to generate a complete route for each machine.
     * @param [in/out] pw Planning workspace containing the necessary data (subfield, initial base routes, etc.) for planning, as well as the resulting processed and connected routes.
     * @param subfieldIdx Index of the subfield (in pw) that will be processed
     * @param settings Settings
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp processRoutes(PlanningWorkspace &pw,
                                size_t subfieldIdx,
                                const Settings& settings) const;


protected:

    /**
     * @brief Pre-process the initial headland and inner-field base routes
     *
     * Sets the timestamp of the route points to -1 (the points in the route, starting by the firts one, until there is something to process), based on the remainin-area map
     * If there is nothing to process in a route, the route is removed
     * @param [in/out] baseRoutes_infield Inner-field base routes (before overload plan) to be processed.
     * @param [in/out] baseRoutes_headland Headland base routes (before overload plan) to be processed.
     */
    void preProcessRoutes(std::vector<arolib::Route> &baseRoutes_infield,
                          std::vector<arolib::HeadlandRoute> &baseRoutes_headland) const;

    /**
     * @brief Connect the (initial, pre-processed) headland and inner-field routes of each base
     *
     * The timestamps and other route-point parameters are adjusted correspondingly (also in baseRoutes_headland and baseRoutes_infield)
     * @param [in/out] baseRoutes_infield Inner-field base routes (before overload plan) to be processed.
     * @param [in/out] baseRoutes_headland Headland base routes (before overload plan) to be processed.
     * @param subfield Processed subfield
     * @param machines Machines used for planning
     * @return Connected and adjusted base routes
     */
    std::vector<Route> connectRoutes(std::vector<Route> &baseRoutes_infield,
                                     std::vector<HeadlandRoute> &baseRoutes_headland,
                                     const Subfield& subfield,
                                     const std::vector<Machine> &machines) const;


    /**
     * @brief Remove the initial points of the base routes where there is nothing to process
     * @param [in/out] baseRoutes base routes to be updated
     * @return Amount of poits removed from all routes
     */
    size_t removeInitialUselessPoints(std::vector<Route> &baseRoutes) const;

    /**
     * @brief Set the connector to connect the headland routes with the IF routes
     */
    void setHeadlandInfieldConnector(std::shared_ptr<IInfieldTracksConnector> tc);


protected:
    std::shared_ptr<IInfieldTracksConnector> m_tracksConnector = std::make_shared<InfieldTracksConnectorDef>(); /**< Infield-tracks connector */
    mutable PlanningWorkspace* m_planningWorkspace = nullptr;/**< Pointer to the planning workspace (if NULL, no methods with a planning workspace as parameter were called) */
    mutable size_t m_pw_subfieldIdx; /**< Index of the subfield to be planned (in case the planning workspace is being used */
};

}

#endif // AROLIB_BASEROUTESPROCESSOR_H
