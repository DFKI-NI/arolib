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
 
#ifndef ARO_INFIELDTRACKSCONNECTORDUBINSORGRAPHBASED_HPP
#define ARO_INFIELDTRACKSCONNECTORDUBINSORGRAPHBASED_HPP

#include <ctime>

#include "arolib/misc/loggingcomponent.h"
#include "arolib/types/pose2D.hpp"
#include "arolib/types/subfield.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/curves_helper.hpp"
#include "arolib/geometry/pathsmoother.hpp"
#include "arolib/planning/track_connectors/infieldtracksconnector.hpp"
#include "arolib/planning/track_connectors/infieldtracksconnectordubins.hpp"
#include "arolib/planning/track_connectors/infieldtracksconnectorgraphbased.hpp"

namespace arolib{

/**
 * @brief Infield-tracks connector that uses a simple Dubins connector, and if it fails, uses a graph-based connector
 */
class InfieldTracksConnectorDubinsOrGraphBased : public InfieldTracksConnectorGraphBased
{
public:
    /**
     * @brief Constructor.
     * @param parentLogger Parent logger
     */
    explicit InfieldTracksConnectorDubinsOrGraphBased(std::shared_ptr<Logger> parentLogger = nullptr);

    /**
     * @brief Get the connection path between two poses.
     * @sa IInfieldTracksConnector::getConnection
     */
    virtual std::vector<Point> getConnection(const Machine& machine,
                                             const Pose2D& _pose_start,
                                             const Pose2D& _pose_end,
                                             double _turningRad = -1,
                                             const std::pair<double, double>& extraDist = std::make_pair(-1, -1),
                                             const Polygon& _limitBoundary = Polygon(),
                                             const Polygon& infieldBoundary = Polygon(),
                                             const Headlands& headlands = Headlands(),
                                             double speed_start = -1,
                                             double maxConnectionLength = -1) const override;


};

}

#endif // ARO_INFIELDTRACKSCONNECTORDUBINSORGRAPHBASED_HPP
