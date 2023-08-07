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
 
#ifndef ARO_INFIELDTRACKSCONNECTORDUBINS_HPP
#define ARO_INFIELDTRACKSCONNECTORDUBINS_HPP

#include <ctime>

#include "arolib/misc/loggingcomponent.h"
#include "arolib/types/pose2D.hpp"
#include "arolib/types/subfield.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/curves_helper.hpp"
#include "arolib/geometry/pathsmoother.hpp"
#include "arolib/planning/track_connectors/infieldtracksconnector.hpp"

namespace arolib{

/**
 * @brief Simple Infield-tracks connector using Dubins paths
 */
class InfieldTracksConnectorDubins : public IInfieldTracksConnector
{
public:
    struct ExtensionParameters{
        double deltaAng0 = -1, deltaDist0 = -1, deltaAngn = -1, deltaDistn = -1;
        size_t distPasses = 2;
        size_t angPasses = 1; //each pass corresponds to 2 computations: +deltaAng and -deltaAng
        static ExtensionParameters NoExtention();
    };

    /**
     * @brief Constructor.
     * @param parentLogger Parent logger
     */
    explicit InfieldTracksConnectorDubins(std::shared_ptr<Logger> parentLogger = nullptr);

    /**
     * @brief Get the connection path between two poses.
     * @sa IInfieldTracksConnector::getConnection
     */
    virtual std::vector<Point> getConnection(const Machine& machine,
                                             const Pose2D& _pose_start,
                                             const Pose2D& _pose_end,
                                             double turningRad = -1,
                                             const std::pair<double, double>& extraDist = std::make_pair(-1, -1),
                                             const Polygon& _limitBoundary = Polygon(),
                                             const Polygon& infieldBoundary = Polygon(),
                                             const Headlands& headlands = Headlands(),
                                             double speed_start = -1,
                                             double maxConnectionLength = -1) const override;

    /**
     * @brief Set the onlyShortest flag.
     * @param onlyShortest If true, only the shortest dubbins path will be checked; otherwise, all 6 options will be checked
     */
    void setOnlyShortest(bool onlyShortest);

    /**
     * @brief Get a connection path between 2 poses (location + orientation) using dubins path
     * @param pose_start Start pose
     * @param pose_end Target pose
     * @param turningRad turning radius
     * @param tracks Input tracks/linestrings
     * @param limitBoundary Overall (limit) boundary
     * @param infieldBoundary Inner-field boundary
     * @param maxConnectionLength Connection length limit
     * @param testAll If false, only the shortest Dubins path will be tested
     * @param extParams Extension parameters for extra checks (if null -> default parameters)
     * @param [out] length Output path length (optional)
     * @return Connection path (empty on error)
     */
    static std::vector<Point> getDubinsPathConnection(const Pose2D& pose_start,
                                                      const Pose2D& pose_end,
                                                      double turningRad,
                                                      const std::vector<const Polygon *> &limitBoundaries,
                                                      const Polygon& infieldBoundary,
                                                      double maxConnectionLength,
                                                      bool testAll,
                                                      const ExtensionParameters* extParams,
                                                      double* length = nullptr);

    /**
     * @brief Get the default extension parameters used for extra checks
     * @param turningRad turning radius
     * @return Default extension parameters
     */
    static ExtensionParameters getDefaultExtensionParameters(double turningRad);

protected:
    bool m_onlyShortest = false;  /**< If true, only the shortest dubbins path will be checked; otherwise, all 6 options will be checked */

};

}

#endif // ARO_INFIELDTRACKSCONNECTORDUBINS_HPP
