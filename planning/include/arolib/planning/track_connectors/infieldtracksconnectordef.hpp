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
 
#ifndef ARO_INFIELDTRACKSCONNECTORDEF_HPP
#define ARO_INFIELDTRACKSCONNECTORDEF_HPP

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

namespace arolib{

/**
 * @brief Default Infield-tracks connector
 */
class InfieldTracksConnectorDef : public IInfieldTracksConnector
{
public:
    /**
     * @brief Constructor.
     * @param parentLogger Parent logger
     */
    explicit InfieldTracksConnectorDef(std::shared_ptr<Logger> parentLogger = nullptr);

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
     * @brief Get an appropiate limit boundary for the subfield and a given turning radius based on the subfield's geometries.
     * @sa IInfieldTracksConnector::getExtendedLimitBoundary
     */
    virtual Polygon getExtendedLimitBoundary(const Subfield& sf,
                                             double turningRad) const override;

protected:

    /**
     * @brief Get a connection path between 2 poses (location + orientation) through a given set of tracks (linestrings).
     * @param pose_start Start pose
     * @param pose_end Target pose
     * @param turningRad turning radius
     * @param tracks Input tracks/linestrings
     * @param limitBoundary Overall (limit) boundary
     * @param infieldBoundary Inner-field boundary
     * @param maxConnectionLength Connection length limit
     * @return Connection path (empty on error)
     */
    virtual std::vector<Point> getConnectionOverTracks(const Pose2D& pose_start,
                                                       const Pose2D& pose_end,
                                                       double turningRad,
                                                       const std::vector<std::vector<Point> > &tracks,
                                                       const Polygon& limitBoundary,
                                                       const Polygon& infieldBoundary,
                                                       double maxConnectionLength) const;
    virtual std::vector<Point> getConnectionOverTracks_1_1(const Pose2D& pose_start,
                                                           const Pose2D& pose_end,
                                                           double turningRad,
                                                           const std::vector<std::vector<Point> > &tracks,
                                                           const Polygon& limitBoundary,
                                                           const Polygon& infieldBoundary,
                                                           double maxConnectionLength) const;
    virtual std::vector<Point> getConnectionOverTracks_1_2(const Pose2D& pose_start,
                                                           const Pose2D& pose_end,
                                                           double turningRad,
                                                           const std::vector<std::vector<Point> > &tracks,
                                                           const Polygon& limitBoundary,
                                                           const Polygon& infieldBoundary,
                                                           double maxConnectionLength) const;

    virtual std::vector<Point> getConnectionOverTracks_2(const Pose2D& pose_start,
                                                         const Pose2D& pose_end,
                                                         double turningRad,
                                                         const std::vector<std::vector<Point> > &tracks,
                                                         const Polygon& limitBoundary,
                                                         const Polygon& infieldBoundary,
                                                         double maxConnectionLength) const;

    std::map<double, std::vector<Point>> getConnectionOverTracks_adjustAndSortTracks(const Pose2D& pose_start,
                                                                                     const Pose2D& pose_end,
                                                                                     double turningRad,
                                                                                     const std::vector<std::vector<Point> > &tracks) const;

    /**
     * @brief Get a connection path between 2 poses (location + orientation) through a complete (surrouding) headland.
     * @param hl Headland
     * @param pose_start Start pose
     * @param pose_end Target pose
     * @param turningRad turning radius
     * @param limitBoundary Overall (limit) boundary
     * @param infieldBoundary Inner-field boundary
     * @param maxConnectionLength Connection length limit
     * @return Connection path (empty on error)
     */
    virtual std::vector<Point> getConnectionOverHeadland( const CompleteHeadland& hl,
                                                          const Pose2D& pose_start,
                                                          const Pose2D& pose_end,
                                                          double turningRad,
                                                          const Polygon& limitBoundary,
                                                          const Polygon& infieldBoundary,
                                                          double maxConnectionLength) const;

    /**
     * @brief Get a connection path between 2 poses (location + orientation) through a partial/side headlands.
     * @param hl Headlands
     * @param pose_start Start pose
     * @param pose_end Target pose
     * @param machine Machine
     * @param turningRad turning radius
     * @param limitBoundary Overall (limit) boundary
     * @param infieldBoundary Inner-field boundary
     * @param maxConnectionLength Connection length limit
     * @return Connection path (empty on error)
     */
    virtual std::vector<Point> getConnectionOverHeadland(const std::vector<PartialHeadland>& hl,
                                                         const Pose2D& pose_start,
                                                         const Pose2D& pose_end,
                                                         const Machine machine,
                                                         double turningRad,
                                                         const Polygon& limitBoundary,
                                                         const Polygon& infieldBoundary,
                                                         double maxConnectionLength) const;

    /**
     * @brief Get a connection path between 2 poses (location + orientation) over the area outside a boundary.
     * @param boundary Boundary
     * @param pose_start Start pose
     * @param pose_end Target pose
     * @param turningRad turning radius
     * @param limitBoundary Overall (limit) boundary
     * @param infieldBoundary Inner-field boundary
     * @param maxConnectionLength Connection length limit
     * @return Connection path (empty on error)
     */
    virtual std::vector<Point> getConnectionOutsideBoundary(Polygon boundary,
                                                             const Pose2D& pose_start,
                                                             const Pose2D& pose_end,
                                                             double turningRad,
                                                             const Polygon& limitBoundary,
                                                             const Polygon& infieldBoundary,
                                                             double maxConnectionLength) const;

    /**
     * @brief Get a connection path between 2 poses (location + orientation) over a given partial/side headland (with possible turning in the adjacent partial headlands).
     *
     * If the machine needs to go to the end of a track of the given partial headland and make a 'U' turn to return over the partial headland, that turn must be done in the corresponding adjacent headland.
     *
     * @param hls All partial headlands
     * @param hl Partial headland used for the connection
     * @param pose_end Target pose
     * @param turningRad turning radius
     * @param limitBoundary Overall (limit) boundary
     * @param infieldBoundary Inner-field boundary
     * @param maxConnectionLength Connection length limit
     * @return Connection path (empty on error)
     */
    virtual std::vector<Point> getConnectionOverSamePartialHeadland(const std::vector<PartialHeadland> &hls,
                                                                    const PartialHeadland& hl,
                                                                    const Pose2D& pose_start,
                                                                    const Pose2D& pose_end,
                                                                    double turningRad,
                                                                    const Polygon& limitBoundary,
                                                                    const Polygon& infieldBoundary,
                                                                    double maxConnectionLength) const;

    /**
     * @brief Get a dubins connection path between 2 poses (location + orientation) over a given partial headland.
     * @param hl Partial headland used for the connection
     * @param pose_end Target pose
     * @param turningRad turning radius
     * @param limitBoundary Overall (limit) boundary
     * @param infieldBoundary Inner-field boundary
     * @param maxConnectionLength Connection length limit
     * @param [out] pathLength Resulting path length
     * @return Connection path (empty on error)
     */
    virtual std::vector<Point> getConnectionOverSamePartialHeadlandWithDubins(const PartialHeadland& hl,
                                                                              const Pose2D& pose_start,
                                                                              const Pose2D& pose_end,
                                                                              double turningRad,
                                                                              const Polygon& limitBoundary,
                                                                              const Polygon& infieldBoundary,
                                                                              double maxConnectionLength,
                                                                              double &pathLength) const;

    /**
     * @brief Get a connection path between 2 poses (location + orientation) through partial/side headlands.
     * @param hls Partial/side headlands
     * @param pose_start Start pose
     * @param pose_end Target pose
     * @param machine Machine
     * @param turningRad turning radius
     * @param limitBoundary Overall (limit) boundary
     * @param infieldBoundary Inner-field boundary
     * @param maxConnectionLength Connection length limit
     * @return Connection path (empty on error)
     */
    virtual std::vector<Point> getConnectionOverAdjacentHeadlands(const std::vector<PartialHeadland>& hls,
                                                                  size_t hl_ind_start,
                                                                  size_t hl_ind_end,
                                                                  const Pose2D& pose_start,
                                                                  const Pose2D& pose_end,
                                                                  double turningRad,
                                                                  const Polygon& limitBoundary,
                                                                  const Polygon& infieldBoundary,
                                                                  double maxConnectionLength) const;

    /**
     * @brief Get a set of (sub) tracks extracted from the given set of closed tracks that will be used to connect the two given poses.
     * @param tracks_in Input closed tracks
     * @param pose_start Start pose
     * @param pose_end Target pose
     * @return Set of (sub) tracks' points
     */
    static std::vector<std::vector<Point>> getSubTracksFromClosedTracks(const std::vector<Track>& tracks_in,
                                                                        const Pose2D& pose_start, const Pose2D& pose_end);

    /**
     * @brief Get a set of (sub) tracks extracted from the given set of closed tracks that will be used to connect the two given poses.
     * @param tracks_in Input closed tracks' points
     * @param pose_start Start pose
     * @param pose_end Target pose
     * @return Set of (sub) tracks' points
     */
    static std::vector<std::vector<Point>> getSubTracksFromClosedTracks(const std::vector<std::vector<Point> > &tracks_in,
                                                                        const Pose2D& pose_start, const Pose2D& pose_end);

    /**
     * @brief Get a set of (sub) tracks extracted from the given set of closed tracks that will be used to connect the two given poses.
     * @param tracks_in Input closed tracks' points
     * @param pose_start Start pose
     * @param pose_end Target pose
     * @return Set of (sub) tracks' points
     */
    static std::vector<std::vector<Point>> getSubTracksFromClosedTracks(const std::vector<const std::vector<Point> *> &tracks_in,
                                                                        const Pose2D& pose_start, const Pose2D& pose_end);

    /**
     * @brief Get a set of (sub) tracks extracted from tracks of the given partial/side headlands, where the tracks of adjacent headlands are connected following the given headland sequence.
     * @param hls Partial/side headlands
     * @param sequence Headland sequence
     * @param [out] tracks Set of resulting (sub) tracks' points
     * @param turningRad turning radius
     * @param maxConnectionLength Connection length limit
     */
    static void getTracksFromHeadlandSequence(const std::vector<PartialHeadland>& hls,
                                               const std::vector<size_t>& sequence,
                                               std::vector<std::vector<Point>>& tracks, double turningRad, double maxConnectionLength);

    /**
     * @brief Get a set of (sub) tracks extracted from tracks of the given partial/side headlands, where the tracks of adjacent headlands are connected following the given headland sequences.
     * @param hls Partial/side headlands
     * @param connectionSequences Headland connection sequences
     * @param [out] tracks Set of resulting (sub) tracks' points
     * @param turningRad turning radius
     * @param maxConnectionLength Connection length limit
     */
    static void getTracksFromHeadlandSequences(const std::vector<PartialHeadland>& hls,
                                                const std::vector<std::vector<size_t>>& connectionSequences,
                                                std::vector<std::vector<Point>>& tracks, double turningRad, double maxConnectionLength);

    /**
     * @brief Sort the headlands following the given headland sequence.
     * @param hls Partial/side headlands
     * @param sequence Headland sequence
     * @return Sorted partial/side headlands
     */
    static std::vector<PartialHeadland> sortHeadlandsFollowingSequence(const std::vector<PartialHeadland>& hls,
                                                                        const std::vector<size_t>& sequence);


};

}

#endif // ARO_INFIELDTRACKSCONNECTORDEF_HPP
