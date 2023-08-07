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
 
#ifndef _AROLIB_TRACKSEQUENCER_HPP
#define _AROLIB_TRACKSEQUENCER_HPP

#include <map>
#include <unordered_map>
#include <mutex>

#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/basic_responses.h"
#include "arolib/types/field.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/types/pose2D.hpp"
#include "arolib/planning/track_connectors/infieldtracksconnector.hpp"

namespace arolib {

  /**
   * @brief Generates the track sequences that the harvesters must follow to harvest the inner-field based on the sequence strategy
   */
  class ITrackSequencer : public LoggingComponent
  {
  public:
      //using PathsMap_t = std::unordered_map< Pose2D, std::unordered_map< Pose2D, PointVec, Pose2D::KeyHash >, Pose2D::KeyHash >;
      using PathsMap_t = std::map< Pose2D, std::map< Pose2D, std::map< int, PointVec >  > >;
      using PathsMapPtr_t = std::shared_ptr< PathsMap_t >;
      using PathsMapConstPtr_t = std::shared_ptr< const PathsMap_t >;

      struct TrackSequencerSettings{
          bool limitStartToExtremaTracks = true; /**< Is the selection of the starting track limited to a track located at an extrema? */
          bool useMachineTurningRad = true; /**< Should the machine turning radius be used in the computations? */

          /**
           * @brief Default constructor
           */
          TrackSequencerSettings() = default;

          /**
           * @brief Parse the parameters from a string map, starting from a default PlannerParameters
           * @param [out] param Parameters
           * @param map String map containing the parameter values
           * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
           * @return True on success
           */
          static bool parseFromStringMap( TrackSequencerSettings& params, const std::map<std::string, std::string>& map, bool strict = false);

          /**
           * @brief Parse the parameters to a string map
           * @param param Parameters
           * @return String map containing the parameter values
           */
          static std::map<std::string, std::string> parseToStringMap( const TrackSequencerSettings& params);
      };

      /**
       * @brief Track points' direction
       */
      enum TrackPointsDirection{
          UNDEF, /**< Undefined */
          FORWARD, /**< Forward direction */
          REVERSE /**< Reverse direction */
      };


      /**
       * @brief Track information
       */
      struct TrackInfo{
          size_t trackIndex; /**< Track index */
          TrackPointsDirection trackPointsDirection; /**< Track points' direction */

          /**
           * @brief Default constructor
           */
          TrackInfo() = default;

          /**
           * @brief Constructor
           * @param ind Track index
           * @param dir Track points' direction
           */
          TrackInfo(size_t ind, TrackPointsDirection dir);

      };

      /**
       * @brief Compute the sequences
       * @param subfield subfield
       * @param machines Working group
       * @param excludeTrackIndexes Indexes of the tracks that should be excluded
       * @param [out] sequences Sequences <machine_id, track_id sequence>
       * @param initRefPose Pose used to select the first track in the sequence (disregarded if NULL or invalid)
      * @return AroResp with error id (0:=OK) and message
       */
      virtual AroResp computeSequences(const Subfield &subfield,
                                       const std::vector<Machine>& machines,
                                       const TrackSequencerSettings& settings,
                                       std::map<MachineId_t, std::vector<ITrackSequencer::TrackInfo>>& sequences,
                                       const Pose2D* initRefPose = nullptr,
                                       const std::set<size_t>& excludeTrackIndexes = {}) = 0;


      /**
       * @brief Set the Infield TracksConnector to be used.
       * @param connector Infield TracksConnector to be used.
       */
      virtual void setInfieldTrackConnector(std::shared_ptr<IInfieldTracksConnector> connector);

      /**
       * @brief Set the map containing the computed paths' between two poses and a specific turning radius.
       * @return Paths' map
       */
      virtual PathsMapConstPtr_t getPathsMap() const;

      /**
       * @brief Get a computed path between two poses and a specific turning radius from the given paths' map.
       * @param map Paths' map.
       * @param pose1 Start pose.
       * @param pose2 End pose.
       * @param turningRad Turning radius.
       * @param checkBidirectional If true, it will also search for paths from pose2 to pose1 if no path from pose1 to pose2 was found.
       * @return Path (empty if not found)
       */
      static PointVec getPathFromMap(PathsMapConstPtr_t map, const Pose2D& pose1, const Pose2D& pose2, double turningRad, bool checkBidirectional = true);

      /**
       * @brief Get a computed path between two poses and a specific turning radius from the local paths' map.
       * @param pose1 Start pose.
       * @param pose2 End pose.
       * @param turningRad Turning radius.
       * @param checkBidirectional If true, it will also search for paths from pose2 to pose1 if no path from pose1 to pose2 was found.
       * @return Path (empty if not found)
       */
      PointVec getPathFromMap(const Pose2D& pose1, const Pose2D& pose2, double turningRad, bool checkBidirectional = true);

  protected:
      /**
       * @brief Constructor.
       * @param childName Child class
       * @param logLevel Log level
       */
      explicit ITrackSequencer(const std::string childName, const LogLevel &logLevel = LogLevel::INFO);

      /**
       * @brief Add a computed path between two poses and a specific turning radius to the local paths' map.
       * @param pose1 Start pose.
       * @param pose2 End pose.
       * @param turningRad Turning radius.
       * @param Path
       */
      void addPathToMap(const Pose2D& pose1, const Pose2D& pose2, double turningRad, const PointVec &path);

  protected:
      std::shared_ptr<IInfieldTracksConnector> m_tracksConnector = nullptr; /**< Infield tracks' connector */
      const PathsMapPtr_t m_pathsMap = std::make_shared<PathsMap_t>(); /**< holds internally generated and saved paths (managed by the childlen): <pose < pose, path > > */
      std::mutex m_mutex_pathsMap; /**< Mutex for operations in the local paths' map */
  };

}
#endif // _AROLIB_TRACKSEQUENCER_HPP
