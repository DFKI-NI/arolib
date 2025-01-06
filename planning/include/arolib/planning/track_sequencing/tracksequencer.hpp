/*
 * Copyright 2021-2025 DFKI GmbH
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

#include <unordered_map>
#include <mutex>

#include "arolib/misc/basic_responses.h"
#include "arolib/geometry/pathsmapmanager.hpp"
#include "arolib/planning/track_connectors/infieldtracksconnector.hpp"

namespace arolib {

  /**
   * @brief Generates the track sequences that the harvesters must follow to harvest the inner-field based on the sequence strategy
   */
  class ITrackSequencer : public LoggingComponent
  {
  public:

      struct TrackSequencerSettings{
          bool limitStartToExtremaTracks = true; /**< Is the selection of the starting track limited to a track located at an extrema? */
          bool useMachineTurningRad = true; /**< Should the machine turning radius be used in the computations? */
          bool considerFieldExit = true; /**< Should the transit to exit the field be considered? */
          double maxSequencePlanningTime = -1; /**< Maximum planning time [s] (disregarded if <= 0) */

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
      using Sequences_t = std::map<MachineId_t, std::vector<ITrackSequencer::TrackInfo>>;

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
                                       Sequences_t& sequences,
                                       const std::map<MachineId_t, Pose2D>& initRefPoses = {},
                                       const std::set<size_t>& excludeTrackIndexes = {}) = 0;


      /**
       * @brief Set the Infield TracksConnector to be used.
       * @param connector Infield TracksConnector to be used.
       */
      virtual void setInfieldTrackConnector(std::shared_ptr<IInfieldTracksConnector> connector);

      /**
       * @brief Set the paths map manager to save and reuse the computed paths' between two poses and a specific turning radius.
       * @param pmm Paths map manager. If nullptr -> it will set a brand new manager
       */
      virtual void setPathsMapManager(geometry::PathsMapManagerPtr_t pmm);

      /**
       * @brief Get the paths map manager used to save and reuse the computed paths' between two poses and a specific turning radius.
       * @return Paths map manager (!= nullptr)
       */
      virtual geometry::PathsMapManagerPtr_t getPathsMapManager();

      /**
       * @brief Get the paths map manager used to save and reuse the computed paths' between two poses and a specific turning radius.
       * @return Paths map manager (!= nullptr)
       */
      virtual geometry::PathsMapManagerConstPtr_t getPathsMapManager() const;

      /**
       * @brief Set the flag stating if all computed paths must be saved in paths map (if applicable)
       * @param saveThem True/false
       */
      virtual void setSaveAllComputedPaths(bool saveThem);

      /**
       * @brief Get the flag stating if all computed paths must be saved in paths map (if applicable)
       * @return True/false
       */
      virtual bool getSaveAllComputedPaths() const;

      /**
       * @brief Set the flag stating if the connecting paths must be saved in paths map (if applicable)
       * @param saveThem True/false
       */
      virtual void setSaveConnectingPaths(bool saveThem);

      /**
       * @brief Get the flag stating if the connecting paths must be saved in paths map (if applicable)
       * @return True/false
       */
      virtual bool getSaveConnectingPaths() const;

      /**
       * @brief Remove selected tracks from sequences
       * @param sequences Original sequences
       * @param trackInds Track indexes to be removed/kept
       * @param removeGivenTracks If true, the trackInds will correspond to tracks to be removed; otherwise, to tracks to be kept.
       * @return Updated sequences
       */
      static Sequences_t removeTracksFromSequence(const Sequences_t& sequences,
                                                  const std::set<size_t>& trackInds,
                                                  bool removeGivenTracks);

  protected:
      /**
       * @brief Constructor.
       * @param childName Child class
       * @param logLevel Log level
       */
      explicit ITrackSequencer(const std::string &childName, const LogLevel &logLevel = LogLevel::INFO);

  protected:
      std::shared_ptr<IInfieldTracksConnector> m_tracksConnector = nullptr; /**< Infield tracks' connector */
      geometry::PathsMapManagerPtr_t m_pathsMapManager = std::make_shared<geometry::PathsMapManager>(); /**< holds internally generated and saved paths */
      bool m_saveAllComputedPaths = false; /**< Flag stating if all computed paths must be saved in m_pathsMap */
      bool m_saveConnectingPaths = true; /**< Flag stating if the connecting paths must be saved in m_pathsMap */
  };

}
#endif // _AROLIB_TRACKSEQUENCER_HPP
