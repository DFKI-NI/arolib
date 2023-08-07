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
 
#ifndef _AROLIB_TRACKSEQUENCERCLOSESTNEXT_HPP
#define _AROLIB_TRACKSEQUENCERCLOSESTNEXT_HPP

#include <map>
#include <future>
#include <mutex>

#include "arolib/planning/track_connectors/infieldtracksconnectordef.hpp"
#include "arolib/planning/track_sequencing/tracksequencer.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/field_geometry_processing.hpp"

namespace arolib {

  /**
   * @brief Generates the track sequences that the primary machine must follow to cover the inner-field.
   */
  class TrackSequencerClosestNext : virtual public ITrackSequencer
  {
  public:
      /**
       * @brief Constructor
       * @param logLevel Log level
       */
      explicit TrackSequencerClosestNext(LogLevel logLevel = LogLevel::INFO);


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
                                       const std::set<size_t>& excludeTrackIndexes = {}) override;


      /**
       * @brief Set the option to use the subfield boundary as a reference connection boundary
       * @param useConnOverBoundaryAsReference Flag stating if the subfield boundary is to be used as a reference connection boundary
       */
      void setUseConnOverBoundaryAsReference(bool useConnOverBoundaryAsReference);


  protected:

      /**
       * @brief Get the next track to be worked
       * @param subfield subfield
       * @param trackInds Vector holding the real indexes of the tracks used in the serach
       * @param assignedTrackIndexes Set of the track indexes already assigned
       * @param machine Machine
       * @param indPrevTrack Index of trackInds corresponding to the previous track
       * @param prevTrackInReverse Is the previous track in reverse?
       * @param sequence Tracks sequence for the machine
       * @param limitBoundary Limit boundary
       * @param connBoundary Connection boundary (disregarded if empty)
       * @param useMachineTurningRad Flag stating if the machine turning radius must be used in the search
       * @return Index of the next track, flag stating if the next track is in reverse
       */
      virtual std::pair<int, bool> getNextTrack(const Subfield &subfield,
                                                const std::vector<size_t>& trackInds,
                                                const std::set<size_t>& assignedTrackIndexes,
                                                const Machine& machine,
                                                size_t indPrevTrack,
                                                bool prevTrackInReverse,
                                                const std::vector<TrackInfo>& sequence,
                                                const Polygon& limitBoundary,
                                                const Polygon &connBoundary,
                                                bool useMachineTurningRad);

      /**
       * @brief Get the next track to be worked using multiple threads
       *
       *
       * @param subfield subfield
       * @param trackInds Vector holding the real indexes of the tracks used in the serach
       * @param assignedTrackIndexes Set of the track indexes already assigned
       * @param machine Machine
       * @param indPrevTrack Index of trackInds corresponding to the previous track
       * @param prevTrackInReverse Is the previous track in reverse?
       * @param sequence Tracks sequence for the machine
       * @param limitBoundary Limit boundary
       * @param connBoundary Connection boundary (disregarded if empty)
       * @param useMachineTurningRad Flag stating if the machine turning radius must be used in the search
       * @return Index of the next track, flag stating if the next track is in reverse
       */
      virtual std::pair<int, bool> getNextTrack2(const Subfield &subfield,
                                                const std::vector<size_t>& trackInds,
                                                const std::set<size_t>& assignedTrackIndexes,
                                                const Machine& machine,
                                                size_t indPrevTrack,
                                                bool prevTrackInReverse,
                                                const std::vector<TrackInfo>& sequence,
                                                const Polygon& limitBoundary,
                                                const Polygon &connBoundary,
                                                bool useMachineTurningRad);//with multiple threads... somehow it is slower :(


      /**
       * @brief Compute connection distances
       * @param connector Inner-field tracks connector
       * @param subfield subfield
       * @param machine machine
       * @param trackFrom Starting track
       * @param trackFromInReverse Point of the starting track in reverse?
       * @param trackNext Goal track
       * @param trackIndFrom Index of the starting track (index of trackInds)
       * @param maxDist Maximum distance allowed for the connection (disregarded if <= 0)
       * @param withTurningRad Flag stating if the turning radius is to be used to compute the connection path
       * @param checkSidesIndependently Flag stating if the track sides are to be checked independently
       * @param computeForTrackStart Flag stating if the connection to the track-start (first point) is to be considered
       * @param computeForTrackEnd Flag stating if the connection to the track-end (last point) is to be considered
       * @param [out] path0 Path to the track-start of trackNext
       * @param [out] pathn Path to the track-end of trackNext
       * @param [out] lengthToNext0 Length of the path to the track-start of trackNext
       * @param [out] lengthToNextn Length of the path to the track-end of trackNext
       * @return connection distance <distance, in reverse> (distance < 0 -> no connection)
       */
      virtual std::pair<double, bool> computeConnectionDistances(std::shared_ptr<IInfieldTracksConnector> connector,
                                                                 const Subfield &subfield,
                                                                 const Machine& machine, const Track &trackFrom, const Track &trackNext,
                                                                 bool trackFromInReverse,
                                                                 double maxDist,
                                                                 const Polygon &limitBoundary,
                                                                 bool withTurningRad,
                                                                 bool checkSidesIndependently,
                                                                 bool computeForTrackStart,
                                                                 bool computeForTrackEnd,
                                                                 PointVec &path0,
                                                                 PointVec &pathn,
                                                                 double &lengthToNext0,
                                                                 double &lengthToNextn);

      /**
       * @brief Check if the tracks are to be worked starting from the first track or the last based on the reference inital point
       * @param subfield subfield
       * @param initRefPoint Point used to select the first track in the sequence (disregarded if NULL or invalid)
       * @param trackInds Valid track indexes
       * @param numMachines Number of machines
       * @return First tracks per machine (index of trackInds, points direction)
       */
      std::vector<std::pair<size_t, TrackPointsDirection> > getFirstTracksIndexes(const Subfield& subfield,
                                                                                  const Point* initRefPoint,
                                                                                  const std::vector<size_t>& trackInds,
                                                                                  size_t numMachines);

      /**
       * @brief Get the average direction (point vector) between consecutive tracks in a track-sequence
       * @param subfield subfield
       * @param sequence Tracks sequence
       * @return Average direction (point vector)
       */
      Point getSequenceAverageDirection(const Subfield& subfield,
                                        const std::vector<TrackInfo>& sequence);

      /**
       * @brief Get the default inner-field tracks' connector
       * @return Default inner-field tracks' connector
       */
      std::shared_ptr<IInfieldTracksConnector> getDefTracksConnector() const;

  protected:
     bool m_useConnOverBoundaryAsReference = true; /**< Flag stating if the subfield boundary is to be used as a reference connection boundary */
  };

}
#endif // _AROLIB_TRACKSEQUENCERCLOSESTNEXT_HPP
