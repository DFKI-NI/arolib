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
 
#ifndef _AROLIB_TRACKSEQUENCERSIMANNEALING_HPP
#define _AROLIB_TRACKSEQUENCERSIMANNEALING_HPP

#include <functional>

#include "arolib/planning/track_sequencing/tracksequencer.hpp"
#include "arolib/planning/track_sequencing/simulated_annealing/simulated_annealing.hpp"

namespace arolib {

  /**
   * @brief Generates the track sequences that the primary machine must follow to cover the inner-field.
   */
  class TrackSequencerSimAnnealing : virtual public ITrackSequencer
  {
  public:

      using SavePathFunction = std::function< void(const Pose2D&, const Pose2D&, double, const PointVec &) >; /**<  */

      /**
       * @brief Holds the track information of a state.
       */
      struct StateTrackInfo{
          int trackInd = -1; /**< Track index (if -1 -> initial location; not a real track) */
          double trackCost = 0; /**< Cost of driving the track */
          double connectionCost = 0; /**< Cost of the connection to the next track */
          bool trackInPointsDirection = false; /**< Flag stating if the track is to be driven in the direction (order) of its points or reversed */
          bool trackInRelativeDirection = false; /**< Flag stating if track-points' direction (order) corresponds to the overall reference direction (in case some tracks start e.g. with the northern point and others with the southern point) */
          Pose2D connPoseStart = Pose2D( Point::invalidPoint() ); /**< Starting pose of the connection to the next track (if invalid -> no further connection) */
          Pose2D connPoseEnd = Pose2D( Point::invalidPoint() ); /**< End pose of the connection to the next track (if invalid -> no further connection) */
          PointVec connectionPath{}; /**< Connecting path to the next track */
      };

      using State = std::map<MachineId_t, std::vector<StateTrackInfo>>; /**< State: <machine Id, <tracks in order of visit> > */

      /**
       * @brief Class used to compute the overall cost of a state.
       */
      class StateCostCalculator{
      public:

          /**
           * @brief Check if the state cost depends on the paths connecting the tracks.
           * @return True if the state cost depends on the paths connecting the tracks.
           */
          virtual bool dependsOnConnectingPaths() const = 0;

          /**
           * @brief Compute the state cost.
           * @param state State.
           * @param prevState Previous state (if null -> no previous state).
           * @return State cost.
           */
          virtual double calc(const State& state, const State* prevState) const = 0;
      };

      /**
       * @brief Abstract class / interface used to compute cost of the connections between tracks (or between the initial location and the first track).
       */
      class ConnectionCostCalculator{
      public:

          /**
           * @brief Check if the connection cost depends on the path.
           * @return True if the connection cost depends on the path.
           */
          virtual bool dependsOnPath() const = 0;


          /**
           * @brief Check if the connection cost depends on the machine.
           * @return True if the connection cost depends on the machine.
           */
          virtual bool dependsOnMachine() const = 0;

          /**
           * @brief Compute the cost of a path connecting two tracks (or the initial location with the first track).
           * @param path Path.
           * @param machine Machine.
           * @return Connection cost.
           */
          virtual double calc(const PointVec& path, const Machine& machine) const = 0;
      };

      /**
       * @brief ConnectionCostCalculator where the cost corresponds to the length of the connecting path.
       */
      class CCCConnectionLength : public ConnectionCostCalculator{
      public:

          /**
           * @brief Check if the connection cost depends on the path.
           * @sa ConnectionCostCalculator::dependsOnPath
           */
          constexpr virtual bool dependsOnPath() const noexcept override final {return true;}

          /**
           * @brief Check if the connection cost depends on the machine.
           * @sa ConnectionCostCalculator::dependsOnMachine
           */
          constexpr virtual bool dependsOnMachine() const noexcept override final {return false;}

          /**
           * @brief Compute the cost (i.e., length) of a path connecting two tracks (or the initial location with the first track).
           * @param path Path.
           * @param machine Machine (disregarded)
           * @return Connection cost.
           */
          virtual double calc(const PointVec& path, const Machine&) const override final;
      };

      /**
       * @brief Abstract class / interface used to compute cost of driving over a track.
       */
      class TrackCostCalculator{
      public:

          /**
           * @brief Check if the track cost depends on the direction the track is driven.
           * @return True if the track cost depends on the direction the track is driven.
           */
          virtual bool dependsOnTrackDirection() const = 0;

          /**
           * @brief Check if the track cost depends on the machine.
           * @return True if the track cost depends on the machine.
           */
          virtual bool dependsOnMachine() const = 0;

          /**
           * @brief Compute the cost of driving over a track.
           * @param track Track.
           * @param trackInPointsDirection Flag stating if the track is to be driven in the track-points direction (order) of reversed.
           * @param machine Machine.
           * @return Track cost.
           */
          virtual double calc(const Track& track, bool trackInPointsDirection, const Machine& machine) const = 0;
      };

      /**
       * @brief TrackCostCalculator with no track costs.
       */
      class NoTrackCostCalculator : public TrackCostCalculator{
      public:
          /**
           * @brief Check if the track cost depends on the direction the track is driven.
           * @sa TrackCostCalculator::dependsOnTrackDirection.
           */
          constexpr virtual bool dependsOnTrackDirection() const noexcept override final {return false;}

          /**
           * @brief Check if the track cost depends on the machine.
           * @sa TrackCostCalculator::dependsOnMachine.
           */
          constexpr virtual bool dependsOnMachine() const noexcept override final {return false;}

          /**
           * @brief Compute the cost of driving over a track.
           * @sa TrackCostCalculator::calc.
           */
          constexpr virtual double calc(const Track& , bool , const Machine& ) const noexcept override final {return 0;}
      };

      /**
       * @brief Abstract class / interface used to compute paths connecting two tracks (or the initial location with the first track).
       */
      class ITracksConnector{
      public:

          /**
           * @brief Holds the connection information.
           */
          struct ConnectionInfo{

              /**
               * @brief Constructor.
               * @param _machine Machine.
               * @param _trackStateInd Index of the track state (w.r.t. the overall state of the machine).
               */
              ConnectionInfo(const Machine& _machine, size_t _trackStateInd): machine(_machine), trackStateInd(_trackStateInd){}

              const Machine& machine; /**< Machine */
              const size_t trackStateInd; /**< Index of the track state (w.r.t. the overall state of the machine) */
          };

          /**
           * @brief Holds the computed connection path.
           */
          struct Connection{

              /**
               * @brief Constructor.
               * @param _savePath Flag stating if the path must be saved.
               */
              Connection(bool _savePath): savePath(_savePath) {}

              PointVec path; /**< Connection path */
              bool savePath; /**< Flag stating if the path must be saved */
          };

          /**
           * @brief Holds the computed connection path to a field exit.
           */
          struct ExitConnection : public Connection{

              /**
               * @brief Constructor.
               * @param _savePath Flag stating if the path must be saved.
               */
              ExitConnection(bool _savePath): Connection(_savePath) {}
              Pose2D endPose; /**< End-pose of the connection (i.e., exit pose) */
          };

          using GetPrecomputedPathFtn = const std::function<PointVec (const Pose2D& /*pose1*/, const Pose2D& /*pose2*/, const Machine& /*machine*/, bool /*checkBidirectional*/)>; /**< Function used to obtain precomputed paths for reuse */


          /**
           * @brief Check if the connection path depends on the overall tracks' sequence.
           * @return True if the connection path depends on the overall tracks' sequence.
           */
          constexpr virtual bool dependsOnTrackSequence() const noexcept = 0;

          /**
           * @brief Get the tracks' connecting paths for a given state.
           * @param state State.
           * @param prevState Previous state (if null -> no previous state).
           * @param subfield Subfield.
           * @param connectionsInfo Hold the information of the connections to be computed.
           * @param getPrecomputedPath Function used to obtain precomputed paths for reuse.
           * @return Computed connections corresponding to the requests in connectionsInfo.
           */
          virtual std::vector<Connection> getTrackStateConnections( const State& state,
                                                                    const State* statePrev,
                                                                    const Subfield& subfield,
                                                                    const std::vector<ConnectionInfo>& connectionsInfo,
                                                                    const GetPrecomputedPathFtn& getPrecomputedPath ) const = 0;


          /**
           * @brief Get the path connecting two given poses (standard connection).
           * @param poseFrom Start pose.
           * @param poseTo End pose.
           * @param subfield Subfield.
           * @param machine Machine.
           * @return Computed connecting path (empty on error).
           */
          virtual std::vector<Point> getStandardConnection( const Pose2D& poseFrom,
                                                            const Pose2D& poseTo,
                                                            const Subfield& subfield,
                                                            const Machine& machine ) const = 0;

          /**
           * @brief Get the possible exit paths for a given state.
           * @param state State.
           * @param prevState Previous state (if null -> no previous state).
           * @param subfield Subfield.
           * @param machine Machine.
           * @param fieldExitPoses Available field exit poses.
           * @return Computed connecting path (empty on error).
           * @param getPrecomputedPath Function used to obtain precomputed paths for reuse.
           * @return Computed exit connections.
           */
          virtual std::vector<ExitConnection> getExitConnections( const State& state,
                                                                  const State* statePrev,
                                                                  const Subfield& subfield,
                                                                  const Machine& machine,
                                                                  const std::map<size_t, Pose2D>& fieldExitPoses,
                                                                  const GetPrecomputedPathFtn& getPrecomputedPath ) const = 0;
      };

      /**
       * @brief Default TracksConnector.
       */
      class TracksConnectorDef : public ITracksConnector{
      public:

          /**
           * @brief Constructor.
           * @param connector Infield tracks' connector.
           */
          TracksConnectorDef(std::shared_ptr<const IInfieldTracksConnector> connector);

          /**
           * @brief Check if the connection path depends on the overall tracks' sequence.
           * @sa ITracksConnector::dependsOnTrackSequence.
           */
          constexpr virtual bool dependsOnTrackSequence() const noexcept override final { return false; }

          /**
           * @brief Get the tracks' connecting paths for a given state.
           * @sa ITracksConnector::getTrackStateConnections.
           */
          virtual std::vector<Connection> getTrackStateConnections( const State& state,
                                                                    const State* statePrev,
                                                                    const Subfield& subfield,
                                                                    const std::vector<ConnectionInfo>& connectionsInfo,
                                                                    const GetPrecomputedPathFtn& getPrecomputedPath ) const override;


          /**
           * @brief Get the path connecting two given poses (standard connection).
           * @sa ITracksConnector::getStandardConnection.
           */
          virtual std::vector<Point> getStandardConnection(const Pose2D& poseFrom,
                                                            const Pose2D& poseTo,
                                                            const Subfield& subfield,
                                                            const Machine& machine ) const override;


          /**
           * @brief Get the possible exit paths for a given state.
           * @sa ITracksConnector::getExitConnections.
           */
          virtual std::vector<ExitConnection> getExitConnections( const State& state,
                                                                  const State* /*statePrev*/,
                                                                  const Subfield& subfield,
                                                                  const Machine& machine,
                                                                  const std::map<size_t, Pose2D>& fieldExitPoses, // < access point index , pose >
                                                                  const GetPrecomputedPathFtn& getPrecomputedPath ) const override;

      protected:
          std::shared_ptr<const IInfieldTracksConnector> m_connector; /**< Infield tracks' connector */
      };

      /**
       * @brief Holds the planning parameters for the TrackSequencerSimAnnealing.
       */
      struct PlanningParameters{
          friend class TrackSequencerSimAnnealing;

          const Subfield* const subfield; /**< Subfield */
          const ITrackSequencer* const sequencer; /**< Track sequencer */
          //const std::shared_ptr<IInfieldTracksConnector> connector; /**< Tracks connector */
          const std::shared_ptr<const ITracksConnector> connector; /**< Tracks connector */
          const std::map<MachineId_t, Machine>* const machines; /**< Machines map: <machine id, machine> */
          const ITrackSequencer::TrackSequencerSettings* const settings; /**< Track sequencer settings */
          const std::map<MachineId_t, Pose2D>* const initRefPoses; /**< Initial reference poses of the machines: <machine id, pose> */
          const std::set<size_t>* const fixedTracks; /**< Tracks fixed in the sequence */
          const std::shared_ptr<const StateCostCalculator> stateCostCalculator; /**< Used to compute the cost of a sequence/state */
          const std::shared_ptr<const ConnectionCostCalculator> connectionCostCalculator; /**< Used to compute the cost of a path connecting tracks (iif stateCostCalculator = nullptr) */
          const std::shared_ptr<const TrackCostCalculator> trackCostCalculator; /**< Used to compute the cost of driving over a track (iif stateCostCalculator = nullptr) */
          const std::shared_ptr<const std::set<size_t>> firstTrackSet; /**< If not empty, holds indexes of the the (only) tracks that can be used as first track (if nullptr, they will be automatically computed from the settings and subfield) */
          const std::map<size_t, Pose2D> fieldExitPoses; /**< Holds the poses of the field exit points, projected to the field boundary ( < access point index , pose > ) */

      private:

          /**
           * @brief Constructor.
           * @param _subfield Subfield.
           * @param _sequencer Track sequencer.
           * @param _connector Tracks connector.
           * @param _machines Machines map: <machine id, machine>.
           * @param _settings Track sequencer settings.
           * @param _initRefPoses Initial reference poses of the machines: <machine id, pose>.
           * @param _fixedTracks Tracks fixed in the sequence.
           * @param _stateCostCalculator Used to compute the cost of a sequence/state.
           * @param _connectionCostCalculator Used to compute the cost of a path connecting tracks (iif stateCostCalculator = nullptr).
           * @param _trackCostCalculator Used to compute the cost of driving over a track (iif stateCostCalculator = nullptr).
           * @param _firstTrackSet If not empty, holds indexes of the the (only) tracks that can be used as first track (if nullptr, they will be automatically computed from the settings and subfield).
           * @param _fieldExitPoses Holds the poses of the field exit points, projected to the field boundary: < access point index , pose >.
           */
          PlanningParameters(const Subfield &_subfield,
                             const ITrackSequencer& _sequencer,
                             //std::shared_ptr<const IInfieldTracksConnector> _connector,
                             std::shared_ptr<const ITracksConnector> _connector,
                             const std::map<MachineId_t, Machine>& _machines,
                             const ITrackSequencer::TrackSequencerSettings& _settings,
                             const std::map<MachineId_t, Pose2D>& _initRefPoses,
                             const std::set<size_t>& _fixedTracks,
                             std::shared_ptr<const StateCostCalculator> _stateCostCalculator,
                             std::shared_ptr<const ConnectionCostCalculator> _connectionCostCalculator,
                             std::shared_ptr<const TrackCostCalculator> _trackCostCalculator,
                             std::shared_ptr<const std::set<size_t>> _firstTrackSet,
                             const std::map<size_t, Pose2D>& _fieldExitPoses);

          /**
           *
           */
          static std::shared_ptr<const std::set<size_t>> _getFirstTrackSet(const Subfield &subfield,
                                                                           const ITrackSequencer::TrackSequencerSettings& settings,
                                                                           std::shared_ptr<const std::set<size_t>> _firstTrackSet);
      };


      /**
       * @brief State generator for TrackSequencerSimAnnealing.
       */
      class StateGenerator : public SimulatedAnnealing<State>::IStateGenerator{
      public:
          using ResultState = SimulatedAnnealing<State>::ResultState;

          /**
           * @brief Initialize the state generator.
           * @param params Planning parameters.
           * @param computeConnectionPath Flag stating if connection paths are to me computed.
           * @param savePathFct Function to save computed paths.
           * @param maxAttemptsNewState Maximum attempts to create a new state.
           */
          void init(PlanningParameters& params, bool computeConnectionPath, SavePathFunction savePathFct, size_t maxAttemptsNewState = 10000);

          /**
           * @brief Get a new state and the respective energy
           * @param currentState Current state
           * @param currentEnergy Current energy
           * @return New state and respective energy (nullptr on failure)
           */
          virtual ResultState getState(const State& currentState, double currentEnergy) override{
              if(!m_params || currentState.empty())
                  return nullptr;
              return _getState(currentState, currentEnergy);
          }

          /**
           * @brief Check if a state was already visited
           * @param state State
           * @return True if visited
           */
          bool isStateVisited (const State& state) const;

          /**
           * @brief Add a state to the visited states
           * @param state State
           * @return False if the state already existed
           */
          bool addStateToVisited(const State& state);

          /**
           * @brief Get the visited states
           * @return Visited states
           */
          const std::set<std::string> visitedStates () const;

          /**
           * @brief Clear the saved visited states
           */
          void clearVisitedStates();

      private:

          /**
           * @brief Get a new state and the respective energy
           * @param currentState Current state
           * @param currentEnergy Current energy
           * @return New state and respective energy (nullptr on failure)
           */
          virtual ResultState _getState(const State& currentState, double currentEnergy) = 0;

          /**
           * @brief Get the identifier (key) of a given state
           * @param state State
           * @return Identifier (key) of the given state
           */
          static std::string toStateStr(const State& state);

          std::set<std::string> m_visitedStates; /**< Hold the identifiers/key of the states tht have been visited so far */
          const PlanningParameters* m_params = nullptr; /**< Planning parameters */


      protected:
          /**
           * @brief Get the planning parameters
           * @return Planning parameters
           */
          const PlanningParameters* params() const;


          bool m_computeConnectionPath; /**< Flag stating if connection paths are to me computed */
          SavePathFunction m_savePathFct; /**< Function to save computed paths */
          size_t m_maxAttemptsNewState; /**< Maximum attempts to create a new state */
      };


      //@todo add StateGeneratorInvertDirectionOfWorkingWindows based on mass and machine capacity (add mass in track to TrackState?)

      /**
       * @brief Constructor
       * @param base Sequencer used to obtain the starting state
       * @param stateGenerator Used to generate new states
       * @param connector Connector used to obtain the paths to the tracks
       * @param stateCostCalculator Used to compute the cost of a complete state/sequence
       * @param connectionCostCalculator Used to compute the cost of a path connecting tracks
       * @param trackCostCalculator Used to compute the cost of driving over a track
       * @param logLevel Log level
       */
      explicit TrackSequencerSimAnnealing(std::unique_ptr<ITrackSequencer> base = nullptr,
                                          std::shared_ptr<StateGenerator> stateGenerator = nullptr,
                                          std::shared_ptr<ITracksConnector> connector = nullptr,
                                          std::shared_ptr<StateCostCalculator> stateCostCalculator = nullptr,
                                          std::shared_ptr<ConnectionCostCalculator> connectionCostCalculator = nullptr,
                                          std::shared_ptr<TrackCostCalculator> trackCostCalculator = nullptr,
                                          LogLevel logLevel = LogLevel::INFO);


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
                                       const std::set<size_t>& excludeTrackIndexes = {}) override;


      /**
       * @brief Set the start temperature for simulated annealing
       * @param startTemp Start temperature
       * @return True on success
       */
      bool setStartTemperature(float startTemp);

      /**
       * @brief Get the start temperature for simulated annealing
       * @return Start temperature
       */
      inline float getStartTemperature() const { return m_startTemp; }

      /**
       * @brief Set the temperature coefficient for simulated annealing
       * @param k Temperature coefficient
       * @return True on success
       */
      bool setTemperatureCoefficient(float k);

      /**
       * @brief Get the temperature coefficient for simulated annealing
       * @return Temperature coefficient
       */
      inline float getTemperatureCoefficient() const { return m_kTemp; }

      /**
       * @brief Set the number of iterations for simulated annealing
       * @param n Number of iterations
       * @return True on success
       */
      bool setIterations(size_t n);

      /**
       * @brief Get the number of iterations for simulated annealing
       * @return Number of iterations
       */
      inline size_t getIterations() const { return m_iterations; }

      /**
       * @brief Set the number of sub-iterations for simulated annealing
       *
       * For each iteration, it will make n number of sub-iterations and select the one with the lowest cost)
       * @param n Number of sub-iterations
       * @return True on success
       */
      bool setSubIterations(size_t n);

      /**
       * @brief Get the number of sub-iterations for simulated annealing
       * @return Number of sub-iterations
       */
      inline size_t getSubIterations() const { return m_subIterations; }

      /**
       * @brief Set the flag stating whether the connection paths computed by the base sequencer are to be used
       * @param use If true, it will use the connection paths computed by the base sequencer
       */
      void setUseBaseConnections(bool use);

      /**
       * @brief Get the flag stating whether the connection paths computed by the base sequencer are to be used
       * @return Flag stating whether the connection paths computed by the base sequencer are to be used
       */
      inline bool getUseBaseConnections() const { return m_useBaseConnections; }


  private:

      /**
       * @brief (Internal) Adds a connecting path to the internal paths DB/manager
       * @param pose1 Start pose.
       * @param pose2 End pose.
       * @param turningRad Turning radius.
       * @param path Path.
       */
      inline auto addPathToMapInternal(const Pose2D& pose1, const Pose2D& pose2, double turningRad, const PointVec &path){
          return m_pathsMapManager->addPathToMap(pose1, pose2, turningRad, path);
      }

  protected:

      /**
       * @brief Get the poses of the field exit points, projected to the field boundary and pointing towars outside of the boundary
       * @param subfield.
       * @return Poses of the field exit points, projected to the field boundary: < access point index , pose >
      */
      static std::map<size_t, Pose2D> getFieldExitPoses(const Subfield &subfield);

  protected:
      std::shared_ptr<StateCostCalculator> m_stateCostCalculator; /**< Used to compute the cost of a complete state/sequence */
      std::shared_ptr<StateGenerator> m_stateGenerator; /**< Used to generate new states */
      std::shared_ptr<ConnectionCostCalculator> m_connectionCostCalculator; /**< Used to compute the cost of a path connecting tracks */
      std::shared_ptr<TrackCostCalculator> m_trackCostCalculator; /**< Used to compute the cost of driving over a track */
      std::unique_ptr<ITrackSequencer> m_base; /**< Sequencer used to obtain the starting state */
      //std::shared_ptr<IInfieldTracksConnector> m_connector; /**< Connector used to obtain the paths to the tracks */
      std::shared_ptr<ITracksConnector> m_connector; /**< Connector used to obtain the paths to the tracks */
      //std::unique_ptr<Sequences_t> m_lastSequences = nullptr; /**< Last computed sequences */
      float m_startTemp = 10; /**< Start temperature */
      float m_kTemp = 0.1; /**< Temperature coefficient */
      size_t m_iterations = 2000; //500; // 200; /**< Number of iterations */
      size_t m_subIterations = 5; //500; // 200; /**< Number of sub-iterations (per iteration) */
      bool m_tryWithSimpleSequencer = true; /**< If true, it will obtain the base sequences with a simple sequencer and use it iif the cost is lower than the one obtained with m_base */
      bool m_useBaseConnections = true; /**< If true, it will use the connection paths computed by the base sequencer */
      //bool m_useLastComputedSequenceAsBase = true; /**< If true, it will use the last computed sequences as base (iif available) */
  };

}
#endif // _AROLIB_TRACKSEQUENCERSIMANNEALING_HPP
