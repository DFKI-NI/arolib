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
 
#ifndef ARO_POSEBASEDASTAR_HPP
#define ARO_POSEBASEDASTAR_HPP

#include <functional>

#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/curves_helper.hpp"
#include "arolib/misc/container_helper.h"
#include "arolib/misc/logger.h"
#include "arolib/misc/randomgeneration.hpp"
#include "arolib/planning/poseBasedGraph.hpp"
#include "arolib/planning/poseBasedCostCalculator.hpp"

namespace arolib{

/**
 * @brief Class used to generate machine (sub)routes/plans using A* search on a arolib graph
 */
class PoseBasedAstar : public LoggingComponent{
    using Cost_t = IPoseBasedCostCalculator::Cost_t;

public:
    using Vertex_t = PoseBasedGraph::Vertex_t;
    using VertexId_t = PoseBasedGraph::VertexId_t;
    using VertexFilterFct = PoseBasedGraph::VertexFilterFct;

    /**
     * @brief Plan parameters
     *
     * About max_time_visit: used to avoid that the machine drives over unharvested vertices (i.e. corresponding to harvesting route points with timestamps >= 0) when a harvesting delay is allowed (i.e. max_time_goal > overload-start timestamp). If a vertex has a (harvesting) timestamp higher that this value, the machine is not allowed to drive over the vertex even if there is a very high allowed harvester delay/waiting-time
     */
    struct PlanParameters{
        double start_time = -1; /**< Time(-stamp) at the start of the plan (i.e. time at which the machine is at the start vertex) */
        double max_time_visit = -1; /**< If a vertex corresponding to an (unharvested) harvesting route-point has a (harvesting) timestamp >= 0 higher than this value, this vertex will be excluded from the search. Disregarded if < 0 */
        double max_time_goal = -1;  /**< Maximum time(-stamp) in which the goal must be reached*/
        Machine machine; /**< Machine used for the search */
        double machine_speed = -1; /**< Speed of the machine */
        double initial_bunker_mass = -1; /**< Machine's initial bunker mass */
        bool overload; /**< Are we planning an overload route-segment? (at the moment always false) */
        std::unordered_set<VertexId_t> exclude = {}; /**< Set of vertices to be excluded from the search path */
        bool includeWaitInCost = true; /**< Should we include the time the machine has to wait to drive over an edge in the edge's cost calculation? */
        std::set<MachineId_t> restrictedMachineIds = {Machine::AllMachineIds};/**< Ids of the machines for which the max_time_visit applies */
        RoutePoint::RoutePointType defaultRPType = RoutePoint::DEFAULT;
        float searchRadius = -1; /**< Radius to search for valid nodes during expansion. If <= 0, the default based on the machine's turning radius is used */
        float noSearchRadius = -1; /**< Radius to avoid search for nodes during expansion (should be smaller that searchRadius). Disregarded if <= 0 */
        float probDirectSearch = 0.1; /**< Probability of checkin the connection to the goal_vt from the current search node [0,1] */
        VertexFilterFct vertexFilterFct = PoseBasedGraph::NoVertexFilterFct;/**< Function used to check if a (valid) vertex must be included (default to always true) */
    };

    /**
     * @brief Properties of a visited node
     */
    struct NodeInfo{
        Vertex_t vertex; /**< Current vertex */
        std::shared_ptr<const NodeInfo> predecessor = nullptr; /**< Predecessor node of the current node */
        Cost_t g; /**< G cost for the current node used for the search */
        Cost_t h; /**< H cost for the current node */
        double time; /**< Time(-stamp) of the current node */
        double waitTime; /**< Total amount of waiting time to reach this node (from the start node) */
        IPoseBasedCostCalculator::CostExtended_t::Path_t connection_path; /**< Connection with timestamps */
        size_t numCrossings = 0; /**< Total number of inner-field cross-edges used to achive the current node from the start node */
        size_t numCrossings_HL = 0; /**< Total number of headland-harvesting cross-edges used to achive the current node from the start node */

        /**
         * @brief Update data
         * @param predecessorNode Predecessor node info
         * @param costExt Cost
         */
        void updateFrom(std::shared_ptr<const NodeInfo> predecessorNode, const IPoseBasedCostCalculator::CostExtended_t& costExt );
    };

    using ClosedList_t = std::/*unordered_*/map<VertexId_t, std::shared_ptr<NodeInfo>>;

    /**
     * @brief Class holding the plan information
     */
    struct Plan{

        /**
         * @brief Constructor.
         */
        explicit Plan() = default;

        /**
         * @brief Adjust the FIELD_ENTRY/FIELD_EXIT route-point types
         *
         * The planned route might use interchangeably the entry/exit types (i.e. a FIELD_ENTRY might be actually a FIELD_EXIT and viceversa), so we assume that the last entry/exit route point is an EXIT or ENTRY point, and from then they must be ENTRY/EXIT/ENTRY/EXIT/etc or EXIT/ENTRY/EXIT/ENTRY/etc...
         * @param lastAccessIsExist Treat last access point as exit (if true) or entry (if false)
        */
        void adjustAccessPoints(bool lastAccessIsExit);

        bool valid = false; /**< Is the plan valid? */
        Cost_t cost = 0;/**< Total plan costs (i.e. the costs used for planning)*/
        double waitTime = 0;  /**< Total amount of waiting time in the plan */
        size_t numCrossings = 0; /**< Total number of inner-field cross-edges in the plan */
        size_t numCrossings_HL = 0; /**< Total number of headland-harvesting cross-edges in the plan */
        std::vector<RoutePoint> route_points; /**< Plan route points */
        ClosedList_t closed_list; /**< A* closed list */
        Vertex_t start_vt; /**< Start vertex */
        Vertex_t goal_vt; /**< Goal vertex */
        double searchRadius = -1; /**< Radius used to search for successor/next nodes */
        double noSearchRadius = -1;  /**< Nodes within this radius will not be taken into account when searching for successor/next nodes */

    };


    /**
     * @brief Setting sfor the A* search
     *
     * soil optimization - coefficient examples (soilOpt_biasCoef, Default_SoilOpt_SoilValueCoef, Default_SoilOpt_TimeCoef)
     *    (   1, 3, 0 ) : balances behaviour between overruns/edge and trying not to go over bad soil. the olv will rather wait to achieve this, yielding higher harvesting values
     *    (0.01, 3, 0 ) : will try to avoid at all costs passing over bad-soil areas, regardless the harvesting time and distance driven by the olvs, and allowing several overruns in the good-soil edges
     *    (   1, 3, 10) : gives higher importance on the time spent in the edges, hence prefering time-efficient paths over protecting the bad-soil areas or controlling overruns
     *
     * cross cost multiplier ranges
     *    Time optimization: ~25
     *    Soil optimization with priority on soil and overruns: ~3
     *    Soil optimization with priority on time: ~15
     */
    struct Settings{
        bool doFinalSearch = true; /**< If true, it will attempt a final search to get a more direct path to the goal */

        /**
         * @brief Constructor.
         */
        explicit Settings(bool _doFinalSearch = true);

        /**
         * @brief Parse the parameters from a string map, starting from a default Settings
         * @param [out] param Parameters
         * @param map String map containing the parameter values
         * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
         * @return True on success
         */
        static bool parseFromStringMap( Settings& params, const std::map<std::string, std::string>& map, bool strict = false);

        /**
         * @brief Parse the parameters to a string map
         * @param param Parameters
         * @return String map containing the parameter values
         */
        static std::map<std::string, std::string> parseToStringMap( const Settings& params);

    };

    /**
     * @brief Constructor
     * @param parameters Plan parameters/settings
     * @param settings Search parameters/settings
     * @param outputFolder Folder where the planning (search) information will be stored (if empty-string, no data will be saved)
     * @param _logger (optional) Pointer to the parent logger (to use the parent's logLevel). If NULL, it creates a default logger.
     */
    explicit PoseBasedAstar(const Settings & settings = Settings(),
                            Logger *_logger = nullptr);

    /**
     * @brief Destructor.
     *
     */
    virtual ~PoseBasedAstar(){}

    /**
     * @brief Plan route from a start pose inside a field to a goal pose inside a field
     * @param graph Graph used for the planning
     * @return True on success
     */
    bool plan(const PoseBasedGraph &graph,
              const Pose2D& start_pose,
              const Pose2D& goal_pose,
              const PlanParameters& params,
              std::shared_ptr<IPoseBasedCostCalculator> costCalculator);

    /**
     * @brief Plan route from a start pose inside the field to an existing goal vertex
     * @param graph Graph used for the planning
     * @return True on success
     */
    bool plan(const PoseBasedGraph &graph,
              const Pose2D& start_pose,
              const VertexId_t& goal_vt_id,
              const PlanParameters& params,
              std::shared_ptr<IPoseBasedCostCalculator> costCalculator);


    /**
     * @brief Plan route from an existing start vertex to a goal pose inside a field
     * @param graph Graph used for the planning
     * @return True on success
     */
    bool plan(const PoseBasedGraph &graph,
              const VertexId_t& start_vt_id,
              const Pose2D& goal_pose,
              const PlanParameters& params,
              std::shared_ptr<IPoseBasedCostCalculator> costCalculator);

    /**
     * @brief Plan route from an existing start vertex to an existing goal vertex
     * @param graph Graph used for the planning
     * @return True on success
     */
    bool plan(const PoseBasedGraph &graph,
              const VertexId_t& start_vt_id,
              const VertexId_t& goal_vt_id,
              const PlanParameters& params,
              std::shared_ptr<IPoseBasedCostCalculator> costCalculator);


    /**
     * @brief Check if the planning was successful and a plan exists
     * @return True on success
     */
    bool hasPlan() const { return m_result.valid; }


    /**
     * @brief Get the planning results
     * @return Planning results
     */
    const Plan& getPlan() const { return m_result; }


    /**
     * @brief Get the path to a given node from the start node
     * @param node Node
     * @param maxLength If >0, it will return the path starting from a predecessor node when this limit is reached
     * @param startAtNode If true, the first point of the path will correspond to the given node and not to the start/predecessor node
     * @return Path
     */
    static std::vector<Point> getPathToNode(std::shared_ptr<const NodeInfo> node, double maxLength = -1, bool startAtNode = true);


private:

    /**
     * @brief Struct holding the open list data
     */
    struct OpenList{
        std::unordered_map<VertexId_t, std::shared_ptr<NodeInfo>> nodes; /**< Nodes map */
        std::multimap<Cost_t, VertexId_t> sortedNodes; /**< Nodes' vertices sorted by cost */
    };

    /**
     * @brief Struct holding the search information
     */
    struct SearchInternalData{
        std::shared_ptr<NodeInfo> current_node = nullptr; /**< Current node */
        OpenList open_list; /**< Open list */
        ClosedList_t closed_list; /**< Closed list */
        std::unordered_set<VertexId_t> invalid_vts; /**< Invalid vertices */
        Vertex_t bestFieldExit; /**< Vertex of the best field exit */
    };


    /**
     * @brief Plan the path
     * @param graph Graph
     * @param start_vt Start vertex
     * @param goal_vt Goal vertex
     * @param params Plan parameters
     * @param costCalculator Cost calculator
     * @return True on success
     */
    bool plan(const PoseBasedGraph &graph,
              const Vertex_t& start_vt,
              const Vertex_t& goal_vt,
              const PlanParameters& params,
              std::shared_ptr<IPoseBasedCostCalculator> costCalculator);


    /**
     * @brief Expand a node
     * @param graph Graph
     * @param sd Current search data
     * @param goal_vt Goal vertex
     * @param params Plan parameters
     * @param costCalculator Cost calculator
     */
    void expandNode(const PoseBasedGraph &graph,
                    SearchInternalData& sd,
                    const Vertex_t &goal_vt,
                    const PlanParameters& params,
                    std::shared_ptr<IPoseBasedCostCalculator> costCalculator);

    /**
     * @brief Add a node to the open list
     * @param [in/out] open_list Open list
     * @param node Node
     * @return True on success
     */
    bool addToOpenList(OpenList &open_list, std::shared_ptr<NodeInfo> node);

    /**
     * @brief Retrieve+remove the top-node (lowest f=g+h cost) from the search open list
     * @param [in/out] open_list Search open list
     * @return Top node
     */
    std::shared_ptr<NodeInfo> removeTopFromOpenList(OpenList &open_list);

    /**
     * @brief Replace a node in the open list
     * @param [in/out] open_list Open list
     * @param node Node
     * @return True on success
     */
    bool replaceInOpenList(OpenList &open_list, std::shared_ptr<NodeInfo> node);

    /**
     * @brief Get a list of valid successors from the current node
     * @param graph Graph
     * @param sd Current search data
     * @param goal_vt Goal vertex
     * @param params Plan parameters
     * @param costCalculator Cost calculator
     * @return Valid successors
     */
    std::unordered_map<VertexId_t, std::shared_ptr<NodeInfo> > getValidSuccessors(const PoseBasedGraph &graph,
                                                                                  SearchInternalData& sd,
                                                                                  const Vertex_t &goal_vt,
                                                                                  const PlanParameters& params,
                                                                                  std::shared_ptr<IPoseBasedCostCalculator> costCalculator);

    /**
     * @brief Append a list of "best-guess" (i.e. potentially good) successors from the current node to the given list
     * @param [in/out] successors List of successors where the new succesors will be inserted
     * @param graph Graph
     * @param sd Current search data
     * @param goal_vt Goal vertex
     * @param params Plan parameters
     * @param costCalculator Cost calculator
     */
    void appendBestGuessSuccessors(std::unordered_map<VertexId_t, std::shared_ptr<NodeInfo> >& successors,
                                   const PoseBasedGraph &graph,
                                   SearchInternalData &sd,
                                   const Vertex_t &goal_vt,
                                   const PoseBasedAstar::PlanParameters &params,
                                   std::shared_ptr<IPoseBasedCostCalculator> costCalculator);

    /**
     * @brief Get a list of successors (inside the field) from the current node, from a given list of vertices
     * @param graph Graph
     * @param sd Current search data
     * @param goal_vt Goal vertex
     * @param vertices Vertices
     * @param params Plan parameters
     * @param costCalculator Cost calculator
     * @return List of successors
     */
    std::unordered_map<VertexId_t, std::shared_ptr<NodeInfo> > getIFSuccessors(const PoseBasedGraph &graph,
                                                                               const SearchInternalData &sd,
                                                                               const Vertex_t &goal_vt,
                                                                               const std::vector<Vertex_t>& vertices,
                                                                               const PlanParameters& params,
                                                                               std::shared_ptr<IPoseBasedCostCalculator> costCalculator);

    /**
     * @brief Get the cost of the shortest valid path from one node to another
     * @param node_from Source node
     * @param vt_to Target vertex
     * @param params Plan parameters
     * @param costCalculator Cost calculator
     * @return Cost
     */
    IPoseBasedCostCalculator::CostExtended_t getCostOfShortestValidPath(std::shared_ptr<const NodeInfo> node_from,
                                                                        const Vertex_t &vt_to,
                                                                        const PlanParameters& params,
                                                                        std::shared_ptr<IPoseBasedCostCalculator> costCalculator);

    /**
     * @brief Update the result data
     * @param goal_node Goal node
     * @param closed_list Closed list
     * @param start_vt Start vertex
     * @param goal_vt Goal vertex
     * @param params Plan parameters
     */
    void updateResult(std::shared_ptr<const NodeInfo> goal_node,
                      const ClosedList_t& closed_list,
                      const Vertex_t &start_vt,
                      const Vertex_t &goal_vt,
                      const PlanParameters& params);

    /**
     * @brief Append a route segment to a given route. The segment will correspond to the connection path beween the given node and its predecessor
     * @param [in/out] route_points Route points
     * @param current_node Node
     * @param params Plan parameters
     * @return True on success
     */
    static bool appendRouteSegment(std::vector<RoutePoint> &route_points,
                                   std::shared_ptr<const NodeInfo> current_node, const PlanParameters &params);

    /**
     * @brief Check for a smoother, more direct solution to reach the goal node
     * @param graph Graph
     * @param goal_node Goal node
     * @param closed_list Closed list
     * @param params Plan parameters
     * @param costCalculator Cost calculator
     */
    void checkForSmootherSolution(const PoseBasedGraph &graph,
                                  std::shared_ptr<NodeInfo> &goal_node,
                                  ClosedList_t& closed_list,
                                  const PlanParameters &params,
                                  std::shared_ptr<IPoseBasedCostCalculator> costCalculator);


private:
    Plan m_result; /**< Search results including all important informatin and processed data */
    Settings m_settings; /**< A* search settings */


    static const Vertex_t::VType AccessVertexTypeValidity_notFound; /**< VType "alias" for "Not found" */
    static const Vertex_t::VType AccessVertexTypeValidity_notCheched; /**< VType "alias" for "Not checked" */
};

}

#endif // ARO_POSEBASEDASTAR_HPP
