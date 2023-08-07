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
 
#ifndef AROLIB_ASTAR_HPP
#define AROLIB_ASTAR_HPP

#include <fstream>
#include <memory>

#include "arolib/planning/path_search/directedgraph.hpp"
#include "arolib/planning/edge_calculators/edgeCostCalculator.hpp"
#include "arolib/planning/edge_calculators/edgeSpeedCalculator.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/misc/container_helper.h"
#include "arolib/misc/logger.h"
#include "arolib/planning/generalplanningparameters.hpp"
#include "arolib/io/io_common.hpp"
#include "arolib/misc/filesystem_helper.h"
#include "arolib/planning/path_search/astarPlan.hpp"
#include "arolib/planning/planningException.hpp"

namespace arolib{

/**
 * @brief Class used to generate machine (sub)routes/plans using A* search on a arolib graph
 */
class Astar : public LoggingComponent{
public:

    /**
     * @brief Properties of a visited node
     */
    struct search_property_struct{
        DirectedGraph::vertex_t current_node; /**< Current node */
        double h; /**< H cost for the current node */
        double g; /**< G cost for the current node used for the search (might not include cost like waiting costs) */
        double totalCost = 0;/**< Total costs for the current node, including waiting costs*/
        double time; /**< Time(-stamp) of the current node */
        double waitTime; /**< Total amount of waiting time to reach this node (from the start node) */
        unsigned long search_id; /**< Search Id (used for node identification during path building) */
        DirectedGraph::vertex_t predecessor = -1; /**< Predecessor node of the current node */
        unsigned long predecessor_search_id; /**< Search Id of the predecesor node */
        DirectedGraph::edge_t edge; /**< Edge connecting the predecessor node to the current node */
        size_t numCrossings = 0; /**< Total number of inner-field cross-edges used to achive the current node from the start node */
        size_t numCrossings_HL = 0; /**< Total number of headland-harvesting cross-edges used to achive the current node from the start node */
    };

    typedef std::pair<double, search_property_struct> Node; /**< <cost, node> */

    /**
     * @brief Strategy used for collision avoidance
     */
    enum CollisionAvoidanceOption{
        WITHOUT_COLLISION_AVOIDANCE, /**< Do not check for collision avoidance */
        COLLISION_AVOIDANCE__INFIELD, /**< Collision avoidance only in searches done in the harvesting areas (i.e. no colision avoidance in the headland when planning for inner-field hrvesting operations) */
        COLLISION_AVOIDANCE__OVERALL /**< Collision avoidance in searches done everywhere (inc. in the headland when planning for inner-field hrvesting operations) */
    };
    /**
      * @brief Get the CollisionAvoidanceOption (enum) from its int value
      * @param value Int value
      * @return CollisionAvoidanceOption
      */
    static CollisionAvoidanceOption intToCollisionAvoidanceOption(int value);


    /**
     * @brief Class used to check if a sucessor is valid during AStar search
     */
    class ISuccesorChecker{

    public:
        enum Validity{
            VALID = 0,
            INVALID__SUCCESSOR_TIMESTAMP,
            INVALID__GOAL_TIME_LIMIT,
            INVALID__SUCCESSOR_EXCLUDED,
            INVALID__EDGE_EXCLUDED,
            INVALID__CURRENT_VERTEX_GOT_BUSY,
            INVALID__COST_LIMIT,
            INVALID__OTHER = 100
        };

        /**
         * @brief Input parameters for the method isSuccessorValid
        */
        struct IsSuccessorValidParams{

            /**
             * @brief Constructor
             *
             * @param machine
             * @param target_timestamp
             * @param waiting_time
             * @param current_search_node
             * @param vt_from
             * @param vt_from_prop
             * @param vt_to
             * @param vt_to
             * @param vt_start
             * @param vt_start_prop
             * @param vt_goal
             * @param vt_goal_prop
             * @param _edge
             * @param _edge_prop
             * @param graph
            */
            explicit IsSuccessorValidParams(const Machine& _machine,
                                            double _vt_to_timestamp,
                                            double _waiting_time,
                                            const Astar::search_property_struct& _current_search_node,
                                            const DirectedGraph::vertex_t& _vt_from,
                                            const DirectedGraph::vertex_property& _vt_from_prop,
                                            const DirectedGraph::vertex_t& _vt_to,
                                            const DirectedGraph::vertex_property& _vt_to_prop,
                                            const DirectedGraph::vertex_t& _vt_start,
                                            const DirectedGraph::vertex_property& _vt_start_prop,
                                            const DirectedGraph::vertex_t& _vt_goal,
                                            const DirectedGraph::vertex_property& _vt_goal_prop,
                                            const DirectedGraph::edge_t& _edge,
                                            const DirectedGraph::edge_property& _edge_prop,
                                            CollisionAvoidanceOption _collisionAvoidanceOption,
                                            const DirectedGraph::Graph& _graph);

            const Machine& machine;
            double target_timestamp;
            double waiting_time;
            const Astar::search_property_struct& current_search_node;
            const DirectedGraph::vertex_t& vt_from;
            const DirectedGraph::vertex_property& vt_from_prop;
            const DirectedGraph::vertex_t& vt_to;
            const DirectedGraph::vertex_property& vt_to_prop;
            const DirectedGraph::vertex_t& vt_start;
            const DirectedGraph::vertex_property& vt_start_prop;
            const DirectedGraph::vertex_t& vt_goal;
            const DirectedGraph::vertex_property& vt_goal_prop;
            const DirectedGraph::edge_t& edge;
            const DirectedGraph::edge_property& edge_prop;
            CollisionAvoidanceOption collisionAvoidanceOption;
            const DirectedGraph::Graph& graph;
        };

        /**
         * @brief Input parameters for the method getMinDurationAtEdge
        */
        struct GetMinDurationAtEdgeParams{

            /**
             * @brief Constructor
             *
             * @param machine
             * @param clearance_time
             * @param current_search_node
             * @param vt_from
             * @param vt_from_prop
             * @param vt_to
             * @param vt_to
             * @param vt_start
             * @param vt_start_prop
             * @param vt_goal
             * @param vt_goal_prop
             * @param _edge
             * @param _edge_prop
             * @param graph
            */
            explicit GetMinDurationAtEdgeParams(const Machine& _machine,
                                                double _clearance_time,
                                                const Astar::search_property_struct& _current_search_node,
                                                const DirectedGraph::vertex_t& _vt_from,
                                                const DirectedGraph::vertex_property& _vt_from_prop,
                                                const DirectedGraph::vertex_t& _vt_to,
                                                const DirectedGraph::vertex_property& _vt_to_prop,
                                                const DirectedGraph::vertex_t& _vt_start,
                                                const DirectedGraph::vertex_property& _vt_start_prop,
                                                const DirectedGraph::vertex_t& _vt_goal,
                                                const DirectedGraph::vertex_property& _vt_goal_prop,
                                                const DirectedGraph::edge_t& _edge,
                                                const DirectedGraph::edge_property& _edge_prop,
                                                CollisionAvoidanceOption _collisionAvoidanceOption,
                                                const DirectedGraph::Graph& _graph);

            const Machine& machine;
            double clearance_time;
            const Astar::search_property_struct& current_search_node;
            const DirectedGraph::vertex_t& vt_from;
            const DirectedGraph::vertex_property& vt_from_prop;
            const DirectedGraph::vertex_t& vt_to;
            const DirectedGraph::vertex_property& vt_to_prop;
            const DirectedGraph::vertex_t& vt_start;
            const DirectedGraph::vertex_property& vt_start_prop;
            const DirectedGraph::vertex_t& vt_goal;
            const DirectedGraph::vertex_property& vt_goal_prop;
            const DirectedGraph::edge_t& edge;
            const DirectedGraph::edge_property& edge_prop;
            CollisionAvoidanceOption collisionAvoidanceOption;
            const DirectedGraph::Graph& graph;
        };

        /**
         * @brief Constructor.
         */
        explicit ISuccesorChecker() = default;

        /**
         * @brief Destructor.
         */
        virtual ~ISuccesorChecker(){ }

        /**
         * @brief Check if a sucesor is valid during AStar search
         * @param params Input parameters
         * @return Validity response
        */
        virtual Validity isSuccessorValid(const IsSuccessorValidParams& params) const = 0;

        /**
         * @brief Get the minimum duration [s] for a machine planning to go over the edge
         * @param params Input parameters
         * @return Minimum duration [s]. If < 0, no minimum duration applies (disregarded during search)
        */
        virtual double getMinDurationAtEdge(const GetMinDurationAtEdgeParams& params) const{ return -1; }


    };

    /**
     * @brief Plan parameters
     *
     * About max_time_visit: used to avoid that the machine drives over unharvested vertices (i.e. corresponding to harvesting route points with timestamps >= 0) when a harvesting delay is allowed (i.e. max_time_goal > overload-start timestamp). If a vertex has a (harvesting) timestamp higher that this value, the machine is not allowed to drive over the vertex even if there is a very high allowed harvester delay/waiting-time
     */
    struct PlanParameters{
        DirectedGraph::vertex_t start_vt = -1; /**< Start vertex */
        DirectedGraph::vertex_t goal_vt = -1; /**< Goal vertex */
        double start_time = -1; /**< Time(-stamp) at the start of the plan (i.e. time at which the machine is at the start vertex) */
        Machine machine; /**< Machine used for the search */
        double machine_speed = -1; /**< Speed of the machine */
        double initial_bunker_mass = -1; /**< Machine's initial bunker mass */
        bool includeWaitInCost = true; /**< Should we include the time the machine has to wait to drive over an edge in the edge's cost calculation? */
        double max_cost = -1; /**< Maximum allowed cost (disregarded if < 0) */

        std::vector<std::shared_ptr<const ISuccesorChecker>> successorCheckers;/**< Checkers used to know if a transition to a certain successor node is valid */
        //const IEdgeSpeedCalculator* edgeSpeedCalculator = nullptr; @todo check how to include correctly a speed calculator to be used
    };

    /**
     * @brief Reason why a node was not visited (used for seach analysis)
     */
    enum ExpansionExclussionReason{
        OK, /**< 0: no reason not to visit --> will be visited */
        IN_OPEN_LIST_AND_BETTER, /**< 1: The node is in the open list with a lower cost */
        IN_VISITED_LIST_AND_BETTER, /**< 2: The node is in the visited list with a lower cost */
        TENTATIVE_TIME_OVER_MAX_TIME_GOAL, /**< 3: The tentative time to reach this node is higher than the maximum allowed time to reach the goal */
        TOWARDS_CURRENT_VERTEX, /**< 4: There is a machine already in that node using that edge to reach the current vertex (collision avoidance) */
        IN_SUCC_EXCLUDE_SET, /**< 5: The node is in the given exclude set */
        IN_EDGE_EXCLUDE_SET, /**< 6: The node is in the given exclude set */
        IN_CLOSED_LIST, /**< 7: The node is in the closed list */
        GRAPH_LOCATION, /**< 8: The node is in the wrong graph location (e.g. the search is done to reach an OL point in the headland, but the node belongs to the inner-field search set) */
        SUCCESSOR_TIMESTAMP_OVER_MAX_TIME_VISIT, /**< 9: The time to reach this node is higher than the maximum allowed time to visit a node */
        CURRENT_VERTEX_GOT_BUSY, /**< 10: Before the machine left the node, the node got occupied (i.e. the machine cannot stay that long in the current node) */
        COST_LIMIT, /**< 11: The g+h cost is higher than the cost limit */
        OTHER = 100 /**< 100: Other */
    };
    /**
      * @brief Get the ExpansionExclussionReason (enum) from its int value
      * @param value Int value
      * @return ExpansionExclussionReason
      */
    static ExpansionExclussionReason intToExpansionExclussionReason(int value);

    /**
      * @brief Parse from ISuccesorChecker::Validity to ExpansionExclussionReason
      * @param value ISuccesorChecker::Validity value
      * @return ExpansionExclussionReason
      */
    static ExpansionExclussionReason toExpansionExclussionReason(ISuccesorChecker::Validity value);


    /**
     * @brief Data / properties of the current node to be expanded (used for seach analysis)
     */
    struct ExpansionAttemptCurrentData{
        search_property_struct node; /**< Node properties */
        Point point; /**< Node location */
    };

    /**
     * @brief Data / properties of the potential successors of the current node to be expanded (used for seach analysis)
     */
    struct ExpansionAttemptSuccessorData{
        DirectedGraph::vertex_t vt = -1; /**< Node vertex */
        Point point; /**< Node location */
        ExpansionExclussionReason exclussionReason; /**< Reason why a node was not visited (if applicable)  */
        double tentative_g = std::nan(""); /**< Tentative G cost for this successor node */
        double tentative_h = std::nan(""); /**< Tentative H cost for this successor node */
        double calc_time = std::nan(""); /**< Calculated time to reach this successor node (based on machine speed, arrival time, etc.) */
        double min_time = std::nan(""); /**< Minimum time needed to reach this successor node (based on the (harvester) timestamp of the successor vertex (i.e. not to drive over non-harvested points), the desired cleareance time (time the machine has to wait after the point is harvested), etc.)  */
        double busy_time = std::nan(""); /**< Time the machine has to wait to reach the successor node because it is busy with another machine(s) (i.e. another machine(s) are in this node at that time) */
        double tentative_time = std::nan(""); /**< Tentative time (-stamp) for this successor node (based on the calc_time, min_time, busy_time, etc.) */
    };

    /**
     * @brief Struct containing the expantion attempts of a node (used for seach analysis)
     */
    struct ExpansionAttemptData{
        int count; /**< Amount of expansion attemps until now (starting from the expantion of the start node) */
        ExpansionAttemptCurrentData current_vt_data; /**< Data / properties of the current node to be expanded */
        std::vector< ExpansionAttemptSuccessorData > successor_vts_data; /**< Data / properties of the potential successors of the current node to be expanded */
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
    struct AStarSettings{
        double clearanceTime = 10; /**< Time a machine has to wait so that another machine clears a vertex before it drives to it */
        CollisionAvoidanceOption collisionAvoidanceOption = CollisionAvoidanceOption::WITHOUT_COLLISION_AVOIDANCE; /**< Strategy used for collision avoidance */
        bool includeWaitInCost = true; /**< Should we include in the edge costs the time that a machine has to wait to go to a vertex (e.g. because it has not been harvested yet or it is occupied by another machine)? */

        /**
         * @brief Constructor.
         * @param _clearanceTime Clearance time
         * @param _collisionAvoidanceOption Collision-avoidance option
         * @param _includeWaitInCost Include waiting time in cost calculations?
         */
        explicit AStarSettings(double _clearanceTime = 10,
                               CollisionAvoidanceOption _collisionAvoidanceOption = CollisionAvoidanceOption::WITHOUT_COLLISION_AVOIDANCE,
                               bool _includeWaitInCost = true);

        /**
         * @brief Parse the parameters from a string map, starting from a default AStarSettings
         * @param [out] param Parameters
         * @param map String map containing the parameter values
         * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the default values will remain)
         * @return True on success
         */
        static bool parseFromStringMap( AStarSettings& params, const std::map<std::string, std::string>& map, bool strict = false);

        /**
         * @brief Parse the parameters to a string map
         * @param param Parameters
         * @return String map containing the parameter values
         */
        static std::map<std::string, std::string> parseToStringMap( const AStarSettings& params);

    };

    /**
     * @brief Constructor
     * @param parameters Plan parameters/settings
     * @param settings Search parameters/settings
     * @param defaultRPType Default route-point type to be set in the generated route-points
     * @param outputFolder Folder where the planning (search) information will be stored (if empty-string, no data will be saved)
     * @param _logger (optional) Pointer to the parent logger (to use the parent's logLevel). If NULL, it creates a default logger.
     */
    explicit Astar(const PlanParameters& parameters,
                   const AStarSettings & settings,
                   RoutePoint::RoutePointType defaultRPType,
                   const std::string& outputFolder = "",
                   std::shared_ptr<Logger> _logger = nullptr);

    /**
     * @brief Destructor.
     */
    virtual ~Astar();

    /**
     * @brief Plan route
     * @param graph Graph used for the planning
     * @param edgeCostCalculator Edge Cost Calculator. Temporary: if = nullptr, uses internal functions
     * @return True on success
     */
    bool plan(const DirectedGraph::Graph &graph,
              std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator);


    /**
     * @brief Check if the planning was successful and a plan exists
     * @return True on success
     */
    bool hasPlan() const;


    /**
     * @brief Get the planning results
     * @return Planning results
     */
    const AstarPlan& getPlan() const;


    /**
     * @brief Get the file name where the expantion attempts information are saved
     * @return File name where the expantion attempts information are saved
     */
    static const std::string& filename_expansionAttempts();

    /**
     * @brief Read the expantion attempts information from a file
     * @param filename Path/name of the file to be read
     * @param [out] vertices Vertices used during the search and their corresponding route points (read from file)
     * @param [out] edges Edges used during the search and their corresponding connected vertices (read from file)
     * @param [out] planParameters Plan parameters/settings used during the search (read from file)
     * @param [out] expansionAttempts Expantion attempts information (read from file)
     * @param _logger (optional) Pointer to the parent logger (to use the parent's logLevel). If NULL, it creates a default logger.
     * @return True on success
     */
    static bool readFile_expansionAttempts(const std::string& filename,
                                           std::map<DirectedGraph::vertex_t, DirectedGraph::vertex_property> &vertices,
                                           std::map<std::pair<DirectedGraph::vertex_t, DirectedGraph::vertex_t>, DirectedGraph::edge_property> &edges,
                                           PlanParameters &planParameters,
                                           std::vector<ExpansionAttemptData> &expansionAttempts,
                                           std::shared_ptr<Logger> _logger = nullptr);

    /**
     * @brief (struct) function to compare the costs of two nodes
     */
    struct get_min_f_struct{
        /**
         * @brief Function to compare the costs of two nodes
         * @param a First node to compare
         * @param b Second node to compare
         * @return True if the cost of node a is >= to the cost of node b
         */
        bool operator()(Node const& a, Node const& b) const{
            return a.first > b.first || a.first == b.first;
        }
    };

private:

    /**
     * @brief Builds a route point based on the vertex properties and other relevant data
     * @param vp Vertex property containing all relevant vertex data
     * @param time Timestamp
     * @return Built route point
     */
    RoutePoint buildRoutePoint(const DirectedGraph::vertex_property &vp, double time);

    /**
     * @brief Builds a route based on the final search lists and saves it, together with other rearch results, in the 'result' internal variable
     * @param graph Graph used for the search
     * @param closed_list Search closed list
     * @param _current_node Current (final) node information
     * @param start_vertex Start vertex
     * @param start_id Search id corresponding to the start node
     * @return True on success
     */
    bool buildPath(const DirectedGraph::Graph &graph,
                   const std::map<DirectedGraph::vertex_t, search_property_struct> &closed_list,
                   const search_property_struct &_current_node,
                   const DirectedGraph::vertex_t &start_vertex,
                   unsigned long start_id);

    /**
     * @brief Checks if two route points are equal
     * @param rp1 Route point 1
     * @param rp2 Route point 2
     * @return True if the route points are considered equal
     */
    bool isSameRoutePoint(const RoutePoint& rp1, const RoutePoint& rp2) const;


    /**
     * @brief Checks if a route point if of an special type
     * @param route_point route_point
     * @return True if the route point if of an special type
     */
    bool isOfSpecialType(const RoutePoint& route_point);

    /**
     * @brief Expand a node (i.e find valid succesors and process them), updates the search lists, updates the search-analysis variables, etc.
     * @param graph Graph used for the search
     * @param current_node Node to be expanded
     * @param [in/out] open_list Search open list
     * @param [in/out] closed_list Search closed list
     * @param goal_prop Vertex property of the goal vertex
     * @param start_prop Vertex property of the start vertex
     * @param start_id Id of the start node
     */
    void expandNode(const DirectedGraph::Graph &graph,
                    const Astar::search_property_struct &current_node,
                    std::multimap<double, Astar::search_property_struct> &open_list,
                    std::map<DirectedGraph::vertex_t, search_property_struct> &closed_list,
                    std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                    const DirectedGraph::vertex_property &goal_prop,
                    const DirectedGraph::vertex_property &start_prop,
                    unsigned long start_id);

    /**
     * @brief Adds a node to the search open list
     * @param [in/out] open_list Search open list
     * @param f Cost of the node to be added (f=g+h)
     * @param node_property Properties' struct of the node to be added
     */
    void add_to_open_list(std::multimap<double, Astar::search_property_struct> &open_list, double f, const search_property_struct& node_property);

    /**
     * @brief Retrieve+remove the top-node (lowest f=g+h cost) from the search open list
     * @param [in/out] open_list Search open list
     * @param Properties' struct of the node in the top of the search open list (i.e. the one with the lowest f=g+h cost)
     * @return Top-node
     */
    search_property_struct remove_top_from_open_list(std::multimap<double, search_property_struct> &open_list);

    /**
     * @brief Update the minimum g cost (internal variable) of the search open list
     * @param open_list Search open list
     * @param removed_g Removed g cost
     * @param new_g new g cost
     */
    void update_min_g_in_open_list(const std::multimap<double, search_property_struct> &open_list, const double& removed_g, const double& new_g);

    /**
     * @brief Update the minimum g cost (internal variable) of the search open list
     * @param open_list Search open list
     */
    void update_min_g_in_open_list(const std::multimap<double, Astar::search_property_struct> &open_list);

    /**
     * @brief Save the expansion attempts information in tzhe file (iif a valid desired folder was set)
     * @param graph Graph
     */
    void saveExpansionAttempts(const DirectedGraph::Graph &graph);

    /**
     * @brief Write search data to the debug file (deprecated)
     * @param current Current node (+properties)
     * @param start Start vertex
     * @param goal Goal vertex
     * @param open_list Search open list
     * @param closed_list Search closed list
     */
    void writeToDebugFile(const DirectedGraph::Graph &graph,
                          const Astar::search_property_struct &current,
                          const DirectedGraph::vertex_t &start,
                          const DirectedGraph::vertex_t &goal,
                          const std::multimap<double, Astar::search_property_struct> &open_list,
                          const std::map<DirectedGraph::vertex_t, search_property_struct> &closed_list);


private:
    AstarPlan m_result; /**< Search results including all important information and processed data */
    PlanParameters m_parameters; /**< Plan parameters/settings */
    AStarSettings m_settings; /**< A* search settings */
    std::string m_outputFolder = ""; /**< Folder where the planning (search) information will be stored (if empty-string, no data will be saved) */

    unsigned long m_search_id_cnt; /**< Id counter to set search ids to new nodes */
    double m_min_g_in_open_list; /**< Minimum G cost in the open list */


    std::map< DirectedGraph::vertex_t,
              std::vector< std::tuple<unsigned int, DirectedGraph::vertex_t, double, double> > > m_openListCandidates; /**< Candidate nodes to be added to the open list */
    unsigned int m_expandNodeCount; /**< Total amount of node-expansion attemped since start of search */
    std::vector<ExpansionAttemptData> m_expansionAttempts; /**< Expantion attempts information */

    RoutePoint::RoutePointType m_defaultRPType = RoutePoint::DEFAULT; /**< Deafult route-point type */

    static const std::string m_filename_expansionAttempts; /**< File name where the expantion attempts information are saved (inside the given folder, if applicable) */
};


}

#endif // AROLIB_ASTAR_HPP
