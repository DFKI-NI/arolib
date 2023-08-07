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
 
#ifndef ARO_INFIELDTRACKSCONNECTORGRAPHBASED_HPP
#define ARO_INFIELDTRACKSCONNECTORGRAPHBASED_HPP

#include <ctime>
#include <future>
#include <mutex>

#include "arolib/misc/loggingcomponent.h"
#include "arolib/types/pose2D.hpp"
#include "arolib/types/subfield.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/planning/track_connectors/infieldtracksconnector.hpp"
#include "arolib/planning/edge_calculators/edgeCostCalculator.hpp"
#include "arolib/planning/path_search/graphhelper.hpp"
#include "arolib/planning/path_search/astar.hpp"
#include "arolib/planning/path_search/astar_successor_checkers.hpp"

namespace arolib{

/**
 * @brief Infield-tracks connector based on graph search
 */
class InfieldTracksConnectorGraphBased : public IInfieldTracksConnector
{
public:
    /**
     * @brief Vertex function parameters
     */
    struct VertexFunctionParams
    {
        DirectedGraph::vertex_t vt;  /**< Vertex */
        DirectedGraph::vertex_t vt_start;  /**< Start vertex of the search */
        DirectedGraph::vertex_t vt_goal;  /**< Goal vertex of the search */
        DirectedGraph::vertex_property vt_prop;  /**< Vertex property */
        DirectedGraph::vertex_property vt_start_prop;  /**< Start vertex property */
        DirectedGraph::vertex_property vt_goal_prop;  /**< Goal vertex property */
        bool vt_start_valid = true;  /**< Flag stating if the start vertex of the search is valid */
        bool vt_goal_valid = true;  /**< Flag stating if the goal vertex of the search is valid */

        /**
         * @brief Set the vertex function parameters from a given SuccessorChecker parameters
         *
         * @param scParams SuccessorChecker parameters
         */
        void fromSuccessorCheckerParams(const Astar::ISuccesorChecker::IsSuccessorValidParams& scParams );
    };

    /**
     * @brief Edge function parameters
     */
    struct EdgeFunctionParams
    {
        DirectedGraph::edge_t edge;  /**< Edge */
        DirectedGraph::edge_property edge_prop;  /**< Edge property */
        DirectedGraph::vertex_t vt_from;  /**< Start vertex of the edge */
        DirectedGraph::vertex_t vt_to;  /**< Successor vertex of the edge */
        DirectedGraph::vertex_t vt_start;  /**< Start vertex of the search */
        DirectedGraph::vertex_t vt_goal;  /**< Goal vertex of the search */
        DirectedGraph::vertex_property vt_from_prop;  /**< Property of the start vertex of the edge */
        DirectedGraph::vertex_property vt_to_prop;  /**< Property of the successor vertex of the edge */
        DirectedGraph::vertex_property vt_start_prop;  /**< Start vertex property */
        DirectedGraph::vertex_property vt_goal_prop;  /**< Goal vertex property */


        /**
         * @brief Set the edge function parameters from a given SuccessorChecker parameters
         *
         * @param scParams SuccessorChecker parameters
         */
        void fromSuccessorCheckerParams(const Astar::ISuccesorChecker::IsSuccessorValidParams& scParams );
    };

    using ExcludeVertexFunc = std::function< bool ( const VertexFunctionParams& ) >;
    using AllowVertexFunc = std::function< bool ( const VertexFunctionParams& ) >;

    using ExcludeEdgeFunc = std::function< bool ( const EdgeFunctionParams& ) >;
    using AllowEdgeFunc = std::function< bool ( const EdgeFunctionParams& ) >;

    /**
     * @brief Constructor.
     * @param parentLogger Parent logger
     */
    explicit InfieldTracksConnectorGraphBased(std::shared_ptr<Logger> parentLogger = nullptr);

    /**
     * @brief Get the connection path between two poses.
     * @sa IInfieldTracksConnector::getConnection
     */
    virtual std::vector<Point> getConnection(const Machine& machine,
                                             const Pose2D& _pose_start,
                                             const Pose2D& _pose_end,
                                             double turningRad = -1,
                                             const std::pair<double, double>& extraDist = std::make_pair(-1, -1),
                                             const Polygon& limitBoundary = Polygon(),
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

    /**
     * @brief Set the graph to be used in the search.
     * @param graph graph
     */
    void setGraph(const DirectedGraph::Graph& graph);

    /**
     * @brief Set the edge cost calculator used in the search.
     * @param costCalculator calculator (if null, sets the default)
     */
    void setCostCalculator(std::shared_ptr<IEdgeCostCalculator> costCalculator);

    /**
     * @brief Set the functions that will exclude vertices and edges to be used that otherwise would be allowed (e.g. boundary vts/edges or certain headland vts/edges).
     * @param func function
     */
    void setExcludeFunctions(const ExcludeVertexFunc& vtFunc, const ExcludeEdgeFunc& edgeFunc);

    /**
     * @brief Set the functions that will allow vertices and edges to be used that otherwise would be excluded (e.g. vertices(edges inside the inner-field).
     * @param func function
     */
    void setAllowFunctions(const AllowVertexFunc& vtFunc, const AllowEdgeFunc& edgeFunc);

    /**
     * @brief Set the maximum amount of edges used to search for via vertices
     * @param val Maximum amount of edges used to search for via vertices (if == 0 -> default)
     */
    void setViaMaxEdges(int val);

    /**
     * @brief Set the maximum amount of vertices to be used in the search to obtain the best vertices corresponding to start and end poses
     * @param val Maximum amount of vertices to be used in the search to obtain the best vertices corresponding to start and end poses (if < 0 -> default; if == 0 -> no limit)
     */
    void setMaxCountCloseVts(int val);

    /**
     * @brief Set the folder where the planning (search) information will be stored (if empty-string, no data will be saved)
     * @param outputFolder Folder where the planning (search) information will be stored (if empty-string, no data will be saved)
     */
    void setOutputSearchFolder(const std::string& outputFolder = "");

protected:


    /**
     * @brief Check called at the beginning to know if the minimum possible length for the given parameters is higher than the given maximum connection length limit.
     * @sa IInfieldTracksConnector::checkForMinPossibleLength
     */
    bool checkForMinPossibleLength(const Pose2D& pose_start,
                                   const Pose2D& pose_end,
                                   double turningRad,
                                   const Polygon& limitBoundary,
                                   const Polygon& infieldBoundary,
                                   double maxConnectionLength ) const override;

    /**
     * @brief Get the connection path between two poses using a single thread
     *
     * @param machine machine
     * @param pose_start Start pose
     * @param track track points
     * @param infieldBoundary Boundary if the inner field
     * @param maxConnectionLength Limit of the connection length (disregarded if <= 0)
     * @param vts0 Potential starting vertices for the search
     * @param vtsn Potential goal vertices for the search
     * @param astarParamsBase Base AStart parameters
     * @param astarSettings AStart settings
     * @param [in, out] minCost Current minimum cost (cost limit)
     * @param [out] initPath Initial connection path segment
     * @param [out] finishPath Last connection path segment
     */
    void getConnectionSingleThread(const Machine& machine,
                                   const Pose2D& pose_start,
                                   const Pose2D& pose_end,
                                   const Polygon& infieldBoundary,
                                   double maxConnectionLength,
                                   const std::vector<DirectedGraph::vertex_t> &vts0,
                                   const std::vector<DirectedGraph::vertex_t> &vtsn,
                                   const Astar::PlanParameters &astarParamsBase,
                                   const Astar::AStarSettings &astarSettings,
                                   double &minCost,
                                   std::vector<Point> &initPath, std::vector<Point> &finishPath) const;

    /**
     * @brief Get the connection path between two poses using multiple threads
     *
     * @param machine machine
     * @param pose_start Start pose
     * @param track track points
     * @param infieldBoundary Boundary if the inner field
     * @param maxConnectionLength Limit of the connection length (disregarded if <= 0)
     * @param vts0 Potential starting vertices for the search
     * @param vtsn Potential goal vertices for the search
     * @param astarParamsBase Base AStart parameters
     * @param astarSettings AStart settings
     * @param [in, out] minCost Current minimum cost (cost limit)
     * @param [out] initPath Initial connection path segment
     * @param [out] finishPath Last connection path segment
     */
    void getConnectionMultiThread(const Machine& machine,
                                  const Pose2D& pose_start,
                                  const Pose2D& pose_end,
                                  const Polygon& infieldBoundary,
                                  double maxConnectionLength,
                                  const std::vector<DirectedGraph::vertex_t> &vts0,
                                  const std::vector<DirectedGraph::vertex_t> &vtsn,
                                  const Astar::PlanParameters &astarParamsBase,
                                  const Astar::AStarSettings &astarSettings,
                                  double &minCost,
                                  std::vector<Point> &initPath, std::vector<Point> &finishPath) const;

    /**
     * @brief Check if a vertex is valid
     *
     * @param params Vertex function parameters
     * @param infieldBoundary Inner-field boundary
     * @param allowInitLocationVts Flag stating if vertices corresponding to machine initial locations should be allowed or excluded     *
     * @return True if valid
     */
    bool isVertexValid(const VertexFunctionParams& params, const Polygon& infieldBoundary, bool allowInitLocationVts = false) const;

    /**
     * @brief Check if a vertex is valid based on the starting pose
     *
     * @param params Vertex function parameters
     * @param pose_start Start pose
     * @param infieldBoundary Inner-field boundary     *
     * @return True if valid
     */
    bool isStartVtValid(const VertexFunctionParams& params, const Pose2D& pose_start, const Polygon& infieldBoundary) const;

    /**
     * @brief Check if a vertex is valid based on the final pose
     *
     * @param params Vertex function parameters
     * @param pose_end Final pose
     * @param infieldBoundary Inner-field boundary     *
     * @return True if valid
     */
    bool isEndVtValid(const VertexFunctionParams& params, const Pose2D& pose_end, const Polygon& infieldBoundary) const;

    /**
     * @brief Check if an edge is valid
     *
     * @param params Edge function parameters
     * @param vt_start Start vertex of the search
     * @param vt_end Goal vertex of the search
     * @param pose_start Start pose
     * @param pose_end Final pose
     * @param prev_vts_init Previous vertices correspoonding to the last points of the initial path segment     *
     * @return True if valid
     */
    bool isEdgeValid(const EdgeFunctionParams& params,
                     const DirectedGraph::vertex_t &vt_start, const DirectedGraph::vertex_t &vt_end,
                     const Pose2D &pose_start, const Pose2D &pose_end,
                     const std::vector<DirectedGraph::vertex_t> &prev_vts_init) const;


    /**
     * @brief Get the folder where the search information will be stored
     *
     * @param params Edge function parameters
     * @param vt_start Start vertex of the search
     * @param vt_end Goal vertex of the search
     * @param vt_via Via (intermediate) vertex
     * @param isInit Flag stating if the search corresponds to a search for the initial path segment     *
     * @return True if valid
     */
    std::string getOutputSearchFolder(const DirectedGraph::vertex_t& vt_start, const DirectedGraph::vertex_t& vt_end, const DirectedGraph::vertex_t& vt_via, bool isInit) const;

    /**
     * @brief Get the (base) future for the search of a connection path between a specific start vertex and potential goal vertices (for search with multiple threads).
     *
     * @param mutex Mutex
     * @param machine machine
     * @param infieldBoundary Boundary if the inner field
     * @param maxConnectionLength Limit of the connection length (disregarded if <= 0)
     * @param pose_start Start pose
     * @param pose_end End pose
     * @param vt_start Start vertex of the search
     * @param vtsn Potential goal vertices of the path search
     * @param [in, out] minCost Current minimum cost (cost limit)
     * @param [out] initPath Initial connection path segment
     * @param [out] finishPath Last connection path segment
     * @return Future
     */
    std::future< void > getFutureStart(std::mutex &mutex,
                                       const Machine &machine, const Polygon &infieldBoundary, const double& maxConnectionLength,
                                       const Pose2D &pose_start, const Pose2D &pose_end,
                                       const DirectedGraph::vertex_t &vt_start,
                                       const std::vector<DirectedGraph::vertex_t> &vtsn,
                                       const Astar::PlanParameters &astarParamsBase,
                                       const Astar::AStarSettings &astarSettings,
                                       double &minCost,
                                       std::vector<Point> &initPath, std::vector<Point> &finishPath) const;

    /**
     * @brief Get the future for the search of a connection path between a specific start vertex and a specific goal vertex via potential intermediate vertices (for search with multiple threads).
     *
     * @param mutex Mutex
     * @param machine machine
     * @param infieldBoundary Boundary if the inner field
     * @param maxConnectionLength Limit of the connection length (disregarded if <= 0)
     * @param pose_start Start pose
     * @param pose_end End pose
     * @param vt_start Start vertex of the search
     * @param vt_end Goal vertex of the path search
     * @param via_vts Potential intermediate vertices of the path search
     * @param [in, out] minCost Current minimum cost (cost limit)
     * @param [out] initPath Initial connection path segment
     * @param [out] finishPath Last connection path segment
     * @param costStart Cost corresponding to the connection between the start pose and the start vertex
     * @return Future
     */
    std::future< void > getFutureEnd(std::mutex &mutex,
                                     const Machine &machine, const Polygon &infieldBoundary, const double& maxConnectionLength,
                                     const Pose2D &pose_start, const Pose2D &pose_end,
                                     const DirectedGraph::vertex_t &vt_start,
                                     const DirectedGraph::vertex_t &vt_end,
                                     const std::set<DirectedGraph::vertex_t>& via_vts,
                                     const Astar::PlanParameters &astarParamsBase,
                                     const Astar::AStarSettings &astarSettings,
                                     double &minCost,
                                     std::vector<Point> &initPath, std::vector<Point> &finishPath,
                                     const double& costStart) const;


    /**
     * @brief Get the future for the search of a connection path between a specific start vertex and a specific goal vertex via a specific intermediate vertex (for search with multiple threads).
     *
     * @param mutex Mutex
     * @param machine machine
     * @param infieldBoundary Boundary if the inner field
     * @param maxConnectionLength Limit of the connection length (disregarded if <= 0)
     * @param pose_start Start pose
     * @param pose_end End pose
     * @param vt_start Start vertex of the search
     * @param vt_via Intermediate vertex of the path search
     * @param vt_end Goal vertex of the path search
     * @param [in, out] minCost Current minimum cost (cost limit)
     * @param [out] initPath Initial connection path segment
     * @param [out] finishPath Last connection path segment
     * @param costStart Cost corresponding to the connection between the start pose and the start vertex
     * @param costEnd Cost corresponding to the connection between the end pose and the goal vertex
     * @param vtParamsBase Base vertex parameters
     * @return Future
     */
    std::future< void > getFutureVia(std::mutex &mutex,
                                     const Machine &machine, const Polygon &infieldBoundary, const double& maxConnectionLength,
                                     const Pose2D &pose_start, const Pose2D &pose_end,
                                     const DirectedGraph::vertex_t &vt_start,
                                     const DirectedGraph::vertex_t &vt_via,
                                     const DirectedGraph::vertex_t &vt_end,
                                     const Astar::PlanParameters &astarParamsBase,
                                     const Astar::AStarSettings &astarSettings,
                                     double &minCost,
                                     std::vector<Point> &initPath, std::vector<Point> &finishPath,
                                     const double& costStart, const double& costEnd,
                                     const VertexFunctionParams& vtParamsBase) const;

protected:

    /**
     * @brief Internal edge-cost calculator.
     */
    class InternalEdgeCostCalculator: public IEdgeCostCalculator{
    public:
        InternalEdgeCostCalculator();

        /**
         * @brief Generates internal parameters from the graph, e.g. edge parameters to be used in the cost calculation
         * @sa generateInternalParameters::calcCost
         */
        virtual void generateInternalParameters(DirectedGraph::Graph& graph) override;

        /**
         * @brief Compute the edge cost for a default edge based on its start/end points' locations.
         * @sa IEdgeCostCalculator::calcCost
         */
        virtual double calcCost(const Machine& machine,
                                const Point& p1,
                                const Point& p2,
                                double time,
                                double waitingTime,
                                double bunkerMass,
                                const std::vector<DirectedGraph::overroll_property>& overruns) override;

        /**
         * @brief Compute the edge cost for a known edge.
         * @sa IEdgeCostCalculator::calcCost
         */
        virtual double calcCost(const Machine& machine,
                                const DirectedGraph::edge_t& edge,
                                const DirectedGraph::edge_property& edge_prop,
                                double time,
                                double waitingTime,
                                double bunkerMass) override;

        /**
         * @brief Calculate the heuristic (for A* search)
         * @sa IEdgeCostCalculator::calcHeuristic
         * @param bestSpeed Maxinum speed throuout the search
         * @param bestWeight Disregarded
         */
        virtual double calcHeuristic(const Machine& machine,
                                     const DirectedGraph::vertex_property &v_prop_current,
                                     const DirectedGraph::vertex_property &v_prop_goal) override;
    protected:


        /**
         * @brief Set other specific parameters from a string map containing the values as string.
         * @sa IEdgeCostCalculator::parseOtherParametersFromStringMap
         */
        virtual bool parseOtherParametersFromStringMap(const std::map<std::string, std::string>& strMap, bool strict) override;

        /**
         * @brief Parse other specific parameters and append them to the string map .
         * @sa IEdgeCostCalculator::parseAndAppendOtherParametersToStringMap
         */
        virtual void parseAndAppendOtherParametersToStringMap(std::map<std::string, std::string>& strMap) const override;

    public:

        std::shared_ptr<IEdgeCostCalculator> m_baseCostCalculator;  /**< Base cost calculator */
        const Polygon* m_boundary = nullptr;  /**< Boundary */
        double m_turningRad = -1;  /**< Turning radius */
    };


    DirectedGraph::Graph m_graph;  /**< Graph used in search */
    std::shared_ptr<InternalEdgeCostCalculator> m_costCalculator = std::make_shared<InternalEdgeCostCalculator>();  /**< Edge cost calculator used in search */
    ExcludeVertexFunc m_excludeVertexFunc;  /**< function that will exclude vertices to be used that otherwise would be allowed */
    ExcludeEdgeFunc m_excludeEdgeFunc;  /**< function that will exclude edges to be used that otherwise would be allowed */
    ExcludeVertexFunc m_allowVertexFunc;  /**< function that will allow vertices to be used that otherwise would be excluded */
    ExcludeEdgeFunc m_allowEdgeFunc;  /**< function that will allow edges to be used that otherwise would be excluded */
    size_t m_viaMaxEdges = 2;  /**< Maximum amount of edges used to search for via vertices */
    size_t m_maxCountCloseVts = 2;  /**< Maximum amount of vertices to be used in the search to obtain the best vertices corresponding to start and end poses */
    std::string m_outputFolder = "";  /**< Folder where the planning (search) information will be stored (if empty-string, no data will be saved) */

};

}

#endif // ARO_INFIELDTRACKSCONNECTORGRAPHBASED_HPP
