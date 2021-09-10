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
 
#ifndef AROLIB_GRAPH_HELPER_HPP
#define AROLIB_GRAPH_HELPER_HPP

#include <functional>

#include <arolib/misc/basic_responses.h>
#include <arolib/types/machine.hpp>
#include <arolib/types/machinedynamicinfo.hpp>
#include <arolib/planning/astarPlan.hpp>

namespace arolib {

/**
 * @brief Get the vertex of a graph located closest to a given point
 * @param p Point
 * @param graph Graph
 * @return Nearest vertex
 */
DirectedGraph::vertex_t find_nearest_vertex(const Point &p, const DirectedGraph::Graph &graph);

/**
 * @brief Get the list of vertices, belonging to a given track, nearest to a given vertex
 * @param graph Graph
 * @param vt Reference vertex
 * @param track_id Track id
 * @param [out] List of output vertices
 * @param maxVertices Maximum amount of vertices in the aoutput list (if =0 -> no limit)
 * @param tolerance If the diference between the minimum distance and a vertex distance is lower that this tolerance, the vertex will be added to the list
 * @return True on success
 */
bool searchNearestVerticesOnTrack(const DirectedGraph::Graph &graph,
                                  const DirectedGraph::vertex_t vt,
                                  const int &track_id,
                                  std::vector<DirectedGraph::vertex_t> &nearestVertices,
                                  size_t maxVertices = 0,
                                  double tolerance = 1e-5);

/**
 * @brief Get the list of vertices, belonging to a given track, nearest to a given vertex
 * @param graph Graph
 * @param vertexSet Set of vertices to be checked
 * @param vt Reference vertex
 * @param track_id Track id
 * @param [out] List of output vertices
 * @param maxVertices Maximum amount of vertices in the aoutput list (if =0 -> no limit)
 * @param tolerance If the diference between the minimum distance and a vertex distance is lower that this tolerance, the vertex will be added to the list
 * @return True on success
 */
bool searchNearestVerticesOnTrack(const DirectedGraph::Graph &graph,
                                  const std::vector<DirectedGraph::vertex_t> vertexSet,
                                  const DirectedGraph::vertex_t vt,
                                  const int &track_id,
                                  std::vector<DirectedGraph::vertex_t> &nearestVertices,
                                  size_t maxVertices = 0,
                                  double tolerance = 1e-5);

/**
 * @brief Get the list of vertices adjascent (i.e. connected) to a given vertex
 * @param graph Graph
 * @param vt Reference vertex
 * @param towards_vt If true, the vertices will be source vertices only; otherwise they will be target vertices only
 * @param exclude Set of vertices to be excluded from the serach
 * @return List of resulting vertices
 */
std::vector<DirectedGraph::vertex_t> getAdjacentVertices(const DirectedGraph::Graph &graph,
                                                         const DirectedGraph::vertex_t vt,
                                                         bool towards_vt,
                                                         std::set<DirectedGraph::vertex_t> exclude = {});

/**
 * @brief Get the list of vertices closest to a given vertex, from the previous and next tracks, and (if applicable) the headland
 * @param graph Graph
 * @param vt Reference vertex
 * @param tolerance If the diference between the minimum distance and a vertex distance is lower that this tolerance, the vertex will be added to the list
 * @return List of resulting vertices
 */
std::vector<DirectedGraph::vertex_t> getClosestVertices(const DirectedGraph::Graph &graph,
                                                        const DirectedGraph::vertex_t vt,
                                                        double tolerance);

/**
 * @brief Get the list of vertices adjascent (i.e. connected) closest to a given vertex, from the previous and next tracks, and (if applicable) the headland
 * @param graph Graph
 * @param vt Reference vertex
 * @param tolerance If the diference between the minimum distance and a vertex distance is lower that this tolerance, the vertex will be added to the list
 * @param exclude Set of vertices to be excluded from the serach
 * @return List of resulting vertices
 */
std::vector<DirectedGraph::vertex_t> getClosestAdjacentVertices(const DirectedGraph::Graph &graph,
                                                                const DirectedGraph::vertex_t vt,
                                                                bool towards_vt,
                                                                double tolerance);


/**
 * @brief Get the list of x vertices in the graph that are valid and closest to the given point
 * @param graph Graph containing the necesary info
 * @param p Reference point
 * @param validityFunction Function used to know if a vertex is valid
 * @param maxVts maximum ampount of vertices to be returned
 * @return Vertices in the graph that are located inside the given circle
 */
std::vector<DirectedGraph::vertex_t> getClosestValidVertices(const DirectedGraph::Graph &graph, const Point& p, const DirectedGraph::Graph::VertexFilterFct &validityFunction, size_t maxVts = 1);


/**
 * @brief Generate the visit period of the vertex corresponding to a given route point
 *
 * The visit period is generated based on the timestamps of the previous and next route-points to the given route-point (if applicable)
 * @param machineId Id of the machine
 * @param route_points Route-points
 * @param machineRadius Machine 'working radius'
 * @param index Index of the route-point to be processed
 * @param graph (optional) Graph with the necesarry meta information to get vertices and edges information. If NULL, some of the variables related to vertices and edges (e.g. nextVt) are not included.
 * @param _logger (optional) Pointer to the parent logger (to use the parent's logLevel). If NULL, it creates a default logger.
 * @return Generated visit period of the vertex corresponding to a given route point
 */
DirectedGraph::VisitPeriod getVisitPeriod(MachineId_t machineId,
                                          const std::vector<RoutePoint>& route_points,
                                          double machineRadius,
                                          size_t index,
                                          const DirectedGraph::Graph *graph = nullptr,
                                          Logger *_logger = nullptr);

/**
 * @brief From a vector of vertices, get the vertex in the graph that is nearest to a given point.
 * @param graph Graph
 * @param goal_point Reference point
 * @param v List of vertices to check
 * @return Closest vertex in the input list
 */
DirectedGraph::vertex_t calcNearestVertexToPoint(const DirectedGraph::Graph &graph,
                                                 const Point &goal_point,
                                                 const std::vector<DirectedGraph::vertex_t>& v);



/**
 * @brief Update proximity map and visiting periods of the vertices overlaping a route segment
 * @param graph
 * @param machine
 * @param route_points
 * @param ind_0 Index of the first route point of the segment
 * @param ind_n Index of the last route point of the segment (inclusive)
 * @return Set with overlaping vertices
 */
std::set<DirectedGraph::vertex_t> updateVisitingPeriods(DirectedGraph::Graph &graph,
                                                                       const Machine& machine,
                                                                       const std::vector<RoutePoint>& route_points,
                                                                       size_t ind_0,
                                                                       size_t ind_n);



/**
 * @brief Insert the vertices corresponding to the machines' initial positions into the graph and connects the to valid  vertices in both sub-graphs (headland and infield subgraphs)
 *
 * @note This vertices will be of type HEADLAND, whereas the vertices from the route-points of headland harvesting are not of type HEADLAND (they are treaten as inner-field harvesting vertices)
 * @param [in/out] graph Graph to be expanded/updated
 * @param subfield Processed subfield
 * @param machines Machines to be used
 * @param machineType Insert the initial-position corresponding to machines of this type
 * @param machineInitialStates Map containing the initial (current) states of the machines
 * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
 * @param field_bounday Field boundary
 * @param edgeSoilValueCalc function to compute the edge soil value (const Point& p1, const Point& p2, double width, bool& error)
 * @param connectInHeadlandSubGraph Connect the vertices to valid ('headland' and harvesting) vertices located in the headland subgraph?
 * @param connectInInfieldSubGraph Connect the vertices to valid ('headland' and harvesting) vertices located in the infield subgraph?
 * @param locateMissingMachinesAtFAP Connect the machines with no current location to a random access point?
 */
AroResp insertOrReplaceInitialPositions(DirectedGraph::Graph & graph,
                                        const Subfield &subfield,
                                        const std::vector<Machine> &machines,
                                        const std::set<Machine::MachineType>& machineTypes,
                                        const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates,
                                        const OutFieldInfo& outFieldInfo,
                                        const Polygon &field_bounday,
                                        bool connectInHeadlandSubGraph = true,
                                        bool connectInInfieldSubGraph = true,
                                        bool locateMissingMachinesAtFAP = false);



}
#endif //AROLIB_GRAPH_HELPER_HPP
