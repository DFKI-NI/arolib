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
 
#ifndef _AROLIB_GRAPH_BUILDER_ROUTES_BASED_H
#define _AROLIB_GRAPH_BUILDER_ROUTES_BASED_H

#include <iostream>
#include <fstream>
#include <tuple>
#include <ctime>
#include <functional>
#include <unordered_set>

#include <arolib/planning/directedgraph.hpp>
#include <arolib/planning/graphhelper.hpp>
#include <arolib/planning/planningworkspace.h>
#include <arolib/planning/graph_building_info.hpp>
#include <arolib/geometry/geometry_helper.hpp>
#include <arolib/geometry/field_geometry_processing.hpp>
#include "arolib/cartography/common.hpp"
#include "arolib/types/headlandroute.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/types/outfieldinfo.hpp"
#include "arolib/types/machinedynamicinfo.hpp"
#include "arolib/misc/logger.h"
#include "arolib/types/route.hpp"
#include "aro_functions.hpp"


namespace arolib{
namespace DirectedGraph{

/**
 * @brief Class used to build the graph using the processed field, harvester routes (headland and infield harvesting separatelly), etc.
 */
class GraphBuilder_RoutesBased : public LoggingComponent, public GraphDataAccessor, protected PlanningWorkspaceAccessor
{

public:

    /**
     * @brief Constructor
     * @param bePreciseWithMaps Perform map (grid) operations preciselly
     * @param logLevel Log level
     */
    explicit GraphBuilder_RoutesBased(const bool& bePreciseWithMaps = true, const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Destructor
     */
    virtual ~GraphBuilder_RoutesBased(){}


    /**
     * @brief Builds the graph
     * @param subfield Processed subfield
     * @param harvInfieldRoutes Harvester routes for inner-field harvesting
     * @param harvHeadlandRoutes Harvester routes for headland harvesting
     * @param resource_points Resource/Silo points
     * @param field_access_points Field-access points
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     * @param machines Machines to be used
     * @param machineInitialStates Map containing the initial (current) states of the machines
     * @param workingWidth Working width for inner-field harvesting
     * @param workingWidthHL Working width for headland harvesting
     * @param soilmap Soil-state map/grid
     * @param includeVisitPeriods Include visit periods of the vertices? (including them might not be necesary if no collition avoidance will be carried out)
     * @param field_bounday Field boundary
     * @return Built graph
     */
    Graph buildGraph(const Subfield &subfield,
                     const std::vector<Route> &harvInfieldRoutes,
                     const std::vector<HeadlandRoute> &harvHeadlandRoutes,
                     const std::vector<ResourcePoint> &resource_points,
                     const std::vector<FieldAccessPoint> &field_access_points,
                     const OutFieldInfo& outFieldInfo,
                     const std::vector<Machine> &machines,
                     const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates,
                     double workingWidth,
                     double workingWidthHL,
                     bool includeVisitPeriods = true,
                     const Polygon& field_bounday = Polygon());

    /**
     * @brief Builds the graph
     * @param pw Planning workspace containing all needed information (processed subfield(s), machines, maps/grids, etc.)
     * @param subfieldIdx Index of the subfield to be used to build the graph
     * @param harvInfieldRoutes Harvester routes for inner-field harvesting
     * @param harvHeadlandRoutes Harvester routes for headland harvesting
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     * @param workingWidth Working width for inner-field harvesting
     * @param workingWidthHL Working width for headland harvesting
     * @param includeVisitPeriods Include visit periods of the vertices? (including them might not be necesary if no collition avoidance will be carried out)
     * @param field_bounday Field boundary
     * @return Built graph
     */
    Graph buildGraph(PlanningWorkspace& pw,
                     size_t subfieldIdx,
                     const std::vector<Route> &harvInfieldRoutes,
                     const std::vector<HeadlandRoute> &harvHeadlandRoutes,
                     const OutFieldInfo& outFieldInfo,
                     double workingWidth,
                     double workingWidthHL,
                     bool includeVisitPeriods = true,
                     const Polygon &field_bounday = Polygon());

    /**
     * @brief Set the output file's path/name for debug and analysis
     * @param filename File name/path where the gragh-building information will be stored (if empty-string, no data will be saved)
     */
    void setOutputFile(const std::string& filename);

private:

    /**
     * @brief Expand the graph with the harvester routes for inner-field harvesting
     * @param [in/out] graph Graph to be expanded/updated
     * @param subfield Processed subfield
     * @param machinesMap Map with the machines to be used (<machine id, machine>)
     * @param routes Harvester routes for inner-field harvesting
     * @param resource_points Resource/Silo points
     * @param field_access_points Field-access points
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     * @param workingWidth Working width for inner-field harvesting
     * @param includeVisitPeriods Include visit periods of the vertices? (including them might not be necesary if no collition avoidance will be carried out)
     * @param baseTimestampInfield Base timestamp for infield routes (i.e 'real' timestamp for the first routepoint, i.e. when the inner-field harvesting starts)
     * @param default_yield_proportion Default yield-proportion [t/ha]
     * @param field_bounday Field boundary
     * @param default_soil_value Default soil-value
     * @return Built graph
     */
    void expandGraph(Graph &graph,
                     const Subfield &subfield,
                     const std::vector<Route> &routes,
                     const std::vector<ResourcePoint>& resource_points,
                     const std::vector<FieldAccessPoint>& field_access_points,
                     const OutFieldInfo& outFieldInfo,
                     double workingWidth,
                     double baseTimestampInfield = -1,
                     const Polygon& field_bounday = Polygon());

    /**
     * @brief Expand the graph with the harvester routes for headland harvesting
     * @param [in/out] graph Graph to be expanded/updated
     * @param subfield Processed subfield
     * @param machinesMap Map with the machines to be used (<machine id, machine>)
     * @param routes Harvester routes for headland harvesting
     * @param resource_points Resource/Silo points
     * @param field_access_points Field-access points
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     * @param workingWidth Working width for inner-field harvesting
     * @param includeVisitPeriods Include visit periods of the vertices? (including them might not be necesary if no collition avoidance will be carried out)
     * @param default_yield_proportion Default yield-proportion [t/ha]
     * @param field_bounday Field boundary
     * @param default_soil_value Default soil-value
     * @return Built graph
     */
    void expandGraph(Graph &graph,
                     const Subfield &subfield,
                     const std::vector<HeadlandRoute> &routes,
                     const std::vector<ResourcePoint>& resource_points,
                     const std::vector<FieldAccessPoint>& field_access_points,
                     const OutFieldInfo& outFieldInfo,
                     double workingWidth,
                     const Polygon& field_bounday = Polygon());

    /**
     * @brief Update the graph with the inner-field harvesting-routepoints (i.e. insert the vertices and edges corresponding to (inner-field) harvesting routepoints)
     *
     * Contains only "harvesting points" (i.e. no headland, field entry/exit or resource points)
     * @param [in/out] graph Graph to be expanded/updated
     * @param subfield Processed subfield
     * @param routes Harvester routes for inner-field harvesting
     * @param machinesMap Map with the machines to be used (<machine id, machine>)
     * @param includeVisitPeriods Include visit periods of the vertices? (including them might not be necesary if no collition avoidance will be carried out)
     * @param default_yield Default yield-proportion [t/ha]
     */
    void insertRoutes(Graph & graph,
                      const Subfield &subfield,
                      const std::vector<Route> &routes);

    /**
     * @brief Update the graph with the headland harvesting-routepoints (i.e. insert the vertices and edges corresponding to (headland) harvesting routepoints)
     *
     * Contains only "harvesting points" (i.e. no boundary, field entry/exit or resource points)
     * @param [in/out] graph Graph to be expanded/updated
     * @param routes Harvester routes for headland harvesting
     * @param machinesMap Map with the machines to be used (<machine id, machine>)
     * @param includeVisitPeriods Include visit periods of the vertices? (including them might not be necesary if no collition avoidance will be carried out)
     * @param default_yield Default yield-proportion [t/ha]
     */
    void insertRoutes(Graph & graph,
                      const std::vector<HeadlandRoute> &routes);

    /**
     * @brief Adds edges between nearest inner-field vertices (corresponding to harvesting-routepoints) on adjacent infield tracks
     * @param [in/out] graph Graph to be expanded/updated
     * @param subfield Processed subfield
     */
    void interconnectTracks(Graph & graph, const Subfield &subfield);

    /**
     * @brief Adds edges between nearest headland-harvesting vertices (corresponding to harvesting-routepoints) on adjacent headland tracks
     * @param [in/out] graph Graph to be expanded/updated
     */
    void interconnectTracksHL(Graph & graph);

    /**
     * @brief Inserts the vertices corresponding to field access points into the graph and connects them to nearest vertex in the headland track (for inner-field harvesting).
     * @param [in/out] graph Graph to be expanded/updated
     * @param field_access_points Field-access points to be inserted
     */
    void insertConnectFieldAccessPoints(Graph & graph,
                                        const std::vector<FieldAccessPoint> &field_access_points);

    /**
     * @brief Inserts the vertices corresponding to connections between field access points.
     * @param [in/out] graph Graph to be expanded/updated
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     */
    void interconnectFieldAccessPoints(Graph & graph,
                                       const OutFieldInfo &outFieldInfo);


    /**
     * @brief Inserts the vertices corresponding to resource points into the graph and connets them to the field access points or to the nearest 'headland' vertex.
     *
     * @note in the case of headland harvesting (hadland sub-graph), the 'headland' points correspond to the boundary points, and not to the route-points in the headland tracks
     * Nornally, the resource points located outside the field are connected to field-access-points vertices. If a resource point is inside the field or if no field-access points are given, the resource points must be connected directly to a suitable 'headland' vertex
     * @param [in/out] graph Graph to be expanded/updated
     * @param resource_points Resource/Silo points to be inserted
     * @param field_access_points Field-access points (the corresponding vertices should have been added aready)
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     * @param field_bounday Field boundary
     */
    void insertConnectResourcePoints(Graph & graph,
                                     const std::vector<ResourcePoint>& resource_points,
                                     const std::vector<FieldAccessPoint> &field_access_points,
                                     const OutFieldInfo &outFieldInfo,
                                     const Polygon &field_bounday);

    /**
     * @brief Inserts a vertex corresponding to an infield resource point or an access point into the graph and connets it to nearest 'headland' vertex.
     *
     * @note In the case of headland harvesting (hadland sub-graph), the 'headland' vertex correspond to the boundary points, and not to the route-points in the headland tracks
     * @note Normally this method is called for field-access points, but in the cases where they are not given or that the resource points lay inside the field, the resource points must be connected directly to a suitable 'headland' vertex
     * @param [in/out] graph Graph to be expanded/updated
     * @param point The point that should be inserted.
     * @param isResourcePoint If true, the pointg will be added as a resource point; if false, as an field-access point.
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     */
    void insertConnectAccessPointOrInfieldResourcePoint(Graph & graph,
                                                        const Point &point,
                                                        const bool &isResourcePoint,
                                                        const OutFieldInfo &outFieldInfo );

    /**
     * @brief Inserts the vertices coresponding to the resource points that are located outside of the field into the graph and connets them to the vertices corresponding to field-access points.
     * @param [in/out] graph Graph to be expanded/updated
     * @param resource_points Resource/Silo points to be inserted
     * @param field_access_points Field-access points (the corresponding vertices should have been added aready)
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     */
    void insertConnectOutfieldResourcePoints( Graph & graph,
                                              const std::vector<ResourcePoint> &resource_points,
                                              const std::vector<FieldAccessPoint>& field_access_points,
                                              const OutFieldInfo &outFieldInfo );

    /**
     * @brief Inserts the vertices coresponding to the resource points that are located outside of the field into the graph and connets them to the vertices corresponding to field-access points.
     * @param [in/out] graph Graph to be expanded/updated
     * @param resource_points Resource/Silo points to be inserted
     * @param field_access_points Field-access points (the corresponding vertices should have been added aready)
     */
    void insertConnectOutfieldResourcePoints(Graph & graph,
                                             const std::vector<ResourcePoint> &resource_points,
                                             const OutFieldInfo::MapUnloadingCosts_t& unloadingCostsMap);

    /**
     * @brief Inserts the vertices coresponding to the points of the headland tracks (of inner-field harvesting) and connects them to existing (harvesting route-points) infield vertices.
     * @param [in/out] graph Graph to be expanded/updated
     * @param subfield Subfield
     * @param baseTimestampInfield Base timestamp for infield routes (i.e 'real' timestamp for the first routepoint, i.e. when the inner-field harvesting starts). Disregarded if = -1.
     *
     */
    void insertHeadlandPoints(Graph & graph,
                              const Subfield &subfield,
                              const double &baseTimestampInfield = -1);

    /**
     * @brief Inserts the vertices coresponding to the points of the headland middle track and connects them to existing (harvesting route-points) infield vertices.
     * @param [in/out] graph Graph to be expanded/updated
     * @param track Headland middle track
     * @param baseTimestampInfield Base timestamp for infield routes (i.e 'real' timestamp for the first routepoint, i.e. when the inner-field harvesting starts). Disregarded if = -1.
     *
     */
    bool insertHeadlandMiddleTrack(Graph & graph,
                                   Polygon track,
                                   const double &baseTimestampInfield = -1);

    /**
     * @brief Connects the HL and IF vertices corresponding to processing route points.
     * @param [in/out] graph Graph to be expanded/updated     *
     */
    void connectHLandIFTracks(Graph & graph);

    /**
     * @brief Get the headland boundary points (i.e. the points iver the field boundary to be used during headland-harvesting).
     * @param [in/out] graph Graph to be expanded/updated
     * @param routes Harvester routes for headland harvesting
     * @param field_bounday Field bounday
     * @param [out] boundary_points Headland boundary points (i.e. the points iver the field boundary to be used during headland-harvesting).
     */
    void getHeadlandBoundaryPoints(const Graph &graph,
                                   const std::vector<HeadlandRoute> &routes,
                                   const Polygon &field_bounday,
                                   std::vector<Point> &boundary_points);

    /**
     * @brief Insert the vertices corresponding to the headland boundary points (i.e. the points in the field boundary to be used during headland-harvesting) into the graph and connect then to the vertices of the first headland track.
     *
     * @note This vertices will be of type HEADLAND, whereas the vertices from the route-points of headland harvesting are not of type HEADLAND (they are treaten as inner-field harvesting vertices)
     * @param [in/out] graph Graph to be expanded/updated
     * @param boundary_points Headland boundary points (i.e. the points iver the field boundary to be used during headland-harvesting) to be added.
     */
    void insertHeadlandBoundaryPoints(Graph & graph,
                                      const std::vector<Point> &boundary_points);

    /**
     * @brief Insert the vertices and corresponding edges corresponding to the connection between the headland sub-graph and the infield sub-graph.
     *
     * @note At the moment it is not necessary because the only way the OLV connect the 2 subgraphs is while following the harvester, which is done directly without graph-based planning
     * @param [in/out] graph Graph to be expanded/updated
     * @param harvHeadlandRoutes Harvester routes for headland harvesting
     * @param harvInfieldRoutes Harvester routes for inner-field harvesting
     */
    void insertInterGraphConnection(Graph & graph,
                                    const std::vector<HeadlandRoute> &harvHeadlandRoutes,
                                    const std::vector<Route> &harvInfieldRoutes);

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
     * @param connectInHeadlandSubGraph Connect the vertices to valid ('headland' and harvesting) vertices located in the headland subgraph?
     * @param connectInInfieldSubGraph Connect the vertices to valid ('headland' and harvesting) vertices located in the infield subgraph?
     */
    void insertInitialPositions(Graph & graph,
                                const Subfield &subfield,
                                const std::vector<Machine> &machines,
                                const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates,
                                const OutFieldInfo& outFieldInfo,
                                const Polygon &field_bounday,
                                bool connectInHeadlandSubGraph = true,
                                bool connectInInfieldSubGraph = true);


    /**
     * @brief Get the 'working radius' of a machine
     *
     * This is used for things such as collision avoidance, and is calculated based on the geometry of the machine
     * @param machinesMap Map with the machines to be used (<machine id, machine>)
     * @param machineId Machine id
     * @return Machine's working radius (<1 if invalid radius or machine not in map)
     */
    double getMachineRadius(const std::map<MachineId_t, Machine>& machinesMap, MachineId_t machineId);

    /**
     * @brief Add the vertices' proximity map to the graph metadata
     *
     * The ProximityMap maps the route-points to their corresponding nearby vertices for each machine
     * @param [in/out] graph Graph to be expanded/updated
     * @param machinesMap Map with the machines to be used (<machine id, machine>)
     * @param harvHeadlandRoutes Harvester routes for headland harvesting
     * @param harvInfieldRoutes Harvester routes for inner-field harvesting
     * @param includeVisitPeriods Include visit periods of the vertices? (including them might not be necesary if no collition avoidance will be carried out)
     */
    void addVisitPeriods(Graph & graph,
                          const std::map<MachineId_t, Machine>& machinesMap,
                          const std::vector<HeadlandRoute> &harvHeadlandRoutes,
                          const std::vector<Route> &harvInfieldRoutes);

    /**
     * @brief Expand or reduce the map/grid to fit the given boundary
     * @param [in/out] map map/grid to be updated
     * @param boundary Boundary to adjust the maps
     * @param default_value Default cell value
     * @return True on success
     */
    bool adjustMap(ArolibGrid_t& map, const Polygon& boundary, const double& default_value);

    /**
     * @brief Inserts a vertex connesponding to a 'headland' point into the graph.
     *
     * In the case of headland harvesting (headland subgraph), the headland points/vertices correspond to the (field) boundary points
     * @param [in/out] graph Graph to be expanded/updated
     * @param hp 'Headland' points to be inserted
     * @param timestamp Timestamp to be set to the route-point related to the inserted vertex
     * @return The inserted vertex.
     */
    vertex_t addHeadlandPoint(Graph &graph, const Point &hp, const double &timestamp = -1);

    /**
     * @brief Inserts the edge(s) connecting a 'headland' vertex to other vertex in the field.
     *
     * @note In the case of headland harvesting (headland subgraph), the headland vertices correspond to the (field) boundary points.
     * @param [in/out] graph Graph to be expanded/updated
     * @param u ('headland') vertex to be connected.
     * @param v Second vertex to be connected.
     * @param edgeType Edge type.
     * @param bidirectional Is the connection bidirectionsl? If true, two edges will be created: one from u to v, and another one from v to u.
     */
    void insertHeadlandEdges(Graph & graph,
                             const vertex_t &u,
                             const vertex_t &v,
                             EdgeType edgeType = EdgeType::DEFAULT,
                             bool bidirectional = true);

    /**
     * @brief Inserts an edge(s) connecting two vertices
     * @note In the case of headland harvesting (headland subgraph), the headland vertices correspond to the (field) boundary points.
     * @param [in/out] graph Graph to be expanded/updated
     * @param distance Edge distance.
     * @param yield Yield-mass under the edge [Kg].
     * @param edgeType Edge type.
     * @param vt1 First vertex to be connected.
     * @param vt2 Second vertex to be connected.
     * @param rp1 Route-point corresponding to the first vertex to be connected (vt1).
     * @param rp2 Route-point corresponding to the second vertex to be connected (vt2).
     * @param width Edge width.
     * @param addedTag Tag indicating 'when' was the edge inserted (for debug and analysis).
     * @param bidirectional Is the connection bidirectionsl? If true, two edges will be created: one from u to v, and another one from v to u.
     * @param onlyIfNotExisting If the edge exists, it is unchanged
     * @param overwrite (if the edge exists) if overwrite=true, the edge_prop is replaced; otherwise a new edge is created;
     * @return true if added;
     */
    bool addEdge(Graph &graph,
                 double distance,
                 EdgeType edgeType,
                 const vertex_t &vt1,
                 const vertex_t &vt2,
                 const RoutePoint &rp1,
                 const RoutePoint &rp2,
                 double width,
                 const std::string &addedTag,
                 bool bidirectional = true,
                 bool onlyIfNotExisting = true,
                 bool overwrite = false);

    /**
     * @brief Get the route-point corresponding to the vertex (in a given track) closest to a given route point
     * @param rpRef Route point (reference)
     * @param trackVertices Vertices (and their properties) belonging to the track to be checked
     * @param [out] rp_out Route-point corresponding to the vertex (in trackVertices) closest to the given route point (rpRef)
     * @param [out] min_dist Distance between rpRef and rp_out
     * @return True if a valid route-point rp_out was found
     */
    bool getClosestPointInTrack(const RoutePoint& rpRef,
                                const std::vector< std::pair<vertex_t, vertex_property> > &trackVertices,
                                RoutePoint& rp_out,
                                double & min_dist);

    /**
     * @brief Get the valid headland vertex (for infield-harvesting) closest to a given route point
     *
     * This methods is used to find suitable vertices corresponding to the headland track points (in infield-harvesting) to be connected to (harvesting) vertices in the inner-field that are not connected to any vertices in the previous/next infield track
     * @param rpRef Route point (reference)
     * @param rpInTrack Route point belonging to the same (infield) track as the one of rpRef, used to check the validity of the headland vertex
     * @param HLVertices Vertices corresponding to the headland track (for infield-harvesting) ( <vertex, <route-point, track> > )
     * @param subfield Processed subfield
     * @param workingWidth Working width, used to check the validity of the headland vertex
     * @param [out] vt_out Vertex (in HLVertices) closest to the given route point (rpRef)
     * @param [out] rp_out Route-point corresponding to the vertex (in HLVertices) closest to the given route point (rpRef)
     * @param [out] min_dist Distance between rpRef and rp_out/vt_out
     * @return True if a valid headland vertex was found
     */
    bool getClosestValidHeadlandVertex(const Point &pRef,
                                       const Point &pInTrack,
                                       int track_id,
                                       const std::map<vertex_t, std::pair< RoutePoint, std::set<int> > >& HLVertices,
                                       const Subfield &subfield,
                                       double workingWidth,
                                       double angleLimit, //deg
                                       vertex_t& vt_out,
                                       RoutePoint &rp_out,
                                       double & min_dist);



    /**
     * @brief Resample the headland tracks perpendicularily based on the infield tracks
     * @param [in/out] sf Subfield
     * @param resolution Sampling resolution
     */
    bool sampleHLTracksPerpendicularly(const Subfield &sf, std::vector<Track> &tracksHL, double resolution);

    /**
     * @brief Add vertices and edges of the headland tracks (which do not correspond to HL processing routes)
     * @param graph Graph
     * @param [in/out] sf Subfield
     */
    void connectHLTracks(DirectedGraph::Graph& graph, const Subfield &sf, std::vector<Track> &tracksHL);


    /**
     * @brief Clear some data
     */
    void clear();

    /**
     * @brief Append a edge to the output buffer (for debug and analysis)
     * @param added Tag indicating at what point was the edge added to the graph
     * @param p0 Route point corresponding to the (start) vertex connected by the edge
     * @param p1 Route point corresponding to the (end) vertex connected by the edge
     * @param edge_type Edge type
     * @param v_in (start) vertex connected by the edge
     * @param v_out (end) vertex connected by the edge
     */
    void appendToOutputBuffer(const std::string &added_desc,
                              const RoutePoint &p0,
                              const RoutePoint &p1,
                              EdgeType edge_type,
                              const vertex_t& vt_from,
                              const vertex_t& vt_to, bool bidirectional);

    /**
     * @brief Save the data in the output buffer in the file (for debug and analysis)
     *
     * If m_outputFile is an empty string, no data will be saved
     */
    void flushOutputBuffer();

private:

    /**
     * @brief Type of sub-gragh being processed
     */
    enum CurrentGraphType{
        GRAPH_HL, /**< Headland sub-gragh */
        GRAPH_IF /**< Inner-field sub-gragh */
    };


    /**
     * @brief Flag used internally to know whether the grid/map values should be calculated using the Planning Workspace or not
     */
    enum CalcGridValueOption{
        CALC_DIRECT,
        CALC_PW
    };

    bool m_bePreciseWithMaps; /**< Perform map (grid) operations preciselly */
    CurrentGraphType m_currentGraphType; /**< Type of sub-gragh being currently processed */
    CalcGridValueOption m_calcGridValueOption; /**< Flag used internally to know whether the grid/map values should be calculated Planning Workspace or not */
    std::map<int, std::map<vertex_t, Point>> m_track_endpoints_map; /**< Map of the vertices corresponding to TRACK_START and TRACK_END routepoints of each inner-field track. <track id, <vertex, location> > */
    std::vector< std::pair<vertex_t, RoutePoint> > m_verticesFirstLastTracks; /**< Vertices belonging to the first and last tracks (for both headland and innerfield harvesting). In the case of infield-harvesting, it does not include TRACK_START and TRACK_END vertices. In the case of headland harvesting, it only includes vertices from the first (outmost) headland track (at least for now) */

    std::set<vertex_t> m_IFVerticesOutsideIF; /**< Vertices corresponding to IF working that actually lay outside of the inner-field boundary */
    PlanningWorkspace *m_planningWorkspace = nullptr;/**< Pointer to the planning workspace (if NULL, no methods with a planning workspace as parameter were called) */
    std::map<MachineId_t,double> m_currentWorkedMass; /**< Current harvested mass per machine */

    std::string m_outputFile = ""; /**< File path/name to save the graph-building information (for debug and analysis). If = "|" or empty-string, no data will be saved */
    GraphBuildingInfoManager m_buildingInfoManager; /**< Contains the graph-building information (for debug and analysis) */

};

}

}
#endif //_AROLIB_GRAPH_BUILDER_ROUTES_BASED_H
