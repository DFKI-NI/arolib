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
 
#ifndef _AROLIB_GRAPH_BUILDER_TRACKS_BASED_H
#define _AROLIB_GRAPH_BUILDER_TRACKS_BASED_H

#include <iostream>
#include <fstream>
#include <tuple>
#include <ctime>
#include <functional>
#include <unordered_set>

#include <arolib/planning/directedgraph.hpp>
#include <arolib/planning/graphhelper.hpp>
#include <arolib/planning/graph_building_info.hpp>
#include <arolib/geometry/geometry_helper.hpp>
#include <arolib/geometry/field_geometry_processing.hpp>
#include "arolib/types/route.hpp"
#include "arolib/types/outfieldinfo.hpp"
#include "arolib/types/machinedynamicinfo.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/misc/logger.h"
#include "arolib/misc/filesystem_helper.h"


namespace arolib{
namespace DirectedGraph{

/**
 * @brief Class used to build the graph using the processed field, harvester routes (headland and infield harvesting separatelly), etc.
 */
class GraphBuilder_TracksBased : public LoggingComponent, public GraphDataAccessor
{

public:

    /**
     * @brief Constructor
     * @param logLevel Log level
     */
    explicit GraphBuilder_TracksBased(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Destructor
     */
    virtual ~GraphBuilder_TracksBased();


    /**
     * @brief Builds the graph
     * @param subfield Processed subfield
     * @param baseWorkingRoutes Base routes of primary machines
     * @param resource_points Resource/Silo points
     * @param field_access_points Field-access points
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     * @param machines Machines to be used
     * @param machineInitialStates Map containing the initial (current) states of the machines
     * @param workingWidthIF Working width for inner-field harvesting
     * @param workingWidthHL Working width for headland harvesting
     * @param includeVisitPeriods Include visit periods of the base routes? (including them might not be necesary if no collition avoidance will be carried out)
     * @param outputFile File name/path to save the building information (disregarded if empty)
     * @return Built graph
     */
    Graph buildGraph(const Subfield &subfield,
                     const std::vector<Route> &baseWorkingRoutes,
                     const std::vector<ResourcePoint> &resource_points,
                     const std::vector<FieldAccessPoint> &field_access_points,
                     const OutFieldInfo& outFieldInfo,
                     const std::vector<Machine> &machines,
                     const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates,
                     double workingWidthIF,
                     double workingWidthHL,
                     bool includeVisitPeriods = true,
                     const std::string& outputFile = "") const;

private:

    /**
     * @brief Builder workspace containing temporary data
     */
    struct BuilderWorkspace{
        Graph graph;
        std::map<MachineId_t, Machine> machines; /**< Machines */
        std::vector< std::vector<vertex_t> > verticesInfieldTracks; /**< Vertices corresponding to the inner field tracks */
        std::unordered_set< vertex_t > verticesInfieldTracks_set; /**< Vertices corresponding to the inner field tracks */
        std::vector< std::pair<vertex_t, Point> > verticesInfieldTracksEnds; /**< Vertices corresponding to the track ends in the inner field (plus next point in track inside the IF) */
        std::vector< std::vector< std::vector<vertex_t> > > verticesHeadlandTracks; /**< Vertices corresponding to the headland(s) tracks */
        std::unordered_map< vertex_t, size_t >  verticesHeadlandTracks_set; /**< Vertices corresponding to the headland(s) tracks */
        std::unordered_map< FieldAccessPointId_t, std::pair<vertex_t, FieldAccessPoint> > verticesAccessPoints; /**< Vertices corresponding to access points */
        std::unordered_set< vertex_t > verticesInfieldTracks_additionalConnection; /**< Vertices corresponding to the inner field tracks that were not fully connected to the adjascent tracks */
        std::unordered_set <vertex_t > IFVerticesOutsideIF; /**< Vertices corresponding to IF working that actually lay outside of the inner-field boundary */
        std::vector< vertex_t > verticesBoundary; /**< Vertices corresponding to the boundary */
        std::unordered_map< vertex_t, Point > verticesHeadlandToBoundary; /**< Headland vertices to be connected to the boundary */
        std::unordered_map< vertex_t, Point > verticesInfieldToBoundary; /**< InnerField vertices to be connected to the boundary */
        std::vector< std::pair<vertex_t, ResourcePoint> > verticesResourcePoints_FAPConn; /**< Vertices corresponding to the resource points to be connected to access points */
        std::vector< std::pair<vertex_t, ResourcePoint> > verticesResourcePoints_boundaryConn; /**< Vertices corresponding to the resource points to be connected to the boundary */
        std::vector< std::pair<vertex_t, ResourcePoint> > verticesResourcePoints_fieldConn; /**< Vertices corresponding to the resource points to be connected to the track points */
        std::vector< std::pair<vertex_t, MachineId_t> > verticesInitialPos_FAPConn; /**< Vertices corresponding to the initial positions to be connected to access points */
        std::vector< std::pair<vertex_t, MachineId_t> > verticesInitialPos_boundaryConn; /**< Vertices corresponding to the initial positions to be connected to the boundary */
        std::vector< std::pair<vertex_t, MachineId_t> > verticesInitialPos_fieldConn; /**< Vertices corresponding to the initial positions to be connected to the track points */
        Polygon innerBoundary_offset; /**< Offset inner-field boundary */
        bool hasSurroundingHeadland = false; /**< Does the field have a surrounding (complete) headland? */
        GraphBuildingInfoManager buildingInfoManager; /**< Contains the graph-building information (for debug and analysis) */
        std::string outputFile = ""; /**< File path/name to save the graph-building information (for debug and analysis). If = "|" or empty-string, no data will be saved */
    };

    /**
     * @brief Add vertices corresponding to inner-field track-points
     * @param [in/out] ws Workspace
     * @param sf Subfield
     */
    void addInnerFieldTracks(BuilderWorkspace & ws, const Subfield &sf) const;


    /**
     * @brief Add edges interconnecting the vertices corresponding to inner-field track-points
     * @param [in/out] ws Workspace
     * @param sf Subfield
     */
    void interconnectInnerFieldTracks(BuilderWorkspace & ws, const Subfield &sf) const;


    /**
     * @brief Add vertices corresponding to headland track-points
     * @param [in/out] ws Workspace
     * @param sf Subfield
     */
    void addHeadlandTracks(BuilderWorkspace & ws, const Subfield &sf) const;

    /**
     * @brief Add edges interconnecting the vertices corresponding to headland track-points
     * @param [in/out] ws Workspace
     * @param boundary_ Subfield boundary
     */
    void interconnectHeadlandTracks(BuilderWorkspace & ws, const Polygon& boundary_) const;

    /**
     * @brief Add edges connecting the vertices corresponding to inner-field track-points to headland vertices
     * @param [in/out] ws Workspace
     * @param sf Subfield
     */
    void connectInnerFieldTracksToHeadland(BuilderWorkspace & ws, const Subfield& sf) const;

    /**
     * @brief Add vertices corresponding to subfield boundary
     * @param [in/out] ws Workspace
     * @param sf Subfield
     */
    void addBoundaryVertices(BuilderWorkspace & ws, const Subfield& sf) const;

    /**
     * @brief Add edges connecting the vertices corresponding to inner-field track-points to boundary vertices (if applicable)
     * @param [in/out] ws Workspace
     * @param sf Subfield
     */
    void connectInnerFieldTracksToBoundary(BuilderWorkspace & ws, const Subfield& sf) const;


    /**
     * @brief Add edges connecting the headland vertices to boundary vertices
     * @param [in/out] ws Workspace
     */
    void connectHeadlandTracksToBoundary(BuilderWorkspace & ws) const;

    /**
     * @brief Add vertices corresponding to field access points
     * @param [in/out] ws Workspace
     * @param sf Subfield
     * @param field_access_points Field access points
     */
    void addFieldAccessPoints(BuilderWorkspace & ws, const Subfield& sf, const std::vector<FieldAccessPoint> &field_access_points) const;

    /**
     * @brief Add edges connecting the field-access vertices to headland vertices
     * @param [in/out] ws Workspace
     */
    void connectFieldAccessPointsToHeadland(BuilderWorkspace & ws) const;

    /**
     * @brief Add edges connecting the field-access vertices to boundary vertices
     * @param [in/out] ws Workspace
     */
    void connectFieldAccessPointsToBoundary(BuilderWorkspace & ws) const;

    /**
     * @brief Add edges corresponding to interconnections between field access points.
     * @param [in/out] ws Workspace
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     */
    void interconnectFieldAccessPoints(BuilderWorkspace & ws,
                                       const OutFieldInfo &outFieldInfo) const;

    /**
     * @brief Add vertices corresponding to resource points
     * @param [in/out] ws Workspace
     * @param sf Subfield
     * @param resource_points Resource points
     * @param field_access_points Field access points
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     */
    void addResourcePoints(BuilderWorkspace & ws,
                           const Subfield& sf,
                           const std::vector<ResourcePoint>& resource_points,
                           const std::vector<FieldAccessPoint> &field_access_points,
                           const OutFieldInfo& outFieldInfo) const;

    /**
     * @brief Add edges connecting the resource-point vertices to field vertices
     * @param [in/out] ws Workspace
     * @param sf Subfield
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     */
    void connectResourcePoints(BuilderWorkspace & ws, const Subfield& sf, const OutFieldInfo& outFieldInfo) const;

    /**
     * @brief Add edges connecting the resource-point vertices to field-access vertices
     * @param [in/out] ws Workspace
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     */
    void connectResourcePointsToAccessPoints(BuilderWorkspace & ws, const OutFieldInfo& outFieldInfo) const;

    /**
     * @brief Add edges connecting the resource-point vertices to track-point vertices (if the resource points are inside the field)
     * @param [in/out] ws Workspace
     * @param sf Subfield
     */
    void connectResourcePointsToTracks(BuilderWorkspace & ws, const Subfield& sf) const;

    /**
     * @brief Add edges connecting the resource-point vertices to boundary vertices
     * @param [in/out] ws Workspace
     */
    void connectResourcePointsToBoundary(BuilderWorkspace & ws) const;

    /**
     * @brief Add vertices corresponding to the initial locations of the machines
     * @param [in/out] ws Workspace
     * @param sf Subfield
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     * @param machineInitialStates Machines' initial states
     */
    void addInitialPositions(BuilderWorkspace & ws,
                             const Subfield& sf,
                             const OutFieldInfo &outFieldInfo,
                             const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates) const;


    /**
     * @brief Add edges connecting the initial-location vertices to field vertices
     * @param [in/out] ws Workspace
     * @param sf Subfield
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     * @param machineInitialStates Machines' initial states
     */
    void connectInitialPositions(BuilderWorkspace & ws, const Subfield& sf, const OutFieldInfo& outFieldInfo, const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates) const;

    /**
     * @brief Add edges connecting the initial-location vertices to field-access vertices
     * @param [in/out] ws Workspace
     * @param outFieldInfo Out-of-field information (inc. travel costs, etc.)
     * @param machineInitialStates Machines' initial states
     */
    void connectInitialPositionsToAccessPoints(BuilderWorkspace & ws, const OutFieldInfo& outFieldInfo, const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates) const;

    /**
     * @brief Add edges connecting the initial-location vertices to track-point vertices
     * @param [in/out] ws Workspace
     * @param sf Subfield
     */
    void connectInitialPositionsToTracks(BuilderWorkspace & ws, const Subfield& sf) const;

    /**
     * @brief Add edges connecting the initial-location vertices to boundary vertices
     * @param [in/out] ws Workspace
     */
    void connectInitialPositionsToBoundary(BuilderWorkspace & ws) const;

    /**
     * @brief Edit the vertex information of vertices corresponding to points of the ẃorking routes (incl. timestamp)
     * @param [in/out] ws Workspace
     * @param baseWorkingRoutes Routes
     */
    void addBaseRouteInformation(BuilderWorkspace & ws, const std::vector<Route> &baseWorkingRoutes) const;

    /**
     * @brief Add visit period to vertices corresponding to points of the ẃorking routes
     * @param [in/out] ws Workspace
     * @param baseWorkingRoutes Routes
     */
    void addVisitPeriods(BuilderWorkspace & ws, const std::vector<Route> &baseWorkingRoutes) const;

    /**
     * @brief Function used to decide whether a connection between a vertex (located outside the IF boundary) and a (IF) track-end vertex is valid
     * @param vp Vertex-property of the vertex
     * @param vp_trackEnd Vertex-property of the track-end vertex
     * @param pPrev Point in track previous to the track-end
     * @param maxDist Maximum allowed distance bewteeen vertices
     * @param True if valid
     */
    static bool isValid_trackEnds(const vertex_property& vp, const vertex_property& vp_trackEnd, const Point pPrev, double maxDist);

    /**
     * @brief Function used to decide whether a connection between a given HL vertex and a reference IF vertex is valid
     * @param vp Vertex-property of the vertex to be checked
     * @param vp_ref Vertex-property of the reference vertex
     * @param pAdj Point adjacent to the reference vertex
     * @param distThreshold Distance threshold/limit (if the distance between the vertex and the reference vertex is higher -> not valid)
     * @param True if valid
     */
    static bool isValid_HL(const vertex_property& vp, const vertex_property& vp_ref, const Point pAdj, double distThreshold);

    /**
     * @brief Function used to decide whether a connection between a given vertex and a reference vertex is valid based on the perpendicularity of the angle formed by the vertices and another point
     * @param vp Vertex-property of the vertex to be checked
     * @param vp_ref Vertex-property of the reference vertex
     * @param pAdj Point adjacent to the reference vertex
     * @param distThreshold Distance threshold/limit (if the distance between the vertex and the reference vertex is higher -> not valid)
     * @param True if valid
     */
    static bool isValid_perpendicular(const vertex_property& vp, const vertex_property& vp_ref, const Point pAdj, double distThreshold);

    /**
     * @brief Add vertices corresponding to the first point of a route (when such point does not have a corresponding vertex already) and connect it via edges to other vertices
     * @param [in/out] ws Workspace
     * @param rp First route-point
     * @param machine_id Id of the machine for that route
     * @param rp_next Next route-point
     * @param vt_nextRP Vertex corresponding to the next route-point
     */
    void addAndConnectFirstRoutePoint(BuilderWorkspace & ws, const RoutePoint& rp, MachineId_t machine_id, const RoutePoint &rp_next, vertex_t* vt_nextRP) const;

    /**
     * @brief Find the vertex closest to a point which is considered valid
     * @param ws Workspace
     * @param pRef Reference point
     * @param vts Vertices to be considered for the search
     * @param isValid Function used to know if a given vertex should be taken into consideration or disregarded
     * @param [out] vt_out Closest vertex
     * @param [out] vt_out vertex_property of the closest vertex
     * @param [out] _minDist Distance bewteen the reference point and the closest vertex (disregarded if null)
     * @return True on success
     */
    bool findClosestValidVertex(BuilderWorkspace& ws,
                                const Point& pRef,
                                const std::vector<vertex_t>& vts,
                                std::function<bool(const vertex_property&)> isValid,
                                vertex_t& vt_out,
                                vertex_property& vp_out,
                                double* _minDist = nullptr) const;

    /**
     * @brief Find the vertex closest to a point which is considered valid and create an edge connectin that vertex with another given vertex
     * @param [in/out] ws Workspace
     * @param vt_from Vertex to be connected to the closest vertex
     * @param vertex_property vertex_property of the vertex to be connected to the closest vertex
     * @param pRef Reference point
     * @param vts Vertices to be considered for the search
     * @param isValid Function used to know if a given vertex should be taken into consideration or disregarded
     * @param bidirectional Should the connection be bidirectional?
     * @param edgeType Type of the edge to be added
     * @param description Connection description
     * @return True on success
     */
    bool connectToClosestValidVertex(BuilderWorkspace& ws,
                                      vertex_t vt_from,
                                      const vertex_property& vp_from,
                                      const Point& pRef,
                                      const std::vector<vertex_t>& vts,
                                      std::function<bool(const vertex_property&)> isValid,
                                      bool bidirectional,
                                      EdgeType edgeType,
                                      std::string description) const;

    /**
     * @brief Find the vertex closest to a point which is considered valid and create an edge connectin that vertex with another given vertex
     * @param [in/out] ws Workspace
     * @param vt_from Vertex to be connected to the closest vertex
     * @param vertex_property vertex_property of the vertex to be connected to the closest vertex
     * @param pRef Reference point
     * @param vts Vertices to be considered for the search
     * @param isValid Function used to know if a given vertex should be taken into consideration or disregarded
     * @param bidirectional Should the connection be bidirectional?
     * @param edgeType Type of the edge to be added
     * @param description Connection description
     * @return True on success
     */
    bool connectToClosestValidVertex(BuilderWorkspace& ws,
                                      vertex_t vt_from,
                                      const vertex_property& vp_from,
                                      const Point& pRef,
                                      const std::vector< std::vector<vertex_t> >& vts,
                                      std::function<bool(const vertex_property&)> isValid,
                                      bool bidirectional,
                                      EdgeType edgeType,
                                      std::string description) const;

    /**
     * @brief Get the point in a given track closest (yet different) to a given point
     * @param pRef Point (reference)
     * @param tracks Tracks
     * @param track_id Track id
     * @param [out] p_out Point in a given track closest (yet different) to pRef
     * @param [out] min_dist Distance between pRef and p_out
     * @return True if a valid point p_out was found
     */
    bool getClosestPointInTrack(const Point &pRef,
                                const std::vector<Track> &tracks,
                                int track_id,
                                Point &p_out,
                                double &min_dist) const;

    /**
     * @brief Inserts an edge(s) connecting two vertices
     * @note In the case of headland harvesting (headland subgraph), the headland vertices correspond to the (field) boundary points.
     * @param ws Workspace
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
     * @return True if added
     */
    bool addEdge(BuilderWorkspace &ws,
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
                 bool overwrite = false) const;


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
    void addAddedEdgeInfo(BuilderWorkspace &ws,
                          const std::string &added_desc,
                          const RoutePoint &p0,
                          const RoutePoint &p1,
                          EdgeType edge_type,
                          const vertex_t& v_in,
                          const vertex_t& v_out,
                          bool bidirectional) const;

    /**
     * @brief Save the data in the output buffer in the file (for debug and analysis)
     *
     * If ws.outputFile is an empty string, no data will be saved
     */
    void flushOutputBuffer(BuilderWorkspace &ws) const;

};

}

}
#endif //_AROLIB_GRAPH_BUILDER_TRACKS_BASED_H
