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
 
#ifndef ARO_POSEBASEDGRAPH_HPP
#define ARO_POSEBASEDGRAPH_HPP

#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <functional>

#include "arolib/misc/loggingcomponent.h"
#include "arolib/types/pose2D.hpp"
#include "arolib/types/field.hpp"
#include "arolib/types/outfieldinfo.hpp"
#include "arolib/types/machinedynamicinfo.hpp"
#include "arolib/geometry/geometry_helper.hpp"

namespace arolib{

/**
 * @brief Graph used for PoseBased search
 */
class PoseBasedGraph : public LoggingComponent{
public:
    using VertexId_t = size_t;

    /**
     * @brief Vertex
     */
    struct Vertex_t{

        /**
         * @brief Vertex direction
         */
        enum VDirection{
            DIR_PARALLEL,
            DIR_PERPENDICULAR
        };

        //alias for access points
        static constexpr VDirection DIR_FIELD_ENTRY = DIR_PARALLEL;
        static constexpr VDirection DIR_FIELD_EXIT = DIR_PERPENDICULAR;

//        static const VDirection DIR_FIELD_ENTRY;
//        static const VDirection DIR_FIELD_EXIT;

        /**
         * @brief Vertex type
         */
        enum VType{
            TYPE_HL_POINT, /**< Headland point */
            TYPE_IF_POINT, /**< Inner-field point */
            TYPE_ACCESSPOINT, /**< Field access point */
            TYPE_IF_RESOURCE_POINT, /**< Resource point (inside the field) */
            TYPE_OF_RESOURCE_POINT, /**< Resource point (outside the field) */
            TYPE_OF_INITIAL_LOCATION /**< Machine initial location */
        };

        VertexId_t id; /**< Vertex id */
        Pose2D pose; /**< Vertex 2D-pose */
        VDirection direction = DIR_PARALLEL; /**< Vertex direction */
        VType type = TYPE_IF_POINT; /**< Vertex type */

        /**
         * @brief Constructor
         */
        Vertex_t() = default;

        /**
         * @brief Check if the vertex is of an 'out-of-field' type
         * @param includAccessPoints Include Access-points vertices as 'out-of-field' types
         * @return True if the vertex is of an 'out-of-field' type
         */
        bool isOfOutfieldType(bool includAccessPoints) const;

        /**
         * @brief Check if the vertex is of 'field-entry' type
         * @return True if the vertex is of 'field-entry' type
         */
        bool isFieldEntryType() const;

        /**
         * @brief Check if the vertex is of 'field-exit' type
         * @return True if the vertex is of 'field-exit' type
         */
        bool isFieldExitType() const;
    };

    /**
     * @brief Struct holding the out-of.field costs
     */
    struct OutFieldCosts{
        OutFieldInfo::MapMachineStateTravelCosts_t travelCostsMap;/**< Travel costs */
    };

    using MachineOutfieldCostsMap = std::unordered_map<MachineId_t, OutFieldCosts>;

    using VertexFilterFct = std::function<bool(const Vertex_t& vt)>;//return true if vertex must be taken into account


    /**
      * @brief VertexFilterFct corresponding to no filter
      * @return Always true
      */
    static inline bool NoVertexFilterFct(const Vertex_t&){return true;}

    /**
     * @brief Constructor.
     * @param parentLogger Parent logger
     */
    explicit PoseBasedGraph(Logger *_parentLogger = nullptr);

    /**
     * @brief Get subset of the graph based on the given filter.
     * @param [out] sub_ss Sub-graph
     * @param Funtion to filter vertices (if the function returns false, the vertices will not be included in the sub-graph)
     */
    void getSubStateSpace(PoseBasedGraph& sub_ss, const VertexFilterFct& filter) const;

    /**
     * @brief Clear graph data
     */
    void clearAll();

    /**
     * @brief  Get a vertex by its Id
     * @param vt_id Vertex Id
     * @param [out] vt Vertex
     * @return True on success
     */
    bool getVertex(VertexId_t vt_id, Vertex_t &vt) const;

    /**
     * @brief  Get all vertices
     * @param sortById Sort them by Id?
     * @return Vertices
     */
    std::vector<Vertex_t> getVertices(bool sortById = false) const;

    /**
     * @brief Add a track vertex to the graph
     * @param vt Vertex
     * @return True on success
     */
    bool addTrackVertex(Vertex_t &vt);

    /**
     * @brief Add a vertex to the graph corresponding to a machine initial location
     * @param vt Vertex
     * @param machineId Machine Id
     * @return True on success
     */
    bool addInitialLocationVertex(Vertex_t &vt, MachineId_t machineId);

    /**
     * @brief Add a vertex to the graph corresponding to a resource point
     * @param vt Vertex
     * @param resPointId Resource-point Id
     * @return True on success
     */
    bool addResourcePointVertex(Vertex_t &vt, ResourcePointId_t resPointId);

    /**
     * @brief Add a vertex to the graph corresponding to a field access point
     * @param vt Vertex
     * @param accessPointId Access-point Id
     * @return True on success
     */
    bool addAccessPointVertex(Vertex_t &vt, FieldAccessPointId_t accessPointId);

    /**
     * @brief Remove a vertex from the graph
     * @param vt_id Vertex id
     * @return True on success
     */
    bool removeVertex(VertexId_t vt_id);

    /**
     * @brief Create an out-of-field connection between two vertices
     * @param vt_from_id Source vertex id
     * @param vt_to_id Target vertex id
     * @param bidirectional Is the connection bidirectional?
     * @param machinesCostsOF Out-of-field costs
     * @return True on success
     */
    bool connectOFVertices(VertexId_t vt_from_id, VertexId_t vt_to_id, bool bidirectional, const MachineOutfieldCostsMap &machinesCostsOF);


    /**
     * @brief Create a graph from a subfield and 'out-of-field' information
     * @param sf Subfield
     * @param outFieldInfo Out-of-field information
     * @return True on success
     */
    bool create(const Subfield& sf,
                 const OutFieldInfo &outFieldInfo );

    /**
     * @brief Update the machine-arrival (initial locations) vertices and connections
     * @param outFieldInfo Out-of-field information
     * @param machineCurrentStates Machines' current states
     * @param _boundary Field boundary
     * @return True on success
     */
    bool updateArrivalVertices(const OutFieldInfo &outFieldInfo,
                                const std::map<MachineId_t, MachineDynamicInfo>& machineCurrentStates,
                                const Polygon& _boundary);

    /**
     * @brief Check if a vertex with a given id exists
     * @param id Vertex id
     * @return True if exists
     */
    bool vertexIdExists(VertexId_t id) const;


    /**
     * @brief Get the vertex Id-Point map
     * @return Vertex Id-Point map
     */
    const std::unordered_map< VertexId_t, Point >& getVertexId_PointMap() const;

    /**
     * @brief Get the Out-of-field connections
     * @return Out-of-field connections
     */
    const std::unordered_map< VertexId_t, std::unordered_map< VertexId_t, MachineOutfieldCostsMap > >& getOFConnections() const;

    /**
     * @brief Get the vertices inside a bounding box
     * @param lowerLeftCorner Lower-left corner of the bounding box
     * @param upperRightCorner Upper-right corner of the bounding box
     * @param precise Check for vertices precisely?
     * @param filter Function to filter out vertices in the check (return true if vertex must be taken into account)
     * @return Vertices inside the bounding box
     */
    std::vector< Vertex_t > getVerticesInBoundingBox(const Point& lowerLeftCorner, const Point& upperRightCorner, bool precise = true, const VertexFilterFct& filter = NoVertexFilterFct) const;

    /**
     * @brief Get the vertices inside a bounding box
     * @param center Center point of the bounding box
     * @param width width of the bounding box
     * @param height height of the bounding box
     * @param precise Check for vertices precisely?
     * @param filter Function to filter out vertices in the check (return true if vertex must be taken into account)
     * @return Vertices inside the bounding box
     */
    std::vector< Vertex_t > getVerticesInBoundingBox(const Point& center, float width, float height, bool precise = true, const VertexFilterFct& filter = NoVertexFilterFct) const;

    /**
     * @brief Get the vertices inside a circle with a given center and radius
     * @param center Center point of the circle
     * @param radius radius of the circle
     * @param filter Function to filter out vertices in the check (return true if vertex must be taken into account)
     * @return Vertices inside the circle
     */
    std::vector< Vertex_t > getVerticesInRadius(const Point& center, float radius, const VertexFilterFct& filter = NoVertexFilterFct) const;

    /**
     * @brief Get the vertices inside a rectangle formed by a (central) line and a width
     * @param p1 First point of the (central) line
     * @param p2 Second point of the (central) line
     * @param Width Width of the rectancle
     * @param filter Function to filter out vertices in the check (return true if vertex must be taken into account)
     * @return Vertices inside the rectangle
     */
    std::vector< Vertex_t > getVerticesInRectangle(const Point &p1, const Point &p2, float width, const VertexFilterFct &filter = NoVertexFilterFct) const;

    /**
     * @brief Get the vertices closest to a given point
     * @param p Point
     * @param maxVts Maximum amount of vertices to be returned
     * @param filter Function to filter out vertices in the check (return true if vertex must be taken into account)
     * @return Vertices
     */
    std::vector< Vertex_t > getVerticesClosestToPoint(const Point &p, size_t maxVts, const VertexFilterFct &filter = NoVertexFilterFct) const;

    /**
     * @brief Get the vertices corresponding to field-entry points
     * @return Vertices corresponding to field-entry points
     */
    std::unordered_map< FieldAccessPointId_t, std::set<VertexId_t> > getEntryVertices() const;

    /**
     * @brief Get the vertices corresponding to field-exit points
     * @return Vertices corresponding to field-exit points
     */
    std::unordered_map< FieldAccessPointId_t, std::set<VertexId_t> > getExitVertices() const;

    /**
     * @brief Get the vertices corresponding to field entry and exit points
     * @param entryVts Vertices corresponding to field-exit points
     * @param exitVts Vertices corresponding to field-exit points
     */
    void getEntryAndExistVertices( std::unordered_map< FieldAccessPointId_t, std::set<VertexId_t> >& entryVts,
                                   std::unordered_map< FieldAccessPointId_t, std::set<VertexId_t> >& exitVts ) const;

    /**
     * @brief Get the vertices corresponding to resource points outside of the field
     * @return Vertices corresponding to resource points outside of the field
     */
    const std::unordered_map< ResourcePointId_t, VertexId_t >& getOFResourcePointVertices() const;

    /**
     * @brief Get the vertices corresponding to machine initial locations outside of the field
     * @return Vertices corresponding to machine initial locations outside of the field
     */
    const std::unordered_map< MachineId_t, VertexId_t >& getOFInitialLocationVertices() const;

    /**
     * @brief Get the vertices corresponding to field access point with a given vertex id
     * @param fap_vt_id Vertex id
     * @param [out] id Access-point id
     * @return True if found
     */
    bool getAccessPointId(VertexId_t fap_vt_id, FieldAccessPointId_t& id) const;

    /**
     * @brief Get the vertices corresponding to resource point with a given vertex id
     * @param resP_vt_id Vertex id
     * @param [out] id Resource-point id
     * @return True if found
     */
    bool getResourcePointId(VertexId_t resP_vt_id, ResourcePointId_t& id) const;

    /**
     * @brief Get the vertices corresponding to an initial location with a given vertex id
     * @param initLoc_vt_id Vertex id
     * @param [out] id Resource-point id
     * @return True if found
     */
    bool getMachineIdFromInitLoc(VertexId_t initLoc_vt_id, MachineId_t& id) const;

    /**
     * @brief Get the bounding box containing all vertices
     * @param [out] lowerLeftCorner Lower-left corner of the bounding box
     * @param [out] upperRightCorner Upper-right corner of the bounding box
     * @return True on success
     */
    bool getVerticesBoundingBox(Point& lowerLeftCorner, Point& upperRightCorner) const;

protected:
    /**
     * @brief Add tracks' vertices
     * @param tracks Tracks
     * @param vType Vertex type
     */
    void addTracksVertices(const std::vector<Track>& tracks, Vertex_t::VType vType);

    /**
     * @brief Add track vertices
     * @param points Track points
     * @param vType Vertex type
     */
    void addTrackVertices(std::vector<Point> points, Vertex_t::VType vType);

    /**
     * @brief Add a track-point vertex
     * @param pPrev Previous track point
     * @param p Track point (coresponding to the vertex to be added)
     * @param pNext Next track point
     * @param vType Vertex type
     */
    void addTrackVertex(const Point &pPrev, const Point &p, const Point &pNext, Vertex_t::VType vType);

    /**
     * @brief Add vertices for field access points
     * @param faps Field access points
     * @param outFieldInfo Out-of-field information
     * @param boundary Field boundary
     */
    void addAccessPointsVertices(std::vector<FieldAccessPoint> faps, const OutFieldInfo &outFieldInfo, Polygon boundary);

    /**
     * @brief Add vertices for resource points located outside of the field
     * @param sf Subfield
     * @param outFieldInfo Out-of-field information
     */
    void addResourcePointsVertices(const Subfield &sf, const OutFieldInfo &outFieldInfo);

    /**
     * @brief Add a vertex for resource point located inside of the field
     * @param sf Subfield
     * @param resP Resource point
     */
    void addIFResourcePointVertices(const Subfield &sf, const ResourcePoint& resP);

    /**
     * @brief Add vertices for the initial machine locations
     * @param outFieldInfo Out-of-field information
     * @param machineCurrentStates Machine current states
     * @param boundary Field boundary
     */
    void addInitialLocationsVertices(const OutFieldInfo &outFieldInfo, const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates, const Polygon& boundary);

protected:
    size_t m_idCount = 0; /**< Vertex id counter */
    std::map< int, std::map< int, std::map<VertexId_t, Vertex_t> > > m_vertices;/**< Map holding the vertices based on their location: <x_location , < y_location, < vertex_id, vertex > > >*/
    std::unordered_map< VertexId_t, Point > m_verticesIds;/**< Map holding the vertices ids and exact location: < vertex_id, location > >*/
    std::unordered_map< VertexId_t, std::unordered_map< VertexId_t, MachineOutfieldCostsMap > > m_edges_OF;/**< Map holding the out-of-field connections between vertices >*/
    std::unordered_map< MachineId_t, VertexId_t > m_verticesInitialLoc;/**< Map holding the vertices for machine initial locations: < machine_id , vertex_id > >*/
    std::unordered_map< ResourcePointId_t, VertexId_t > m_verticesResourcePoints;/**< Map holding the vertices for resource points: < resource_point_id , vertex_id > >*/
    std::unordered_map< FieldAccessPointId_t, std::unordered_set<VertexId_t> > m_verticesAccessPoints;/**< Map holding the vertices for field access points: < access_point_id , vertex_id > >*/
};

}

#endif // ARO_POSEBASEDGRAPH_HPP
