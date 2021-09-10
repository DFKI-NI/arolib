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
 
#ifndef AROLIB_DIRECTED_GRAPH__H
#define AROLIB_DIRECTED_GRAPH__H

#include <iostream>
#include <fstream>
#include <tuple>
#include <unordered_map>
#include <functional>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <arolib/geometry/geometry_helper.hpp>
#include "arolib/cartography/common.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/types/outfieldinfo.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/types/route.hpp"


using boost::geometry::append;

namespace arolib{

namespace DirectedGraph{

//forward declaring for the typedefs
struct VisitPeriod;
struct vertex_property;
struct edge_property;
struct overroll_property;


typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, DirectedGraph::vertex_property, DirectedGraph::edge_property> boost_directed_graph;
typedef boost::graph_traits<boost_directed_graph>::vertex_descriptor vertex_t;
typedef boost::graph_traits<boost_directed_graph>::edge_descriptor edge_t;
typedef std::pair<vertex_t, vertex_property> vertex_pair;
typedef std::pair<edge_t, edge_property> edge_pair;
typedef boost::graph_traits<boost_directed_graph>::edge_iterator edgeIterator;
typedef std::pair<edgeIterator, edgeIterator> edge_iter;
typedef boost::graph_traits<boost_directed_graph>::vertex_iterator vertexIterator;
typedef std::pair<vertexIterator, vertexIterator> vertex_iter;
typedef boost::graph_traits <boost_directed_graph>::out_edge_iterator out_edge_iterator;
typedef boost::graph_traits <boost_directed_graph>::in_edge_iterator in_edge_iterator;
typedef boost::graph_traits <boost_directed_graph>::adjacency_iterator adj_iterator;
typedef std::pair<DirectedGraph::edge_t, DirectedGraph::overroll_property> EdgeOverrun;


/**
 * @brief Types of edges
 */
enum EdgeType{
    DEFAULT,/**< Deafault (normal) type. Usually used for harvesting edges */
    CROSS,/**< Cross edges. Edges connecting vertices of two different tracks which posses higher costs */
    BOUNDARY_CONN,/**< Connection from/to a field boundary vertex with much higher costs */
    FAP_TO_RP,/**< Edge connecting a field access point and a resource point */
    RP_TO_FAP,/**< Edge connecting a resource point and a field access point */
    FAP_TO_FAP,/**< Edge connecting two field access points */
    INIT/**< Edge connecting the initial point (initial machine position) with valid vertices inside the field */
};

/**
  * @brief Get the EdgeType (enum) from its int value
  * @param value Int value
  * @return EdgeType
  */
EdgeType intToEdgeType(int value);

/**
 * @brief Struct holding the information of the visit (drive over) of a machine to a vertex
 */
struct VisitPeriod{
    MachineId_t machineId = -100;/**< Id of the machine driving over the vertex */
    double time_in = 0;/**< Time(-stamp) the machine enters the vertex 'area' */
    double time_out = -1;/**< Time(-stamp) the machine leaves the vertex 'area' */
    double timestamp = -1;/**< Timestamp of the routepoint corresponding to the visit (time_in <= timestamp <= time_out) */
    std::vector< std::pair<vertex_t, RoutePoint> > next_vt;/**< vector holding the (next) vertices (and corresponding route point) to visit from this vertex */

    /**
     * @brief Default constructor
     */
    explicit VisitPeriod() = default;

    /**
     * @brief Constructor
     * @param _machineId Id of the machine driving over the vertex
     * @param _time_in Time(-stamp) the machine enters the vertex 'area'
     * @param _time_out Time(-stamp) the machine leaves the vertex 'area'
     * @param _timestamp Timestamp of the routepoint corresponding to the visit (time_in <= timestamp <= time_out)
     * @param _next_vt vector holding the (next) vertices (and corresponding route point) to visit from this vertex
     */
    explicit VisitPeriod(MachineId_t _machineId,
                         double _time_in,
                         double _time_out,
                         double _timestamp,
                         std::vector< std::pair<vertex_t, RoutePoint> > _next_vt = {});

    /**
     * @brief Checks if the vertex is busy (not available) at a certain period of time
     *
     * It also gives the time that the machine would have to wait until the vertex is available, a list of machines occupying this vertex at the given time (period), and whether any of the machines that is occupying this vertex at the given time (period) is planned to visit a given vertex from this vertex
     * The period to be checked is given by [time_in, time_in+duration]
     * @param map Map containing the visit periods of the vertex, where the key is the time (-stamp) at which the visit started (i.e. when the machine entered the vertex 'area')
     * @param time_in Timestamp corresponding to the start of the period to be checked
     * @param duration Duration of the period to be checked
     * @param maxMachines Maximum number of machines allowed to be in the vertex at the same time
     * @param [out] waiting Time that the machine would have to wait until the vertex is available (i.e. has space for the machine)
     * @param currentVt Vertex from which the machine wants to access the vertex under consideration
     * @param [out] towardsCurrentVt True if there is a machine located in the vertex under consideration (at any point during the give period [time_in, time_in+duration]) visiting currentVt (used to avoid machines driving over the same edge in opposite directions)
     * @param machineIds Ids of the machines located in (visiting) the vertex under consideration (at any point during the give period [time_in, time_in+duration])
     * @param excludedMachines Ids of the machines to be excluded from the check (the visit periods of these machines will be disregarded)
     * @return True if the vertex is busy (not available) at the given period of time
     */
    static bool isBusy(const std::multimap<double, VisitPeriod> &map,
                       double time_in,
                       double duration,
                       int maxMachines,
                       double &waiting,
                       vertex_t currentVt,
                       bool &towardsCurrentVt,
                       std::vector<MachineId_t> &machineIds,
                       std::set<MachineId_t> excludedMachines = {});


    /**
     * @brief Get the set of machines visiting in a given period of time
     * @param map Map containing the visit periods of the vertex, where the key is the time (-stamp) at which the visit started (i.e. when the machine entered the vertex 'area')
     * @param time_in Timestamp corresponding to the start of the period to be checked
     * @param duration Duration of the period to be checked
     * @param excludedMachines Ids of the machines to be excluded from the check (the visit periods of these machines will be disregarded)
     * @return Set of visiting machines
     */
    static std::set<MachineId_t> getVisitingMachines(const std::multimap<double, VisitPeriod> &map,
                                                     double time_in,
                                                     double duration,
                                                     std::set<MachineId_t> excludedMachines = {});


    /**
     * @brief Check if a machine is visiting in a given period of time
     * @param map Map containing the visit periods of the vertex, where the key is the time (-stamp) at which the visit started (i.e. when the machine entered the vertex 'area')
     * @param machineId Id of the machine
     * @param time_in Timestamp corresponding to the start of the period to be checked
     * @param duration Duration of the period to be checked
     * @return True if the machine is visiting
     */
    static bool isMachineVisiting(const std::multimap<double, VisitPeriod> &map,
                                    MachineId_t machineId,
                                    double time_in,
                                    double time_out);
};

/**
 * @brief Properties of a vertex
 */
struct vertex_property{

    /**
     * @brief Location of the vertex in the graph (inner-field harvesting, headland harvesting, ...)
     * We can consider the graph as two sub-graphs: one corresponding to headland harvesting and the other one corresponding to inner-field harvesting)
     * In some cases, it is not allowed to drive from a HEADLAND vertex to a INFIELD vertex (and viseversa)
     * DEFAULT locations can be understood to belong to both sub-graphs
     */
    enum GraphLocation{
        DEFAULT,/**< Belongs to both 'sub-graphs' */
        HEADLAND,/**< Belongs to the 'headland-harvesting sub-graph' */
        INFIELD/**< Belongs to the 'inner-field-hrvesting sub-graph' */
    };
    static GraphLocation intToGraphLocation(int value);

    RoutePoint route_point;/**< Route-point corresponding to the vertex (for harvesting-routepoints, it corresponds to the route point of the harvester route that was used to generate the vertex) */
    int harvester_id = -1;/**< if the vertex corresponds to a harveting route-point, this is the id of the harvester harvesting this point */
    GraphLocation graph_location = GraphLocation::DEFAULT;/**< location in the graph (sub-graph to which the vertex belongs) */
    std::map< OutFieldInfo::MachineId_t, OutFieldInfo::UnloadingCosts > unloadingCosts;/**< Map holding the unloading costs for each machine for the vertex (only for resource point vertices) */
    std::multimap<double, VisitPeriod> visitPeriods;/**< Map holding the information of the visits to the vertex, where the key is the time corresponding to the start of the visit period (it is a MULTImap to check visits in headland-track during infield harvesting, which can be multiple (i.e. more than one machine can visit this vertices at the same time)) */
};

/**
 * @brief Properties of an edge
 */
struct edge_property{
    Point p0;/**< Point corresponding to the start vertex */
    Point p1;/**< Point corresponding to the end vertex */
    double defWidth = -1;/**< Default working width of the edge */
    double distance = -1;/**< Edge's distance [m] */
    std::vector<overroll_property> overruns;/**< Vector containing the overruns of the edge */
    EdgeType edge_type = EdgeType::DEFAULT;/**< Type of edge */
    std::map< OutFieldInfo::MachineId_t , std::map< OutFieldInfo::MachineBunkerState , OutFieldInfo::TravelCosts > > travelCosts;/**< Map containing edge (travel) costs (only for out-of-field edges corresponding to silo-travel) */
    OutFieldInfo::TravelCosts arrivalCosts;//**< Edge (arrival travel) costs (only for initial-position edges) */
    edge_t revEdge; //**< Holds the reverse edge (when bidirectional) */
    bool bidirectional = false; //**< Is this edge part of a bidirectional connection between 2 vertices? */
    std::map<int, double> customValues;//**< Map holding custom values */

    /**
     * @brief Constructor
     */
    explicit edge_property() = default;

    /**
     * @brief Check if the edge_type is any of the given types
     * @param types Types to check
     * @return types True if the edge_type is any of the given types
     */
    bool isOfType(const std::set<EdgeType>& types);

    /**
     * @brief Get a new key for new custom values
     * @return Key
     */
    static int getNewCustomValueKey();

private:
    static int m_countCustomValKey;//**< Counter for custom values keys */
};

/**
 * @brief Properties of an edge overrun
 */
struct overroll_property{
    int machine_id;/**< Id of the machine driving over the edge */
    double weight;/**< Weight of the machine driving over the edge [Kg] */
    double duration;/**< Time the machine spent driving over the edge [s] */
};


/**
 * @brief Arolib graph
 */
class Graph : public boost_directed_graph , public LoggingComponent{

    friend class GraphDataAccessor;

public:

    /**
      * @brief Struct to get hash value for RoutePoint key
      */
    struct RoutePoint_KeyHash{
        std::size_t operator()(const RoutePoint &rp) const;
        static std::size_t get(const RoutePoint &rp, std::size_t seed = 0);
    };

    /**
      * @brief Struct to check if RoutePoint keys are equal
      */
    struct RoutePoint_KeyEqual{
        bool operator()(const RoutePoint &rp1, const RoutePoint &rp2) const;
    };

    //typedef std::unordered_map<RoutePoint, vertex_t, RoutePoint_KeyHash, RoutePoint_KeyEqual> RoutePoint2VertexMap_t;
    //typedef std::unordered_map<RoutePoint, std::set<vertex_t>, RoutePoint_KeyHash, RoutePoint_KeyEqual> RoutePoint2VertexSetMap_t;
    typedef std::map<RoutePoint, vertex_t> RoutePoint2VertexMap_t;
    typedef std::map<RoutePoint, std::set<vertex_t>> RoutePoint2VertexSetMap_t;

    using VertexFilterFct = std::function<bool(const vertex_t& vt, const vertex_property& vertex_prop)>;/**< Vertex filter function */ //


    /**
      * @brief VertexFilterFct corresponding to no filter
      * @return Always true
      */
    static inline bool NoVertexFilterFct(const vertex_t&, const vertex_property&){return true;}

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit Graph(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Destructor
     */
    virtual ~Graph() = default;


    // copy assignment
    //virtual Graph& operator=(const Graph& other);

    /**
     * @brief Clear the graph data
     */
    void clear();

    /**
     * @brief Add a vertrex to the graph
     * @param vp Vertex property
     * @return Added vertex;
     */
    vertex_t addVertex(const vertex_property& vp);

    /**
     * @brief Add an edge to the graph
     * @param v1 Vertex 1
     * @param v2 Vertex 2
     * @param edgeProp Edge properties
     * @param bidirectional Bidirectional connection between vertices
     * @param onlyIfNotExisting If the edge exists, it is unchanged
     * @param overwrite (if the edge exists) if overwrite=true, the edge_prop is replaced; otherwise a new edge is created;
     * @return true if added;
     */
    bool addEdge(const vertex_t& v1,
                 const vertex_t& v2,
                 const edge_property& edgeProp,
                 bool bidirectional,
                 bool onlyIfNotExisting = true,
                 bool overwrite = false);

    /**
     * @brief Finds a vertex based on the location
     * @param p Point / location
     * @param vertexRPTypes Only take into account vertices with these route-point types. If empty, all types are used
     * @param vertexGraphLocations Only take into account vertices in these graph locations. If empty, all locations are used
     * @return Found vertex. If no vertex found, returns -1
     */
    vertex_t findVertexByLocation(const Point& p,
                                  std::set<RoutePoint::RoutePointType> vertexRPTypes = {},
                                  std::set<vertex_property::GraphLocation> vertexGraphLocations = {}) const;

    /**
     * @brief Finds the closest vertex to a point, laying at a distance < maxDistance
     * @param p Point / location
     * @param maxDistance Distance limit
     * @param vertexRPTypes Only take into account vertices with these route-point types. If empty, all types are used
     * @param vertexGraphLocations Only take into account vertices in these graph locations. If empty, all locations are used
     * @return Found vertex. If no vertex found, returns -1
     */
    vertex_t findClosestVertex(const Point& p,
                               double maxDistance,
                               std::set<RoutePoint::RoutePointType> vertexRPTypes = {},
                               std::set<vertex_property::GraphLocation> vertexGraphLocations = {}) const;


    /**
     * @brief Saves the visit schedule of the graph vertices in a given file (CSV)
     * @param filename File path/name (CSV)
     * @param sep Character used as value separator in the CSV file
     */
    void saveVisitSchedule(const std::string& filename = "/tmp/graph_visit_schedule.csv", char sep = ';') const;

    /**
     * @brief Saves the information of of the graph vertices in a given file (CSV)
     * @param filename File path/name (CSV)
     * @param sep Character used as value separator in the CSV file
     */
    void saveVerticesInfo(const std::string& filename = "/tmp/graph_vertices.csv", char sep = ';') const;

    /**
     * @brief Add an overrun to an edge
     * @param e edge
     * @param overrun Overrun to be added
     * @return True on success
     */
    bool addOverrun(const DirectedGraph::edge_t &e,
                    const DirectedGraph::overroll_property& overrun);

    /**
     * @brief Add an overrun to an edge connecting two given vertices
     * @param v1 Vertex 1
     * @param v2 Vertex 2
     * @param overrun Overrun to be added
     * @return True on success
     */
    bool addOverrun(const DirectedGraph::vertex_t &v1,
                    const DirectedGraph::vertex_t &v2,
                    const DirectedGraph::overroll_property& overrun);

    /**
     * @brief Get the graph's out-of-field information
     * @return out-of-field information
     */
    const OutFieldInfo& outFieldInfo() const;


    /**
     * @brief Removes the visit periods of a set of given machines with a timestamp higher than the given timestamp.
     * @param [in/out] graph Graph to be updated
     * @param machineIds Ids of the machines whose visit periods must be removed
     * @param timestamp Reference timestamp. The visits of this machine that happened after this timestamp will be removed.
     * @return A map with the removed visit periods and the corresponding vertices
     */
    std::map< DirectedGraph::vertex_t, std::vector<DirectedGraph::VisitPeriod> >
            removeVisitPeriodsAfterTimestamp(std::set<MachineId_t> machineIds,
                                             double timestamp);

    /**
     * @brief Get the working width for inner-field harvesting
     * @return Working width for inner-field harvesting (-1 if not set / unknown)
     */
    double workingWidth_IF() const;

    /**
     * @brief Get the working width for headland harvesting
     * @return Working width for headland harvesting (-1 if not set / unknown)
     */
    double workingWidth_HL() const;

    /**
     * @brief Get the maximum infield-track id
     * @return Maximum infield-track id
     */
    int maxTrackId_IF() const;

    /**
     * @brief Get the minimum infield-track id
     * @return Minimum infield-track id
     */
    int minTrackId_IF() const;

    /**
     * @brief Get the maximum headland-track id
     * @return Maximum headland-track id
     */
    int maxTrackId_HL() const;

    /**
     * @brief Get the minimum headland-track id
     * @return Minimum headland-track id
     */
    int minTrackId_HL() const;

    /**
     * @brief Get the map mapping headland-points to their corresponding vertices
     * @return Map mapping headland-points to their corresponding vertices
     */
    std::map<Point, vertex_t>& HLpoint_vertex_map();

    /**
     * @brief Get the map mapping headland-points to their corresponding vertices
     * @return Map mapping headland-points to their corresponding vertices
     */
    const std::map<Point, vertex_t>& HLpoint_vertex_map() const;

    /**
     * @brief Get the map mapping (harvesting) route-points to their corresponding vertices
     * @return Map mapping route-points to their corresponding vertices
     */
    RoutePoint2VertexMap_t& routepoint_vertex_map();

    /**
     * @brief Get the map mapping (harvesting) route-points to their corresponding vertices
     * @return Map mapping route-points to their corresponding vertices
     */
    const RoutePoint2VertexMap_t& routepoint_vertex_map() const;

    /**
     * @brief Get the vertices corresponding to headland processing route points
     * @return Vertices corresponding to headland processing route points
     */
    std::vector<vertex_t>& HLProcessingVertices();

    /**
     * @brief Get the vertices corresponding to headland processing route points
     * @return Vertices corresponding to headland processing route points
     */
    const std::vector<vertex_t>& HLProcessingVertices() const;

    /**
     * @brief Get the map mapping field-access-points to their corresponding vertices
     * @return Map mapping field-access-points to their corresponding vertices
     */
    std::map<FieldAccessPoint, vertex_t>& accesspoint_vertex_map();

    /**
     * @brief Get the map mapping field-access-points to their corresponding vertices
     * @return Map mapping field-access-points to their corresponding vertices
     */
    const std::map<FieldAccessPoint, vertex_t>& accesspoint_vertex_map() const;

    /**
     * @brief Get the map mapping resource-points to their corresponding vertices
     * @return Map mapping resource-points to their corresponding vertices
     */
    std::map<ResourcePoint, vertex_t>& resourcepoint_vertex_map();

    /**
     * @brief Get the map mapping resource-points to their corresponding vertices
     * @return Map mapping resource-points to their corresponding vertices
     */
    const std::map<ResourcePoint, vertex_t>& resourcepoint_vertex_map() const;

    /**
     * @brief Get the map mapping machines' initial-position-points to their corresponding vertices
     * @return Map mapping machines' initial-position-points to their corresponding vertices
     */
    std::map<MachineId_t, vertex_t>& initialpoint_vertex_map();

    /**
     * @brief Get the map mapping machines' initial-position-points to their corresponding vertices
     * @return Map mapping machines' initial-position-points to their corresponding vertices
     */
    const std::map<MachineId_t, vertex_t> &initialpoint_vertex_map() const;


    /**
     * @brief Get the vertices inside a bounding box
     * @param lowerLeftCorner Lower-left corner of the bounding box
     * @param upperRightCorner Upper-right corner of the bounding box
     * @param precise Check for vertices precisely?
     * @param filter Function to filter out vertices in the check (return true if vertex must be taken into account)
     * @return Vertices inside the bounding box
     */
    std::vector<vertex_t> getVerticesInBoundingBox(const Point &lowerLeftCorner, const Point &upperRightCorner, bool precise, const VertexFilterFct &filter = NoVertexFilterFct) const;

    /**
     * @brief Get the vertices inside a bounding box
     * @param center Center point of the bounding box
     * @param width width of the bounding box
     * @param height height of the bounding box
     * @param precise Check for vertices precisely?
     * @param filter Function to filter out vertices in the check (return true if vertex must be taken into account)
     * @return Vertices inside the bounding box
     */
    std::vector<vertex_t> getVerticesInBoundingBox(const Point &center, float width, float height, bool precise, const VertexFilterFct &filter = NoVertexFilterFct) const;

    /**
     * @brief Get the vertices inside a circle with a given center and radius
     * @param center Center point of the circle
     * @param radius radius of the circle
     * @param filter Function to filter out vertices in the check (return true if vertex must be taken into account)
     * @return Vertices inside the circle
     */
    std::vector<vertex_t> getVerticesInRadius(const Point &center, float radius, const VertexFilterFct &filter = NoVertexFilterFct) const;

    /**
     * @brief Get the vertices inside a rectangle formed by a (central) line and a width
     * @param p1 First point of the (central) line
     * @param p2 Second point of the (central) line
     * @param Width Width of the rectancle
     * @param filter Function to filter out vertices in the check (return true if vertex must be taken into account)
     * @return Vertices inside the rectangle
     */
    std::vector<vertex_t> getVerticesInRectangle(Point p1, Point p2, float width, const VertexFilterFct &filter = NoVertexFilterFct) const;


    /**
     * @brief Get the number of vertices in the graph
     * @return Number of vertices in the graph
     */
    int size() const;


    /**
    * @brief Struct holding all the matadata of the graph
    */
   struct GraphMetaData{
       double workingWidth_IF; /**< Working width for inner-field harvesting (-1 if not set / unknown) >*/
       double workingWidth_HL; /**< Working width for headland harvesting (-1 if not set / unknown) >*/
       int maxTrackId_IF; /**< Maximum infield-track id >*/
       int minTrackId_IF; /**< Minimum infield-track id >*/
       int maxTrackId_HL; /**< Maximum headland-track id >*/
       int minTrackId_HL; /**< Minimum headland-track id >*/
       OutFieldInfo outFieldInfo; /**< Out-of-field information (travel costs, etc.) >*/
       std::map<Point, vertex_t> HLpoint_vertex_map; /**< Map mapping headland-points to their corresponding vertices >*/
       RoutePoint2VertexMap_t routepoint_vertex_map; /**< Map mapping route-points to their corresponding vertices (contains only "processing points" (i.e. no headland, field entry/exit or resource points)) >*/
       std::map<FieldAccessPoint, vertex_t> accesspoint_vertex_map; /**< Map mapping field-access-points to their corresponding vertices >*/
       std::map<ResourcePoint, vertex_t> resourcepoint_vertex_map; /**< Map mapping resource-points to their corresponding vertices >*/
       std::map<MachineId_t, vertex_t> initialpoint_vertex_map; /**< Map mapping machines' initial-position-points to their corresponding vertices >*/
       std::vector<vertex_t> HLProcessingVertices; /**< Vertices corresponding to headland-working route-points >*/

       std::map< int, std::map< int, std::set< vertex_t > > > verticesLocationMap;  /**< Map holding the vertices based on their location: <x_location , < y_location, vts > >*/
   };

protected:

    GraphMetaData m_meta; /**< Matadata of the graph >*/

};

/**
 * @brief Class used to access the graph protected/private data if necessary (e.g when building the graph)
 */
class GraphDataAccessor
{
protected:
    /**
     * @brief Get the (meta) data of a graph
     */
    inline static Graph::GraphMetaData& getGraphData(Graph& graph){ return graph.m_meta; }
};

}

inline std::ostream& operator<< (std::ostream &out, const DirectedGraph::Graph& graph) 
{
    out << "graph with " << graph.size() << " vertices";
    return out;
}

}
#endif //AROLIB_DIRECTED_GRAPH__H
