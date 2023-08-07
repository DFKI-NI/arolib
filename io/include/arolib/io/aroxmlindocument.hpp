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
 
#ifndef AROLIB_IO_AROXMLINDOCUMENT_HPP
#define AROLIB_IO_AROXMLINDOCUMENT_HPP

#include <ostream>
#include <fstream>
#include <sstream>
#include <functional>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/optional/optional.hpp>
#include <boost/algorithm/string.hpp>

#include "xmlindocument.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/misc/base64Utility.hpp"
#include "arolib/misc/tuple_helper.h"

namespace arolib {
namespace io {

/**
 * @brief Arolib XML input document for Arolib types
 */
class AroXMLInDocument : public XMLInDocument, public DirectedGraph::GraphDataAccessor{

public:

    /**
     * @brief Constructor.
     *
     * @param logLevel Log level
     */
    explicit AroXMLInDocument(LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Destructor.
     */
    virtual ~AroXMLInDocument();

    /**
     * @brief Read the proyection type for the coordinates from the given base handler.
     * @return True on success.
     */
    bool readCoodinatesInputType(const ReadHandler & base);


    /**
     * @brief Read a point from the given base handler.
     * @param base Base handler.
     * @param [out] pt Point read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, Point &pt );

    /**
     * @brief Read points from the given base handler.
     * @param base Base handler.
     * @param [out] pts Points read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, std::vector<Point> &pts );

    /**
     * @brief Read a Linestring from the given base handler.
     * @param base Base handler.
     * @param [out] ls Linestring read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, Linestring &ls );

    /**
     * @brief Read a Polygon from the given base handler.
     * @param base Base handler.
     * @param [out] poly Polygon read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, Polygon &poly );


    /**
     * @brief Read a field-access point from the given base handler.
     * @param base Base handler.
     * @param [out] pt Field-access point read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, FieldAccessPoint &pt );

    /**
     * @brief Read field-access points from the given base handler.
     * @param base Base handler.
     * @param [out] pts Field-access points read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, std::vector<FieldAccessPoint> &pts );


    /**
     * @brief Read a resource point from the given base handler.
     * @param base Base handler.
     * @param [out] pt Resource point read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, ResourcePoint &pt );

    /**
     * @brief Read resource points from the given base handler.
     * @param base Base handler.
     * @param [out] pts Resource points read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, std::vector<ResourcePoint> &pts );

    /**
     * @brief Read a set of resource types from the given base handler.
     * @param base Base handler.
     * @param [out] resourceTypes Resource types read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, std::set<ResourcePoint::ResourceType> &resourceTypes );


    /**
     * @brief Read a route point from the given base handler.
     * @param base Base handler.
     * @param [out] pt Route point read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, RoutePoint &pt );

    /**
     * @brief Read route points from the given base handler.
     * @param base Base handler.
     * @param [out] pts Route points read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, std::vector<RoutePoint> &pts );


    /**
     * @brief Read a track from the given base handler.
     * @param base Base handler.
     * @param [out] track Track read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, Track &track );

    /**
     * @brief Read tracks from the given base handler.
     * @param base Base handler.
     * @param [out] tracks Tracks read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, std::vector<Track> &tracks );


    /**
     * @brief Read headlands from the given base handler.
     * @param headlands Base handler.
     * @param [out] pt Headlands read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, Headlands &headlands );

    /**
     * @brief Read a complete headland from the given base handler.
     * @param base Base handler.
     * @param [out] hl Complete headland read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, CompleteHeadland &hl );

    /**
     * @brief Read a partial headland from the given base handler.
     * @param base Base handler.
     * @param [out] hl Partial headland read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, PartialHeadland &hl );

    /**
     * @brief Read partial headlands from the given base handler.
     * @param base Base handler.
     * @param [out] headlands Partial headlands read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, std::vector<PartialHeadland> &headlands );


    /**
     * @brief Read a Obstacle from the given base handler.
     * @param base Base handler.
     * @param [out] obs Obstacle read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, Obstacle &obs );

    /**
     * @brief Read Obstacles from the given base handler.
     * @param base Base handler.
     * @param [out] obstacles Obstacles read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, std::vector<Obstacle> &obstacles );


    /**
     * @brief Read a Subfield from the given base handler.
     * @param base Base handler.
     * @param [out] sf Subfield read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, Subfield &sf );

    /**
     * @brief Read Subfields from the given base handler.
     * @param base Base handler.
     * @param [out] sfs Subfields read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, std::vector<Subfield> &sfs );

    /**
     * @brief Read a field from the given base handler.
     * @param base Base handler.
     * @param [out] field Field read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, Field& field );


    /**
     * @brief Read a Machine from the given base handler.
     * @param base Base handler.
     * @param [out] m Machine read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, Machine& m );

    /**
     * @brief Read machines from the given base handler.
     * @param base Base handler.
     * @param [out] machines Machines read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, std::vector<Machine>& machines );


    /**
     * @brief Read a route from the given base handler.
     * @param base Base handler.
     * @param [out] route Route read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, Route& route );

    /**
     * @brief Read routes from the given base handler.
     * @param base Base handler.
     * @param [out] routes Routes read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, std::vector<Route>& routes );

    /**
     * @brief Read MachineDynamicInfo for a machine from the given base handler.
     * @param base Base handler.
     * @param [out] dynamicInfo MachineDynamicInfo read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, std::pair<MachineId_t, MachineDynamicInfo> &dynamicInfo );

    /**
     * @brief Read a map of MachineDynamicInfo from the given base handler.
     * @param base Base handler.
     * @param [out] dynamicInfo Map of MachineDynamicInfo read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, std::map<MachineId_t, MachineDynamicInfo>& dynamicInfo );


    /**
     * @brief Read OutFieldInfo from the given base handler.
     * @param base Base handler.
     * @param [out] ofi OutFieldInfo read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, OutFieldInfo& ofi );

    /**
     * @brief Read a map of OutFieldInfo from the given base handler.
     * @param base Base handler.
     * @param [out] map Map of OutFieldInfo read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base,
               std::map< int , std::map< int , std::map< OutFieldInfo::MachineId_t , std::map< OutFieldInfo::MachineBunkerState , OutFieldInfo::TravelCosts > > > >& map );

    /**
     * @brief Read an Unloading-Costs map from the given base handler.
     * @param base Base handler.
     * @param [out] map Unloading-Costs map  read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, OutFieldInfo::MapUnloadingCosts_t& map );

    /**
     * @brief Read an Arrival-Costs map from the given base handler.
     * @param base Base handler.
     * @param [out] map Arrival-Costs map  read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, OutFieldInfo::MapArrivalCosts_t& map );

    /**
     * @brief Read TravelCosts from the given base handler.
     * @param base Base handler.
     * @param [out] tc TravelCosts read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, OutFieldInfo::TravelCosts& tc );

    /**
     * @brief Read UnloadingCosts from the given base handler.
     * @param base Base handler.
     * @param [out] uc Point UnloadingCosts from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, OutFieldInfo::UnloadingCosts& uc );


    /**
     * @brief Read a map of gridmaps.
     * @param base Base handler.
     * @param [out] map Map of OutFieldInfo read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base,
               std::map< std::string , ArolibGrid_t >& gridmaps );


    /**
     * @brief Read a Graph from the given base handler.
     * @param base Base handler.
     * @param [out] graph Graph read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, DirectedGraph::Graph& graph );

    /**
     * @brief Read vertex data from the given base handler.
     * @param base Base handler.
     * @param [out] vt Vertex data read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, DirectedGraph::vertex_pair &vt );

    /**
     * @brief Read edge data from the given base handler.
     * @param base Base handler.
     * @param [out] edge Edge (string format) read from the base handler
     * @param [out] vt_from Source vertex for the read edge
     * @param [out] vt_to Target vertex for the read edge
     * @param [out] e_prop Graph edge_property read from the base handler
     * @param [out] revEdge Reverse edge for the read edge_property
     * @return True on success.
     */
    bool read( const ReadHandler & base, std::string &edge, DirectedGraph::vertex_t& vt_from, DirectedGraph::vertex_t& vt_to,
                                         DirectedGraph::edge_property& e_prop, std::string &revEdge);

    /**
     * @brief Read a Graph vertex_property from the given base handler.
     * @param base Base handler.
     * @param [out] v_prop Graph vertex_property read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, DirectedGraph::vertex_property& v_prop );

    /**
     * @brief Read a Graph edge_property from the given base handler.
     * @param base Base handler.
     * @param [out] e_prop Graph edge_property read from the base handler
     * @param [out] revEdge Reverse edge for the read edge_property
     * @return True on success.
     */
    bool read( const ReadHandler & base, DirectedGraph::edge_property& e_prop, std::string &revEdge );

    /**
     * @brief Read a Graph VisitPeriod from the given base handler.
     * @param base Base handler.
     * @param [out] vp Graph VisitPeriod read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, DirectedGraph::VisitPeriod& vp );

    /**
     * @brief Read a Graph overroll_property from the given base handler.
     * @param base Base handler.
     * @param [out] o Graph overroll_property read from the base handler
     * @return True on success.
     */
    bool read( const ReadHandler & base, DirectedGraph::overroll_property& o );

    /**
     * @brief Read a Graph meta data.
     * @param base Base handler.
     * @param [out] meta Graph metadata
     * @return True on success.
     */
    bool read(const ReadHandler & base, DirectedGraph::Graph::GraphMetaData& meta, const std::unordered_map<DirectedGraph::vertex_t, DirectedGraph::vertex_t> &verticesMap );

    /**
     * @brief Read object/value from base ReadHandler.
     *
     * Note: needed because overloading is not managed in inheritance, but we want to use the available (standard) reads from XMLInDocument
     * @param base Base handler
     * @param [out] value Object/value read from the base handler
     * @return True on success
     */
    template< typename T,
              typename = typename std::enable_if< XMLInDocument::has_read_method<T>::value, void >::type >
    bool read( const ReadHandler & base, T& value ){
        return XMLInDocument::read(base, value);
    }

//    bool read( const ReadHandler & base, std::string& value){
//        return XMLInDocument::read(base, value);
//    }

//    bool read( const ReadHandler & base,
//               std::map<std::string, std::map<std::string, std::string> > &values ){
//        return XMLInDocument::read(base, values);
//    }

//    template< typename T,
//              typename = typename std::enable_if< std::is_arithmetic<T>::value, void >::type >
//    bool read( const ReadHandler & base, T& value ){
//        return XMLInDocument::read(base, value);
//    }

    //---


    /**
     * @brief Read an object/value with a given tag (and optional parent tags) from the base handler (root) of the document.
     * @param [out] value Object/value read
     * @param tag Expected tag for the value
     * @param parentTags Parent tags to reach the location of the value/tag
     * @return True on success
     */
    template<typename T,
             typename = typename std::enable_if< !std::is_same<T, ReadHandler>::value
                                                 && !std::is_same<T, boost::property_tree::ptree>::value >::type>
    bool read(T& value, const std::string& tag = UseDefaultTag, const std::vector<std::string>& parentTags = {}){

        bool wasOpen = m_isDocOpen;
        if(!wasOpen){
            if(!openDocument()){
                if(isReadyToRead())
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "Cannot open document");
                return false;
            }
        }

        bool resp = read(value, m_base, tag, parentTags);
        if(!wasOpen)
            closeDocument();
        return resp;
    }


    /**
     * @brief Read an object/value with a given tag (and optional parent tags) from a given base handler.
     * @param [out] value Object/value read
     * @param base Base handler
     * @param tag Expected tag for the value
     * @param parentTags Parent tags to reach the location of the value/tag
     * @return True on success
     */
    template<typename T,
             typename = typename std::enable_if< !std::is_same<T, ReadHandler>::value
                                                 && !std::is_same<T, boost::property_tree::ptree>::value >::type>
    bool read(T& value, const ReadHandler & base, std::string tag = UseDefaultTag, const std::vector<std::string>& parentTags = {}){

        if(!m_isDocOpen){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
            return false;
        }

        ReadHandler branch;
        if(!getBranch(base, parentTags, branch))
            return false;

        tag = getTag<T>(tag);
        if(tag.empty()){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid tag");
            return false;
        }

        if(!getBranch(branch, tag, branch)){
            logger().printOut(LogLevel::WARNING, __FUNCTION__, "Cannot open main tag '" + tag + "'");
            return false;
        }
        bool resp = read(branch, value);

        return resp;
    }

    /**
     * @brief Read multiple objects/values with a given tag (and optional parent tags) from the base handler (root) of the document.
     * @param [out] values Objects/values read
     * @param tag Expected tag for the value
     * @param parentTags Parent tags to reach the location of the value/tag
     * @param strict If true, all values under the given tag must be read without errors
     * @return True on success
     */
    template<typename T,
             typename = typename std::enable_if< !std::is_same<T, ReadHandler>::value
                                                 && !std::is_same<T, boost::property_tree::ptree>::value >::type>
    bool readMultiple( std::vector<T>& values, std::string tag = UseDefaultTag, const std::vector<std::string>& parentTags = {}, bool strict = false){

        values.clear();
        bool wasOpen = m_isDocOpen;
        if(!wasOpen){
            if(!openDocument()){
                if(isReadyToRead())
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "Cannot open document");
                return false;
            }
        }

        tag = getTag<T>(tag);
        if(tag.empty()){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid tag");
            return false;
        }

        ReadHandler branch;
        if(!getBranchHandler(parentTags, branch))
            return false;
        size_t count = 0;

        BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, RHTree(branch)){
            if(v.first == tag){
                ++count;
                values.push_back( T() );
                if(!read( createRH(v.second) , values.back())){
                    values.pop_back();
                    if(strict){
                        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error reading element # " + std::to_string(count));
                        return false;
                    }
                    logger().printOut(LogLevel::WARNING, __FUNCTION__, "Error reading element # " + std::to_string(count));
                }
            }
        }

        if(!wasOpen)
            closeDocument();
        return true;
    }


    /**
     * @brief Read multiple objects/values with a given tag from a given base handler.
     * @param base Base handler
     * @param [out] values Objects/values read
     * @param tag Expected tag for the value
     * @param strict If true, all values under the given tag must be read without errors
     * @return True on success
     */
    template<typename T,
             typename = typename std::enable_if< !std::is_same<T, ReadHandler>::value
                                                 && !std::is_same<T, boost::property_tree::ptree>::value >::type>
    bool readMultiple( const ReadHandler & base, std::vector<T>& values, std::string tag = UseDefaultTag, bool strict = false){

        values.clear();

        if(!m_isDocOpen){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
            return false;
        }

        tag = getTag<T>(tag);
        if(tag.empty()){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid tag");
            return false;
        }

        size_t count = 0;
        BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, RHTree(base)){
            if(v.first == tag){
                ++count;
                values.push_back( T() );
                if(!read( createRH(v.second) , values.back())){
                    values.pop_back();
                    if(strict){
                        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error reading element # " + std::to_string(count));
                        return false;
                    }
                    logger().printOut(LogLevel::WARNING, __FUNCTION__, "Error reading element # " + std::to_string(count));
                }
            }
        }
        return true;
    }


    /**
     * @brief Read a field from a (Arolib-formatted) XML file.
     * @param filename Filename
     * @param [out] field Read Field
     * @param coordinatesType_out Projection type for the target coordinates
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readField(const std::string& filename,
                          Field& field,
                          Point::ProjectionType coordinatesType_out = Point::UTM,
                          const std::vector<std::string>& parentTags = {},
                          LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Read a field and OutFieldInfo from a (Arolib-formatted) XML file.
     * @param filename Filename
     * @param [out] field Read Field
     * @param [out] outFieldInfo Read OutFieldInfo
     * @param coordinatesType_out Projection type for the target coordinates
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readField(const std::string& filename,
                          Field& field,
                          OutFieldInfo &outFieldInfo,
                          Point::ProjectionType coordinatesType_out = Point::UTM,
                          const std::vector<std::string>& parentTags = {},
                          LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Read fields from a (Arolib-formatted) XML file.
     * @param filename Filename
     * @param [out] fields Read fields
     * @param coordinatesType_out Projection type for the target coordinates
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readFields(const std::string& filename,
                           std::vector<Field>& fields,
                           Point::ProjectionType coordinatesType_out = Point::UTM,
                           const std::vector<std::string>& parentTags = {},
                           LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Read a Machine from a (Arolib-formatted) XML file.
     * @param filename Filename
     * @param [out] machine Read machine
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readMachine(const std::string& filename,
                            Machine &machine,
                            const std::vector<std::string>& parentTags = {},
                            LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Read machines from a (Arolib-formatted) XML file.
     * @param filename Filename
     * @param [out] machines Read machines
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readMachines(const std::string& filename,
                             std::vector<Machine>& machines,
                             const std::vector<std::string>& parentTags = {},
                             LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Read configuration parameters (as string map) from a (Arolib-formatted) XML file.
     * @param filename Filename
     * @param [out] configParameters Read configuration parameters (as string map)
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readConfigParameters(const std::string& filename,
                                     std::map<std::string, std::map<std::string, std::string> > &configParameters,
                                     const std::vector<std::string>& parentTags = {},
                                     LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Read OutFieldInfo from a (Arolib-formatted) XML file.
     * @param filename Filename
     * @param [out] outFieldInfo Read OutFieldInfo
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readOutFieldInfo(const std::string& filename,
                                 OutFieldInfo &outFieldInfo,
                                 const std::vector<std::string>& parentTags = {},
                                 LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Read resource points from a (arolib-formatted) XML file
     * @param filename File name/path
     * @param [out] resource_points Read resource points
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return true on success
     */
    static bool readResourcePoints(const std::string& filename,
                                   std::vector<ResourcePoint> &resource_points,
                                   const std::vector<std::string>& parentTags = {},
                                   LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Read plan parameters from a (Arolib-formatted) XML file.
     * @param filename Filename
     * @param [out] field Read field
     * @param [out] workingGroup Read working group (machines)
     * @param [out] configParameters Read configuration parameters (as string map)
     * @param [out] outFieldInfo Read OutFieldInfo
     * @param [out] machinesDynamicInfo Read MachineDynamicInfo
     * @param [out] gridmaps Read gridmaps
     * @param coordinatesType_out Projection type for the target coordinates
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readPlanParameters(const std::string& filename,
                                   Field& field,
                                   std::vector<Machine>& workingGroup,
                                   std::map<std::string, std::map<std::string, std::string> > &configParameters,
                                   OutFieldInfo &outFieldInfo,
                                   std::map<MachineId_t, MachineDynamicInfo>& machinesDynamicInfo,
                                   std::map<std::string, ArolibGrid_t> &gridmaps,
                                   Point::ProjectionType coordinatesType_out = Point::UTM,
                                   const std::vector<std::string>& parentTags = {},
                                   LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Read plan parameters from a (Arolib-formatted) XML file.
     * @param filename Filename
     * @param [out] workingGroup Read working group (machines)
     * @param [out] configParameters Read configuration parameters (as string map)
     * @param [out] outFieldInfo Read OutFieldInfo
     * @param [out] machinesDynamicInfo Read MachineDynamicInfo
     * @param [out] gridmaps Read gridmaps
     * @param coordinatesType_out Projection type for the target coordinates
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readPlanParameters(const std::string& filename,
                                   std::vector<Machine>& workingGroup,
                                   std::map<std::string, std::map<std::string, std::string> > &configParameters,
                                   OutFieldInfo &outFieldInfo,
                                   std::map<MachineId_t, MachineDynamicInfo>& machinesDynamicInfo,
                                   std::map<std::string, ArolibGrid_t>& gridmaps,
                                   Point::ProjectionType coordinatesType_out = Point::UTM,
                                   const std::vector<std::string>& parentTags = {},
                                   LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Read plan parameters from a (Arolib-formatted) XML file.
     * @param filename Filename
     * @param [out] field Read field
     * @param [out] workingGroup Read working group (machines)
     * @param [out] configParameters Read configuration parameters (as string map)
     * @param [out] outFieldInfo Read OutFieldInfo
     * @param [out] machinesDynamicInfo Read MachineDynamicInfo
     * @param [out] yieldmap_tifBase64 Read yieldmap (base64-encoded)
     * @param [out] drynessmap_tifBase64 Read drynessmap (base64-encoded)
     * @param [out] soilmap_tifBase64 Read soilmap (base64-encoded)
     * @param [out] remainingAreaMap_tifBase64 Read remaining-area map (base64-encoded)
     * @param coordinatesType_out Projection type for the target coordinates
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readPlanParameters(const std::string& filename,
                                   Field& field,
                                   std::vector<Machine>& workingGroup,
                                   std::map<std::string, std::map<std::string, std::string> > &configParameters,
                                   OutFieldInfo &outFieldInfo,
                                   std::map<MachineId_t, MachineDynamicInfo>& machinesDynamicInfo,
                                   std::string &yieldmap_tifBase64,
                                   std::string &drynessmap_tifBase64,
                                   std::string &soilmap_tifBase64,
                                   std::string &remainingAreaMap_tifBase64,
                                   Point::ProjectionType coordinatesType_out = Point::UTM,
                                   const std::vector<std::string>& parentTags = {},
                                   LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Read plan parameters from a (Arolib-formatted) XML file.
     * @param filename Filename
     * @param [out] workingGroup Read working group (machines)
     * @param [out] configParameters Read configuration parameters (as string map)
     * @param [out] outFieldInfo Read OutFieldInfo
     * @param [out] machinesDynamicInfo Read MachineDynamicInfo
     * @param [out] yieldmap_tifBase64 Read yieldmap (base64-encoded)
     * @param [out] drynessmap_tifBase64 Read drynessmap (base64-encoded)
     * @param [out] soilmap_tifBase64 Read soilmap (base64-encoded)
     * @param [out] remainingAreaMap_tifBase64 Read remaining-area map (base64-encoded)
     * @param coordinatesType_out Projection type for the target coordinates
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readPlanParameters( const std::string& filename,
                                    std::vector<Machine>& workingGroup,
                                    std::map<std::string, std::map<std::string, std::string> > &configParameters,
                                    OutFieldInfo &outFieldInfo,
                                    std::map<MachineId_t, MachineDynamicInfo>& machinesDynamicInfo,
                                    std::string &yieldmap_tifBase64,
                                    std::string &drynessmap_tifBase64,
                                    std::string &soilmap_tifBase64,
                                    std::string &remainingAreaMap_tifBase64,
                                    Point::ProjectionType coordinatesType_out = Point::UTM,
                                    const std::vector<std::string>& parentTags = {},
                                    LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Read a routes from a (Arolib-formatted) XML file.
     * @param filename Filename
     * @param [out] routes Read routes
     * @param syncRoutes Should the routes's base timestamp be synchronized so that all have the same one?
     * @param coordinatesType_out Projection type for the target coordinates
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readPlan( const std::string &filename,
                          std::map<int, std::vector<Route> > &routes,
                          bool syncRoutes = true,
                          Point::ProjectionType coordinatesType_out = Point::UTM,
                          const std::vector<std::string>& parentTags = {},
                          LogLevel logLevel = LogLevel::INFO );


    /**
     * @brief Read plan data from a (Arolib-formatted) XML file.
     * @param filename Filename
     * @param [out] field Read field
     * @param [out] workingGroup Read working group (machines)
     * @param [out] routes Read routes
     * @param [out] yieldmap_tifBase64 Read yieldmap (base64-encoded)
     * @param [out] drynessmap_tifBase64 Read drynessmap (base64-encoded)
     * @param [out] soilmap_tifBase64 Read soilmap (base64-encoded)
     * @param [out] remainingAreaMap_tifBase64 Read remaining-area map (base64-encoded)
     * @param syncRoutes Should the routes's base timestamp be synchronized so that all have the same one?
     * @param coordinatesType_out Projection type for the target coordinates
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readPlan( const std::string &filename,
                          Field& field,
                          std::vector<Machine>& workingGroup,
                          std::map<int, std::vector<Route> > &routes,
                          std::string &yieldmap_tifBase64,
                          std::string &drynessmap_tifBase64,
                          std::string &soilmap_tifBase64,
                          std::string &remainingAreaMap_tifBase64,
                          bool syncRoutes = true,
                          Point::ProjectionType coordinatesType_out = Point::UTM,
                          const std::vector<std::string>& parentTags = {},
                          LogLevel logLevel = LogLevel::INFO );


    /**
     * @brief Read plan data from a (Arolib-formatted) XML file.
     * @param filename Filename
     * @param [out] field Read field
     * @param [out] workingGroup Read working group (machines)
     * @param [out] routes Read routes
     * @param [out] gridmaps Gridmaps by name
     * @param syncRoutes Should the routes's base timestamp be synchronized so that all have the same one?
     * @param coordinatesType_out Projection type for the target coordinates
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readPlan( const std::string &filename,
                          Field& field,
                          std::vector<Machine>& workingGroup,
                          std::map<int, std::vector<Route> > &routes,
                          std::map<std::string, ArolibGrid_t > &gridmaps,
                          bool syncRoutes = true,
                          Point::ProjectionType coordinatesType_out = Point::UTM,
                          const std::vector<std::string>& parentTags = {},
                          LogLevel logLevel = LogLevel::INFO );

    /**
     * @brief Read a Graph from a (Arolib-formatted) XML file.
     * @param filename Filename
     * @param [out] graph Read graph
     * @param coordinatesType_out Projection type for the target coordinates
     * @param parentTags Parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readGraph(const std::string &filename,
                          DirectedGraph::Graph &graph,
                          Point::ProjectionType coordinatesType_out = Point::UTM,
                          const std::vector<std::string>& parentTags = {},
                          LogLevel logLevel = LogLevel::INFO );

    /**
     * @brief Get (general) strict flag.
     * @return (general) strict flag
     */
    bool strict() const;

    /**
     * @brief Set (general) strict flag.
     *
     * If set to false, some of the object properties will be set to the default values if not present in the KML file
     * @param strict (general) strict flag
     */
    void setStrict(bool strict);

protected:
    /**
     * @brief Open document (AroDocument derived class implementation).
     *
     * @return True on success
     */
    virtual bool openDoc() override;

    /**
     * @brief Parse a string holding coordinates to the respective point.
     *
     * @param coordinates Coordinates as string
     * @param [out] pt Output point
     * @return True on success
     */
    bool stringToPoint(std::string coordinates, Point& pt);

    /**
     * @brief Parse a string holding multiple coordinates to the respective points.
     *
     * @param coordinates Coordinates as string
     * @param [out] pts Output points
     * @return True on success
     */
    bool stringToPointList(std::string coordinates, std::vector<Point>& pts);


protected:
    bool m_strict = true; /**< If set to false, some of the object properties will be set to the default values if not present in the KML file. Otherwise, the read methods will fail */
};

}
}//end namespace arolib


#endif //AROLIB_IO_AROXMLINDOCUMENT_HPP

