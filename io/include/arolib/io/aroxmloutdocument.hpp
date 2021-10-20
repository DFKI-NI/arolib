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
 
#ifndef AROLIB_IO_AROXMLOUTDOCUMENT_HPP
#define AROLIB_IO_AROXMLOUTDOCUMENT_HPP

#include <ostream>
#include <fstream>
#include <sstream>

#include "xmloutdocument.hpp"
#include "io_common.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/misc/base64Utility.hpp"

namespace arolib {
namespace io {


/**
 * @brief Arolib XML input document for Arolib types
 */
class AroXMLOutDocument : public XMLOutDocument{

public:
    /**
     * @brief Constructor.
     *
     * @param logLevel Log level
     */
    explicit AroXMLOutDocument(LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Destructor.
     */
    virtual ~AroXMLOutDocument();

    /**
      * @brief Set the coordinates (projection) type for the points in the document
      *
      * It must be set before opening the document
      * @param in Cooridinate (projection) type of the source
      * @param out Cooridinate (projection) type of the target
      * @return True on success
      */
    virtual bool setCoordinatesTypes(Point::ProjectionType in, Point::ProjectionType out) override;

    /**
     * @brief Add/write object/value with a given tag.
     *
     * Note: needed because overloading is not managed in inheritance, but we want to use the available (standard) adds from XMLOutDocument
     * @param value Object/value to be written
     * @param tag Tag
     * @return True on success
     */
    template< typename T,
              typename = typename std::enable_if< XMLOutDocument::has_add_method<T>::value, void >::type >
    bool add( const T & value, const std::string& tag ){
        return XMLOutDocument::add(value, tag);
    }

//    template< typename T,
//              typename = typename std::enable_if< ( std::is_arithmetic<T>::value || std::is_enum<T>::value )
//                                                    && !std::is_void<T>::value, void >::type >
//    bool add(const T& value, std::string tag){
//        return XMLOutDocument::add(value, tag);
//    }

//    bool add(const char* value, std::string tag){
//        return XMLOutDocument::add(value, tag);
//    }
//    bool add(const std::string& value, std::string tag){
//        return XMLOutDocument::add(value, tag);
//    }
//    bool add(const std::map<std::string, std::map<std::string, std::string> > &values, std::string tag = ""){
//        return XMLOutDocument::add(values, tag);
//    }




    /**
     * @brief Add/write multiple items with a given tag.
     * @param items Items to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @return True on success
     */
    template< typename T >
    bool add(const std::vector<T>& items, std::string tag){
        return add(&items, tag);
    }


    /**
     * @brief Add/write multiple items with a given tag
     * @param items Items to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @return True on success
     */
    template< typename T ,
              typename = typename std::enable_if< ( !std::is_same<T*, const char*>::value ) >::type >
    bool add(const std::vector<T>* items, const std::string& tag){
        for(auto& item : *items){
            if( !add(item, tag) )
                return false;
        }
        return true;
    }


    /**
     * @brief Add/write an item with a given tag (pair input)
     * @param items Item pair holding the object to be written and the respective tag
     * @return True on success
     */
    template< typename T ,
              typename = typename std::enable_if< ( !std::is_same<T*, const char*>::value ) >::type >
    bool add(const std::pair<T*, std::string>& item){
        return add(*item.first, item.second);
    }


    /**
     * @brief Add/write an item with a given tag (pair input)
     * @param items Item pair holding the object to be written and the respective tag
     * @return True on success
     */
    template< typename T >
    bool add(const std::pair<T, std::string>& item){
        return add(item.first, item.second);
    }


    /**
     * @brief Add/write items with given tags (pair input)
     * @param item Next item to be added (pair holding the object to be written and the respective tag)
     * @param items remaining items to be added
     * @return True on success
     */
    template< typename T, typename... Types >
    bool add(const std::pair<T, std::string> &item, const Types& ... items){
        if(!add(item))
            return false;
        return add(items...);
    }

    /**
     * @brief Add/write a point with a given name and tag.
     * @param pt Point to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const Point& pt, std::string tag = "");

    /**
     * @brief Add/write points with a given name and tag.
     * @param pts Points to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const std::vector<Point>& pts, std::string tag = "");


    /**
     * @brief Add/write a Linestring with a given name and tag.
     * @param ls Linestring to be written
     * @param tag Tag (if empty, no XML-tag will be opened)

     * @return True on success
     */
    bool add(const Linestring& ls, std::string tag = "");

    /**
     * @brief Add/write a Polygon with a given name and tag.
     * @param poly Polygon to be written
     * @param tag Tag (if empty, no XML-tag will be opened)

     * @return True on success
     */
    bool add(const Polygon& poly, std::string tag = "");


    /**
     * @brief Add/write a FieldAccessPoint with a given name and tag.
     * @param pt FieldAccessPoint to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const FieldAccessPoint &pt, std::string tag = "");

    /**
     * @brief Add/write FieldAccessPoints with a given name and tag.
     * @param pts FieldAccessPoints to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const std::vector<FieldAccessPoint>& pts, std::string tag = "");


    /**
     * @brief Add/write a ResourcePoint with a given name and tag.
     * @param pt ResourcePoint to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const ResourcePoint &pt, std::string tag = "");

    /**
     * @brief Add/write ResourcePoints with a given name and tag.
     * @param pts ResourcePoints to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const std::vector<ResourcePoint>& pts, std::string tag = "");

    /**
     * @brief Add/write a set of ResourceTypes with a given name and tag.
     * @param types Set of ResourceTypes to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const std::set<ResourcePoint::ResourceType> types, std::string tag = "");


    /**
     * @brief Add/write a RoutePoint with a given name and tag.
     * @param pt RoutePoint to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const RoutePoint& pt, std::string tag = "");

    /**
     * @brief Add/write RoutePoints with a given name and tag.
     * @param pts RoutePoints to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const std::vector<RoutePoint>& pts, std::string tag = "");


    /**
     * @brief Add/write a HeadlandPoint with a given name and tag.
     * @param pt HeadlandPoint to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const HeadlandPoint &pt, std::string tag = "");

    /**
     * @brief Add/write HeadlandPoints with a given name and tag.
     * @param pts HeadlandPoints to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const std::vector<HeadlandPoint>& pts, std::string tag = "");


    /**
     * @brief Add/write a Track with a given name and tag.
     * @param track Track to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const Track& track, std::string tag = "");

    /**
     * @brief Add/write Tracks with a given name and tag.
     * @param tracks Tracks to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const std::vector<Track>& tracks, std::string tag = "");


    /**
     * @brief Add/write Headlands with a given name and tag.
     * @param headlands Headlands to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const Headlands& headlands, std::string tag = "");

    /**
     * @brief Add/write a complete headland with a given name and tag.
     * @param hl Complete headland to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const CompleteHeadland& hl, std::string tag = "");


    /**
     * @brief Add/write an obstacle with a given name and tag.
     * @param obs Obstacle to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const Obstacle& obs, std::string tag = "");

    /**
     * @brief Add/write Obstacles with a given name and tag.
     * @param obstacles Obstacles to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const std::vector<Obstacle>& obstacles, std::string tag = "");


    /**
     * @brief Add/write a Subfield with a given name and tag.
     * @param sf Subfield to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const Subfield& sf, std::string tag = "");

    /**
     * @brief Add/write a Subfields with a given name and tag.
     * @param subfields Subfields to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const std::vector<Subfield>& subfields, std::string tag = "");

    /**
     * @brief Add/write a Field with a given name and tag.
     * @param field Field to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const Field& field, std::string tag = "");


    /**
     * @brief Add/write a Machine with a given name and tag.
     * @param machine Machine to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const Machine& machine, std::string tag = "");

    /**
     * @brief Add/write Machines with a given name and tag.
     * @param machines Machines to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const std::vector<Machine>& machines, std::string tag = "");


    /**
     * @brief Add/write a Route with a given name and tag.
     * @param route Route to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const Route& route, std::string tag = "");

    /**
     * @brief Add/write Routes with a given name and tag.
     * @param routes Routes to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const std::vector<Route>& routes, std::string tag = "");

    /**
     * @brief Add/write a HeadlandRoute with a given name and tag.
     * @param route HeadlandRoute to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const HeadlandRoute& route, std::string tag = "");

    /**
     * @brief Add/write HeadlandRoutes with a given name and tag.
     * @param routes HeadlandRoutes to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const std::vector<HeadlandRoute>& routes, std::string tag = "");


    /**
     * @brief Add/write MachineDynamicInfo for a machine with a given name and tag.
     * @param dynamicInfo <Machine id, MachineDynamicInfo>-pair to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const std::pair<MachineId_t, MachineDynamicInfo> &dynamicInfo,
             std::string tag = "");

    /**
     * @brief Add/write a map of MachineDynamicInfo for different machines with a given name and tag.
     * @param dynamicInfo <Machine id, MachineDynamicInfo>-map to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const std::map<MachineId_t, MachineDynamicInfo>& dynamicInfo,
             std::string tag = "");


    /**
     * @brief Add/write OutFieldInfo with a given name and tag.
     * @param ofi OutFieldInfo to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const OutFieldInfo& ofi, std::string tag = "");

    /**
     * @brief Add/write a OutFieldInfo-map with a given name and tag.
     * @param map OutFieldInfo-map to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add( const std::map<int, std::map<int, std::map<OutFieldInfo::MachineId_t, std::map<OutFieldInfo::MachineBunkerState, OutFieldInfo::TravelCosts> > > > &map,
              std::string tag = "" );


    /**
     * @brief Add/write an Unloading-Costs map with a given name and tag.
     * @param map Unloading-Costs map to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add( const OutFieldInfo::MapUnloadingCosts_t& map, std::string tag = "" );


    /**
     * @brief Add/write an Arrival-Costs map with a given name and tag.
     * @param map Arrival-Costs map to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add( const OutFieldInfo::MapArrivalCosts_t& map, std::string tag = "" );


    /**
     * @brief Add/write UnloadingCosts with a given name and tag.
     * @param c UnloadingCosts to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const OutFieldInfo::UnloadingCosts& c, std::string tag = "" );


    /**
     * @brief Add/write TravelCosts with a given name and tag.
     * @param c TravelCosts to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const OutFieldInfo::TravelCosts& c,  std::string tag = "" );


    /**
     * @brief Add/write a gridmap with a given name and tag.
     * @param grid Gridmap to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add( const ArolibGrid_t& grid, std::string tag = "" );


    /**
     * @brief Add/write a Graph with a given name and tag.
     * @param graph Graph to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add( const DirectedGraph::Graph& graph, std::string tag = "" );

    /**
     * @brief Add/write a Graph vertex_pair with a given name and tag.
     * @param vt Graph vertex_pair to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const DirectedGraph::vertex_pair &vt, std::string tag = "" );


    /**
     * @brief Add/write a Graph edge_pair with a given name and tag.
     * @param edge Graph edge_pair to be written
     * @param graph Graph to which the edge belongs
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add(const DirectedGraph::edge_pair &edge, const DirectedGraph::Graph &graph, std::string tag = "" );


    /**
     * @brief Add/write an edge with a given name and tag.
     * @param edge Edge to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add( const DirectedGraph::edge_t& edge, std::string tag = "" );


    /**
     * @brief Add/write a Graph vertex_property with a given name and tag.
     * @param v_prop Graph vertex_property to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add( const DirectedGraph::vertex_property& v_prop, std::string tag = "" );


    /**
     * @brief Add/write a Graph edge_property with a given name and tag.
     * @param e_prop Graph edge_property to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add( const DirectedGraph::edge_property& e_prop, std::string tag = "" );


    /**
     * @brief Add/write a Graph VisitPeriod with a given name and tag.
     * @param vp Graph VisitPeriod to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add( const DirectedGraph::VisitPeriod& vp, std::string tag = "" );


    /**
     * @brief Add/write a Graph overroll_property with a given name and tag.
     * @param o Graph overroll_property to be written
     * @param tag Tag (if empty, no XML-tag will be opened)
     * @return True on success
     */
    bool add( const DirectedGraph::overroll_property& o, std::string tag = "" );


    /**
     * @brief Save a field in a XML file
     * @param filename Filename
     * @param field Field to be written
     * @param coordinatesType_in Projection type of the coordinates in the source (field)
     * @param coordinatesType_out Projection type of the coordinates in the target (file)
     * @param OutFieldInfo OutFieldInfo to be written (optional)
     * @return True on success
     */
    static bool saveField( const std::string& filename,
                           const Field& field,
                           Point::ProjectionType coordinatesType_in = Point::UTM,
                           Point::ProjectionType coordinatesType_out = Point::WGS,
                           const OutFieldInfo &outFieldInfo = OutFieldInfo() );

    /**
     * @brief Save fields in a XML file
     * @param filename Filename
     * @param fields Fields to be written
     * @param coordinatesType_in Projection type of the coordinates in the source (field)
     * @param coordinatesType_out Projection type of the coordinates in the target (file)
     * @return True on success
     */
    static bool saveFields( const std::string& filename,
                            const std::vector<Field>& fields,
                            Point::ProjectionType coordinatesType_in = Point::UTM,
                            Point::ProjectionType coordinatesType_out = Point::WGS );

    /**
     * @brief Save machines in a XML file
     * @param filename Filename
     * @param machines Machines to be written
     * @return True on success
     */
    static bool saveMachines( const std::string& filename,
                              const std::vector<Machine>& machines );

    /**
     * @brief Save configuration parameters (given as string map) in a XML file
     * @param filename Filename
     * @param configParameters Configuration parameters (given as string map) to be written
     * @return True on success
     */
    static bool saveConfigParameters( const std::string& filename,
                                      const std::map<std::string, std::map<std::string, std::string> > &configParameters );

    /**
     * @brief Save OutFieldInfo in a XML file
     * @param filename Filename
     * @param outFieldInfo OutFieldInfo to be written
     * @return True on success
     */
    static bool saveOutFieldInfo( const std::string& filename,
                                  const OutFieldInfo &outFieldInfo );

    /**
     * @brief Save plan parameters in a XML file
     * @param filename Filename
     * @param field Field to be written
     * @param workingGroup Working group (machines) to be written
     * @param configParameters Configuration parameters (given as string map) to be written
     * @param OutFieldInfo OutFieldInfo to be written
     * @param machinesDynamicInfo MachineDynamicInfo-map to be written
     * @param coordinatesType_in Projection type of the coordinates in the source
     * @param coordinatesType_out Projection type of the coordinates in the target (file)
     * @return True on success
     */
    static bool savePlanParameters( const std::string& filename,
                                    const Field& field,
                                    const std::vector<Machine>& workingGroup,
                                    const std::map<std::string, std::map<std::string, std::string> > &configParameters,
                                    const OutFieldInfo &outFieldInfo = OutFieldInfo(),
                                    const std::map<MachineId_t, MachineDynamicInfo>& machinesDynamicInfo = {},
                                    Point::ProjectionType coordinatesType_in = Point::UTM,
                                    Point::ProjectionType coordinatesType_out = Point::WGS );

    /**
     * @brief Save plan parameters in a XML file
     * @param filename Filename
     * @param field Field to be written
     * @param workingGroup Working group (machines) to be written
     * @param configParameters Configuration parameters (given as string map) to be written
     * @param OutFieldInfo OutFieldInfo to be written
     * @param machinesDynamicInfo MachineDynamicInfo-map to be written
     * @param yieldmap_tifBase64 Yield-map (base64-encoded) to be written (disregarded if empty)
     * @param drynessmap_tifBase64 Dryness-map (base64-encoded) to be written (disregarded if empty)
     * @param soilmap_tifBase64 Soil-map (base64-encoded) to be written (disregarded if empty)
     * @param remainingAreaMap_tifBase64 Remaining-area-map (base64-encoded) to be written (disregarded if empty)
     * @param coordinatesType_in Projection type of the coordinates in the source
     * @param coordinatesType_out Projection type of the coordinates in the target (file)
     * @return True on success
     */
    static bool savePlanParameters( const std::string& filename,
                                    const std::vector<Machine>& workingGroup,
                                    const std::map<std::string, std::map<std::string, std::string> > &configParameters,
                                    const OutFieldInfo &outFieldInfo = OutFieldInfo(),
                                    const std::map<MachineId_t, MachineDynamicInfo>& machinesDynamicInfo = {},
                                    const std::string &yieldmap_tifBase64 = "",
                                    const std::string &drynessmap_tifBase64 = "",
                                    const std::string &soilmap_tifBase64 = "",
                                    const std::string &remainingAreaMap_tifBase64 = "",
                                    Point::ProjectionType coordinatesType_in = Point::UTM,
                                    Point::ProjectionType coordinatesType_out = Point::WGS);

    /**
     * @brief Save routes in a XML file
     * @param filename Filename
     * @param routes Routes to be written
     * @param coordinatesType_in Projection type of the coordinates in the source
     * @param coordinatesType_out Projection type of the coordinates in the target (file)
     * @return True on success
     */
    static bool savePlan( const std::string &filename,
                          const std::map<int, std::vector<Route> > &routes,
                          Point::ProjectionType coordinatesType_in = Point::UTM,
                          Point::ProjectionType coordinatesType_out = Point::WGS );

    /**
     * @brief Save routes in a XML file
     * @param filename Filename
     * @param routes Routes to be written
     * @param coordinatesType_in Projection type of the coordinates in the source
     * @param coordinatesType_out Projection type of the coordinates in the target (file)
     * @return True on success
     */
    static bool savePlan( const std::string &filename,
                          const std::vector< std::vector<Route> > &routes,
                          Point::ProjectionType coordinatesType_in = Point::UTM,
                          Point::ProjectionType coordinatesType_out = Point::WGS );

    /**
     * @brief Save headland routes in a XML file
     * @param filename Filename
     * @param routes Headland routes to be written
     * @param coordinatesType_in Projection type of the coordinates in the source
     * @param coordinatesType_out Projection type of the coordinates in the target (file)
     * @return True on success
     */
    static bool savePlan( const std::string &filename,
                          const std::map<int, std::vector<HeadlandRoute> > &routes,
                          Point::ProjectionType coordinatesType_in = Point::UTM,
                          Point::ProjectionType coordinatesType_out = Point::WGS );

    /**
     * @brief Save headland routes in a XML file
     * @param filename Filename
     * @param routes Headland routes to be written
     * @param coordinatesType_in Projection type of the coordinates in the source
     * @param coordinatesType_out Projection type of the coordinates in the target (file)
     * @return True on success
     */
    static bool savePlan( const std::string &filename,
                          const std::vector< std::vector<HeadlandRoute> > &routes,
                          Point::ProjectionType coordinatesType_in = Point::UTM,
                          Point::ProjectionType coordinatesType_out = Point::WGS );

    /**
     * @brief Save plan in a XML file
     * @param filename Filename
     * @param field Field to be written
     * @param workingGroup Working group (machines) to be written
     * @param routes Routes to be written
     * @param yieldmap_tifBase64 Yield-map (base64-encoded) to be written (disregarded if empty)
     * @param drynessmap_tifBase64 Dryness-map (base64-encoded) to be written (disregarded if empty)
     * @param soilmap_tifBase64 Soil-map (base64-encoded) to be written (disregarded if empty)
     * @param remainingAreaMap_tifBase64 Remaining-area-map (base64-encoded) to be written (disregarded if empty)
     * @param coordinatesType_in Projection type of the coordinates in the source
     * @param coordinatesType_out Projection type of the coordinates in the target (file)
     * @return True on success
     */
    static bool savePlan( const std::string &filename,
                          const Field& field,
                          const std::vector<Machine>& workingGroup,
                          const std::map<int, std::vector<Route> > &routes,
                          const std::string &yieldmap_tifBase64 = "",
                          const std::string &drynessmap_tifBase64 = "",
                          const std::string &soilmap_tifBase64 = "",
                          const std::string &remainingAreaMap_tifBase64 = "",
                          Point::ProjectionType coordinatesType_in = Point::UTM,
                          Point::ProjectionType coordinatesType_out = Point::WGS );

    /**
     * @brief Save plan in a XML file
     * @param filename Filename
     * @param field Field to be written
     * @param workingGroup Working group (machines) to be written
     * @param routes Routes to be written
     * @param yieldmap_tifBase64 Yield-map (base64-encoded) to be written (disregarded if empty)
     * @param drynessmap_tifBase64 Dryness-map (base64-encoded) to be written (disregarded if empty)
     * @param soilmap_tifBase64 Soil-map (base64-encoded) to be written (disregarded if empty)
     * @param remainingAreaMap_tifBase64 Remaining-area-map (base64-encoded) to be written (disregarded if empty)
     * @param coordinatesType_in Projection type of the coordinates in the source
     * @param coordinatesType_out Projection type of the coordinates in the target (file)
     * @return True on success
     */
    static bool savePlan( const std::string &filename,
                          const Field& field,
                          const std::vector<Machine>& workingGroup,
                          const std::vector< std::vector<Route> > &routes,
                          const std::string &yieldmap_tifBase64 = "",
                          const std::string &drynessmap_tifBase64 = "",
                          const std::string &soilmap_tifBase64 = "",
                          const std::string &remainingAreaMap_tifBase64 = "",
                          Point::ProjectionType coordinatesType_in = Point::UTM,
                          Point::ProjectionType coordinatesType_out = Point::WGS );

    /**
     * @brief Save plan in a XML file
     * @param filename Filename
     * @param field Field to be written
     * @param workingGroup Working group (machines) to be written
     * @param routes Routes to be written
     * @param gridmaps Gridmaps by name
     * @param coordinatesType_in Projection type of the coordinates in the source
     * @param coordinatesType_out Projection type of the coordinates in the target (file)
     * @return True on success
     */
    static bool savePlan( const std::string &filename,
                          const Field& field,
                          const std::vector<Machine>& workingGroup,
                          const std::map<int, std::vector<Route> > &routes,
                          const std::map<std::string, ArolibGrid_t* > &gridmaps,
                          Point::ProjectionType coordinatesType_in = Point::UTM,
                          Point::ProjectionType coordinatesType_out = Point::WGS );

    /**
     * @brief Save plan in a XML file
     * @param filename Filename
     * @param field Field to be written
     * @param workingGroup Working group (machines) to be written
     * @param routes Routes to be written
     * @param gridmaps Gridmaps by name
     * @param coordinatesType_in Projection type of the coordinates in the source
     * @param coordinatesType_out Projection type of the coordinates in the target (file)
     * @return True on success
     */
    static bool savePlan( const std::string &filename,
                          const Field& field,
                          const std::vector<Machine>& workingGroup,
                          const std::vector< std::vector<Route> > &routes,
                          const std::map<std::string, ArolibGrid_t* > &gridmaps,
                          Point::ProjectionType coordinatesType_in = Point::UTM,
                          Point::ProjectionType coordinatesType_out = Point::WGS );

    /**
     * @brief Save plan in a XML file
     * @param filename Filename
     * @param field Field to be written
     * @param workingGroup Working group (machines) to be written
     * @param routes Headland routes to be written
     * @param yieldmap_tifBase64 Yield-map (base64-encoded) to be written (disregarded if empty)
     * @param drynessmap_tifBase64 Dryness-map (base64-encoded) to be written (disregarded if empty)
     * @param soilmap_tifBase64 Soil-map (base64-encoded) to be written (disregarded if empty)
     * @param remainingAreaMap_tifBase64 Remaining-area-map (base64-encoded) to be written (disregarded if empty)
     * @param coordinatesType_in Projection type of the coordinates in the source
     * @param coordinatesType_out Projection type of the coordinates in the target (file)
     * @return True on success
     */
    static bool savePlan( const std::string &filename,
                          const Field& field,
                          const std::vector<Machine>& workingGroup,
                          const std::map<int, std::vector<HeadlandRoute> > &routes,
                          const std::string &yieldmap_tifBase64 = "",
                          const std::string &drynessmap_tifBase64 = "",
                          const std::string &soilmap_tifBase64 = "",
                          const std::string &remainingAreaMap_tifBase64 = "",
                          Point::ProjectionType coordinatesType_in = Point::UTM,
                          Point::ProjectionType coordinatesType_out = Point::WGS );

    /**
     * @brief Save plan in a XML file
     * @param filename Filename
     * @param field Field to be written
     * @param workingGroup Working group (machines) to be written
     * @param routes Headland routes to be written
     * @param yieldmap_tifBase64 Yield-map (base64-encoded) to be written (disregarded if empty)
     * @param drynessmap_tifBase64 Dryness-map (base64-encoded) to be written (disregarded if empty)
     * @param soilmap_tifBase64 Soil-map (base64-encoded) to be written (disregarded if empty)
     * @param remainingAreaMap_tifBase64 Remaining-area-map (base64-encoded) to be written (disregarded if empty)
     * @param coordinatesType_in Projection type of the coordinates in the source
     * @param coordinatesType_out Projection type of the coordinates in the target (file)
     * @return True on success
     */
    static bool savePlan( const std::string &filename,
                          const Field& field,
                          const std::vector<Machine>& workingGroup,
                          const std::vector< std::vector<HeadlandRoute> > &routes,
                          const std::string &yieldmap_tifBase64 = "",
                          const std::string &drynessmap_tifBase64 = "",
                          const std::string &soilmap_tifBase64 = "",
                          const std::string &remainingAreaMap_tifBase64 = "",
                          Point::ProjectionType coordinatesType_in = Point::UTM,
                          Point::ProjectionType coordinatesType_out = Point::WGS );

    /**
     * @brief Save ha Graph in a XML file
     * @param filename Filename
     * @param graph Graph to be written
     * @param coordinatesType_in Projection type of the coordinates in the source
     * @param coordinatesType_out Projection type of the coordinates in the target (file)
     * @return True on success
     */
    static bool saveGraph( const std::string &filename,
                           const DirectedGraph::Graph &graph,
                           Point::ProjectionType coordinatesType_in = Point::UTM,
                           Point::ProjectionType coordinatesType_out = Point::WGS );

protected:

    /**
     * @brief Open document (AroDocument derived class implementation).
     *
     * @return True on success
     */
    virtual bool openDoc() override;
};

}
}//end namespace arolib


#endif //AROLIB_IO_AROXMLOUTDOCUMENT_HPP

