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
 
#include "arolib/io/aroxmloutdocument.hpp"

namespace arolib {
namespace io {

namespace {

template< typename T,
          typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type >
std::string to_single_string( const std::vector<T> & values, char sep = ';' ){
    std::string str_data;
    if(values.empty())
        return str_data;
    for(auto val : values){
        if(std::is_floating_point<T>::value)
            str_data += double2string(val) + sep;
        else
            str_data += std::to_string(val) + sep;
    }
    str_data.pop_back();
    return str_data;
}

template< typename T,
          typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type >
std::string to_single_string( const std::set<T> & values, char sep = ';' ){
    std::string str_data;
    if(values.empty())
        return str_data;
    for(auto val : values)
        if(std::is_floating_point<T>::value)
            str_data += double2string(val) + sep;
        else
            str_data += std::to_string(val) + sep;
    str_data.pop_back();
    return str_data;
}

}


AroXMLOutDocument::AroXMLOutDocument(LogLevel logLevel):
    XMLOutDocument(logLevel)
{
}

AroXMLOutDocument::~AroXMLOutDocument()
{
}

bool AroXMLOutDocument::setCoordinatesTypes(Point::ProjectionType in, Point::ProjectionType out)
{
    if(m_isDocOpen){
        logger().printError(__FUNCTION__, "Cannot set the coordinates' types when the document is already open");
        return false;
    }
    m_coordinatesType_in = in;
    m_coordinatesType_out = out;
    return true;
}

bool AroXMLOutDocument::add(const Point &pt, std::string tag) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    tag = getTag<std::vector<Point>>(tag);
    if(!tag.empty()){
        tabs();
        *m_os << "<" << tag << ">";
    }

    try{
        if(m_coordinatesType_in == m_coordinatesType_out)
            *m_os <<pt.x<<","<<pt.y<<","<<pt.z;
        else if(m_coordinatesType_in == Point::UTM){
            Point pt_geo = pt;
            arolib::CoordTransformer::GetInstance().convert_to_geodetic(pt, pt_geo);
            *m_os <<pt_geo.x<<","<<pt_geo.y<<","<<pt_geo.z;
        }
        else{
            Point pt_utm = pt;
            arolib::CoordTransformer::GetInstance().convert_to_cartesian(pt, pt_utm);
            *m_os <<pt_utm.x<<","<<pt_utm.y<<","<<pt_utm.z;
        }
    }
    catch(...){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Exception cought. point = " + pt.toString(10));
        ok = false;
    }

    if(!tag.empty())
        *m_os << "</" << tag << ">\n";
    else
        *m_os << " ";
    return ok;
}

bool AroXMLOutDocument::add(const std::vector<Point> &pts, std::string tag) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;
//    if(pts.empty())
//        return true;
    tag = getTag<std::vector<Point>>(tag);
    if(!tag.empty()){
        openTag(tag);
        tabs();
    }
    for(auto &p : pts){
        ok &= add(p);
        if(!ok)
            break;
    }
    if(!tag.empty()){
        *m_os << std::endl;
        closeTag();
    }
    return ok;
}

bool AroXMLOutDocument::add(const Linestring &ls, std::string tag) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;
//    if(ls.points.empty())
//        return true;
    tag = getTag<Linestring>(tag);
    if(!tag.empty())
        openTag(tag);
    ok &= add(ls.id, "id");
    ok &= add(ls.points, UseDefaultTag);
    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const Polygon &poly, std::string tag) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;
//    if(poly.points.empty())
//        return true;
    tag = getTag<Polygon>(tag);
    if(!tag.empty())
        openTag(tag);
    ok &= add(poly.points, UseDefaultTag);
    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const FieldAccessPoint &pt, std::string tag) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    tag = getTag<FieldAccessPoint>(tag);
    if(!tag.empty())
        openTag(tag);
    ok &= add(pt.id, "id");
    ok &= add(pt.point(), UseDefaultTag);
    ok &= add(pt.accessType, UseDefaultTag);
    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const std::vector<FieldAccessPoint> &pts, std::string tag) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    if(pts.empty())
        return true;
    tag = getTag<std::vector<FieldAccessPoint>>(tag);
    if(!tag.empty())
        openTag(tag);
    for(auto &p : pts){
        ok &= add(p, UseDefaultTag);
        if (!ok)
            break;
    }
    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const ResourcePoint &pt, std::string tag) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    tag = getTag<ResourcePoint>(tag);
    if(!tag.empty())
        openTag(tag);
    ok &= add(pt.id, "id");
    ok &= add(pt.point(), UseDefaultTag);
    ok &= add(pt.resourceTypes, UseDefaultTag);
    ok &= add(pt.defaultUnloadingTime, "defaultUnloadingTime");
    ok &= add(pt.defaultUnloadingTimePerKg, "defaultUnloadingTimePerKg");
    ok &= add(pt.geometry, "geometry");
    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const std::vector<ResourcePoint> &pts, std::string tag) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    if(pts.empty())
        return true;
    tag = getTag<std::vector<ResourcePoint>>(tag);
    if(!tag.empty())
        openTag(tag);
    for(auto &p : pts){
        ok &= add(p, UseDefaultTag);
        if(!ok)
            break;
    }
    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const std::set<ResourcePoint::ResourceType> types, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    if(types.empty())
        return true;
    tag = getTag<std::set<ResourcePoint::ResourceType>>(tag);

    std::string sTypes;
    for(auto &t : types)
        sTypes += ( std::to_string( (int)t ) + "," );
    sTypes.pop_back();

    add(sTypes, tag);

    return ok;

}

bool AroXMLOutDocument::add(const RoutePoint &pt, std::string tag) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    tag = getTag<RoutePoint>(tag);
    if(!tag.empty())
        openTag(tag);
    ok &= add(pt.point(), UseDefaultTag);
    ok &= add(pt.bunker_mass, "bunker_mass");
    ok &= add(pt.bunker_volume, "bunker_volume");
    ok &= add(pt.worked_mass, "worked_mass");
    ok &= add(pt.worked_volume, "worked_volume");
    ok &= add(pt.time_stamp, "time_stamp");
    ok &= add(pt.track_id, "track_id");
    ok &= add(pt.type, UseDefaultTag);
    //@todo add machineRelations
    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const std::vector<RoutePoint> &pts, std::string tag) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    if(pts.empty())
        return true;
    tag = getTag<std::vector<RoutePoint>>(tag);
    if(!tag.empty())
        openTag(tag);
    for(auto &p : pts){
        ok &= add(p, UseDefaultTag);
        if(!ok)
            break;
    }
    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const Track &track, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<Track>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(track.id, "id");
    ok &= add(track.type, UseDefaultTag);
    ok &= add(track.width, "width");
    ok &= add(track.points, UseDefaultTag);
    ok &= add(track.boundary, "boundary");

    if(!tag.empty())
        closeTag();

    return ok;
}

bool AroXMLOutDocument::add(const std::vector<Track> &tracks, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<std::vector<Track>>(tag);
    if(!tag.empty())
        openTag(tag);

    for(auto &track : tracks){
        ok &= add(track, UseDefaultTag);
        if(!ok)
            break;
    }
    if(!tag.empty())
        closeTag();

    return ok;

}

bool AroXMLOutDocument::add(const Headlands &headlands, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<Headlands>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(headlands.complete, UseDefaultTag);
    ok &= add(headlands.partial, UseDefaultTag);

    if(!tag.empty())
        closeTag();

    return ok;

}

bool AroXMLOutDocument::add(const CompleteHeadland &hl, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<CompleteHeadland>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(hl.headlandWidth, "width");
    ok &= add(hl.middle_track, "middle_track");
    ok &= add(hl.tracks, UseDefaultTag);
    ok &= add(hl.boundaries.first, "boundary_out");
    ok &= add(hl.boundaries.second, "boundary_in");

    if(!tag.empty())
        closeTag();

    return ok;
}

bool AroXMLOutDocument::add(const PartialHeadland &hl, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<PartialHeadland>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(hl.id, "id");
    ok &= add(hl.boundary, "boundary");
    ok &= add(hl.connectingHeadlandIds.first, "connectingHeadlandId1");
    ok &= add(hl.connectingHeadlandIds.second, "connectingHeadlandId2");
    ok &= add(hl.tracks, UseDefaultTag);

    if(!tag.empty())
        closeTag();

    return ok;

}

bool AroXMLOutDocument::add(const std::vector<PartialHeadland> &headlands, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<std::vector<PartialHeadland>>(tag);
    if(!tag.empty())
        openTag(tag);

    for(auto &hl : headlands){
        ok &= add(hl, UseDefaultTag);
        if(!ok)
            break;
    }
    if(!tag.empty())
        closeTag();

    return ok;

}

bool AroXMLOutDocument::add(const Obstacle &obs, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<Obstacle>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(obs.type, UseDefaultTag);
    ok &= add(obs.type_description, "type_description");
    ok &= add(obs.boundary, "boundary");

    if(!tag.empty())
        closeTag();

    return ok;
}

bool AroXMLOutDocument::add(const std::vector<Obstacle> &obstacles, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<std::vector<Obstacle>>(tag);
    if(!tag.empty())
        openTag(tag);

    for(auto &obs : obstacles){
        ok &= add(obs, UseDefaultTag);
        if(!ok)
            break;
    }
    if(!tag.empty())
        closeTag();

    return ok;
}


bool AroXMLOutDocument::add(const Subfield &sf, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<Subfield>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(sf.id, "id");

    ok &= add(sf.boundary_outer, "outer_boundary");
    ok &= add(sf.boundary_inner, "inner_boundary");

    ok &= add(sf.access_points, UseDefaultTag);
    ok &= add(sf.resource_points, UseDefaultTag);

    openTag("reference_lines");
    for(auto &rl : sf.reference_lines){
        ok &= add(rl, "reference_line");
        if(!ok)
            break;
    }
    closeTag();

    if(sf.reference_line_A.isValid() && sf.reference_line_B.isValid()){
        ok &= add(sf.reference_line_A, "reference_line_A");
        ok &= add(sf.reference_line_B, "reference_line_B");
    }


    ok &= add(sf.headlands, UseDefaultTag);

    ok &= add(sf.obstacles, UseDefaultTag);

    ok &= add(sf.tracks, UseDefaultTag);

    if(!tag.empty())
        closeTag();

    return ok;
}

bool AroXMLOutDocument::add(const std::vector<Subfield> &subfields, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<std::vector<Subfield>>(tag);
    if(!tag.empty())
        openTag(tag);

    for(auto &sf : subfields){
        ok &= add(sf, UseDefaultTag);
        if(!ok)
            break;
    }
    if(!tag.empty())
        closeTag();

    return ok;
}

bool AroXMLOutDocument::add(const Field &field, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<Field>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(field.id, "id");
    ok &= add(field.name, "name");
    ok &= add(field.filename, "filename");
    ok &= add(field.outer_boundary, "outer_boundary");
    ok &= add(field.subfields, UseDefaultTag);

    if(!field.external_roads.empty()){
        openTag("external_roads");
        ok &= add(field.external_roads, "external_road");
        closeTag();
    }

    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const Machine &machine, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    tag = getTag<Machine>(tag);
    if(!tag.empty())
        openTag(tag);
    ok &= add(machine.id, "id");
    ok &= add(machine.machinetype, UseDefaultTag);
    ok &= add(machine.manufacturer, "manufacturer");
    ok &= add(machine.model, "model");
    ok &= add(machine.width, "width");
    ok &= add(machine.length, "length");
    ok &= add(machine.height, "height");
    ok &= add(machine.weight, "weight");
    ok &= add(machine.bunker_mass, "bunker_mass");
    ok &= add(machine.bunker_volume, "bunker_volume");
    ok &= add(machine.working_width, "working_width");
    ok &= add(machine.max_speed_empty, "max_speed_empty");
    ok &= add(machine.max_speed_full, "max_speed_full");
    ok &= add(machine.def_working_speed, "def_working_speed");
    ok &= add(machine.unloading_speed_mass, "unloading_speed_mass");
    ok &= add(machine.unloading_speed_volume, "unloading_speed_volume");
    ok &= add(machine.turning_radius, "turning_radius");
    ok &= add(machine.num_axis, "num_axis");
    ok &= add(machine.axis_distance, "axis_distance");
    ok &= add(machine.gauge, "gauge");
    ok &= add(machine.engine_power, "engine_power");
    ok &= add(machine.knifes_count, "knifes_count");
    ok &= add((size_t) machine.unload_sides, "unload_sides");
    ok &= add(machine.machineassignment, UseDefaultTag);
    ok &= add(machine.id_intern, "id_intern");
    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const std::vector<Machine> &machines, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    tag = getTag<std::vector<Machine>>(tag);
    if(!tag.empty())
        openTag(tag);

    for(const auto &m : machines){
        if(!ok)
            break;
        ok &= add(m, UseDefaultTag);
    }

    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const Route &route, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    tag = getTag<Route>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(route.machine_id, "machine_id");
    ok &= add(route.route_id, "route_id");
    ok &= add(route.route_points, UseDefaultTag);

    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const std::vector<Route> &routes, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    tag = getTag<std::vector<Route>>(tag);
    if(!tag.empty())
        openTag(tag);

    for(const auto &route : routes){
        if(!ok)
            break;
        ok &= add(route, UseDefaultTag);
    }

    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const std::pair< MachineId_t, MachineDynamicInfo >& dynamicInfo, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    tag = getTag<MachineDynamicInfo>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(dynamicInfo.first, "machine_id");
    ok &= add(dynamicInfo.second.position, UseDefaultTag);
    ok &= add(dynamicInfo.second.bunkerMass, "bunkerMass");
    ok &= add(dynamicInfo.second.bunkerVolume, "bunkerVolume");
    ok &= add(dynamicInfo.second.timestamp, "timestamp");

    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const std::map<MachineId_t, MachineDynamicInfo> &dynamicInfo, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    tag = getTag<std::map<MachineId_t, MachineDynamicInfo>>(tag);
    if(!tag.empty())
        openTag(tag);

    for(const auto &di : dynamicInfo){
        if(!ok)
            break;
        ok &= add(std::make_pair(di.first, di.second), UseDefaultTag);
    }

    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const OutFieldInfo &ofi, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    tag = getTag<OutFieldInfo>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(ofi.mapAccessPoint2ResourcePoint(), "mapAccessPoint2ResourcePoint");
    ok &= add(ofi.mapResourcePoint2AccessPoint(), "mapResourcePoint2AccessPoint");
    ok &= add(ofi.mapUnloadingCosts(), UseDefaultTag);
    ok &= add(ofi.mapArrivalCosts(), UseDefaultTag);

    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const std::map<int, std::map<int, std::map<OutFieldInfo::MachineId_t, std::map<OutFieldInfo::MachineBunkerState, OutFieldInfo::TravelCosts> > > > &map, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    if(!tag.empty())
        openTag(tag);

    for(const auto &v1 : map){//id_from
        openTag("from");
        ok &= add(v1.first, "id");

        for(const auto &v2 : v1.second){//id_to
            openTag("to");
            ok &= add(v2.first, "id");

            for(const auto &m : v2.second){//id_machine
                openTag( "machine" );
                ok &= add(m.first, "id");

                for(const auto &s : m.second){//bunker_state
                    openTag( getTag<OutFieldInfo::MachineBunkerState>() );

                    ok &= add(s.first, "state");
                    ok &= add(s.second, UseDefaultTag);

                    closeTag();

                    if(!ok)
                        break;
                }
                closeTag();
                if(!ok)
                    break;
            }
            closeTag();
            if(!ok)
                break;
        }
        closeTag();
        if(!ok)
            break;
    }

    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const OutFieldInfo::MapUnloadingCosts_t &map, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<OutFieldInfo::MapUnloadingCosts_t>(tag);
    if(!tag.empty())
        openTag(tag);

    for(const auto &rp : map){//id_rp
        openTag( "resource_point" );
        ok &= add(rp.first, "id");

        for(const auto &m : rp.second){//id_machine
            openTag( "machine" );

            ok &= add(m.first, "id");
            ok &= add(m.second, UseDefaultTag);

            closeTag();
            if(!ok)
                break;
        }

        closeTag();
        if(!ok)
            break;
    }

    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const OutFieldInfo::MapArrivalCosts_t &map, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    tag = getTag<OutFieldInfo::MapArrivalCosts_t>(tag);
    if(!tag.empty())
        openTag(tag);
    for(const auto &fap : map){//field_access_point
        openTag( "field_access_point" );
        ok &= add(fap.first, "id");

        for(const auto &m : fap.second){//id_machine
            openTag( "machine" );
            ok &= add(m.first, "id");

            for(const auto &s : m.second){//bunker_state
                openTag( getTag<OutFieldInfo::MachineBunkerState>() );
                ok &= add(s.first, "state");
                ok &= add(s.second, UseDefaultTag);
                closeTag();
                if(!ok)
                    break;
            }

            closeTag();
            if(!ok)
                break;
        }

        closeTag();
        if(!ok)
            break;
    }
    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const OutFieldInfo::UnloadingCosts &c, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    tag = getTag<OutFieldInfo::UnloadingCosts>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(c.time, "time");
    ok &= add(c.time_per_kg, "time_per_kg");

    if(!tag.empty())
        closeTag();
    return ok;

}

bool AroXMLOutDocument::add(const OutFieldInfo::TravelCosts &c, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    tag = getTag<OutFieldInfo::TravelCosts>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(c.time, "time");
    ok &= add(c.time_per_kg, "time_per_kg");
    ok &= add(c.distance, "distance");

    if(!tag.empty())
        closeTag();
    return ok;

}

bool AroXMLOutDocument::add(const ArolibGrid_t &grid, std::string tag)
{
    if(!isReadyToWrite())
        return false;
    if(!grid.isAllocated()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The grid is not allocated");
        return false;
    }
    tag = getTag<ArolibGrid_t>(tag);
    std::string sTiff;
    if(!grid.saveGridAsGeoTiffString(sTiff))
        return false;

    return add( base64_encode(sTiff), tag );
}

bool AroXMLOutDocument::add(const std::map<std::string, const ArolibGrid_t*>& gridmaps, std::string tag)
{
    if(gridmaps.empty())
        return true;

    bool ok = true;
    if(!isReadyToWrite())
        return false;
    tag = getTag<std::map<std::string, const ArolibGrid_t*>>(tag);
    if(!tag.empty())
        openTag(tag);

    for(auto& it : gridmaps){
        if(!it.second)
            continue;
        ok &= add( *it.second, it.first );
    }

    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const DirectedGraph::Graph &graph, std::string tag, bool with_meta)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<DirectedGraph::Graph>(tag);
    if(!tag.empty())
        openTag(tag);

    openTag("vertices");
    ok &= add(boost::num_vertices(graph), "count");
    for(DirectedGraph::vertex_iter it = vertices(graph); it.first != it.second; it.first++){
        ok &= add( std::make_pair( *it.first, graph[*it.first] ) , UseDefaultTag);
    }
    closeTag();

    openTag("edges");
    ok &= add(boost::num_edges(graph), "count");
    for(DirectedGraph::edge_iter it = edges(graph); it.first != it.second; it.first++){
        ok &= add( std::make_pair( *it.first, graph[*it.first] ), graph, UseDefaultTag);
    }
    closeTag();

    if(with_meta)
        ok &= add( graph.get_meta(), UseDefaultTag);

    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const DirectedGraph::vertex_pair &vt, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<DirectedGraph::vertex_pair>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(vt.first, "id");
    ok &= add(vt.second, UseDefaultTag);

    if(!tag.empty())
        closeTag();
    return ok;

}

bool AroXMLOutDocument::add(const DirectedGraph::edge_pair &edge, const DirectedGraph::Graph &graph, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<DirectedGraph::edge_pair>(tag);
    if(!tag.empty())
        openTag(tag);

    DirectedGraph::vertex_t vt_from = source(edge.first, graph);
    DirectedGraph::vertex_t vt_to = target(edge.first, graph);

    ok &= add(edge.first, UseDefaultTag);
    ok &= add(vt_from, "vt_from");
    ok &= add(vt_to, "vt_to");
    ok &= add(edge.second, UseDefaultTag);

    if(!tag.empty())
        closeTag();

    return ok;

}

bool AroXMLOutDocument::add(const DirectedGraph::edge_t &edge, std::string tag)
{
    std::stringstream ss;
    ss << edge;
    tag = getTag<DirectedGraph::edge_t>(tag);
    return add(ss.str(), tag);
}

bool AroXMLOutDocument::add(const DirectedGraph::vertex_property &v_prop, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<DirectedGraph::vertex_property>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(v_prop.route_point, UseDefaultTag);
    ok &= add(v_prop.harvester_id, "workingMachineId");
    if(!v_prop.unloadingCosts.empty()){
        openTag("unloadingCosts");
        for(auto& it : v_prop.unloadingCosts){
            openTag("machine");
            ok &= add(it.first, "machine_id");
            ok &= add(it.second, UseDefaultTag);
            closeTag();
        }
        closeTag();
    }
    if(!v_prop.visitPeriods.empty()){
        openTag("visitPeriods");
        for(auto& it : v_prop.visitPeriods){
            openTag(getTag<DirectedGraph::VisitPeriod>());
            ok &= add(it.first, "key");
            ok &= add(it.second, UseDefaultTag);
            closeTag();
        }
        closeTag();
    }

    if(!tag.empty())
        closeTag();
    return ok;

}

bool AroXMLOutDocument::add(const DirectedGraph::edge_property &e_prop, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<DirectedGraph::edge_property>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(e_prop.edge_type, UseDefaultTag);
    ok &= add(e_prop.p0, "p_from");
    ok &= add(e_prop.p1, "p_to");
    ok &= add(e_prop.bidirectional, "bidir");
    if(e_prop.bidirectional)
        ok &= add(e_prop.revEdge, "rev_edge");
    ok &= add(e_prop.defWidth, "def_width");
    ok &= add(e_prop.distance, "dist");
    ok &= add(e_prop.arrivalCosts, "arrival_costs");
    if(!e_prop.overruns.empty()){
        openTag( getTag( e_prop.overruns) );
        ok &= add(e_prop.overruns, UseDefaultTag);
        closeTag();
    }

    if(!tag.empty())
        closeTag();
    return ok;

}

bool AroXMLOutDocument::add(const DirectedGraph::VisitPeriod &vp, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<DirectedGraph::VisitPeriod>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(vp.machineId, "machine_id");
    ok &= add(vp.timestamp, "timestamp");
    ok &= add(vp.time_in, "time_in");
    ok &= add(vp.time_out, "time_out");

    if(!vp.next_vt.empty()){
        openTag("next_vts");
        for(auto& vt_p : vp.next_vt){
            openTag("vertex");
            ok &= add(vt_p.first, "vertex_id");
            ok &= add(vt_p.second, UseDefaultTag);
            closeTag();
        }
        closeTag();
    }

    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const DirectedGraph::overroll_property &o, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<DirectedGraph::overroll_property>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(o.machine_id, "machine_id");
    ok &= add(o.weight, "weight");
    ok &= add(o.duration, "duration");

    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroXMLOutDocument::add(const DirectedGraph::Graph::GraphMetaData &meta, std::string tag)
{

    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<DirectedGraph::Graph::GraphMetaData>(tag);
    if(!tag.empty())
        openTag(tag);

    ok &= add(meta.hasPartialHeadlands, "hasPartialHeadlands");
    ok &= add(meta.workingWidth_IF, "workingWidth_IF");
    ok &= add(meta.workingWidth_HL, "workingWidth_HL");

    openTag("tracks_vertices_map");
    for(auto& it : meta.tracks_vertices_map){
        openTag("item");
        ok &= add(it.first, "id");
        ok &= add( to_single_string(it.second) , "vts");
        closeTag();
    }
    closeTag();

    if(!meta.extremaTrackIds_IF.empty())
        ok &= add( to_single_string(meta.extremaTrackIds_IF) , "extremaTrackIds_IF");

    openTag("adjacentTrackIds_IF");
    for(auto& it : meta.adjacentTrackIds_IF){
        openTag("item");
        ok &= add(it.first, "id");
        ok &= add( to_single_string(it.second) , "adj");
        closeTag();
    }
    closeTag();

    openTag("adjacentTrackIds_HL");
    for(auto& it : meta.adjacentTrackIds_HL){
        openTag("item");
        ok &= add(it.first, "id");
        ok &= add( to_single_string(it.second) , "adj");
        closeTag();
    }
    closeTag();

    if(!meta.outermostTrackIds_HL.empty())
        ok &= add( to_single_string(meta.outermostTrackIds_HL) , "outermostTrackIds_HL");


    ok &= add(meta.outFieldInfo, UseDefaultTag);


    openTag("routepoint_vertex_map");
    for(auto& it : meta.routepoint_vertex_map){
        openTag("item");
        ok &= add(it.first, UseDefaultTag);
        ok &= add( std::to_string(it.second) , "vt");
        closeTag();
    }
    closeTag();


    openTag("accesspoint_vertex_map");
    for(auto& it : meta.accesspoint_vertex_map){
        openTag("item");
        ok &= add(it.first, UseDefaultTag);
        ok &= add( std::to_string(it.second) , "vt");
        closeTag();
    }
    closeTag();


    openTag("resourcepoint_vertex_map");
    for(auto& it : meta.resourcepoint_vertex_map){
        openTag("item");
        ok &= add(it.first, UseDefaultTag);
        ok &= add( std::to_string(it.second) , "vt");
        closeTag();
    }
    closeTag();


    openTag("initialpoint_vertex_map");
    for(auto& it : meta.initialpoint_vertex_map){
        openTag("item");
        ok &= add(it.first, "machine_id");
        ok &= add( std::to_string(it.second) , "vt");
        closeTag();
    }
    closeTag();

    if(!meta.boundary_vts.empty())
        ok &= add( to_single_string(meta.boundary_vts) , "boundary_vts");


    //@note the verticesLocationMap will not be saved. when reading, the graph.regeneratVerticesLocationMap must be called after reading the vertices

    if(!tag.empty())
        closeTag();
    return ok;
}


bool AroXMLOutDocument::saveField(const std::string &filename, const Field &field, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out, const OutFieldInfo &outFieldInfo)
{
    AroXMLOutDocument doc;
    doc.setCoordinatesTypes(coordinatesType_in, coordinatesType_out);
    return doc.openFile(filename) &&
           doc.openDocument() &&
           doc.add(field, UseDefaultTag) &&
           doc.add(outFieldInfo, UseDefaultTag) &&
           doc.closeDocument() &&
           doc.closeFile();
}

bool AroXMLOutDocument::saveFields(const std::string &filename, const std::vector<Field> &fields, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out)
{
    AroXMLOutDocument doc;
    doc.setCoordinatesTypes(coordinatesType_in, coordinatesType_out);
    return doc.openFile(filename) &&
           doc.openDocument() &&
           doc.add(fields, UseDefaultTag) &&
           doc.closeDocument() &&
           doc.closeFile();

}

bool AroXMLOutDocument::saveMachines(const std::string &filename, const std::vector<Machine> &machines)
{
    AroXMLOutDocument doc;
    return doc.openFile(filename) &&
           doc.openDocument() &&
           doc.add(machines, UseDefaultTag) &&
           doc.closeDocument() &&
           doc.closeFile();
}

bool AroXMLOutDocument::saveConfigParameters(const std::string &filename, const std::map<std::string, std::map<std::string, std::string> > &configParameters)
{
    AroXMLOutDocument doc;
    return doc.openFile(filename) &&
           doc.openDocument() &&
           doc.add(configParameters, "configParameters") &&
           doc.closeDocument() &&
           doc.closeFile();
}

bool AroXMLOutDocument::saveOutFieldInfo(const std::string &filename, const OutFieldInfo &outFieldInfo)
{
    AroXMLOutDocument doc;
    return doc.openFile(filename) &&
           doc.openDocument() &&
           doc.add(outFieldInfo, UseDefaultTag) &&
           doc.closeDocument() &&
           doc.closeFile();
}

bool AroXMLOutDocument::savePlanParameters(const std::string &filename,
                                           const Field &field,
                                           const std::vector<Machine> &workingGroup,
                                           const std::map<std::string, std::map<std::string, std::string> > &configParameters,
                                           const OutFieldInfo &outFieldInfo,
                                           const std::map<MachineId_t, MachineDynamicInfo> &machinesDynamicInfo,
                                           const std::map<std::string, const ArolibGrid_t *> gridmaps,
                                           Point::ProjectionType coordinatesType_in,
                                           Point::ProjectionType coordinatesType_out)
{
    AroXMLOutDocument doc;    
    doc.setCoordinatesTypes(coordinatesType_in, coordinatesType_out);
    return doc.openFile(filename) &&
           doc.openDocument() &&
           doc.add(field, UseDefaultTag) &&
           doc.add(workingGroup, UseDefaultTag) &&
           doc.add(configParameters, "configParameters") &&
           doc.add(outFieldInfo, UseDefaultTag) &&
           doc.add(machinesDynamicInfo, UseDefaultTag) &&
           doc.add(gridmaps, UseDefaultTag) &&
           doc.closeDocument() &&
           doc.closeFile();
}

bool AroXMLOutDocument::savePlanParameters(const std::string &filename,
                                           const std::vector<Machine> &workingGroup,
                                           const std::map<std::string, std::map<std::string, std::string> > &configParameters,
                                           const OutFieldInfo &outFieldInfo,
                                           const std::map<MachineId_t, MachineDynamicInfo> &machinesDynamicInfo,
                                           const std::map<std::string, const ArolibGrid_t *> gridmaps,
                                           Point::ProjectionType coordinatesType_in,
                                           Point::ProjectionType coordinatesType_out)
{
    AroXMLOutDocument doc;
    doc.setCoordinatesTypes(coordinatesType_in, coordinatesType_out);
    return doc.openFile(filename) &&
           doc.openDocument() &&
           doc.add(workingGroup, UseDefaultTag) &&
           doc.add(configParameters, "configParameters") &&
           doc.add(outFieldInfo, UseDefaultTag) &&
           doc.add(machinesDynamicInfo, UseDefaultTag) &&
           doc.add(gridmaps, UseDefaultTag) &&
           doc.closeDocument() &&
           doc.closeFile();
}

bool AroXMLOutDocument::savePlanParameters(const std::string &filename,
                                        const std::vector<Machine> &workingGroup,
                                        const std::map<std::string, std::map<std::string, std::string> > &configParameters,
                                        const OutFieldInfo &outFieldInfo,
                                        const std::map<MachineId_t, MachineDynamicInfo> &machinesDynamicInfo,
                                        const std::string &yieldmap_tifBase64,
                                        const std::string &drynessmap_tifBase64,
                                        const std::string &soilmap_tifBase64,
                                        const std::string &remainingAreaMap_tifBase64, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out)
{
    AroXMLOutDocument doc;
    doc.setCoordinatesTypes(coordinatesType_in, coordinatesType_out);
    return doc.openFile(filename) &&
           doc.openDocument() &&
           doc.add(workingGroup, UseDefaultTag) &&
           doc.add(configParameters, "configParameters") &&
           doc.add(outFieldInfo, UseDefaultTag) &&
           doc.add(machinesDynamicInfo, UseDefaultTag) &&
           ( yieldmap_tifBase64.empty() ? true : doc.add(yieldmap_tifBase64, "yieldmap_tifBase64") ) &&
           ( drynessmap_tifBase64.empty() ? true : doc.add(drynessmap_tifBase64, "drynessmap_tifBase64") ) &&
           ( soilmap_tifBase64.empty() ? true : doc.add(soilmap_tifBase64, "soilmap_tifBase64") ) &&
           ( remainingAreaMap_tifBase64.empty() ? true : doc.add(remainingAreaMap_tifBase64, "remainingAreaMap_tifBase64") ) &&
           doc.closeDocument() &&
           doc.closeFile();
}

bool AroXMLOutDocument::savePlan(const std::string &filename, const std::map<int, std::vector<Route> > &routes, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out)
{
    AroXMLOutDocument doc;
    bool ok = true;

    doc.setCoordinatesTypes(coordinatesType_in, coordinatesType_out);

    if( !doc.openFile(filename) ||
        !doc.openDocument() )
        return false;

    doc.openTag("plan");

    for(auto &sf_it : routes){
        doc.openTag(getTag<Subfield>());
        ok &= doc.add(sf_it.first, "id");
        ok &= doc.add(sf_it.second, UseDefaultTag);
        doc.closeTag();
        if(!ok)
            break;
    }

    return doc.closeDocument() &&
           doc.closeFile() &&
           ok;
}

bool AroXMLOutDocument::savePlan(const std::string &filename, const std::vector<std::vector<Route> > &routes, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out)
{
    AroXMLOutDocument doc;
    bool ok = true;

    doc.setCoordinatesTypes(coordinatesType_in, coordinatesType_out);

    if( !doc.openFile(filename) ||
        !doc.openDocument() )
        return false;

    doc.openTag("plan");

    for(size_t i = 0 ; i < routes.size() ; ++i){
        doc.openTag(getTag<Subfield>());
        ok &= doc.add(i, "id");
        ok &= doc.add(routes.at(i), UseDefaultTag);
        doc.closeTag();
        if(!ok)
            break;
    }

    return doc.closeDocument() &&
           doc.closeFile() &&
           ok;
}

bool AroXMLOutDocument::savePlan(const std::string &filename,
                              const Field &field,
                              const std::vector<Machine> &workingGroup,
                              const std::map<int, std::vector<Route> > &routes,
                              const std::string &yieldmap_tifBase64,
                              const std::string &drynessmap_tifBase64,
                              const std::string &soilmap_tifBase64,
                              const std::string &remainingAreaMap_tifBase64, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out)
{
    AroXMLOutDocument doc;
    bool ok = true;

    doc.setCoordinatesTypes(coordinatesType_in, coordinatesType_out);

    if( !doc.openFile(filename) ||
        !doc.openDocument() )
        return false;

    doc.openTag("plan");

    ok &= doc.add(field, UseDefaultTag);
    ok &= doc.add(workingGroup, UseDefaultTag);

    for(auto &sf_it : routes){
        doc.openTag(getTag<Subfield>());
        ok &= doc.add(sf_it.first, "id");
        ok &= doc.add(sf_it.second, UseDefaultTag);
        doc.closeTag();
        if(!ok)
            break;
    }

    return ( yieldmap_tifBase64.empty() ? true : doc.add(yieldmap_tifBase64, "yieldmap_tifBase64") ) &&
           ( drynessmap_tifBase64.empty() ? true : doc.add(drynessmap_tifBase64, "drynessmap_tifBase64") ) &&
           ( soilmap_tifBase64.empty() ? true : doc.add(soilmap_tifBase64, "soilmap_tifBase64") ) &&
           ( remainingAreaMap_tifBase64.empty() ? true : doc.add(remainingAreaMap_tifBase64, "remainingAreaMap_tifBase64") ) &&
           doc.closeDocument() &&
           doc.closeFile() &&
           ok;

}

bool AroXMLOutDocument::savePlan(const std::string &filename,
                              const Field &field,
                              const std::vector<Machine> &workingGroup,
                              const std::vector<std::vector<Route> > &routes,
                              const std::string &yieldmap_tifBase64,
                              const std::string &drynessmap_tifBase64,
                              const std::string &soilmap_tifBase64,
                              const std::string &remainingAreaMap_tifBase64, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out)
{
    AroXMLOutDocument doc;
    bool ok = true;

    doc.setCoordinatesTypes(coordinatesType_in, coordinatesType_out);

    if( !doc.openFile(filename) ||
        !doc.openDocument() )
        return false;

    doc.openTag("plan");

    ok &= doc.add(field, UseDefaultTag);
    ok &= doc.add(workingGroup, UseDefaultTag);

    for(size_t i = 0 ; i < routes.size() ; ++i){
        if(!ok)
            break;
        doc.openTag(getTag<Subfield>());
        ok &= doc.add(i, "id");
        ok &= doc.add(routes.at(i), UseDefaultTag);
        doc.closeTag();
    }

    return ( yieldmap_tifBase64.empty() ? true : doc.add(yieldmap_tifBase64, "yieldmap_tifBase64") ) &&
           ( drynessmap_tifBase64.empty() ? true : doc.add(drynessmap_tifBase64, "drynessmap_tifBase64") ) &&
           ( soilmap_tifBase64.empty() ? true : doc.add(soilmap_tifBase64, "soilmap_tifBase64") ) &&
           ( remainingAreaMap_tifBase64.empty() ? true : doc.add(remainingAreaMap_tifBase64, "remainingAreaMap_tifBase64") ) &&
           doc.closeDocument() &&
           doc.closeFile() &&
           ok;

}

bool AroXMLOutDocument::savePlan(const std::string &filename,
                                 const Field &field,
                                 const std::vector<Machine> &workingGroup,
                                 const std::map<int, std::vector<Route> > &routes,
                                 const std::map<std::string, ArolibGrid_t*> &gridmaps,
                                 Point::ProjectionType coordinatesType_in,
                                 Point::ProjectionType coordinatesType_out)
{
    AroXMLOutDocument doc;
    bool ok = true;

    doc.setCoordinatesTypes(coordinatesType_in, coordinatesType_out);

    if( !doc.openFile(filename) ||
        !doc.openDocument() )
        return false;

    doc.openTag("plan");

    ok &= doc.add(field, UseDefaultTag);
    ok &= doc.add(workingGroup, UseDefaultTag);

    for(auto &sf_it : routes){
        if(!ok)
            break;
        doc.openTag(getTag<Subfield>());
        ok &= doc.add(sf_it.first, "id");
        ok &= doc.add(sf_it.second, UseDefaultTag);
        doc.closeTag();
    }

    if(!gridmaps.empty() && ok){
        doc.openTag("gridmaps");
        for(const auto& it_g : gridmaps){
            if(!ok)
                break;
            std::string mapStr;
            ArolibGrid_t* gridmap = it_g.second;
            if( !gridmap || !gridmap->isAllocated() )
                continue;
            if(!gridmap->saveGridAsGeoTiffString(mapStr) || mapStr.empty())
                continue;
            ok &= doc.add( base64_encode(mapStr) , it_g.first);
        }
        doc.closeTag();
    }

    return doc.closeDocument() &&
           doc.closeFile() &&
           ok;
}

bool AroXMLOutDocument::savePlan(const std::string &filename,
                                 const Field &field,
                                 const std::vector<Machine> &workingGroup,
                                 const std::vector<std::vector<Route> > &routes,
                                 const std::map<std::string, ArolibGrid_t*> &gridmaps,
                                 Point::ProjectionType coordinatesType_in,
                                 Point::ProjectionType coordinatesType_out)
{
    AroXMLOutDocument doc;
    bool ok = true;

    doc.setCoordinatesTypes(coordinatesType_in, coordinatesType_out);

    if( !doc.openFile(filename) ||
        !doc.openDocument() )
        return false;

    doc.openTag("plan");

    ok &= doc.add(field, UseDefaultTag);
    ok &= doc.add(workingGroup, UseDefaultTag);

    for(size_t i = 0 ; i < routes.size() ; ++i){
        if(!ok)
            break;
        doc.openTag(getTag<Subfield>());
        ok &= doc.add(i, "id");
        ok &= doc.add(routes.at(i), UseDefaultTag);
        doc.closeTag();
    }

    if(!gridmaps.empty() && ok){
        doc.openTag("gridmaps");
        for(const auto& it_g : gridmaps){
            if(!ok)
                break;
            std::string mapStr;
            ArolibGrid_t* gridmap = it_g.second;
            if( !gridmap || !gridmap->isAllocated() )
                continue;
            if(!gridmap->saveGridAsGeoTiffString(mapStr) || mapStr.empty())
                continue;
            ok &= doc.add( base64_encode(mapStr) , it_g.first);
        }
        doc.closeTag();
    }

    return doc.closeDocument() &&
           doc.closeFile() &&
           ok;

}

bool AroXMLOutDocument::saveGraph(const std::string &filename, const DirectedGraph::Graph &graph, bool with_meta, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out)
{
    AroXMLOutDocument doc;
    bool ok = true;

    doc.setCoordinatesTypes(coordinatesType_in, coordinatesType_out);

    return doc.openFile(filename) &&
           doc.openDocument() &&
           doc.add(graph, UseDefaultTag, with_meta) &&
           doc.closeDocument() &&
           doc.closeFile() &&
           ok;
}

bool AroXMLOutDocument::openDoc()
{
    if(!XMLOutDocument::openDoc())
        return false;

    std::string coordType = ( m_coordinatesType_out == Point::WGS ? "WGS" : "UTM");
    return add(coordType, m_coordTypeTag);
}

}
}//end namespace arolib

