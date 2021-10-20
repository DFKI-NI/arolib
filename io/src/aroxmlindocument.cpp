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
 
#include "arolib/io/aroxmlindocument.hpp"

namespace arolib {
namespace io {

AroXMLInDocument::AroXMLInDocument(LogLevel logLevel):
    XMLInDocument(logLevel)
{
}

AroXMLInDocument::~AroXMLInDocument()
{
}

bool AroXMLInDocument::readCoodinatesInputType(const XMLInDocument::ReadHandler &base)
{
    m_coordinatesType_in = Point::WGS;
    std::string sType, sType0;
    if( !getValue(RHTree(base), m_coordTypeTag, sType0) )
        return false;

    sType = boost::to_upper_copy(sType0);
    if(sType != "UTM" && sType != "WGS"){
        m_logger.printWarning(__FUNCTION__, "Invalid coordinates type " + sType0 + ". Using default." );
        return false;
    }
    if(sType == "UTM")
        m_coordinatesType_in = Point::UTM;
    return true;
}


bool AroXMLInDocument::read( const ReadHandler & base, Point &pt)
{
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    return stringToPoint(RHTree(base).get_value<std::string>() , pt );
}

bool AroXMLInDocument::read( const ReadHandler & base, std::vector<Point> &pts)
{
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    return stringToPointList(RHTree(base).get_value<std::string>() , pts );
}

bool AroXMLInDocument::read( const ReadHandler & base, Linestring &ls){
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ls = Linestring();
        bool ok_id = false;
        ReadHandler branch;

        ok_id = getValue(RHTree(base), "id", ls.id);

        if(getBranch(base, getTag(ls.points), branch)){
            if(!read(branch, ls.points))
                return false;
        }
        return ( !m_strict || (ok_id) );
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Linestring: ") + e.what());
        return false;
    }
}

bool AroXMLInDocument::read( const ReadHandler & base, Polygon &poly){
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ReadHandler branch;
        if(getBranch(base, getTag(poly.points), branch)){
            if( !read(branch, poly.points) )
                return false;            
            geometry::correct_polygon(poly);
        }

        return true;
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Polygon: ") + e.what());
        return false;
    }
}

bool AroXMLInDocument::read( const ReadHandler & base, FieldAccessPoint &pt){

    pt = FieldAccessPoint();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        bool ok_id = false;
        bool ok_at = false;
        ReadHandler branch;
        int tmp;

        ok_id = getValue(RHTree(base), "id", pt.id);

        if(!getBranch(base, getTag(pt.point()), branch))
            return false;
        if(!read(branch, pt.point()))
            return false;

        if( getValue(RHTree(base), getTag(pt.accessType), tmp) ){
            pt.accessType = FieldAccessPoint::intToAccessPointType(tmp);
            ok_at = true;
        }

        return ( !m_strict || (ok_id && ok_at) );
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("FieldAccessPoint: ") + e.what());
        return false;
    }
}

bool AroXMLInDocument::read( const ReadHandler & base, std::vector<FieldAccessPoint> &pts){
    return readMultiple(base, pts, UseDefaultTag, true);
}

bool AroXMLInDocument::read( const ReadHandler & base, ResourcePoint &pt){

    pt = ResourcePoint();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        bool ok_id = false;
        bool ok_ut = false;
        ReadHandler branch;

        ok_id = getValue(RHTree(base), "id", pt.id);

        if(!getBranch(base, getTag(pt.point()), branch))
            return false;
        if(!read(branch, pt.point()))
            return false;

        if(getBranch(base, getTag(pt.resourceTypes), branch)){
            if(!read(branch, pt.resourceTypes))
                return false;
        }

        ok_ut = getValue(RHTree(base), "defaultUnloadingTime", pt.defaultUnloadingTime);
        getValue(RHTree(base), "defaultUnloadingTimePerKg", pt.defaultUnloadingTimePerKg);

        if(getBranch(base, "geometry", branch)){
            if(!read(branch, pt.geometry))
                return false;
        }

        return ( !m_strict || (ok_id && ok_ut) );
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("ResourcePoint: ") + e.what());
        return false;
    }
}

bool AroXMLInDocument::read( const ReadHandler & base, std::vector<ResourcePoint> &pts){
    return readMultiple(base, pts, UseDefaultTag, true);
}


bool AroXMLInDocument::read(const AroXMLInDocument::ReadHandler &base, std::set<ResourcePoint::ResourceType> &resourceTypes)
{
    resourceTypes.clear();
    std::string sTypes;
    if(!read(base, sTypes))
        return false;

    try{
        std::vector<std::string> vTypes;
        boost::split(vTypes, sTypes, boost::is_any_of(","));

        for(auto &sType : vTypes){
            if(sType.empty())
                continue;
            try{
                ResourcePoint::ResourceType type = ResourcePoint::intToResourceType( std::stoi(sType) );
                resourceTypes.insert( type );
            }
            catch(std::exception &e){
                m_logger.printWarning(__FUNCTION__, std::string("Invalid resource types: ") + e.what() );
                continue;
            }
        }
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Cannot parse/split the resource types '" + sTypes + "': " + e.what());
        return false;
    }

    return true;
}

bool AroXMLInDocument::read( const ReadHandler & base, RoutePoint &pt){
    pt = RoutePoint();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        bool ok_bm = false;
        bool ok_bv = false;
        bool ok_hm = false;
        bool ok_hv = false;
        bool ok_ts = false;
        bool ok_ti = false;
        bool ok_t = false;
        ReadHandler branch;
        int tmp;

        if(!getBranch(base, getTag(pt.point()), branch))
            return false;
        if(!read(branch, pt.point()))
            return false;

        ok_bm = getValue(RHTree(base), "bunker_mass", pt.bunker_mass);
        ok_bv = getValue(RHTree(base), "bunker_volume", pt.bunker_volume);
        ok_hm = getValue(RHTree(base), "harvested_mass", pt.harvested_mass);
        ok_hv = getValue(RHTree(base), "harvested_volume", pt.harvested_volume);
        ok_ts = getValue(RHTree(base), "time_stamp", pt.time_stamp);
        ok_ti = getValue(RHTree(base), "track_id", pt.track_id);
        getValue(RHTree(base), "track_idx", pt.track_idx);

        if( getValue(RHTree(base), getTag(pt.type), tmp) ){
            pt.type = RoutePoint::intToRoutePointType(tmp);
            ok_t = true;
        }

        return ( !m_strict || (ok_bm && ok_bv && ok_hm && ok_hv && ok_ts && ok_ti && ok_t) );
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("RoutePoint: ") + e.what());
        return false;
    }
}

bool AroXMLInDocument::read( const ReadHandler & base, std::vector<RoutePoint> &pts){
    return readMultiple(base, pts, UseDefaultTag, true);
}

bool AroXMLInDocument::read( const ReadHandler & base, HeadlandPoint &pt){
    pt = HeadlandPoint();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        bool ok_ti = false;
        ReadHandler branch;

        if(!getBranch(base, getTag(pt.point()), branch))
            return false;
        if(!read(branch, pt.point()))
            return false;

        ok_ti = getValue(RHTree(base), "track_id", pt.track_id);

        return ( !m_strict || (ok_ti) );
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("RoutePoint: ") + e.what());
        return false;
    }
}

bool AroXMLInDocument::read( const ReadHandler & base, std::vector<HeadlandPoint> &pts){
    return readMultiple(base, pts, UseDefaultTag, true);
}
bool AroXMLInDocument::read( const ReadHandler & base, Track &track){
    track = Track();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ReadHandler branch;

        getValue(RHTree(base), "id", track.id);

        int tmp;
        if( getValue(RHTree(base), getTag(track.type), tmp) ){
            track.type = Track::intToTrackType(tmp);
        }

        if(getBranch(base, getTag(track.points), branch)){
            if(!read(branch, track.points))
                return false;
        }

        if(getBranch(base, "boundary", branch)){
            if(!read(branch, track.boundary))
                return false;
        }

        getValue(RHTree(base), "width", track.width);

        return (true);
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Track: ") + e.what());
        return false;
    }
}

bool AroXMLInDocument::read( const ReadHandler & base, std::vector<Track> &tracks){
    return readMultiple(base, tracks, UseDefaultTag, true);
}

bool AroXMLInDocument::read(const AroXMLInDocument::ReadHandler &base, Headlands &headlands)
{
    headlands = Headlands();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ReadHandler branch;

        if(getBranch(base, getTag(headlands.complete), branch)){
            if(!read(branch, headlands.complete))
                return false;
        }

        return (true);
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Headlands: ") + e.what());
        return false;
    }

}

bool AroXMLInDocument::read(const AroXMLInDocument::ReadHandler &base, CompleteHeadland &hl)
{
    hl = CompleteHeadland();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ReadHandler branch;

        getValue( RHTree(base), "width", hl.headlandWidth);

        if(getBranch(base, "middle_track", branch)){
            if(!read(branch, hl.middle_track))
                return false;
        }

        if(getBranch(base, getTag(hl.tracks), branch)){
            if(!read(branch, hl.tracks))
                return false;
        }

        if(getBranch(base, "boundary_out", branch)){
            if(!read(branch, hl.boundaries.first))
                return false;
        }
        if(getBranch(base, "boundary_in", branch)){
            if(!read(branch, hl.boundaries.second))
                return false;
        }

        return (true);
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("CompleteHeadland: ") + e.what());
        return false;
    }

}

bool AroXMLInDocument::read(const AroXMLInDocument::ReadHandler &base, Obstacle &obs)
{
    obs = Obstacle();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ReadHandler branch;
        int tmp;
        if( getValue(RHTree(base), getTag(obs.type), tmp) ){
            obs.type = Obstacle::intToObstacleType(tmp);
        }

        getValue(RHTree(base), "type_description", obs.type_description);

        if(getBranch(base, "boundary", branch)){
            if(!read(branch, obs.boundary))
                return false;
        }

        return true;
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Obstacle: ") + e.what());
        return false;
    }

}

bool AroXMLInDocument::read(const AroXMLInDocument::ReadHandler &base, std::vector<Obstacle> &obstacles)
{
    return readMultiple(base, obstacles, UseDefaultTag, true);
}

bool AroXMLInDocument::read( const ReadHandler & base, Subfield &sf){

    sf = Subfield();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        bool ok_ob = false;
        ReadHandler branch;

        getValue(RHTree(base), "id", sf.id);

        if(getBranch(base, "outer_boundary", branch)){
            if(!read(branch, sf.boundary_outer))
                return false;
            ok_ob = true;
        }
        if(getBranch(base, "inner_boundary", branch)){
            if(!read(branch, sf.boundary_inner))
                return false;
        }

        if(getBranch(base, getTag(sf.access_points), branch)){
            if(!read(branch, sf.access_points))
                return false;
        }
        if(getBranch(base, getTag(sf.resource_points), branch)){
            if(!read(branch, sf.resource_points))
                return false;
        }

        if(getBranch(base, "reference_lines", branch)){
            BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, RHTree(branch) ){
                if(v.first == "reference_line"){
                    sf.reference_lines.push_back( Linestring() );
                    if(!read( createRH(v.second), sf.reference_lines.back())){
                        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error reading reference line " + std::to_string(sf.reference_lines.size()));
                        sf.reference_lines.pop_back();
                        return false;
                    }
                }
            }
        }

        if(getBranch(base, "reference_line_A", branch)){
            if(!read(branch, sf.reference_line_A)){
                sf.reference_line_A.setInvalid();
                //return false;
            }
        }
        if(getBranch(base, "reference_line_B", branch)){
            if(!read(branch, sf.reference_line_B)){
                sf.reference_line_B.setInvalid();
                //return false;
            }
        }

        if(getBranch(base, getTag(sf.obstacles), branch)){
            if(!read(branch, sf.obstacles)){
                //return false;

                //[legacy] try previous version where 'obstacles' was a vector of boundaries
                sf.obstacles.clear();
                if(getBranch(base, "obstacles", branch)){
                    BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, RHTree(branch) ){
                        if(v.first == "obstacle"){
                            sf.obstacles.push_back( Obstacle() );
                            sf.obstacles.back().type = Obstacle::OBS_OTHER;
                            if(!read( createRH(v.second), sf.obstacles.back().boundary)){
                                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error reading obstacle " + std::to_string(sf.obstacles.size()));
                                sf.obstacles.pop_back();
                                return false;
                            }
                        }
                    }
                }
            }
        }

        if(getBranch(base, getTag(sf.headlands), branch)){
            if(!read(branch, sf.headlands))
                return false;
        }
        else{//[legacy] try previous version
            if(getBranch(base, "planned_headland", branch)){
                if(!read(branch, sf.headlands.complete.middle_track))
                    return false;
            }

            if(getBranch(base, "headland_tracks", branch)){
                if(!read(branch, sf.headlands.complete.tracks))
                    return false;
            }
        }


        if(getBranch(base, getTag(sf.tracks), branch)){
            if(!read(branch, sf.tracks))
                return false;
        }

        return ( !m_strict || (ok_ob) );
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Subfield: ") + e.what());
        return false;
    }

}

bool AroXMLInDocument::read( const ReadHandler & base, std::vector<Subfield> &sfs){
    return readMultiple(base, sfs, UseDefaultTag, true);
}

bool AroXMLInDocument::read( const ReadHandler & base, Field &field){
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        bool ok_id = false;
        bool ok_n = false;
        ReadHandler branch;
        field.clear();

        ok_id = getValue(RHTree(base), "id", field.id);
        ok_n = getValue(RHTree(base), "name", field.name);
        getValue(RHTree(base), "filename", field.filename);

        if(!getBranch(base, "outer_boundary", branch))
            return false;
        if(!read(branch, field.outer_boundary))
            return false;

        if(getBranch(base, getTag(field.subfields), branch)){
            if( !read(branch,field.subfields) )
                return false;
        }

        if(getBranch(base, "external_roads", branch)){
            if( !readMultiple(branch, field.external_roads, "external_road") )
                return false;
        }

        return ( !m_strict || (ok_id && ok_n) );
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Field: ") + e.what());
        return false;
    }
}

bool AroXMLInDocument::read( const ReadHandler & base, Machine &m){

    m = Machine();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        size_t optionalCount = 0;
        size_t requiredCount = 0;
        int tmp;

        optionalCount += getValue(RHTree(base), "id", m.id);

        if(getValue(RHTree(base), getTag(m.machinetype), tmp)){
            ++requiredCount;
            m.machinetype = Machine::intToMachineType( tmp );
        }

        optionalCount += getValue(RHTree(base), "manufacturer", m.manufacturer);
        optionalCount += getValue(RHTree(base), "model", m.model);
        optionalCount += getValue(RHTree(base), "width", m.width);
        optionalCount += getValue(RHTree(base), "length", m.length);
        optionalCount += getValue(RHTree(base), "height", m.height);
        optionalCount += getValue(RHTree(base), "weight", m.weight);
        requiredCount += getValue(RHTree(base), "bunker_mass", m.bunker_mass);
        optionalCount += getValue(RHTree(base), "bunker_volume", m.bunker_volume);
        requiredCount += getValue(RHTree(base), "working_width", m.working_width);
        requiredCount += getValue(RHTree(base), "max_speed_empty", m.max_speed_empty);
        requiredCount += getValue(RHTree(base), "max_speed_full", m.max_speed_full);
        requiredCount += getValue(RHTree(base), "def_working_speed", m.def_working_speed);
        getValue(RHTree(base), "turning_radius", m.turning_radius);
        getValue(RHTree(base), "axis_distance", m.axis_distance);
        getValue(RHTree(base), "gauge", m.gauge);
        getValue(RHTree(base), "engine_power", m.engine_power);
        getValue(RHTree(base), "knifes_count", m.knifes_count);

        if(getValue(RHTree(base), getTag(m.machineassignment), tmp))
            m.machineassignment = Machine::intToMachineAssignment( tmp );

        size_t unload_sides;
        if(!getValue(RHTree(base), "unload_sides", unload_sides)){
            m.unload_sides = 0;
            if(m.machinetype == Machine::HARVESTER)
                m.unload_sides = Machine::DefaultDownloadSides;
        }
        else
            m.unload_sides = unload_sides;

        getValue(RHTree(base), "id_intern", m.id_intern);

        return (requiredCount == 6) && ( !m_strict || optionalCount == 8 );
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Machine: ") + e.what());
        return false;
    }

}

bool AroXMLInDocument::read( const ReadHandler & base, std::vector<Machine> &machines){
    return readMultiple(base, machines, UseDefaultTag, true);
}

bool AroXMLInDocument::read( const ReadHandler & base, Route &route){

    route = Route();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    bool ok_mid = false;
    bool ok_rid = false;
    ReadHandler branch;

    ok_mid = getValue(RHTree(base), "machine_id", route.machine_id);
    ok_rid = getValue(RHTree(base), "route_id", route.route_id);

    if( getBranch(base, getTag(route.route_points), branch) ){
        if(!read(branch, route.route_points))
            return false;
    }

    return ( !m_strict || (ok_mid && ok_rid) );
}

bool AroXMLInDocument::read( const ReadHandler & base, std::vector<Route> &routes){
    return readMultiple(base, routes, UseDefaultTag, true);
}

bool AroXMLInDocument::read( const ReadHandler & base, HeadlandRoute &route){

    route = HeadlandRoute();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }


    bool ok_mid = false;
    bool ok_rid = false;
    bool ok_sfid = false;
    ReadHandler branch;


    ok_mid = getValue(RHTree(base), "machine_id", route.machine_id);
    ok_rid = getValue(RHTree(base), "route_id", route.route_id);
    ok_sfid = getValue(RHTree(base), "subfield_id", route.subfield_id);

    if( getBranch(base, getTag(route.route_points), branch) ){
        if(!read(branch, route.route_points))
            return false;
    }

    return ( !m_strict || (ok_mid && ok_rid && ok_sfid) );
}

bool AroXMLInDocument::read( const ReadHandler & base, std::vector<HeadlandRoute> &routes){
    return readMultiple(base, routes, UseDefaultTag, true);
}

bool AroXMLInDocument::read( const ReadHandler & base, std::pair< MachineId_t, MachineDynamicInfo> &dynamicInfo){

    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    ReadHandler branch;

    dynamicInfo.second.bunkerMass = 0;
    dynamicInfo.second.bunkerVolume = 0;

    if(!getValue(RHTree(base), "machine_id", dynamicInfo.first)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "MachineDynamicInfo: MachineId not found");
        return false;
    }
    if(!getBranch(base, getTag(dynamicInfo.second.position), branch)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "MachineDynamicInfo: Position not found");
        return false;
    }
    else{
        if( !read(branch, dynamicInfo.second.position) )
            return false;
    }
    getValue(RHTree(base), "bunkerMass", dynamicInfo.second.bunkerMass);
    getValue(RHTree(base), "bunkerVolume", dynamicInfo.second.bunkerVolume);
    return true;
}

bool AroXMLInDocument::read( const ReadHandler & base, std::map<MachineId_t, MachineDynamicInfo> &dynamicInfo){

    dynamicInfo.clear();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    size_t count = 0;

    auto tag_mdi = getTag<MachineDynamicInfo>();

    BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, RHTree(base) ){
        if(v.first == tag_mdi){
            ++count;
            std::pair<MachineId_t, MachineDynamicInfo> di;
            if ( !read( createRH(v.second), di) ){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error reading machine dynamic info " + std::to_string(count));
                return false;
            }
            dynamicInfo.insert(di);
        }
    }
    return true;
}

bool AroXMLInDocument::read(const ReadHandler & base, OutFieldInfo &ofi){

    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    ReadHandler branch;

    if( getBranch(base, "mapAccessPoint2ResourcePoint", branch) ){
        if(!read(branch, ofi.mapAccessPoint2ResourcePoint()))
            return false;
    }
    if( getBranch(base, "mapResourcePoint2AccessPoint", branch) ){
        if(!read(branch, ofi.mapResourcePoint2AccessPoint()))
            return false;
    }
    if( getBranch(base, getTag(ofi.mapUnloadingCosts()), branch) ){
        if(!read(branch, ofi.mapUnloadingCosts()))
            return false;
    }
    if( getBranch(base, getTag(ofi.mapArrivalCosts()), branch) ){
        if(!read(branch, ofi.mapArrivalCosts()))
            return false;
    }
    return true;
}

bool AroXMLInDocument::read( const ReadHandler & base, std::map<int, std::map<int, std::map<OutFieldInfo::MachineId_t, std::map<OutFieldInfo::MachineBunkerState, OutFieldInfo::TravelCosts> > > > &map){

    map.clear();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    OutFieldInfo::TravelCosts tc;
    auto tag_mbs = getTag<OutFieldInfo::MachineBunkerState>();
    auto tag_tc = getTag(tc);

    BOOST_FOREACH( boost::property_tree::ptree::value_type const& v1, RHTree(base)){
        if(v1.first == "from"){
            bool ok_from = false;

            int id_from;
            if(!getValue(v1.second, "id", id_from))
                return false;

            BOOST_FOREACH( boost::property_tree::ptree::value_type const& v2, v1.second){
                if(v2.first == "to"){
                    bool ok_to = false;

                    int id_to;
                    if(!getValue(v2.second, "id", id_to))
                        return false;

                    BOOST_FOREACH( boost::property_tree::ptree::value_type const& m, v2.second){
                        if(m.first == "machine"){
                            bool ok_m = false;

                            OutFieldInfo::MachineId_t m_id;
                            if(!getValue(m.second, "id", m_id))
                                return false;

                            BOOST_FOREACH( boost::property_tree::ptree::value_type const& s, m.second){
                                if(s.first == tag_mbs){
                                    bool ok_s = false;
                                    int tmp;

                                    if(!getValue(s.second, "state", tmp))
                                        return false;
                                    OutFieldInfo::MachineBunkerState bs = OutFieldInfo::intToMachineBunkerState(tmp);

                                    ReadHandler branch;
                                    if(!getBranch(s.second, tag_tc, RHTree(branch)))
                                        return false;

                                    if(!read(branch, tc))
                                        return false;

                                    map[id_from][id_to][m_id][bs]=tc;
                                    ok_from = ok_to = ok_m = ok_s = true;

                                    if(!ok_s)
                                        return false;
                                }
                            }
                            if(!ok_m)
                                return false;
                        }
                    }
                    if(!ok_to)
                        return false;
                }
            }
            if(!ok_from)
                return false;
        }
    }
    return true;
}

bool AroXMLInDocument::read( const ReadHandler & base, OutFieldInfo::MapUnloadingCosts_t &map){

    map.clear();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    OutFieldInfo::UnloadingCosts uc;
    auto tag_uc = getTag(uc);

    BOOST_FOREACH( boost::property_tree::ptree::value_type const& rp, RHTree(base)){
        if(rp.first == "resource_point"){
            bool ok_rp;
            ResourcePointId_t rp_id;
            if(!getValue(rp.second, "id", rp_id))
                return false;

            BOOST_FOREACH( boost::property_tree::ptree::value_type const& m, rp.second){
                if(m.first == "machine"){
                    bool ok_m;

                    OutFieldInfo::MachineId_t m_id;
                    if(!getValue(m.second, "id", m_id))
                        return false;

                    ReadHandler branch;
                    if(!getBranch(m.second, tag_uc, RHTree(branch)))
                        return false;

                    if(!read(branch, uc))
                        return false;

                    map[rp_id][m_id]=uc;
                    ok_rp = ok_m = true;

                    if(!ok_m)
                        return false;
                }
            }
            if(!ok_rp)
                return false;
        }
    }
    return true;
}

bool AroXMLInDocument::read( const ReadHandler & base, OutFieldInfo::MapArrivalCosts_t &map){

    map.clear();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    OutFieldInfo::TravelCosts ac;
    auto tag_ac = getTag(ac);

    BOOST_FOREACH( boost::property_tree::ptree::value_type const& fap, RHTree(base)){
        if(fap.first == "field_access_point"){
            bool ok_fap;
            FieldAccessPointId_t fap_id;
            if(!getValue(fap.second, "id", fap_id))
                return false;

            BOOST_FOREACH( boost::property_tree::ptree::value_type const& m, fap.second){
                if(m.first == "machine"){
                    bool ok_m;

                    OutFieldInfo::MachineId_t m_id;
                    if(!getValue(m.second, "id", m_id))
                        return false;

                    BOOST_FOREACH( boost::property_tree::ptree::value_type const& s, m.second){
                        if(s.first == "bunker_state"){
                            bool ok_s = false;
                            int tmp;

                            if(!getValue(s.second, "state", tmp))
                                return false;
                            OutFieldInfo::MachineBunkerState bs = OutFieldInfo::intToMachineBunkerState(tmp);

                            ReadHandler branch;
                            if(!getBranch(s.second, tag_ac, RHTree(branch)))
                                return false;

                            if(!read(branch, ac))
                                return false;

                            map[fap_id][m_id][bs]=ac;
                            ok_fap = ok_m = ok_s = true;

                            if(!ok_s)
                                return false;
                        }
                    }
                    if(!ok_m)
                        return false;
                }
            }
            if(!ok_fap)
                return false;
        }
    }
    return true;
}

bool AroXMLInDocument::read( const ReadHandler & base, OutFieldInfo::TravelCosts &tc)
{
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    bool ok_t = false;

    ok_t = getValue(RHTree(base), "time", tc.time);
    getValue(RHTree(base), "time_per_kg", tc.time_per_kg);
    getValue(RHTree(base), "distance", tc.distance);

    return ok_t;
}

bool AroXMLInDocument::read( const ReadHandler & base, OutFieldInfo::UnloadingCosts &uc)
{
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    bool ok_t = false;

    ok_t = getValue(RHTree(base), "time", uc.time);
    getValue(RHTree(base), "time_per_kg", uc.time_per_kg);

    return ok_t;
}

bool AroXMLInDocument::read(const XMLInDocument::ReadHandler &base, DirectedGraph::Graph &graph)
{
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    graph.clear();

    //Note: the new vertices and edges ids may differ from the originals
    std::unordered_map<DirectedGraph::vertex_t, DirectedGraph::vertex_t> verticesMap;// < old_vertex_id <vertex_prop, new_vertex_id> >
    std::unordered_map<std::string, DirectedGraph::edge_t> edgesMap;// < old_edge_id <edge_prop, new_edge_id> >
    std::vector< std::pair<DirectedGraph::edge_t, std::string> > revEdges;

    ReadHandler branch;

    if(!getBranch(base, "vertices", branch) )
        return false;

    size_t count = 1;
    auto tag_vt = getTag<DirectedGraph::vertex_pair>();

    BOOST_FOREACH( boost::property_tree::ptree::value_type const& val, RHTree(branch)){
        if(val.first == tag_vt){
            DirectedGraph::vertex_pair old_vt;
            if(!read(createRH(val.second), old_vt)){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error reading vertex on entry " + std::to_string(count));
                return false;
            }
            verticesMap[old_vt.first] = graph.addVertex(old_vt.second);
            ++count;
        }
    }

    if(!getBranch(base, "edges", branch) )
        return false;

    count = 0;
    auto tag_e = getTag<DirectedGraph::edge_pair>();

    BOOST_FOREACH( boost::property_tree::ptree::value_type const& val, RHTree(branch)){
        if(val.first == tag_e){
            DirectedGraph::vertex_t vt_from, vt_to;
            std::string edge, revEdge;
            DirectedGraph::edge_property e_prop;

            if(!read(createRH(val.second), edge, vt_from, vt_to, e_prop, revEdge)){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error reading edge on entry " + std::to_string(count));
                return false;
            }

            auto it_vt_from = verticesMap.find(vt_from);
            if(it_vt_from == verticesMap.end()){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Vertex [from] " + std::to_string(vt_from) + " of edge " + edge + " on entry " + std::to_string(count) + " not found.");
                return false;
            }

            auto it_vt_to = verticesMap.find(vt_to);
            if(it_vt_to == verticesMap.end()){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Vertex [to] " + std::to_string(vt_to) + " of edge " + edge + " on entry " + std::to_string(count) + " not found.");
                return false;
            }

            if(!graph.addEdge(it_vt_from->second, it_vt_to->second, e_prop, false)){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error adding edge " + edge + " on entry " + std::to_string(count));
                return false;
            }

            DirectedGraph::edge_t new_edge;
            bool found;
            boost::tie(new_edge, found) = boost::edge(it_vt_from->second, it_vt_to->second, graph);

            if(!found){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error retrieving edge " + edge + " on entry " + std::to_string(count));
                return false;
            }

            edgesMap[edge] = new_edge;
            if(!revEdge.empty())
                revEdges.emplace_back( std::make_pair(new_edge, revEdge) );
            ++count;
        }
    }

    for(auto& revEdge : revEdges){
        auto it_edge = edgesMap.find(revEdge.second);
        if(it_edge == edgesMap.end()){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Reverse edge " + revEdge.second + " not found.");
            return false;
        }

        DirectedGraph::edge_property& e_prop = graph[revEdge.first];
        e_prop.revEdge = it_edge->second;
    }

    return true;
}

bool AroXMLInDocument::read(const XMLInDocument::ReadHandler &base, DirectedGraph::vertex_pair &vt)
{
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    if(!getValue(RHTree(base), "id", vt.first))
        return false;

    ReadHandler branch;

    if(!getBranch(base, getTag(vt.second), branch))
        return false;
    if(!read(branch, vt.second))
        return false;

    return true;

}

bool AroXMLInDocument::read(const XMLInDocument::ReadHandler &base, std::string &edge, DirectedGraph::vertex_t &vt_from, DirectedGraph::vertex_t &vt_to,
                                                                    DirectedGraph::edge_property &e_prop, std::string &revEdge)
{
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    if(!getValue(RHTree(base), getTag<DirectedGraph::edge_t>(), edge))
        return false;
    if(!getValue(RHTree(base), "vt_from", vt_from))
        return false;
    if(!getValue(RHTree(base), "vt_to", vt_to))
        return false;

    ReadHandler branch;

    if(!getBranch(base, getTag(e_prop), branch))
        return false;
    if(!read(branch, e_prop, revEdge))
        return false;

    return true;

}

bool AroXMLInDocument::read(const XMLInDocument::ReadHandler &base, DirectedGraph::vertex_property &v_prop)
{
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    if(!getValue(RHTree(base), "workingMachineId", v_prop.harvester_id))
        return false;

    ReadHandler branch;

    if(!getBranch(base, getTag(v_prop.route_point), branch))
        return false;
    if(!read(branch, v_prop.route_point))
        return false;

    v_prop.unloadingCosts.clear();
    if( getBranch(base, "unloadingCosts", branch) ){
        OutFieldInfo::UnloadingCosts uc;
        auto tag_uc = getTag(uc);

        BOOST_FOREACH( boost::property_tree::ptree::value_type const& val, RHTree(branch)){
            if(val.first == "machine"){
                MachineId_t m_id;

                if(!getValue(val.second, "machine_id", m_id))
                    return false;

                ReadHandler branch2;

                if(!getBranch(val.second, tag_uc, RHTree(branch2)))
                    return false;
                if(!read(branch2, uc))
                    return false;

                v_prop.unloadingCosts[m_id] = uc;
            }
        }
    }

    v_prop.visitPeriods.clear();
    if( getBranch(base, "visitPeriods", branch) ){
        DirectedGraph::VisitPeriod vp;
        auto tag_vp = getTag(vp);

        BOOST_FOREACH( boost::property_tree::ptree::value_type const& val, RHTree(branch)){
            if(val.first == tag_vp){
                double key;

                if(!getValue(val.second, "key", key))
                    return false;

                ReadHandler branch2;

                if(!getBranch(val.second, tag_vp, RHTree(branch2)))
                    return false;
                if(!read( branch2, vp))
                    return false;

                v_prop.visitPeriods.insert( std::make_pair(key, vp) );
            }
        }
    }

    return true;

}

bool AroXMLInDocument::read(const XMLInDocument::ReadHandler &base, DirectedGraph::edge_property &e_prop, std::string &revEdge)
{
    try{

        if(!m_isDocOpen){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
            return false;
        }
        if(!getValue(RHTree(base), "bidir", e_prop.bidirectional))
            return false;
        if(e_prop.bidirectional & !getValue(RHTree(base), "rev_edge", revEdge))
            return false;
        if(!getValue(RHTree(base), "def_width", e_prop.defWidth))
            return false;
        if(!getValue(RHTree(base), "dist", e_prop.distance))
            return false;


        int tmp;
        if(!getValue(RHTree(base), getTag(e_prop.edge_type), tmp))
            return false;
        e_prop.edge_type = DirectedGraph::intToEdgeType(tmp);

        ReadHandler branch;

        if(!getBranch(base, "p_from", branch))
            return false;
        if(!read(branch, e_prop.p0))
            return false;

        if(!getBranch(base, "p_to", branch))
            return false;
        if(!read(branch, e_prop.p1))
            return false;

        if(getBranch(base, "arrival_costs", branch)){
            if(!read(branch, e_prop.arrivalCosts))
                return false;
        }

        e_prop.overruns.clear();
        if( getBranch(base, getTag(e_prop.overruns), branch) ){
            if(!readMultiple(branch, e_prop.overruns, UseDefaultTag, true))
                return false;
        }

        return true;

    }
    catch(...){
        return false;
    }

}

bool AroXMLInDocument::read(const XMLInDocument::ReadHandler &base, DirectedGraph::VisitPeriod &vp)
{
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    if(!getValue(RHTree(base), "machine_id", vp.machineId))
        return false;
    if(!getValue(RHTree(base), "timestamp", vp.timestamp))
        return false;
    if(!getValue(RHTree(base), "time_in", vp.time_in))
        return false;
    if(!getValue(RHTree(base), "time_out", vp.time_out))
        return false;

    ReadHandler branch;

    vp.next_vt.clear();
    if( getBranch(base, "next_vts", branch) ){
        RoutePoint rp;
        auto tag_rp = getTag(rp);

        BOOST_FOREACH( boost::property_tree::ptree::value_type const& val, RHTree(branch)){
            if(val.first == "vertex"){
                DirectedGraph::vertex_t vt;
                ReadHandler branch2;

                if(!getValue(val.second, "vertex_id", vt))
                    return false;

                if(!getBranch(val.second, tag_rp, RHTree(branch2)))
                    return false;
                if(!read(branch2, rp))
                    return false;

                vp.next_vt.emplace_back( std::make_pair(vt, rp) );
            }
        }
    }

    return true;

}

bool AroXMLInDocument::read(const XMLInDocument::ReadHandler &base, DirectedGraph::overroll_property &o)
{
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    if(!getValue(RHTree(base), "machine_id", o.machine_id))
        return false;
    if(!getValue(RHTree(base), "weight", o.weight))
        return false;
    if(!getValue(RHTree(base), "duration", o.duration))
        return false;

    return true;

}

bool AroXMLInDocument::readField(const std::string &filename, Field &field, Point::ProjectionType coordinatesType_out, const std::vector<std::string> &parentTags, LogLevel logLevel)
{
    AroXMLInDocument doc;

    doc.setCoordinatesType(coordinatesType_out);

    field.clear();

    if( !doc.openFile(filename) )
        return false;

    //required
    return doc.read( field, getTag(field), parentTags ) &&
           doc.closeFile();

}

bool AroXMLInDocument::readField(const std::string &filename, Field &field, OutFieldInfo &outFieldInfo, Point::ProjectionType coordinatesType_out, const std::vector<std::string> &parentTags, LogLevel logLevel)
{
    AroXMLInDocument doc;

    doc.setCoordinatesType(coordinatesType_out);

    field.clear();
    outFieldInfo.clearAll();

    if( !doc.openFile(filename) )
        return false;

    //optional
    doc.read(field, getTag(outFieldInfo), parentTags);

    //required
    return doc.read( field, getTag(field), parentTags )
           && doc.closeFile();

}

bool AroXMLInDocument::readFields(const std::string &filename, std::vector<Field> &fields, Point::ProjectionType coordinatesType_out, const std::vector<std::string> &parentTags, LogLevel logLevel)
{
    AroXMLInDocument doc(logLevel);

    doc.setCoordinatesType(coordinatesType_out);

    fields.clear();

    if( !doc.openFile(filename) || !doc.openDocument() )
        return false;

    //required
    return doc.readMultiple( fields, UseDefaultTag, parentTags, false )
           && doc.closeDocument()
           && doc.closeFile();

}

bool AroXMLInDocument::readMachine(const std::string &filename,
                                   Machine &machine,
                                   const std::vector<std::string> &parentTags,
                                   LogLevel logLevel)
{
    AroXMLInDocument doc(logLevel);

    if( !doc.openFile(filename) )
        return false;

    //required
    return doc.read( machine , getTag(machine), parentTags)
           && doc.closeFile();
}

bool AroXMLInDocument::readMachines(const std::string &filename,
                                 std::vector<Machine> &machines,
                                 const std::vector<std::string> &parentTags, LogLevel logLevel)
{
    AroXMLInDocument doc(logLevel);
    machines.clear();

    if( !doc.openFile(filename) )
        return false;

    //required
    return doc.read( machines , getTag(machines), parentTags)
           && doc.closeFile();
}

bool AroXMLInDocument::readConfigParameters(const std::string &filename,
                                         std::map<std::string, std::map<std::string, std::string> > &configParameters,
                                         const std::vector<std::string> &parentTags, LogLevel logLevel)
{
    AroXMLInDocument doc(logLevel);
    bool ok = true;
    configParameters.clear();

    if( !doc.openFile(filename) )
        return false;

    //required
    return doc.read( configParameters, "configParameters", parentTags)
           && doc.closeFile();

}

bool AroXMLInDocument::readOutFieldInfo(const std::string &filename, OutFieldInfo &outFieldInfo, const std::vector<std::string> &parentTags, LogLevel logLevel)
{
    AroXMLInDocument doc(logLevel);
    bool ok = true;
    outFieldInfo.clearAll();

    if( !doc.openFile(filename) )
        return false;

    //required
    return doc.read( outFieldInfo , getTag(outFieldInfo), parentTags)
           && doc.closeFile();

}

bool AroXMLInDocument::readPlanParameters(const std::string &filename,
                                       Field &field,
                                       std::vector<Machine> &workingGroup,
                                       std::map<std::string, std::map<std::string, std::string> > &configParameters,
                                       OutFieldInfo &outFieldInfo,
                                       std::map<MachineId_t, MachineDynamicInfo> &machinesDynamicInfo,
                                       std::string &yieldmap_tifBase64,
                                       std::string &drynessmap_tifBase64,
                                       std::string &soilmap_tifBase64,
                                       std::string &remainingAreaMap_tifBase64, Point::ProjectionType coordinatesType_out,
                                       const std::vector<std::string> &parentTags,
                                       LogLevel logLevel)
{
    AroXMLInDocument doc(logLevel);

    doc.setCoordinatesType(coordinatesType_out);

    field.clear();
    workingGroup.clear();
    configParameters.clear();
    outFieldInfo.clearAll();
    machinesDynamicInfo.clear();
    yieldmap_tifBase64.clear();
    drynessmap_tifBase64.clear();
    soilmap_tifBase64.clear();
    remainingAreaMap_tifBase64.clear();

    if( !doc.openFile(filename) || !doc.openDocument() )
        return false;

    ReadHandler branch;
    if(!doc.getBranchHandler(parentTags, branch))
        return false;

    //optional
    doc.read( yieldmap_tifBase64, branch, "yieldmap_tifBase64" );
    doc.read( drynessmap_tifBase64, branch, "drynessmap_tifBase64" );
    doc.read( soilmap_tifBase64, branch, "soilmap_tifBase64" );
    doc.read( remainingAreaMap_tifBase64, branch, "remainingAreaMap_tifBase64" );
    doc.read( outFieldInfo, branch );
    doc.read( machinesDynamicInfo, branch );

    //required
    return doc.read( field, branch )
           && doc.read( workingGroup, branch )
           && doc.read( configParameters, branch, "configParameters" )
           && doc.closeFile();
}

bool AroXMLInDocument::readPlanParameters(const std::string &filename,
                                       std::vector<Machine> &workingGroup,
                                       std::map<std::string, std::map<std::string, std::string> > &configParameters,
                                       OutFieldInfo &outFieldInfo,
                                       std::map<MachineId_t, MachineDynamicInfo> &machinesDynamicInfo,
                                       std::string &yieldmap_tifBase64,
                                       std::string &drynessmap_tifBase64,
                                       std::string &soilmap_tifBase64,
                                       std::string &remainingAreaMap_tifBase64, Point::ProjectionType coordinatesType_out,
                                       const std::vector<std::string> &parentTags, LogLevel logLevel)
{
    AroXMLInDocument doc(logLevel);

    doc.setCoordinatesType(coordinatesType_out);

    workingGroup.clear();
    configParameters.clear();
    outFieldInfo.clearAll();
    machinesDynamicInfo.clear();
    yieldmap_tifBase64.clear();
    drynessmap_tifBase64.clear();
    soilmap_tifBase64.clear();
    remainingAreaMap_tifBase64.clear();

    if( !doc.openFile(filename) || !doc.openDocument() )
        return false;

    ReadHandler branch;
    if(!doc.getBranchHandler(parentTags, branch))
        return false;

    //optional
    doc.read( yieldmap_tifBase64, branch, "yieldmap_tifBase64" );
    doc.read( drynessmap_tifBase64, branch, "drynessmap_tifBase64" );
    doc.read( soilmap_tifBase64, branch, "soilmap_tifBase64" );
    doc.read( remainingAreaMap_tifBase64, branch, "remainingAreaMap_tifBase64" );
    doc.read( outFieldInfo, branch );
    doc.read( machinesDynamicInfo, branch );

    //required
    return doc.read( workingGroup, branch )
           && doc.read( configParameters, branch, "configParameters" )
           && doc.closeFile();

}

bool AroXMLInDocument::readPlan(const std::string &filename,
                             std::map<int, std::vector<Route> > &routes, bool syncRoutes, Point::ProjectionType coordinatesType_out,
                             const std::vector<std::string> &parentTags,
                             LogLevel logLevel)
{
    AroXMLInDocument doc(logLevel);
    Logger logger(logLevel, "AroXMLInDocument");

    doc.setCoordinatesType(coordinatesType_out);

    routes.clear();

    if( !doc.openFile(filename) || !doc.openDocument() )
        return false;

    std::vector<std::string> parentTags_plan = parentTags;
    parentTags_plan.emplace_back("plan");

    ReadHandler branch;
    if(!doc.getBranchHandler(parentTags_plan, branch))
        return false;

    size_t count = 0;
    return doc.getMultiBranchHandlers(branch,
                                      getTag<Subfield>(),
                                      [&](const ReadHandler& rh) {
                                          count++;
                                          int id;
                                          if( !doc.read(id, rh, "id" ) ){
                                              logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error reading 'id' of element #" + std::to_string( count ));
                                              return true;
                                          }
                                          auto it = routes.find(id);
                                          if(it != routes.end()){
                                              logger.printOut(LogLevel::WARNING, __FUNCTION__, "Repeated 'id' " + std::to_string( id ) + " in element #" + std::to_string( count ) + ". Desregarding input.");
                                              return true;
                                          }
                                          std::vector<Route> rts;
                                          ReadHandler branch;
                                          doc.getBranch( rh, getTag(rts), branch );
                                          if( !doc.read(branch, rts ) ){
                                              logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error reading 'routes' of element #" + std::to_string( count ));
                                              return true;
                                          }
                                          if(syncRoutes)
                                              Route::syncRoutes(rts);
                                          routes[id] = rts;
                                          return true;
                                      })

            && doc.closeFile();

}

bool AroXMLInDocument::readPlan(const std::string &filename,
                             std::map<int, std::vector<HeadlandRoute> > &routes,
                             bool syncRoutes, Point::ProjectionType coordinatesType_out,
                             const std::vector<std::string> &parentTags,
                             LogLevel logLevel)
{
    AroXMLInDocument doc(logLevel);
    Logger logger(logLevel, "AroXMLInDocument");

    doc.setCoordinatesType(coordinatesType_out);

    routes.clear();

    if( !doc.openFile(filename) || !doc.openDocument() )
        return false;

    std::vector<std::string> parentTags_plan = parentTags;
    parentTags_plan.emplace_back("plan");

    ReadHandler branch;
    if(!doc.getBranchHandler(parentTags_plan, branch))
        return false;

    size_t count = 0;
    return doc.getMultiBranchHandlers(branch,
                                      getTag<Subfield>(),
                                      [&](const ReadHandler& rh) {
                                          count++;
                                          int id;
                                          if( !doc.read(id, rh, "id" ) ){
                                              logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error reading 'id' of element #" + std::to_string( count ));
                                              return true;
                                          }
                                          auto it = routes.find(id);
                                          if(it != routes.end()){
                                              logger.printOut(LogLevel::WARNING, __FUNCTION__, "Repeated 'id' " + std::to_string( id ) + " in element #" + std::to_string( count ) + ". Desregarding input.");
                                              return true;
                                          }
                                          std::vector<HeadlandRoute> rts;
                                          ReadHandler branch;
                                          doc.getBranch( rh, getTag(rts), branch );
                                          if( !doc.read(branch, rts ) ){
                                              logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error reading 'routes' of element #" + std::to_string( count ));
                                              return true;
                                          }
                                          if(syncRoutes)
                                              HeadlandRoute::syncRoutes(rts);
                                          routes[id] = rts;
                                          return true;
                                      })

            && doc.closeFile();
}

bool AroXMLInDocument::readPlan(const std::string &filename,
                             Field &field,
                             std::vector<Machine> &workingGroup,
                             std::map<int, std::vector<Route> > &routes,
                             std::string &yieldmap_tifBase64,
                             std::string &drynessmap_tifBase64,
                             std::string &soilmap_tifBase64,
                             std::string &remainingAreaMap_tifBase64,
                             bool syncRoutes, Point::ProjectionType coordinatesType_out,
                             const std::vector<std::string> &parentTags,
                             LogLevel logLevel)
{
    AroXMLInDocument doc(logLevel);
    Logger logger(logLevel, "AroXMLInDocument");

    doc.setCoordinatesType(coordinatesType_out);

    routes.clear();

    if( !doc.openFile(filename) || !doc.openDocument() )
        return false;

    std::vector<std::string> parentTags_plan = parentTags;
    parentTags_plan.emplace_back("plan");

    ReadHandler branch;
    if(!doc.getBranchHandler(parentTags_plan, branch))
        return false;

    //optional
    doc.read( yieldmap_tifBase64, branch, "yieldmap_tifBase64" );
    doc.read( drynessmap_tifBase64, branch, "drynessmap_tifBase64" );
    doc.read( soilmap_tifBase64, branch, "soilmap_tifBase64" );
    doc.read( remainingAreaMap_tifBase64, branch, "remainingAreaMap_tifBase64" );

    size_t count = 0;
    return doc.read(field, branch)
           && doc.read(workingGroup, branch)
           && doc.getMultiBranchHandlers(branch,
                                      getTag<Subfield>(),
                                      [&](const ReadHandler& rh) {
                                          count++;
                                          int id;
                                          if( !doc.read(id, rh, "id" ) ){
                                              logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error reading 'id' of element #" + std::to_string( count ));
                                              return true;
                                          }
                                          auto it = routes.find(id);
                                          if(it != routes.end()){
                                              logger.printOut(LogLevel::WARNING, __FUNCTION__, "Repeated 'id' " + std::to_string( id ) + " in element #" + std::to_string( count ) + ". Desregarding input.");
                                              return true;
                                          }
                                          std::vector<Route> rts;
                                          ReadHandler branch;
                                          doc.getBranch( rh, getTag(rts), branch );
                                          if( !doc.read(branch, rts ) ){
                                              logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error reading 'routes' of element #" + std::to_string( count ));
                                              return true;
                                          }
                                          if(syncRoutes)
                                              Route::syncRoutes(rts);
                                          routes[id] = rts;
                                          return true;
                                      })

            && doc.closeFile();
}

bool AroXMLInDocument::readPlan(const std::string &filename,
                                Field &field, std::vector<Machine> &workingGroup,
                                std::map<int, std::vector<Route> > &routes,
                                std::map<std::string, ArolibGrid_t> &gridmaps,
                                bool syncRoutes,
                                Point::ProjectionType coordinatesType_out,
                                const std::vector<std::string> &parentTags,
                                LogLevel logLevel)
{
    AroXMLInDocument doc(logLevel);
    Logger logger(logLevel, "AroXMLInDocument");

    doc.setCoordinatesType(coordinatesType_out);

    routes.clear();
    gridmaps.clear();

    if( !doc.openFile(filename) || !doc.openDocument() )
        return false;

    std::vector<std::string> parentTags_plan = parentTags;
    parentTags_plan.emplace_back("plan");

    ReadHandler branch;
    if(!doc.getBranchHandler(parentTags_plan, branch))
        return false;

    //optional
    ReadHandler branch_gridmaps;
    if(doc.getBranchHandler(branch, "gridmaps", branch_gridmaps)){
        BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, RHTree(branch_gridmaps) ){

            try{
                std::string mapStr_b64;
                if( !doc.read(createRH(v.second), mapStr_b64) )
                    continue;

                ArolibGrid_t gridmap;
                if(!gridmap.readGridFromGeoTiffString( base64_decode(mapStr_b64) ))
                    continue;
                gridmaps[v.first] = gridmap;
            }
            catch(...){

            }
        }
    }

    size_t count = 0;
    return doc.read(field, branch)
           && doc.read(workingGroup, branch)
           && doc.getMultiBranchHandlers(branch,
                                      getTag<Subfield>(),
                                      [&](const ReadHandler& rh) {
                                          count++;
                                          int id;
                                          if( !doc.read(id, rh, "id" ) ){
                                              logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error reading 'id' of element #" + std::to_string( count ));
                                              return true;
                                          }
                                          auto it = routes.find(id);
                                          if(it != routes.end()){
                                              logger.printOut(LogLevel::WARNING, __FUNCTION__, "Repeated 'id' " + std::to_string( id ) + " in element #" + std::to_string( count ) + ". Desregarding input.");
                                              return true;
                                          }
                                          std::vector<Route> rts;
                                          ReadHandler branch;
                                          doc.getBranch( rh, getTag(rts), branch );
                                          if( !doc.read(branch, rts ) ){
                                              logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error reading 'routes' of element #" + std::to_string( count ));
                                              return true;
                                          }
                                          if(syncRoutes)
                                              Route::syncRoutes(rts);
                                          routes[id] = rts;
                                          return true;
                                      })

            && doc.closeFile();

}

bool AroXMLInDocument::readPlan(const std::string &filename,
                             Field &field,
                             std::vector<Machine> &workingGroup,
                             std::map<int, std::vector<HeadlandRoute> > &routes,
                             std::string &yieldmap_tifBase64,
                             std::string &drynessmap_tifBase64,
                             std::string &soilmap_tifBase64,
                             std::string &remainingAreaMap_tifBase64,
                             bool syncRoutes, Point::ProjectionType coordinatesType_out,
                             const std::vector<std::string> &parentTags,
                             LogLevel logLevel)
{
    AroXMLInDocument doc(logLevel);
    Logger logger(logLevel, "AroXMLInDocument");

    doc.setCoordinatesType(coordinatesType_out);

    routes.clear();

    if( !doc.openFile(filename) || !doc.openDocument() )
        return false;

    std::vector<std::string> parentTags_plan = parentTags;
    parentTags_plan.emplace_back("plan");

    ReadHandler branch;
    if(!doc.getBranchHandler(parentTags_plan, branch))
        return false;

    //optional
    doc.read( yieldmap_tifBase64, branch, "yieldmap_tifBase64" );
    doc.read( drynessmap_tifBase64, branch, "drynessmap_tifBase64" );
    doc.read( soilmap_tifBase64, branch, "soilmap_tifBase64" );
    doc.read( remainingAreaMap_tifBase64, branch, "remainingAreaMap_tifBase64" );

    size_t count = 0;
    return doc.read(field, branch)
           && doc.read(workingGroup, branch)
           && doc.getMultiBranchHandlers(branch,
                                      getTag<Subfield>(),
                                      [&](const ReadHandler& rh) {
                                          count++;
                                          int id;
                                          if( !doc.read(id, rh, "id" ) ){
                                              logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error reading 'id' of element #" + std::to_string( count ));
                                              return true;
                                          }
                                          auto it = routes.find(id);
                                          if(it != routes.end()){
                                              logger.printOut(LogLevel::WARNING, __FUNCTION__, "Repeated 'id' " + std::to_string( id ) + " in element #" + std::to_string( count ) + ". Desregarding input.");
                                              return true;
                                          }
                                          std::vector<HeadlandRoute> rts;
                                          ReadHandler branch;
                                          doc.getBranch( rh, getTag(rts), branch );
                                          if( !doc.read(branch, rts ) ){
                                              logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error reading 'routes' of element #" + std::to_string( count ));
                                              return true;
                                          }
                                          if(syncRoutes)
                                              HeadlandRoute::syncRoutes(rts);
                                          routes[id] = rts;
                                          return true;
                                      })

            && doc.closeFile();
}

bool AroXMLInDocument::readGraph(const std::string &filename, DirectedGraph::Graph &graph, Point::ProjectionType coordinatesType_out, const std::vector<std::string> &parentTags, LogLevel logLevel)
{

    AroXMLInDocument doc(logLevel);

    doc.setCoordinatesType(coordinatesType_out);

    graph.clear();

    if( !doc.openFile(filename) || !doc.openDocument() )
        return false;

    ReadHandler branch;
    if(!doc.getBranchHandler(parentTags, branch))
        return false;

    //required
    return doc.read( graph, branch )
           && doc.closeFile();
}


bool AroXMLInDocument::strict() const
{
    return m_strict;
}

void AroXMLInDocument::setStrict(bool strict)
{
    m_strict = strict;
}

bool AroXMLInDocument::openDoc()
{
    if(!XMLInDocument::openDoc())
        return false;

    readCoodinatesInputType(m_base);

    return true;
}

bool AroXMLInDocument::stringToPoint(std::string coordinates, Point &pt){
    try{
        std::vector<std::string> xyz;
        boost::split(xyz, coordinates, boost::is_any_of(","));

        Point pt_in;
        if (xyz.size() == 2)
            pt_in = Point(string2double(xyz[0]), string2double(xyz[1]), 0);
        else if (xyz.size() == 3)
            pt_in = Point(string2double(xyz[0]), string2double(xyz[1]), string2double(xyz[2]));
        else{
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Cannot parse/split the point '" + coordinates + "'");
            return false;
        }

        if(m_coordinatesType_in == m_coordinatesType_out)
            pt = pt_in;
        else if(m_coordinatesType_in == Point::WGS){
            arolib::CoordTransformer::GetInstance().convert_to_cartesian(pt_in, pt);
        }
        else{
            arolib::CoordTransformer::GetInstance().convert_to_geodetic(pt_in, pt);
        }
        return true;
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Cannot parse/split the point '" + coordinates + "': " + e.what());
        return false;
    }
}

bool AroXMLInDocument::stringToPointList(std::string coordinates, std::vector<Point> &pts){
    boost::trim(coordinates);
    std::vector<std::string> strs;
    boost::split(strs, coordinates, boost::is_any_of(" "));

    pts.clear();
    for(unsigned int i = 0; i < strs.size(); i++){
        if(strs.at(i).empty())
            continue;
        Point pt;
        if( !stringToPoint(strs.at(i), pt) ){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error reading point " + std::to_string(i));
            return false;
        }
        pts.push_back(pt);
    }
    return true;
}

}
}//end namespace arolib

