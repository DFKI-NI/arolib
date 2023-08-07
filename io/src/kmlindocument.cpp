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
 
#include "arolib/io/kmlindocument.hpp"

namespace arolib {

KMLInDocument::KmlTag::KmlTag(const std::string &_kml_tag, const std::string &_name):
    kml_tag(_kml_tag),
    name(_name)
{

}

KMLInDocument::KMLInDocument(LogLevel logLevel):
    AroInDocument(logLevel)
{
    m_logger = Logger(logLevel, __FUNCTION__);
}

KMLInDocument::~KMLInDocument()
{
}

bool KMLInDocument::openDocument(ReadHandler *rh)
{
    if(!AroInDocument::openDocument())
        return false;
    if(rh)
        *rh = m_base;
    return true;
}

bool KMLInDocument::hasBranch(const ReadHandler &base, const std::string &name){
    if( base.t.get_child_optional(name) )
        return true;
    return false;
}

bool KMLInDocument::hasBranch(const std::string &name)
{
    if(!m_isDocOpen)
        return false;
    return hasBranch(m_base, name);
}

bool KMLInDocument::hasBranch(const KMLInDocument::ReadHandler &base, const KMLInDocument::KmlTag &kmlTag)
{
    ReadHandler branch;
    if(!getBranch(base, kmlTag.kml_tag, branch))
        return false;

    std::string name;
    return ( getValue(branch.t, KMLTag_name, name) && name == kmlTag.name );
}

bool KMLInDocument::hasBranch(const KMLInDocument::KmlTag &kmlTag)
{
    if(!m_isDocOpen)
        return false;
    return hasBranch(m_base, kmlTag);
}

bool KMLInDocument::getBranchHandler(const KMLInDocument::ReadHandler &base, const std::string &name, KMLInDocument::ReadHandler &branch, Logger *_logger)
{
    try{
        auto ochild = base.t.get_child_optional(name);
        if(!ochild)
            return false;
        branch.t = *ochild;
        return true;
    }
    catch(...){
        return false;
    }

}

bool KMLInDocument::getBranchHandler(const KMLInDocument::ReadHandler &base, const std::vector<std::string> &tags, KMLInDocument::ReadHandler &branch, Logger* _logger)
{
    branch = base;
    Logger logger(LogLevel::CRITIC, "KMLInDocument");
    logger.setParent(_logger);
    for(size_t i = 0 ; i < tags.size() ; ++i){
        if(!getBranchHandler(branch, tags.at(i), branch, _logger)){
            logger.printOut(LogLevel::WARNING, __FUNCTION__, "Cannot open parent tag '" + std::to_string(i) + ": '" + tags.at(i) + "'");
            return false;
        }
    }
    return true;
}

bool KMLInDocument::getBranchHandler(const std::string &name, ReadHandler &branch)
{
    if(!m_isDocOpen)
        return false;
    return getBranchHandler(m_base, name, branch, &m_logger);

}

bool KMLInDocument::getBranchHandler(const std::vector<std::string> &tags, ReadHandler &branch)
{
    if(!m_isDocOpen)
        return false;
    return getBranchHandler(m_base, tags, branch, &m_logger);
}

bool KMLInDocument::getMultiBranchHandlers(const ReadHandler &base,
                                           const std::string &tag,
                                           const std::function<bool (const KMLInDocument::ReadHandler &)> funct,
                                           const std::vector<std::string> &parentTags,
                                           Logger *_logger)
{
    ReadHandler branch;
    if(!getBranchHandler(base, parentTags, branch, _logger))
        return false;

    BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, branch.t){
        if(v.first == tag){
            if (!funct( ReadHandler(v.second) ) )
                return false;
        }
    }
    return true;
}

bool KMLInDocument::getMultiBranchHandlers(const std::string &tag,
                                           const std::function<bool (const ReadHandler &)> funct,
                                           const std::vector<std::string> &parentTags)
{
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }
    return getMultiBranchHandlers(m_base, tag, funct, parentTags, &m_logger);
}

bool KMLInDocument::readCoodinatesInputType(const KMLInDocument::ReadHandler &base)
{
    m_coordinatesType_in = Point::WGS;
    std::string sType, sType0;
    if( !getValue(base.t, m_coordTypeTag, sType0) )
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

bool KMLInDocument::read(const ReadHandler &base, std::string &value){
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    auto ovalue = base.t.get_value_optional<std::string>();
    if(!ovalue)
        return false;
    value = *ovalue;
    return true;
}

bool KMLInDocument::read( const ReadHandler & base, std::map<std::string, std::map<std::string, std::string> > &values){ // map<tag, map< name , value > >

    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, base.t){
            BOOST_FOREACH( boost::property_tree::ptree::value_type const& vv, v.second){
                values[v.first][vv.first] =  vv.second.get_value<std::string>();
            }
        }
        return true;
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Values map: ") + e.what());
        return false;
    }
}

bool KMLInDocument::read( const ReadHandler & base, Point &pt)
{
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    return stringToPoint(base.t.get_value<std::string>() , pt );
}

bool KMLInDocument::read( const ReadHandler & base, std::vector<Point> &pts)
{
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    return stringToPointList(base.t.get_value<std::string>() , pts );
}

bool KMLInDocument::read( const ReadHandler & base, Linestring &ls){
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ls = Linestring();
        bool ok_id = false;
        ReadHandler branch;

        ok_id = getValue(base.t, "id", ls.id);

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

bool KMLInDocument::read( const ReadHandler & base, Polygon &poly){
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ReadHandler branch;
        if(getBranch(base, getTag(poly.points), branch)){
            if( !read(branch, poly.points) )
                return false;
        }

        return true;
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Polygon: ") + e.what());
        return false;
    }
}

bool KMLInDocument::read( const ReadHandler & base, FieldAccessPoint &pt){

    pt = FieldAccessPoint();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        bool ok_id = false;
        bool ok_at = false;
        bool ok_ap = false;
        ReadHandler branch;
        int tmp;

        ok_id = getValue(base.t, "id", pt.id);

        if(!getBranch(base, getTag(pt.point()), branch))
            return false;
        if(!read(branch, pt.point()))
            return false;

        if( getValue(base.t, getTag(pt.accessType), tmp) ){
            pt.accessType = FieldAccessPoint::intToAccessPointType(tmp);
            ok_at = true;
        }

        if( getValue(base.t, getTag(pt.accessPermission), tmp) ){
            pt.accessPermission = FieldAccessPoint::intToAccessPointPermission(tmp);
            ok_ap = true;
        }

        return ( !m_strict || (ok_id && ok_at && ok_ap) );
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("FieldAccessPoint: ") + e.what());
        return false;
    }
}

bool KMLInDocument::read( const ReadHandler & base, std::vector<FieldAccessPoint> &pts){
    return readMultiple(base, pts, UseDefaultTag, true);
}

bool KMLInDocument::read( const ReadHandler & base, ResourcePoint &pt){

    pt = ResourcePoint();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        bool ok_id = false;
        bool ok_ut = false;
        ReadHandler branch;

        ok_id = getValue(base.t, "id", pt.id);

        if(!getBranch(base, getTag(pt.point()), branch))
            return false;
        if(!read(branch, pt.point()))
            return false;

        if(getBranch(base, getTag(pt.resourceTypes), branch)){
            if(!read(branch, pt.resourceTypes))
                return false;
        }

        ok_ut = getValue(base.t, "defaultUnloadingTime", pt.defaultUnloadingTime);
        getValue(base.t, "defaultUnloadingTimePerKg", pt.defaultUnloadingTimePerKg);

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

bool KMLInDocument::read( const ReadHandler & base, std::vector<ResourcePoint> &pts){
    return readMultiple(base, pts, UseDefaultTag, true);
}


bool KMLInDocument::read(const KMLInDocument::ReadHandler &base, std::set<ResourcePoint::ResourceType> &resourceTypes)
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

bool KMLInDocument::read( const ReadHandler & base, RoutePoint &pt){
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

        if(!getBranch(base, getTag(pt.point), branch))
            return false;
        if(!read(branch, pt.point))
            return false;

        ok_bm = getValue(base.t, "bunker_mass", pt.bunker_mass);
        ok_bv = getValue(base.t, "bunker_volume", pt.bunker_volume);
        ok_hm = getValue(base.t, "harvested_mass", pt.harvested_mass);
        ok_hv = getValue(base.t, "harvested_volume", pt.harvested_volume);
        ok_ts = getValue(base.t, "time_stamp", pt.time_stamp);
        ok_ti = getValue(base.t, "track_id", pt.track_id);
        getValue(base.t, "track_idx", pt.track_idx);

        if( getValue(base.t, getTag(pt.type), tmp) ){
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

bool KMLInDocument::read( const ReadHandler & base, std::vector<RoutePoint> &pts){
    return readMultiple(base, pts, UseDefaultTag, true);
}

bool KMLInDocument::read( const ReadHandler & base, HeadlandPoint &pt){
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

        ok_ti = getValue(base.t, "track_id", pt.track_id);

        return ( !m_strict || (ok_ti) );
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("RoutePoint: ") + e.what());
        return false;
    }
}

bool KMLInDocument::read( const ReadHandler & base, std::vector<HeadlandPoint> &pts){
    return readMultiple(base, pts, UseDefaultTag, true);
}
bool KMLInDocument::read( const ReadHandler & base, Track &track){
    track = Track();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ReadHandler branch;

        getValue(base.t, "id", track.id);

        if(getBranch(base, getTag(track.points), branch)){
            if(!read(branch, track.points))
                return false;
        }

        getValue(base.t, "distToNext", track.distToNext);
        getValue(base.t, "distToPrev", track.distToPrev);

        return (true);
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Track: ") + e.what());
        return false;
    }
}

bool KMLInDocument::read( const ReadHandler & base, std::vector<Track> &tracks){
    return readMultiple(base, tracks, UseDefaultTag, true);
}

bool KMLInDocument::read(const KMLInDocument::ReadHandler &base, Headlands &headlands)
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

        if(getBranch(base, getTag(headlands.partial), branch)){
            if(!read(branch, headlands.partial))
                return false;
        }

        return (true);
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Headlands: ") + e.what());
        return false;
    }

}

bool KMLInDocument::read(const KMLInDocument::ReadHandler &base, CompleteHeadland &hl)
{
    hl = CompleteHeadland();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ReadHandler branch;

        if(getBranch(base, getTag(hl.headland_points), branch)){
            if(!read(branch, hl.headland_points))
                return false;
        }

        if(getBranch(base, "middle_track", branch)){
            if(!read(branch, hl.middle_track))
                return false;
        }

        if(getBranch(base, getTag(hl.tracks), branch)){
            if(!read(branch, hl.tracks))
                return false;
        }

        return (true);
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("CompleteHeadland: ") + e.what());
        return false;
    }

}

bool KMLInDocument::read(const KMLInDocument::ReadHandler &base, PartialHeadland &hl)
{
    hl = PartialHeadland();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ReadHandler branch;
        bool ok_ob = false;

        getValue(base.t, "id", hl.id);

        if(getBranch(base, "boundary", branch)){
            if(!read(branch, hl.boundary))
                return false;
            ok_ob = true;
        }

        if(getBranch(base, getTag(hl.tracks), branch)){
            if(!read(branch, hl.tracks))
                return false;
        }

        return ( !m_strict || (ok_ob) );
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("PartialHeadland: ") + e.what());
        return false;
    }

}

bool KMLInDocument::read(const KMLInDocument::ReadHandler &base, std::vector<PartialHeadland> &headlands)
{
    return readMultiple(base, headlands, UseDefaultTag, true);
}

bool KMLInDocument::read(const KMLInDocument::ReadHandler &base, Obstacle &obs)
{
    obs = Obstacle();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ReadHandler branch;
        int tmp;
        if( getValue(base.t, getTag(obs.type), tmp) ){
            obs.type = Obstacle::intToObstacleType(tmp);
        }

        getValue(base.t, "type_description", obs.type_description);

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

bool KMLInDocument::read(const KMLInDocument::ReadHandler &base, std::vector<Obstacle> &obstacles)
{
    return readMultiple(base, obstacles, UseDefaultTag, true);
}

bool KMLInDocument::read( const ReadHandler & base, Subfield &sf){

    sf = Subfield();
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        bool ok_d = false;

        ReadHandler branch;

        std::string desc;
        ok_d = getValue(base.t, KMLTag_description, desc);

        if(ok_d)
            ok_d = readFromDescription(sf, desc);

        if( !read(sf.boundary_outer, base, KmlTag(KMLTag_placemark, "outer_boundary")) )
            return false;

        if( hasBranch(base, KmlTag(KMLTag_placemark, "inner_boundary")) ){
            if( !read(sf.boundary_inner, base, KmlTag(KMLTag_placemark, "inner_boundary")) )
                return false;
        }

        if( hasBranch(base, KmlTag(KMLTag_folder, getTag(sf.access_points))) ){
            if( !read(sf.boundary_inner, base, KmlTag(KMLTag_placemark, "inner_boundary")) )
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
            BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, branch.t ){
                if(v.first == "reference_line"){
                    sf.reference_lines.push_back( Linestring() );
                    if(!read( ReadHandler(v.second), sf.reference_lines.back())){
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
                    BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, branch.t ){
                        if(v.first == "obstacle"){
                            sf.obstacles.push_back( Obstacle() );
                            sf.obstacles.back().type = Obstacle::OBS_OTHER;
                            if(!read( ReadHandler(v.second), sf.obstacles.back().boundary)){
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
            if(getBranch(base, getTag(sf.headlands.complete.headland_points), branch)){
                if(!read(branch, sf.headlands.complete.headland_points))
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

        return ( !m_strict || (ok_d) );
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Subfield: ") + e.what());
        return false;
    }

}

bool KMLInDocument::read( const ReadHandler & base, std::vector<Subfield> &sfs){
    return readMultiple(base, sfs, KmlTag(KMLTag_folder), true);
}

bool KMLInDocument::read( const ReadHandler & base, Field &field){
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        bool ok_id = false;
        bool ok_n = false;
        bool ok_d = false;
        ReadHandler branch;
        field.clear();

        std::string desc;
        ok_d = getValue(base.t, KMLTag_description, desc);

        if(ok_d)
            ok_d = readFromDescription(field, desc);

        if( !read(field.outer_boundary, branch, KmlTag(KMLTag_placemark, "outer_boundary")) )
            return false;

        if(!getBranch(base, "outer_boundary", branch))
            return false;
        if(!read(branch, field.outer_boundary))
            return false;

        if(getBranch(base, getTag(field.subfields), branch)){
            if( !read(branch, field.subfields) )
                return false;
        }
        else{
            field.subfields.push_back(Subfield());
            field.subfields.back().id = 0;
            field.subfields.back().boundary_outer = field.outer_boundary;
        }

        return ( !m_strict || (ok_id && ok_n) );
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Field: ") + e.what());
        return false;
    }
}


bool KMLInDocument::readField(const std::string &filename, Field &field, Point::ProjectionType coordinatesType_out, const std::vector<std::string> &parentTags, LogLevel logLevel)
{
    KMLInDocument doc;

    doc.setCoordinatesType(coordinatesType_out);

    field.clear();

    if( !doc.openFile(filename) )
        return false;

    //required
    return doc.read( field, KmlTag(KMLTag_folder), parentTags ) &&
           doc.closeFile();

//    ReadHandler branch;
//    if(!getBranch(parentTags, branch))
//        return false;
//    if(!getBranch(branch, KMLTag_folder, branch))
//        return false;
//    std::string name;
//    if(!getValue(branch.t, KMLTag_name, name) || name != getTag(field))
//        return false;

//    //required
//    return doc.read( branch, field ) &&
//           doc.closeFile();

}

bool KMLInDocument::readFields(const std::string &filename, std::vector<Field> &fields, Point::ProjectionType coordinatesType_out, const std::vector<std::string> &parentTags, LogLevel logLevel)
{
    KMLInDocument doc(logLevel);

    doc.setCoordinatesType(coordinatesType_out);

    fields.clear();

    if( !doc.openFile(filename) || !doc.openDocument() )
        return false;

    //required
    return doc.readMultiple( fields, KmlTag(KMLTag_folder), parentTags, false )
           && doc.closeDocument()
           && doc.closeFile();

}

bool KMLInDocument::strict() const
{
    return m_strict;
}

void KMLInDocument::setStrict(bool strict)
{
    m_strict = strict;
}

bool KMLInDocument::openDoc()
{
    if(!AroInDocument::isReadyToRead())
        return false;

    m_is->seekg(0, m_is->beg);

    try{
        read_xml(*m_is, m_base.t);

        if(!getBranchHandler(m_base, m_docTag, m_base)){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Cannot open document tag.");
            return false;
        }

        readCoodinatesInputType(m_base);

        return true;
    }
    catch (std::exception& e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Error reading input stream/file: '") + e.what() + "'");
        return false;
    }
}

bool KMLInDocument::closeDoc()
{
    if(!AroInDocument::isReadyToRead())
        return false;
    m_base = ReadHandler();
    return true;
}

bool KMLInDocument::stringToPoint(std::string coordinates, Point &pt){
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
            pt.utm_zone = arolib::CoordTransformer::GetInstance().getUTMZone();
            pt.utm_designator = arolib::CoordTransformer::GetInstance().getUTMDesignator();
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

bool KMLInDocument::stringToPointList(std::string coordinates, std::vector<Point> &pts){
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


bool KMLInDocument::getBranch(const boost::property_tree::ptree & base, const std::string &name, boost::property_tree::ptree &branch)
{
    try{
        auto ochild = base.get_child_optional(name);
        if(!ochild)
            return false;
        branch = *ochild;
        return true;
    }
    catch(...){
        return false;
    }

}

bool KMLInDocument::getBranch(const boost::property_tree::ptree & base, const std::vector<std::string> &tags, boost::property_tree::ptree &branch)
{
    branch = base;
    for(size_t i = 0 ; i < tags.size() ; ++i){
        if(!getBranch(branch, tags.at(i), branch)){
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Cannot open tag '" + std::to_string(i) + ": '" + tags.at(i) + "'");
            return false;
        }
    }
    return true;

}

bool KMLInDocument::getBranch(const std::string &name, boost::property_tree::ptree &branch)
{
    return getBranch(m_base.t, name, branch);
}

bool KMLInDocument::getBranch(const std::vector<std::string> &tags, boost::property_tree::ptree &branch)
{
    return getBranch(m_base.t, tags, branch);
}

bool KMLInDocument::getBranch(const KMLInDocument::ReadHandler &base, const std::string &name, KMLInDocument::ReadHandler &branch)
{
    return getBranch(base.t, name, branch.t);
}

bool KMLInDocument::getBranch(const KMLInDocument::ReadHandler &base, const std::vector<std::string> &tags, KMLInDocument::ReadHandler &branch)
{
    return getBranch(base.t, tags, branch.t);
}

bool KMLInDocument::getBranch(const std::string &name, KMLInDocument::ReadHandler &branch)
{
    return getBranch(m_base.t, name, branch.t);
}


bool KMLInDocument::getBranch(const std::vector<std::string> &tags, ReadHandler &branch)
{
    return getBranch(m_base.t, tags, branch.t);
}

bool KMLInDocument::readFromDescription(const Field &field, const std::string &description)
{
    bool ok = true;





//    ok_id = getValue(base.t, "id", field.id);
//    ok_n = getValue(base.t, "name", field.name);
//    getValue(base.t, "filename", field.filename);

//    if(!getBranch(base, "outer_boundary", branch))
//        return false;
//    if(!read(branch, field.outer_boundary))
//        return false;

//    if(getBranch(base, getTag(field.subfields), branch)){
//        if( !read(branch,field.subfields) )
//            return false;
//    }

//    return ( !m_strict || (ok_id && ok_n) );

    return ok;

}

bool KMLInDocument::readFromDescription(const Subfield &sf, const std::string &description)
{
    bool ok = true;
    return ok;

}

bool KMLInDocument::readFromDescription(const Linestring &ls, const std::string &description)
{
    bool ok = true;
    return ok;

}

bool KMLInDocument::readFromDescription(const FieldAccessPoint &pt, const std::string &description)
{
    bool ok = true;
    return ok;

}

bool KMLInDocument::readFromDescription(const ResourcePoint &pt, const std::string &description)
{
    bool ok = true;
    return ok;

}

bool KMLInDocument::readFromDescription(const HeadlandPoint &pt, const std::string &description)
{
    bool ok = true;
    return ok;

}

bool KMLInDocument::readFromDescription(const RoutePoint &pt, const std::string &description)
{
    bool ok = true;
    return ok;

}

bool KMLInDocument::readFromDescription(const Track &track, const std::string &description)
{
    bool ok = true;
    return ok;

}

bool KMLInDocument::readFromDescription(const Obstacle &obs, const std::string &description)
{
    bool ok = true;
    return ok;
}

}//end namespace arolib

