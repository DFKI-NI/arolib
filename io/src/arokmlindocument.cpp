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
 
#include "arolib/io/arokmlindocument.hpp"

namespace arolib {
namespace io {

AroKMLInDocument::KmlTag::KmlTag(const std::string &_kml_tag, const std::string &_name):
    kml_tag(_kml_tag),
    name(_name)
{

}

AroKMLInDocument::AroKMLInDocument(LogLevel logLevel):
    XMLInDocument(logLevel)
{
}

AroKMLInDocument::~AroKMLInDocument()
{
}

bool AroKMLInDocument::read(const ReadHandler & base, Point &pt, std::string *description)
{
    if(!m_isDocOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    ReadHandler branch;

    if(getBranch(base, KMLTag_point, branch)){
        if(!getBranch(branch, KMLTag_coordinates, branch))
            return false;

        if( !stringToPoint(RHTree(branch).get_value<std::string>() , pt ) )
            return false;
    }

    if(description)
        getValue(RHTree(base), KMLTag_description, *description);

    return true;
}

bool AroKMLInDocument::read(const ReadHandler & base, std::vector<Point> &pts, std::string *description)
{
    if(!m_isDocOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    ReadHandler branch;

    if(getBranch(base, KMLTag_linestring, branch)){
        if(!getBranch(branch, KMLTag_coordinates, branch))
            return false;

        if( !stringToPointList(RHTree(branch).get_value<std::string>() , pts ) )
            return false;
    }

    if(description)
        getValue(RHTree(base), KMLTag_description, *description);

    return true;
}

bool AroKMLInDocument::read(const ReadHandler & base, Linestring &ls, std::string *description){
    if(!m_isDocOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ls = Linestring();

        if(!readFromDescription(base, ls))
            return false;


        ReadHandler branch;

        if(getBranch(base, KMLTag_linestring, branch)){
            if(!getBranch(branch, KMLTag_coordinates, branch))
                return false;

            if( !stringToPointList(RHTree(branch).get_value<std::string>() , ls.points ) )
                return false;
        }

        if(description)
            getValue(RHTree(base), KMLTag_description, *description);

        return true;
    }
    catch(std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("Linestring: ") + e.what());
        return false;
    }
}

bool AroKMLInDocument::read(const ReadHandler & base, Polygon &poly, std::string *description){

    if(!m_isDocOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        poly.points.clear();

        ReadHandler branch;

//        if(!readFromDescription(base, poly))
//            return false;

        if(getBranch(base, KMLTag_polygon, branch)){
            if(!getBranch(branch, {KMLTag_polygonBoundary, KMLTag_linearring, KMLTag_coordinates}, branch))
                return false;

            if( !stringToPointList(RHTree(branch).get_value<std::string>() , poly.points ) )
                return false;

            geometry::correct_polygon(poly);
        }

        if(description)
            getValue(RHTree(base), KMLTag_description, *description);

        return true;
    }
    catch(std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("Polygon: ") + e.what());
        return false;
    }
}

bool AroKMLInDocument::read( const ReadHandler & base, FieldAccessPoint &pt){

    pt = FieldAccessPoint();
    if(!m_isDocOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        if(!readFromDescription(base, pt))
            return false;

        if(!read(base, pt.point()))
            return false;

        return true;
    }
    catch(std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("FieldAccessPoint: ") + e.what());
        return false;
    }
}

bool AroKMLInDocument::read( const ReadHandler & base, std::vector<FieldAccessPoint> &pts){
    return readMultiple(base, pts, KmlTag( FieldAccessPoint() ), true);
}

bool AroKMLInDocument::read( const ReadHandler & base, ResourcePoint &pt){

    pt = ResourcePoint();
    if(!m_isDocOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        bool ok_p = false;
        bool ok_g = false;

        if(!readFromDescription(base, pt))
            return false;

        ok_p = read(pt.point(), base, KmlTag( pt.point(), getTag<ResourcePoint>() ));
        ok_g = read(pt.geometry, base, KmlTag( pt.geometry, getTag<ResourcePoint>() + "_geometry" ));

        if(!ok_p && !ok_g)
            return false;

        return true;
    }
    catch(std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("ResourcePoint: ") + e.what());
        return false;
    }
}

bool AroKMLInDocument::read( const ReadHandler & base, std::vector<ResourcePoint> &pts){
    return readMultiple(base, pts, KmlTag( ResourcePoint() ), true);
}

bool AroKMLInDocument::read( const ReadHandler & base, RoutePoint &pt){
    pt = RoutePoint();
    if(!m_isDocOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        if(!readFromDescription(base, pt))
            return false;

        if(!read(base, pt.point()))
            return false;

        return true;
    }
    catch(std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("RoutePoint: ") + e.what());
        return false;
    }
}

bool AroKMLInDocument::read( const ReadHandler & base, std::vector<RoutePoint> &pts){
    return readMultiple(base, pts, KmlTag( RoutePoint() ), true);
}

bool AroKMLInDocument::read( const ReadHandler & base, Track &track){
    track = Track();

    if(!m_isDocOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        if(!readFromDescription(base, track))
            return false;

        ReadHandler branch;
        bool ok_pts = false;

        if(getKmlBranch(base, KmlTag( track.points, "points" ), branch)){
            if(!read(branch, track.points))
                return false;
            ok_pts = true;
        }

        if(getKmlBranch(base, KmlTag( track.boundary, "boundary" ), branch)){
            if(!read(branch, track.boundary))
                return false;
        }

        if(!ok_pts && m_strict)
            return false;

        return true;
    }
    catch(std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("Track: ") + e.what());
        return false;
    }
}

bool AroKMLInDocument::read( const ReadHandler & base, std::vector<Track> &tracks){
    return readMultiple(base, tracks, KmlTag( Track() ), true);
}

bool AroKMLInDocument::read(const AroKMLInDocument::ReadHandler &base, Headlands &headlands)
{
    headlands = Headlands();
    if(!m_isDocOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ReadHandler branch;

//        if(!readFromDescription(base, hl))
//            return false;

        if(getKmlBranch(base, KmlTag( headlands.complete ), branch)){
            if(!read(branch, headlands.complete))
                return false;
        }

        if(getKmlBranch(base, KmlTag( headlands.partial ), branch)){
            if(!read(branch, headlands.partial))
                return false;
        }

        return true;
    }
    catch(std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("Headlands: ") + e.what());
        return false;
    }

}

bool AroKMLInDocument::read(const AroKMLInDocument::ReadHandler &base, CompleteHeadland &hl)
{
    hl = CompleteHeadland();
    if(!m_isDocOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ReadHandler branch;
        bool ok_d = false;
        bool ok_t = false;
        bool ok_mt = false;

        ok_d = readFromDescription(base, hl);

        if(getKmlBranch(base, KmlTag( hl.middle_track, "middle_track" ), branch)){
            if(!read(branch, hl.middle_track))
                return false;
            ok_mt = true;
        }

        if(getKmlBranch(base, KmlTag( hl.tracks ), branch)){
            if(!read(branch, hl.tracks))
                return false;
            ok_t = true;
        }

        if(getKmlBranch(base, KmlTag( hl.boundaries.first, "boundary_out" ), branch)){
            if(!read(branch, hl.boundaries.first))
                return false;
        }
        if(getKmlBranch(base, KmlTag( hl.boundaries.second, "boundary_in" ), branch)){
            if(!read(branch, hl.boundaries.second))
                return false;
        }

        if(!ok_d && (ok_mt || ok_t) && m_strict)
            return false;//a headland exists but no description was given

        return true;
    }
    catch(std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("CompleteHeadland: ") + e.what());
        return false;
    }

}

bool AroKMLInDocument::read(const AroKMLInDocument::ReadHandler &base, PartialHeadland &hl)
{
    hl = PartialHeadland();
    if(!m_isDocOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ReadHandler branch;
        bool ok_ob = false;

        if(!readFromDescription(base, hl))
            return false;

        if(getKmlBranch(base, KmlTag( hl.boundary, "boundary" ), branch)){
            if(!read(branch, hl.boundary))
                return false;
            ok_ob = true;
        }

        if(getKmlBranch(base, KmlTag( hl.tracks ), branch)){
            if(!read(branch, hl.tracks))
                return false;
        }

        return ( !m_strict || (ok_ob) );
    }
    catch(std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("PartialHeadland: ") + e.what());
        return false;
    }

}

bool AroKMLInDocument::read(const AroKMLInDocument::ReadHandler &base, std::vector<PartialHeadland> &headlands)
{
    return readMultiple(base, headlands, KmlTag( PartialHeadland() ), true);
}

bool AroKMLInDocument::read(const AroKMLInDocument::ReadHandler &base, Obstacle &obs)
{
    obs = Obstacle();
    if(!m_isDocOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ReadHandler branch;

        if(!readFromDescription(base, obs))
            return false;

        if(getKmlBranch(base, KmlTag( obs.boundary, "boundary" ), branch)){
            if(!read(branch, obs.boundary))
                return false;
        }

        return true;
    }
    catch(std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("Obstacle: ") + e.what());
        return false;
    }

}

bool AroKMLInDocument::read(const AroKMLInDocument::ReadHandler &base, std::vector<Obstacle> &obstacles)
{
    return readMultiple(base, obstacles, KmlTag( Obstacle() ), true);
}

bool AroKMLInDocument::read( const ReadHandler & base, Subfield &sf){

    sf = Subfield();
    if(!m_isDocOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        bool ok_ob = false;
        ReadHandler branch;

        if(!readFromDescription(base, sf))
            return false;

        if(getKmlBranch(base, KmlTag( sf.boundary_outer, "outer_boundary" ), branch)){
            if(!read(branch, sf.boundary_outer))
                return false;
            ok_ob = true;
        }
        if(getKmlBranch(base, KmlTag( sf.boundary_inner, "inner_boundary" ), branch)){
            if(!read(branch, sf.boundary_inner))
                return false;
        }

        if(getKmlBranch(base, KmlTag( sf.access_points ), branch)){
            if(!read(branch, sf.access_points))
                return false;
        }
        if(getKmlBranch(base, KmlTag( sf.resource_points ), branch)){
            if(!read(branch, sf.resource_points))
                return false;
        }

        if(getKmlBranch(base, KmlTag(KMLTag_folder, "reference_lines"), branch)){
            if(!readMultiple(branch, sf.reference_lines, KmlTag( Linestring(), "reference_line" ), m_strict))
                return false;
        }

        if(getKmlBranch(base, KmlTag(KMLTag_folder, "reference_line_A_B"), branch)){
            if(!read(sf.reference_line_A, branch, KmlTag( sf.reference_line_A, "reference_line_A"))){
                sf.reference_line_A.setInvalid();
                //return false;
            }
            if(!read(sf.reference_line_B, branch, KmlTag( sf.reference_line_B, "reference_line_B"))){
                sf.reference_line_B.setInvalid();
                //return false;
            }
        }


        if(getKmlBranch(base, KmlTag( sf.obstacles ), branch)){
            if(!read(branch, sf.obstacles))
                return false;
        }

        if(getKmlBranch(base, KmlTag( sf.headlands ), branch)){
            if(!read(branch, sf.headlands))
                return false;
        }

        if(getKmlBranch(base, KmlTag( sf.tracks ), branch)){
            if(!read(branch, sf.tracks))
                return false;
        }

        return ( !m_strict || (ok_ob) );
    }
    catch(std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("Subfield: ") + e.what());
        return false;
    }

}

bool AroKMLInDocument::read( const ReadHandler & base, std::vector<Subfield> &sfs){
    return readMultiple(base, sfs, KmlTag( Subfield() ), true);
}

bool AroKMLInDocument::read( const ReadHandler & base, Field &field){
    if(!m_isDocOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        ReadHandler branch;
        field.clear();

        if(!readFromDescription(base, field))
            return false;

        if( !read(field.outer_boundary, base, KmlTag( field.outer_boundary, "outer_boundary" )) )
            return false;

        if(getKmlBranch(base, KmlTag( field.subfields ), branch)){
            if( !read(branch, field.subfields) )
                return false;
        }
        else{
            field.subfields.push_back(Subfield());
            field.subfields.back().boundary_outer = field.outer_boundary;
        }

        if(getKmlBranch(base, KmlTag( field.external_roads, "external_roads" ), branch)){
            if( !readMultiple(branch, field.external_roads, KmlTag( Linestring(), "" )) )
                return false;
        }

        return true;
    }
    catch(std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("Field: ") + e.what());
        return false;
    }
}


bool AroKMLInDocument::readField(const std::string &filename, Field &field, Point::ProjectionType coordinatesType_out, const std::vector<std::string> &parentTags, LogLevel logLevel)
{
    AroKMLInDocument doc(logLevel);

    doc.setCoordinatesType(coordinatesType_out);

    field.clear();

    if( !doc.openFile(filename) )
        return false;

    bool ok = doc.read( field, KmlTag( field ), parentTags );
    if(ok)
        field.filename = filename;

    //required
    return ok &&
           doc.closeFile();

}

bool AroKMLInDocument::readFields(const std::string &filename, std::vector<Field> &fields, Point::ProjectionType coordinatesType_out, const std::vector<std::string> &parentTags, LogLevel logLevel)
{
    AroKMLInDocument doc(logLevel);

    doc.setCoordinatesType(coordinatesType_out);

    fields.clear();

    if( !doc.openFile(filename) || !doc.openDocument() )
        return false;

    bool ok = doc.readMultiple( fields, KmlTag( Field() ), parentTags, false );
    if(ok){
        for(auto& field : fields)
            field.filename = filename;
    }

    //required
    return ok
           && doc.closeDocument()
           && doc.closeFile();

}


bool AroKMLInDocument::strict() const
{
    return m_strict;
}

void AroKMLInDocument::setStrict(bool strict)
{
    m_strict = strict;
}

bool AroKMLInDocument::openDoc()
{
    if(!AroInDocument::isReadyToRead())
        return false;

    m_is->seekg(0, m_is->beg);

    try{
        read_xml(*m_is, RHTree(m_base));

        if(!getBranchHandler(m_base, {"kml", m_docTag}, m_base)){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Cannot open document tag.");
            return false;
        }

        return true;
    }
    catch (std::exception& e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, std::string("Error reading input stream/file: '") + e.what() + "'");
        return false;
    }
}

bool AroKMLInDocument::stringToPoint(std::string coordinates, Point &pt){
    try{
        std::vector<std::string> xyz;
        boost::split(xyz, coordinates, boost::is_any_of(","));

        Point pt_in;
        if (xyz.size() == 2)
            pt_in = Point(string2double(xyz[0]), string2double(xyz[1]), 0);
        else if (xyz.size() == 3)
            pt_in = Point(string2double(xyz[0]), string2double(xyz[1]), string2double(xyz[2]));
        else{
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Cannot parse/split the point '" + coordinates + "'");
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
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Cannot parse/split the point '" + coordinates + "': " + e.what());
        return false;
    }
}

bool AroKMLInDocument::stringToPointList(std::string coordinates, std::vector<Point> &pts){
    boost::trim(coordinates);
    std::vector<std::string> strs;
    boost::split(strs, coordinates, boost::is_any_of(" "));

    pts.clear();
    for(unsigned int i = 0; i < strs.size(); i++){
        if(strs.at(i).empty())
            continue;
        Point pt;
        if( !stringToPoint(strs.at(i), pt) ){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error reading point " + std::to_string(i));
            return false;
        }
        pts.push_back(pt);
    }
    return true;
}


bool AroKMLInDocument::getKmlBranch(const boost::property_tree::ptree & base, const KmlTag &kmlTag, boost::property_tree::ptree &branch)
{
    try{
        BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, base){
            if(v.first == kmlTag.kml_tag){
                if(!kmlTag.name.empty()){
                    std::string name;
                    if(!getValue(v.second, KMLTag_name, name) || name != kmlTag.name)
                        continue;
                }
                branch = v.second;
                return true;
            }
        }

        return false;
    }
    catch(...){
        return false;
    }

}

bool AroKMLInDocument::getKmlBranch(const boost::property_tree::ptree & base, const std::vector<KmlTag> &kmlTags, boost::property_tree::ptree &branch)
{
    branch = base;
    for(size_t i = 0 ; i < kmlTags.size() ; ++i){
        if(!getKmlBranch(branch, kmlTags.at(i), branch)){
            logger().printOut(LogLevel::WARNING, __FUNCTION__, "Cannot open tag '" + std::to_string(i) + ": '" + kmlTags.at(i).kml_tag + "' - '" + kmlTags.at(i).name + "'");
            return false;
        }
    }
    return true;

}

bool AroKMLInDocument::getKmlBranch(const KmlTag &kmlTag, boost::property_tree::ptree &branch)
{
    return getKmlBranch(RHTree(m_base), kmlTag, branch);
}

bool AroKMLInDocument::getKmlBranch(const std::vector<KmlTag> &kmlTags, boost::property_tree::ptree &branch)
{
    return getKmlBranch(RHTree(m_base), kmlTags, branch);
}

bool AroKMLInDocument::getKmlBranch(const AroKMLInDocument::ReadHandler &base, const KmlTag &kmlTag, AroKMLInDocument::ReadHandler &branch)
{
    return getKmlBranch(RHTree(base), kmlTag, RHTree(branch));
}

bool AroKMLInDocument::getKmlBranch(const AroKMLInDocument::ReadHandler &base, const std::vector<KmlTag> &kmlTags, AroKMLInDocument::ReadHandler &branch)
{
    return getKmlBranch(RHTree(base), kmlTags, RHTree(branch));
}

bool AroKMLInDocument::getKmlBranch(const KmlTag &kmlTag, AroKMLInDocument::ReadHandler &branch)
{
    return getKmlBranch(RHTree(m_base), kmlTag, RHTree(branch));
}


bool AroKMLInDocument::getKmlBranch(const std::vector<KmlTag> &kmlTags, ReadHandler &branch)
{
    return getKmlBranch(RHTree(m_base), kmlTags, RHTree(branch));
}

bool AroKMLInDocument::readFromDescription(Field &field, const std::string &description, bool strict)
{
    bool ok_id = false;
    bool ok_n = false;

    auto items = getDescriptionItems(description);
    for(auto & item : items){
        auto& key = item.first;
        auto& value = item.second;
        try{
            if(key == "id"){
                field.id = std::stoi(value);
                ok_id = true;
            }
            else if(key == "name"){
                field.name = value;
                ok_n = true;
            }
            else if(key == "filename"){
                field.filename = value;
            }
        }
        catch(...){

        }
    }

    return ( !strict || (ok_id && ok_n) );
}

bool AroKMLInDocument::readFromDescription(Subfield &sf, const std::string &description, bool strict)
{
    auto items = getDescriptionItems(description);
    for(auto & item : items){
        auto& key = item.first;
        auto& value = item.second;
        try{
            if(key == "id")
                sf.id = std::stoi(value);
        }
        catch(...){

        }
    }

    return true;
}

bool AroKMLInDocument::readFromDescription(Linestring &ls, const std::string &description, bool strict)
{
    bool ok_id = false;
    auto items = getDescriptionItems(description);
    for(auto & item : items){
        auto& key = item.first;
        auto& value = item.second;
        try{
            if(key == "id"){
                ls.id = std::stoi(value);
                ok_id = true;
            }
        }
        catch(...){

        }
    }
    return ( !strict || (ok_id) );

}

bool AroKMLInDocument::readFromDescription(FieldAccessPoint &pt, const std::string &description, bool strict)
{
    bool ok_id = false;

    auto items = getDescriptionItems(description);
    for(auto & item : items){
        auto& key = item.first;
        auto& value = item.second;
        try{
            if(key == "id"){
                pt.id = std::stoi(value);
                ok_id = true;
            }
            else if(key == getTag(pt.accessType)){
                pt.accessType = FieldAccessPoint::intToAccessPointType( std::stoi(value) );
            }
        }
        catch(...){

        }
    }

    return ( !strict || (ok_id) );
}

bool AroKMLInDocument::readFromDescription(ResourcePoint &pt, const std::string &description, bool strict)
{
    bool ok_id = false;
    bool ok_ut = false;


    auto items = getDescriptionItems(description);
    for(auto & item : items){
        auto& key = item.first;
        auto& value = item.second;
        try{
            if(key == "id"){
                pt.id = std::stoi(value);
                ok_id = true;
            }
            else if(key == "defaultUnloadingTime"){
                pt.defaultUnloadingTime = string2double(value);
            }
            else if(key == "defaultUnloadingTimePerKg"){
                pt.defaultUnloadingTimePerKg = string2double(value);
            }
            else if(key == getTag(pt.resourceTypes)){
                pt.resourceTypes.clear();
                std::vector<std::string> types;
                boost::split(types, value, boost::is_any_of(","));
                for(auto & type : types){
                    boost::trim(type);
                    try{
                        pt.resourceTypes.insert( ResourcePoint::intToResourceType( std::stoi(type) ) );
                    }
                    catch(...){

                    }
                }
                ok_ut = !pt.resourceTypes.empty();
            }
        }
        catch(...){

        }
    }

    return ( !strict || (ok_id) );
}

bool AroKMLInDocument::readFromDescription(RoutePoint &pt, const std::string &description, bool strict)
{
    bool ok_ts = false;
    bool ok_bm = false;
    bool ok_bv = false;
    bool ok_hm = false;
    bool ok_hv = false;
    bool ok_ti = false;
    bool ok_t = false;

    auto items = getDescriptionItems(description);
    for(auto & item : items){
        auto& key = item.first;
        auto& value = item.second;
        try{
            if(key == getTag(pt.type)){
                pt.type = RoutePoint::intToRoutePointType( std::stoi(value) );
                ok_t = true;
            }
            else if(key == "time_stamp"){
                pt.time_stamp = string2double(value);
                ok_ts = true;
            }
            else if(key == "bunker_mass"){
                pt.bunker_mass = string2double(value);
                ok_bm = true;
            }
            else if(key == "bunker_volume"){
                pt.bunker_volume = string2double(value);
                ok_bv = true;
            }
            else if(key == "worked_mass"){
                pt.worked_mass = string2double(value);
                ok_hm = true;
            }
            else if(key == "worked_volume"){
                pt.worked_volume = string2double(value);
                ok_hv = true;
            }
            else if(key == "track_id"){
                pt.track_id = std::stoi(value);
                ok_ti = true;
            }
        }
        catch(...){

        }
    }

    return ( !strict || (ok_bm && ok_bv && ok_hm && ok_hv && ok_ts && ok_ti && ok_t) );
}

bool AroKMLInDocument::readFromDescription(Track &track, const std::string &description, bool strict)
{
    bool ok_id = false;

    auto items = getDescriptionItems(description);
    for(auto & item : items){
        auto& key = item.first;
        auto& value = item.second;
        try{
            if(key == "id"){
                track.id = std::stoi(value);
                ok_id = true;
            }
            else if(key == getTag(track.type)){
                track.type = Track::intToTrackType( std::stoi(value) );
            }
            else if(key == "width"){
                track.width = string2double(value);
            }
        }
        catch(...){

        }
    }

    return ( !strict || (ok_id) );
}

bool AroKMLInDocument::readFromDescription(Obstacle &obs, const std::string &description, bool strict)
{
    auto items = getDescriptionItems(description);
    for(auto & item : items){
        auto& key = item.first;
        auto& value = item.second;
        try{
            if(key == getTag(obs.type)){
                obs.type = Obstacle::intToObstacleType( std::stoi(value) );
            }
            else if(key == "type_description"){
                obs.type_description = value;
            }
        }
        catch(...){

        }
    }
    return true;
}

bool AroKMLInDocument::readFromDescription(CompleteHeadland &hl, const std::string &description, bool strict)
{
    auto items = getDescriptionItems(description);
    for(auto & item : items){
        auto& key = item.first;
        auto& value = item.second;
        try{
            if(key == "width"){
                hl.headlandWidth = string2double( value );
            }
        }
        catch(...){
        }
    }
    return true;
}

bool AroKMLInDocument::readFromDescription(PartialHeadland &hl, const std::string &description, bool strict)
{
    bool ok_id = false;
    auto connectingHeadlandId1 = hl.connectingHeadlandIds.first;
    auto connectingHeadlandId2 = hl.connectingHeadlandIds.second;
    auto items = getDescriptionItems(description);
    for(auto & item : items){
        auto& key = item.first;
        auto& value = item.second;
        try{
            if(key == "id"){
                hl.id = std::stoi(value);
                ok_id = true;
            }
            else if(key == "connectingHeadlandId1"){
                connectingHeadlandId1 = std::stoi(value);
            }
            else if(key == "connectingHeadlandId2"){
                connectingHeadlandId2 = std::stoi(value);
            }
        }
        catch(...){
        }
    }
    hl.connectingHeadlandIds = std::make_pair(connectingHeadlandId1, connectingHeadlandId2);
    return ( !strict || (ok_id) );
}

std::vector<std::pair<std::string, std::string> > AroKMLInDocument::getDescriptionItems(const std::string& description)
{
    std::vector<std::pair<std::string, std::string> > ret;

    std::string cNextItem = "\n";
    if( description.find(';') == 0 )
        cNextItem = ";";

    std::vector<std::string> lines;
    boost::split(lines, description, boost::is_any_of(cNextItem));

    for(auto& line : lines){
        auto indEq = line.find('=');
        if(indEq == std::string::npos)
            continue;
        std::string left = line.substr(0, indEq);
        std::string right = line.substr(indEq+1);

        boost::trim(left);
        boost::trim(right);

        for(size_t i = 0 ; i < left.size() ; ++i){
            if( left.at(i) < 33 ){//escape character or space
                left.erase(i, 1);
                --i;
            }
        }

        if(left.empty())
            continue;

        ret.emplace_back( std::make_pair(left, right) );
    }

    return ret;
}

}
}//end namespace arolib

