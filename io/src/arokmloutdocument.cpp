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
 
#include "arolib/io/arokmloutdocument.hpp"

namespace arolib {
namespace io {

const std::string AroKMLOutDocument::UseDefaultStyle = "<>";

const int AroKMLOutDocument::KmlStyle::WidthBoldBold = 5;
const int AroKMLOutDocument::KmlStyle::WidthBold = 3;
const int AroKMLOutDocument::KmlStyle::WidthNormal = 2;
const int AroKMLOutDocument::KmlStyle::WidthThin = 1;

AroKMLOutDocument::KmlStyle::KmlStyle(const std::string &_color, int _width, double _scale, const std::string &_iconRef, bool _fill):
    color(_color),
    width(_width),
    scale(_scale),
    iconRef(_iconRef),
    fill(_fill)
{

}

AroKMLOutDocument::AroKMLOutDocument(LogLevel logLevel):
    XMLOutDocument(logLevel)
{
    initDefaultStyles();
}

AroKMLOutDocument::~AroKMLOutDocument()
{
}

bool AroKMLOutDocument::setCoordinatesTypes(Point::ProjectionType in, Point::ProjectionType out)
{
    if(m_isDocOpen){
        m_logger.printError(__FUNCTION__, "Cannot set the coordinates' types when the document is already open");
        return false;
    }
    m_coordinatesType_in = in;
    m_coordinatesType_out = out;
    return true;
}

bool AroKMLOutDocument::add(const Point &pt, const std::string &name, const std::string &description, const std::string& styleUrl) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    openTag(KMLTag_placemark);

    add(name, KMLTag_name);

    if(!styleUrl.empty())
        add("#"+styleUrl, KMLTag_styleUrl);

    if(!description.empty())
        add(description, KMLTag_description);

    openTag(KMLTag_point);
    openTag(KMLTag_coordinates);

    tabs();
    try{
        if(m_coordinatesType_in == Point::UTM){
            Point pt_geo = pt;
            arolib::CoordTransformer::GetInstance().convert_to_geodetic(pt, pt_geo);
            *m_os <<pt_geo.x<<","<<pt_geo.y<<","<<pt_geo.z;
        }
        else
            *m_os <<pt.x<<","<<pt.y<<","<<pt.z;
    }
    catch(...){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Exception cought. point = " + pt.toString(10));
        ok = false;
    }
    *m_os << std::endl;

    closeTag();
    closeTag();
    closeTag();

    return ok;
}

bool AroKMLOutDocument::add(const std::vector<Point> &pts, const std::string &name, bool asLinestring, const std::string &description, const std::string& styleUrl) {

    if(!asLinestring){
        for(auto&pt : pts){
            if(!add(pt, name, description, styleUrl))
                return false;
        }
        return true;
    }

    bool ok = true;
    if(!isReadyToWrite())
        return false;

    if(pts.empty())
        return true;

    openTag(KMLTag_placemark);

    add(name, KMLTag_name);

    if(!styleUrl.empty())
        add("#"+styleUrl, KMLTag_styleUrl);

    if(!description.empty())
        add(description, KMLTag_description);

    openTag(KMLTag_linestring);
    openTag(KMLTag_coordinates);

    tabs();
    for(size_t i = 0 ; i < pts.size() ; ++i){
        auto& pt = pts.at(i);
        try{
            if(m_coordinatesType_in == Point::UTM){
                Point pt_geo = pt;
                arolib::CoordTransformer::GetInstance().convert_to_geodetic(pt, pt_geo);
                *m_os <<pt_geo.x<<","<<pt_geo.y<<","<<pt_geo.z<<" ";
            }
            else
                *m_os <<pt.x<<","<<pt.y<<","<<pt.z<<" ";
        }
        catch(...){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Exception cought. point = " + pt.toString(10));
            ok = false;
        }
    }
    *m_os << std::endl;

    closeTag();
    closeTag();
    closeTag();

    return ok;
}

bool AroKMLOutDocument::add(const Linestring &ls, const std::string &name, const std::string &description, const std::string& styleUrl) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    if(ls.points.empty())
        return true;

    openTag(KMLTag_placemark);

    add(name, KMLTag_name);

    if(!styleUrl.empty())
        add("#"+styleUrl, KMLTag_styleUrl);

    if(!description.empty())
        add(description, KMLTag_description);
    else
        add(getDescription(ls), KMLTag_description);

    openTag(KMLTag_linestring);
    openTag(KMLTag_coordinates);

    tabs();
    for(size_t i = 0 ; i < ls.points.size() ; ++i){
        auto& pt = ls.points.at(i);
        try{
            if(m_coordinatesType_in == Point::UTM){
                Point pt_geo = pt;
                arolib::CoordTransformer::GetInstance().convert_to_geodetic(pt, pt_geo);
                *m_os <<pt_geo.x<<","<<pt_geo.y<<","<<pt_geo.z<<" ";
            }
            else
                *m_os <<pt.x<<","<<pt.y<<","<<pt.z<<" ";
        }
        catch(...){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Exception cought. point = " + pt.toString(10));
            ok = false;
        }
    }
    *m_os << std::endl;

    closeTag();
    closeTag();
    closeTag();

    return ok;
}

bool AroKMLOutDocument::add(const std::vector<Linestring> &lss, const std::string &name_prefix, const std::string& styleUrl, bool useOnlyPrefix)
{
    for(size_t i = 0 ; i < lss.size() ; ++i){
        auto name = name_prefix;
        if(!useOnlyPrefix)
            name += ("_" + std::to_string(i));
        if(!add(lss.at(i), name , "", styleUrl))
            return false;
    }
    return true;
}

bool AroKMLOutDocument::add(const Polygon &poly, const std::string &name, const std::string &description, const std::string& styleUrl) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    if(poly.points.empty())
        return true;

    openTag(KMLTag_placemark);

    add(name, KMLTag_name);

    if(!styleUrl.empty())
        add("#"+styleUrl, KMLTag_styleUrl);

    if(!description.empty())
        add(description, KMLTag_description);

    openTag(KMLTag_polygon);
    openTag(KMLTag_polygonBoundary);
    openTag(KMLTag_linearring);
    openTag(KMLTag_coordinates);

    tabs();
    for(size_t i = 0 ; i < poly.points.size() ; ++i){
        auto& pt = poly.points.at(i);
        try{
            if(m_coordinatesType_in == Point::UTM){
                Point pt_geo = pt;
                arolib::CoordTransformer::GetInstance().convert_to_geodetic(pt, pt_geo);
                *m_os <<pt_geo.x<<","<<pt_geo.y<<","<<pt_geo.z<<" ";
            }
            else
                *m_os <<pt.x<<","<<pt.y<<","<<pt.z<<" ";
        }
        catch(...){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Exception cought. point = " + pt.toString(10));
            ok = false;
        }
    }
    *m_os << std::endl;

    closeTag();
    closeTag();
    closeTag();
    closeTag();
    closeTag();

    return ok;
}

bool AroKMLOutDocument::add(const FieldAccessPoint &pt, std::string tag, const std::string &styleUrl) {
    tag = getTag<FieldAccessPoint>(tag);
    if(tag.empty())
        tag = getTag(pt);
    return add(pt.point(), tag, getDescription(pt), styleUrl);
}

bool AroKMLOutDocument::add(const std::vector<FieldAccessPoint> &pts, std::string tag, const std::string &styleUrl) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    if(pts.empty())
        return true;
    tag = getTag<std::vector<FieldAccessPoint>>(tag);
    if(!tag.empty()){
        openTag(KMLTag_folder);
        ok &= add(tag, KMLTag_name);
    }
    for(auto &p : pts){
        ok &= add(p, UseDefaultTag, getStyleUrl(Geom_FieldAccessPoint, styleUrl));
        if (!ok)
            break;
    }
    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroKMLOutDocument::add(const ResourcePoint &pt, std::string tag, const std::string &styleUrl) {
    bool ok = true;

    tag = getTag<ResourcePoint>(tag);
    if(!tag.empty()){
        openTag(KMLTag_folder);
        ok &= add(tag, KMLTag_name);
    }
    else if(!pt.geometry.points.empty())
        return false;

    ok &= add(getDescription(pt), KMLTag_description);

    ok &= add(pt.point(), tag, "", getStyleUrl(Geom_ResourcePoint, styleUrl));
    ok &= add(pt.geometry, tag + "_geometry", "", getStyleUrl(Geom_ResourcePoint, styleUrl));

    if(!tag.empty())
        closeTag();//folder
    return ok;
}

bool AroKMLOutDocument::add(const std::vector<ResourcePoint> &pts, std::string tag, const std::string &styleUrl) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    if(pts.empty())
        return true;
    tag = getTag<std::vector<ResourcePoint>>(tag);
    if(!tag.empty()){
        openTag(KMLTag_folder);
        ok &= add(tag, KMLTag_name);
    }
    for(auto &p : pts){
        ok &= add(p, UseDefaultTag, styleUrl);
        if(!ok)
            break;
    }
    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroKMLOutDocument::add(const RoutePoint &pt, std::string tag, const std::string &styleUrl) {
    tag = getTag<RoutePoint>(tag);
    if(tag.empty())
        tag = getTag(pt);
    return add(pt.point(), tag, getDescription(pt), getStyleUrl(Geom_ResourcePoint, styleUrl));
}

bool AroKMLOutDocument::add(const std::vector<RoutePoint> &pts, bool asLinestring, std::string tag, const std::string &styleUrl) {
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    if(pts.empty())
        return true;
    tag = getTag<std::vector<RoutePoint>>(tag);

    if(asLinestring){
        if(tag.empty())
            tag = getTag(pts);
        return add( RoutePoint::toPoints(pts), tag, true, "", getStyleUrl(Geom_RoutePoint, styleUrl));
    }

    if(!tag.empty()){
        openTag(KMLTag_folder);
        ok &= add(tag, KMLTag_name);
    }
    for(auto &p : pts){
        ok &= add(p, UseDefaultTag, styleUrl);
        if(!ok)
            break;
    }
    if(!tag.empty())
        closeTag();
    return ok;
}

bool AroKMLOutDocument::add(const Track &track, std::string tag, const std::string &styleUrl_points, const std::string &styleUrl_boundary)
{

    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<Track>(tag);
    if(!tag.empty()){
        openTag(KMLTag_folder);
        ok &= add(tag, KMLTag_name);
    }

    GeometryType geomType_points = track.isHeadlandTrack() ? Geom_Headland_track : Geom_Infield_track;
    GeometryType geomType_boundary = track.isHeadlandTrack() ? Geom_Headland_track_boundary : Geom_Infield_track_boundary;

    ok &= add(getDescription(track), KMLTag_description);

    ok &= add(track.points, "points", true, "", getStyleUrl(geomType_points, styleUrl_points));
    ok &= add(track.boundary, "boundary", "", getStyleUrl(geomType_boundary, styleUrl_boundary));

    if(!tag.empty())
        closeTag();

    return ok;
}

bool AroKMLOutDocument::add(const std::vector<Track> &tracks, std::string tag, const std::string &styleUrl_points, const std::string &styleUrl_boundary)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    if(tracks.empty())
        return true;

    tag = getTag<std::vector<Track>>(tag);
    if(!tag.empty()){
        openTag(KMLTag_folder);
        ok &= add(tag, KMLTag_name);
    }

    for(auto &track : tracks){
        ok &= add(track, UseDefaultTag, styleUrl_points, styleUrl_boundary);
        if(!ok)
            break;
    }
    if(!tag.empty())
        closeTag();

    return ok;

}

bool AroKMLOutDocument::add(const Headlands &headlands, std::string tag, const std::string& styleUrl_complete, const std::string& styleUrl_partial)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<Headlands>(tag);
    if(!tag.empty()){
        openTag(KMLTag_folder);
        ok &= add(tag, KMLTag_name);
    }

    ok &= add(headlands.complete, UseDefaultTag, styleUrl_complete);

    if(!tag.empty())
        closeTag();

    return ok;

}

bool AroKMLOutDocument::add(const CompleteHeadland &hl, std::string tag, const std::string& styleUrl)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<CompleteHeadland>(tag);
    if(!tag.empty()){
        openTag(KMLTag_folder);
        ok &= add(tag, KMLTag_name);
    }

    ok &= add(getDescription(hl), KMLTag_description);
    ok &= add(hl.middle_track, "middle_track", "", getStyleUrl(Geom_Headland_midTrack, styleUrl));
    ok &= add(hl.tracks, UseDefaultTag, getStyleUrl(Geom_Headland_track, styleUrl), getStyleUrl(Geom_Headland_track_boundary, styleUrl));
    ok &= add(hl.boundaries.first, "boundary_out", "", getStyleUrl(Geom_Headland_boundary, styleUrl) );
    ok &= add(hl.boundaries.second, "boundary_in", "", getStyleUrl(Geom_Headland_boundary, styleUrl) );

    if(!tag.empty())
        closeTag();

    return ok;
}


bool AroKMLOutDocument::add(const Obstacle &obs, std::string tag, const std::string &styleUrl){
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<Obstacle>(tag);
    if(!tag.empty()){
        openTag(KMLTag_folder);
        ok &= add(tag, KMLTag_name);
    }

    ok &= add(getDescription(obs), KMLTag_description);
    ok &= add(obs.boundary, "boundary", "", getStyleUrl(Geom_Obstacle_boundary, styleUrl));

    if(!tag.empty())
        closeTag();

    return ok;
}

bool AroKMLOutDocument::add(const std::vector<Obstacle> &obstacles, std::string tag, const std::string &styleUrl)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    if(obstacles.empty())
        return true;

    tag = getTag<std::vector<Obstacle>>(tag);
    if(!tag.empty()){
        openTag(KMLTag_folder);
        ok &= add(tag, KMLTag_name);
    }

    for(auto &obs : obstacles){
        ok &= add(obs, UseDefaultTag, styleUrl);
        if(!ok)
            break;
    }
    if(!tag.empty())
        closeTag();

    return ok;
}


bool AroKMLOutDocument::add(const Subfield &sf, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<Subfield>(tag);
    if(!tag.empty()){
        openTag(KMLTag_folder);
        ok &= add(tag, KMLTag_name);
    }

    ok &= add(getDescription(sf), KMLTag_description);

    ok &= add(sf.boundary_outer, "outer_boundary", "", getDefaultStyleUrl(Geom_Subfield_outerBoundary));
    ok &= add(sf.boundary_inner, "inner_boundary", "", getDefaultStyleUrl(Geom_Subfield_innerBoundary));

    ok &= add(sf.access_points, UseDefaultTag, UseDefaultStyle);
    ok &= add(sf.resource_points, UseDefaultTag, UseDefaultStyle);

    if(!sf.reference_lines.empty()){
        openTag(KMLTag_folder);
        ok &= add("reference_lines", KMLTag_name);
        for(auto & rl : sf.reference_lines)
            ok &= add(rl, "reference_line", getDescription(rl), getDefaultStyleUrl(Geom_ReferenceLine));
        closeTag();
    }

    if(sf.reference_line_A.isValid() && sf.reference_line_B.isValid()){
        openTag(KMLTag_folder);
        ok &= add("reference_line_A_B", KMLTag_name);
        ok &= add(sf.reference_line_A, "reference_line_A");
        ok &= add(sf.reference_line_B, "reference_line_B");
        closeTag();
    }

    ok &= add(sf.headlands, UseDefaultTag, UseDefaultStyle, UseDefaultStyle);

    ok &= add(sf.obstacles, UseDefaultTag, UseDefaultStyle);

    ok &= add(sf.tracks, UseDefaultTag, UseDefaultStyle, UseDefaultStyle);

    if(!tag.empty())
        closeTag();//folder

    return ok;
}

bool AroKMLOutDocument::add(const std::vector<Subfield> &subfields, std::string tag)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    if(subfields.empty())
        return true;

    tag = getTag<std::vector<Subfield>>(tag);
    if(!tag.empty()){
        openTag(KMLTag_folder);
        ok &= add(tag, KMLTag_name);
    }

    for(auto &sf : subfields){
        ok &= add(sf, UseDefaultTag);
        if(!ok)
            break;
    }

    if(!tag.empty())
        closeTag();//folder

    return ok;
}

bool AroKMLOutDocument::add(const Field &field, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    tag = getTag<Field>(tag);
    if(!tag.empty()){
        openTag(KMLTag_folder);
        ok &= add(tag, KMLTag_name);
    }

    ok &= add(getDescription(field), KMLTag_description);
    ok &= add(field.outer_boundary, "outer_boundary", "", getDefaultStyleUrl(Geom_Field_outerBoundary));
    ok &= add(field.subfields, UseDefaultTag);

    if(!field.external_roads.empty()){
        openTag(KMLTag_folder);
        ok &= add("external_roads", KMLTag_name);
        if(!tag.empty())
        ok &= add(field.external_roads, "external_road", getDefaultStyleUrl(Geom_Path), true);
        closeTag();//folder
    }

    if(!tag.empty())
        closeTag();//folder

    return ok;
}

bool AroKMLOutDocument::addStyle(const std::string& styleUrl, const AroKMLOutDocument::KmlStyle &style)
{
    if(!AroOutDocument::isReadyToWrite())
        return false;
    if(styleUrl.empty())
        return false;

    openTag(KMLTag_style, "id='"+styleUrl+"'");

    if(!style.color.empty() || style.scale > 1e-6 || !style.iconRef.empty()){
        openTag(KMLTag_iconstyle);
        if(!style.color.empty() /*&& style.iconRef.empty()*/)
            add(style.color, KMLTag_color);
        if(style.scale > 1e-6)
            add(style.scale, KMLTag_scale);
        if(!style.iconRef.empty()){
            openTag(KMLTag_icon);
            add(style.iconRef, KMLTag_iconref);
            closeTag();
        }
        closeTag();

        openTag(KMLTag_labelstyle);
        add("00FFFFFF", KMLTag_color);
        add(0.5, KMLTag_labelstyle);
        closeTag();
    }

    if(!style.color.empty() || style.scale > 1e-6 || style.width > 0){
        openTag(KMLTag_linestyle);
        if(!style.color.empty())
            add(style.color, KMLTag_color);
        if(style.scale > 1e-6)
            add(style.scale, KMLTag_scale);
        if(style.width > 0)
            add(style.width, KMLTag_width);
        closeTag();
    }

    if(!style.color.empty() || style.scale > 1e-6 || style.width > 0){
        openTag(KMLTag_polystyle);
        if(!style.color.empty())
            add(style.color, KMLTag_color);
        if(style.scale > 1e-6)
            add(style.scale, KMLTag_scale);
        if(style.width > 0)
            add(style.width, KMLTag_width);
        add(style.fill, KMLTag_polyfill);
        closeTag();
    }

    closeTag();

    return true;

}

bool AroKMLOutDocument::addDefaultStyle(AroKMLOutDocument::GeometryType geomType)
{
    try{
        return addStyle( getDefaultStyleUrl(geomType), m_styles.at(geomType) );
    }
    catch(...){
        return false;
    }
}

bool AroKMLOutDocument::addDefaultStyles(const std::set<AroKMLOutDocument::GeometryType>& geomTypes)
{
    std::set<AroKMLOutDocument::GeometryType> geomTypes2;
    const std::set<AroKMLOutDocument::GeometryType> *pGeomTypes = &geomTypes;

    if(geomTypes.empty()){
        geomTypes2 = {Geom_Field_outerBoundary,
                      Geom_Subfield_outerBoundary,
                      Geom_Subfield_innerBoundary,
                      Geom_Obstacle_boundary,
                      Geom_ReferenceLine,
                      Geom_ResourcePoint,
                      Geom_FieldAccessPoint,
                      Geom_RoutePoint,
                      Geom_Infield_track,
                      Geom_Infield_track_boundary,
                      Geom_Headland_track,
                      Geom_Headland_track_boundary,
                      Geom_Headland_midTrack,
                      Geom_Headland_boundary,
                      Geom_Path};
        pGeomTypes = &geomTypes2;
    }

    for(auto t : *pGeomTypes){
        if(!addDefaultStyle(t))
            return false;
    }
    return true;

}

bool AroKMLOutDocument::saveField(const std::string &filename, const Field &field, Point::ProjectionType coordinatesType_in)
{
    AroKMLOutDocument doc;
    doc.setCoordinatesTypes(coordinatesType_in, Point::WGS);
    return doc.openFile(filename) &&
           doc.openDocument() &&
           doc.add( get_filename(filename, true), KMLTag_name ) &&
           doc.add(field, UseDefaultTag) &&
           doc.addDefaultStyles() &&
           doc.closeDocument() &&
           doc.closeFile();
}

bool AroKMLOutDocument::saveFields(const std::string &filename, const std::vector<Field> &fields, Point::ProjectionType coordinatesType_in)
{
    AroKMLOutDocument doc;
    doc.setCoordinatesTypes(coordinatesType_in, Point::WGS);
    return doc.openFile(filename) &&
           doc.openDocument() &&
           doc.add( get_filename(filename, true), KMLTag_name ) &&
           doc.add(fields, UseDefaultTag) &&
           doc.addDefaultStyles() &&
           doc.closeDocument() &&
           doc.closeFile();

}

bool AroKMLOutDocument::saveRoute(const std::string &filename, const Route& route, Point::ProjectionType coordinatesType_in)
{
    AroKMLOutDocument doc;
    doc.setCoordinatesTypes(coordinatesType_in, Point::WGS);
    return doc.openFile(filename) &&
           doc.openDocument() &&
        //    doc.add( get_filename(filename, true), KMLTag_name ) &&
        //    doc.add(fields, UseDefaultTag) &&
           doc.add(RoutePoint::toPoints(route.route_points), "route") &&
        //    doc.addDefaultStyles() &&
           doc.closeDocument() &&
           doc.closeFile();

}


std::string AroKMLOutDocument::getDefaultStyleUrl(AroKMLOutDocument::GeometryType geomType)
{
    if(geomType == Geom_Field_outerBoundary)
        return "Geom_Field_outerBoundary_defStyle";
    if(geomType == Geom_Subfield_outerBoundary)
        return "Geom_Subfield_outerBoundary_defStyle";
    if(geomType == Geom_Subfield_innerBoundary)
        return "Geom_Subfield_innerBoundary_defStyle";
    if(geomType == Geom_Obstacle_boundary)
        return "Geom_Obstacle_boundary_defStyle";
    if(geomType == Geom_ReferenceLine)
        return "Geom_ReferenceLine_defStyle";
    if(geomType == Geom_ResourcePoint)
        return "Geom_ResourcePoint_defStyle";
    if(geomType == Geom_FieldAccessPoint)
        return "Geom_FieldAccessPoint_defStyle";
    if(geomType == Geom_Infield_track)
        return "Geom_Infield_track_defStyle";
    if(geomType == Geom_Infield_track_boundary)
        return "Geom_Infield_track_boundary_defStyle";
    if(geomType == Geom_Headland_track)
        return "Geom_Headland_track_defStyle";
    if(geomType == Geom_Headland_track_boundary)
        return "Geom_Headland_track_boundary_defStyle";
    if(geomType == Geom_Headland_midTrack)
        return "Geom_Headland_midTrack_defStyle";
    if(geomType == Geom_Headland_boundary)
        return "Geom_Headland_boundary_defStyle";
    if(geomType == Geom_RoutePoint)
        return "Geom_RoutePoint_defStyle";
    if(geomType == Geom_Path)
        return "Geom_Path_defStyle";
    return "";
}

bool AroKMLOutDocument::openDoc()
{
    if(!AroOutDocument::isReadyToWrite())
        return false;

    *m_os << std::setprecision(12)
          << "<?xml version='1.0' encoding='UTF-8'?>\n";

    openTag("kml", "xmlns='http://www.opengis.net/kml/2.2'");

    return openTag(KMLTag_document);
}

void AroKMLOutDocument::initDefaultStyles()
{
    m_styles = { { Geom_Field_outerBoundary, KmlStyle("FF606060", KmlStyle::WidthBold, -1, "") },
                 { Geom_Subfield_outerBoundary, KmlStyle("FF000000", KmlStyle::WidthBold, -1, "") },
                 { Geom_Subfield_innerBoundary, KmlStyle("FF000000", KmlStyle::WidthBold, -1, "") },
                 { Geom_Obstacle_boundary, KmlStyle("FF0000FF", KmlStyle::WidthNormal, -1, "") },
                 { Geom_ReferenceLine, KmlStyle("FFFFFF00", KmlStyle::WidthNormal, -1, "") },
                 { Geom_ResourcePoint, KmlStyle("FFFF007F", -1, -1, "http://maps.google.com/mapfiles/kml/pal3/icon31.png") },
                 { Geom_FieldAccessPoint, KmlStyle("FF0080FF", -1, -1, "http://maps.google.com/mapfiles/kml/pal4/icon28.png") },
                 { Geom_Infield_track, KmlStyle("FFFF0000", KmlStyle::WidthNormal, -1, "") },
                 { Geom_Infield_track_boundary, KmlStyle("3FFF0000", KmlStyle::WidthThin, -1, "", true) },
                 { Geom_Headland_track, KmlStyle("FFFF0000", KmlStyle::WidthNormal, -1, "") },
                 { Geom_Headland_track_boundary, KmlStyle("3FFF0000", KmlStyle::WidthThin, -1, "", true) },
                 { Geom_Headland_midTrack, KmlStyle("FFFF6666", KmlStyle::WidthNormal, -1, "") },
                 { Geom_Headland_boundary, KmlStyle("FF909090", KmlStyle::WidthNormal, -1, "") },
                 { Geom_RoutePoint, KmlStyle("FF00FF00", KmlStyle::WidthNormal, -1, "http://maps.google.com/mapfiles/kml/pal4/icon57.png") },
                 { Geom_Path, KmlStyle("FF00FF00", KmlStyle::WidthNormal, -1, "") } };
}

std::string AroKMLOutDocument::getStyleUrl(AroKMLOutDocument::GeometryType geomType, const std::string &styleUrl){
    if(styleUrl != UseDefaultStyle)
        return styleUrl;
    return getDefaultStyleUrl(geomType);
}

std::string AroKMLOutDocument::getDescription(const Field &field)
{
    return    "id=" + std::to_string(field.id) + "\n"
            + "name=" + field.name;
}

std::string AroKMLOutDocument::getDescription(const Subfield &sf)
{
    return    "id=" + std::to_string(sf.id);
}

std::string AroKMLOutDocument::getDescription(const Linestring &ls)
{
    return    "id=" + std::to_string(ls.id);
}

std::string AroKMLOutDocument::getDescription(const FieldAccessPoint &pt)
{
    return    "id=" + std::to_string(pt.id) + "\n"
            + getTag(pt.accessType) + "=" + std::to_string(pt.accessType);
}

std::string AroKMLOutDocument::getDescription(const ResourcePoint &pt)
{
    std::string resourceTypes;
    for(auto& t : pt.resourceTypes){
        resourceTypes += std::to_string(t) + ",";
    }
    if(!resourceTypes.empty())
        resourceTypes.pop_back();

    return    "id=" + std::to_string(pt.id) + "\n"
            + getTag(pt.resourceTypes) + "=" + resourceTypes + "\n"
            + "defaultUnloadingTime=" + double2string(pt.defaultUnloadingTime) + "\n"
            + "defaultUnloadingTimePerKg=" + double2string(pt.defaultUnloadingTimePerKg);

}

std::string AroKMLOutDocument::getDescription(const HeadlandPoint &pt)
{
    return    "track_id=" + std::to_string(pt.track_id);
}

std::string AroKMLOutDocument::getDescription(const RoutePoint &pt)
{
    return    getTag(pt.type) + "=" + std::to_string(pt.type) + "\n"
            + "time_stamp=" + double2string(pt.time_stamp) + "\n"
            + "track_id=" + std::to_string(pt.track_id) + "\n"
            + "track_idx=" + std::to_string(pt.track_idx) + "\n"
            + "bunker_mass=" + double2string(pt.bunker_mass) + "\n"
            + "bunker_volume=" + double2string(pt.bunker_volume) + "\n"
            + "harvested_mass=" + double2string(pt.harvested_mass) + "\n"
            + "harvested_volume=" + double2string(pt.harvested_volume);
    //todo add machineRelations

}

std::string AroKMLOutDocument::getDescription(const Track &track)
{
    return    "id=" + std::to_string(track.id) + "\n"
            + "width=" + double2string(track.width) + "\n"
            + getTag(track.type) + "=" + std::to_string(track.type);

}

std::string AroKMLOutDocument::getDescription(const Obstacle &obs)
{
    return    getTag(obs.type) + "=" + std::to_string(obs.type) + "\n"
            + "type_description=" + obs.type_description;
}

std::string AroKMLOutDocument::getDescription(const CompleteHeadland &hl)
{
    return  "width=" + double2string(hl.headlandWidth);
}

}
}//end namespace arolib

