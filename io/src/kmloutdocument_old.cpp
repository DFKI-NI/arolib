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
 
#include "arolib/io/kmloutdocument_old.hpp"

namespace arolib {

const std::string KMLOutDocumentOld::DefaultStyle = "<>";

const int KMLOutDocumentOld::KmlStyle::WidthBoldBold = 5;
const int KMLOutDocumentOld::KmlStyle::WidthBold = 3;
const int KMLOutDocumentOld::KmlStyle::WidthNormal = 2;
const int KMLOutDocumentOld::KmlStyle::WidthThin = 1;

KMLOutDocumentOld::KmlStyle::KmlStyle(const std::string &_color, int _width, double _scale, const std::string &_iconRef, bool _fill):
    color(_color),
    width(_width),
    scale(_scale),
    iconRef(_iconRef),
    fill(_fill)
{

}

KMLOutDocumentOld::KMLOutDocumentOld(LogLevel logLevel):
    AroOutDocument(logLevel)
{
    m_logger = Logger(logLevel, __FUNCTION__);

    initDefaultStyles();
}

KMLOutDocumentOld::~KMLOutDocumentOld()
{
    if(m_isDocOpen)
        closeDoc();
}

bool KMLOutDocumentOld::setCoordinatesTypes(Point::ProjectionType in, Point::ProjectionType out)
{
    if(m_isDocOpen){
        m_logger.printError(__FUNCTION__, "Cannot set the coordinates' types when the document is already open");
        return false;
    }
    m_coordinatesType_in = in;
    m_coordinatesType_out = out;
    return true;
}

bool KMLOutDocumentOld::openTag(std::string tag, const std::string &extra){
    if(!isReadyToWrite())
        return false;
    m_openTags.emplace_back(tag);
    if(tag.empty())
        return true;
    tabs(m_nTabs++);
    if(extra.empty())
        *m_os << "<" << tag << ">\n";
    else
        *m_os << "<" << tag << " " << extra << ">\n";
    return true;
}

bool KMLOutDocumentOld::closeTag(int numTags){
    if(!isReadyToWrite())
        return false;
    if(numTags < 0 || numTags >= m_openTags.size()){
        while (!m_openTags.empty()){
            if(m_openTags.back().empty()){
                m_openTags.pop_back();
                continue;
            }
            tabs(--m_nTabs);
            *m_os << "</" << m_openTags.back() << ">\n";
            m_openTags.pop_back();
        }
        return true;
    }

    for(; numTags > 0; --numTags){
        if(m_openTags.back().empty()){
            m_openTags.pop_back();
            continue;
        }
        tabs(--m_nTabs);
        *m_os << "</" << m_openTags.back() << ">\n";
        m_openTags.pop_back();
    }
    return true;
}

bool KMLOutDocumentOld::add(const char *value, std::string tag)
{
    return add(std::string(value), tag);
}

bool KMLOutDocumentOld::add(const std::string &value, std::string tag){
    if(!isReadyToWrite())
        return false;
    if(value.empty())
        return true;
    tabs();
    *m_os << "<" << tag << ">" << value << "</" << tag << ">\n";
    return true;
}

bool KMLOutDocumentOld::add(const Point &pt, const std::string &name, const std::string &description, const std::string& styleUrl) {
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

bool KMLOutDocumentOld::add(const std::vector<Point> &pts, const std::string &name, bool asLinestring, const std::string &description, const std::string& styleUrl) {

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

bool KMLOutDocumentOld::add(const Linestring &ls, const std::string &name, const std::string &description, const std::string& styleUrl) {
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

bool KMLOutDocumentOld::add(const std::vector<Linestring> &lss, const std::string &name_prefix, const std::string& styleUrl)
{
    for(size_t i = 0 ; i < lss.size() ; ++i){
        if(!add(lss.at(i), name_prefix + "_" + std::to_string(i), "", styleUrl))
            return false;
    }
    return true;
}

bool KMLOutDocumentOld::add(const Polygon &poly, const std::string &name, const std::string &description, const std::string& styleUrl) {
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

bool KMLOutDocumentOld::add(const FieldAccessPoint &pt, const std::string& styleUrl) {
    std::string tag = getTag(pt);
    return add(pt.point(), tag + "_" + std::to_string(pt.id), getDescription(pt), styleUrl);
}

bool KMLOutDocumentOld::add(const std::vector<FieldAccessPoint> &pts, const std::string& styleUrl) {
    for(auto& pt : pts){
        if(!add(pt, styleUrl))
            return false;
    }
    return true;
}

bool KMLOutDocumentOld::add(const ResourcePoint &pt, const std::string& styleUrl) {
    std::string tag = getTag(pt);
    return add(pt.point(), tag + "_" + std::to_string(pt.id), getDescription(pt), styleUrl);
}

bool KMLOutDocumentOld::add(const std::vector<ResourcePoint> &pts, const std::string& styleUrl) {
    for(auto& pt : pts){
        if(!add(pt, styleUrl))
            return false;
    }
    return true;
}

bool KMLOutDocumentOld::add(const std::vector<HeadlandPoint> &pts, const std::string& styleUrl) {
    return add( HeadlandPoint::toPoints(pts), getTag(pts), true, "", styleUrl );
}

bool KMLOutDocumentOld::add(const Track &track, std::string tag, const std::string& styleUrl)
{
    tag = getTag<Track>(tag);
    return add(track.points, tag + "_" + std::to_string(track.id), true, getDescription(track), styleUrl);
}

bool KMLOutDocumentOld::add(const std::vector<Track> &tracks, std::string tag, const std::string& styleUrl)
{
    for(auto& track : tracks){
        if(!add(track, tag, styleUrl))
            return false;
    }
    return true;
}

bool KMLOutDocumentOld::add(const Headlands &headlands, const std::string &styleUrl_complete, const std::string &styleUrl_partial)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    ok &= add(headlands.complete, styleUrl_complete);
    ok &= add(headlands.partial, styleUrl_partial);

    return ok;

}

bool KMLOutDocumentOld::add(const CompleteHeadland &hl, const std::string &styleUrl)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    std::string tag = getTag(hl);

    ok &= add( HeadlandPoint::toPoints(hl.headland_points), tag + "_" + getTag(hl.headland_points), true, "", styleUrl );
    ok &= add(hl.middle_track, tag + "_" + "middle_track", styleUrl);
    ok &= add(hl.tracks, tag + "_" + getTag<Track>(), styleUrl);

    return ok;
}

bool KMLOutDocumentOld::add(const PartialHeadland &hl, const std::string &styleUrl)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    std::string tag = getTag(hl);

    ok &= add(hl.boundary, tag + "_" + std::to_string(hl.id) + "_" + "_boundary", styleUrl);
    ok &= add(hl.tracks, tag + "_" + getTag<Track>(), styleUrl);

    return ok;

}

bool KMLOutDocumentOld::add(const std::vector<PartialHeadland> &headlands, const std::string &styleUrl)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    for(auto &hl : headlands){
        ok &= add(hl, styleUrl);
        if(!ok)
            break;
    }

    return ok;

}

bool KMLOutDocumentOld::add(const Obstacle &obs, int id, const std::string& styleUrl){
    std::string tag = getTag(obs);
    return add(obs.boundary, tag + "_" + std::to_string(id), getDescription(obs), styleUrl);
}

bool KMLOutDocumentOld::add(const std::vector<Obstacle> &obstacles, const std::string& styleUrl)
{
    for(size_t i = 0 ; i < obstacles.size() ; ++i){
        if(! add(obstacles.at(i), i, styleUrl) )
            return false;
    }
    return true;
}


bool KMLOutDocumentOld::add(const Subfield &sf){
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    ok &= add(getDescription(sf), KMLTag_description);

    ok &= add(sf.boundary_outer, "outer_boundary", "", getDefaultStyleUrl(Geom_Subfield_outerBoundary));
    ok &= add(sf.boundary_inner, "inner_boundary", "", getDefaultStyleUrl(Geom_Subfield_innerBoundary));

    ok &= add(sf.access_points, getDefaultStyleUrl(Geom_FieldAccessPoint));
    ok &= add(sf.resource_points, getDefaultStyleUrl(Geom_ResourcePoint));

    ok &= add(sf.reference_lines, "reference_line", getDefaultStyleUrl(Geom_ReferenceLine));

    if(sf.reference_line_A.isInvalid() && sf.reference_line_B.isInvalid()){
        ok &= add(sf.reference_line_A, "reference_line_A");
        ok &= add(sf.reference_line_B, "reference_line_B");
    }

    ok &= add(sf.headlands, getDefaultStyleUrl(Geom_Headland_complete), getDefaultStyleUrl(Geom_Headland_partial));

    ok &= add(sf.obstacles, getDefaultStyleUrl(Geom_Obstacle_boundary));

    ok &= add(sf.tracks, UseDefaultTag, getDefaultStyleUrl(Geom_Infield_track));

    return ok;
}

bool KMLOutDocumentOld::add(const std::vector<Subfield> &subfields)
{
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    std::string tag = getTag<Subfield>();
    for(size_t i = 0 ; i < subfields.size() ; ++i){
        openTag(KMLTag_folder);
        ok &= add(tag + "_" + std::to_string(i), KMLTag_name);
        if(!add(subfields.at(i)))
            return false;
        closeTag();
    }

    return ok;
}

bool KMLOutDocumentOld::add(const Field &field){
    bool ok = true;
    if(!isReadyToWrite())
        return false;

    //add field (general) data

    openTag(KMLTag_folder);

    ok &= add(field.name, KMLTag_name);
    ok &= add(getDescription(field), KMLTag_description);
    ok &= add(field.outer_boundary, "outer_boundary", "", getDefaultStyleUrl(Geom_Field_outerBoundary));

    closeTag();

    ok &= add(field.subfields);

    return ok;
}

bool KMLOutDocumentOld::addStyle(const std::string& styleUrl, const KMLOutDocumentOld::KmlStyle &style)
{
    if(!AroOutDocument::isReadyToWrite())
        return false;
    if(styleUrl.empty())
        return false;

    openTag(KMLTag_style, "id='"+styleUrl+"'");

    if(!style.color.empty() || style.scale > 1e-6 || !style.iconRef.empty()){
        openTag(KMLTag_iconstyle);
        if(!style.color.empty())
            add(style.color, KMLTag_color);
        if(style.scale > 1e-6)
            add(style.scale, KMLTag_scale);
        if(!style.iconRef.empty()){
            openTag(KMLTag_icon);
            add(style.iconRef, KMLTag_iconref);
            closeTag();
        }
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

bool KMLOutDocumentOld::addDefaultStyle(KMLOutDocumentOld::GeometryType geomType)
{
    try{
        return addStyle( getDefaultStyleUrl(geomType), m_styles.at(geomType) );
    }
    catch(...){
        return false;
    }
}

bool KMLOutDocumentOld::addDefaultStyles(const std::set<KMLOutDocumentOld::GeometryType>& geomTypes)
{
    std::set<KMLOutDocumentOld::GeometryType> geomTypes2;
    const std::set<KMLOutDocumentOld::GeometryType> *pGeomTypes = &geomTypes;

    if(geomTypes.empty()){
        geomTypes2 = {Geom_Field_outerBoundary,
                      Geom_Subfield_outerBoundary,
                      Geom_Subfield_innerBoundary,
                      Geom_Obstacle_boundary,
                      Geom_ReferenceLine,
                      Geom_ResourcePoint,
                      Geom_FieldAccessPoint,
                      Geom_HeadlandPoints,
                      Geom_Infield_track,
                      Geom_Headland_track,
                      Geom_Headland_midTrack,
                      Geom_Headland_complete,
                      Geom_Headland_partial};
        pGeomTypes = &geomTypes2;
    }

    for(auto t : *pGeomTypes){
        if(!addDefaultStyle(t))
            return false;
    }
    return true;

}

bool KMLOutDocumentOld::saveField(const std::string &filename, const Field &field, Point::ProjectionType coordinatesType_in)
{
    KMLOutDocumentOld doc;
    doc.setCoordinatesTypes(coordinatesType_in, Point::WGS);
    return doc.openFile(filename) &&
           doc.openDocument() &&
           doc.add("FIELD - " + field.name, KMLTag_name) &&
           doc.add(field) &&
           doc.addDefaultStyles() &&
           doc.closeDocument() &&
           doc.closeFile();
}

std::string KMLOutDocumentOld::getDefaultStyleUrl(KMLOutDocumentOld::GeometryType geomType)
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
    if(geomType == Geom_HeadlandPoints)
        return "Geom_HeadlandPoints_defStyle";
    if(geomType == Geom_Infield_track)
        return "Geom_Infield_track_defStyle";
    if(geomType == Geom_Headland_track)
        return "Geom_Headland_track_defStyle";
    if(geomType == Geom_Headland_midTrack)
        return "Geom_Headland_midTrack_defStyle";
    if(geomType == Geom_Headland_complete)
        return "Geom_Headland_complete";
    if(geomType == Geom_Headland_partial)
        return "Geom_Headland_partial";
    return "";
}

bool KMLOutDocumentOld::openDoc()
{
    if(!AroOutDocument::isReadyToWrite())
        return false;

    *m_os << std::setprecision(12)
          << "<?xml version='1.0' encoding='UTF-8'?>\n";

    openTag("kml", "xmlns='http://www.opengis.net/kml/2.2'");

    openTag(KMLTag_document);

    return true;
}

bool KMLOutDocumentOld::closeDoc()
{
    if(!AroOutDocument::isReadyToWrite())
        return false;
    closeTag(-1);
    return true;
}

void KMLOutDocumentOld::tabs(int n){
    if(n <= 0)
        return;
    *m_os << std::string(n, '\t');
}

void KMLOutDocumentOld::tabs()
{
    if(m_nTabs <= 0)
        return;
    *m_os << std::string(m_nTabs, '\t');
}

void KMLOutDocumentOld::initDefaultStyles()
{
    m_styles = { { Geom_Field_outerBoundary, KmlStyle("FF606060", KmlStyle::WidthBold, -1, "") },
                 { Geom_Subfield_outerBoundary, KmlStyle("FF000000", KmlStyle::WidthBold, -1, "") },
                 { Geom_Subfield_innerBoundary, KmlStyle("FF000000", KmlStyle::WidthBold, -1, "") },
                 { Geom_Obstacle_boundary, KmlStyle("FFFF0000", KmlStyle::WidthNormal, -1, "") },
                 { Geom_ReferenceLine, KmlStyle("FFFFFF00", KmlStyle::WidthNormal, -1, "") },
                 { Geom_ResourcePoint, KmlStyle("FFFF007F", -1, -1, "http://maps.google.com/mapfiles/kml/pal3/icon31.png") },
                 { Geom_FieldAccessPoint, KmlStyle("FF0080FF", -1, -1, "http://maps.google.com/mapfiles/kml/pal4/icon28.png") },
                 { Geom_HeadlandPoints, KmlStyle("FFFF6666", KmlStyle::WidthNormal, -1, "") },
                 { Geom_Infield_track, KmlStyle("FFFF0000", KmlStyle::WidthNormal, -1, "") },
                 { Geom_Headland_track, KmlStyle("FFFF0000", KmlStyle::WidthNormal, -1, "") },
                 { Geom_Headland_midTrack, KmlStyle("FFFF6666", KmlStyle::WidthNormal, -1, "") },
                 { Geom_Headland_complete, KmlStyle("FF909090", KmlStyle::WidthNormal, -1, "") },
                 { Geom_Headland_partial, KmlStyle("FF909090", KmlStyle::WidthNormal, -1, "") } };
}

std::string KMLOutDocumentOld::getDescription(const Field &field)
{
    return    "id=" + std::to_string(field.id) + "\n"
            + "name=" + field.name;
}

std::string KMLOutDocumentOld::getDescription(const Subfield &sf)
{
    return    "id=" + std::to_string(sf.id);
}

std::string KMLOutDocumentOld::getDescription(const Linestring &ls)
{
    return    "id=" + std::to_string(ls.id);
}

std::string KMLOutDocumentOld::getDescription(const FieldAccessPoint &pt)
{
    return    "id=" + std::to_string(pt.id) + "\n"
            + "accessType=" + std::to_string(pt.accessType) + "\n"
            + "accessPermission=" + std::to_string(pt.accessPermission);
}

std::string KMLOutDocumentOld::getDescription(const ResourcePoint &pt)
{
    std::string resourceTypes;
    for(auto& t : pt.resourceTypes){
        resourceTypes += std::to_string(t) + ",";
    }
    if(!resourceTypes.empty())
        resourceTypes.pop_back();

    return    "id=" + std::to_string(pt.id) + "\n"
            + "resourceTypes=" + resourceTypes + "\n"
            + "defaultUnloadingTime=" + double2string(pt.defaultUnloadingTime) + "\n"
            + "defaultUnloadingTimePerKg=" + double2string(pt.defaultUnloadingTimePerKg);

}

std::string KMLOutDocumentOld::getDescription(const HeadlandPoint &pt)
{
    return    "track_id=" + std::to_string(pt.track_id);
}

std::string KMLOutDocumentOld::getDescription(const Track &track)
{
    return    "id=" + std::to_string(track.id) + "\n"
            + "distToPrev=" + double2string(track.distToPrev) + "\n"
            + "distToNext=" + double2string(track.distToNext);

}

std::string KMLOutDocumentOld::getDescription(const Obstacle &obs)
{
    return    "type=" + std::to_string(obs.type);
}

}//end namespace arolib

