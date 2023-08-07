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
 
#ifndef AROLIB_IO_AROKMLOUTDOCUMENT_HPP
#define AROLIB_IO_AROKMLOUTDOCUMENT_HPP

#include <ostream>
#include <fstream>
#include <sstream>

#include "arodocument.hpp"
#include "xmloutdocument.hpp"
#include "kmltags.hpp"
#include "arolib/misc/filesystem_helper.h"
#include "io_common.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/misc/base64Utility.hpp"
#include "arolib/types/route.hpp"

namespace arolib {
namespace io {

/**
 * @brief Arolib KML input document for Arolib types
 */
class AroKMLOutDocument : public XMLOutDocument{

public:

    /**
     * @brief Geometry types
     */
    enum GeometryType{
        Geom_Field_outerBoundary,
        Geom_Subfield_outerBoundary,
        Geom_Subfield_innerBoundary,
        Geom_Obstacle_boundary,
        Geom_ReferenceLine,
        Geom_ResourcePoint,
        Geom_RoutePoint,
        Geom_Infield_track,
        Geom_Infield_track_boundary,
        Geom_Headland_track,
        Geom_Headland_track_boundary,
        Geom_Headland_midTrack,
        Geom_Headland_boundary,
        Geom_FieldAccessPoint,
        Geom_Path
    };

    /**
     * @brief KML style for geometries
     */
    struct KmlStyle{
        std::string color = "";  /**< Color (aabbggrr) */
        int width = -1;  /**< Width */
        double scale = -1; /**< Scale */
        std::string iconRef = ""; /**< Icon reference */
        bool fill = false; /**< Fill */

        /**
         * @brief Constructor
         */
        KmlStyle(const std::string& _color = "",
                 int _width = -1,
                 double _scale = -1,
                 const std::string& _iconRef = "",
                 bool _fill = false);

        static const int WidthBoldBold; /**< Very bold width */
        static const int WidthBold; /**< Bold width */
        static const int WidthNormal; /**< Normal width */
        static const int WidthThin; /**< Thin width */
    };


    /**
     * @brief Constructor.
     *
     * @param logLevel Log level
     */
    explicit AroKMLOutDocument(LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Destructor.
     */
    virtual ~AroKMLOutDocument();

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

    /**
     * @brief Add/write a point with a given name, description, and KML style url.
     * @param pt Point to be written
     * @param name Name
     * @param description Description (disregarded if empty)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const Point& pt, const std::string& name, const std::string& description = "", const std::string& styleUrl = "");

    /**
     * @brief Add/write multiple points under a given name, description, and KML style url.
     * @param pts Points to be written
     * @param name Name
     * @param asLinestring Should the ponts be written as a Linestring type?
     * @param description Description (disregarded if empty)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const std::vector<Point>& pts, const std::string& name, bool asLinestring = true, const std::string& description = "", const std::string& styleUrl = "");


    /**
     * @brief Add/write a linestring with a given name, description, and KML style url.
     * @param ls Linestring to be written
     * @param name Name
     * @param description Description (disregarded if empty)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const Linestring& ls, const std::string& name, const std::string& description = "", const std::string& styleUrl = "");

    /**
     * @brief Add/write multiple linestrings with a given name prefix, description, and KML style url.
     * @param lss Linestrings to be written
     * @param name_prefix Name prefix
     * @param styleUrl KML style url (disregarded if empty)
     * @param useOnlyPrefix Should the prefix be used as name? If false, the names of the items will be suffixed with the index number.
     * @return True on success
     */
    bool add(const std::vector<Linestring>& lss, const std::string& name_prefix, const std::string& styleUrl = "", bool useOnlyPrefix = true);

    /**
     * @brief Add/write a polygon with a given name, description, and KML style url.
     * @param poly Polygon to be written
     * @param name Name
     * @param description Description (disregarded if empty)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const Polygon& poly, const std::string& name, const std::string& description = "", const std::string& styleUrl = "");


    /**
     * @brief Add/write a field-access point with a given tag and KML style url.
     * @param pt Field-access point to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const FieldAccessPoint &pt, std::string tag = "", const std::string& styleUrl = "");

    /**
     * @brief Add/write a set of field-access points with a given tag and KML style url.
     * @param pts Field-access points to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const std::vector<FieldAccessPoint>& pts, std::string tag = "", const std::string& styleUrl = "");

    /**
     * @brief Add/write a resource point with a given tag and KML style url.
     * @param pt Resource point to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const ResourcePoint &pt,  std::string tag = "", const std::string& styleUrl = "");

    /**
     * @brief Add/write a set of resource points with a given tag and KML style url.
     * @param pt Resource point to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const std::vector<ResourcePoint>& pts, std::string tag = "", const std::string& styleUrl = "");

    /**
     * @brief Add/write a route point with a given tag and KML style url.
     * @param pt Route point to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const RoutePoint& pt, std::string tag = "", const std::string& styleUrl = "");

    /**
     * @brief Add/write a set of route points with a given tag and KML style url.
     * @param pts Route points to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const std::vector<RoutePoint>& pts, bool asLinestring = true, std::string tag = "", const std::string& styleUrl = "");

    /**
     * @brief Add/write a track with a given tag and KML style url.
     * @param track Track to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const Track& track, std::string tag = "", const std::string& styleUrl_points = "", const std::string& styleUrl_boundary = "");

    /**
     * @brief Add/write a set of tracks with a given tag and KML style url.
     * @param tracks Tracks to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const std::vector<Track>& tracks, std::string tag = "", const std::string& styleUrl_points = "", const std::string& styleUrl_boundary = "");

    /**
     * @brief Add/write headlands with a given tag and KML style urls.
     * @param pt Point to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @param styleUrl_complete KML style url for the complete headland (disregarded if empty)
     * @param styleUrl_partial KML style url for the partial headlands (disregarded if empty)
     * @return True on success
     */
    bool add(const Headlands& headlands, std::string tag = "", const std::string &styleUrl_complete = "", const std::string &styleUrl_partial = "");

    /**
     * @brief Add/write a complete headland with a given tag and KML style url.
     * @param hl Complete headland to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const CompleteHeadland& hl, std::string tag = "", const std::string &styleUrl = "");

    /**
     * @brief Add/write a partial headland with a given tag and KML style url.
     * @param hl Partial headland to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const PartialHeadland& hl, std::string tag = "", const std::string &styleUrl = "");

    /**
     * @brief Add/write a set of partial headlands with a given tag and KML style url.
     * @param headlands Partiasl headland to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const std::vector<PartialHeadland>& headlands, std::string tag = "", const std::string& styleUrl = "");

    /**
     * @brief Add/write a obstacle with a given tag and KML style url.
     * @param obs Obstacle to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const Obstacle& obs, std::string tag = "", const std::string& styleUrl = "");

    /**
     * @brief Add/write a set of obstacles with a given tag and KML style url.
     * @param obstacles Obstacles to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @param styleUrl KML style url (disregarded if empty)
     * @return True on success
     */
    bool add(const std::vector<Obstacle>& obstacles, std::string tag = "", const std::string& styleUrl = "");

    /**
     * @brief Add/write a subfield with a given tag and KML style url.
     * @param sf Subfield to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @return True on success
     */
    bool add(const Subfield& sf, std::string tag = "");

    /**
     * @brief Add/write a set of subfields with a given tag and KML style url.
     * @param subfields Subfields to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @return True on success
     */
    bool add(const std::vector<Subfield>& subfields, std::string tag = "");

    /**
     * @brief Add/write a field with a given tag and KML style url.
     * @param field Field to be written
     * @param tag Tag (if empty, the default tag for the type will be used)
     * @return True on success
     */
    bool add(const Field& field, std::string tag = "");


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
     * @brief Add a style to the KML document under the given url
     * @param styleUrl Style Url
     * @param style KML style
     * @return True on success
     */
    bool addStyle(const std::string & styleUrl, const KmlStyle& style);

    /**
     * @brief Add the default style to the KML document for a given geometry type
     * @param geomType Geometry type
     * @return True on success
     */
    bool addDefaultStyle(GeometryType geomType);

    /**
     * @brief Add the default style to the KML document for a given set of geometry type
     * @param geomTypes Geometry types (if empty, styles foe all geometry types will be added)
     * @return True on success
     */
    bool addDefaultStyles(const std::set<GeometryType> &geomTypes = {});


    /**
     * @brief Save a field in a KML file
     * @param filename Filename
     * @param field Field to be written
     * @param coordinatesType_in Projection type of the coordinates in the source (field)
     * @return True on success
     */
    static bool saveField(const std::string& filename,
                           const Field& field,
                           Point::ProjectionType coordinatesType_in = Point::UTM);

    /**
     * @brief Save multiple fields in a KML file
     * @param filename Filename
     * @param fields Fields to be written
     * @param coordinatesType_in Projection type of the coordinates in the source (fields)
     * @return True on success
     */
    static bool saveFields( const std::string& filename,
                            const std::vector<Field>& fields,
                            Point::ProjectionType coordinatesType_in = Point::UTM );


    /**
     * @brief Save a route in a KML file
     * @param filename Filename
     * @param route Route to be written
     * @param coordinatesType_in Projection type of the coordinates in the source (route)
     * @return True on success
     */
    static bool saveRoute( const std::string& filename,
                        const Route& route,
                        Point::ProjectionType coordinatesType_in = Point::UTM );


    /**
     * @brief Get the default style-url for a given gepmetry type
     * @param geomType Geometry type
     * @return Style-url
     */
    static std::string getDefaultStyleUrl(GeometryType geomType);

    static const std::string UseDefaultStyle; /**< Default value to state that the default style must be used */

protected:

    /**
     * @brief Open document (AroDocument derived class implementation).
     *
     * @return True on success
     */
    virtual bool openDoc() override;


    /**
     * @brief Initialize the default styles for the geometry types
     */
    void initDefaultStyles();


    /**
     * @brief Get the Style-Url to be used for a given geometry type and styleUrl.
     *
     * If styleUrl == UseDefaultStyle, the default style for the given geometry type will be returned; otherwise, styleUrl will be returned.
     * @param geomType Geometry type
     * @param styleUrl Input style-Url
     * @return Style-Url
     */
    static std::string getStyleUrl(GeometryType geomType, const std::string& styleUrl);


    /**
     * @brief Get the description for a field (holding its properties)
     * @param field Field
     * @return Description
     */
    static std::string getDescription(const Field& field);

    /**
     * @brief Get the description for a subfield (holding its properties)
     * @param sf Subfield
     * @return Description
     */
    static std::string getDescription(const Subfield& sf);

    /**
     * @brief Get the description for a linestring (holding its properties)
     * @param ls Linestring
     * @return Description
     */
    static std::string getDescription(const Linestring& ls);

    /**
     * @brief Get the description for a field-access point (holding its properties)
     * @param pt Field-access point
     */
    static std::string getDescription(const FieldAccessPoint& pt);

    /**
     * @brief Get the description for a resource point (holding its properties)
     * @param pt Resource point
     * @return Description
     */
    static std::string getDescription(const ResourcePoint& pt);

    /**
     * @brief Get the description for a route point (holding its properties)
     * @param pt Route point
     * @return Description
     */
    static std::string getDescription(const RoutePoint& pt);

    /**
     * @brief Get the description for a track (holding its properties)
     * @param track Track
     * @return Description
     */
    static std::string getDescription(const Track& track);

    /**
     * @brief Get the description for an obstacle (holding its properties)
     * @param obs Obstacle
     * @return Description
     */
    static std::string getDescription(const Obstacle& obs);

    /**
     * @brief Get the description for a complete headland (holding its properties)
     * @param hl Complete headland
     * @return Description
     */
    static std::string getDescription(const CompleteHeadland& hl);

    /**
     * @brief Get the description for a partial headland (holding its properties)
     * @param hl Partial headland
     * @return Description
     */
    static std::string getDescription(const PartialHeadland& hl);

protected:
    std::map<GeometryType, KmlStyle> m_styles; /**< Map holding the default KML styles for each geometry type */
};

}
}//end namespace arolib


#endif //AROLIB_IO_AROKMLOUTDOCUMENT_HPP

