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
 
#ifndef AROLIB_IO_KMLOutDocumentOld_HPP
#define AROLIB_IO_KMLOutDocumentOld_HPP

#include <ostream>
#include <fstream>
#include <sstream>

#include "arodocument.hpp"
#include "xmldocument.hpp"
#include "kmltags.hpp"
#include "io_common.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/misc/base64Utility.hpp"

namespace arolib {

class KMLOutDocumentOld : public AroOutDocument, public XMLDocumentBase{

public:

    enum GeometryType{
        Geom_Field_outerBoundary,
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
        Geom_Headland_partial
    };

    struct KmlStyle{
        std::string color = "";//aabbggrr
        int width = -1;
        double scale = -1;
        std::string iconRef = "";
        bool fill = false;
        KmlStyle(const std::string& _color = "",
                 int _width = -1,
                 double _scale = -1,
                 const std::string& _iconRef = "",
                 bool _fill = false);
        static const int WidthBoldBold;
        static const int WidthBold;
        static const int WidthNormal;
        static const int WidthThin;
    };

    explicit KMLOutDocumentOld(LogLevel logLevel = LogLevel::INFO);
    virtual ~KMLOutDocumentOld();

    /**
      * @brief Set the coordinates (projection) type for the points in the document
      *
      * It must be set before opening the document
      * @param in Cooridinate (projection) type of the source
      * @param out Cooridinate (projection) type of the target
      * @return True on success
      */
    virtual bool setCoordinatesTypes(Point::ProjectionType in, Point::ProjectionType out) override;

    bool openTag(std::string tag, const std::string& extra = "");
    bool closeTag(int numTags = 1);

    template< typename T,
              typename = typename std::enable_if< ( std::is_arithmetic<T>::value || std::is_enum<T>::value ) && !std::is_void<T>::value >::type >
    bool add(const T& value, std::string tag){
        if(!isReadyToWrite())
            return false;
        tag = getTag<T>(tag);
        tabs();
        *m_os << "<" << tag << ">" << value << "</" << tag << ">\n";
        return true;
    }

    template< typename T >
    bool add(const std::vector<T>& items, std::string tag){
        return add(&items, tag);
    }

    template< typename T ,
              typename = typename std::enable_if< ( !std::is_same<T*, const char*>::value ) >::type >
    bool add(const std::vector<T>* items, const std::string& tag){
        for(auto& item : *items){
            if( !add(item, tag) )
                return false;
        }
        return true;
    }

    template< typename T ,
              typename = typename std::enable_if< ( !std::is_same<T*, const char*>::value ) >::type >
    bool add(const std::pair<T*, std::string>& item){
        return add(*item.first, item.second);
    }

    template< typename T >
    bool add(const std::pair<T, std::string>& item){
        return add(item.first, item.second);
    }

    template< typename T, typename... Types >
    bool add(const std::pair<T, std::string> &item, const Types& ... items){
        if(!add(item))
            return false;
        return add(items...);
    }

    bool add(const char* value, std::string tag);
    bool add(const std::string& value, std::string tag);

    bool add(const Point& pt, const std::string& name, const std::string& description = "", const std::string& styleUrl = "");
    bool add(const std::vector<Point>& pts, const std::string& name, bool asLinestring = true, const std::string& description = "", const std::string& styleUrl = "");

    bool add(const Linestring& ls, const std::string& name, const std::string& description = "", const std::string& styleUrl = "");
    bool add(const std::vector<Linestring>& lss, const std::string& name_prefix, const std::string& styleUrl = "");
    bool add(const Polygon& poly, const std::string& name, const std::string& description = "", const std::string& styleUrl = "");

    bool add(const FieldAccessPoint &pt, const std::string& styleUrl = "");
    bool add(const std::vector<FieldAccessPoint>& pts, const std::string& styleUrl = "");

    bool add(const ResourcePoint &pt, const std::string& styleUrl = "");
    bool add(const std::vector<ResourcePoint>& pts, const std::string& styleUrl = "");

    bool add(const std::vector<HeadlandPoint>& pts, const std::string& styleUrl = "");

    bool add(const Track& track, std::string tag = UseDefaultTag, const std::string& styleUrl = "");
    bool add(const std::vector<Track>& tracks, std::string tag = UseDefaultTag, const std::string& styleUrl = "");

    bool add(const Headlands& headlands, const std::string& styleUrl_complete = "", const std::string& styleUrl_partial = "");
    bool add(const CompleteHeadland& hl, const std::string& styleUrl = "");
    bool add(const PartialHeadland& hl, const std::string& styleUrl = "");
    bool add(const std::vector<PartialHeadland>& headlands, const std::string& styleUrl = "");

    bool add(const Obstacle& obs, int id, const std::string& styleUrl = "");
    bool add(const std::vector<Obstacle>& obstacles, const std::string& styleUrl = "");

    bool add(const Subfield& sf);
    bool add(const std::vector<Subfield>& subfields);
    bool add(const Field& field);

    bool addStyle(const std::string & styleUrl, const KmlStyle& style);
    bool addDefaultStyle(GeometryType geomType);
    bool addDefaultStyles(const std::set<GeometryType> &geomTypes = {});

    static bool saveField( const std::string& filename,
                           const Field& field,
                           Point::ProjectionType coordinatesType_in = Point::UTM );

    static std::string getDefaultStyleUrl(GeometryType geomType);

    static const std::string DefaultStyle;


protected:
    virtual bool openDoc();
    virtual bool closeDoc();

    inline void tabs(int n);
    void tabs();

    void initDefaultStyles();

    static std::string getDescription(const Field& field);
    static std::string getDescription(const Subfield& sf);
    static std::string getDescription(const Linestring& ls);
    static std::string getDescription(const FieldAccessPoint& pt);
    static std::string getDescription(const ResourcePoint& pt);
    static std::string getDescription(const HeadlandPoint& pt);
    static std::string getDescription(const Track& track);
    static std::string getDescription(const Obstacle& obs);

protected:
    int m_nTabs = 0;
    static const std::map<std::type_index, std::string> m_tags;
    std::vector<std::string> m_openTags;

    std::map<GeometryType, KmlStyle> m_styles;

};

}//end namespace arolib


#endif //AROLIB_IO_KMLOutDocumentOld_HPP

