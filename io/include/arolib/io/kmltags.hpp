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
 
#ifndef AROLIB_IO_KMLTAGSMANAGER_HPP
#define AROLIB_IO_KMLTAGSMANAGER_HPP

#include "arolib/types/field.hpp"

namespace arolib {
namespace io {

const std::string KMLTag_document = "Document"; /**<KML-tag - Document */
const std::string KMLTag_folder = "Folder"; /**<KML-tag - Folder */
const std::string KMLTag_placemark = "Placemark"; /**<KML-tag - Placemark */
const std::string KMLTag_name = "name"; /**<KML-tag - name */
const std::string KMLTag_description = "description"; /**<KML-tag - description */
const std::string KMLTag_coordinates = "coordinates"; /**<KML-tag - coordinates */
const std::string KMLTag_point = "Point"; /**<KML-tag - Point */
const std::string KMLTag_linestring = "LineString"; /**<KML-tag - LineString */
const std::string KMLTag_polygon = "Polygon"; /**<KML-tag - Polygon */
const std::string KMLTag_polygonBoundary = "outerBoundaryIs"; /**<KML-tag - outerBoundaryIs */
const std::string KMLTag_linearring = "LinearRing"; /**<KML-tag - LinearRing */
const std::string KMLTag_style = "Style"; /**<KML-tag - Style */
const std::string KMLTag_styleUrl = "styleUrl"; /**<KML-tag - styleUrl */
const std::string KMLTag_linestyle = "LineStyle"; /**<KML-tag - LineStyle */
const std::string KMLTag_polystyle = "PolyStyle"; /**<KML-tag - PolyStyle */
const std::string KMLTag_iconstyle = "IconStyle"; /**<KML-tag - IconStyle */
const std::string KMLTag_labelstyle = "LabelStyle"; /**<KML-tag - LabelStyle */
const std::string KMLTag_icon = "Icon"; /**<KML-tag - Icon */
const std::string KMLTag_color = "color"; /**<KML-tag - color */
const std::string KMLTag_scale = "scale"; /**<KML-tag - scale */
const std::string KMLTag_width = "width"; /**<KML-tag - width */
const std::string KMLTag_iconref = "href"; /**<KML-tag - href */
const std::string KMLTag_polyfill = "fill"; /**<KML-tag - fill */

/**
 * @brief Get the KML base tag for a given type T
 * @param filename File name/path
 * @return KML base tag
 */
template <typename T>
static const std::string& getKMLBaseTag(const T&){
    //exceptions
    if( std::is_same<T, ResourcePoint>::value )
        return KMLTag_folder;

    if( std::is_base_of<Point, T>::value
            || std::is_same< std::vector<Point>, T >::value
            || std::is_base_of<Linestring, T>::value
            || std::is_base_of<Polygon, T>::value )
        return KMLTag_placemark;

    if( std::is_arithmetic<T>::value
         || std::is_enum<T>::value
         || std::is_same<T*, std::string>::value
         || std::is_same<T*, const char*>::value )//should be avoided, but fits more to description than to folder
        return KMLTag_description;

    //default -> Folder
    return KMLTag_folder;
}

}
}//end namespace arolib


#endif //AROLIB_IO_KLMTAGSMANAGER_HPP

