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
 
#ifndef AROLIB_IO_KML_HPP
#define AROLIB_IO_KML_HPP

#include <string>
#include <sstream>
#include <iterator>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional/optional.hpp>

#include "arolib/io/arokmlindocument.hpp"
#include "arolib/io/arokmloutdocument.hpp"
#include "arolib/geometry/geometry_helper.hpp"


namespace arolib {
namespace io {

/**
 * @brief Read a field from a (arolib-formatted) KML file
 * @param filename File name/path
 * @param [out] field Read field
 * @param coordinatesType_out Projection type for the target coordinates
 * @return true on success
 */
bool readFieldKML(const std::string& filename, Field& field, Point::ProjectionType coordinatesType_out = Point::UTM);

/**
 * @brief Read fields from a (arolib-formatted) KML file
 * @param filename File name/path
 * @param [out] fields Read fields
 * @param coordinatesType_out Projection type for the target coordinates
 * @return true on success
 */
bool readFieldsKML(const std::string& filename, std::vector<Field>& fields, Point::ProjectionType coordinatesType_out = Point::UTM);

/**
 * @brief Write/save a field in a (arolib-formatted) KML file
 * @param filename File name/path
 * @param field Field to be written/saved
 * @param coordinatesType_in Projection type of the coordinates in the source (field)
 * @return true on success
 */
bool writeFieldKML(const std::string& filename, const Field &field, Point::ProjectionType coordinatesType_in = Point::UTM);

/**
 * @brief Write/save fields in a (arolib-formatted) KML file
 * @param filename File name/path
 * @param fields Fields to be written/saved
 * @param coordinatesType_in Projection type of the coordinates in the source (field)
 * @return true on success
 */
bool writeFieldsKML(const std::string& filename, const std::vector<arolib::Field>& fields, Point::ProjectionType coordinatesType_in = Point::UTM);


/**
 * @brief [deprecated] Read a list of fields from a given KML file.
 *
 * No structure of the kml file is needed (as in read_field_kml).
 * Each Polygon geometry in the kml fiel is considered as a field outer boundary. Each
 * linestring is considered as reference line of a field if it is completly
 * inside the fields outer border.
 * @param filename File name/path
 * @param field Read fields
 * @return true on success
 */
bool read_field_kml_best_guess(const std::string& filename, std::vector<arolib::Field>& fields);

/**
 * @brief [deprecated] Read a list of fields from a given KML file.
 *
 * No structure of the kml file is needed (as in read_field_kml).
 * Each Polygon geometry in the kml fiel is considered as a field outer boundary. Each
 * linestring is considered as reference line of a field if it is completly
 * inside the fields outer border.
 * @param filename File name/path
 * @param field Read fields
 * @return true on success
 */
bool read_field_kml_best_guess__ed(const std::string& filename, std::vector<arolib::Field>& fields);

}
}

#endif //AROLIB_IO_KML_HPP
