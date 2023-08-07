/*
 * Copyright 2023  DFKI GmbH and Universität Osnabrück
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
 
#ifndef AROLIB_IO_HDF5_HPP
#define AROLIB_IO_HDF5_HPP
#undef H5_USE_BOOST
#define H5_USE_BOOST

#include <boost/multi_array.hpp>
#include "arolib/misc/logger.h"
#include "arolib/types/field.hpp"
#include "arolib/types/resourcepoint.hpp"
#include "arolib/cartography/common.hpp"

#include <map>

#include <hdf5_hl.h>
#include <highfive/H5DataSet.hpp>
#include <highfive/H5DataSpace.hpp>
#include <highfive/H5File.hpp>
#include <highfive/H5Easy.hpp>

namespace arolib {
namespace io {

/**
 * @brief Holds the basic information of a grid saved in an HDF5 file
 * @param id Grid id (name)
 * @param layers Number of leyers
 * @return  true on success
 */
struct HDF5GridInfo{
    std::string id;
    size_t layers;
};


/**
 * @brief Reads a Field from the given HDF5 file and fills the
 *        geven Field reference. In contrast to read_field_kml_best_guess it
 *        fills the complete field structure not only outer boundary,
 *        obstacles and reference lines.
 * @param file Handle to an open HDF5 file
 * @param filename the filename
 * @param field the field reeference that is filled in this function
 * @return  true on success
 */
bool read_field_hdf5(HighFive::File& file, const std::string& fieldname, arolib::Field& f);

/**
 * @brief Reads a Field from the given HDF5 file and fills the
 *        geven Field reference. In contrast to read_field_kml_best_guess it
 *        fills the complete field structure not only outer boundary,
 *        obstacles and reference lines.
 * @param filename Name of the HDF5 file
 * @param filename the filename
 * @param field the field reeference that is filled in this function
 * @return
 */
bool read_field_hdf5(const std::string& filename, const std::string& fieldname, arolib::Field& f);

/**
 * @brief Writes a Field to the given HDF5 file
 * @param filename Name of the HDF5 file
 * @param filename the filename
 * @param field the field reference
 * @return  true on success
 */
bool write_field_hdf5(const std::string& file_path, const std::string& field_name, const arolib::Field &f);

/**
 * @brief Deletes a field in a HDF5 file
 *
 * @param file_path
 * @param field_name
 * @return true on success
 * @return false on failure
 */
bool delete_field_hdf5(const std::string& file_path, const std::string& field_name);


/**
 * @brief Get the field names in a hdf5 file
 *
 * @param file_path
 * @return std::vector<std::string>
 */
std::vector<std::string> get_field_names_hdf5(const std::string& file_path);

/**
 * @brief Reads a single-layer grid (map) from a HDF5 file, given it's type and name
 *        The grid structure in a HDF5 file is:
 *            grid_type (e.g. soilmap)
 *              |
 *              |
 *              ---> grid_name
 *
 * @param file_path
 * @param grid_type
 * @param grid_name
 * @param grid
 * @return true
 * @return false
 */
bool read_grid_hdf5(const std::string& file_path, const std::string& grid_type, const std::string& grid_name, arolib::ArolibGrid_t &grid);

/**
 * @brief Reads a multi-layer grid (map) from a HDF5 file, given it's type and name
 *        The grid structure in a HDF5 file is:
 *            grid_type (e.g. soilmap)
 *              |
 *              |
 *              ---> grid_name
 *
 * @param file_path
 * @param grid_type
 * @param grid_name
 * @param layers
 * @return true
 * @return false
 */

bool read_grid_hdf5(const std::string& file_path, const std::string& grid_type, const std::string& grid_name, std::vector< arolib::ArolibGrid_t > &layers);

/**
 * @brief Writes a single-layer grid to a HDF5 file. If a grid with the same name exists it will be overwritten.
 *         The grid structure in a HDF5 file is:
 *            grid_type (e.g. soilmap)
 *              |
 *              |
 *              ---> grid_name
 *
 * @param file_path
 * @param grid_type
 * @param grid_name
 * @param grid
 * @return true
 * @return false
 */
bool write_grid_hdf5(const std::string& file_path, const std::string& grid_type, const std::string& grid_name, const arolib::ArolibGrid_t &grid);

/**
 * @brief Writes a multi-layer grid to a HDF5 file. If a grid with the same name exists it will be overwritten.
 *         The grid structure in a HDF5 file is:
 *            grid_type (e.g. soilmap)
 *              |
 *              |
 *              ---> grid_name
 *
 * @param file_path
 * @param grid_type
 * @param grid_name
 * @param layers
 * @return true
 * @return false
 */
bool write_grid_hdf5(const std::string& file_path, const std::string& grid_type, const std::string& grid_name, const std::vector<const arolib::ArolibGrid_t*> &layers);

/**
 * @brief Delete a grid in a HDF5 file
 *
 * @param file_path
 * @param grid_type
 * @param grid_name
 * @return true
 * @return false
 */
bool delete_grid_hdf5(const std::string& file_path, const std::string& grid_type, const std::string& grid_name);



/**
 * @brief Get the grid types in a hdf5 file
 *
 * @param file_path
 * @return std::vector<std::string>
 */
std::vector<std::string> get_grid_types_hdf5(const std::string& file_path);

/**
 * @brief Get the grid info in a hdf5 file
 *
 * @param file_path
 * @return std::vector<HDF5GridInfo>
 */
std::map< std::string, std::vector<HDF5GridInfo> > get_grid_list_hdf5(const std::string& file_path);

/**
 * @brief Get the grid info in a hdf5 file for a specific type
 *
 * @param file_path
 * @return std::vector<HDF5GridInfo>
 */
std::vector<HDF5GridInfo> get_grid_list_hdf5(const std::string& file_path, const std::string& grid_type);
/**
 * @brief Get the grid names in a hdf5 file
 *
 * @param file_path
 * @return std::vector<std::string>
 */
std::vector<std::string> get_grid_names_hdf5(const std::string& file_path, const std::string& grid_type);

/**
 * @brief Get the number of layers of a grid.
 * @param file_path
 * @param grid_type
 * @param grid_name
 * @return number of layers or -1 if something wen't wrong (wrong path, malformed shape)
 */
int get_grid_layer_count_hdf5(const std::string& file_path, const std::string& grid_type, const std::string& grid_name);


}
}

#endif
