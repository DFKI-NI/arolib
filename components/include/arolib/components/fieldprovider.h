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
 
#ifndef AROLIB_FIELDPROVIDER_H
#define AROLIB_FIELDPROVIDER_H

#include <unistd.h>
#include <iostream>
#include <math.h>
#include <string>
#include <dirent.h>
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/io/io_kml.hpp"
#include "arolib/io/io_xml.hpp"
#include "arolib/io/io_hdf5.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"

#include "arolib/misc/basic_responses.h"

namespace arolib {

/**
 * @brief Class used to load fields from files
 */
class FieldProvider : public LoggingComponent
{
public:

    /**
     * @brief Type of file
     */
    enum FileType{
        KML,
        XML,
        HDF5,
        UNDEFINED
    };

    /**
     * @brief Structure containing the important search-related information of the field.
     */
    struct SubfieldInfo {
        int id; /**< Id */
        size_t numRefLines; /**< Number of reference lines */
    };

    /**
     * @brief Structure containing the important search-related information of the field.
     */
    struct FieldInfo {
        FileType filetype; /**< File type */
        std::string filename; /**< File name/path */
        std::string fieldname; /**< Field name */

        int id; /**< Id */
        std::vector<SubfieldInfo> subfields; /**< Subfields */
    };

    /**
     * @brief Constructor.
     *
     * Automatically creates the (local) database of fields' info based on the fields located in in the directories 'baseDir/klm',  'baseDir/xml'
     * @param baseDir Base directory (searches are done from this directory)
     * @param allowMultipartFields Flag to state of multipart fields are supported (@TODO for future use)
     * @param logLevel Log level
     */
    explicit FieldProvider(const std::string &baseDir,
                           const LogLevel& logLevel = LogLevel::INFO);

    /**
     * Copy constructor.
     * @param other Other
     */
    FieldProvider(const FieldProvider& other);

    /**
     * Copy assignment.
     * @param other Other
     * @return this
     */
    FieldProvider& operator=(const FieldProvider& other);


    /**
     * @brief Retrieve the list of field ids in the (local) data base.
     * @param [out] fieldsIds Field ids
     * @param fieldsIds Filter the output list with the given file type (extension)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getAllFieldsIds(std::vector<std::string>& fieldsIds, const FileType& fileType = FileType::UNDEFINED, bool sortById = true);

    /**
     * @brief Retrieve the list of fields info in the (local) data base.
     * @param [out] fieldsInfo Field info
     * @param fieldsIds Filter the output list with the given file type (extension)
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getAllFieldsInfo(std::map<std::string, FieldInfo> &fieldsInfo, const FileType &fileType = FileType::UNDEFINED);

    /**
     * @brief Retrieve the ifo of a field.
     * @param fieldId Id of the field
     * @param [out] fieldInfo Field info
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getFieldInfo(const std::string &fieldId, FieldInfo& fieldInfo);

    /**
     * @brief Read a field from the (local) data base.
     * @param fieldId Id of the field to be retrieved
     * @param [out] field Field
     * @param [out] filename Filename of the retrieved field
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp readField(const std::string &fieldId, Field& field, std::string &filename);

    /**
     * @brief Read a field from the given file.
     * @param filename Filename/path
     * @param [out] field Field
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp readFieldFromFile(const std::string &filename, Field& field);

    /**
     * @brief Save a field in a file.
     * @param field Field to be saved
     * @param filename File path/name
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp saveField(const Field& field, const std::string& filename);

    /**
     * @brief Get the list of maps available for a field
     * @param fieldId Field Id
     * @param [out] maps List of maps grouped by type: <type, names>
     * @return True on success
     */
    AroResp getFieldMapsList(const std::string &fieldId, std::map< std::string, std::vector<std::string> >& maps);

    /**
     * @brief Request a field map (one grid is returned per layer)
     * @param fieldId Field Id
     * @param type Map type
     * @param name Map name
     * @param [out] map Map layers
     * @return True on success
     */
    AroResp getFieldMap(const std::string &fieldId, const std::string& type, const std::string& name, std::vector<ArolibGrid_t>& map);



protected:

    /**
     * @brief Retrieve the list of fields in KLM format found in a directory and save it in the (local) data base.
     * @param directory_name Directory to search for fields
     * @param [out] num_fields Number of fields successfully added to the (local) data base
     * @return Number of files that where found and read
     */
    int getFieldList_kml(std::string directory_name, unsigned int& num_fields);

    /**
     * @brief Retrieve the list of fields in XML format found in a directory and save it in the (local) data base.
     * @param directory_name Directory to search for fields
     * @param [out] num_fields Number of fields successfully added to the (local) data base
     * @return Number of files that where found and read
     */
    int getFieldList_xml(std::string directory_name, unsigned int& num_fields);

    /**
     * @brief Retrieve the list of fields found in an HDF5 file.
     * @param directory_name Directory to search for fields
     * @param [out] num_fields Number of fields successfully added to the (local) data base
     * @return Number of files that where found and read
     */
    int getFieldList_hdf5(std::string directory_name, unsigned int& num_fields);

    /**
     * @brief Read a field from a KML file based on the data saved in the (local) data base.
     * @param fi Struct containing the information of the field (inc. filename and fieldname)
     * @param [out] field Read field
     * @return True on success
     */
    bool getField_kml(const FieldInfo& fi, Field& field);

    /**
     * @brief Read a field from a XML file based on the data saved in the (local) data base.
     * @param fi Struct containing the information of the field (inc. filename and fieldname)
     * @param [out] field Read field
     * @return True on success
     */
    bool getField_xml(const FieldInfo& fi, Field& field);

    /**
     * @brief Create field id (key).
     * @param field Field
     * @param filename filename
     * @return Create field id (key).
     */
    std::string createFieldKey(const Field& field, std::string filename, const std::string &fileEndPattern = "");

    /**
     * @brief Create field id (key).
     * @param fieldId Field Id
     * @param fieldName Field name
     * @param filename filename
     * @return Create field id (key).
     */
    std::string createFieldKey(int fieldId, const std::string& fieldName, std::string filename, const std::string &fileEndPattern = "");

    /**
     * @brief Create a FieldInfo.
     * @param field Field
     * @param filepath filepath
     * @param filename filename
     * @param filetype filetype
     * @return Created FieldInfo
     */
    FieldInfo createFieldInfo(const Field& field, const std::string& filepath, const std::string& filename, FileType filetype);

    /**
     * @brief Read a field from a HDF5 file based
     * @param fi Struct containing the information of the field (inc. filename and fieldname)
     * @param [out] field Read field
     * @return True on success
     */
    bool getField_hdf5(const FieldInfo& fi, Field& field);

    /**
     * @brief Get the list of maps available for a field
     * @param fi Struct containing the information of the field (inc. filename and fieldname)
     * @param [out] maps List of maps grouped by type: <type, names>
     * @return True on success
     */
    bool getFieldMapsList_hdf5(const FieldInfo& fi, std::map< std::string, std::vector<std::string> >& maps);

    /**
     * @brief Request a field map (one grid is returned per layer)
     * @param fi Struct containing the information of the field (inc. filename and fieldname)
     * @param type Map type
     * @param name Map name
     * @param [out] map Map layers
     * @return True on success
     */
    bool getFieldMap_hdf5(const FieldInfo& fi, const std::string& type, const std::string& name, std::vector<ArolibGrid_t>& map);

    /**
     * @brief Utility function to compare field Ids (used for sorting)
     * @param a Id 1
     * @param b Id 2
     * @return Compare result
     */
    static bool compareIds(const std::string& a, const std::string& b);


protected:
    std::map<std::string, FieldInfo> m_allFields;// /**< (local) database containing the field information*/

    int m_idCount = 0; /**< Counter for field ids */
};


}

#endif // AROLIB_FIELDPROVIDER_H
