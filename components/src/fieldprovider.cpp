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
 
#include "arolib/components/fieldprovider.h"

#include <H5Tpublic.h>
#include <hdf5_hl.h>
#include <highfive/H5DataSet.hpp>
#include <highfive/H5DataSpace.hpp>
#include <highfive/H5File.hpp>

namespace arolib {

using namespace arolib::io;

FieldProvider::FieldProvider(const std::string &baseDir,
                             const LogLevel& logLevel):
    LoggingComponent(logLevel, __FUNCTION__)

{
    int nbFilesKml, nbFilesXml, nbFilesHDF5;
    unsigned int nbFieldsKml, nbFieldsXml, nbFieldsHDF5;

    std::string slash = "/";
    if(baseDir.empty() || baseDir.rfind("/") == baseDir.size()-1)
        slash = "";

    std::string kmlDir = baseDir + slash + "kml/";
    m_logger.printOut(LogLevel::INFO, "", "Field directory - KML: " + kmlDir);
    nbFilesKml = getFieldList_kml(kmlDir, nbFieldsKml);
    m_logger.printOut(LogLevel::INFO, "", "\tFound: " + std::to_string(nbFieldsKml) + " fields in " + std::to_string(nbFilesKml) + " KML files.\n");

    std::string xmlDir = baseDir + slash + "xml/";
    m_logger.printOut(LogLevel::INFO, "", "Field directory - XML: " + xmlDir);
    nbFilesXml = getFieldList_xml(xmlDir, nbFieldsXml);
    m_logger.printOut(LogLevel::INFO, "", "\tFound: " + std::to_string(nbFieldsXml) + " fields in " + std::to_string(nbFilesXml) + " XML files.\n");

    std::string hdf5Dir = baseDir + slash + "hdf5/";
    m_logger.printOut(LogLevel::INFO, "", "Field directory - HDF5: " + baseDir); // using baseDir as directory for HDF5
    nbFilesHDF5 = getFieldList_hdf5(hdf5Dir, nbFieldsHDF5);
    m_logger.printOut(LogLevel::INFO, "", "\tFound: " + std::to_string(nbFieldsHDF5) + " fields in given HDF5 file.\n");
}

FieldProvider::FieldProvider(const FieldProvider &other):
    LoggingComponent(other),
    m_allFields(other.m_allFields)
{
}

FieldProvider &FieldProvider::operator=(const FieldProvider &other){
    if (this != &other) {
        this->m_logger = other.m_logger;
        this->m_allFields = other.m_allFields;
    }
    return *this;
}

AroResp FieldProvider::getAllFieldsIds(std::vector<std::string> &fieldsIds, const FileType &fileType, bool sortById)
{
    fieldsIds.clear();
    for(auto const &entry : m_allFields){
        if(fileType == UNDEFINED || fileType == entry.second.filetype)
            fieldsIds.push_back( entry.first );
    }

    if(sortById)
        std::sort(fieldsIds.begin(),fieldsIds.end(), compareIds );
    return AroResp(0,"OK");
}

AroResp FieldProvider::getAllFieldsInfo(std::map<std::string, FieldProvider::FieldInfo> &fieldsInfo, const FieldProvider::FileType &fileType)
{
    fieldsInfo.clear();
    for(auto const &entry : m_allFields){
        if(fileType == UNDEFINED || fileType == entry.second.filetype)
            fieldsInfo.insert( fieldsInfo.end(), entry );
    }
    return AroResp(0,"OK");
}

AroResp FieldProvider::getFieldInfo(const std::string &fieldId, FieldProvider::FieldInfo &fieldInfo)
{
    // read the selected field
    auto it = m_allFields.find(fieldId);
    if (it == m_allFields.end())
        return AroResp(1,"Field id '" + fieldId + "' not found in the map");
    fieldInfo = it->second;
    return AroResp(0,"OK");
}

AroResp FieldProvider::readField(const std::string &fieldId, Field &field, std::string & filename)
{
    bool fieldFound = false;

    // read the selected field
    auto it = m_allFields.find(fieldId);
    if (it == m_allFields.end())
        return AroResp(1,"Field id '" + fieldId + "' not found in the map");
    if (it->second.filetype == FileType::KML)
        fieldFound = getField_kml(it->second, field);
    else if (it->second.filetype == FileType::XML)
        fieldFound = getField_xml(it->second, field);
    else if (it->second.filetype == FileType::HDF5)
        fieldFound = getField_hdf5(it->second, field);
    else
        return AroResp(1,"Information of field id '" + fieldId + "' is corrupted (filetype)");

    if (fieldFound){
        filename = it->second.filename;

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Read " + it->second.filename + " (" + it->second.fieldname + ")");
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, std::string("\n") +
                 "   filename                : " + field.filename + "\n" +
                 "   number of subfields     : " + std::to_string( field.subfields.size() ) + "\n" +
                 "   outer boundary nr points: " + std::to_string( field.outer_boundary.points.size() ) + "\n" );
        if(!field.subfields.empty()){
            std::string msgOut = "Subfields:\n";
            for(auto &sf : field.subfields){
                msgOut += "   Subfield " + std::to_string(sf.id) + ":\n" +
                          "      outer boundary nr points: " + std::to_string( sf.boundary_outer.points.size() ) + "\n" +
                          "      inner boundary nr points: " + std::to_string( sf.boundary_inner.points.size() ) + "\n" +
                          "      nr rerefence lines      : " + std::to_string( sf.reference_lines.size() ) + "\n";
            }
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, msgOut );
        }

        return AroResp(0,"OK");

    }

    return AroResp(1,"Field \"" +  it->second.fieldname + "\" not found in the field '" + fieldId + "'");

}

AroResp FieldProvider::readFieldFromFile(const std::string &filename, Field &field)
{
    if( fileHasExtension(filename, "xml") ){
        if(!readFieldXML(filename, field))
            return AroResp(1, "Error reading file.");
    }
    else if( fileHasExtension(filename, "kml") ){
        if(!readFieldKML(filename, field))
            return AroResp(1, "Error reading file.");
    }
    else if( fileHasExtension(filename, "h5") ){
        try
        {
            HighFive::File hdf5_file(filename, HighFive::File::ReadOnly);
            if (!hdf5_file.isValid())
                return AroResp(1, "Invalid file.");

            HighFive::Group root = hdf5_file.getGroup("/");
            if (!root.exist(std::string("fields")))
                return AroResp(1, "No fields in given file.");

            HighFive::Group field_group = root.getGroup("fields");
            std::vector<std::string> field_names = field_group.listObjectNames();
            if(field_names.empty())
                return AroResp(1, "No fields in given file.");
            if(field_names.size() > 1)
                m_logger.printWarning(__FUNCTION__, "The given file has more than one field. Reading only the first one.");

            Field field;
            if(!read_field_hdf5(filename, field_names.front(),field))
                return AroResp(1, "Error reading field in file.");
        }
        catch (std::exception& e)
        {
            return AroResp(1, "Error reading field in file (exception): " + std::string( e.what() ));
        }
    }
    else if ( fileHasExtension(filename, "") ){
        m_logger.printWarning(__FUNCTION__, "No file extension (file format) recognized. Attempting to read available formats.");
        AroResp resp;
        resp = readFieldFromFile(filename + ".xml", field);
        if( !resp.isError() )
            return resp;
        resp = readFieldFromFile(filename + ".kml", field);
        if( !resp.isError() )
            return resp;
        resp = readFieldFromFile(filename + ".h5", field);
        if( !resp.isError() )
            return resp;
        return AroResp(1, "Error reading file");
    }
    else
        return AroResp(1, "Error reading file (unsupported format)");

    return AroResp::ok();
}

AroResp FieldProvider::saveField(const Field &field, const std::string &filename) {

    try {
        std::string file_extension = filename.substr(filename.rfind(".") + 1);

        if(file_extension == "kml") {
            if (!writeFieldKML(filename,field)) {
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Could not write file " + filename);
                return AroResp(1, "Could not write file " + filename );
            }
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "File " + filename + " written.");
        }
        else if(file_extension == "xml") {
            if (!writeFieldXML(filename,field)) {
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Could not write file " + filename);
                return AroResp(1, "Could not write file " + filename );
            }
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "File " + filename + " written.");
        }
        else {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Field name has unknown extension: " + file_extension );
            return AroResp(1, "Field name has unknown extension: " + file_extension );
        }
    } catch (std::exception &e) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what() );
        return AroResp(1, e.what() );
    }
    return AroResp(0,"OK");
}

AroResp FieldProvider::getFieldMapsList(const std::string &fieldId, std::map<std::string, std::vector<std::string> > &maps)
{
    bool ok = false;
    maps.clear();

    // read the selected field
    auto it = m_allFields.find(fieldId);
    if (it == m_allFields.end())
        return AroResp(1,"Field id '" + fieldId + "' not found in the map");
    if (it->second.filetype == FileType::KML
            ||(it->second.filetype == FileType::XML) ){
        logger().printWarning(__FUNCTION__, "Command not supported for the file type of the given field");
        ok = true;
    }
    else if (it->second.filetype == FileType::HDF5)
        ok = getFieldMapsList_hdf5(it->second, maps);
    else
        return AroResp(1,"Information of field id '" + fieldId + "' is corrupted (filetype)");

    if (!ok)
        return AroResp(1,"Error reading list of maps for field id '" + fieldId + "'");

    return AroResp(0,"OK");
}

AroResp FieldProvider::getFieldMap(const std::string &fieldId, const std::string &type, const std::string &name, std::vector<ArolibGrid_t> &map)
{
    bool ok = false;
    map.clear();

    // read the selected field
    auto it = m_allFields.find(fieldId);
    if (it == m_allFields.end())
        return AroResp(1,"Field id '" + fieldId + "' not found in the map");
    if (it->second.filetype == FileType::KML
            ||(it->second.filetype == FileType::XML) ){
        logger().printWarning(__FUNCTION__, "Command not supported for the file type of the given field");
        ok = true;
    }
    else if (it->second.filetype == FileType::HDF5)
        ok = getFieldMap_hdf5(it->second, type, name, map);
    else
        return AroResp(1,"Information of field id '" + fieldId + "' is corrupted (filetype)");

    if (!ok)
        return AroResp(1,"Error reading '" + type + "' map '" + name + "' for field id '" + fieldId + "'");

    return AroResp(0,"OK");

}

int FieldProvider::getFieldList_kml(std::string directory_name, unsigned int &num_fields) {
    append_slash_to_dir(directory_name);
    num_fields =  0;
    std::string pattern = ".kml";
    std::vector<std::string> filelist = get_filenames_recursive(directory_name, pattern, false, true);

    // go through all km files, get the fields
    for (size_t i=0; i < filelist.size(); i++) {
        std::vector<arolib::Field> fields;

        bool ok = readFieldsKML(directory_name + filelist.at(i), fields);

        if(!ok || fields.empty())
            read_field_kml_best_guess(directory_name + filelist.at(i), fields);

        for (size_t j=0; j < fields.size(); j++) {
            fields.at(j).id = m_idCount++;
            std::string id = createFieldKey(fields.at(j), filelist.at(i), pattern);

            if ( m_allFields.find(id) == m_allFields.end() ){
                ++num_fields;
                m_allFields[id] = createFieldInfo(fields.at(j), directory_name, filelist.at(i), FileType::KML);
            }
        }
    }

    return filelist.size();
}


int FieldProvider::getFieldList_xml(std::string directory_name, unsigned int &num_fields)
{
    append_slash_to_dir(directory_name);
    num_fields =  0;
    std::string pattern = ".xml";
    std::vector<std::string> filelist = get_filenames_recursive(directory_name, pattern, false, true);

    // go through all xml files, get the fields
    for (size_t i=0; i < filelist.size(); i++) {
        Field field;
        if( !readFieldXML(directory_name + filelist.at(i), field) )
            continue;

        field.id = m_idCount++;
        std::string id = createFieldKey(field, filelist.at(i), pattern);

        if ( m_allFields.find(id) == m_allFields.end() ){
            ++num_fields;
            m_allFields[id] = createFieldInfo(field, directory_name, filelist.at(i), FileType::XML);
        }
    }

    return filelist.size();

}

int FieldProvider::getFieldList_hdf5(std::string directory_name,
                                    unsigned int &num_fields) {

   append_slash_to_dir(directory_name);
   num_fields =  0;

   std::vector<std::string> filelist = get_filenames_recursive(directory_name, ".h5", false, true);

   // go through all files, get the fields
   for (size_t i=0; i < filelist.size(); i++) {
       auto file_name = directory_name + filelist.at(i);

       try
       {
           auto field_names = get_field_names_hdf5(file_name);
           for (size_t j = 0; j < field_names.size(); j++)
           {
               Field field;
               if(!read_field_hdf5(file_name, field_names.at(j),field)) {
                   m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error reading field " + field_names.at(j) + " in file " + file_name);
                   continue;
               }

               field.id = m_idCount++;
               std::string id = createFieldKey(field, filelist.at(i));

               // delete the "h5" from the string
               int index = id.find(".h5");
               if (index != std::string::npos) {
                   id.replace(index, 3, "");
               }

               if ( m_allFields.find(id) == m_allFields.end() ){
                   ++num_fields;
                   m_allFields[id] = createFieldInfo(field, directory_name, filelist.at(i), FileType::HDF5);
               }
           }

       }
       catch (std::exception& e)
       {
           m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error reading file " + file_name + " (exception): " + e.what());
       }

   }

   return filelist.size();

}


bool FieldProvider::getField_kml(const FieldProvider::FieldInfo &fi, arolib::Field &field){

    std::vector<arolib::Field> fields;

    bool ok = readFieldsKML(fi.filename, fields);
    if(!ok || fields.empty()){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error reading fields (readFieldsKML). Attempting with read_field_kml_best_guess..." );
        if( !read_field_kml_best_guess(fi.filename, fields) ){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error reading fields" );
            return false;
        }
    }

    for (size_t i = 0; i < fields.size(); ++i) {
        if(fields.at(i).name == fi.fieldname) {
            field = fields.at(i);
            return true;
        }
    }

    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error reading desired field" );
    return false;
}

bool FieldProvider::getField_xml(const FieldProvider::FieldInfo &fi, Field &field)
{
    if( !readFieldXML(fi.filename, field) ){
        m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "***** ERROR-XML - readFieldXML *****" );
        return false;
    }
    return true;

}

bool FieldProvider::getField_hdf5(const FieldProvider::FieldInfo &fi, Field &field) {
    Field f;
    if(read_field_hdf5(fi.filename, fi.fieldname,f)) {
        field = f;
        return true;
    }

    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "***** ERROR-HDF5 - read_field_hdf5 *****" );
    return false;
}

bool FieldProvider::getFieldMapsList_hdf5(const FieldProvider::FieldInfo &fi, std::map<std::string, std::vector<std::string> > &maps)
{
    //@TODO
    return true;
}

bool FieldProvider::getFieldMap_hdf5(const FieldProvider::FieldInfo &fi, const std::string &type, const std::string &name, std::vector<ArolibGrid_t> &map)
{
    //@TODO
    return true;
}

bool FieldProvider::compareIds(const std::string &a, const std::string &b){
    try{
        auto ind1_a = a.find(" ]");
        auto ind1_b = b.find(" ]");
        if(ind1_a == std::string::npos || ind1_b == std::string::npos)
            return a < b;
        int id_a = std::stoi( a.substr(2, ind1_a-2) );
        int id_b = std::stoi( b.substr(2, ind1_b-2) );
        return id_a < id_b;
    }
    catch(...){
        return a < b;
    }

}

std::string FieldProvider::createFieldKey(const Field &field, std::string filename, const std::string &fileEndPattern)
{
    return  createFieldKey(field.id, field.name, filename, fileEndPattern);
}

std::string FieldProvider::createFieldKey(int fieldId, const std::string &fieldName, std::string filename, const std::string &fileEndPattern)
{
    if(!fileEndPattern.empty()){
        int index = filename.find(fileEndPattern);
        if (index == filename.size() - fileEndPattern.size())
            filename.replace(index, fileEndPattern.size(), "");
    }
    return  "[ " + std::to_string(fieldId) + " ] " + filename + " - " + fieldName;
}

FieldProvider::FieldInfo FieldProvider::createFieldInfo(const Field &field, const std::string &filepath, const std::string &filename, FieldProvider::FileType filetype)
{
    FieldInfo fi;
    fi.filename  = filepath + filename;
    fi.fieldname = field.name;
    fi.filetype = filetype;
    fi.id = field.id;
    for(auto &sf : field.subfields){
        fi.subfields.emplace_back( SubfieldInfo() );
        fi.subfields.back().id = sf.id;
        fi.subfields.back().numRefLines = sf.reference_lines.size();
    }

    return fi;

}

}
