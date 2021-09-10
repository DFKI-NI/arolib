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
 
#ifndef AROLIB_MACHINEDB_H
#define AROLIB_MACHINEDB_H
#include <fstream>
#include <vector>
#include <map>
#include <dirent.h>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"
#include "arolib/types/machine.hpp"
#include "arolib/io/io_xml.hpp"
#include "arolib/io/io_common.hpp"
#include "arolib/misc/basic_responses.h"

namespace arolib {

/**
 * @brief Class used to load machines from files
 */
class MachineDB : public LoggingComponent
{
public:

    /**
     * @brief Tags for the input parameters
     */
    enum EntryTag{
        TAG_MANUFACTURER = 0,
        TAG_MODEL,
        TAG_WIDTH,
        TAG_LENGTH,
        TAG_HEIGHT,
        TAG_WEIGHT,
        TAG_BUNKER_MASS,
        TAG_BUNKER_VOLUME,
        TAG_WORKING_WIDTH,
        TAG_MAX_SPEED_EMPTY,
        TAG_MAX_SPEED_FULL,
        TAG_DEF_WORKING_SPEED,
        TAG_NUM_AXIS,
        TAG_TURNING_RADIUS,
        TAG_AXIS_DISTANCE,
        TAG_GAUGE,
        TAG_ENGINE_POWER,
        TAG_KNIFES_COUNT,
        TAG_DOWNLOAD_SIDE,
        TAG_MACHINE_TYPE,
        TAG_MACHINE_ASSIGNMENT
    };

    /**
     * @brief Constructor.
     * @param baseDir Base directory (searches are done from this directory)
     * @param logLevel Log level
     */
    explicit MachineDB(const std::string & baseDir,
                       const LogLevel &logLevel = LogLevel::INFO);

    /**
     * @brief Set base directory.
     * @param baseDir Base directory (searches are done from this directory)
     */
    void setBaseDir(const std::string & baseDir);


    /**
     * @brief Read the files in the base directory.
     */
    void readFiles();


    /**
     * @brief Set the header tags located in the first row of the CSV file corresponding to the machine properties.
     *
     * If a tag for a machine property is not specified, the defualt tag is used
     * @brief headerTags Tags.
     */
    void setCSVHeaderTags(const std::map<EntryTag, std::string>& headerTags);

    /**
     * @brief Set the entries corresponding to required properties.
     *
     * If a property tag is required and not found, the reading of the file will fail
     * @brief requiredEntries Flags stating which properties are required.
     */
    void setRequiredEntries(const std::map<EntryTag, bool>& requiredEntries);

    /**
     * @brief Set the amount of empty lines expected before and after the header line containing the tags.
     * @brief beforeHeaders Amount of empty lines expected beforethe header line containing the tags.
     * @brief afterHeaders Amount of empty lines expected after the header line containing the tags.
     */
    void setCSVEmptyLines(size_t beforeHeaders, size_t afterHeaders);

    /**
     * @brief Set the delimiter used to read CSV files.
     * @brief delimiter CSV delimiter.
     */
    void setCSVFieldDelimiter(char delimiter);

    /**
     * @brief Set the delimiter/separator that was used to separate between primaty and secondary manufacturers in the input TAG_MANUFACTURER.
     * @brief sep Separator
     */
    void setSecondaryManufactuturerSeparator(const std::string &sep = "");

    /**
     * @brief Get the list of (primary) manufacturers.
     * @brief [out] manufacturers List of manufacturers
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp getManufacturers(std::vector<std::string> &manufacturers);

    /**
     * @brief Get the (filtered) list of models of a given manufacturer.
     * @brief manufacturer manufacturer (if empty -> all manufacturers)
     * @brief [out] models List of models
     * @return AroResp with error id (0:=OK) and message
     */
    arolib::AroResp getModelsFromManufacturer(const std::string & manufacturer,
                                                      std::vector<std::string> &models);

    /**
     * @brief Get the (filtered) list of machines of given machine types, models, and manufacturers.
     * @brief manufacturers Manufacturers filter (if empty -> all manufacturers)
     * @brief models Models filter (if empty -> all models)
     * @brief types Machine types filter (if empty -> all types)
     * @brief [out] machines List of machines
     * @return AroResp with error id (0:=OK) and message
     */
    arolib::AroResp getMachines(const std::set<std::string> & manufacturers, //empty = all manufacturers
                                      const std::set<std::string> & models, //empty = all models
                                      const std::set<Machine::MachineType> & types, //empty = all types
                                      std::vector<Machine> &machines);

    /**
     * @brief Get the machine with the given model and manufacturer.
     * @brief manufacturer Manufacturer
     * @brief model Model
     * @brief [out] machine Machine
     * @return AroResp with error id (0:=OK) and message
     */
    arolib::AroResp getMachine(const std::string & manufacturer,
                                     const std::string & model,
                                     Machine &machine);

    /**
     * @brief Get all available machines.
     * @brief manufacturer Manufacturer
     * @brief [out] machines List of machines
     * @return AroResp with error id (0:=OK) and message
     */
    arolib::AroResp getAllMachines(std::vector<Machine> &machines);

    /**
     * @brief Print the list of all available machines.
     */
    void printList();

    /**
     * @brief Save the list of all available machines in an xml file.
     * @brief filename File name/path
     * @return True on success
     */
    bool saveListXML(const std::string& filename);
    

protected:
    /**
     * @brief Reas machines from the CSV files in the base directory.
     */
    void readCSVfiles();

    /**
     * @brief Parse the list of machines from the CSV file.
     * @brief filename File name/path
     */
    void parseCSV(std::string filename);

    /**
     * @brief Parse the list of machines from the CSV file.
     * @brief filename File name/path
     */
    void parseCSVOld(std::string filename);

    /**
     * @brief Parse the number of axis from a string.
     * @brief val String value
     * @return Number of axis
     */
    int string2numAxis(std::string val);

protected:
    std::string m_baseDir; /**< Base directory */
    std::map<std::string, std::map< std::string, Machine > > m_machineDB; /**< List of machines: map< manufacturer , map<model, machine> > */
    std::map<int, std::string> m_CSVHeaderTags; /**< CSV header tags */
    std::map<std::string, std::vector<int>> m_CSVHeaderTags_rev; /**< CSV header tags (reverse map) */
    std::map<int, bool> m_requiredEntries; /**< Required entries/properties */
    size_t m_CSVEmptyLinesBeforeHeaders = 0; /**< Amount of empty lines expected before the header line containing the tags */
    size_t m_CSVEmptyLinesAfterHeaders = 0; /**< Amount of empty lines expected after the header line containing the tags */
    char m_CSVFieldDelimiter = ','; /**< Delimiter used to read CSV files */
    std::string m_secondaryManufactuturerSeparator = ""; /**< Delimiter/separator that was used to separate between primaty and secondary manufacturers in the input TAG_MANUFACTURER */

    const EntryTag m_lastTag = TAG_MACHINE_ASSIGNMENT; /**< Last tag */
};

}

#endif // AROLIB_MACHINEDB_H
