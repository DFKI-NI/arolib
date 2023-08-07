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
 
#include "arolib/components/machinedb.h"

namespace arolib {

using namespace arolib::io;

MachineDB::MachineDB(const std::string &baseDir, const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_baseDir(baseDir)
{
    if(!baseDir.empty() && baseDir.rfind("/") != baseDir.size()-1)
        m_baseDir += "/";

    setCSVHeaderTags({ {TAG_MANUFACTURER, "manufacturer"},
                       {TAG_MODEL, "model"},
                       {TAG_WIDTH, "width"},
                       {TAG_LENGTH, "length"},
                       {TAG_HEIGHT, "height"},
                       {TAG_WEIGHT, "weight"},
                       {TAG_BUNKER_MASS, "bunker mass"},
                       {TAG_BUNKER_VOLUME, "bunker volume"},
                       {TAG_WORKING_WIDTH, "working width"},
                       {TAG_MAX_SPEED_EMPTY, "max speed empty"},
                       {TAG_MAX_SPEED_FULL, "max speed full"},
                       {TAG_DEF_WORKING_SPEED, "def working speed"},
                       {TAG_UNLOADING_SPEED_MASS, "unloading speed mass"},
                       {TAG_UNLOADING_SPEED_VOLUME, "unloading speed volume"},
                       {TAG_TURNING_RADIUS, "turning radius"},
                       {TAG_NUM_AXIS, "num axis"},
                       {TAG_AXIS_DISTANCE, "axis distance"},
                       {TAG_GAUGE, "gauge"},
                       {TAG_ENGINE_POWER, "engine power"},
                       {TAG_KNIFES_COUNT, "knifes count"},
                       {TAG_DOWNLOAD_SIDE, "unload_sides"},
                       {TAG_MACHINE_TYPE, "machine type"},
                       {TAG_MACHINE_ASSIGNMENT, "machine assignment"}  });

    setRequiredEntries( { {TAG_MANUFACTURER, true},
                          {TAG_MODEL, true},
                          {TAG_WIDTH, true},
                          {TAG_LENGTH, false},
                          {TAG_HEIGHT, false},
                          {TAG_WEIGHT, true},
                          {TAG_BUNKER_MASS, true},
                          {TAG_BUNKER_VOLUME, false},
                          {TAG_WORKING_WIDTH, true},
                          {TAG_MAX_SPEED_EMPTY, true},
                          {TAG_MAX_SPEED_FULL, true},
                          {TAG_DEF_WORKING_SPEED, true},
                          {TAG_UNLOADING_SPEED_MASS, false},
                          {TAG_UNLOADING_SPEED_VOLUME, false},
                          {TAG_TURNING_RADIUS, false},
                          {TAG_NUM_AXIS, false},
                          {TAG_AXIS_DISTANCE, false},
                          {TAG_GAUGE, false},
                          {TAG_ENGINE_POWER, false},
                          {TAG_KNIFES_COUNT, false},
                          {TAG_DOWNLOAD_SIDE, false},
                          {TAG_MACHINE_TYPE, true},
                          {TAG_MACHINE_ASSIGNMENT, false} });
}

void MachineDB::setBaseDir(const std::string &baseDir)
{
    m_baseDir = baseDir;
    if(!baseDir.empty() && baseDir.rfind("/") != baseDir.size()-1)
        m_baseDir += "/";
}

void MachineDB::readFiles()
{
    m_machineDB.clear();

    readCSVfiles();
}

void MachineDB::setCSVHeaderTags(const std::map<MachineDB::EntryTag, std::string> &headerTags)
{
    for(auto & it : headerTags){
        if(!it.second.empty()){
            m_CSVHeaderTags[it.first] = it.second;
        }
    }

    m_CSVHeaderTags_rev.clear();
    for(auto & it : m_CSVHeaderTags)
        m_CSVHeaderTags_rev[it.second].emplace_back(it.first);
}

void MachineDB::setRequiredEntries(const std::map<MachineDB::EntryTag, bool> &requiredEntries)
{
    for(auto & it : requiredEntries)
        m_requiredEntries[it.first] = it.second;

}

void MachineDB::setCSVEmptyLines(size_t beforeHeaders, size_t afterHeaders)
{
    m_CSVEmptyLinesBeforeHeaders = beforeHeaders;
    m_CSVEmptyLinesAfterHeaders = afterHeaders;
}

void MachineDB::setCSVFieldDelimiter(char delimiter)
{
    m_CSVFieldDelimiter = delimiter;
}

void MachineDB::setSecondaryManufactuturerSeparator(const std::string &sep)
{
    m_secondaryManufactuturerSeparator = sep;
}

AroResp MachineDB::getManufacturers(std::vector<std::string> &manufacturers) {
    manufacturers.clear();
    for (auto iterMap : m_machineDB)
        manufacturers.push_back(iterMap.first);
    return AroResp(0, "OK");
}

AroResp MachineDB::getModelsFromManufacturer(const std::string &manufacturer, std::vector<std::string> &models) {
    models.clear();

    if(manufacturer.empty()){
        for(auto& it : m_machineDB){
            for(auto& it2 : it.second)
                models.emplace_back(it2.first);
        }
    }
    else{
        auto iterMap = m_machineDB.find(manufacturer);
        if (iterMap == m_machineDB.end())
            return AroResp(0, "No machines of manufacturer '" + manufacturer + "' found in the DB.");

        for (auto it : iterMap->second)
            models.emplace_back(it.first);
    }

    if(models.empty())
        return AroResp(0, "No machines of manufacturer '" + manufacturer + "' found in the DB.");

    return AroResp(0, "OK");
}

AroResp MachineDB::getMachines(const std::set<std::string> &manufacturers,
                                     const std::set<std::string> &models,
                                     const std::set<Machine::MachineType> &types,
                                     std::vector<Machine> &machines)
{
    machines.clear();
    for(auto& it_man : m_machineDB){
        if(manufacturers.empty() || manufacturers.find(it_man.first) != manufacturers.end()){
            for(auto& it_mod : it_man.second){
                if(models.empty() || models.find(it_mod.first) != models.end()){
                    if(types.empty() || types.find(it_mod.second.machinetype) != types.end()){
                        machines.emplace_back(it_mod.second);
                    }
                }
            }
        }
    }
    return AroResp(0, "OK");
}

AroResp MachineDB::getMachine(const std::string &manufacturer, const std::string &model, Machine &machine) {

    auto it1 = m_machineDB.find(manufacturer);
    if (it1 == m_machineDB.end())
        return AroResp(1, "No machines of manufacturer '" + manufacturer + "' found in the DB.");

    auto it2 = it1->second.find(model);
    if(it2 == it1->second.end())
        return AroResp(1, "No machine with manufacturer '" + manufacturer +
                             "' and type '" + model + "' found in the DB.");
    machine = it2->second;
    return AroResp(0, "OK");

}

AroResp MachineDB::getAllMachines(std::vector<Machine> &machines)
{
    machines.clear();
    for(auto &it1 : m_machineDB){
        for(auto &it2 : it1.second){
            machines.push_back(it2.second);
        }
    }

    return AroResp(0, "OK");
}

void MachineDB::printList()
{
    std::cout << std::endl << "*************** MACHINES ***************" << std::endl;
    for (auto it1 : m_machineDB) {
        std::cout << "  " << it1.first << std::endl;
        for (auto it2 : it1.second ) {
            std::cout << "  ::  " << it2.first << std::endl;
        }
    }
    std::cout << "*************** MACHINES ***************" << std::endl << std::endl;

}

bool MachineDB::saveListXML(const std::string &filename)
{
    std::vector<Machine> machines;
    for (auto it1 : m_machineDB) {
        for (auto it2 : it1.second )
            machines.push_back(it2.second);
    }

    if(!writeMachinesXML(filename, machines) ){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error saving list of machines in " + filename);
        return false;
    }
    logger().printOut(LogLevel::INFO, __FUNCTION__, "List of machines saved in " + filename);
    return true;
}

void MachineDB::readCSVfiles()
{

    std::vector<std::string> filelist;
    std::string pattern = ".csv";

    struct dirent *pDirent;
    DIR *pDir;
    pDir = opendir(m_baseDir.c_str());
    if (pDir == NULL)
        return;

    while ((pDirent = readdir(pDir)) != NULL) {
        std::string filename = pDirent->d_name;
        if(filename.rfind(pattern) == filename.size() - pattern.size())
            filelist.push_back(filename);
    }
    closedir (pDir);
    std::sort(filelist.begin(), filelist.end());

    for(auto& filename : filelist)
        parseCSV(m_baseDir + filename);


}
void MachineDB::parseCSV(std::string filename) {

    typedef boost::tokenizer< boost::escaped_list_separator<char> , std::string::const_iterator, std::string> Tokenizer;
    boost::escaped_list_separator<char> seps(('\\', m_CSVFieldDelimiter, '\\'));
    std::string line;

    logger().printOut(LogLevel::INFO, __FUNCTION__, "Reading " + filename + "...");

    std::ifstream infile(filename);
    if(!infile.is_open()){
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Unable to open file " + filename);
        return;
    }

    int lineCount = 1;

    for(size_t i = 0 ; i < m_CSVEmptyLinesBeforeHeaders ; i++){
        if(!std::getline(infile, line)){
            infile.close();
            return;
        }
        ++lineCount;
    }

    if(!std::getline(infile, line)){
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Unable to read headers in file " + filename);
        return;
    }
    ++lineCount;

    std::map<int, std::vector<int>> mapIndex2Tags;
    std::vector<int> mapTag2index (m_lastTag+1, -1);

    //get tags indexes
    {
        Tokenizer tok(line, seps);

//        std::string delimiter;
//        delimiter.push_back(m_CSVFieldDelimiter);
//        std::vector<std::string> tok;
//        boost::split(tok, line, boost::is_any_of(delimiter));


        int idx = 0;
        for (auto tag : tok) {
            auto it_tag = m_CSVHeaderTags_rev.find(tag);
            if(it_tag != m_CSVHeaderTags_rev.end()){
                for(auto &tag2 : it_tag->second){
                    mapTag2index.at(tag2) = idx;
                    mapIndex2Tags[idx].emplace_back(tag2);
                }
            }
            ++idx;
        }

        for(int i = 0 ; i < mapTag2index.size() ; i++){
            if( mapTag2index.at(i) < 0 && m_requiredEntries.at(i) ){
                logger().printOut(LogLevel::WARNING, __FUNCTION__, "Not all required header entries are available in file " + filename + ": Tag \"" + m_CSVHeaderTags[i] + "\" is missing.");
                return;
            }
        }
    }

    for(size_t i = 0 ; i < m_CSVEmptyLinesAfterHeaders ; i++){
        if(!std::getline(infile, line)){
            infile.close();
            return;
        }
        ++lineCount;
    }

    --lineCount;
    int machineCount = 0;

    while (std::getline(infile, line))
    {
        ++lineCount;

        if (line.size() == 0) continue;
        if (line[0] == '#') continue;

        Tokenizer tok(line, seps);

//        std::string delimiter;
//        delimiter.push_back(m_CSVFieldDelimiter);
//        std::vector<std::string> tok;
//        boost::split(tok, line, boost::is_any_of(m_CSVFieldDelimiter));

        Machine m;
        m.id_intern = -1;
        m.machineassignment = Machine::HEADLAND_INFIELD;
        m.unload_sides = Machine::DefaultDownloadSides;

        bool ok = true;

        std::vector<std::string> values(m_lastTag+1);

        size_t idx = 0;
        try{
            for (auto val : tok) {
                auto it1 = mapIndex2Tags.find(idx);
                if (it1 != mapIndex2Tags.end()){
                    for(auto & tag : it1->second)
                        values.at(tag) = val;
                }
                ++idx;
            }
        }
        catch(std::exception &e){
            logger().printOut(LogLevel::WARNING, __FUNCTION__, "Line " + std::to_string(lineCount) + ", index " + std::to_string(idx) + ": exception = " + e.what() );
            ok = false;
            break;
        }

        for(size_t i = 0 ; i < values.size() ; ++i){
            std::string& val = values.at(i);
            boost::trim(val);
            if(val.empty() && m_requiredEntries.at(i) ){
                logger().printOut(LogLevel::WARNING, __FUNCTION__, "Line " + std::to_string(lineCount) + ": required entry " + m_CSVHeaderTags[i] + " is missing");
                ok = false;
                break;
            }
            else if(!val.empty()){
                try {
                    if(i == TAG_MANUFACTURER){
                        auto ind = val.find(m_secondaryManufactuturerSeparator);
                        if(ind == 0 || ind == std::string::npos)
                            m.manufacturer = val;
                        else
                            m.manufacturer = val.substr(0, ind);
                        boost::trim(m.manufacturer);
                    }
                    else if(i == TAG_MODEL){
                        m.model = val;
                    }
                    else if(i == TAG_WIDTH){
                        m.width = string2double(val);
                    }
                    else if(i == TAG_LENGTH){
                        m.length = string2double(val);
                    }
                    else if(i == TAG_HEIGHT){
                        m.height = string2double(val);
                    }
                    else if(i == TAG_WEIGHT){
                        m.weight = string2double(val);
                    }
                    else if(i == TAG_BUNKER_MASS){
                        m.bunker_mass = string2double(val);
                    }
                    else if(i == TAG_BUNKER_VOLUME){
                        m.bunker_volume = string2double(val);
                    }
                    else if(i == TAG_WORKING_WIDTH){
                        m.working_width = string2double(val);
                    }
                    else if(i == TAG_MAX_SPEED_EMPTY){
                        m.max_speed_empty = string2double(val);
                    }
                    else if(i == TAG_MAX_SPEED_FULL){
                        m.max_speed_full = string2double(val);
                    }
                    else if(i == TAG_DEF_WORKING_SPEED){
                        m.def_working_speed = string2double(val);
                    }
                    else if(i == TAG_UNLOADING_SPEED_MASS){
                        m.unloading_speed_mass = string2double(val);
                    }
                    else if(i == TAG_UNLOADING_SPEED_VOLUME){
                        m.unloading_speed_volume = string2double(val);
                    }
                    else if(i == TAG_TURNING_RADIUS){
                        m.turning_radius = string2double(val);
                    }
                    else if(i == TAG_NUM_AXIS){
                        m.num_axis = string2numAxis(val);
                    }
                    else if(i == TAG_AXIS_DISTANCE){
                        m.axis_distance = string2double(val);
                    }
                    else if(i == TAG_GAUGE){
                        m.gauge = string2double(val);
                    }
                    else if(i == TAG_ENGINE_POWER){
                        m.engine_power = string2double(val);
                    }
                    else if(i == TAG_KNIFES_COUNT){
                        m.knifes_count = string2double(val);
                    }
                    else if(i == TAG_DOWNLOAD_SIDE){
                        m.unload_sides = 0;
                        std::transform( val.begin(), val.end(), val.begin(),
                                        [](unsigned char c){ return std::toupper(c); } );
                        if (val.find("R") != std::string::npos )
                            m.unload_sides |= Machine::WorkingSide_RIGHT;
                        if (val.find("L") != std::string::npos )
                            m.unload_sides |= Machine::WorkingSide_LEFT;
                        if (val.find("B") != std::string::npos )
                            m.unload_sides |= Machine::WorkingSide_BACK;
                        if (val.find("F") != std::string::npos )
                            m.unload_sides |= Machine::WorkingSide_FRONT;
                    }
                    else if(i == TAG_MACHINE_TYPE){
                        std::transform( val.begin(), val.end(), val.begin(),
                                        [](unsigned char c){ return std::tolower(c); } );
                        if ( val.find("olv") != std::string::npos
                              || val.find("tv") != std::string::npos
                              || val.find("anhaenger") != std::string::npos )
                            m.machinetype = Machine::OLV;

                        else if ( val.find("harvester") != std::string::npos
                                 || val.find("haecksler") != std::string::npos
                                  || val.find("maehdrescher") != std::string::npos
                                  || val.find("ruebenroder") != std::string::npos )
                            m.machinetype = Machine::HARVESTER;

                        else if ( val.find("sower") != std::string::npos
                                 || val.find("seeder") != std::string::npos
                                  || val.find("einzelkornsaegeraet") != std::string::npos )
                            m.machinetype = Machine::SOWER;

                        else if ( val.find("sprayer") != std::string::npos
                                  || val.find("fertilizer") != std::string::npos
                                  || val.find("manure") != std::string::npos
                                  || val.find("spritze") != std::string::npos
                                  || val.find("duenger") != std::string::npos
                                  || val.find("guelle") != std::string::npos )
                            m.machinetype = Machine::SPRAYER;

                        else if ( val.find("cultivator") != std::string::npos
                                  || val.find("grubber") != std::string::npos )
                            m.machinetype = Machine::CULTIVATOR;

                        else if ( val.find("plough") != std::string::npos
                                  || val.find("harrow") != std::string::npos
                                  || val.find("volldrehpflug") != std::string::npos
                                  || val.find("kurzscheibenegge") != std::string::npos )
                            m.machinetype = Machine::PLOUGH;

                        else if (val.find("scanner") != std::string::npos)
                            m.machinetype = Machine::SCANNER;

                        else  {
                            m.machinetype = Machine::UNDEFINED_TYPE;
                            logger().printOut(LogLevel::WARNING, __FUNCTION__, "Line " + std::to_string(lineCount) + ": unknown machine type " + val);
                        }

                    }
                    else if(i == TAG_MACHINE_ASSIGNMENT){
                        m.machineassignment = Machine::intToMachineAssignment( std::stoi(val) );
                    }
                }
                catch (std::exception &e) {
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid argument '" + m_CSVHeaderTags[i] + "' in line " + std::to_string(lineCount) + ": " + e.what());
                    if(m_requiredEntries.at( i )){
                        ok = false;
                        break;
                    }
                }

            }
        }

        if(!ok)
            continue;

        auto it1 = m_machineDB.find(m.manufacturer);
        if(it1 != m_machineDB.end()){
            auto it2 = it1->second.find(m.model);
            if(it2 != it1->second.end()){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Line " + std::to_string(lineCount)
                         + ": a machine from manufacturer '" + m.manufacturer + "' and model '" + m.model + "' already exists in the DB. The new entry will be disregarded");
                continue;
            }
            it1->second[ m.model ] = m;
        }

        m_machineDB[ m.manufacturer ][ m.model ] = m;
        ++machineCount;
    }

    logger().printOut(LogLevel::INFO, __FUNCTION__, std::to_string(machineCount) + " machines read from file.");
}


void MachineDB::parseCSVOld(std::string filename) {
    
    typedef boost::tokenizer< boost::escaped_list_separator<char> , std::string::const_iterator, std::string> Tokenizer;
    boost::escaped_list_separator<char> seps('\\', ',', '\"');
    std::string line;

    logger().printOut(LogLevel::INFO, __FUNCTION__, "Reading " + filename + "...");

    std::ifstream infile(filename);
    std::getline(infile, line); //the first two lines do not contain machine info

    int machineNr = 0;
    while (std::getline(infile, line))
    {
        if (line.size() == 0) continue;
        if (line[0] == '#') continue;

        machineNr++;
        Tokenizer tok(line, seps);
        int idx = 0;
        Machine m;

        m.machineassignment = Machine::MachineAssignment::HEADLAND_INFIELD;

        for (auto i : tok) {
            //std::cout << "TOKEN [" << idx << "]: " << i << std::endl;

            try {
                switch (idx) {
                case 0:
                    if (i.length() == 0) {
                        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Machine #" + std::to_string(machineNr) + ": entry 'id' is empty!");
                        m.id_intern = 0;
                    } else {
                        m.id_intern = std::stoi(i);
                    }
                    break;
                case 1:
                    if (i.length() == 0) {
                        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Machine #" + std::to_string(machineNr) + ": entry 'machine type' is empty!");
                    }

                    else if (i == "Schlepper")    m.machinetype = Machine::OLV;
                    else if (i == "Maehdrescher") m.machinetype = Machine::HARVESTER;
                    else if (i == "Ruebenroder")  m.machinetype = Machine::HARVESTER;
                    else  {
                        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Machine #" + std::to_string(machineNr) + ": unknown machine type " + i);
                    }
                    break;
                case 2:
                    if (i.length() == 0) {
                        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Machine #" + std::to_string(machineNr) + ": entry 'manufacturer' is empty!");
                    }
                    m.manufacturer = i;
                    break;
                case 3:
                    if (i.length() == 0) {
                        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Machine #" + std::to_string(machineNr) + ": entry 'model' is empty!");
                    }
                    m.model = i;
                    break;
                case 4:
                    if (i.length() == 0) {
                        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Machine #" + std::to_string(machineNr) + ": entry 'width' is empty!");
                        m.width = 0;
                    } else {
                        m.width = string2double(i);
                    }
                    break;
                case 5:
                    if (i.length() == 0) {
                        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Machine #" + std::to_string(machineNr) + ": entry 'length' is empty!");
                        m.length = 0;
                    } else {
                        m.length = string2double(i);
                    }
                    break;
                case 6:
                    if (i.length() == 0) {
                        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Machine #" + std::to_string(machineNr) + ": entry 'height' is empty!");
                        m.height = 0;
                    } else {
                        m.height = string2double(i);
                    }
                    break;
                case 7:
                    if (i.length() == 0) {
                        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Machine #" + std::to_string(machineNr) + ": entry 'weight' is empty!");
                        m.weight = 0;
                    } else {
                        m.weight = string2double(i);
                    }
                    break;
                case 8:
                    if (i.length() == 0) {
                        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Machine #" + std::to_string(machineNr) + ": entry 'bunker volume' is empty!");
                        m.bunker_mass = 0;
                    } else {
                        m.bunker_mass = string2double(i);
                    }
                    break;
                case 9:
                    if (i.length() == 0) {
                        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Machine #" + std::to_string(machineNr) + ": entry 'working width' is empty!");
                        m.working_width = 0;
                    } else {
                        m.working_width = string2double(i);
                    }
                    break;
                case 10:
                    if (i.length() == 0) {
                        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Machine #" + std::to_string(machineNr) + ": entry 'working speed' is empty!");
                        m.def_working_speed = 0;
                    } else {
                        m.def_working_speed = string2double(i);
                    }
                    break;
                case 11:
                    if (i.length() == 0) {
                        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Machine #" + std::to_string(machineNr) + ": entry 'max speed' is empty!");
                        m.max_speed_empty = 0;
                        m.max_speed_full = 0;
                    } else {
                        m.max_speed_empty = string2double(i);
                        m.max_speed_full = string2double(i);
                    }
                    break;
                default:
                    break;
                }
            } catch (std::invalid_argument) {
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid argument");
                return;
            }

            idx++;
        }

        auto it1 = m_machineDB.find(m.manufacturer);
        if(it1 != m_machineDB.end()){
            auto it2 = it1->second.find(m.model);
            if(it2 != it1->second.end()){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Machine #" + std::to_string(machineNr)
                         + ": a machine from manufacturer '" + m.manufacturer + "' and model '" + m.model + "' already exists in the DB. The new entry will be disregarded");
                continue;
            }
            it1->second[ m.model ] = m;
        }
        else
            m_machineDB[ m.manufacturer ][ m.model ] = m;

        int machineCount = 0;
        for(auto it : m_machineDB){
            machineCount += it.second.size();
        }
        logger().printOut(LogLevel::INFO, __FUNCTION__, std::to_string(machineCount) + " machines read from file.");
    }
}

int MachineDB::string2numAxis(std::string val)
{
    auto ind = val.find('{');
    while (ind != std::string::npos){
        val.erase( val.begin() + ind );
        ind = val.find('{');
    }
    ind = val.find('}');
    while (ind != std::string::npos){
        val.erase( val.begin() + ind );
        ind = val.find('{');
    }


    std::vector<std::string> vals;
    boost::split(vals, val, boost::is_any_of(","));

    if(vals.empty())
        return -1;

    try{
        return std::stoi(vals.front());
    }
    catch(...){
        return -1;
    }


}

}
