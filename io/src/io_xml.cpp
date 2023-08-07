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
 
#include "arolib/io/io_xml.hpp"
#include "arolib/io/io_common.hpp"

#include <sstream>
#include <iterator>
#include <stdlib.h>

#include "arolib/types/coordtransformer.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/optional/optional.hpp>

namespace arolib {
namespace io {

const LogLevel defLogLevel = LogLevel::WARNING;

bool writeFieldXML(const std::string &filename, const Field &field, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out, const OutFieldInfo &outFieldInfo)
{
    Logger logger(defLogLevel);
    if (!AroXMLOutDocument::saveField(filename, field, coordinatesType_in, coordinatesType_out, outFieldInfo)) {
        logger.printError( __FUNCTION__, "Error saving field");
        return false;
    }
    return true;
}

bool writeFieldsXML(const std::string &filename, const std::vector<Field> &fields, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);
    if (!AroXMLOutDocument::saveFields(filename, fields, coordinatesType_in, coordinatesType_out)) {
        logger.printError( __FUNCTION__, "Error saving fields");
        return false;
    }
    return true;
}

bool readFieldXML(const std::string &filename, Field &field, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);
    if (!AroXMLInDocument::readField(filename, field, coordinatesType_out, {}, defLogLevel)) {
        logger.printError( __FUNCTION__, "Error reading field");
        return false;
    }
    return true;
}

bool readFieldsXML(const std::string &filename, std::vector<Field> &fields, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);
    if (!AroXMLInDocument::readFields(filename, fields, coordinatesType_out, {}, defLogLevel)) {
        logger.printError( __FUNCTION__, "Error reading fields");
        return false;
    }
    return true;
}


bool writeMachinesXML(const std::string &filename,
                      const std::vector<Machine> &machines)
{

    Logger logger(defLogLevel);

    if (!AroXMLOutDocument::saveMachines(filename, machines)) {
        logger.printError( __FUNCTION__, "Error saving machines");
        return false;
    }
    return true;
}

bool readMachinesXML(const std::string &filename, std::vector<Machine> &machines)
{
    Logger logger(defLogLevel);

    if (!AroXMLInDocument::readMachines(filename, machines, {}, defLogLevel)) {
        logger.printError( __FUNCTION__, "Error reading machines");
        return false;
    }
    return true;
}


bool writeConfigParametersXML(const std::string &filename, const std::map<std::string, std::map<std::string, std::string> > &configParameters)
{
    Logger logger(defLogLevel);
    if (!AroXMLOutDocument::saveConfigParameters(filename, configParameters)) {
        logger.printError( __FUNCTION__, "Error saving config parameters");
        return false;
    }
    return true;
}

bool readConfigParametersXML(const std::string &filename, std::map<std::string, std::map<std::string, std::string> > &configParameters)
{
    Logger logger(defLogLevel);
    if (!AroXMLInDocument::readConfigParameters(filename, configParameters, {}, defLogLevel)) {
        logger.printError( __FUNCTION__, "Error reading config parameters");
        return false;
    }
    return true;
}

bool writeOutFieldInfoXML(const std::string &filename, const OutFieldInfo &outFieldInfo)
{
    Logger logger(defLogLevel);

    if (!AroXMLOutDocument::saveOutFieldInfo(filename, outFieldInfo)) {
        logger.printError( __FUNCTION__, "Error saving outFieldInfo");
        return false;
    }
    return true;
}

bool readOutFieldInfoXML(const std::string &filename, OutFieldInfo &outFieldInfo)
{
    Logger logger(defLogLevel);

    if (!AroXMLInDocument::readOutFieldInfo(filename, outFieldInfo, {}, defLogLevel)) {
        logger.printError( __FUNCTION__, "Error reading outFieldInfo");
        return false;
    }
    return true;
}

bool readResourcePoints(const std::string &filename, std::vector<ResourcePoint> &resource_points)
{
    Logger logger(defLogLevel);

    if (!AroXMLInDocument::readResourcePoints(filename, resource_points, {}, defLogLevel)) {
        logger.printError( __FUNCTION__, "Error reading resource points");
        return false;
    }
    return true;
}

bool writePlanParametersXML(const std::string &filename,
                            const Field &field,
                            const std::vector<Machine> &workingGroup,
                            const std::map<std::string, std::map<std::string, std::string> >& configParameters,
                            const OutFieldInfo &outFieldInfo,
                            const std::map<MachineId_t, MachineDynamicInfo> &machinesDynamicInfo,
                            const std::map<std::string, const ArolibGrid_t *> gridmaps,
                            Point::ProjectionType coordinatesType_in,
                            Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);


    if (!AroXMLOutDocument::savePlanParameters(filename,
                                               field,
                                               workingGroup,
                                               configParameters,
                                               outFieldInfo,
                                               machinesDynamicInfo,
                                               gridmaps,
                                               coordinatesType_in,
                                               coordinatesType_out)) {
        logger.printError( __FUNCTION__, "Error saving plan parameters");
        return false;
    }
    return true;

}

bool writePlanParametersXML(const std::string &filename,
                            const std::vector<Machine> &workingGroup,
                            const std::map<std::string, std::map<std::string, std::string> >& configParameters,
                            const OutFieldInfo &outFieldInfo,
                            const std::map<MachineId_t, MachineDynamicInfo> &machinesDynamicInfo,
                            const std::map<std::string, const ArolibGrid_t *> gridmaps,
                            Point::ProjectionType coordinatesType_in,
                            Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);


    if (!AroXMLOutDocument::savePlanParameters(filename,
                                               workingGroup,
                                               configParameters,
                                               outFieldInfo,
                                               machinesDynamicInfo,
                                               gridmaps,
                                               coordinatesType_in,
                                               coordinatesType_out)) {
        logger.printError( __FUNCTION__, "Error saving plan parameters");
        return false;
    }
    return true;

}

bool writePlanParametersXML(const std::string &filename,
                            const std::vector<Machine> &workingGroup,
                            const std::map<std::string, std::map<std::string, std::string> > &configParameters,
                            const OutFieldInfo &outFieldInfo,
                            const std::map<MachineId_t, MachineDynamicInfo> &machinesDynamicInfo,
                            const std::string &yieldmap_tifBase64,
                            const std::string &drynessmap_tifBase64,
                            const std::string &soilmap_tifBase64,
                            const std::string &remainingAreaMap_tifBase64,
                            Point::ProjectionType coordinatesType_in,
                            Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);


    if (!AroXMLOutDocument::savePlanParameters(filename,
                                workingGroup,
                                configParameters,
                                outFieldInfo,
                                machinesDynamicInfo,
                                yieldmap_tifBase64,
                                drynessmap_tifBase64,
                                soilmap_tifBase64,
                                remainingAreaMap_tifBase64, coordinatesType_in, coordinatesType_out)) {
        logger.printError( __FUNCTION__, "Error saving plan parameters");
        return false;
    }
    return true;
}

bool readPlanParametersXML(const std::string& filename,
                           Field& field,
                           std::vector<Machine>& workingGroup,
                           std::map<std::string, std::map<std::string, std::string> > &configParameters,
                           OutFieldInfo &outFieldInfo,
                           std::map<MachineId_t, MachineDynamicInfo>& machinesDynamicInfo,
                           std::map<std::string, ArolibGrid_t> &gridmaps,
                           Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);


    if (!AroXMLInDocument::readPlanParameters(filename,
                                              field,
                                              workingGroup,
                                              configParameters,
                                              outFieldInfo,
                                              machinesDynamicInfo,
                                              gridmaps,
                                              coordinatesType_out)) {
        logger.printError( __FUNCTION__, "Error reading plan parameters");
        return false;
    }
    return true;

}

bool readPlanParametersXML(const std::string& filename,
                           std::vector<Machine>& workingGroup,
                           std::map<std::string, std::map<std::string, std::string> > &configParameters,
                           OutFieldInfo &outFieldInfo,
                           std::map<MachineId_t, MachineDynamicInfo>& machinesDynamicInfo,
                           std::map<std::string, ArolibGrid_t> &gridmaps,
                           Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);


    if (!AroXMLInDocument::readPlanParameters(filename,
                                              workingGroup,
                                              configParameters,
                                              outFieldInfo,
                                              machinesDynamicInfo,
                                              gridmaps,
                                              coordinatesType_out)) {
        logger.printError( __FUNCTION__, "Error reading plan parameters");
        return false;
    }
    return true;

}

bool readPlanParametersXML(const std::string &filename,
                           Field &field,
                           std::vector<Machine> &workingGroup,
                           std::map<std::string, std::map<std::string, std::string> > &configParameters,
                           OutFieldInfo &outFieldInfo,
                           std::map<MachineId_t, MachineDynamicInfo> &machinesDynamicInfo,
                           std::string &yieldmap_tifBase64,
                           std::string &drynessmap_tifBase64,
                           std::string &soilmap_tifBase64,
                           std::string &remainingAreaMap_tifBase64, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);


    if (!AroXMLInDocument::readPlanParameters(filename,
                                field,
                                workingGroup,
                                configParameters,
                                outFieldInfo,
                                machinesDynamicInfo,
                                yieldmap_tifBase64,
                                drynessmap_tifBase64,
                                soilmap_tifBase64,
                                remainingAreaMap_tifBase64,
                                           coordinatesType_out)) {
        logger.printError( __FUNCTION__, "Error reading plan parameters");
        return false;
    }
    return true;

}

bool readPlanParametersXML(const std::string &filename,
                           std::vector<Machine> &workingGroup,
                           std::map<std::string, std::map<std::string, std::string> > &configParameters,
                           OutFieldInfo &outFieldInfo,
                           std::map<MachineId_t, MachineDynamicInfo> &machinesDynamicInfo,
                           std::string &yieldmap_tifBase64,
                           std::string &drynessmap_tifBase64,
                           std::string &soilmap_tifBase64,
                           std::string &remainingAreaMap_tifBase64, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);


    if (!AroXMLInDocument::readPlanParameters(filename,
                                workingGroup,
                                configParameters,
                                outFieldInfo,
                                machinesDynamicInfo,
                                yieldmap_tifBase64,
                                drynessmap_tifBase64,
                                soilmap_tifBase64,
                                remainingAreaMap_tifBase64,
                                           coordinatesType_out)) {
        logger.printError( __FUNCTION__, "Error reading plan parameters");
        return false;
    }
    return true;
}

bool writePlanXML(const std::string &filename,
                  const std::map<int, std::vector<Route> > &routes, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);


    if (!AroXMLOutDocument::savePlan(filename, routes, coordinatesType_in, coordinatesType_out)) {
        logger.printError( __FUNCTION__, "Error saving plan");
        return false;
    }
    return true;

}


bool writePlanXML(const std::string &filename, const std::vector<std::vector<Route> > &routes, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);


    if (!AroXMLOutDocument::savePlan(filename, routes, coordinatesType_in, coordinatesType_out)) {
        logger.printError( __FUNCTION__, "Error saving plan");
        return false;
    }
    return true;
}

bool writePlanXML(const std::string &filename, const Field &field, const std::vector<Machine> &workingGroup, const std::map<int, std::vector<Route> > &routes, const std::string &yieldmap_tifBase64, const std::string &drynessmap_tifBase64, const std::string &soilmap_tifBase64, const std::string &remainingAreaMap_tifBase64, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);


    if (!AroXMLOutDocument::savePlan(filename,
                      field,
                      workingGroup,
                      routes,
                      yieldmap_tifBase64,
                      drynessmap_tifBase64,
                      soilmap_tifBase64,
                      remainingAreaMap_tifBase64, coordinatesType_in, coordinatesType_out)) {
        logger.printError( __FUNCTION__, "Error saving plan");
        return false;
    }
    return true;

}

bool writePlanXML(const std::string &filename, const Field &field, const std::vector<Machine> &workingGroup, const std::vector<std::vector<Route> > &routes, const std::string &yieldmap_tifBase64, const std::string &drynessmap_tifBase64, const std::string &soilmap_tifBase64, const std::string &remainingAreaMap_tifBase64, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);


    if (!AroXMLOutDocument::savePlan(filename,
                                     field,
                                     workingGroup,
                                     routes,
                                     yieldmap_tifBase64,
                                     drynessmap_tifBase64,
                                     soilmap_tifBase64,
                                     remainingAreaMap_tifBase64, coordinatesType_in, coordinatesType_out)) {
        logger.printError( __FUNCTION__, "Error saving plan");
        return false;
    }
    return true;
}


bool writePlanXML(const std::string &filename, const Field &field, const std::vector<Machine> &workingGroup, const std::map<int, std::vector<Route> > &routes, const std::map<std::string, ArolibGrid_t*> &gridmaps, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);


    if (!AroXMLOutDocument::savePlan(filename,
                                     field,
                                     workingGroup,
                                     routes,
                                     gridmaps,
                                     coordinatesType_in,
                                     coordinatesType_out)) {
        logger.printError( __FUNCTION__, "Error saving plan");
        return false;
    }
    return true;

}

bool writePlanXML(const std::string &filename, const Field &field, const std::vector<Machine> &workingGroup, const std::vector<std::vector<Route> > &routes, const std::map<std::string, ArolibGrid_t*> &gridmaps, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);


    if (!AroXMLOutDocument::savePlan(filename,
                                     field,
                                     workingGroup,
                                     routes,
                                     gridmaps,
                                     coordinatesType_in,
                                     coordinatesType_out)) {
        logger.printError( __FUNCTION__, "Error saving plan");
        return false;
    }
    return true;

}

bool readPlanXML(const std::string &filename,
                 std::map<int, std::vector<Route> > &routes,
                 bool syncRoutes, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);

    if (!AroXMLInDocument::readPlan(filename,
                                    routes,
                                    syncRoutes,
                                    coordinatesType_out,
                                    {},
                                    defLogLevel)) {
        logger.printError( __FUNCTION__, "Error reading plan");
        return false;
    }
    return true;
}

bool readPlanXML(const std::string &filename, Field &field, std::vector<Machine> &workingGroup, std::map<int, std::vector<Route> > &routes, std::string &yieldmap_tifBase64, std::string &drynessmap_tifBase64, std::string &soilmap_tifBase64, std::string &remainingAreaMap_tifBase64, bool syncRoutes, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);


    if (!AroXMLInDocument::readPlan(filename,
                      field,
                      workingGroup,
                      routes,
                      yieldmap_tifBase64,
                      drynessmap_tifBase64,
                      soilmap_tifBase64,
                      remainingAreaMap_tifBase64,
                      syncRoutes,
                                 coordinatesType_out,
                      {},
                      defLogLevel)) {
        logger.printError( __FUNCTION__, "Error reading plan");
        return false;
    }
    return true;

}

bool readPlanXML(const std::string &filename, Field &field, std::vector<Machine> &workingGroup, std::map<int, std::vector<Route> > &routes, std::map<std::string, ArolibGrid_t> &gridmaps, bool syncRoutes, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);


    if (!AroXMLInDocument::readPlan(filename,
                                    field,
                                    workingGroup,
                                    routes,
                                    gridmaps,
                                    syncRoutes,
                                    coordinatesType_out,
                                    {},
                                    defLogLevel)) {
        logger.printError( __FUNCTION__, "Error reading plan");
        return false;
    }
    return true;

}

bool writeGraphXML(const std::string &filename, const DirectedGraph::Graph &graph, bool with_meta, Point::ProjectionType coordinatesType_in, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);
    if (!AroXMLOutDocument::saveGraph(filename,
                                      graph, with_meta,
                                      coordinatesType_in, coordinatesType_out)) {
        logger.printError( __FUNCTION__, "Error saving graph");
        return false;
    }
    return true;
}

bool readGraphXML(const std::string &filename, DirectedGraph::Graph &graph, Point::ProjectionType coordinatesType_out)
{
    Logger logger(defLogLevel);

    if (!AroXMLInDocument::readGraph(filename,
                                     graph,
                                     coordinatesType_out,
                                     {},
                                     defLogLevel)) {
        logger.printError( __FUNCTION__, "Error reading graph");
        return false;
    }
    return true;

}


}

}
