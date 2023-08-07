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
 
#ifndef AROLIB_IO_XML_HPP
#define AROLIB_IO_XML_HPP

#include <string>
#include <iostream>
#include <fstream>
#include <map>
#include <ctime>

#include "arolib/io/aroxmloutdocument.hpp"
#include "arolib/io/aroxmlindocument.hpp"

#include "arolib/types/machine.hpp"
#include "arolib/types/subfield.hpp"
#include "arolib/types/field.hpp"
#include "arolib/types/fieldaccesspoint.hpp"
#include "arolib/types/resourcepoint.hpp"
#include "arolib/types/outfieldinfo.hpp"
#include "arolib/types/machinedynamicinfo.hpp"
#include "arolib/types/route.hpp"
#include "arolib/planning/path_search/directedgraph.hpp"

namespace arolib {
namespace io {


/**
 * @brief Write/save a field in a (arolib-formatted) XML file
 * @param filename File name/path
 * @param field Field to be written/saves
 * @param coordinatesType_in Projection type for the coordinates in the source (field)
 * @param coordinatesType_out Projection type for the coordinates in the target (file)
 * @return true on success
 */
bool writeFieldXML(const std::string& filename,
                   const Field& field,
                   Point::ProjectionType coordinatesType_in = Point::UTM,
                   Point::ProjectionType coordinatesType_out = Point::WGS,
                   const OutFieldInfo &outFieldInfo = OutFieldInfo() );


/**
 * @brief Write/save a set of fields in a (arolib-formatted) XML file
 * @param filename File name/path
 * @param fields Fields to be written/saves
 * @param coordinatesType_in Projection type for the coordinates in the source (field)
 * @param coordinatesType_out Projection type for the coordinates in the target (file)
 * @return true on success
 */
bool writeFieldsXML(const std::string& filename,
                    const std::vector<Field>& fields,
                    Point::ProjectionType coordinatesType_in = Point::UTM,
                    Point::ProjectionType coordinatesType_out = Point::WGS );


/**
 * @brief Read a field from a (arolib-formatted) XML file
 * @param filename File name/path
 * @param [out] field Read field
 * @param coordinatesType_out Projection type for the coordinates in the target (field)
 * @return true on success
 */
bool readFieldXML(const std::string& filename,
                  Field& field,
                  Point::ProjectionType coordinatesType_out = Point::UTM);


/**
 * @brief Read a set of fields from a (arolib-formatted) XML file
 * @param filename File name/path
 * @param [out] fields Read fields
 * @param coordinatesType_out Projection type for the coordinates in the target (field)
 * @return true on success
 */
bool readFieldsXML(const std::string& filename,
                   std::vector<Field>& fields,
                   Point::ProjectionType coordinatesType_out = Point::UTM);



/**
 * @brief Write/save machines in a (arolib-formatted) XML file
 * @param filename File name/path
 * @param machines Machines to be written/saves
 * @return true on success
 */
bool writeMachinesXML(const std::string& filename,
                    const std::vector<Machine>& machines);

/**
 * @brief Read machines from a (arolib-formatted) XML file
 * @param filename File name/path
 * @param [out] machines Read machines
 * @return true on success
 */
bool readMachinesXML(const std::string& filename, std::vector<Machine>& machines);



/**
 * @brief Write/save configuration parameters (given as string-maps) in a (arolib-formatted) XML file
 * @param filename File name/path
 * @param configParameters Configuration parameters (given as string-maps) to be written/saves
 * @return true on success
 */
bool writeConfigParametersXML(const std::string& filename,
                            const std::map<std::string, std::map<std::string, std::string> > &configParameters );

/**
 * @brief Read configuration parameters (given as string-maps) from a (arolib-formatted) XML file
 * @param filename File name/path
 * @param [out] configParameters Read configuration parameters (given as string-maps)
 * @return true on success
 */
bool readConfigParametersXML(const std::string& filename,
                           std::map<std::string, std::map<std::string, std::string> > &configParameters);


/**
 * @brief Write/save OutFieldInfo in a (arolib-formatted) XML file
 * @param filename File name/path
 * @param outFieldInfo OutFieldInfo to be written/saves
 * @return true on success
 */
bool writeOutFieldInfoXML(const std::string& filename,
                         const OutFieldInfo &outFieldInfo );

/**
 * @brief Read OutFieldInfo from a (arolib-formatted) XML file
 * @param filename File name/path
 * @param [out] outFieldInfo Read OutFieldInfo
 * @return true on success
 */
bool readOutFieldInfoXML(const std::string& filename,
                        OutFieldInfo &outFieldInfo);


/**
 * @brief Read resource points from a (arolib-formatted) XML file
 * @param filename File name/path
 * @param [out] resource_points Read resource points
 * @return true on success
 */
bool readResourcePoints(const std::string& filename,
                         std::vector<ResourcePoint> &resource_points);


/**
 * @brief Save plan parameters in a XML file
 * @param filename Filename
 * @param field Field to be written
 * @param workingGroup Working group (machines) to be written
 * @param configParameters Configuration parameters (given as string map) to be written
 * @param OutFieldInfo OutFieldInfo to be written
 * @param machinesDynamicInfo MachineDynamicInfo-map to be written
 * @param coordinatesType_in Projection type of the coordinates in the source
 * @param coordinatesType_out Projection type of the coordinates in the target (file)
 * @return True on success
 */
bool writePlanParametersXML(const std::string& filename,
                            const Field& field,
                            const std::vector<Machine>& workingGroup,
                            const std::map<std::string, std::map<std::string, std::string> > &configParameters,
                            const OutFieldInfo &outFieldInfo = OutFieldInfo(),
                            const std::map<MachineId_t, MachineDynamicInfo>& machinesDynamicInfo = {},
                            const std::map<std::string, const ArolibGrid_t*> gridmaps = {},
                            Point::ProjectionType coordinatesType_in = Point::UTM,
                            Point::ProjectionType coordinatesType_out = Point::WGS);


/**
 * @brief Save plan parameters in a XML file
 * @param filename Filename
 * @param workingGroup Working group (machines) to be written
 * @param configParameters Configuration parameters (given as string map) to be written
 * @param OutFieldInfo OutFieldInfo to be written
 * @param machinesDynamicInfo MachineDynamicInfo-map to be written
 * @param coordinatesType_in Projection type of the coordinates in the source
 * @param coordinatesType_out Projection type of the coordinates in the target (file)
 * @return True on success
 */
bool writePlanParametersXML(const std::string& filename,
                            const std::vector<Machine>& workingGroup,
                            const std::map<std::string, std::map<std::string, std::string> > &configParameters,
                            const OutFieldInfo &outFieldInfo = OutFieldInfo(),
                            const std::map<MachineId_t, MachineDynamicInfo>& machinesDynamicInfo = {},
                            const std::map<std::string, const ArolibGrid_t*> gridmaps = {},
                            Point::ProjectionType coordinatesType_in = Point::UTM,
                            Point::ProjectionType coordinatesType_out = Point::WGS);

/**
 * @brief Save plan parameters in a XML file
 * @param filename Filename
 * @param workingGroup Working group (machines) to be written
 * @param configParameters Configuration parameters (given as string map) to be written
 * @param OutFieldInfo OutFieldInfo to be written
 * @param machinesDynamicInfo MachineDynamicInfo-map to be written
 * @param yieldmap_tifBase64 Yield-map (base64-encoded) to be written (disregarded if empty)
 * @param drynessmap_tifBase64 Dryness-map (base64-encoded) to be written (disregarded if empty)
 * @param soilmap_tifBase64 Soil-map (base64-encoded) to be written (disregarded if empty)
 * @param remainingAreaMap_tifBase64 Remaining-area-map (base64-encoded) to be written (disregarded if empty)
 * @param coordinatesType_in Projection type of the coordinates in the source
 * @param coordinatesType_out Projection type of the coordinates in the target (file)
 * @return True on success
 */
bool writePlanParametersXML(const std::string& filename,
                          const std::vector<Machine>& workingGroup,
                          const std::map<std::string, std::map<std::string, std::string> > &configParameters,
                          const OutFieldInfo &outFieldInfo = OutFieldInfo(),
                          const std::map<MachineId_t, MachineDynamicInfo>& machinesDynamicInfo = {},
                          const std::string &yieldmap_tifBase64 = "",
                          const std::string &drynessmap_tifBase64 = "",
                          const std::string &soilmap_tifBase64 = "",
                          const std::string &remainingAreaMap_tifBase64 = "",
                          Point::ProjectionType coordinatesType_in = Point::UTM,
                          Point::ProjectionType coordinatesType_out = Point::WGS);

/**
 * @brief Read plan parameters from a (Arolib-formatted) XML file.
 * @param filename Filename
 * @param [out] field Read field
 * @param [out] workingGroup Read working group (machines)
 * @param [out] configParameters Read configuration parameters (as string map)
 * @param [out] outFieldInfo Read OutFieldInfo
 * @param [out] machinesDynamicInfo Read MachineDynamicInfo
 * @param [out] gridmaps Read gridmaps
 * @param coordinatesType_out Projection type for the target coordinates
 * @param parentTags Parent tags to reach the location of the data to be read
 * @param logLevel Log level
 * @return True on success
 */
bool readPlanParametersXML(const std::string& filename,
                           Field& field,
                           std::vector<Machine>& workingGroup,
                           std::map<std::string, std::map<std::string, std::string> > &configParameters,
                           OutFieldInfo &outFieldInfo,
                           std::map<MachineId_t, MachineDynamicInfo>& machinesDynamicInfo,
                           std::map<std::string, ArolibGrid_t> &gridmaps,
                           Point::ProjectionType coordinatesType_out = Point::UTM);

/**
 * @brief Read plan parameters from a (Arolib-formatted) XML file.
 * @param filename Filename
 * @param [out] workingGroup Read working group (machines)
 * @param [out] configParameters Read configuration parameters (as string map)
 * @param [out] outFieldInfo Read OutFieldInfo
 * @param [out] machinesDynamicInfo Read MachineDynamicInfo
 * @param [out] gridmaps Read gridmaps
 * @param coordinatesType_out Projection type for the target coordinates
 * @param parentTags Parent tags to reach the location of the data to be read
 * @param logLevel Log level
 * @return True on success
 */
bool readPlanParametersXML(const std::string& filename,
                           std::vector<Machine>& workingGroup,
                           std::map<std::string, std::map<std::string, std::string> > &configParameters,
                           OutFieldInfo &outFieldInfo,
                           std::map<MachineId_t, MachineDynamicInfo>& machinesDynamicInfo,
                           std::map<std::string, ArolibGrid_t> &gridmaps,
                           Point::ProjectionType coordinatesType_out = Point::UTM);

/**
 * @brief Read plan parameters from a (Arolib-formatted) XML file.
 * @param filename Filename
 * @param [out] field Read field
 * @param [out] workingGroup Read working group (machines)
 * @param [out] configParameters Read configuration parameters (as string map)
 * @param [out] outFieldInfo Read OutFieldInfo
 * @param [out] machinesDynamicInfo Read MachineDynamicInfo
 * @param [out] yieldmap_tifBase64 Read yieldmap (base64-encoded)
 * @param [out] drynessmap_tifBase64 Read drynessmap (base64-encoded)
 * @param [out] soilmap_tifBase64 Read soilmap (base64-encoded)
 * @param [out] remainingAreaMap_tifBase64 Read remaining-area map (base64-encoded)
 * @param coordinatesType_out Projection type for the target coordinates
 * @param parentTags Parent tags to reach the location of the data to be read
 * @param logLevel Log level
 * @return True on success
 */
bool readPlanParametersXML(const std::string& filename,
                          Field& field,
                          std::vector<Machine>& workingGroup,
                          std::map<std::string, std::map<std::string, std::string> > &configParameters,
                          OutFieldInfo &outFieldInfo,
                          std::map<MachineId_t, MachineDynamicInfo>& machinesDynamicInfo,
                          std::string &yieldmap_tifBase64,
                          std::string &drynessmap_tifBase64,
                          std::string &soilmap_tifBase64,
                          std::string &remainingAreaMap_tifBase64,
                         Point::ProjectionType coordinatesType_out = Point::UTM);

/**
 * @brief Read plan parameters from a (Arolib-formatted) XML file.
 * @param filename Filename
 * @param [out] workingGroup Read working group (machines)
 * @param [out] configParameters Read configuration parameters (as string map)
 * @param [out] outFieldInfo Read OutFieldInfo
 * @param [out] machinesDynamicInfo Read MachineDynamicInfo
 * @param [out] yieldmap_tifBase64 Read yieldmap (base64-encoded)
 * @param [out] drynessmap_tifBase64 Read drynessmap (base64-encoded)
 * @param [out] soilmap_tifBase64 Read soilmap (base64-encoded)
 * @param [out] remainingAreaMap_tifBase64 Read remaining-area map (base64-encoded)
 * @param coordinatesType_out Projection type for the target coordinates
 * @param parentTags Parent tags to reach the location of the data to be read
 * @param logLevel Log level
 * @return True on success
 */
bool readPlanParametersXML( const std::string& filename,
                          std::vector<Machine>& workingGroup,
                          std::map<std::string, std::map<std::string, std::string> > &configParameters,
                          OutFieldInfo &outFieldInfo,
                          std::map<MachineId_t, MachineDynamicInfo>& machinesDynamicInfo,
                          std::string &yieldmap_tifBase64,
                          std::string &drynessmap_tifBase64,
                          std::string &soilmap_tifBase64,
                          std::string &remainingAreaMap_tifBase64,
                          Point::ProjectionType coordinatesType_out = Point::UTM);

/**
 * @brief Save routes in a XML file
 * @param filename Filename
 * @param routes Routes to be written
 * @param coordinatesType_in Projection type of the coordinates in the source
 * @param coordinatesType_out Projection type of the coordinates in the target (file)
 * @return True on success
 */
bool writePlanXML( const std::string &filename,
                 const std::map<int, std::vector<Route> > &routes,
                 Point::ProjectionType coordinatesType_in = Point::UTM,
                 Point::ProjectionType coordinatesType_out = Point::WGS );

/**
 * @brief Save routes in a XML file
 * @param filename Filename
 * @param routes Routes to be written
 * @param coordinatesType_in Projection type of the coordinates in the source
 * @param coordinatesType_out Projection type of the coordinates in the target (file)
 * @return True on success
 */
bool writePlanXML( const std::string &filename,
                 const std::vector< std::vector<Route> > &routes,
                 Point::ProjectionType coordinatesType_in = Point::UTM,
                 Point::ProjectionType coordinatesType_out = Point::WGS );

/**
 * @brief Save plan in a XML file
 * @param filename Filename
 * @param field Field to be written
 * @param workingGroup Working group (machines) to be written
 * @param routes Routes to be written
 * @param yieldmap_tifBase64 Yield-map (base64-encoded) to be written (disregarded if empty)
 * @param drynessmap_tifBase64 Dryness-map (base64-encoded) to be written (disregarded if empty)
 * @param soilmap_tifBase64 Soil-map (base64-encoded) to be written (disregarded if empty)
 * @param remainingAreaMap_tifBase64 Remaining-area-map (base64-encoded) to be written (disregarded if empty)
 * @param coordinatesType_in Projection type of the coordinates in the source
 * @param coordinatesType_out Projection type of the coordinates in the target (file)
 * @return True on success
 */
bool writePlanXML( const std::string &filename,
                 const Field& field,
                 const std::vector<Machine>& workingGroup,
                 const std::map<int, std::vector<Route> > &routes,
                 const std::string &yieldmap_tifBase64 = "",
                 const std::string &drynessmap_tifBase64 = "",
                 const std::string &soilmap_tifBase64 = "",
                 const std::string &remainingAreaMap_tifBase64 = "",
                 Point::ProjectionType coordinatesType_in = Point::UTM,
                 Point::ProjectionType coordinatesType_out = Point::WGS );

/**
 * @brief Save plan in a XML file
 * @param filename Filename
 * @param field Field to be written
 * @param workingGroup Working group (machines) to be written
 * @param routes Routes to be written
 * @param yieldmap_tifBase64 Yield-map (base64-encoded) to be written (disregarded if empty)
 * @param drynessmap_tifBase64 Dryness-map (base64-encoded) to be written (disregarded if empty)
 * @param soilmap_tifBase64 Soil-map (base64-encoded) to be written (disregarded if empty)
 * @param remainingAreaMap_tifBase64 Remaining-area-map (base64-encoded) to be written (disregarded if empty)
 * @param coordinatesType_in Projection type of the coordinates in the source
 * @param coordinatesType_out Projection type of the coordinates in the target (file)
 * @return True on success
 */
bool writePlanXML( const std::string &filename,
                 const Field& field,
                 const std::vector<Machine>& workingGroup,
                 const std::vector< std::vector<Route> > &routes,
                 const std::string &yieldmap_tifBase64 = "",
                 const std::string &drynessmap_tifBase64 = "",
                 const std::string &soilmap_tifBase64 = "",
                 const std::string &remainingAreaMap_tifBase64 = "",
                 Point::ProjectionType coordinatesType_in = Point::UTM,
                 Point::ProjectionType coordinatesType_out = Point::WGS );

/**
 * @brief Save plan in a XML file
 * @param filename Filename
 * @param field Field to be written
 * @param workingGroup Working group (machines) to be written
 * @param routes Routes to be written
 * @param gridmaps Gridmaps by name
 * @param coordinatesType_in Projection type of the coordinates in the source
 * @param coordinatesType_out Projection type of the coordinates in the target (file)
 * @return True on success
 */
bool writePlanXML( const std::string &filename,
                   const Field& field,
                   const std::vector<Machine>& workingGroup,
                   const std::map< int, std::vector<Route> > &routes,
                   const std::map<std::string, ArolibGrid_t* > &gridmaps,
                   Point::ProjectionType coordinatesType_in = Point::UTM,
                   Point::ProjectionType coordinatesType_out = Point::WGS );

/**
 * @brief Save plan in a XML file
 * @param filename Filename
 * @param field Field to be written
 * @param workingGroup Working group (machines) to be written
 * @param routes Routes to be written
 * @param gridmaps Gridmaps by name
 * @param coordinatesType_in Projection type of the coordinates in the source
 * @param coordinatesType_out Projection type of the coordinates in the target (file)
 * @return True on success
 */
bool writePlanXML( const std::string &filename,
                   const Field& field,
                   const std::vector<Machine>& workingGroup,
                   const std::vector< std::vector<Route> > &routes,
                   const std::map<std::string, ArolibGrid_t* > &gridmaps,
                   Point::ProjectionType coordinatesType_in = Point::UTM,
                   Point::ProjectionType coordinatesType_out = Point::WGS );

/**
 * @brief Read a routes from a (Arolib-formatted) XML file.
 * @param filename Filename
 * @param [out] routes Read routes
 * @param syncRoutes Should the routes's base timestamp be synchronized so that all have the same one?
 * @param coordinatesType_out Projection type for the target coordinates
 * @return True on success
 */
bool readPlanXML( const std::string &filename,
                std::map<int, std::vector<Route> > &routes,
                bool syncRoutes = true,
                Point::ProjectionType coordinatesType_out = Point::UTM);

/**
 * @brief Read plan data from a (Arolib-formatted) XML file.
 * @param filename Filename
 * @param [out] field Read field
 * @param [out] workingGroup Read working group (machines)
 * @param [out] routes Read routes
 * @param [out] yieldmap_tifBase64 Read yieldmap (base64-encoded)
 * @param [out] drynessmap_tifBase64 Read drynessmap (base64-encoded)
 * @param [out] soilmap_tifBase64 Read soilmap (base64-encoded)
 * @param [out] remainingAreaMap_tifBase64 Read remaining-area map (base64-encoded)
 * @param syncRoutes Should the routes's base timestamp be synchronized so that all have the same one?
 * @param coordinatesType_out Projection type for the target coordinates
 * @return True on success
 */
bool readPlanXML( const std::string &filename,
                Field& field,
                std::vector<Machine>& workingGroup,
                std::map<int, std::vector<Route> > &routes,
                std::string &yieldmap_tifBase64,
                std::string &drynessmap_tifBase64,
                std::string &soilmap_tifBase64,
                std::string &remainingAreaMap_tifBase64,
                bool syncRoutes = true,
                Point::ProjectionType coordinatesType_out = Point::UTM );

/**
 * @brief Read plan data from a (Arolib-formatted) XML file.
 * @param filename Filename
 * @param [out] field Read field
 * @param [out] workingGroup Read working group (machines)
 * @param [out] routes Read routes
 * @param [out] gridmaps Gridmaps by name
 * @param syncRoutes Should the routes's base timestamp be synchronized so that all have the same one?
 * @param coordinatesType_out Projection type for the target coordinates
 * @return True on success
 */
bool readPlanXML( const std::string &filename,
                Field& field,
                std::vector<Machine>& workingGroup,
                std::map<int, std::vector<Route> > &routes,
                  std::map<std::string, ArolibGrid_t > &gridmaps,
                bool syncRoutes = true,
                Point::ProjectionType coordinatesType_out = Point::UTM );

/**
 * @brief Save ha Graph in a XML file
 * @param filename Filename
 * @param graph Graph to be written
 * @param coordinatesType_in Projection type of the coordinates in the source
 * @param coordinatesType_out Projection type of the coordinates in the target (file)
 * @return True on success
 */
bool writeGraphXML(const std::string &filename,
                   const DirectedGraph::Graph &graph,
                   bool with_meta = false,
                   Point::ProjectionType coordinatesType_in = Point::UTM,
                   Point::ProjectionType coordinatesType_out = Point::WGS );

/**
 * @brief Read a Graph from a (Arolib-formatted) XML file.
 * @param filename Filename
 * @param [out] graph Read graph
 * @param coordinatesType_out Projection type for the target coordinates
 * @return True on success
 */
bool readGraphXML( const std::string &filename,
                DirectedGraph::Graph &graph,
                Point::ProjectionType coordinatesType_out = Point::UTM );

}
}


#endif //AROLIB_IO_XML_HPP

