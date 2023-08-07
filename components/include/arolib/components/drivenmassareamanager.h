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
 
#ifndef AROLIB_DRIVENMASSAREAMANAGER_H
#define AROLIB_DRIVENMASSAREAMANAGER_H

#include <map>

#include "arolib/misc/basic_responses.h"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/types/field.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/cartography/common.hpp"
#include "arolib/cartography/sharedgridsmanager.hpp"
#include "arolib/geometry/geometry_helper.hpp"

namespace arolib {

/**
 * @brief Class used to generate and update remaining area maps
 */
class DrivenMassAreaManager : public LoggingComponent
{
public:
    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    DrivenMassAreaManager(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Initialized the class with a virgin field. The remaining-area map is created with the field outer boundary.
     * @param field Working field
     * @param cellsize Grid's cell size
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp init(const Field &field, double cellsize);

    /**
     * @brief Initialized the class with a virgin field, computing an appropiate cell size from the machineS' working group. The remaining-area map is created with the field outer boundary.
     * @param field Working field
     * @param machines Working group. The cell size is computed from their working widths
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp init(const Field &field, const std::vector<Machine> &machines);

    /**
     * @brief Initialized the class with a current remaining-area map.
     * @param field Working field
     * @param basemap Current map.
     * @param initMap If true, the map values will be initialized based on the field geometries; otherwise the map will be used as it is.
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp init(const Field &field, const ArolibGrid_t& basemap, bool initMap);

    /**
     * @brief Set shared CellsInfoManager to record shared edge cells data
     * @param cim CellsInfoManager. If null, no recording will be done
     */
    void setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim);

    /**
     * @brief Check if it is ready (i.e. correctly initialized).
     * @return True if ready
     */
    bool isReady() const;

    /**
     * @brief Clear all data
     */
    void clear();

    /**
     * @brief Forget the last machine location.
     * @param machineId Machine id
     */
    void forgetMachineLocation(const MachineId_t &machineId);

    /**
     * @brief Forget the last location of all machines.
     */
    void forgetAllMachineLocations();


    /**
     * @brief Set the default value of the location of the GPS w.r.t. the machine length from the fron of the machine.
     * @param val Value [0, 1]; if <0 the machine length is not taken into account
     * @return True if accepted
     */
    bool setDefaultGPSFrontDisplacement(float val);

    /**
     * @brief Set the value of the location of the GPS w.r.t. the machine length from the fron of the machine.
     * @param val Value [0, 1]; if <0 the machine length is not taken into account
     * @return True if accepted
     */
    bool setMachineGPSFrontDisplacement(MachineId_t machineId, float val);

    /**
     * @brief Set the option to use edge polygon intersection (true) or cells analysys (false) for repeating areas between previous edge and new edge.
     * @param usePolygonIntersection Use edge polygon intersection (true, recommended) or cells analysys (false) for repeating areas between previous edge and new edge.
     */
    void setUsePolygonIntersection(bool usePolygonIntersection);

    /**
     * @brief Updates the grid/map with the given location, machine mass and machine bunke mass (using the edge from the last location, if existent).
     * @param machine Driving machine
     * @param pt New location
     * @param bunker_mass Bunker mass
     * @param be_precise Perform map/grid operations precisely
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp addData(const Machine &machine, const Point& pt, double bunker_mass, bool be_precise = true);


    /**
     * @brief Get the current remaining-area map (if not ready, it is not allocated).
     * @return Current remaining-area map
     */
    gridmap::SharedGridsManager::ConstGridPtr getDrivenMassAreaMap() const;

protected:
    using GridCellsInfoMap_t = std::map<int, std::map<int, float>>;


    /**
     * @brief Struct holding the important data for the map update
     */
    struct Data{
        std::vector<Point> points; /**< Previous driven edge */
        GridCellsInfoMap_t cellsInfoMass; /**< Driven cells info of the previous edge <machineId, <x, y, mass> */
        GridCellsInfoMap_t cellsInfoOverlap; /**< Driven cells info of the previous edge <machineId, <x, y, overlap> */
        Polygon machineProj; /**< Previous machine projection */
        double massTotal = 0; /**< Previous machine projection */
    };

    /**
     * @brief Get the initialized DrivenMassAreaMap.
     * @return Initialized DrivenMassAreaMap
     */
    gridmap::SharedGridsManager::GridPtr initDrivenMassAreaMap();


    /**
     * @brief Convect a vector of GridCellOverlap into a map < x , < y , overlap > >.
     * @return Converted map
     */
    GridCellsInfoMap_t cellsInfoVecToMap(const std::vector<gridmap::GridmapLayout::GridCellOverlap>& vec);


    /**
     * @brief Get the value from a GridCellsInfoMap_t corresponding to a given x, y.
     * @param cellsInfoMap cellsInfoMap
     * @param x x
     * @param y y
     * @param [out] val Value for the given x, y
     * @return True on success
     */
    bool cellsInfoMapGetValue(const GridCellsInfoMap_t& cellsInfoMap, int x, int y, float& val);

    /**
     * @brief Remove the value from a GridCellsInfoMap_t corresponding to a given x, y.
     * @param [in/out] cellsInfoMap cellsInfoMap to be updated
     * @param x x
     * @param y y
     * @param [out] val Value for the given x, y
     */
    void removeCellsInfoFromMap(GridCellsInfoMap_t& cellsInfoMap, int x, int y);

    /**
     * @brief Add new data based on the polygon (machine projection) intersection.
     * @param machine Driving machine
     * @param pt New location
     * @param bunker_mass Bunker mass
     * @param be_precise Perform map/grid operations precisely
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp addData_polygonIntersection(const Machine &machine, const Point& pt, double bunker_mass, bool be_precise = true);


    /**
     * @brief Add new data based on cells analysis.
     * @param machine Driving machine
     * @param pt New location
     * @param bunker_mass Bunker mass
     * @param be_precise Perform map/grid operations precisely
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp addData_cellsAnalysis(const Machine &machine, const Point& pt, double bunker_mass, bool be_precise = true);

protected:

    bool m_ready = false; /**< Is the manager ready? */
    Field m_field; /**< Field */
    gridmap::SharedGridsManager::GridPtr m_drivenMassAreaMap = nullptr; /**< Remaining area map */
    gridmap::SharedGridsManager m_gridsManager; /**< Shared grids manager >*/
    std::map<MachineId_t, Data> m_prevData; /**< Previous data per machine */
    float m_defaultGPSFrontDisplacement = 0.05; /**< Default value of the location of the GPS w.r.t. the machine length from the fron of the machine >*/
    std::map<MachineId_t, float> m_GPSFrontDisplacements; /**< Default value of the location of the GPS w.r.t. the machine length from the fron of the machine >*/
    bool m_usePolygonIntersection = true; /**< Use edge polygon intersection (true, recommended) or cells analysys (false) for repeating areas between previous edge and new edge >*/
    static const std::string DrivenMassAreaMapName; /**< Remainingg-are grid-map name >*/


};

}

#endif // AROLIB_DRIVENMASSAREAMANAGER_H
