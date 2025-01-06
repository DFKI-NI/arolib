/*
 * Copyright 2021-2025 DFKI GmbH
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

#include "arolib/cartography/common.hpp"
#include "arolib/cartography/sharedgridsmanager.hpp"
#include "arolib/misc/basic_responses.h"
#include "arolib/types/field.hpp"

namespace arolib {

/**
 * @brief Class used to generate and update remaining area maps
 */
class DrivenMassAreaManager : public LoggingComponent
{
public:
    using GridType = gridmap::SharedGridsManager::GridType;
    using GridPtr = gridmap::SharedGridsManager::GridPtr;
    using ConstGridPtr = gridmap::SharedGridsManager::ConstGridPtr;


    /**
     * @brief Precision option
     */
    enum PrecisionOption{
        LOW_PRECISION,
        MEDIUM_PRECISION,
        HIGH_PRECISION
    };

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
     * @brief Set the distance threshold used to forget previously driven cells.
     * @param dist Distance threshold
     */
    void setCellDistanceThreshold(float dist = 2);

    /**
     * @brief Set the option to use edge polygon intersection (true) or cells analysys (false) for repeating areas between previous edge and new edge.
     *
     * Only for MEDIUM_PRECISION and HIGH_PRECISION
     *
     * @param usePolygonIntersection Use edge polygon intersection (true) or cells analysys (false, recommended) for repeating areas between previous edge and new edge.
     */
    void setUsePolygonIntersection(bool usePolygonIntersection);

    /**
     * @brief Set the part/percentage of the machine length [0, 1] used for the machine/edge projections.
     * @param k [0, 1]
     */
    void setProjMachineLength(float k = 1.0);

    /**
     * @brief Updates the grid/map with the given location, machine mass and machine bunke mass (using the edge from the last location, if existent).
     * @param machine Driving machine
     * @param pt New location
     * @param bunker_mass Bunker mass
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp addData(const Machine &machine, Point pt, double bunker_mass);


    /**
     * @brief Get the current remaining-area map (if not ready, it is not allocated).
     * @return Current remaining-area map
     */
    ConstGridPtr getDrivenMassAreaMap() const;


    /**
     * @brief Get the projection of an edge.
     * @param machine Driving machine
     * @param pBack Previous point
     * @param pFront New point
     * @param width Edge width (if <0 -> machine.width)
     * @param [out] pBackEd Back point of the projection
     * @param [out] pBackEd Front point of the projection
     * @return Projection
     */
    Polygon getEdgeProjection(const Machine& machine, Point pBack, Point pFront, double width = -1, Point* pBackEd = nullptr, Point* pFrontEd = nullptr);

    /**
     * @brief Get the projection of a machine.
     * @param machine Driving machine
     * @param pBack Previous point
     * @param pFront New point
     * @param width Edge width (if <0 -> machine.width)
     * @param [out] pBackEd Back point of the projection
     * @param [out] pBackEd Front point of the projection
     * @return Projection
     */
    Polygon getMachineProjection(const Machine& machine, Point pBack, Point pFront, double width = -1, Point* pBackEd = nullptr, Point* pFrontEd = nullptr);

    /**
     * @brief Filter the current gridmap with a RemainingAreaMap.
     * @param ram RemainingAreaMap
     * @param threshold Filter threshold (0, 1) for the ram values
     * @return True on success
     */
    bool filterWithRemainingAreaMap(const ArolibGrid_t& ram, float threshold, bool be_precise);

    /**
     * @brief Filter the current gridmap with a RemainingAreaMap.
     * @param ram RemainingAreaMap
     * @param threshold Filter threshold (0, 1) for the ram values
     * @return True on success
     */
    bool filterWithRemainingAreaMap(std::shared_ptr<const ArolibGrid_t> ram, float threshold, bool be_precise);

protected:

    /**
     * @brief Struct holding the important data for a cell
     */
    struct CellData{
        float mass = 0; /**< mass */
        std::vector<std::shared_ptr<Polygon>> machineProj; /**< Machine projections that caused an overlap in the cell */
        std::map<std::string, double> overlappedMiniCells; /**< (mini) cells (and respective mass) overlapped in the cell */
    };

    /**
     * @brief Struct holding the important data for the map update
     */
    struct Data{
        std::vector<Point> points; /**< Previous driven edge */
        std::map<std::string, CellData> cellsData; /**< Driven cells data of the previous edges */
        Polygon machineProj; /**< Previous machine projection */
        float massTotal = 0; /**< Total mass driven over */
    };

    /**
     * @brief Get the key of a cell from its x,y
     * @param x x index
     * @param y y index
     * @return Cell string key
     */
    std::string toCellKey(size_t x, size_t y);

    /**
     * @brief Get the x,y of a cell from its string key
     * @param key Cell string key
     * @param [out] x x index
     * @param [out] y y index
     */
    void toCellCoords(const std::string& key, size_t& x, size_t& y);

    /**
     * @brief Get the (mini) layout of a given cell
     * @param x x index
     * @param y y index
     * @param be_precise Generate the layout with a higher resolution for better precision?
     * @return Layout
     */
    gridmap::GridmapLayout getCellMiniLayout(size_t& x, size_t& y, bool be_precise = true);

    /**
     * @brief Get the initialized DrivenMassAreaMap.
     * @return Initialized DrivenMassAreaMap
     */
    GridPtr initDrivenMassAreaMap();


    /**
     * @brief Convect a vector of GridCellOverlap into a map < x , < y , overlap > >.
     * @return Converted map
     */
    std::map<std::string, float> cellsInfoVecToMap(const std::vector<gridmap::GridmapLayout::GridCellOverlap>& vec);

    bool isCellCloseToMachine(const std::string& cell, const CellData &cellData, const Polygon &machineProj);

    /**
     * @brief Add new data with low precision.
     * @param machine Driving machine
     * @param pt New location
     * @param bunker_mass Bunker mass
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp addData_simple(const Machine &machine, const Point& pt, double bunker_mass);

    /**
     * @brief Add new data based on the polygon (machine projection) intersection.
     * @param machine Driving machine
     * @param pt New location
     * @param bunker_mass Bunker mass
     * @param be_precise Perform map/grid operations precisely
     * @return AroResp with error id (0:=OK) and message
     */
    AroResp addData_cellsAnalysis(const Machine &machine, const Point& pt, double bunker_mass, bool be_precise = true);

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
     * @brief Add new data based on the polygon (machine projection) intersection.
     * @param machine Driving machine
     * @param pt New location
     * @return Filtered point
     */
    Point getFilteredPoint(const Machine &machine, const Point& pt);

protected:

    bool m_ready = false; /**< Is the manager ready? */
    Field m_field; /**< Field */
    GridPtr m_drivenMassAreaMap = nullptr; /**< Driven-mass map map */
    gridmap::SharedGridsManager m_gridsManager; /**< Shared grids manager >*/
    std::map<MachineId_t, Data> m_prevData; /**< Previous data per machine */
    float m_defaultGPSFrontDisplacement = 0.05; /**< Default value of the location of the GPS w.r.t. the machine length from the front of the machine >*/
    std::map<MachineId_t, float> m_GPSFrontDisplacements; /**< Default value of the location of the GPS w.r.t. the machine length from the fron of the machine >*/
    float m_cellDistanceThreshold = 2; /**< Distance used to remove previous cells from memory >*/
    bool m_usePolygonIntersection = false; /**< Use edge polygon intersection (true) or cells analysys (false, recommended) for repeating areas between previous edge and new edge >*/
    float m_projMachineLength = 1.0;  /**< Part of the machine length [0, 1] used for the machine/edge projections >*/
    PrecisionOption m_precision = PrecisionOption::MEDIUM_PRECISION; /**< Precision option >*/
    static const std::string DrivenMassAreaMapName; /**< Remainingg-are grid-map name >*/
    bool m_filterAng = true; /**< Filter points based on angle */


};

}

#endif // AROLIB_DRIVENMASSAREAMANAGER_H
