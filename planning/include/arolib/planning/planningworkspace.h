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
 
#ifndef AROLIB_PLANNINGWORKSPACE_H
#define AROLIB_PLANNINGWORKSPACE_H

#include <map>
#include <iostream>

#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/logger.h"
#include "arolib/types/machine.hpp"
#include "arolib/types/field.hpp"
#include "arolib/types/headlandroute.hpp"
#include "arolib/types/outfieldinfo.hpp"
#include "arolib/types/machinedynamicinfo.hpp"
#include "arolib/types/route.hpp"
#include "arolib/cartography/common.hpp"


namespace arolib {

namespace DirectedGraph{
class DirectedGraphBuilder;
}
class PlanningWorkspaceAccessor;

/**
 * @brief Class holding all the data needed for planning (fields, maps/grids, routes, machines, etc...)
 */
class PlanningWorkspace : public LoggingComponent
{
public:

    /**
     * @brief Grid/map type
     */
    enum GridType{
        YIELD, /**< Yield-proportion map/grid */
        DRYNESS, /**< Drymatter map/grid */
        SOIL, /**< Soil map/grid */
        REMAINING_AREA, /**< Remaining(unhrvested)-area map/grid */
        GridType_TOTAL /**< DO NOT USE TO SET A GRID TYPE (used as total number of types) */
    };

    /**
     * @brief Route type
     */
    enum RouteType{
        BASE_HEADLAND, /**< Base-route for headland processing */
        BASE_INFIELD, /**< Base-route for inner-field processing */
        BASE_PROCESSED, /**< Processed base routes (inc. connected base routes) */
        PLANNED, /**< Planned routes */
        RouteType_TOTAL /**< DO NOT USE TO SET A ROUTE TYPE (used as total number of types) */
    };

    /**
     * @brief Process
     */
    enum ProcessType{
        PROCESS_HEADLAND, /**< Headland processing */
        PROCESS_INFIELD, /**< Inner-field processing */
    };

    /**
     * @brief Holds the geometric information of an edge
     */
    struct Edge{
        Point p0; /**< First point */
        Point p1; /**< Second point */
        double width; /**< Edge width */

        /**
         * @brief Constructor.
         * @param _p0 First point.
         * @param _p1 Second point.
         * @param _width Edge width.
         */
        explicit Edge(const Point &_p0, const Point &_p1, double _width);

        /**
         * @brief operator==.
         *
         * If p1 == e.p0 and p0 == e.p1, it is considered that the edge points are equal.
         * @param e Other edge.
         * @return True if the edge is equal to the given (other) edge
         */
        bool operator==(const Edge &e) const;

        /**
         * @brief operator>.
         *
         * Mostly based on the distance between the edges and the origin [0,0]. Used for sorting the edges.
         * @param e Other edge.
         * @return True if the edge is > than the given (other) edge
         */
        bool operator>(const Edge &e) const;

        /**
         * @brief operator<.
         *
         * Mostly based on the distance between the edges and the origin [0,0]. Used for sorting the edges.
         * @param e Other edge.
         * @return True if the edge is < than the given (other) edge
         */
        bool operator<(const Edge &e) const;
    };

    /**
     * @brief Constructor
     * @param logLevel Log level
     */
    explicit PlanningWorkspace(const LogLevel &logLevel = LogLevel::INFO);

    /**
     * @brief Clears/removes all parameters (field, routes, edges, etc).
     */
    void clearAll();

    /**
     * @brief Clears all parameters (boundaries, tracks, etc) of a given subfield.
     * @param subfieldIdx Index of the subfield to be cleared. If < 0, clears all subfields.
     * @param onlyInfield If true, only the subfield parameters related to the inner-field (tracks, etc) are cleared; otherwise, all parameters (including headland parameters such as inner boundary, headland tracks, etc) are cleared.
     * @return True on success.
     */
    bool clearSubfield(int subfieldIdx = -1,
                       bool onlyInfield = false);

    /**
     * @brief Clears/removes the desired routes and respective edges.
     * @param routeType Type of routes to be removed. If = RouteType_TOTAL, clears all routes.
     */
    void clearRoutes(RouteType routeType);

    /**
     * @brief Set the working field.
     * @param _field Field.
     */
    void setField(const Field& _field);

    /**
     * @brief Set the working group of machines.
     * @param _machines Working group.
     */
    void setMachines(const std::vector<Machine> &_machines);

    /**
     * @brief Set the out-of-field information (inc. arrival times, transport times, etc.).
     * @param outFieldInfo Out-of-field information.
     */
    void setOutFieldInfo(const OutFieldInfo &_outFieldInfo);

    /**
     * @brief Set the map containing the current states of the machines.
     * @param _machineCurrentStates Map containing the current states of the machines
     */
    void setMachineCurrentStates(const std::map<MachineId_t, MachineDynamicInfo> &_machineCurrentStates);

    /**
     * @brief Sets/updates a grid/map. Automatically updates the edges if the map geometry (boundig box and resolution) changes w.r.t. the currently saved one.
     * @param gridType Type of grid/map to be set/updated
     * @param grid New grid/map
     * @param updateCellsIfDifferent If the new grid/map is (geometrically) different from the currently saved one (inc. if no current grid is set), the information of the cells from existing edges are updated (iif they were set already).
     */
    void setGrid(PlanningWorkspace::GridType gridType, const ArolibGrid_t& grid, bool updateCellsIfDifferent = false);

    /**
     * @brief Gets a grid/map.
     * @param gridType Type of grid/map
     * @return grid/map
     */
    const ArolibGrid_t& getGrid(PlanningWorkspace::GridType gridType);

    /**
     * @brief Retrieve the working field
     * @return Working field.
     */
    const Field& getField() const {return m_field;}

    /**
     * @brief Retrieve the base routes for headland harvesting
     * @return Map containing the base routes for headland harvesting for each subfield. <subfield id, routes> (at the moment, the subfield id = subfield index in the fields' subfields vector).
     */
    const std::map< int, std::vector<HeadlandRoute> >& getBaseRoutes_headland(bool processed) const;

    /**
     * @brief Retrieve the base routes for headland harvesting
     * @return Map containing the base routes for headland harvesting for each subfield. <subfield id, routes> (at the moment, the subfield id = subfield index in the fields' subfields vector).
     */
    std::map< int, std::vector<HeadlandRoute> >& getBaseRoutes_headland(bool processed);

    /**
     * @brief Retrieve the base routes for inner-field harvesting
     * @return Map containing the base routes for inner-field harvesting for each subfield. <subfield id, routes> (at the moment, the subfield id = subfield index in the fields' subfields vector).
     */
    const std::map< int, std::vector<Route> >& getBaseRoutes_infield(bool processed) const;

    /**
     * @brief Retrieve the base routes for inner-field harvesting
     * @return Map containing the base routes for inner-field harvesting for each subfield. <subfield id, routes> (at the moment, the subfield id = subfield index in the fields' subfields vector).
     */
    std::map< int, std::vector<Route> >& getBaseRoutes_infield(bool processed);

    /**
     * @brief Retrieve the connected base routes
     * @return Map containing the connected base routes. <subfield id, routes> (at the moment, the subfield id = subfield index in the fields' subfields vector).
     */
    std::map< int, std::vector<Route> >& getConnectedBaseRoutes() {return m_connectedBaseRoutes;}

    /**
     * @brief Retrieve the connected base routes
     * @return Map containing the connected base routes. <subfield id, routes> (at the moment, the subfield id = subfield index in the fields' subfields vector).
     */
    const std::map< int, std::vector<Route> >& getConnectedBaseRoutes() const {return m_connectedBaseRoutes;}

    /**
     * @brief Retrieve the planned routes (harvesters and OLVs)
     * @return Map containing the planned routes (harvesters and OLVs) for each subfield. <subfield id, routes> (at the moment, the subfield id = subfield index in the fields' subfields vector).
     */
    const std::map< int, std::vector<Route> >& getPlannedRoutes() const {return m_plannedRoutes;}

    /**
     * @brief Retrieve the planned routes (harvesters and OLVs)
     * @return Map containing the planned routes (harvesters and OLVs) for each subfield. <subfield id, routes> (at the moment, the subfield id = subfield index in the fields' subfields vector).
     */
    std::map< int, std::vector<Route> >& getPlannedRoutes() {return m_plannedRoutes;}


    /**
     * @brief Obtain the grid cells info of an edge. If the edge doesn exist, it updates the working space edges' properties
     * @param gridType Grid type
     * @param processType Process related to the set of edges to be checked/updated
     * @param p0 First edge point
     * @param p1 Second edge point
     * @param width Edge width
     * @return Grid cells info of the edge
     */
    const std::vector<gridmap::GridmapLayout::GridCellOverlap> &getEdgeCellsInfo(PlanningWorkspace::GridType gridType,
                                                      ProcessType processType,
                                                      const Point p0,
                                                      const Point p1,
                                                      double width);

protected:

    /**
     * @brief Holds the edge complex information
     */
    struct EdgeProperties{
        std::vector< std::vector<gridmap::GridmapLayout::GridCellOverlap> > gridCellsInfo; /**< Holds the information of the cells related to an edge (i.e. the cells that an edge overlaps) for each of the grid types */
        std::vector<bool> gridCellsInfoIsSet; /**< Flag used to know if the information of the cells related to an edge is set (for each of the grid types) */

        /**
         * @brief Constructor.
         *
         * Initializes the vectors gridCellsInfo and gridCellsInfoIsSet.
         */
        explicit EdgeProperties(): gridCellsInfo(PlanningWorkspace::GridType::GridType_TOTAL), gridCellsInfoIsSet(PlanningWorkspace::GridType::GridType_TOTAL, false){}

        /**
         * @brief Updates the (currently saved) information of the cells related to a given edge (i.e. the cells that the edge overlaps) for a given grid type.
         * @param type Only the cells' information related to this type of grid will be updated.
         * @param edge Edge to be updated.
         * @param addIfNotSet If true and the cells' information related to the given edge and grid type does not exist yet, it will obtain the cells' info and add it.
         */
        void updateGridCells(PlanningWorkspace::GridType type, const ArolibGrid_t &grid, const Edge &edge, bool addIfNotSet);

        /**
         * @brief Clear/remove all the (currently saved) information of all the cells related to a given grid type.
         * @param type Only the cells' information related to this type of grid will be removed. If = GridType_TOTAL, all cells' information will be remodved (i.e. related to all grid types)
         */
        void clearGridCells(PlanningWorkspace::GridType type);
    };

    /**
     * @brief Clear/remove the desired routes and respective edges.
     * @param routeType Type of route (and corresponding edges) To be removed. If = RouteType_TOTAL, all route types will be removed.
     */
    void clearRouteMaps(RouteType routeType);

    /**
     * @brief Update the cells' information of the given edges related to a given grid.
     * @param [in/out] edges Map containing the edges and coresponding edge properties (inc. cells' information) to be updated
     * @param gridType Only the cells' information related to this type of grid will be updated.
     * @param grid Grid used for the calculations and update (must correspond to the given gridType).
     * @param addIfNotSet If true and the cells' information related to the given edge and grid type does not exist yet, it will obtain the cells' info and add it..
     */
    static void updateGridCells(std::map<Edge, EdgeProperties>& edges, PlanningWorkspace::GridType gridType, const ArolibGrid_t &grid, bool addIfNotSet);

    /**
     * @brief Remove the cells' information of the given edges related to a given grid.
     * @param [in/out] edges Map containing the edges and coresponding edge properties (inc. cells' information) to be updated
     * @param gridType Only the cells' information related to this type of grid will be updated. If = GridType_TOTAL, the cells' info related to all grid types will be removed.
     */
    static void clearGridCells(std::map<Edge, EdgeProperties>& edges, PlanningWorkspace::GridType gridType);

protected:
    Field m_field; /**< Working field */
    std::vector<Machine> m_machines; /**< Working group of machines */
    OutFieldInfo m_outFieldInfo; /**< Out-of-field information (inc. arrival times, transport times, etc.) */
    std::map<MachineId_t, MachineDynamicInfo> m_machineCurrentStates; /**< Current states of the machines */
    std::vector< ArolibGrid_t > m_maps; /**< Vector containing all working grids/maps */
    std::map< int, std::vector<HeadlandRoute> > m_baseRoutes_headland; /**< Map containing the base routes for headland processing for each subfield. <subfield id, routes> (at the moment, the subfield id = subfield index in the fields' subfields vector). */
    std::map< int, std::vector<Route> > m_baseRoutes_infield; /**< Map containing the base routes for inner-field processing for each subfield. <subfield id, routes> (at the moment, the subfield id = subfield index in the fields' subfields vector). */
    std::map< int, std::vector<HeadlandRoute> > m_baseRoutesProcessed_headland; /**< Map containing processed the base routes for headland processing for each subfield. <subfield id, routes> (at the moment, the subfield id = subfield index in the fields' subfields vector). */
    std::map< int, std::vector<Route> > m_baseRoutesProcessed_infield; /**< Map containing the processed base routes for inner-field processing for each subfield. <subfield id, routes> (at the moment, the subfield id = subfield index in the fields' subfields vector). */
    std::map< int, std::vector<Route> > m_connectedBaseRoutes; /**< Map containing the connected processsed base routes for each subfield. <subfield id, routes> (at the moment, the subfield id = subfield index in the fields' subfields vector) */
    std::map< int, std::vector<Route> > m_plannedRoutes; /**< Map containing the planned routes (all machines) for each subfield. <subfield id, routes> (at the moment, the subfield id = subfield index in the fields' subfields vector) */
    std::map<Edge, EdgeProperties> m_edgesHeadland; /**< Map containing the edges and coresponding edge properties (inc. cells' information (inc. what cells overlap with the edge and how much)) related to headland harvesting */
    std::map<Edge, EdgeProperties> m_edgesInfield; /**< Map containing the edges and coresponding edge properties (inc. cells' information (inc. what cells overlap with the edge and how much)) related to inner-field harvesting */

    //The following classes can access all private/protected members
    friend class PlanningWorkspaceAccessor;

};



/**
 * @brief Class to access the protected members and methods of PlanningWorkspace
 *
 * Any class (planner, etc) that wants to access directly the members and methods of a PlanningWorkspace must inherit from PlanningWorkspaceAccessor
 */
class PlanningWorkspaceAccessor{
protected:
    using EdgeProperties = PlanningWorkspace::EdgeProperties;

    /**
     * @brief Clear/remove the desired routes and respective edges from the given PlanningWorkspace.
     * @param [in/out] pw PlanningWorkspace.
     * @param routeType Type of route (and corresponding edges) To be removed. If = RouteType_TOTAL, all route types will be removed.
     */
    static inline void clearRouteMaps(PlanningWorkspace& pw, PlanningWorkspace::RouteType routeType){ return pw.clearRouteMaps(routeType); }

    /**
     * @brief Update the cells' information of the given edges related to a given grid from the given PlanningWorkspace.
     * @param [in/out] edges Map containing the edges and coresponding edge properties (inc. cells' information) to be updated
     * @param gridType Only the cells' information related to this type of grid will be updated.
     * @param grid Grid used for the calculations and update (must correspond to the given gridType).
     * @param addIfNotSet If true and the cells' information related to the given edge and grid type does not exist yet, it will obtain the cells' info and add it..
     */
    static inline void updateGridCells(std::map<PlanningWorkspace::Edge, EdgeProperties>& edges, PlanningWorkspace::GridType gridType, const ArolibGrid_t &grid, bool addIfNotSet){ return PlanningWorkspace::updateGridCells(edges, gridType, grid, addIfNotSet); }

    /**
     * @brief Remove the cells' information of the given edges related to a given grid.
     * @param [in/out] edges Map containing the edges and coresponding edge properties (inc. cells' information) to be updated
     * @param gridType Only the cells' information related to this type of grid will be updated. If = GridType_TOTAL, the cells' info related to all grid types will be removed.
     */
    static inline void clearGridCells(std::map<PlanningWorkspace::Edge, EdgeProperties>& edges, PlanningWorkspace::GridType gridType){ return PlanningWorkspace::clearGridCells(edges, gridType); }



    /**
     * @brief Get the field from the given PlanningWorkspace.
     * @param pw PlanningWorkspace.
     * @return Field (reference to)
     */
    static inline Field& getField(PlanningWorkspace& pw){ return pw.m_field; }

    /**
     * @brief Get the machines (working group) from the given PlanningWorkspace.
     * @param pw PlanningWorkspace.
     * @return Machines (reference to)
     */
    static inline std::vector<Machine>& getMachines(PlanningWorkspace& pw){ return pw.m_machines; }

    /**
     * @brief Get the out-of-field information from the given PlanningWorkspace.
     * @param pw PlanningWorkspace.
     * @return Out-of-field information (reference to)
     */
    static inline OutFieldInfo& getOutFieldInfo(PlanningWorkspace& pw){ return pw.m_outFieldInfo; }

    /**
     * @brief Get the machines' current states from the given PlanningWorkspace.
     * @param pw PlanningWorkspace.
     * @return Machines' current states (reference to)
     */
    static inline std::map<MachineId_t, MachineDynamicInfo>& getMachineCurrentStates(PlanningWorkspace& pw){ return pw.m_machineCurrentStates; }

    /**
     * @brief Get the gridmaps from the given PlanningWorkspace.
     * @param pw PlanningWorkspace.
     * @return Gridmaps (reference to)
     */
    static inline std::vector< ArolibGrid_t >& getMaps(PlanningWorkspace& pw){ return pw.m_maps; }

    /**
     * @brief Get the headland base routes from the given PlanningWorkspace.
     * @param pw PlanningWorkspace.
     * @return Headland base routes (reference to)
     */
    static inline std::map< int, std::vector<HeadlandRoute> >& getBaseRoutes_headland(PlanningWorkspace& pw){ return pw.m_baseRoutes_headland; }

    /**
     * @brief Get the infield base routes from the given PlanningWorkspace.
     * @param pw PlanningWorkspace.
     * @return Infield base routes (reference to)
     */
    static inline std::map< int, std::vector<Route> >& getBaseRoutes_infield(PlanningWorkspace& pw){ return pw.m_baseRoutes_infield; }

    /**
     * @brief Get the processed headland base routes from the given PlanningWorkspace.
     * @param pw PlanningWorkspace.
     * @return Headland processed base routes (reference to)
     */
    static inline std::map< int, std::vector<HeadlandRoute> >& getBaseRoutesProcessed_headland(PlanningWorkspace& pw){ return pw.m_baseRoutesProcessed_headland; }

    /**
     * @brief Get the processed infield base routes from the given PlanningWorkspace.
     * @param pw PlanningWorkspace.
     * @return Infield processed base routes (reference to)
     */
    static inline std::map< int, std::vector<Route> >& getBaseRoutesProcessed_infield(PlanningWorkspace& pw){ return pw.m_baseRoutesProcessed_infield; }

    /**
     * @brief Get the connected base routes from the given PlanningWorkspace.
     * @param pw PlanningWorkspace.
     * @return Connected base routes (reference to)
     */
    static inline std::map< int, std::vector<Route> >& getConnectedBaseRoutes(PlanningWorkspace& pw){ return pw.m_connectedBaseRoutes; }

    /**
     * @brief Get the planned base routes from the given PlanningWorkspace.
     * @param pw PlanningWorkspace.
     * @return Planned base routes (reference to)
     */
    static inline std::map< int, std::vector<Route> >& getPlannedRoutes(PlanningWorkspace& pw){ return pw.m_plannedRoutes; }

    /**
     * @brief Get the headland edges from the given PlanningWorkspace.
     * @param pw PlanningWorkspace.
     * @return Headland edges (reference to)
     */
    static inline std::map<PlanningWorkspace::Edge, EdgeProperties>& getEdgesHeadland(PlanningWorkspace& pw){ return pw.m_edgesHeadland; }

    /**
     * @brief Get the infield edges from the given PlanningWorkspace.
     * @param pw PlanningWorkspace.
     * @return Infield edges (reference to)
     */
    static inline std::map<PlanningWorkspace::Edge, EdgeProperties>& getEdgesInfield(PlanningWorkspace& pw){ return pw.m_edgesInfield; }
};

}

#endif // AROLIB_PLANNINGWORKSPACE_H
