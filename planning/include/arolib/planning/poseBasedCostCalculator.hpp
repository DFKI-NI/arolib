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
 
#ifndef ARO_POSEBASEDCOSTCALCULATOR_HPP
#define ARO_POSEBASEDCOSTCALCULATOR_HPP

#include <functional>

#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/curves_helper.hpp"
#include "arolib/misc/container_helper.h"
#include "arolib/misc/logger.h"
#include "arolib/cartography/common.hpp"
#include "arolib/cartography/gridcellsinfomanager.hpp"
#include "arolib/planning/poseBasedGraph.hpp"
#include "arolib/planning/visit_period.hpp"

namespace arolib{

/**
 * @brief Interface for the cost calculator used in PoseBased search
 */
class IPoseBasedCostCalculator : public LoggingComponent, public IMappableParameters{
public:

    /**
     * @brief Type of restricted area
     */
    enum RestrictedAreaOption{
        RESTRICT_WORKED_AREAS, /**< Do not drive over worked areas */
        RESTRICT_UNWORKED_AREAS /**< Do not drive over unworked areas */
    };

    /**
      * @brief Get the RestrictedAreaOption (enum) from its int value
      * @brief value Int value
      * @return RestrictedAreaOption
      */
    static RestrictedAreaOption intToRestrictedAreaOption(int value);

    /**
     * @brief Cell containing information related to restricted areas
     */
    struct RestrictedAreaCell{
        double timestamp; /**< Timestamp of when the cell is worked */
        MachineId_t machineId; /**< Machine working the cell */
    };

    using Vertex_t = PoseBasedGraph::Vertex_t;
    using VertexId_t = PoseBasedGraph::VertexId_t;
    using Cost_t = long double;

    using GridmapField_t = gridmap::Gridmap<int>;
    using GridmapObstacles_t = gridmap::Gridmap<bool>;
    using GridmapRestrictedAreas_t = gridmap::Gridmap<RestrictedAreaCell>;
    using GridmapVisits_t = gridmap::Gridmap< std::vector<VisitPeriod> >;

    /**
     * @brief General cost calculator parameters
     */
    struct GeneralParameters : public IMappableParameters{
        bool includeWaitInCost = true; /**< Should the time that a machine has to wait for a region to become available (e.g. because it has not been harvested yet or it is occupied by another machine) be included in the cost? */
        bool withCollisionAvoidance = false; /**< Check for collision avoidance */
        double clearanceTime = 10; /**< Time a machine has to wait so that another machine clears a region before it drives over it */
        double crossCostCoef = 1; /**< Coefficient for cost calculation when the segment crosses inner-field tracks */
        double crossCostCoef_HL = 1; /**< Coefficient for cost calculation when the segment crosses headland tracks */
        double outOfBoundaryCostCoef = 1; /**< Coefficient for cost calculation when the segment drives over the boundary */
        RestrictedAreaOption restrictedAreaOption = RESTRICT_UNWORKED_AREAS;/**< Type of restricted area */

        /**
         * @brief Updates the parameters with the values parsed from a string map
         * @param map String map containing the parameter values
         * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the current values will remain)
         * @return True on success
         */
        virtual bool parseFromStringMap(const std::map<std::string, std::string>& strMap, bool strict) override;

        /**
         * @brief Parse the parameters to a string map
         * @return String map containing the parameter values
         */
        virtual std::map<std::string, std::string> parseToStringMap() const override;
    };

    /**
     * @brief Struct holding the (extended) cost information
     */
    struct CostExtended_t{

        /**
         * @brief Validity of the cost
         */
        enum Validity{
            VALID, /**< Valid */
            INVALID, /**< Invalid (unespecified reason) */
            INVALID__OBSTACLE, /**< Invalid (because of an obstacle) */
            INVALID__RESTRICTED_AREA, /**< Invalid (because area is restricted ) */
            INVALID__BUSY_AREA, /**< Invalid (because area is busy ) */
            INVALID__MISSING_OUTFIELD_INFO, /**< Invalid (because out-field-info is missing ) */
            INVALID__SPEED /**< Invalid (because of speed ) */
        };

        /**
         * @brief Path point
         */
        struct PathPoint{
            Point p; /**< Point */
            double time_stamp; /**< Time-stamp */

            /**
             * @brief Constructor
             * @param _p Point
             * @param _time_stamp Time-stamp
             */
            PathPoint(const Point& _p = Point(), double _time_stamp = -1);
        };
        using Path_t = std::vector<PathPoint>;

        Validity valid = VALID; /**< Cost validity */
        Cost_t cost = 0;/**< cost */
        double time; /**< Time needed to reach the last point */
        double waitTime; /**< Total amount of waiting time to reach the last point (from the start node) */
        int numCrossings = -1; /**< Total number of inner-field cross-edges used to achieve the current node from the start node */
        int numCrossings_HL = -1; /**< Total number of headland-harvesting cross-edges used to achieve the current node from the start node */
        Path_t path; /**< Path with the respective timestamps (only for IF cost calculations) */
    };

    using CalcSpeedFunct = std::function< double ( const Machine&, //machine
                                                   const std::vector<Point>&, //path
                                                   double//bunker mass
                                                  ) >; //


    /**
     * @brief Constructor
     * @param parentLogger Parent logger
     */
    explicit IPoseBasedCostCalculator(Logger* parentLogger = nullptr);

    /**
     * @brief Set the function used to compute the speed
     * @param f Function used to compute the speed
     */
    virtual void setCalcSpeedFunct(const CalcSpeedFunct& f);

    /**
     * @brief Check if a vertex is valid
     * @param vt Vertex
     * @param m machine
     * @param time_start Starting time
     * @param restrictedMachines Set of machines used for "restricted areas" checks
     * @return Validity
     */
    virtual CostExtended_t::Validity isVertexValid(const Vertex_t& vt, const Machine * const m, double time_start, const std::set<MachineId_t>& restrictedMachines) const;

    /**
     * @brief Compute the cost for a path (driving inside the field)
     * @param path Path
     * @param m machine
     * @param time_start Starting time
     * @param bunkerMass Bunker mass
     * @param restrictedMachines Set of machines used for "restricted areas" checks
     * @return Cost
     */
    virtual CostExtended_t calcCost( const std::vector<Point>& path, const Machine& m, double time_start, double bunkerMass, const std::set<MachineId_t>& restrictedMachines ) const = 0;

    /**
     * @brief Compute the cost for a path (driving outside the field)
     * @param OFCosts Costs for out-of-field driving
     * @param m machine
     * @param time_start Starting time
     * @param bunkerMass Bunker mass
     * @return Cost
     */
    virtual CostExtended_t calcCost( const PoseBasedGraph::OutFieldCosts& OFCosts, const Machine& m, double time_start, double bunkerMass ) const = 0;

    /**
     * @brief Compute the heuristic cost from one vertex to another
     * @param start_vt Start vertex
     * @param goal_vt Goal vertex
     * @param m machine
     * @return Cost
     */
    virtual Cost_t calcHeuristic( const Vertex_t& start_vt, const Vertex_t& goal_vt, const Machine& m ) const = 0;

    /**
     * @brief Compute the heuristic cost from one vertex to another
     * @param graph Graph
     * @param start_vt Start vertex
     * @param goal_vt Goal vertex
     * @param m machine
     * @return Cost
     */
    virtual Cost_t calcHeuristic( const PoseBasedGraph& graph, const Vertex_t& start_vt, const Vertex_t& goal_vt, const Machine& m  ) const = 0;

    /**
     * @brief Get the best out-of-field connection between a set of starting vertices and a set of goal vertices
     * @param graph Graph
     * @param start_vts Start vertices
     * @param goal_vts Goal vertices
     * @param machine_id Machine Id
     * @param [out] best_start_vt Id of the start vertex for the best connection
     * @param [out] best_goal_vt Id of the goal vertex for the best connection
     * @param [out] best_cost Cost for the best connection
     * @return True if connection found
     */
    virtual bool getBestOutfieldConnection(const PoseBasedGraph& graph,
                                            const std::unordered_set<VertexId_t> &start_vts,
                                            const std::unordered_set<VertexId_t> &goal_vts,
                                            const MachineId_t &machine_id,
                                            VertexId_t &best_start_vt,
                                            VertexId_t &best_goal_vt,
                                            Cost_t& best_cost) const;


    /**
     * @brief Default speed calculation
     * @param m Machine
     * @param
     * @param bunkerMass Bunker mass
     * @return Speed
     */
    static double CalcSpeedDefault (const Machine& m, const std::vector<Point>&, double bunkerMass);

    /**
     * @brief Set the gridmap corresponding to the field geometries.
     *
     * No cellvalue : no info
     * Cellvalues >= 0 : track ids
     * Cellvalues = -1 : obstacles
     * Otherwise : Out of boundary
     *
     * This map is not used for obstacle avoidance. For that, set the obstacles using setObstacles() or addObstacles()
     *
     * @param gridmap Gridmap
     * @return True on success
     */
    virtual bool setGridmap_field(const GridmapField_t& gridmap);

    /**
     * @brief Set the gridmap corresponding to restricted-areas information.
     * @param gridmap Gridmap
     * @return True on success
     */
    virtual bool setGridmap_restrictedAreas(const GridmapRestrictedAreas_t& gridmap);

    /**
     * @brief Set the gridmap corresponding to visits information.
     * @param gridmap Gridmap
     * @return True on success
     */
    virtual bool setGridmap_visits(const GridmapVisits_t& gridmap);

    /**
     * @brief Remove a gridmap
     * @param name Gridmap name
     * @return True on success
     */
    virtual bool removeGridmap(const std::string &name);

    /**
     * @brief Get the gridmap corresponding to the field geometries.
     * @return Gridmap
     */
    virtual const GridmapField_t& getGridmap_field() const;

    /**
     * @brief Get the gridmap corresponding to restricted-areas information.
     * @return Gridmap
     */
    virtual const GridmapRestrictedAreas_t& getGridmap_restrictedAreas() const;

    /**
     * @brief Get the gridmap corresponding to visits information.
     * @return Gridmap
     */
    virtual const GridmapVisits_t& getGridmap_visits() const;

    /**
     * @brief Sets/replace the list of obstacles.
     * @param obs Obstacles
     * @return Number of valid obstacles added
     */
    virtual size_t setObstacles(const std::vector<Obstacle>& obs);

    /**
     * @brief Adds an obstacle to the the list of obstacles.
     * @param obs Obstacle
     * @return True is the obstacle is valid and was added
     */
    virtual bool addObstacle(Obstacle obs);

    /**
     * @brief Get the list of obstacles.
     * @return Obstacles
     */
    virtual const std::vector<Obstacle>& getObstacles() const;

    /**
     * @brief Register a delay for a machine.
     * @param machineId Machine Id
     * @param time_from Time(-stamp) of when the delay happens
     * @param delay Delay duration
     */
    void registerDelay(MachineId_t machineId, double time_from, double delay);

    /**
     * @brief Initialize the gridmap corresponding to the field geometries from a subfield.
     * @param sf Subfield
     * @return True on success
     */
    bool initGridmap_field(const Subfield &sf);

    /**
     * @brief Add visit periods from a route.
     * @param machine_id Machine Id
     * @param route_points Route points
     * @param width Machine width
     * @param idx_from Route point index from where to start adding the visit periods (if <0 -> all route points)
     */
    void addVisitPeriods(MachineId_t machine_id, const std::vector<RoutePoint>& route_points, double width, int idx_from = -1);

    /**
     * @brief Add information related to restricted areas from a route.
     * @param machine_id Machine Id
     * @param route_points Route points
     * @param width Machine width
     * @param idx_from Route point index from where to start adding the visit periods (if <0 -> all route points)
     * @param startFromLast If true, the visit periods will be added starting from the last route point
     */
    void addRestrictedAreaInfo(MachineId_t machine_id, const std::vector<RoutePoint>& route_points, double width, int idx_from = -1, bool startFromLast = true);

    /**
     * @brief Set the general parameters
     * @param params general parameters
     */
    virtual void setGeneralParameters(const GeneralParameters& params);

    /**
     * @brief Updates the parameters with the values parsed from a string map
     * @param map String map containing the parameter values
     * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the current values will remain)
     * @return True on success
     */
    virtual bool parseFromStringMap(const std::map<std::string, std::string>& strMap, bool strict) final;

    /**
     * @brief Parse the parameters to a string map
     * @return String map containing the parameter values
     */
    virtual std::map<std::string, std::string> parseToStringMap() const final;

    /**
     * @brief Check if a calculator if of a given type T.
     * @return True if the same type.
     */
    template<typename T,
             typename = typename std::enable_if< std::is_base_of<IPoseBasedCostCalculator, T>::value >::type>
    bool isOfType() const {
        return typeid(*this) == typeid(T);
    }

    static const std::string GridMap_Field; /**< Name of the Field-geometries gridmap */
    static const std::string GridMap_RestrictedAreas; /**< Name of the restricted-areas information gridmap */
    static const std::string GridMap_Visits; /**< Name of the visits gridmap */

    static const int CellValue_obstacle; /**< Value to be set in cells overlaping with obstacles */
    static const int CellValue_outOfBoundary; /**< Value to be set in cells located outside the boundary */

protected:

    /**
     * @brief Key used to sort computed cells.
     */
    struct ComputedGridCellsKey{
        gridmap::GridmapLayout lo; /**< Gridmap layout */
        bool precise; /**< Computed precicely? */

        /**
         * @brief Comparison operator <.
         * @param other Other
         * @return True if this < other
         */
        bool operator <(const ComputedGridCellsKey& other) const{
            if(precise != other.precise)
                return !precise;
            return lo < other.lo;
        }
    };


    /**
     * @brief Constructor.
     * @param childName Child class
     * @param parentLogger Parent logger
     */
    explicit IPoseBasedCostCalculator(const std::string childName, Logger* parentLogger = nullptr);

    /**
     * @brief Get the list of cells' overlaps for a given path.
     *
     * If the overlaps were computed already (i.e. in computedCells), returns the previously computed cells; otherwise computes the cells and saves them in computedCells.
     * Assumes the previously computed cells were computed with the same points and width
     *
     * @param [in/out] computedCells Map of computed cells
     * @param lo Gridmap layout
     * @param precise Make the computations precisely?
     * @param path Path
     * @param width Machine width
     * @return List of computed cells' overlaps
     */
    static std::vector<gridmap::GridmapLayout::GridCellOverlap>* getOverlapCells(std::map<ComputedGridCellsKey, std::vector<gridmap::GridmapLayout::GridCellOverlap>> & computedCells,
                                                                                 const gridmap::GridmapLayout& lo,
                                                                                 bool precise,
                                                                                 const std::vector<Point> & path,
                                                                                 double width);


    /**
     * @brief Check if the path goes over an obstacle.
     *
     * If the overlaps were computed already (i.e. in computedCells), uses the previously computed cells; otherwise computes the cells and saves them in computedCells.
     * Assumes the previously computed cells were computed with the same points and width
     *
     * @param [in/out] computedCells Map of computed cells
     * @param precise Make the computations precisely?
     * @param path Path
     * @param width Machine width
     * @return True if there are NO obstacles in the path
     */
    virtual bool checkObstaclesInPath(std::map<ComputedGridCellsKey, std::vector<gridmap::GridmapLayout::GridCellOverlap>> & computedCells,
                                      bool precise,
                                      const std::vector<Point> & path,
                                      double width) const;

    /**
     * @brief Gets the number of times a path crosses from one IF track to another IF track.
     * @param path Path
     * @return Number of times a path crosses from one IF track to another IF track
     */
    virtual size_t getInnerFieldTrackCrossings(std::vector<Point> path) const;

    /**
     * @brief Computes the route segment based on the gridmaps, internal parameters, etc, and save the results (route, waiting time, error) in the costInfo.
     * @param path Path
     * @param m Machine (disregarded if null)
     * @param width Path width
     * @param time_start Starting time of the path
     * @param speed Speed
     * @param restrictedMachines Set of machines used for "restricted areas" checks
     * @param precise Make the computations precisely?
     * @param [in/out] costInfo Cost information
     */
    virtual void computeRouteSegment(const std::vector<Point>& path,
                                     const Machine *m,
                                     double width,
                                     double time_start,
                                     double speed,
                                     const std::set<MachineId_t> &restrictedMachines,
                                     bool precise,
                                     CostExtended_t& costInfo) const;

    /**
     * @brief Set other specific parameters from a string map containing the values as string.
     * @param strMap String map containing the parameter values
     * @param strict If true, all parameters have to be in the map to suceed; if false, only parameters present in the map will be set (otherwise the current values will remain)
     * @return True on success
     */
    virtual bool parseOtherParametersFromStringMap(const std::map<std::string, std::string>& strMap, bool strict) = 0;

    /**
     * @brief Parse other specific parameters and append them to the string map .
     * @param [in/out] strMap String map containing the parameter values
     */
    virtual void parseAndAppendOtherParametersToStringMap(std::map<std::string, std::string>& strMap) const = 0;

protected:
    GeneralParameters m_general; /**< General parameters */
    CalcSpeedFunct m_calcSpeedFunct = CalcSpeedDefault; /**< Function used to compute the machine speed when driving in the field */
    GridmapField_t m_gridmap_field; /**< Gridmap containing the information of the field geometries (boundary, tracks, obstacles) */
    GridmapRestrictedAreas_t m_gridmap_restrictedAreas; /**< Gridmap containing the information of restricted areas (worked/unworked areas) */
    GridmapVisits_t m_gridmap_visits; /**< Gridmap containing the information of visits of machines to an area */
    std::map<MachineId_t, std::set< VisitPeriod::DelayInfo > > m_delays; /**< Delay information for each machine */
    std::vector<Obstacle> m_obstacles; /**< Obstacles */
};

//------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------PBCC_timeOpt---------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------

/**
 * @brief Cost calculator for time-optimization
 */
class PBCC_timeOpt : public IPoseBasedCostCalculator{
public:

    /**
     * @brief Constructor.
     * @param parentLogger Parent logger
     */
    explicit PBCC_timeOpt(Logger* parentLogger = nullptr);

    /**
     * @brief Compute the cost for a path (driving inside the field)
     * @sa IPoseBasedCostCalculator::calcCost
     */
    virtual CostExtended_t calcCost( const std::vector<Point>& path, const Machine& m, double time_start, double bunkerMass, const std::set<MachineId_t>& restrictedMachines ) const override;

    /**
     * @brief Compute the cost for a path (driving outside the field)
     * @sa IPoseBasedCostCalculator::calcCost
     */
    virtual CostExtended_t calcCost( const PoseBasedGraph::OutFieldCosts& OFCosts, const Machine& m, double time_start, double bunkerMass ) const override;

    /**
     * @brief Compute the heuristic cost from one vertex to another
     * @sa IPoseBasedCostCalculator::calcHeuristic
     */
    virtual Cost_t calcHeuristic( const Vertex_t& start_vt, const Vertex_t& goal_vt, const Machine& m ) const override;

    /**
     * @brief Compute the heuristic cost from one vertex to another
     * @sa IPoseBasedCostCalculator::calcHeuristic
     */
    virtual Cost_t calcHeuristic( const PoseBasedGraph& graph, const Vertex_t& start_vt, const Vertex_t& goal_vt, const Machine& m  ) const override;



protected:

    /**
     * @brief Set other specific parameters from a string map containing the values as string.
     * @sa IPoseBasedCostCalculator::parseOtherParametersFromStringMap
     */
    virtual bool parseOtherParametersFromStringMap(const std::map<std::string, std::string>& strMap, bool strict) override;

    /**
     * @brief Parse other specific parameters and append them to the string map .
     * @sa IPoseBasedCostCalculator::parseAndAppendOtherParametersToStringMap
     */
    virtual void parseAndAppendOtherParametersToStringMap(std::map<std::string, std::string>& strMap) const override;

};


}

#endif // ARO_POSEBASEDCOSTCALCULATOR_HPP
