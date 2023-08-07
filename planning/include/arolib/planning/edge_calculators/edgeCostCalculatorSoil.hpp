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
 
#ifndef ARO_EDGECOSTCALCULATORSOIL_HPP
#define ARO_EDGECOSTCALCULATORSOIL_HPP

#include <memory>
#include <functional>
#include <unordered_map>
#include <sstream>

#include "arolib/planning/edge_calculators/edgeCostCalculator.hpp"

namespace arolib{


//----------------------------------------------ECC_soilOptimization----------------------------------------------------

/**
 * @brief Default class used to compute the edge cost following soil optimization
 *
 * cost = K * distance * ( prev_weights_sum * K_prevWeights + current_weight )
 *                     * ( K_bias + soilValue ^ Kpow_soil * K_soil + K_time / speed );
 *
 * K = (1 + crossCostMult)            if    edge_type == EdgeType::CROSS
 * K = (1 + headlandCrossCostMult)    if    edge_type == EdgeType::CROSS_HL
 * K = (1 + boundaryCrossCostMult)    if    edge_type == EdgeType::BOUNDARY_CONN
 * K = K_outside                      if    edge is for transit outside of the field
 * K = 1                              if    otherwise
 *
 * prev_weights_sum = soilValue = 0   if    edge is for transit outside of the field
 */
class ECC_soilOptimization1 : public IEdgeCostCalculator
{
public:

    /**
     * @brief Coefficients for the cost equation
     *
     * soil optimization - coefficient examples (soilOpt_biasCoef, Default_SoilOpt_SoilValueCoef, Default_SoilOpt_TimeCoef) with K_prevWeights = 0.5 and Kpow_soil = 2
     *    ( 0.2, 3, 0 ) : balances behaviour between overruns/edge and trying not to go over bad soil. the olv will rather wait to achieve this, yielding higher harvesting values
     *    (0.01, 3, 0 ) : will try to avoid at all costs passing over bad-soil areas, regardless the harvesting time and distance driven by the olvs, and allowing several overruns in the good-soil edges
     *    ( 0.2, 3, 10) : gives higher importance on the time spent in the edges, hence prefering time-efficient paths over protecting the bad-soil areas or controlling overruns
     */
    struct CostCoefficients : public IMappableParameters{
        double K_prevWeights = 0.5;/**< Multiplier to the sum of weights of previous driving */
        double K_bias = 0.2; /**< Bias coefficient */
        double K_soil = 3; /**< Soil value coefficient */
        double Kpow_soil = 2; /**< Soil value power coefficient */
        double K_time = 0.01; /**< Time coefficient */
        double K_outside = 0.5; /**< Coefficient */

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
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit ECC_soilOptimization1(const LogLevel& logLevel = LogLevel::INFO);

    // copy constructor - not created automatically because of m_mutex; required for pyarolib
    explicit ECC_soilOptimization1(const ECC_soilOptimization1& other);

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit ECC_soilOptimization1(const CostCoefficients& costCoefficients,
                                  const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit ECC_soilOptimization1(const CostCoefficients& costCoefficients,
                                  const Polygon& boundary,
                                  const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Generates internal parameters from the graph, e.g. edge parameters to be used in the cost calculation
     * @sa generateInternalParameters::calcCost
     */
    virtual void generateInternalParameters(DirectedGraph::Graph& graph) override;

    /**
     * @brief Compute the edge cost.
     * @sa IEdgeCostCalculator::calcCost
     */
    virtual double calcCost(const Machine& machine,
                            const Point& p1,
                            const Point& p2,
                            double time,
                            double waitingTime,
                            double bunkerMass,
                            const std::vector<DirectedGraph::overroll_property>& overruns) override;

    /**
     * @brief Compute the edge cost.
     * @sa IEdgeCostCalculator::calcCost
     */
    virtual double calcCost(const Machine& machine,
                            const DirectedGraph::edge_t& edge,
                            const DirectedGraph::edge_property& edge_prop,
                            double time,
                            double waitingTime,
                            double bunkerMass) override;

    /**
     * @brief Calculate the heuristic (for A* search)
     * @sa IEdgeCostCalculator::calcHeuristic
     * @param bestSpeed Maxinum speed throuout the search
     * @param bestWeight Minimum weight throuout the search
     */
    virtual double calcHeuristic(const Machine& machine,
                                 const DirectedGraph::vertex_property &v_prop_current,
                                 const DirectedGraph::vertex_property &v_prop_goal) override;

    /**
     * @brief Set the cost coefficients
     * @param costCoefficients Cost coefficients
     */
    virtual void setCostCoefficients(const CostCoefficients& costCoefficients);

    /**
     * @brief Set the field boundary
     * @param boundary Boundary (if empty, disregarded)
     */
    virtual void setBoundary(const Polygon& boundary);

    /**
     * @brief Set shared CellsInfoManager to record shared edge cells data
     * @param cim CellsInfoManager. If null, no recording will be done
     */
    virtual void setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim);

    /**
     * @brief Set the field soil-cost map
     * @param map Soil-cost map (if null, previous saved cost map will be removed)
     * @return True on success
     */
    virtual bool setSoilCostMap(std::shared_ptr<const ArolibGrid_t> map);

    /**
     * @brief Set map-computations precision
     * @param precise Precise calculation option
     */
    virtual void setMapComputationPrecision(gridmap::SharedGridsManager::PreciseCalculationOption precise);

protected:
    /**
     * @brief Set other specific parameters from a string map containing the values as string.
     * @sa IEdgeCostCalculator::parseOtherParametersFromStringMap
     */
    virtual bool parseOtherParametersFromStringMap(const std::map<std::string, std::string>& strMap, bool strict ) override;

    /**
     * @brief Parse other specific parameters and append them to the string map .
     * @sa IEdgeCostCalculator::parseAndAppendOtherParametersToStringMap
     */
    virtual void parseAndAppendOtherParametersToStringMap(std::map<std::string, std::string>& strMap) const override;

    /**
     * @brief Get the soil-cost of an edge
     *
     * If the cost for the given Edge (string) id is knows, it retuns it; otherwise it computes it using the edges points and width and saves+returns it
     * @param Edge string id
     * @param p1 Point 1 of the edge
     * @param p2 Point 2 of the edge
     * @param width Width of the edge
     * @return Soil-cost
     */
    double getSoilCost(const std::string &edgeStr,
                       const Point& p1,
                       const Point& p2,
                       double width);

    /**
     * @brief Get the soil-cost of an edge from its edge-property
     * @param edge_prop Edge-property
     * @return Soil-cost
     */
    double getSoilCost(const DirectedGraph::edge_property& edge_prop);

    /**
     * @brief Internal edge cost calculation
     * @param machine Machine
     * @param Edge string id
     * @param edge_prop Edge-property
     * @param time Time spent transversing the edge [s]
     * @param waitingTime Part of 'time' spent waiting [s]
     * @param mass Overall mass (machine + bunker mass) [Kg]
     * @param overruns Edge overruns
     * @return Edge cost
     */
    double calc_cost(const Machine &machine,
                     const std::string& edgeStr,
                     const DirectedGraph::edge_property& edge_prop,
                     double time,
                     double waitingTime,
                     double mass,
                     const std::vector<DirectedGraph::overroll_property> &overruns);

    /**
     * @brief Get an edge string Id
     * @param edge Edge
     * @return Edge string Id
     */
    static std::string edge2string( const DirectedGraph::edge_t& edge);

protected:
    std::mutex m_mutex; /**< Mutex >*/
    CostCoefficients m_costCoefficients; /**< Cost coefficients >*/
    Polygon m_boundary; /**< Boundary >*/
    gridmap::SharedGridsManager m_gridsManager; /**< Shared grids manager >*/
    gridmap::SharedGridsManager::PreciseCalculationOption m_mapPrecision = gridmap::SharedGridsManager::PRECISE; /**< Maps presicion calculation option >*/
    std::unordered_map<std::string, double> m_edgeSoilCosts; /**< Edge soil-costs >*/
    int m_soilValuesKey = -1; /**< Soil-values key for edge_prop::customValues >*/
    static const std::string SoilMapName; /**< Soil cost-map name >*/
    static const std::set<DirectedGraph::EdgeType> m_noSoilCostTypes; /**< Edge types with no soil-cost >*/

};

}

#endif // ARO_EDGECOSTCALCULATOR_HPP
