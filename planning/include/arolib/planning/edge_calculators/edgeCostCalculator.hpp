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
 
#ifndef ARO_EDGECOSTCALCULATOR_HPP
#define ARO_EDGECOSTCALCULATOR_HPP

#include <memory>
#include <functional>
#include <unordered_map>
#include <sstream>

#include "arolib/planning/planningworkspace.h"
#include "arolib/planning/path_search/directedgraph.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/misc/basicconversions.hpp"
#include "arolib/cartography/sharedgridsmanager.hpp"

namespace arolib{

/**
 * @brief Interface class used to compute the cost in a graph edge
 */
class IEdgeCostCalculator : public virtual LoggingComponent, public IMappableParameters
{
public:

    /**
     * @brief General EdgeCostCalculator parameters
     */
    struct GeneralParameters : public IMappableParameters{
        double crossCostMult = 1; /**< Coefficient for infield 'crossing' edges >*/
        double headlandCrossCostMult = 0; /**< Coefficient for headland 'crossing' edges >*/
        double boundaryCrossCostMult = 5; /**< Coefficient for 'boundary-crossing' edges >*/

        /**
         * @brief Updates the parameters with the values parsed from a string map
         * @param strMap String map containing the parameter values
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
    explicit IEdgeCostCalculator(const LogLevel& logLevel = LogLevel::INFO);


    /**
     * @brief Generates internal parameters from the graph, e.g. edge parameters to be used in the cost calculation
     *
     * Should be called after all other parameters have been set and when the complete graph has been constructed. Removing edges/vertices from the later on will cause undesired calculations.
     * @param logLevel Log level
     */
    virtual void generateInternalParameters(DirectedGraph::Graph& graph) = 0;

    /**
     * @brief Calculate the edge cost
     * @param machine Machine transversing the edge
     * @param p1 First point of the edge (corresoinding to the start vertex)
     * @param p2 Second point of the edge (corresoinding to the end vertex)
     * @param time Time spent transversing the edge [s]
     * @param waitingTime Part of 'time' spent waiting [s]
     * @param bunkerMass Bunker mass [Kg]
     * @param overruns Overruns
     * @return Edge cost
     */
    virtual double calcCost(const Machine& machine,
                            const Point& p1,
                            const Point& p2,
                            double time,
                            double waitingTime,
                            double bunkerMass,
                            const std::vector<DirectedGraph::overroll_property>& overruns) = 0;

    /**
     * @brief Calculate the edge cost from a known edge
     * @param machine Machine transversing the edge
     * @param edge Edge properties
     * @param edge_prop Edge properties
     * @param time Time spent transversing the edge [s]
     * @param waitingTime Part of 'time' spent waiting [s]
     * @param bunkerMass Bunker mass [Kg]
     * @return Edge cost
     */
    virtual double calcCost(const Machine& machine,
                            const DirectedGraph::edge_t& edge,
                            const DirectedGraph::edge_property& edge_prop,
                            double time,
                            double waitingTime,
                            double bunkerMass) = 0;


    /**
     * @brief Calculate the heuristic (for A* search)
     * @brief Calculate the edge cost
     * @param machine Machine
     * @param v_prop_current Vertex info of the current node
     * @param v_prop_goal Vertex info of the goal node
     * @return Edge cost
     */
    virtual double calcHeuristic(const Machine& machine,
                                 const DirectedGraph::vertex_property &v_prop_current,
                                 const DirectedGraph::vertex_property &v_prop_goal) = 0;

    /**
     * @brief Set the general parameters
     * @param params general parameters
     */
    virtual void setGeneralParameters(const GeneralParameters& params);

    /**
     * @brief Set the general parameters
     * @param params general parameters
     */
    virtual GeneralParameters getGeneralParameters() const;

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
             typename = typename std::enable_if< std::is_base_of<IEdgeCostCalculator, T>::value >::type>
    bool isOfType() const {
        return typeid(*this) == typeid(T);
    }


protected:
    /**
     * @brief Constructor.
     * @param logLevel Log level
     * @param childName Child class
     */
    explicit IEdgeCostCalculator(const std::string childName, const LogLevel &logLevel = LogLevel::INFO);

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
    GeneralParameters m_general; /**< General parameters >*/
};


//----------------------------------------------ECC_CustomEdgeCostCalculator----------------------------------------------------

/**
 * @brief Custom Edge-Cost Calculator
 */
class CustomEdgeCostCalculator : public IEdgeCostCalculator
{
public:
    using CalcCostFunc1 = std::function< double  ( const Machine&,
                                                   const Point&,
                                                   const Point&,
                                                   double,
                                                   double,
                                                   double,
                                                   const std::vector<DirectedGraph::overroll_property>&) >;
    using CalcCostFunc2 = std::function< double  ( const Machine&,
                                                   const DirectedGraph::edge_t&,
                                                   const DirectedGraph::edge_property&,
                                                   double,
                                                   double,
                                                   double ) >;
    using CalcHeuristicFunc = std::function< double  ( const Machine&,
                                                       const DirectedGraph::vertex_property &,
                                                       const DirectedGraph::vertex_property & ) >;
    using GenInternalParamsFun = std::function< void  ( const DirectedGraph::Graph& ) >;

    /**
     * @brief Constructor.
     * @param _calcCost Function to be called when IEdgeCostCalculator::calcCost (version 1) is called
     * @param _calcCost2 Function to be called when IEdgeCostCalculator::calcCost (version 2) is called
     * @param _calcHeuristic Function to be called when IEdgeCostCalculator::calcHeuristic is called
     * @param _genInternalParams Function to be called when IEdgeCostCalculator::generateInternalParameters is called
     * @param logLevel Log level
     */
    explicit CustomEdgeCostCalculator(const CalcCostFunc1& _calcCost1,
                                      const CalcCostFunc2& _calcCost2,
                                      const CalcHeuristicFunc& _calcHeuristic,
                                      const GenInternalParamsFun& _genInternalParams,
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
     */
    virtual double calcHeuristic(const Machine& machine,
                                 const DirectedGraph::vertex_property &v_prop_current,
                                 const DirectedGraph::vertex_property &v_prop_goal) override;


protected:

    /**
     * @brief Set other specific parameters from a string map containing the values as string.
     * @sa IEdgeCostCalculator::parseOtherParametersFromStringMap
     */
    virtual bool parseOtherParametersFromStringMap(const std::map<std::string, std::string>& , bool ) override { return true; }

    /**
     * @brief Parse other specific parameters and append them to the string map .
     * @sa IEdgeCostCalculator::parseAndAppendOtherParametersToStringMap
     */
    virtual void parseAndAppendOtherParametersToStringMap(std::map<std::string, std::string>& strMap) const override {}

    CalcCostFunc1 m_calcCost1; /**< Function to be called when IEdgeCostCalculator::calcCost (version 1) is called >*/
    CalcCostFunc2 m_calcCost2; /**< Function to be called when IEdgeCostCalculator::calcCost (version 2) is called >*/
    CalcHeuristicFunc m_calcHeuristic; /**< Function to be called when IEdgeCostCalculator::calcHeuristic is called >*/
    GenInternalParamsFun m_genInternalParams; /**< Function to be called when IEdgeCostCalculator::generateInternalParameters is called >*/
};



//----------------------------------------------ECC_timeOptimization----------------------------------------------------
/**
 * @brief Default class used to compute the edge cost following time optimization
 */
class ECC_timeOptimization : public IEdgeCostCalculator
{
public:

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit ECC_timeOptimization(const LogLevel& logLevel = LogLevel::INFO);

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
                            double bunkerMass, const std::vector<DirectedGraph::overroll_property>&) override;

    /**
     * @brief Compute the edge cost.
     * @sa IEdgeCostCalculator::calcCost
     */
    virtual double calcCost(const Machine&,
                            const DirectedGraph::edge_t& edge,
                            const DirectedGraph::edge_property& edge_prop,
                            double time,
                            double waitingTime,
                            double bunkerMass) override;

    /**
     * @brief Calculate the heuristic (for A* search)
     * @sa IEdgeCostCalculator::calcHeuristic
     * @param bestSpeed Maxinum speed throuout the search
     * @param bestWeight Disregarded
     */
    virtual double calcHeuristic(const Machine& machine,
                                 const DirectedGraph::vertex_property &v_prop_current,
                                 const DirectedGraph::vertex_property &v_prop_goal) override;
protected:

    /**
     * @brief Set other specific parameters from a string map containing the values as string.
     * @sa IEdgeCostCalculator::parseOtherParametersFromStringMap
     */
    virtual bool parseOtherParametersFromStringMap(const std::map<std::string, std::string>& , bool ) override { return true; }

    /**
     * @brief Parse other specific parameters and append them to the string map .
     * @sa IEdgeCostCalculator::parseAndAppendOtherParametersToStringMap
     */
    virtual void parseAndAppendOtherParametersToStringMap(std::map<std::string, std::string>& strMap) const override {}

};


//----------------------------------------------ECC_distanceOptimization----------------------------------------------------
/**
 * @brief Default class used to compute the edge cost following time optimization
 */
class ECC_distanceOptimization : public IEdgeCostCalculator
{
public:

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit ECC_distanceOptimization(const LogLevel& logLevel = LogLevel::INFO);

    /**
     * @brief Generates internal parameters from the graph, e.g. edge parameters to be used in the cost calculation
     * @sa generateInternalParameters::calcCost
     */
    virtual void generateInternalParameters(DirectedGraph::Graph& graph) override;

    /**
     * @brief Compute the edge cost for a default edge based on its start/end points' locations.
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
     * @brief Compute the edge cost for a known edge.
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
     * @param bestWeight Disregarded
     */
    virtual double calcHeuristic(const Machine& machine,
                                 const DirectedGraph::vertex_property &v_prop_current,
                                 const DirectedGraph::vertex_property &v_prop_goal) override;
protected:


    /**
     * @brief Set other specific parameters from a string map containing the values as string.
     * @sa IEdgeCostCalculator::parseOtherParametersFromStringMap
     */
    virtual bool parseOtherParametersFromStringMap(const std::map<std::string, std::string>& , bool ) override { return true; }

    /**
     * @brief Parse other specific parameters and append them to the string map .
     * @sa IEdgeCostCalculator::parseAndAppendOtherParametersToStringMap
     */
    virtual void parseAndAppendOtherParametersToStringMap(std::map<std::string, std::string>& strMap) const override {}

};

}

#endif // ARO_EDGECOSTCALCULATOR_HPP
