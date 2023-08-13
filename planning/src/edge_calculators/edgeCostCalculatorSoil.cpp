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
 

#include "arolib/planning/edge_calculators/edgeCostCalculatorSoil.hpp"

namespace arolib{

//----------------------------------------------ECC_soilOptimization----------------------------------------------------


const std::string ECC_soilOptimization1::SoilMapName = "SOIL";
const std::set<DirectedGraph::EdgeType> ECC_soilOptimization1::m_noSoilCostTypes = { DirectedGraph::FAP_TO_RP,
                                                                                     DirectedGraph::RP_TO_FAP,
                                                                                     DirectedGraph::FAP_TO_FAP,
                                                                                     DirectedGraph::EdgeType::INIT };

bool ECC_soilOptimization1::CostCoefficients::parseFromStringMap(const std::map<std::string, std::string> &strMap, bool strict)
{
    ECC_soilOptimization1::CostCoefficients tmp = *this;

    std::map<std::string, double*> dMap = { {"K_prevWeights" , &tmp.K_prevWeights},
                                            {"K_bias" , &tmp.K_bias},
                                            {"K_soil" , &tmp.K_soil},
                                            {"Kpow_soil" , &tmp.Kpow_soil},
                                            {"K_time" , &tmp.K_time},
                                            {"K_outside" , &tmp.K_outside} };

    if( !setValuesFromStringMap( strMap, dMap, strict) )
        return false;

    *this = tmp;
    return true;
}

std::map<std::string, std::string> ECC_soilOptimization1::CostCoefficients::parseToStringMap() const
{
    std::map<std::string, std::string> ret;
    ret["K_prevWeights"] = double2string( K_prevWeights );
    ret["K_bias"] = double2string( K_bias );
    ret["K_soil"] = double2string( K_soil );
    ret["Kpow_soil"] = double2string( Kpow_soil );
    ret["K_time"] = double2string( K_time );
    ret["K_outside"] = double2string( K_outside );
    return ret;
}

ECC_soilOptimization1::ECC_soilOptimization1(const LogLevel &logLevel):
    IEdgeCostCalculator(__FUNCTION__, logLevel),
    m_gridsManager(logLevel)
{
    m_general.crossCostMult = 3.0;
    m_general.headlandCrossCostMult = 0;
    m_general.boundaryCrossCostMult = 12;

    m_gridsManager.logger().setParent(loggerPtr());
}

ECC_soilOptimization1::ECC_soilOptimization1(const ECC_soilOptimization1& other):
    IEdgeCostCalculator(__FUNCTION__, other.logger().logLevel())
{
    m_costCoefficients = other.m_costCoefficients;
    m_boundary = other.m_boundary;
    m_gridsManager = other.m_gridsManager;
    m_mapPrecision = other.m_mapPrecision;
    m_edgeSoilCosts = other.m_edgeSoilCosts;
    m_soilValuesKey = other.m_soilValuesKey;
}

ECC_soilOptimization1::ECC_soilOptimization1(const ECC_soilOptimization1::CostCoefficients &costCoefficients, const LogLevel &logLevel):
    IEdgeCostCalculator(__FUNCTION__, logLevel),
    m_costCoefficients(costCoefficients)
{

}

ECC_soilOptimization1::ECC_soilOptimization1(const ECC_soilOptimization1::CostCoefficients &costCoefficients, const Polygon &boundary, const LogLevel &logLevel):
    IEdgeCostCalculator(__FUNCTION__, logLevel),
    m_costCoefficients(costCoefficients),
    m_boundary(boundary)
{

}

void ECC_soilOptimization1::generateInternalParameters(DirectedGraph::Graph& graph)
{
    std::lock_guard<std::mutex> lg(m_mutex);

    m_edgeSoilCosts.clear();

    if( !m_gridsManager.hasGrid(SoilMapName) ){
        logger().printWarning(__FUNCTION__, "No soil-cost mas hap been set.");
        return;
    }

    m_soilValuesKey = DirectedGraph::edge_property::getNewCustomValueKey();

    std::unordered_map<std::string, double> revEdges;
    for(DirectedGraph::edge_iter ed = edges(graph); ed.first != ed.second; ed.first++){
        DirectedGraph::edge_property& edge_prop = graph[*ed.first];
        auto edgeStr = edge2string(*ed.first);

        auto it_revEdge = revEdges.find(edgeStr);
        if(it_revEdge != revEdges.end()){
            edge_prop.customValues[m_soilValuesKey] = it_revEdge->second;
            continue;
        }

        double soilCost = 0;

        if ( m_noSoilCostTypes.find(edge_prop.edge_type) == m_noSoilCostTypes.end() ){

            std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;
            double width = std::max(0.0, edge_prop.defWidth);
            double distance = arolib::geometry::calc_dist(edge_prop.p0, edge_prop.p1);
            double area = distance * width;

            if(area > 1e-9){
                m_gridsManager.getCellsInfoUnderLine(SoilMapName, edge_prop.p0, edge_prop.p1, width, m_mapPrecision, cellsInfo);

                bool errorTmp;
                soilCost = m_gridsManager.getGrid(SoilMapName)->getCellsComputedValue(cellsInfo,
                                                                                      ArolibGrid_t::AVERAGE_TOTAL,
                                                                                      area,
                                                                                      false,
                                                                                      &errorTmp );
                if(errorTmp)
                    soilCost = 0;
            }
        }

        edge_prop.customValues[m_soilValuesKey] = soilCost;

        if(edge_prop.bidirectional)
            revEdges[edge2string(edge_prop.revEdge)] = soilCost;
    }
}

double ECC_soilOptimization1::calcCost(const Machine &machine, const Point &p1, const Point &p2, double time, double waitingTime, double bunkerMass, const std::vector<DirectedGraph::overroll_property> &overruns)
{

    DirectedGraph::edge_property edge_prop;
    edge_prop.edge_type = DirectedGraph::EdgeType::DEFAULT;
    edge_prop.p0 = p1;
    edge_prop.p1 = p2;
    edge_prop.distance = arolib::geometry::calc_dist(p1, p2);
    edge_prop.defWidth = std::max(0.0, machine.width > 1e-9 ? machine.width : machine.working_width);
    return calc_cost(machine,
                     "",
                     edge_prop,
                     time,
                     waitingTime,
                     std::max(0.0, machine.weight) + std::max(0.0, bunkerMass),
                     overruns);


}

double ECC_soilOptimization1::calcCost(const Machine &machine, const DirectedGraph::edge_t &edge, const DirectedGraph::edge_property& edge_prop, double time, double waitingTime, double bunkerMass)
{
    return calc_cost(machine,
                     edge2string(edge),
                     edge_prop,
                     time,
                     waitingTime,
                     std::max(0.0, machine.weight) + std::max(0.0, bunkerMass),
                     edge_prop.overruns);
}

double ECC_soilOptimization1::calcHeuristic(const Machine &machine, const DirectedGraph::vertex_property &v_prop_current, const DirectedGraph::vertex_property &v_prop_goal)
{
    bool inPoly_c = true, inPoly_g = true;
    if(m_boundary.points.size() > 3){
        inPoly_c = arolib::geometry::in_polygon(v_prop_current.route_point, m_boundary);
        inPoly_g = arolib::geometry::in_polygon(v_prop_goal.route_point, m_boundary);
    }

    double distance = arolib::geometry::calc_dist( v_prop_current.route_point, v_prop_goal.route_point );
    double time = 0;
    if(machine.max_speed_empty != 0)
        time = distance / std::fabs(machine.max_speed_empty);

    if(inPoly_c == inPoly_g){
        DirectedGraph::edge_property edge_prop;
        edge_prop.edge_type = inPoly_c ? DirectedGraph::EdgeType::DEFAULT : DirectedGraph::EdgeType::FAP_TO_RP;
        edge_prop.distance = distance;
        edge_prop.defWidth = 0;
        edge_prop.customValues[m_soilValuesKey] = 0;
        return calc_cost(machine,
                         "",
                         edge_prop,
                         time,
                         0,
                         std::max(0.0, machine.weight),
                         {});
    }

    double distanceIF = 0;//minimum distance that the machine would need to enter/leave the bounday
    if(inPoly_c)
        distanceIF = arolib::geometry::calc_dist_to_linestring(m_boundary.points, v_prop_current.route_point);
    else
        distanceIF = arolib::geometry::calc_dist_to_linestring(m_boundary.points, v_prop_goal.route_point);

    double timeIF = 0;
    if(distance > 0)
        timeIF = time * distanceIF / distance;

    DirectedGraph::edge_property epIF, epOF;
    epIF.edge_type = DirectedGraph::EdgeType::DEFAULT;
    epIF.distance = std::max(0.0, distanceIF);
    epIF.defWidth = 0;
    epIF.customValues[m_soilValuesKey] = 0;

    epOF.edge_type = DirectedGraph::EdgeType::FAP_TO_RP;
    epOF.distance = std::max(0.0, distance - distanceIF);
    epOF.defWidth = 0;
    epOF.customValues[m_soilValuesKey] = 0;

    return   calc_cost(machine,
                       "",
                       epIF,
                       timeIF,
                       0,
                       std::max(0.0, machine.weight),
                       {})//for infield segment
            + calc_cost(machine,
                        "",
                        epOF,
                        std::max(0.0, time - timeIF),
                        0,
                        std::max(0.0, machine.weight),
                        {});//for outfield segment
}

void ECC_soilOptimization1::setCostCoefficients(const ECC_soilOptimization1::CostCoefficients &costCoefficients)
{
    m_costCoefficients = costCoefficients;
}

void ECC_soilOptimization1::setBoundary(const Polygon &boundary)
{
    m_boundary = boundary;
}

void ECC_soilOptimization1::setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim)
{
    m_gridsManager.setCellsInfoManager(cim, true);
}

bool ECC_soilOptimization1::setSoilCostMap(std::shared_ptr<const ArolibGrid_t> map)
{
    if(!map || !map->isAllocated()){
        if(m_gridsManager.hasGrid(SoilMapName) && !m_gridsManager.removeGrid(SoilMapName)){
            logger().printError(__FUNCTION__, "Error removing soil cost map from grids manager");
            //return false;
        }
        return true;
    }
    return m_gridsManager.addGrid(SoilMapName, map, true);
}

void ECC_soilOptimization1::setMapComputationPrecision(gridmap::SharedGridsManager::PreciseCalculationOption precise)
{
    m_mapPrecision = precise;
}

bool ECC_soilOptimization1::parseOtherParametersFromStringMap(const std::map<std::string, std::string> &strMap, bool strict)
{
    return m_costCoefficients.parseFromStringMap(strMap, strict);
}

void ECC_soilOptimization1::parseAndAppendOtherParametersToStringMap(std::map<std::string, std::string> &strMap) const
{
    auto tmp = m_costCoefficients.parseToStringMap();
    strMap.insert( tmp.begin(), tmp.end() );
}

double ECC_soilOptimization1::getSoilCost(const std::string &edgeStr, const Point &p1, const Point &p2, double width)
{

    if( !edgeStr.empty() ){
        auto it_edge = m_edgeSoilCosts.find(edgeStr);
        if(it_edge != m_edgeSoilCosts.end())
            return it_edge->second;
    }

    //return 0;

    double area = arolib::geometry::calc_dist(p1, p2) * width;

    if( area > 1e-9 && m_gridsManager.hasGrid(SoilMapName) ){
        std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;
        m_gridsManager.getCellsInfoUnderLine(SoilMapName, p1, p2, width, m_mapPrecision, cellsInfo);

        bool errorTmp;
        double soilCost = m_gridsManager.getGrid(SoilMapName)->getCellsComputedValue(cellsInfo,
                                                                                     ArolibGrid_t::AVERAGE_TOTAL,
                                                                                     area,
                                                                                     false,
                                                                                     &errorTmp );
        if(!errorTmp)
            return soilCost;
    }
    return 0;
}

double ECC_soilOptimization1::getSoilCost(const DirectedGraph::edge_property &edge_prop)
{
    //return edge_prop.soilValue;

    const auto it_soliValue = edge_prop.customValues.find(m_soilValuesKey);
    if( it_soliValue != edge_prop.customValues.end() )
        return it_soliValue->second;

    double soilCost = 0;
    double area = arolib::geometry::calc_dist(edge_prop.p0, edge_prop.p1) * edge_prop.defWidth;

    if( area > 1e-9 && m_gridsManager.hasGrid(SoilMapName) ){
        std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;
        m_gridsManager.getCellsInfoUnderLine(SoilMapName, edge_prop.p0, edge_prop.p1, edge_prop.defWidth, m_mapPrecision, cellsInfo);

        bool errorTmp;
        soilCost = m_gridsManager.getGrid(SoilMapName)->getCellsComputedValue(cellsInfo,
                                                                              ArolibGrid_t::AVERAGE_TOTAL,
                                                                              area,
                                                                              false,
                                                                              &errorTmp );

        if(errorTmp)
            soilCost = 0;
    }

    return soilCost;

}

double ECC_soilOptimization1::calc_cost(const Machine& machine, const std::string &edgeStr, const DirectedGraph::edge_property &edge_prop, double time, double waitingTime, double mass, const std::vector<DirectedGraph::overroll_property> &overruns)
{
    std::lock_guard<std::mutex> lg(m_mutex);

    double distance = edge_prop.distance > -1e-9 ? std::max(0.0, edge_prop.distance) : arolib::geometry::calc_dist(edge_prop.p0, edge_prop.p1);

    if(edge_prop.edge_type == DirectedGraph::FAP_TO_RP
            || edge_prop.edge_type == DirectedGraph::RP_TO_FAP
            || edge_prop.edge_type == DirectedGraph::FAP_TO_FAP){//get the data data from the out-of-field information
        OutFieldInfo::MachineBunkerState bunkerState = OutFieldInfo::MACHINE_LOADED;
        if(mass-std::max(0.0, machine.weight) < 1e-9 + 0.4*std::max(0.0, machine.bunker_mass) )
            bunkerState = OutFieldInfo::MACHINE_EMPTY;
        OutFieldInfo::TravelCosts tc;
        if( OutFieldInfo::getTravelCost( edge_prop.travelCosts, machine.id, bunkerState, tc ) ){
            if(tc.distance > -1e-9)
                distance = tc.distance;
        }

    }
//    else if( edge_prop.edge_type == DirectedGraph::INIT ){//get the data data from the out-of-field information
//        if(edge_prop.arrivalCosts.distance > -1e-9)
//            distance = edge_prop.arrivalCosts.distance;
//    }

    double weight_sum = 0.0;
    double soilCost = 0.0;
    double k_overall = 1.0;

    if ( m_noSoilCostTypes.find(edge_prop.edge_type) != m_noSoilCostTypes.end() ){
        //return 0.1 * distance + 0.1 * time;//no soil to be perserved, yet shorter paths are prefered
        k_overall = m_costCoefficients.K_outside;
    }
    else{
        for (auto &o : overruns)
            weight_sum += o.weight;
        weight_sum *= m_costCoefficients.K_prevWeights;  // influence of previous overruns is reduced

        //soilCost = getSoilCost(edgeStr, edge_prop.p0, edge_prop.p1, edge_prop.defWidth);
        soilCost = getSoilCost(edge_prop);
    }

    double speed_inv = time;
    distance = std::fabs(distance);
    if(distance > 1e-3)
        speed_inv /= distance;

    weight_sum += std::max(0.0, mass);


    double cost = distance * weight_sum * (m_costCoefficients.K_bias
                                           + std::pow(soilCost,  m_costCoefficients.Kpow_soil) * m_costCoefficients.K_soil
                                           + speed_inv * m_costCoefficients.K_time);
    // higher cost for crossing
    if (edge_prop.edge_type == DirectedGraph::EdgeType::CROSS)
        cost *= (1 + m_general.crossCostMult);
    else if(edge_prop.edge_type == DirectedGraph::EdgeType::CROSS_HL)
        cost *= (1 + m_general.headlandCrossCostMult);
    else if(edge_prop.edge_type == DirectedGraph::EdgeType::BOUNDARY_CONN)
        cost *= (1 + m_general.boundaryCrossCostMult);
    else
        cost *= k_overall;

    //just checking for nan values
    if( std::isnan(cost) )
        logger().printOut( LogLevel::CRITIC, __FUNCTION__, 10,
                           "Cost is NAN!",
                           "\n\t cost = ", cost,
                           "\n\t weight_sum = ", weight_sum,
                           "\n\t distance = ", distance,
                           "\n\t speed_inv = ", speed_inv );

    return cost;

}

std::string ECC_soilOptimization1::edge2string(const DirectedGraph::edge_t &edge)
{
    std::stringstream ss;
    ss << edge;
    return ss.str();
}





}

