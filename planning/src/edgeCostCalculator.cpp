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
 

#include "arolib/planning/edgeCostCalculator.hpp"

namespace arolib{


bool IEdgeCostCalculator::GeneralParameters::parseFromStringMap(const std::map<std::string, std::string> &strMap, bool strict)
{
    IEdgeCostCalculator::GeneralParameters tmp = *this;

    try{
        std::map<std::string, double*> dMap = { {"crossCostMult" , &tmp.crossCostMult},
                                                {"boundaryCrossCostMult" , &tmp.boundaryCrossCostMult} };

        if( !setValuesFromStringMap( strMap, dMap, strict) )
            return false;

    } catch(...){ return false; }

    *this = tmp;

    return true;

}

std::map<std::string, std::string> IEdgeCostCalculator::GeneralParameters::parseToStringMap() const
{
    std::map<std::string, std::string> ret;
    ret["crossCostMult"] = double2string( crossCostMult );
    ret["boundaryCrossCostMult"] = double2string( boundaryCrossCostMult );
    return ret;
}

IEdgeCostCalculator::IEdgeCostCalculator(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

IEdgeCostCalculator::IEdgeCostCalculator(const std::string childName, const LogLevel &logLevel):
    LoggingComponent(logLevel, childName)
{

}

void IEdgeCostCalculator::setGeneralParameters(const IEdgeCostCalculator::GeneralParameters &params)
{
    m_general = params;
}

IEdgeCostCalculator::GeneralParameters IEdgeCostCalculator::getGeneralParameters() const
{
    return m_general;
}

bool IEdgeCostCalculator::parseFromStringMap(const std::map<std::string, std::string> &strMap, bool strict)
{
    if( !m_general.parseFromStringMap(strMap, strict) )
        return false;

    if( !parseOtherParametersFromStringMap(strMap, strict) )
        return false;

    return true;

}

std::map<std::string, std::string> IEdgeCostCalculator::parseToStringMap() const
{
    std::map<std::string, std::string> ret = m_general.parseToStringMap();
    parseAndAppendOtherParametersToStringMap(ret);
    return ret;
}


//----------------------------------------------ECC_CustomEdgeCostCalculator--------------------------------------------


CustomEdgeCostCalculator::CustomEdgeCostCalculator(const CalcCostFunc1 &_calcCost1,
                                                   const CalcCostFunc2 &_calcCost2,
                                                   const CalcHeuristicFunc &_calcHeuristic, const GenInternalParamsFun &_genInternalParams,
                                                   const LogLevel &logLevel):
    IEdgeCostCalculator(__FUNCTION__, logLevel)
  , m_calcCost1(_calcCost1)
  , m_calcCost2(_calcCost2)
  , m_calcHeuristic(_calcHeuristic)
  , m_genInternalParams(_genInternalParams)
{

}

void CustomEdgeCostCalculator::generateInternalParameters(DirectedGraph::Graph& graph)
{
    m_genInternalParams(graph);
}

double CustomEdgeCostCalculator::calcCost(const Machine &machine, const Point &p1, const Point &p2, double time, double waitingTime, double bunkerMass, const std::vector<DirectedGraph::overroll_property> &overruns)
{
    return m_calcCost1(machine, p1, p2, time, waitingTime, bunkerMass, overruns);
}

double CustomEdgeCostCalculator::calcCost(const Machine &machine, const DirectedGraph::edge_t &edge, const DirectedGraph::edge_property& edge_prop, double time, double waitingTime, double bunkerMass)
{
    return m_calcCost2(machine, edge, edge_prop, time, waitingTime, bunkerMass);
}

double CustomEdgeCostCalculator::calcHeuristic(const Machine &machine, const DirectedGraph::vertex_property &v_prop_current, const DirectedGraph::vertex_property &v_prop_goal)
{
    return m_calcHeuristic(machine, v_prop_current, v_prop_goal);
}



//----------------------------------------------ECC_timeOptimization----------------------------------------------------

ECC_timeOptimization::ECC_timeOptimization(const LogLevel &logLevel):
    IEdgeCostCalculator(__FUNCTION__, logLevel)
{
    m_general.crossCostMult = 25;
    m_general.boundaryCrossCostMult = 50;
}

void ECC_timeOptimization::generateInternalParameters(DirectedGraph::Graph& graph)
{

}

double ECC_timeOptimization::calcCost(const Machine &machine, const Point &p1, const Point &p2, double time, double waitingTime, double bunkerMass, const std::vector<DirectedGraph::overroll_property> &)
{
    DirectedGraph::edge_property edge_prop;
    edge_prop.edge_type = DirectedGraph::EdgeType::DEFAULT;
    edge_prop.p0 = p1;
    edge_prop.p1 = p2;
    edge_prop.distance = arolib::geometry::calc_dist(p1, p2);
    return calcCost(machine,
                    DirectedGraph::edge_t(),//not used
                    edge_prop,
                    time,
                    waitingTime,
                    bunkerMass);

}

double ECC_timeOptimization::calcCost(const Machine &machine, const DirectedGraph::edge_t &, const DirectedGraph::edge_property& edge_prop, double time, double waitingTime, double bunkerMass)
{
    const double multDist = 1e-4;
    double penalty = 0;
    if(edge_prop.edge_type == DirectedGraph::EdgeType::CROSS)
        penalty = (time - waitingTime) * m_general.crossCostMult;
    else if(edge_prop.edge_type == DirectedGraph::EdgeType::BOUNDARY_CONN)
        penalty = (time - waitingTime) * m_general.boundaryCrossCostMult;

    double distance = edge_prop.distance > -1e-9 ? std::max(0.0, edge_prop.distance) : arolib::geometry::calc_dist(edge_prop.p0, edge_prop.p1);
    if(edge_prop.edge_type == DirectedGraph::FAP_TO_RP
            || edge_prop.edge_type == DirectedGraph::RP_TO_FAP
            || edge_prop.edge_type == DirectedGraph::FAP_TO_FAP){//get the data data from the out-of-field information
        OutFieldInfo::MachineBunkerState bunkerState = OutFieldInfo::MACHINE_LOADED;
        if(bunkerMass < 1e-9 + 0.4*std::max(0.0, machine.bunker_mass) )
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

    double cost = time + penalty + multDist*distance;//a distance factor is added so that if two plans have similar time-costs, the shortest one will give lower costs

    //just checking for nan values
    if( std::isnan(cost) )
        m_logger.printOut( LogLevel::CRITIC, __FUNCTION__, 10,
                           "Cost is NAN!",
                           "\n\t cost = ", cost,
                           "\n\t time = ", time,
                           "\n\t penalty = ", penalty,
                           "\n\t multDist = ", multDist,
                           "\n\t distance = ", distance );
    return cost;

}

double ECC_timeOptimization::calcHeuristic(const Machine &machine, const DirectedGraph::vertex_property &v_prop_current, const DirectedGraph::vertex_property &v_prop_goal)
{
    double distance = arolib::geometry::calc_dist( v_prop_current.route_point, v_prop_goal.route_point );
    double time = 0;
    if(machine.max_speed_empty != 0)
        time = distance / std::fabs(machine.max_speed_empty);

    DirectedGraph::edge_property edge_prop;
    edge_prop.edge_type = DirectedGraph::EdgeType::DEFAULT;
    edge_prop.p0 = v_prop_current.route_point.point();
    edge_prop.p1 = v_prop_goal.route_point.point();
    edge_prop.distance = arolib::geometry::calc_dist(edge_prop.p0, edge_prop.p1);

    return calcCost( machine,
                     DirectedGraph::edge_t(),//not used
                     edge_prop,
                     time,
                     0,
                     std::max(0.0, machine.weight) );
}


//----------------------------------------------ECC_distanceOptimization----------------------------------------------------


ECC_distanceOptimization::ECC_distanceOptimization(const LogLevel &logLevel) :
    IEdgeCostCalculator(__FUNCTION__, logLevel)
{
    m_general.crossCostMult = 25;
    m_general.boundaryCrossCostMult = 50;
}

void ECC_distanceOptimization::generateInternalParameters(DirectedGraph::Graph& graph)
{

}

double ECC_distanceOptimization::calcCost(const Machine &machine, const Point &p1, const Point &p2, double time, double waitingTime, double bunkerMass, const std::vector<DirectedGraph::overroll_property> &overruns)
{
    DirectedGraph::edge_property edge_prop;
    edge_prop.edge_type = DirectedGraph::EdgeType::DEFAULT;
    edge_prop.p0 = p1;
    edge_prop.p1 = p2;
    edge_prop.distance = arolib::geometry::calc_dist(p1, p2);
    return calcCost(machine,
                    DirectedGraph::edge_t(),//not used
                    edge_prop,
                    time,
                    waitingTime,
                    bunkerMass);
}

double ECC_distanceOptimization::calcCost(const Machine &machine, const DirectedGraph::edge_t &edge, const DirectedGraph::edge_property& edge_prop, double time, double, double bunkerMass)
{
    const double multTime = 1e-4;


    double distance = edge_prop.distance > -1e-9 ? std::max(0.0, edge_prop.distance) : arolib::geometry::calc_dist(edge_prop.p0, edge_prop.p1);

    if(edge_prop.edge_type == DirectedGraph::FAP_TO_RP
            || edge_prop.edge_type == DirectedGraph::RP_TO_FAP
            || edge_prop.edge_type == DirectedGraph::FAP_TO_FAP){//get the data data from the out-of-field information
        OutFieldInfo::MachineBunkerState bunkerState = OutFieldInfo::MACHINE_LOADED;
        if(bunkerMass < 1e-9 + 0.4*std::max(0.0, machine.bunker_mass) )
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

    double penalty = 0;
    if(edge_prop.edge_type == DirectedGraph::EdgeType::CROSS)
        penalty = distance * m_general.crossCostMult;
    else if(edge_prop.edge_type == DirectedGraph::EdgeType::BOUNDARY_CONN)
        penalty = distance * m_general.boundaryCrossCostMult;

    double cost = distance + penalty + multTime*time;//a time factor is added so that if two plans have similar distance-costs, the shortest one will give lower costs

    //just checking for nan values
    if( std::isnan(cost) )
        m_logger.printOut( LogLevel::CRITIC, __FUNCTION__, 10,
                           "Cost is NAN!",
                           "\n\t cost = ", cost,
                           "\n\t time = ", time,
                           "\n\t penalty = ", penalty,
                           "\n\t multTime = ", multTime,
                           "\n\t distance = ", distance );
    return cost;
}

double ECC_distanceOptimization::calcHeuristic(const Machine &machine, const DirectedGraph::vertex_property &v_prop_current, const DirectedGraph::vertex_property &v_prop_goal)
{
    double distance = arolib::geometry::calc_dist( v_prop_current.route_point, v_prop_goal.route_point );
    double time = 0;
    if(machine.max_speed_empty != 0)
        time = distance / std::fabs(machine.max_speed_empty);

    DirectedGraph::edge_property edge_prop;
    edge_prop.edge_type = DirectedGraph::EdgeType::DEFAULT;
    edge_prop.p0 = v_prop_current.route_point.point();
    edge_prop.p1 = v_prop_goal.route_point.point();
    edge_prop.distance = arolib::geometry::calc_dist(edge_prop.p0, edge_prop.p1);

    return calcCost( machine,
                     DirectedGraph::edge_t(),//not used
                     edge_prop,
                     time,
                     0,
                     std::max(0.0, machine.weight) );
}

//----------------------------------------------ECC_soilOptimization----------------------------------------------------


const std::string ECC_soilOptimization1::SoilMapName = "SOIL";
const std::set<DirectedGraph::EdgeType> ECC_soilOptimization1::m_noSoilCostTypes = { DirectedGraph::FAP_TO_RP,
                                                                                     DirectedGraph::RP_TO_FAP,
                                                                                     DirectedGraph::FAP_TO_FAP,
                                                                                     DirectedGraph::EdgeType::INIT };

bool ECC_soilOptimization1::CostCoefficients::parseFromStringMap(const std::map<std::string, std::string> &strMap, bool strict)
{
    ECC_soilOptimization1::CostCoefficients tmp = *this;

    try{
        std::map<std::string, double*> dMap = { {"K_prevWeights" , &tmp.K_prevWeights},
                                                {"K_bias" , &tmp.K_bias},
                                                {"K_soil" , &tmp.K_soil},
                                                {"Kpow_soil" , &tmp.Kpow_soil},
                                                {"K_time" , &tmp.K_time} };

        if( !setValuesFromStringMap( strMap, dMap, strict) )
            return false;

    } catch(...){ return false; }

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
    return ret;
}

ECC_soilOptimization1::ECC_soilOptimization1(const LogLevel &logLevel):
    IEdgeCostCalculator(__FUNCTION__, logLevel),
    m_gridsManager(logLevel)
{
    m_general.crossCostMult = 2.5;
    m_general.boundaryCrossCostMult = 10;

    m_gridsManager.logger().setParent(&m_logger);
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
        logger().printWarning(__FUNCTION__, "No soil-cost mas has been set.");
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

//    std::unordered_set<std::string> revEdges;
//    for(DirectedGraph::edge_iter ed = edges(graph); ed.first != ed.second; ed.first++){
//        auto edgeStr = edge2string(*ed.first);
//        if(revEdges.find(edgeStr) != revEdges.end())//added already
//            continue;

//        DirectedGraph::edge_property& edge_prop = graph[*ed.first];

//        if ( m_noSoilCostTypes.find(edge_prop.edge_type) != m_noSoilCostTypes.end() )
//            continue;

//        std::vector<GridCellInfo> cellsInfo;
//        double width = std::max(0.0, edge_prop.defWidth);
//        double distance = arolib::geometry::calc_dist(edge_prop.p0, edge_prop.p1);
//        double area = distance * width;

//        double soilCost = 0;

//        if(area > 1e-9){
//            m_gridsManager.getCellsInfoUnderLine(SoilMapName, edge_prop.p0, edge_prop.p1, width, cellsInfo);

//            bool error;
//            soilCost = m_gridsManager.getGrid(SoilMapName)->getCellsComputedValue(cellsInfo,
//                                                                          error,
//                                                                          false,
//                                                                          ArolibGrid_t::AVERAGE_TOTAL,
//                                                                          area );
//            if(error)
//                soilCost = 0;
//        }

//        m_edgeSoilCosts[edgeStr] = soilCost;

//        if(edge_prop.bidirectional){
//            auto revEdgeStr = edge2string(edge_prop.revEdge);
//            m_edgeSoilCosts[revEdgeStr] = soilCost;
//            revEdges.insert(revEdgeStr);
//        }
//    }

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

    if ( m_noSoilCostTypes.find(edge_prop.edge_type) != m_noSoilCostTypes.end() )
        return 0.1 * distance + 0.1 * time;//no soil to be perserved, yet shorter paths are prefered

    double speed_inv = time;
    if(std::fabs(distance) > 1e-3)
        speed_inv /= distance;

    double weight_sum = 0.0;
    for (auto &o : overruns)
        weight_sum += o.weight;

    weight_sum *= m_costCoefficients.K_prevWeights;  // influence of previous overruns is reduced
    weight_sum += std::max(0.0, mass);

    //double soilCost = getSoilCost(edgeStr, edge_prop.p0, edge_prop.p1, edge_prop.defWidth);
    double soilCost = getSoilCost(edge_prop);

    double cost = distance * weight_sum * (m_costCoefficients.K_bias
                                           + std::pow(soilCost,  m_costCoefficients.Kpow_soil) * m_costCoefficients.K_soil
                                           + speed_inv * m_costCoefficients.K_time);
    // higher cost for crossing
    if (edge_prop.edge_type == DirectedGraph::EdgeType::CROSS)
        cost *= (1 + m_general.crossCostMult);
    else if(edge_prop.edge_type == DirectedGraph::EdgeType::BOUNDARY_CONN)
        cost *= (1 + m_general.boundaryCrossCostMult);

    //just checking for nan values
    if( std::isnan(cost) )
        m_logger.printOut( LogLevel::CRITIC, __FUNCTION__, 10,
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

