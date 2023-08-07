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
 

#include "arolib/planning/edge_calculators/edgeCostCalculator.hpp"

namespace arolib{


bool IEdgeCostCalculator::GeneralParameters::parseFromStringMap(const std::map<std::string, std::string> &strMap, bool strict)
{
    IEdgeCostCalculator::GeneralParameters tmp = *this;

    try{
        std::map<std::string, double*> dMap = { {"crossCostMult" , &tmp.crossCostMult},
                                                {"headlandCrossCostMult" , &tmp.headlandCrossCostMult},
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
    ret["headlandCrossCostMult"] = double2string( headlandCrossCostMult );
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
    m_general.headlandCrossCostMult = 1;
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
    else if(edge_prop.edge_type == DirectedGraph::EdgeType::CROSS_HL)
        penalty = (time - waitingTime) * m_general.headlandCrossCostMult;
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
        logger().printOut( LogLevel::CRITIC, __FUNCTION__, 10,
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
    m_general.headlandCrossCostMult = 1;
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
    else if(edge_prop.edge_type == DirectedGraph::EdgeType::CROSS_HL)
        penalty = distance * m_general.headlandCrossCostMult;
    else if(edge_prop.edge_type == DirectedGraph::EdgeType::BOUNDARY_CONN)
        penalty = distance * m_general.boundaryCrossCostMult;

    double cost = distance + penalty + multTime*time;//a time factor is added so that if two plans have similar distance-costs, the shortest one will give lower costs

    //just checking for nan values
    if( std::isnan(cost) )
        logger().printOut( LogLevel::CRITIC, __FUNCTION__, 10,
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

}

