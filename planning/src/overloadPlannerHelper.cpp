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
 
#include "arolib/planning/overloadPlannerHelper.hpp"
#include "arolib/planning/graphhelper.hpp"
#include "arolib/planning/astar.hpp"
#include "arolib/planning/olvPlan.hpp"

#include <ctime>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/register/ring.hpp>

BOOST_GEOMETRY_REGISTER_POINT_2D(arolib::Point, double, cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_LINESTRING(std::vector<arolib::Point>)
BOOST_GEOMETRY_REGISTER_RING(std::deque<arolib::Point>)


namespace arolib{

OverloadPlannerHelper::OverloadPlannerHelper(const arolib::LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__){

}


void OverloadPlannerHelper::build_directGraph(const Subfield &sf,
                                              const std::vector<Route> &harvInfieldRoutes,
                                              const std::vector<HeadlandRoute> &harvHeadlandRoutes,
                                              const OutFieldInfo &outFieldInfo,
                                              const std::vector<Machine> &machines,
                                              const std::map<MachineId_t, MachineDynamicInfo> &machineInitialStates,
                                              double defYield,
                                              const ArolibGrid_t &yieldmap,
                                              const ArolibGrid_t &soilmap,
                                              double machineWidthIF,
                                              double machineWidthHL,
                                              bool bePreciseWithMaps)
{
    m_graph.clear();
    DirectedGraph::DirectedGraphBuilder graphBuilder(bePreciseWithMaps, m_logger.logLevel());
    graphBuilder.logger().setParent(&m_logger);
    graphBuilder.setOutputFile("/tmp/graph_data.txt");
    m_graph = graphBuilder.buildGraph(sf,
                                      harvInfieldRoutes,
                                      harvHeadlandRoutes,
                                      sf.resource_points,
                                      sf.access_points,
                                      outFieldInfo,
                                      machines,
                                      machineInitialStates,
                                      sf.headland_points,
                                      machineWidthIF,
                                      machineWidthHL,
                                      yieldmap,
                                      soilmap,
                                      false,
                                      defYield,
                                      sf.boundary_outer,
                                      0);

}


void OverloadPlannerHelper::build_directGraph(PlanningWorkspace &pw,
                                              size_t subfieldIdx,
                                              const std::vector<Route> &harvInfieldRoutes,
                                              const std::vector<HeadlandRoute> &harvHeadlandRoutes,
                                              const OutFieldInfo &outFieldInfo,
                                              double defYield,
                                              double machineWidthIF,
                                              double machineWidthHL,
                                              bool bePreciseWithMaps)
{
    if(subfieldIdx >= pw.m_field.subfields.size())
        throw std::invalid_argument( "OverloadPlannerHelper::build_directGraph: Invalid subfield index" );
    const Subfield &subfield = pw.m_field.subfields.at(subfieldIdx);

    m_graph.clear();
    DirectedGraph::DirectedGraphBuilder graphBuilder(bePreciseWithMaps, m_logger.logLevel());
    graphBuilder.logger().setParent(&m_logger);
    graphBuilder.setOutputFile("/tmp/graph_data.txt");
    m_graph = graphBuilder.buildGraph(pw,
                                      subfieldIdx,
                                      harvInfieldRoutes,
                                      harvHeadlandRoutes,
                                      outFieldInfo,
                                      machineWidthIF,
                                      machineWidthHL,
                                      false,
                                      defYield,
                                      subfield.boundary_outer,
                                      0);

}


void OverloadPlannerHelper::insert_olv_route_into_graph(const Route &olv_route, const Machine &olv) {
    if (olv_route.route_points.size() < 2) {
        return;
    }

    RoutePoint first_rp = olv_route.route_points.at(0);
    DirectedGraph::vertex_t prev_vt = find_nearest_vertex(first_rp.point, m_graph);

    // get vertices that are relevant to calculate distance to next routepoint
    std::vector<DirectedGraph::vertex_t> relevant_vts = calcConnectedVertices(m_graph, prev_vt);
    relevant_vts.push_back(prev_vt);

    for (int i = 1; i < olv_route.route_points.size(); ++i) {

        RoutePoint current_rp = olv_route.route_points.at(i);
        DirectedGraph::vertex_t current_vt = calcNearestVertexToPoint(m_graph, current_rp.point, relevant_vts);
        // TODO Do we have to deal with outliers?

        // reached a new vt?
        if (current_vt != prev_vt) {
            // add overroll to edges
            DirectedGraph::overroll_property overroll;
            overroll.machine_id = olv.id;
            overroll.time = current_rp.time_stamp;
            overroll.weight = current_rp.bunker_mass + olv.weight;
            addOverrollToEdges(m_graph, overroll, prev_vt, current_vt);

            // update relevant vertices and prev_vt
            relevant_vts.clear();
            for (DirectedGraph::vertex_t vt : calcConnectedVertices(m_graph, current_vt)) {
                if (vt != prev_vt) { // do not add the previous vertex as we won't drive backwards
                    relevant_vts.push_back(vt);
                }
            }
            relevant_vts.push_back(current_vt);
            prev_vt = current_vt;
        }
    }
}

void OverloadPlannerHelper::update_graph_rps_timestamp(const Route &harv_route, bool set_to_zero) {
    for (RoutePoint rp : harv_route.route_points) {
        auto vt_it = m_graph.routepoint_vertex_map().find(rp);
        if(vt_it == m_graph.routepoint_vertex_map().end())
            continue;
        DirectedGraph::vertex_t vt = vt_it->second;
        if (set_to_zero) {
            m_graph[vt].route_point.time_stamp = 0.0;
        } else {
            m_graph[vt].route_point.time_stamp = rp.time_stamp;
        }
    }
}

DirectedGraph::Graph OverloadPlannerHelper::getGraph() const {
    return m_graph;
}

std::vector<Route> OverloadPlannerHelper::plan(const std::vector<Route> &harv_routes,
                                         const std::vector<Machine> &machines,
                                         const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                         const OutFieldInfo &outFieldInfo,
                                         const OverloadPlannerHelper::PlannerSettings &settings)
{
    std::vector<std::vector<Machine>> overloadMachines(harv_routes.size());
    int pos = 0;

    std::vector<Route> ret;

    if(harv_routes.empty()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No harvester routes were given");
        return ret;
    }

    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Overload machines:");
    for (int i = 0; i < machines.size(); ++i) {
        Machine m = machines.at(i);
        if (m.machinetype == Machine::OLV) {
            overloadMachines.at(pos).push_back(m);
            if (m.bunker_mass < 1)
                m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Overload machine has bunker mass capacity " + std::to_string(m.bunker_mass) );
            pos = (pos + 1) % harv_routes.size();

            m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "\t" + m.toString());
        }
    }
    m_harvester_routes.clear();
    bool planning_error = false;
    double plan_cost = 0.0;
    // plans for the different harvester routes are independet of each other
    for (int i = 0; i < harv_routes.size(); ++i) {

        bool found_plan = false;
        double min_cost = std::numeric_limits<double>::max();
        std::vector<Route> min_routes;
        Route min_harvester_route;
        DirectedGraph::Graph min_graph;

        double duration = 0.0;
        if (settings.olvOrderStrategy == CHECK_ALL_PERMUTATIONS || settings.olvOrderStrategy == CHECK_ALL_PERMUTATIONS__START_WITH_ORIGINAL) {
            clock_t time_start = clock();
            std::vector<int> perm;
            int numFixed = 0;


            //start with the likely one to be the right one
            if (settings.olvOrderStrategy == CHECK_ALL_PERMUTATIONS){
                overloadMachines.at(i) = SimpleMultiOLVPlanner::reorderWorkingGroup(m_graph,
                                                                              overloadMachines.at(i),
                                                                              harv_routes.at(i),
                                                                              machineCurrentStates,
                                                                              settings.olvSwitchingStrategy,
                                                                              0,
                                                                              settings.harvestedMassLimit,
                                                                              &m_logger);
            }

            if(settings.numFixedInitalOlvsInOrder < 0){
                numFixed = estimateNumFixedInitalOlvsInOrder(harv_routes.at(i),
                                                             overloadMachines.at(i),
                                                             machineCurrentStates);
            }
            else{
                numFixed = std::max( 0, std::min(settings.numFixedInitalOlvsInOrder, (int)overloadMachines.at(i).size() ) );
            }

            for (int j = numFixed; j < overloadMachines.at(i).size(); ++j)
                perm.emplace_back(j);

            do {

                m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Planning for OLV permutation ", -1, false);
                std::vector<Machine> olvPermutation;
                for (int j = 0; j < numFixed; ++j){
                    olvPermutation.emplace_back(overloadMachines.at(i).at(j));
                    m_logger.printOut(LogLevel::DEBUG, std::to_string(j)
                                                       + "(" + std::to_string(olvPermutation.back().id) + ") ", -1, false);
                }

                for (int j = 0; j < perm.size(); ++j) {
                    olvPermutation.emplace_back(overloadMachines.at(i).at(perm.at(j)));
                    m_logger.printOut(LogLevel::DEBUG, std::to_string(perm.at(j))
                                                       + "(" + std::to_string(olvPermutation.back().id) + ") ", -1, false);
                }
                m_logger.printOut(LogLevel::DEBUG, "");
                SimpleMultiOLVPlanner multiOLVPlanner(m_graph,
                                                harv_routes.at(i),
                                                olvPermutation,
                                                machineCurrentStates,
                                                settings,
                                                m_logger.logLevel());
                multiOLVPlanner.logger().setParent(&m_logger);

                std::string sError = multiOLVPlanner.plan(min_cost, false);
                if(sError.empty()){
                    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Found plan with cost: " + std::to_string( multiOLVPlanner.getCost() ) );

                    if (multiOLVPlanner.getCost() < min_cost) {
                        m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "New best cost: " + std::to_string(min_cost) + " --> " + std::to_string( multiOLVPlanner.getCost() ) );
                        found_plan = true;
                        min_cost = multiOLVPlanner.getCost();
                        min_routes = multiOLVPlanner.getRoutes();
                        min_harvester_route = multiOLVPlanner.getHarvesterRoute();
                        min_graph = multiOLVPlanner.getGraph();
                    }
                }
                else
                    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Could not find a plan: " + sError );

                duration = (clock() - time_start) / (double) CLOCKS_PER_SEC;
                m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "##### elapsed time since start of planning: " + std::to_string(duration) + " seconds\n" );

            } while (std::next_permutation(begin(perm), end(perm)) && duration < settings.max_planning_time);

        }
        else {
            if(settings.olvOrderStrategy == ESTIMATE_OPTIMAL_ORDER){
                int numFixed = 0;
                if(settings.numFixedInitalOlvsInOrder < 0){
                    numFixed = estimateNumFixedInitalOlvsInOrder(harv_routes.at(i),
                                                                 overloadMachines.at(i),
                                                                 machineCurrentStates);
                }
                else{
                    numFixed = std::max( 0, std::min(settings.numFixedInitalOlvsInOrder, (int)overloadMachines.at(i).size() ) );
                }
                overloadMachines.at(i) = SimpleMultiOLVPlanner::reorderWorkingGroup(m_graph,
                                                                              overloadMachines.at(i),
                                                                              harv_routes.at(i),
                                                                              machineCurrentStates,
                                                                              settings.olvSwitchingStrategy,
                                                                              numFixed,
                                                                              settings.harvestedMassLimit,
                                                                              &m_logger);
            }
            SimpleMultiOLVPlanner multiOLVPlanner(m_graph,
                                            harv_routes.at(i),
                                            overloadMachines.at(i),
                                            machineCurrentStates,
                                            settings,
                                            m_logger.logLevel());
            multiOLVPlanner.logger().setParent(&m_logger);

            std::string sError = multiOLVPlanner.plan(min_cost, false);
            if(sError.empty()){
                min_cost = multiOLVPlanner.getCost();
                min_routes = multiOLVPlanner.getRoutes();
                min_graph = multiOLVPlanner.getGraph();
                min_harvester_route = multiOLVPlanner.getHarvesterRoute();
                found_plan = true;
            }
            else
                m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Could not find a plan: " + sError );
        }

        if (found_plan) {
            ret.insert(ret.end(), min_routes.begin(), min_routes.end());
            m_harvester_routes.push_back(min_harvester_route);
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Best plan for route of harvester with machine_id " + std::to_string( harv_routes.at(i).machine_id ) + " has cost " + std::to_string( min_cost ) );
            m_graph = min_graph;
            plan_cost += min_cost;
        } else {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Could not find a plan for route of harvester with machine_id " + std::to_string( harv_routes.at(i).machine_id ) );
            planning_error = true;
            break;
        }

    }

    if (planning_error) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Could not find a plan." );
        ret.clear();
    }
    else
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Found plan with overall cost: " + std::to_string( plan_cost ) );

    return ret;
}

OverloadPlannerHelper::OlvOrderStrategy OverloadPlannerHelper::intToOlvOrderStrategy(int value)
{
    if(value == OlvOrderStrategy::KEEP_ORIGINAL_ORDER)
        return OlvOrderStrategy::KEEP_ORIGINAL_ORDER;
    else if(value == OlvOrderStrategy::ESTIMATE_OPTIMAL_ORDER)
        return OlvOrderStrategy::ESTIMATE_OPTIMAL_ORDER;
    else if(value == OlvOrderStrategy::CHECK_ALL_PERMUTATIONS)
        return OlvOrderStrategy::CHECK_ALL_PERMUTATIONS;
    else if(value == OlvOrderStrategy::CHECK_ALL_PERMUTATIONS__START_WITH_ORIGINAL)
        return OlvOrderStrategy::CHECK_ALL_PERMUTATIONS__START_WITH_ORIGINAL;

    throw std::invalid_argument( "The given value does not correspond to any OverloadPlannerHelper::OlvOrderStrategy" );
}

typedef std::pair<DirectedGraph::vertex_t, DirectedGraph::vertex_property> vertexPair;

bool compare_harvested_mass(const vertexPair& first,
           const vertexPair& second) {
    return first.second.harvested_mass < second.second.harvested_mass;
}

typedef std::pair<DirectedGraph::vertex_t, double> vertexTimePair; // current position and time of a machine

//double OverloadPlannerHelper::extractHarvesterSwitchingTime(DirectedGraph::vertex_t goal_vt) {
//    DirectedGraph::in_edge_iterator in_begin, in_end;
//    double goal_mass = graph[goal_vt].harvested_mass;
//    int goal_harvester_id = graph[goal_vt].harvester_id;
//    double harvest_time = -1;
//    for (boost::tie(in_begin, in_end) = in_edges(goal_vt,graph); in_begin != in_end; ++in_begin)
//    {
//        DirectedGraph::vertex_t prev = source(*in_begin, graph);
//        DirectedGraph::vertex_property prev_prop = graph[prev];
//        if (goal_harvester_id == prev_prop.harvester_id && prev_prop.harvested_mass < goal_mass) {
//            harvest_time = graph[*in_begin].harvest_time;
//        }
//    }
//    return harvest_time;
//}

void OverloadPlannerHelper::updateGraph(double current_time) {

    // remove overruns that are in the future
    DirectedGraph::edge_iter ei;
    for (ei = edges(m_graph); ei.first != ei.second; ++ei.first) {
        DirectedGraph::edge_property e = m_graph[*ei.first];
        for (std::vector<DirectedGraph::overroll_property>::iterator oit = e.overruns.begin(); oit != e.overruns.end(); ) {
            if (oit->time > current_time) {
                oit = e.overruns.erase(oit);
            } else {
                ++oit;
            }
        }
    }

}

size_t OverloadPlannerHelper::estimateNumFixedInitalOlvsInOrder(const Route &harv_route,
                                                          const std::vector<Machine> &olv_machines,
                                                          const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates)
{
    if(olv_machines.size() < 2)
        return 0;

    size_t numFixed = 0;

    Point initHarvPos;
    for(auto &rp : harv_route.route_points){
        if( rp.type != RoutePoint::HEADLAND && rp.time_stamp >= -0.00001 ){
            initHarvPos = rp.point;
            break;
        }
    }

    for(auto &m : olv_machines){
        if(m.machinetype != Machine::OLV)
            continue;
        auto it0 = machineCurrentStates.find(m.id);
        if (it0 == machineCurrentStates.end())
            return numFixed;
        if(it0->second.bunkerMass >= m.bunker_mass * OLVPlan::OlvMaxCapacityMultiplier)
            return numFixed;
        if( arolib::geometry::calc_dist( it0->second.position, initHarvPos ) > 20 )
            return numFixed;
        ++numFixed;
    }
    return numFixed;
}

}
