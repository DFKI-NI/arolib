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
 
#include "arolib/planning/multiolvplanner.h"

namespace arolib{

namespace{
int factorial(int n)
{
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}
}


MultiOLVPlanner::OlvOrderStrategy MultiOLVPlanner::intToOlvOrderStrategy(int value)
{
    if(value == OlvOrderStrategy::FOLLOW_OLVS_ORDER)
        return OlvOrderStrategy::FOLLOW_OLVS_ORDER;
    else if(value == OlvOrderStrategy::ESTIMATE_OPTIMAL_ORDER)
        return OlvOrderStrategy::ESTIMATE_OPTIMAL_ORDER;
    else if(value == OlvOrderStrategy::CHECK_ALL_PERMUTATIONS)
        return OlvOrderStrategy::CHECK_ALL_PERMUTATIONS;
    else if(value == OlvOrderStrategy::CHECK_ALL_PERMUTATIONS__START_FOLLOWING_OLVS_ORDER)
        return OlvOrderStrategy::CHECK_ALL_PERMUTATIONS__START_FOLLOWING_OLVS_ORDER;

    throw std::invalid_argument( "The given value does not correspond to any MultiOLVPlanner::OlvOrderStrategy" );

}

MultiOLVPlanner::OlvAssignmentStrategy MultiOLVPlanner::intToOlvAssignmentStrategy(int value)
{
    if(value == OlvAssignmentStrategy::EXCLUSIVE)
        return OlvAssignmentStrategy::EXCLUSIVE;
    else if(value == OlvAssignmentStrategy::SHARED_OLVS)
        return OlvAssignmentStrategy::SHARED_OLVS;

    throw std::invalid_argument( "The given value does not correspond to any MultiOLVPlanner::OlvAssignmentStrategy" );
}

MultiOLVPlanner::ThreadsOption MultiOLVPlanner::intToThreadsOption(int value)
{
    if(value == ThreadsOption::SINGLE_THREAD)
        return ThreadsOption::SINGLE_THREAD;
    else if(value == ThreadsOption::MULTIPLE_THREADS)
        return ThreadsOption::MULTIPLE_THREADS;

    throw std::invalid_argument( "The given value does not correspond to any MultiOLVPlanner::ThreadsOption" );

}

void MultiOLVPlanner::PlanData::init(const DirectedGraph::Graph &_graph, const std::vector<Route> &harv_routes)
{
    graph = _graph;

    planOverallCost = 0;
    subplanCost_exclusive = std::numeric_limits<double>::max();
    planOK = false;

    harvesterRoutes = harv_routes;

    overallDelays = std::vector<double>(harv_routes.size(),0);
    planCosts = std::vector<double>(harv_routes.size(), std::nanf("1"));//, std::numeric_limits<double>::max());
    lastHarvIndexes = std::vector<int>(harv_routes.size(), -1);
    plannedRoutes.clear();

    int minId = std::numeric_limits<int>::max();
    int maxId = std::numeric_limits<int>::lowest();
    if(harv_routes.empty())
        minId = maxId = 0;
    for(auto& route : harv_routes){
        minId = std::min(route.route_id, minId);
        maxId = std::max(route.route_id, maxId);
        //lastHarvIndexes[route.route_id] = -1;//used when this was a map
    }
    routeIdDelta = 10;
    while ( maxId - minId > routeIdDelta )
        routeIdDelta *= 10;
    routeIdDelta *= 10;
    routeIdDelta += minId;

}

void MultiOLVPlanner::PlanData::updateOverallCost()
{
    planOverallCost = 0;
    for(auto &c : planCosts){
        if(!std::isnan(c))
            planOverallCost += c;
    }
}

bool MultiOLVPlanner::PlannerSettings::parseFromStringMap(MultiOLVPlanner::PlannerSettings &params, const std::map<std::string, std::string> &map, bool strict)
{
    MultiOLVPlanner::PlannerSettings tmp;

    if( !OverloadActivitiesPlanner::PlannerSettings::parseFromStringMap(tmp, map, strict) )
        return false;
    if( !OLVPlan::PlannerSettings::parseFromStringMap(tmp, map, strict) )
        return false;

    int replanning_strategy, olvOrderStrategy, olvAssignmentStrategy, threadsOption;
    std::map<std::string, double*> dMap = { {"max_waiting_time" , &tmp.max_waiting_time},
                                            {"harvestedMassLimit" , &tmp.harvestedMassLimit},
                                            {"max_planning_time" , &tmp.max_planning_time} };
    std::map<std::string, int*> iMap = { {"numOverloadActivities" , &tmp.numOverloadActivities},
                                            {"numFixedInitalOlvsInOrder" , &tmp.numFixedInitalOlvsInOrder} };
    std::map<std::string, bool*> bMap = { {"sendLastOlvToResourcePoint" , &tmp.sendLastOlvToResourcePoint} ,
                                          {"leaveRoutePointBetweenOLActivities" , &tmp.leaveRoutePointBetweenOLActivities}};
    std::map<std::string, int*> enumMap = { {"replanning_strategy" , &replanning_strategy},
                                            {"olvOrderStrategy" , &olvOrderStrategy},
                                            {"olvAssignmentStrategy" , &olvAssignmentStrategy},
                                            {"threadsOption" , &threadsOption} };

    if( !setValuesFromStringMap( map, dMap, strict)
            || !setValuesFromStringMap( map, iMap, strict)
            || !setValuesFromStringMap( map, bMap, strict)
            || !setValuesFromStringMap( map, enumMap, strict) )
        return false;

    tmp.olvOrderStrategy = intToOlvOrderStrategy( olvOrderStrategy );
    tmp.olvAssignmentStrategy = intToOlvAssignmentStrategy( olvAssignmentStrategy );
    tmp.threadsOption = intToThreadsOption( threadsOption );

    params = tmp;
    return true;
}

std::map<std::string, std::string> MultiOLVPlanner::PlannerSettings::parseToStringMap(const MultiOLVPlanner::PlannerSettings &params)
{
    std::map<std::string, std::string> ret, subMap;
    subMap = OverloadActivitiesPlanner::PlannerSettings::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );
    subMap = OLVPlan::PlannerSettings::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );

    ret["max_waiting_time"] = double2string( params.max_waiting_time );
    ret["harvestedMassLimit"] = double2string( params.harvestedMassLimit );
    ret["max_planning_time"] = double2string( params.max_planning_time );
    ret["numOverloadActivities"] = std::to_string( params.numOverloadActivities );
    ret["numFixedInitalOlvsInOrder"] = std::to_string( params.numFixedInitalOlvsInOrder );
    ret["sendLastOlvToResourcePoint"] = std::to_string( params.sendLastOlvToResourcePoint );
    ret["olvOrderStrategy"] = std::to_string( params.olvOrderStrategy );
    ret["olvAssignmentStrategy"] = std::to_string( params.olvAssignmentStrategy );
    ret["threadsOption"] = std::to_string( params.threadsOption );
    ret["leaveRoutePointBetweenOLActivities"] = std::to_string( params.leaveRoutePointBetweenOLActivities );

    return ret;

}

MultiOLVPlanner::MultiOLVPlanner(const DirectedGraph::Graph &graph,
                                 const std::vector<Route> &base_routes,
                                 const std::vector<Machine> &machines,
                                 const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                 const MultiOLVPlanner::PlannerSettings &settings,
                                 std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                                 const std::string &outputFolder,
                                 LogLevel logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_graph(graph),
    m_machineInitialStates(machineCurrentStates),
    m_settings(settings),
    m_edgeCostCalculator(edgeCostCalculator),
    m_outputFolder(outputFolder)
{
    if(!m_outputFolder.empty() && m_outputFolder.back() != '/')
        m_outputFolder += "/";

    m_outputFolderEd = m_outputFolder;

    for(auto& m : machines){
        if(m.machinetype == Machine::HARVESTER)
            m_harvesters[m.id] = m;
        else if(m.machinetype == Machine::OLV)
            m_olvs.emplace_back(m);
    }


    m_harvesterRoutes.reserve(base_routes.size());
    for(const auto& route : base_routes){
        double machineTimestamp = 0;
        auto mdi_it = m_machineInitialStates.find(route.machine_id);
        if(mdi_it != m_machineInitialStates.end())
            machineTimestamp = std::max(0.0, mdi_it->second.timestamp);

        size_t indStart = route.route_points.size();
        for(size_t i = 0 ; i < route.route_points.size() ; ++i){
            if(route.route_points.at(i).time_stamp > -1e-9){
                indStart = i;
                break;
            }
        }

        if(indStart > 0)//set the timestamps of the vertices corresponding to the disregarded segment to "worked"
            resetTimestampsFromBaseRoute(m_graph, route, 0, indStart-1);

        m_harvesterRoutes.emplace_back(Route());
        route.copyToWithoutPoints(m_harvesterRoutes.back(), true);
        if(indStart < route.route_points.size()){

            m_harvesterRoutes.back().route_points.insert(m_harvesterRoutes.back().route_points.end(),
                                                         route.route_points.begin() + indStart,
                                                         route.route_points.end());
            if(machineTimestamp > 1e-9 && m_harvesterRoutes.back().route_points.front().time_stamp < machineTimestamp){
                double delta_time = machineTimestamp - m_harvesterRoutes.back().route_points.front().time_stamp;
                for(auto& rp : m_harvesterRoutes.back().route_points)
                    rp.time_stamp += delta_time;
            }

            //update the timestamps of the vertices corresponding to the remaining route points
            updateTimestampsFromBaseRoute(m_graph, m_harvesterRoutes.back(), 0, 0, -1, 0);
        }
    }
}

void MultiOLVPlanner::reset()
{
    m_bestPlan.init(m_graph, m_harvesterRoutes);
    m_currentPlan.init(m_graph, m_harvesterRoutes);
}

std::string MultiOLVPlanner::planAll()
{
    if(!m_edgeCostCalculator){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return "Invalid edgeCostCalculator";
    }

    if(m_harvesterRoutes.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "No harvester routes were given");
        return "No harvester routes were given";
    }
    for(auto &r : m_harvesterRoutes){
        if(r.route_points.size() < 2){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "One or more harvester routes are invalid");
            return "One or more harvester routes are invalid";
        }
    }

    if(m_olvs.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "No overload machines were given");
        return "No overload machines were given";
    }

    if(!MachineDynamicInfo::isValid(m_machineInitialStates)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid machine initial states");
        return "Invalid machine initial states";
    }

    reset();

    std::string sError;

    if(m_settings.olvAssignmentStrategy == OlvAssignmentStrategy::EXCLUSIVE )
        sError = planAll_exclusive();
    else
        sError = planAll_shared();

    if(!sError.empty())
        return sError;

    updateHarvesterRouteRelations();
    m_bestPlan.updateOverallCost();

    return "";//ok

}

std::vector<Route> MultiOLVPlanner::getPlannedRoutes()
{
    std::vector<Route> ret;
    int idCount = 0;
    for(auto &it : m_bestPlan.plannedRoutes){
        it.second.route_id = m_bestPlan.routeIdDelta + idCount;//set a unique route id
        it.second.machine_id = it.first;
        ret.emplace_back(it.second);
    }
    return ret;

}

void MultiOLVPlanner::addHarvesterSoilValues(const std::map<MachineId_t, Machine> &harvesters)
{
    for(auto &route : m_harvesterRoutes){

        auto it_m = harvesters.find(route.machine_id);
        if(it_m == harvesters.end())
            continue;

        for(size_t i = 0 ; i+1 < route.route_points.size() ; ++i){
            auto & rp0 = route.route_points.at(i);
            auto & rp1 = route.route_points.at(i+1);
            auto it_0 = m_graph.routepoint_vertex_map().find( rp0 );
            if(it_0 == m_graph.routepoint_vertex_map().end())
                continue;
            auto it_1 = m_graph.routepoint_vertex_map().find( rp1 );
            if(it_1 == m_graph.routepoint_vertex_map().end())
                continue;

            DirectedGraph::overroll_property overrun;
            overrun.duration = rp1.time_stamp - rp0.time_stamp;
            overrun.machine_id = it_m->second.id;
            overrun.weight = it_m->second.weight;

            m_graph.addOverrun( it_0->second, it_1->second, overrun );
        }

    }
}

std::string MultiOLVPlanner::planAll_exclusive()
{
    int numThreads = std::thread::hardware_concurrency();
    if(numThreads < 2 || m_settings.threadsOption == ThreadsOption::SINGLE_THREAD)
        return planAll_exclusive__singleThread();

    std::vector<std::vector<Machine>> overloadMachines(m_harvesterRoutes.size());

    /* Assign olvs to the harvester routes, one by one to each harvester route in the order given by the user
     * For example, if there are 2 harvester routes and the working group given by the user is m_olvs = <OLV1, OLV2, OLV3, OLV4, OLV5>, the assignment will be:
     *       First harvester route: <OLV1, OLV3, OLV5>
     *       Second harvester route: <OLV2, OLV4>
     */
    int pos = 0;
    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Overload machines:");
    for (int i = 0; i < m_olvs.size(); ++i) {
        Machine m = m_olvs.at(i);
        overloadMachines.at(pos).push_back(m);
        if (m.bunker_mass < 1)
            logger().printOut(LogLevel::WARNING, __FUNCTION__, "Overload machine has bunker mass capacity " + std::to_string(m.bunker_mass) );
        pos = (pos + 1) % overloadMachines.size();

        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "\t" + m.toString());
    }

    m_bestPlan.plannedRoutes.clear();
    m_currentPlan.plannedRoutes.clear();

    if (m_settings.olvOrderStrategy == FOLLOW_OLVS_ORDER || m_settings.olvOrderStrategy == ESTIMATE_OPTIMAL_ORDER)//only ONE permutation will be checked --> compute a single plan
        return planAll_exclusive_singlePlan(overloadMachines);

    // Compute the plans (olv route and adjusted harvester route) for each harvester route
    // @TODO: at the moment, plans for the different harvester routes are independent of each other. however, if a harvester route has a delay, the previously planned harvester routes have to be checked (new temporal constraints (timestamps of when a vvertex was harvested / is available) may apply)
    for (int i = 0; i < m_harvesterRoutes.size(); ++i) {

        std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

        std::vector<int> perm;
        int numFixed = 0;


        if (m_settings.olvOrderStrategy == CHECK_ALL_PERMUTATIONS){
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Reordering working group... " );
            //start with the permutation that is likely to be the best one (other plans will be discarded faster)
            overloadMachines.at(i) = reorderWorkingGroup(overloadMachines.at(i),
                                                         m_harvesterRoutes.at(i),
                                                         m_machineInitialStates,
                                                         m_settings,
                                                         0,
                                                         m_settings.numOverloadActivities,
                                                         m_settings.harvestedMassLimit,
                                                         m_settings.leaveRoutePointBetweenOLActivities,
                                                         loggerPtr());
        }

        if(m_settings.numFixedInitalOlvsInOrder < 0){//estimate the optimal number of fixed OLVs to avoid unnecessary computations of computations likely to NOT be the best one
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Estimating optimal OLV order... " );
            numFixed = estimateNumFixedInitalOlvsInOrder(m_harvesterRoutes.at(i),
                                                         overloadMachines.at(i),
                                                         m_machineInitialStates);
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, std::to_string(numFixed) + " initial fixed olvs estimated automatically");
        }
        else
            numFixed = std::max( 0, std::min(m_settings.numFixedInitalOlvsInOrder, (int)overloadMachines.at(i).size() ) );

        for (int j = numFixed; j < overloadMachines.at(i).size(); ++j)//build the permutation's (initial) order (without including the initial fixed olvs)
            perm.emplace_back(j);

        bool found_plan = false;
        size_t countPermutations = 0;
        std::string sBestPermutation;

        const int numPerms = factorial(perm.size());
        numThreads = std::max( 1, std::min(numThreads, numPerms) );

        //assign permutations to the different threads
        std::vector< std::vector< std::vector<int> > > parallelPerm(numThreads);//vector of permutations per thread
        size_t indParallelPerm = 0;
        do {
            parallelPerm.at(indParallelPerm).emplace_back(perm);

            ++indParallelPerm;
            if(indParallelPerm >= parallelPerm.size())
                indParallelPerm = 0;

        } while ( std::next_permutation(begin(perm), end(perm)) ); //update the next permutation (if there is)


        std::string sPermutation_base = "";
        std::string permutationSubfolder_base = "perm_";
        std::vector<Machine> overloadMachines_fixed;
        for (int j = 0; j < numFixed; ++j){//first add the fixed OLVs
            overloadMachines_fixed.emplace_back(overloadMachines.at(i).at(j));
            sPermutation_base += ( std::to_string(j) + "(" + std::to_string(overloadMachines_fixed.back().id) + ") ");
            permutationSubfolder_base += ( std::to_string(overloadMachines_fixed.back().id) + "_" );
        }

        std::vector< std::future< void > > futures_perm (numThreads);
        std::mutex mutex;

        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Started planning for " + std::to_string(numPerms) + "  permutations in " + std::to_string(numThreads) + " threads... " );
        for(size_t kk = 0 ; kk < futures_perm.size() ; ++kk){
            auto& fu = futures_perm.at(kk);
            auto& perms = parallelPerm.at(kk);

            fu = std::async( std::launch::async,
                             [this,
                              i,
                              &found_plan,
                              &sBestPermutation,
                              &overloadMachines,
                              &overloadMachines_fixed,
                              &perms,
                              &sPermutation_base,
                              &permutationSubfolder_base,
                              &mutex,
                              &time_start,
                              &countPermutations,
                              &numThreads](){

                double duration = 0;
                const auto& bestPlanCost = m_bestPlan.planCosts.at(i);//const reference to the costs

                for(auto& perm : perms){

                    std::string outputFolder = m_outputFolder;

                    //build the OLVs vector used for this permutation (i.e. fixed initial OLVs + current permutation of not-fixed OLVs)
                    std::string sPermutation = sPermutation_base, permutationSubfolder = permutationSubfolder_base;
                    auto olvPermutation = overloadMachines_fixed;
                    for (int j = 0; j < perm.size(); ++j) {//add the remaining OLVs corresponding to the current permutation
                        olvPermutation.emplace_back(overloadMachines.at(i).at(perm.at(j)));
                        sPermutation += ( std::to_string(perm.at(j)) + "(" + std::to_string(olvPermutation.back().id) + ") " );
                        permutationSubfolder += ( std::to_string(olvPermutation.back().id) + "-" );
                    }
                    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Planning for OLV permutation " + sPermutation);

                    if(!outputFolder.empty() && !permutationSubfolder.empty()){//update the folder where the planning (search) information the current permutation will be stored
                        permutationSubfolder.back() = '/';
                        outputFolder = outputFolder + permutationSubfolder;
                    }

                    auto oldBestCost = bestPlanCost;

                    std::string sError = planSingle_exclusive(i, olvPermutation, m_settings.max_planning_time-duration, outputFolder, numThreads>1);
                    {
                        std::lock_guard<std::mutex> guard(mutex);
                        if(sError.empty()){//a plan was computed and is better that the current BEST plan
                            found_plan = true;
                            logger().printOut(LogLevel::DEBUG, __FUNCTION__, sPermutation + " : Found new best plan with cost: " + std::to_string( bestPlanCost ) );
                            logger().printOut(LogLevel::DEBUG, __FUNCTION__, sPermutation + " : New best cost: " + std::to_string(oldBestCost) + " --> " + std::to_string( bestPlanCost ) );
                            sBestPermutation = sPermutation;
                        }
                        else //either no plan was computed (e.g. because of an error, or the max planning time was reached), or the computed plan has a higher cost that the curren best plan
                            logger().printOut(LogLevel::DEBUG, __FUNCTION__, sPermutation + " : Could not find a (-/better) plan: " + sError );

                        ++countPermutations;
                    }

                    duration = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
                    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "##### elapsed time since start of planning: " + std::to_string(duration) + " seconds\n" );
                    if(m_settings.max_planning_time > 1e-5 && duration > m_settings.max_planning_time)
                        break;
                }
            });
        }

        for(auto& fu : futures_perm)
            fu.wait();

        if (!found_plan){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Could not find a plan for route of harvester with machine_id " + std::to_string( m_harvesterRoutes.at(i).machine_id ) );
            return "Could not find a plan for route of harvester with machine_id " + std::to_string( m_harvesterRoutes.at(i).machine_id );
        }

        logger().printOut(LogLevel::INFO, __FUNCTION__, "Best plan for route of harvester with machine_id " + std::to_string( m_harvesterRoutes.at(i).machine_id )
                          + " found with cost " + std::to_string( m_bestPlan.planCosts.at(i) )
                          + " (overall plan cost = " + std::to_string( m_bestPlan.planOverallCost )
                          + ") after checking " + std::to_string(countPermutations)
                          + "/" + std::to_string(numPerms)
                          + " permutations ( best permutation : " + sBestPermutation + ")");

        m_graph = m_bestPlan.graph;

    }

    return "";

}

std::string MultiOLVPlanner::planAll_exclusive__singleThread()
{
    std::vector<std::vector<Machine>> overloadMachines(m_harvesterRoutes.size());

    /* Assign olvs to the harvester routes, one by one to each harvester route in the order given by the user
         * For example, if there are 2 harvester routes and the working group given by the user is m_olvs = <OLV1, OLV2, OLV3, OLV4, OLV5>, the assignment will be:
         *       First harvester route: <OLV1, OLV3, OLV5>
         *       Second harvester route: <OLV2, OLV4>
         */
    int pos = 0;
    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Overload machines:");
    for (int i = 0; i < m_olvs.size(); ++i) {
        Machine m = m_olvs.at(i);
        overloadMachines.at(pos).push_back(m);
        if (m.bunker_mass < 1)
            logger().printOut(LogLevel::WARNING, __FUNCTION__, "Overload machine has bunker mass capacity " + std::to_string(m.bunker_mass) );
        pos = (pos + 1) % overloadMachines.size();

        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "\t" + m.toString());
    }

    m_bestPlan.plannedRoutes.clear();

    if (m_settings.olvOrderStrategy == FOLLOW_OLVS_ORDER || m_settings.olvOrderStrategy == ESTIMATE_OPTIMAL_ORDER)//only ONE permutation will be checked --> compute a single plan
        return planAll_exclusive_singlePlan(overloadMachines);

    // Compute the plans (olv route and adjusted harvester route) for each harvester route
    // @TODO: at the moment, plans for the different harvester routes are independent of each other. however, if a harvester route has a delay, the previously planned harvester routes have to be checked (new temporal constraints (timestamps of when a vvertex was harvested / is available) may apply)
    for (int i = 0; i < m_harvesterRoutes.size(); ++i) {

        auto& bestPlanCost = m_bestPlan.planCosts.at(i);//reference to the costs (changes will be applied directly to the costs of the best plan)

        double duration = 0.0;

        std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

        std::vector<int> perm;
        int numFixed = 0;


        if (m_settings.olvOrderStrategy == CHECK_ALL_PERMUTATIONS){
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Reordering working group... " );
            //start with the permutation that is likely to be the best one (other plans will be discarded faster)
            overloadMachines.at(i) = reorderWorkingGroup(overloadMachines.at(i),
                                                         m_harvesterRoutes.at(i),
                                                         m_machineInitialStates,
                                                         m_settings,
                                                         0,
                                                         m_settings.numOverloadActivities,
                                                         m_settings.harvestedMassLimit,
                                                         m_settings.leaveRoutePointBetweenOLActivities,
                                                         loggerPtr());
        }

        if(m_settings.numFixedInitalOlvsInOrder < 0){//estimate the optimal number of fixed OLVs to avoid unnecessary computations of computations likely to NOT be the best one
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Estimating optimal OLV order... " );
            numFixed = estimateNumFixedInitalOlvsInOrder(m_harvesterRoutes.at(i),
                                                         overloadMachines.at(i),
                                                         m_machineInitialStates);
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, std::to_string(numFixed) + " initial fixed olvs estimated automatically");
        }
        else
            numFixed = std::max( 0, std::min(m_settings.numFixedInitalOlvsInOrder, (int)overloadMachines.at(i).size() ) );

        for (int j = numFixed; j < overloadMachines.at(i).size(); ++j)//build the permutation's (initial) order (without including the initial fixed olvs)
            perm.emplace_back(j);

        bool found_plan = false;
        size_t countPermutations = 0;
        std::string sBestPermutation;

        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "started planning for permutations... " );
        do {
            double oldBestCost = bestPlanCost;

            ++countPermutations;

            //build the OLVs vector used for this permutation (i.e. fixed initial OLVs + current permutation of not-fixed OLVs)
            std::string sPermutation, permutationSubfolder = "perm_";
            std::vector<Machine> olvPermutation;
            for (int j = 0; j < numFixed; ++j){//first add the fixed OLVs
                olvPermutation.emplace_back(overloadMachines.at(i).at(j));
                sPermutation += ( std::to_string(j) + "(" + std::to_string(olvPermutation.back().id) + ") ");
                permutationSubfolder += ( std::to_string(olvPermutation.back().id) + "_" );
            }

            for (int j = 0; j < perm.size(); ++j) {//add the remaining OLVs corresponding to the current permutation
                olvPermutation.emplace_back(overloadMachines.at(i).at(perm.at(j)));
                sPermutation += ( std::to_string(perm.at(j)) + "(" + std::to_string(olvPermutation.back().id) + ") " );
                permutationSubfolder += ( std::to_string(olvPermutation.back().id) + "-" );
            }
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Planning for OLV permutation " + sPermutation);

            std::string outputFolder;
            if(!m_outputFolder.empty() && !permutationSubfolder.empty()){//update the folder where the planning (search) information the current permutation will be stored
                permutationSubfolder.back() = '/';
                outputFolder = m_outputFolder + permutationSubfolder;
            }

            std::string sError = planSingle_exclusive(i, olvPermutation, m_settings.max_planning_time-duration, outputFolder, false);
            if(sError.empty()){//a plan was computed and is better that the current BEST plan
                found_plan = true;
                logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Found new best plan with cost: " + std::to_string( bestPlanCost ) );
                logger().printOut(LogLevel::DEBUG, __FUNCTION__, "New best cost: " + std::to_string(oldBestCost) + " --> " + std::to_string( bestPlanCost ) );
                sBestPermutation = sPermutation;
            }
            else //either no plan was computed (e.g. because of an error, or the max planning time was reached), or the computed plan has a higher cost that the curren best plan
                logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Could not find a (-/better) plan: " + sError );

            duration = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "##### elapsed time since start of planning: " + std::to_string(duration) + " seconds\n" );

        } while (std::next_permutation(begin(perm), end(perm)) //update the next permutation (if there is)
                 && ( m_settings.max_planning_time < 1e-5 || duration < m_settings.max_planning_time) ); //check if the planning time limit was reached

        if (!found_plan){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Could not find a plan for route of harvester with machine_id " + std::to_string( m_harvesterRoutes.at(i).machine_id ) );
            return "Could not find a plan for route of harvester with machine_id " + std::to_string( m_harvesterRoutes.at(i).machine_id );
        }

        logger().printOut(LogLevel::INFO, __FUNCTION__, "Best plan for route of harvester with machine_id " + std::to_string( m_harvesterRoutes.at(i).machine_id )
                          + " found with cost " + std::to_string( m_bestPlan.planCosts.at(i) )
                          + " (overall plan cost = " + std::to_string( m_bestPlan.planOverallCost )
                          + ") after checking " + std::to_string(countPermutations)
                          + "/" + std::to_string( factorial(perm.size()) )
                          + " permutations ( best permutation : " + sBestPermutation + ")");

        m_graph = m_bestPlan.graph;

    }

    return "";

}

std::string MultiOLVPlanner::planAll_exclusive_singlePlan(const std::vector<std::vector<Machine>>& olvs)
{
    // Compute the plans (olv route and adjusted harvester route) for each harvester route
    // @TODO: at the moment, plans for the different harvester routes are independent of each other. however, if a harvester route has a delay, the previously planned harvester routes have to be checked (new temporal constraints (timestamps of when a vvertex was harvested / is available) may apply)
    for (int i = 0; i < m_harvesterRoutes.size(); ++i) {

        const Route &harvRoute = m_harvesterRoutes.at(i);
        auto overloadMachines = olvs.at(i);

        if(m_settings.olvOrderStrategy == ESTIMATE_OPTIMAL_ORDER){//estimate the OLVs order that is likely to be the best one
            int numFixed = 0;
            if(m_settings.numFixedInitalOlvsInOrder < 0){
                numFixed = estimateNumFixedInitalOlvsInOrder(harvRoute,
                                                             overloadMachines,
                                                             m_machineInitialStates);
                logger().printOut(LogLevel::DEBUG, __FUNCTION__, std::to_string(numFixed) + " initial fixed olvs estimated automatically");
            }
            else
                numFixed = std::max( 0, std::min(m_settings.numFixedInitalOlvsInOrder, (int)overloadMachines.size() ) );

            overloadMachines = reorderWorkingGroup(overloadMachines,
                                                   harvRoute,
                                                   m_machineInitialStates,
                                                   m_settings,
                                                   numFixed,
                                                   m_settings.numOverloadActivities,
                                                   m_settings.harvestedMassLimit,
                                                   m_settings.leaveRoutePointBetweenOLActivities,
                                                   loggerPtr());
        }

        std::string outputFolder = m_outputFolder;
        if(!outputFolder.empty() && !overloadMachines.empty()){//update the folder where the planning (search) information will be stored
            std::string permutationSubfolder;
            for (auto& m : overloadMachines)
                permutationSubfolder += ( std::to_string(m.id) + "-" );
            permutationSubfolder.back() = '/';
            outputFolder = outputFolder + permutationSubfolder;
        }

        std::string sError = planSingle_exclusive(i, overloadMachines, m_settings.max_planning_time, outputFolder, false);
        if(!sError.empty()){
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Could not find a plan for route of harvester " + std::to_string(harvRoute.machine_id)
                              + ": " + sError );
            return "Could not find a plan for route of harvester " + std::to_string(harvRoute.machine_id) + ": " + sError;
        }
    }

    return "";

}

std::string MultiOLVPlanner::planAll_shared()
{
    //@TODO

    return "";
}

std::string MultiOLVPlanner::planSingle_exclusive(size_t indHarvRoute,
                                                  std::vector<Machine> olvs,
                                                  double max_planning_time,
                                                  const std::string &folderName,
                                                  bool disableFurtherLogging)
{

    if(olvs.empty()){
        logger().printOut(LogLevel::CRITIC, __FUNCTION__, "No overloading machines assigened to harvester route");
        return "No overloading machines assigened to harvester route";
    }

    std::map<MachineId_t, size_t> plannedRoutesRefIndex;
    for(auto &m : olvs)
        plannedRoutesRefIndex[m.id] = 0;

    PlanData currentPlan;
    currentPlan.init(m_graph, m_harvesterRoutes);//initialize the current plan (for the curren permutation) with the original graph and harvester routes

    //get references to the current plan's data (changes will be applied directly!)
    auto& harvRoute = currentPlan.harvesterRoutes.at(indHarvRoute);
    auto& plannedRoutes = currentPlan.plannedRoutes;
    auto& overallDelay = currentPlan.overallDelays.at(indHarvRoute);
    auto& graph = currentPlan.graph;
    auto& planCost = currentPlan.planCosts.at(indHarvRoute);
    auto& bestPlanCost = m_bestPlan.planCosts.at(indHarvRoute);

    std::shared_ptr<Machine> harv = nullptr;
    auto it_harv = m_harvesters.find(harvRoute.machine_id);
    if(it_harv == m_harvesters.end())
        logger().printOut(LogLevel::WARNING, __FUNCTION__, 10, "Harvester id ", harvRoute.machine_id, " of route not found in working group");
    else if(it_harv->second.unload_sides == 0 )
        logger().printOut(LogLevel::WARNING, __FUNCTION__, 10, "Harvester with id ", harvRoute.machine_id, " has no downloading sides. Disregarding harvester information.");
    else
        harv = std::make_shared<Machine>(it_harv->second);

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Resource & Access points: ");

    //obtain the vertices corresponding to field access points and resource/silo points
    std::vector<DirectedGraph::vertex_t> resource_vertices;
    std::vector<DirectedGraph::vertex_t> accessPoint_vertices;

    for(const auto& it1 : graph.resourcepoint_vertex_map()){
        const ResourcePoint& resPoint = it1.first;
        if(resPoint.resourceTypes.find(ResourcePoint::ResourceType_UNLOADING) != resPoint.resourceTypes.end()){
            resource_vertices.push_back(it1.second);
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "\t\t\tAdded vertex for resource point " + std::to_string(resPoint.id) + ": " + resPoint.toString() );
        }
    }

    for(const auto& it1 : graph.accesspoint_vertex_map()){
        const FieldAccessPoint& fap = it1.first;
        accessPoint_vertices.push_back(it1.second);
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "\t\t\tAdded vertex for access point " + std::to_string(fap.id) + ": " + fap.toString() );
    }

    //get the bunker level of the fisrt OLV in the list
    double first_olv_bunker_level = 0.0;
    auto it0 = m_machineInitialStates.find(olvs.at(0).id);
    if (it0 != m_machineInitialStates.end())
        first_olv_bunker_level = it0->second.bunkerMass;

    // init new olvplan objects for each of the assigned OLVs (no planning involved)
    std::map<MachineId_t, OLVPlan> olvplan_map;//<OLV id, OLVPlan>
    for (int i = 0; i < olvs.size(); ++i){

        //get current/initial bunker level for the machine
        MachineDynamicInfo mdi;
        auto it0 = m_machineInitialStates.find(olvs.at(i).id);
        if (it0 != m_machineInitialStates.end())
            mdi = it0->second;
        else{
            mdi.bunkerMass = 0;
            mdi.bunkerVolume = 0;
            mdi.timestamp = 0;
        }

        //update the folder where the planning (search) information of the current OLV will be stored
        std::string folderName_M = folderName;
        if(!folderName_M.empty()){
            folderName_M += ( "M" + std::to_string(olvs.at(i).id) + "/" );
            if (!io::create_directory(folderName_M, true)){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating output folder '" + folderName_M + "'" );
                folderName_M.clear();
            }
        }

        //create a new olvplan for this machine and add it to the map
        auto it_plan = olvplan_map.insert(std::make_pair(olvs.at(i).id,
                                                         OLVPlan ( olvs.at(i),
                                                                   resource_vertices,
                                                                   accessPoint_vertices,
                                                                   m_settings,
                                                                   mdi,
                                                                   m_edgeCostCalculator,
                                                                   folderName_M,
                                                                   logger().logLevel())) );
        if(it_plan.second){
            if(disableFurtherLogging)//don't print anything in the OLVPlan
                it_plan.first->second.logger().setLogLevel(LogLevel::NONE);
            else//set the parent of the OLVPlan's logger to be the logger of this MultiOLVPlanner instance
                it_plan.first->second.logger().setParent(loggerPtr());
        }
    }

    int numOverloadActivities = m_settings.numOverloadActivities;
    if(numOverloadActivities <= 0)
        numOverloadActivities = std::numeric_limits<int>::max();

    auto overloadActivities = OverloadActivitiesPlanner::computeOverloadActivities(m_settings,
                                                                                   harvRoute,
                                                                                   olvs,
                                                                                   m_machineInitialStates,
                                                                                   m_settings.numOverloadActivities,
                                                                                   m_settings.harvestedMassLimit,
                                                                                   m_settings.leaveRoutePointBetweenOLActivities,
                                                                                   loggerPtr());

    //update harv routes and OLActivites if the activities start in the same rp taht the previous activity finished
    int countAddedRPs = 0;
    for(size_t i = 1 ; i < overloadActivities.size() ; ++i){
        auto& act = overloadActivities.at(i);
        auto& actPrev = overloadActivities.at(i-1);
        act.start_index += countAddedRPs;
        act.end_index += countAddedRPs;
        if(act.start_index == actPrev.end_index){
            auto rp = harvRoute.route_points.at(act.start_index);
            harvRoute.route_points.insert( harvRoute.route_points.begin() + act.start_index, rp );
            ++countAddedRPs;
            ++act.start_index;
            ++act.end_index;
        }
    }

    //check which of the OLVs were assigned an overload window and save their ids
    std::set<MachineId_t> olvsInDutty;
    for(auto &a : overloadActivities){
        if( olvsInDutty.find(a.machine.id) != olvsInDutty.end() )
            break;
        olvsInDutty.insert(a.machine.id);
    }

    logger().printOut(LogLevel::INFO, __FUNCTION__, "Computed " + std::to_string( overloadActivities.size() ) + " overloadActivities (" + (m_settings.switchOnlyAtTrackEnd?"SWITCH_AT_TRACK_END_ONLY":"SWITCH OVERALL") + ")");

    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

    // plan each overload activity/window
    std::vector< std::pair<MachineId_t, double> > overload_info_prev;

    for(size_t ola_ind = 0 ; ola_ind < overloadActivities.size() ; ++ola_ind){
        auto& ola = overloadActivities.at(ola_ind);
        double delay = 0.0;
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Planning overload for olv " + std::to_string( ola.machine.id ) + "..." );

        OLVPlan &olvplan = olvplan_map.at(ola.machine.id);//reference!

        double duration = 0.001 * std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
        if(max_planning_time > 1e-5 && duration > max_planning_time){
            logger().printOut(LogLevel::WARNING, __FUNCTION__, "Planning aborted: maximum planning time reached: " + std::to_string(duration) + "/" + std::to_string(max_planning_time) );
            return "Planning aborted: maximum planning time reached";
        }

        //plan overload with the given max waiting time
        if(! olvplan.planOverload( graph,
                                   harvRoute,
                                   harv,
                                   ola,
                                   overload_info_prev,
                                   m_settings.max_waiting_time ) ){

            logger().printOut(LogLevel::CRITIC, __FUNCTION__, "Could not find overload plan for olv " + std::to_string( ola.machine.id )
                              + " with max waiting time " + std::to_string( m_settings.max_waiting_time ) );
            return "Could not find overload plan for olv " + std::to_string( ola.machine.id )
                    + " with max waiting time " + std::to_string( m_settings.max_waiting_time );

        }

        //update harvester visit periods from the overloading segment (incl. non working segment connecting to next window)
        if(it_harv != m_harvesters.end() && m_settings.collisionAvoidanceOption != Astar::WITHOUT_COLLISION_AVOIDANCE)//update visit periods
            updateVisitPeriods(graph,
                               it_harv->second,
                               harvRoute.route_points,
                               ola.start_index,
                               ( ola_ind+1 < overloadActivities.size() ? overloadActivities.at(ola_ind+1).end_index : ola.end_index));

        delay = olvplan.getDelay();
        overallDelay += delay;
        size_t numCrossings_IF, numCrossings_HL;
        size_t numCrossings = olvplan.getNumCrossings(numCrossings_IF, numCrossings_HL);
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Found overload plan for olv " + std::to_string( ola.machine.id )
                          + ". Current cost = " + std::to_string( olvplan.getCost() )
                          + ". Current num. crossings = " + std::to_string(numCrossings)
                          + "( " + std::to_string(numCrossings_IF) + "[IF], " + std::to_string(numCrossings_HL) + "[HL] )");

        if(!updatePlanCost(currentPlan,
                           olvplan_map,
                           overallDelay,
                           harvRoute.route_points.at( ola.start_index ),//assuming the delay happened at the start of the overload activity
                           indHarvRoute,
                           harv)){//update the costs of the current plan (including the last computed route segments) and check if they are still better than the overall costs of the current BEST plan
            logger().printOut(LogLevel::WARNING, __FUNCTION__, "Planning aborted: current plan cost " + std::to_string( planCost ) + " is higher than the limit cost " + std::to_string( bestPlanCost )  );
            return "Planning aborted: current plan cost " + std::to_string( planCost ) + " is higher than the limit cost " + std::to_string( bestPlanCost );
        }

        if(!overload_info_prev.empty() && overload_info_prev.front().first == ola.machine.id)
            pop_front(overload_info_prev);
        overload_info_prev.emplace_back( std::make_pair(ola.machine.id, harvRoute.route_points.at(ola.end_index).time_stamp) );
    }

    size_t numCrossings_total = 0;
    size_t numCrossings_IF_total = 0;
    size_t numCrossings_HL_total = 0;
    for (auto &it : olvplan_map) {//finish the plans of all OLVs
        if(olvsInDutty.find(it.first) == olvsInDutty.end()
                && m_machineInitialStates.find(it.first) == m_machineInitialStates.end() )//machine is not on dutty and its current state is unknown --> do not plan any route
            continue;

        OLVPlan& olvplan = it.second;

        //compute the last route segments for this OLV
        if( !olvplan.finishPlan(graph, m_settings.sendLastOlvToResourcePoint, harvRoute, overload_info_prev) ){
            logger().printOut(LogLevel::CRITIC, __FUNCTION__, "Could not finish the plan for olv " + std::to_string( it.first ));
            return "Could not finish the plan for olv " + std::to_string( it.first );
        }

        //retrieve the complete planned route for this OLV
        plannedRoutes[it.first] = olvplan.getRoute(-1);//id will be set when calling getPlannedRoutes

        size_t numCrossings_IF, numCrossings_HL;
        size_t numCrossings = olvplan.getNumCrossings(numCrossings_IF, numCrossings_HL);
        numCrossings_IF_total += numCrossings_IF;
        numCrossings_HL_total += numCrossings_HL;
        numCrossings_total += numCrossings;
        logger().printOut(LogLevel::INFO, __FUNCTION__, "Successfully finished the plan for olv " + std::to_string( it.first ) + " with cost " + std::to_string(it.second.getCost())
                          + " and num. crossings = " + std::to_string(numCrossings)
                          + "( " + std::to_string(numCrossings_IF) + "[IF], " + std::to_string(numCrossings_HL) + "[HL] )");

        if(!updatePlanCost(currentPlan,
                           olvplan_map,
                           overallDelay,
                           Point(),
                           indHarvRoute,
                           harv)){//update the costs of the current plan (including the last computed route segment) and check if they are still better than the overall costs of the current BEST plan
            logger().printOut(LogLevel::WARNING, __FUNCTION__, "Planning aborted: current plan cost " + std::to_string( planCost ) + " is higher than the limit cost " + std::to_string( bestPlanCost )  );
            return "Planning aborted: current plan cost " + std::to_string( planCost ) + " is higher than the limit cost " + std::to_string( bestPlanCost );
        }
    }

    //remove unworked segments of the main route
    if(!overloadActivities.empty() && overloadActivities.back().end_index+1 < harvRoute.route_points.size())
        harvRoute.route_points.erase(harvRoute.route_points.begin() + overloadActivities.back().end_index + 1, harvRoute.route_points.end() );

    logger().printOut(LogLevel::INFO, __FUNCTION__, "Successfully finished the plan for olvs with a total cost " + std::to_string(planCost)
                      + ", a total delay of " + std::to_string(overallDelay)
                      + ", and a total of " + std::to_string(numCrossings_total)
                      + " crossings (IF: " + std::to_string(numCrossings_IF_total) + "; HL: " + std::to_string(numCrossings_HL_total) + ")");

    currentPlan.lastHarvIndexes.at(indHarvRoute) = harvRoute.route_points.size() - 1;//set it to the last index
    updateBestPlan(currentPlan, indHarvRoute, plannedRoutesRefIndex, true);
    return "";
}

bool MultiOLVPlanner::updatePlanCost(MultiOLVPlanner::PlanData &plan, const std::map<int, OLVPlan> &olvplan_map, double delay, const Point &delayLocation, size_t indHarvRoute, const std::shared_ptr<Machine> harv) const
{
    std::lock_guard<std::mutex> guard(m_mutex_bestPlan);

    auto &planCost = plan.planCosts.at(indHarvRoute);//reference --> updates to the costs in plan are done directly!
    planCost = 0;

    //add the plan costs of all machines
    for(auto &it : olvplan_map)
        planCost += it.second.getCost();

    if(harv)//add the cost corresponding to the delay of the harvseter routes
        planCost += m_edgeCostCalculator->calcCost(*harv, delayLocation, delayLocation, delay, delay, 0, {});

    if(std::isnan(planCost))//for debug, sometimes the cost is nan :/
        logger().printOut(LogLevel::ERROR, "m_plan_cost is NAN!");

    double bestCost = m_bestPlan.planCosts.at(indHarvRoute);
    if(std::isnan(bestCost))
        bestCost = std::numeric_limits<double>::max();

    if(m_settings.olvAssignmentStrategy == OlvAssignmentStrategy::SHARED_OLVS)//must check how to do here regarding insignificant cost difference
        return planCost < bestCost;

    //check if the current plan costs are (significantly) lower/higher than the ones from the curren best plan
    double cost_diff = planCost - bestCost;
    if( std::fabs(cost_diff) > 1e-5 )
        return cost_diff < 0;

    // the difference between the costs of the plans is not significant -> check which plans finishes first the harvesting+overload (i.e. compare the timestamps of the last route point of the routes)

    double maxTime = -1, bestMaxTime = -1;

    for(auto &it : olvplan_map)
        maxTime = std::max( maxTime, it.second.getRouteLastTimestamp(-1) );

    for(const auto& it : m_bestPlan.plannedRoutes){
        if( olvplan_map.find(it.first) == olvplan_map.end() )//for HARVESTER_EXCLUSIVE, this olv is not assigned to the harvester
            continue;
        if(it.second.route_points.empty())
            continue;
        bestMaxTime = std::max( bestMaxTime, it.second.route_points.back().time_stamp );
    }

    return maxTime < bestMaxTime;
}

std::string MultiOLVPlanner::updateBestPlan(const MultiOLVPlanner::PlanData &newPlan, size_t indHarvRoute, std::map<MachineId_t, size_t> plannedRoutesRefIndex, bool checkCost)
{
    std::lock_guard<std::mutex> guard(m_mutex_bestPlan);

    if(checkCost && m_bestPlan.planCosts.at(indHarvRoute) < newPlan.planCosts.at(indHarvRoute))
        return "New plan has higher costs than the current best plan";

    m_bestPlan.graph = newPlan.graph;
    m_bestPlan.lastHarvIndexes.at(indHarvRoute) = newPlan.lastHarvIndexes.at(indHarvRoute);
    m_bestPlan.harvesterRoutes.at(indHarvRoute) = newPlan.harvesterRoutes.at(indHarvRoute);
    m_bestPlan.overallDelays.at(indHarvRoute) = newPlan.overallDelays.at(indHarvRoute);
    m_bestPlan.planCosts.at(indHarvRoute) = newPlan.planCosts.at(indHarvRoute);

    std::set<MachineId_t> routesToRemove;//ids of the OLVS whose routes have to be completelly replaced/removed from the plan

    for(auto &it_r : m_bestPlan.plannedRoutes){
        auto it_ref = plannedRoutesRefIndex.find(it_r.first);
        if(it_ref == plannedRoutesRefIndex.end())//don't change this route
            continue;

        if (it_ref->second == 0){//this route must be completelly replaced/removed
            routesToRemove.insert(it_r.first);
            continue;
        }

        Route &route = m_bestPlan.plannedRoutes[it_r.first];
        if (it_ref->second > route.route_points.size())
            continue;

        //the route segment (starting from the given route-point index until the end) must be replaced --> delete the segment
        route.route_points.erase( route.route_points.begin() + it_ref->second, route.route_points.end() );
    }

    //delete the routes that will be completelly replaced/removed
    for(auto& id2Remove : routesToRemove)
        m_bestPlan.plannedRoutes.erase(id2Remove);

    for(auto &it_r : newPlan.plannedRoutes){
        auto it_r2 = m_bestPlan.plannedRoutes.find(it_r.first);
        if(it_r2 != m_bestPlan.plannedRoutes.end()){//add new route points to existing route
            Route &route = it_r2->second;
            route.route_points.insert( route.route_points.end(), it_r.second.route_points.begin(), it_r.second.route_points.end() );
        }
        else//add new route
            m_bestPlan.plannedRoutes[it_r.first] = it_r.second;
    }


    m_bestPlan.planOK = true;
    m_bestPlan.updateOverallCost();
    return "";

}


std::vector<Machine> MultiOLVPlanner::reorderWorkingGroup(const std::vector<Machine> &_overloadMachines,
                                                          const Route &harvesterRoute,
                                                          const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                          const OverloadActivitiesPlanner::PlannerSettings &activitiesPlannerSettings,
                                                          const size_t &numFixedInitalOlvsInOrder,
                                                          int numOverloadActivities,
                                                          double harvestedMassLimit,
                                                          bool leaveRoutePointBetweenOLActivities,
                                                          std::shared_ptr<Logger> _logger)
{
    std::vector<Machine> orderedWorkingGroup;
    std::vector<OLVPlan::OverloadInfo> overloadActivities;

    if(_overloadMachines.size() < 2)
        return _overloadMachines;

    std::vector<Machine> overloadMachines, fullMachines, rejectedMachines;
    double first_olv_bunker_level = 0.0;

    for(size_t i = 0 ; i < _overloadMachines.size() ; ++i){
        if(i < numFixedInitalOlvsInOrder){//add the initila fixed OLVs directly to the ordered list
            orderedWorkingGroup.push_back(_overloadMachines.at(i));
            continue;
        }

        //check if the machine if (almost-) full -> if it is, add it to the fullMachines list
        auto it0 = machineCurrentStates.find(_overloadMachines.at(i).id);
        if (it0 != machineCurrentStates.end()){
            if(it0->second.bunkerMass >= _overloadMachines.at(i).bunker_mass * OLVPlan::OlvMaxCapacityMultiplier){
                fullMachines.push_back(_overloadMachines.at(i));
                continue;
            }
        }

        //if the machine is neither fixed nor full, add it to the overloadMachines list
        overloadMachines.push_back(_overloadMachines.at(i));
    }


    //build a temporal ordered group to check the initial overload activities (<orderedWorkingGroup, overloadMachines, fullMachines>)
    std::vector<Machine> overloadMachinesTmp = orderedWorkingGroup;
    overloadMachinesTmp.insert( overloadMachinesTmp.end(), overloadMachines.begin(), overloadMachines.end() );
    overloadMachinesTmp.insert( overloadMachinesTmp.end(), fullMachines.begin(), fullMachines.end() );

    //compute the OL activities with the temporal working group to see where is the first overload-start point corresponding to the not-ordered machines
    overloadActivities = OverloadActivitiesPlanner::computeOverloadActivities(activitiesPlannerSettings,
                                                                              harvesterRoute,
                                                                              overloadMachinesTmp,
                                                                              machineCurrentStates,
                                                                              numOverloadActivities,
                                                                              harvestedMassLimit,
                                                                              leaveRoutePointBetweenOLActivities,
                                                                              _logger);

    if(overloadActivities.empty())//there was an error --> return the original list
        return _overloadMachines;

    size_t indCurrentActivity = 0;

    while (!overloadMachines.empty() || !fullMachines.empty()){

        Point startPoint = harvesterRoute.route_points.at( overloadActivities.at(indCurrentActivity).start_index ).point();

        //add the machine in overloadMachines that is closest to the overload-start point of the current overload-window
        if(!overloadMachines.empty()){//add first all the machines that are not full
            double minDist = std::numeric_limits<double>::max();
            int minDistInd = -1;
            for(size_t i = 0 ; i < overloadMachines.size() ; i++){
                Point machinePos = startPoint;
                double bunkerMass = 0;
                auto it0 = machineCurrentStates.find(overloadMachines.at(i).id);
                if (it0 != machineCurrentStates.end()){
                    machinePos = it0->second.position;
                    bunkerMass = it0->second.bunkerMass;
                }
                double dist = arolib::geometry::calc_dist(startPoint, machinePos);
                if(overloadMachines.at(i).bunker_mass > 1e-6)
                    dist += (1-bunkerMass/overloadMachines.at(i).bunker_mass) * 5;//if two machines are very close, choose the one that is more loaded
                if( minDist > dist ){
                    minDist = dist;
                    minDistInd = i;
                }
            }

            //update the ordered list with the machine that is closest to the curren overload-start and remove it from the overloadMachines list
            orderedWorkingGroup.push_back( overloadMachines.at(minDistInd) );
            overloadMachines.erase(overloadMachines.begin() + minDistInd);
        }
        else{//add the machine in fullMachines that is closest to the overload-start point of the current overload-window
            double maxDist = std::numeric_limits<double>::lowest();
            int maxDistInd = -1;
            for(size_t i = 0 ; i < fullMachines.size() ; i++){
                Point machinePos(0,0);
                auto it0 = machineCurrentStates.find(fullMachines.at(i).id);
                if (it0 != machineCurrentStates.end())
                    machinePos = it0->second.position;
                double dist = arolib::geometry::calc_dist(startPoint, machinePos);
                if( maxDist < dist ){
                    maxDist = dist;
                    maxDistInd = i;
                }
            }

            //update the ordered list with the machine that is closest to the curren overload-start and remove it from the fullMachines list
            orderedWorkingGroup.push_back( fullMachines.at(maxDistInd) );
            fullMachines.erase(fullMachines.begin() + maxDistInd);
        }

        //create a temporal working group with the new addition to orderedWorkingGroup, completed with the remaining machines that are not in order (<orderedWorkingGroup, overloadMachines, fullMachines>)
        overloadMachinesTmp = orderedWorkingGroup;
        overloadMachinesTmp.insert( overloadMachinesTmp.end(), overloadMachines.begin(), overloadMachines.end() );
        overloadMachinesTmp.insert( overloadMachinesTmp.end(), fullMachines.begin(), fullMachines.end() );

        if(orderedWorkingGroup.size() == 1){//this will happen iif not fixed machines were initially added to the ordered list
            auto it0 = machineCurrentStates.find(orderedWorkingGroup.front().id);
            if (it0 != machineCurrentStates.end()) {
                first_olv_bunker_level = it0->second.bunkerMass;
            }
        }

        //compute the OL activities with the temporal working group to see where is the first overload-start point corresponding to the not-ordered machines
        overloadActivities = OverloadActivitiesPlanner::computeOverloadActivities(activitiesPlannerSettings,
                                                                                  harvesterRoute,
                                                                                  overloadMachinesTmp,
                                                                                  machineCurrentStates,
                                                                                  numOverloadActivities,
                                                                                  harvestedMassLimit,
                                                                                  leaveRoutePointBetweenOLActivities,
                                                                                  _logger);

        //check if the OL activities planner disregarded the last-added machine to the ordered workinggroup
        if( overloadActivities.at(indCurrentActivity).machine.id != orderedWorkingGroup.back().id
                || overloadActivities.at(indCurrentActivity).toResourcePointFirst ){
            rejectedMachines.push_back( orderedWorkingGroup.back() );
            orderedWorkingGroup.pop_back();
        }
        else
            indCurrentActivity++;

        if(indCurrentActivity >= overloadActivities.size())//overload activities for the complete harvester route have been calculated --> no mora machines are needed
            break;

    }

//    //check if the remaining rejected machines have to be added  (they are needed to complete all the activities)
//    if(orderedWorkingGroup.size() < overloadActivities.size())
//        orderedWorkingGroup.insert( orderedWorkingGroup.end(), rejectedMachines.begin(), rejectedMachines.end() );

    //add the remaining olvs to the working group (they might need to be sent to an exit point, even if no OL activity will be scheduled for them
    orderedWorkingGroup.insert( orderedWorkingGroup.end(), overloadMachines.begin(), overloadMachines.end() );
    orderedWorkingGroup.insert( orderedWorkingGroup.end(), fullMachines.begin(), fullMachines.end() );
    orderedWorkingGroup.insert( orderedWorkingGroup.end(), rejectedMachines.begin(), rejectedMachines.end() );

    return orderedWorkingGroup;

}

size_t MultiOLVPlanner::estimateNumFixedInitalOlvsInOrder(const Route &harv_route, const std::vector<Machine> &olv_machines, const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates, Logger *_logger)
{
    if(olv_machines.size() < 2)
        return 0;

    size_t numFixed = 0;
    const double eps_dist = 20;

    //get the index of the route-point of the harvester route corresponding to the first point where there is something to harvest (i.e. corresponding to the first overload-start)
    Point initHarvPos;
    for(auto &rp : harv_route.route_points){
        if( rp.isOfWorkingType(RoutePoint::HARVESTING) && rp.time_stamp >= -0.00001 ){
            initHarvPos = rp.point();
            break;
        }
    }

    for(size_t i = 0 ; i < olv_machines.size() ; ++i){
        auto &m = olv_machines.at(i);
        if(m.machinetype != Machine::OLV)
            continue;
        auto it1 = machineCurrentStates.find(m.id);
        if (it1 == machineCurrentStates.end())//if no current state is given don't check anymore
            return numFixed;
        if(it1->second.bunkerMass >= m.bunker_mass * OLVPlan::OlvMaxCapacityMultiplier)//if it is relativelly full, stop
            return numFixed;
        if( arolib::geometry::calc_dist( it1->second.position, initHarvPos ) > eps_dist )//if it is relativelly far from the first OL start, stop
            return numFixed;
        if(it1->second.bunkerMass < 0.01 * m.bunker_mass
                && i+1 < olv_machines.size()){//this machine is not overloading. it should be fixed iif the next machine is not very close to the harvester
            auto &m2 = olv_machines.at(i+1);
            auto it2 = machineCurrentStates.find(m2.id);
            if (it2 != machineCurrentStates.end()
                    && it2->second.bunkerMass < m.bunker_mass * OLVPlan::OlvMaxCapacityMultiplier
                    && arolib::geometry::calc_dist( it2->second.position, initHarvPos ) < eps_dist){
                return numFixed;
            }
        }
        //the machine should be fixed
        ++numFixed;
    }
    return numFixed;
}

void MultiOLVPlanner::updateHarvesterRouteRelations()
{
    for(auto &harvRoute : m_bestPlan.harvesterRoutes){
        for(auto &olvRoute_it : m_bestPlan.plannedRoutes){
            auto &olvRoute = olvRoute_it.second;
            for(size_t i = 0 ; i < olvRoute.route_points.size() ; ++i){
                for(auto &mir : olvRoute.route_points.at(i).machineRelations){
                    if( harvRoute.machine_id == mir.machine_id &&
                            harvRoute.route_id == mir.route_id &&
                            mir.routePointIndex < harvRoute.route_points.size() ){
                        RoutePoint::MachineRelationInfo mir_harv;
                        mir_harv.machine_id = olvRoute.machine_id;
                        mir_harv.route_id = olvRoute.route_id;
                        mir_harv.routePointIndex = i;
                        mir_harv.routePointType = olvRoute.route_points.at(i).type;
                        harvRoute.route_points.at( mir.routePointIndex ).machineRelations.push_back(mir_harv);
                    }
                }
            }
        }
    }
}

}
