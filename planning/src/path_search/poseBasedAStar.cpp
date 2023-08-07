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
 
#include "arolib/planning/path_search/poseBasedAStar.hpp"

namespace arolib{

const PoseBasedAstar::Vertex_t::VType PoseBasedAstar::AccessVertexTypeValidity_notFound = Vertex_t::TYPE_OF_INITIAL_LOCATION;
const PoseBasedAstar::Vertex_t::VType PoseBasedAstar::AccessVertexTypeValidity_notCheched = Vertex_t::TYPE_IF_POINT;

void PoseBasedAstar::NodeInfo::updateFrom(std::shared_ptr<const NodeInfo> predecessorNode, const IPoseBasedCostCalculator::CostExtended_t &costExt)
{
    predecessor = predecessorNode;
    g = predecessorNode->g + costExt.cost;
    time = predecessorNode->time + costExt.time;
    waitTime = predecessorNode->waitTime + costExt.waitTime;
    numCrossings = predecessorNode->numCrossings + costExt.numCrossings;
    numCrossings_HL = predecessorNode->numCrossings_HL + costExt.numCrossings_HL;
    connection_path = costExt.path;
}

PoseBasedAstar::Settings::Settings(bool _doFinalSearch)
    : doFinalSearch(_doFinalSearch)
{

}

bool PoseBasedAstar::Settings::parseFromStringMap(PoseBasedAstar::Settings &params, const std::map<std::string, std::string> &map, bool strict)
{

    Settings tmp;

    std::map<std::string, bool*> bMap = { {"doFinalSearch" , &tmp.doFinalSearch} };

    if( !setValuesFromStringMap( map, bMap, strict) )
        return false;

    params = tmp;
    return true;
}

std::map<std::string, std::string> PoseBasedAstar::Settings::parseToStringMap(const PoseBasedAstar::Settings &params)
{
    std::map<std::string, std::string> ret;
    ret["doFinalSearch"] = std::to_string( params.doFinalSearch );
    return ret;

}

PoseBasedAstar::PoseBasedAstar(const PoseBasedAstar::Settings &settings, std::shared_ptr<Logger> _logger)
    : LoggingComponent(_logger, __FUNCTION__)
    , m_settings(settings)
{

}

bool PoseBasedAstar::plan(const PoseBasedGraph &graph,
                          const Pose2D &start_pose,
                          const Pose2D &goal_pose,
                          const PoseBasedAstar::PlanParameters &params,
                          std::shared_ptr<IPoseBasedCostCalculator> costCalculator)
{
    if( is_nan(start_pose.angle) ){
        logger().printError(__FUNCTION__, "Invalid start pose angle");
        return false;
    }

    Vertex_t start_vt;
    start_vt.pose = start_pose;
    start_vt.direction = Vertex_t::DIR_PARALLEL;
    start_vt.type = Vertex_t::TYPE_IF_POINT;
    start_vt.id = -1;
    while (graph.vertexIdExists(start_vt.id))
        start_vt.id--;

    Vertex_t goal_vt;
    goal_vt.pose = goal_pose;
    goal_vt.direction = Vertex_t::DIR_PARALLEL;
    goal_vt.type = Vertex_t::TYPE_IF_POINT;
    goal_vt.id = start_vt.id-1;
    while (graph.vertexIdExists(goal_vt.id))
        goal_vt.id--;

    return plan( graph, start_vt, goal_vt, params, costCalculator );
}

bool PoseBasedAstar::plan(const PoseBasedGraph &graph,
                          const Pose2D &start_pose,
                          const VertexId_t &goal_vt_id,
                          const PoseBasedAstar::PlanParameters &params,
                          std::shared_ptr<IPoseBasedCostCalculator> costCalculator)
{
    if( is_nan(start_pose.angle) ){
        logger().printError(__FUNCTION__, "Invalid start pose angle");
        return false;
    }

    Vertex_t start_vt;
    start_vt.pose = start_pose;
    start_vt.direction = Vertex_t::DIR_PARALLEL;
    start_vt.type = Vertex_t::TYPE_IF_POINT;
    start_vt.id = -1;
    while (graph.vertexIdExists(start_vt.id))
        start_vt.id--;

    Vertex_t goal_vt;
    if(!graph.getVertex(goal_vt_id, goal_vt)){
        logger().printError(__FUNCTION__, "Goal vertex " + std::to_string(goal_vt_id) + " not found in graph");
        return false;
    }

    return plan( graph, start_vt, goal_vt, params, costCalculator );
}

bool PoseBasedAstar::plan(const PoseBasedGraph &graph,
                          const VertexId_t &start_vt_id,
                          const Pose2D &goal_pose,
                          const PoseBasedAstar::PlanParameters &params,
                          std::shared_ptr<IPoseBasedCostCalculator> costCalculator)
{
    Vertex_t start_vt;
    if(!graph.getVertex(start_vt_id, start_vt)){
        logger().printError(__FUNCTION__, "Start vertex " + std::to_string(start_vt_id) + " not found in graph");
        return false;
    }

    Vertex_t goal_vt;
    goal_vt.pose = goal_pose;
    goal_vt.direction = Vertex_t::DIR_PARALLEL;
    goal_vt.type = Vertex_t::TYPE_IF_POINT;
    goal_vt.id = start_vt.id-1;
    while (graph.vertexIdExists(goal_vt.id))
        goal_vt.id--;

    return plan( graph, start_vt, goal_vt, params, costCalculator );
}

bool PoseBasedAstar::plan(const PoseBasedGraph &graph,
                          const VertexId_t &start_vt_id,
                          const VertexId_t &goal_vt_id,
                          const PoseBasedAstar::PlanParameters &params,
                          std::shared_ptr<IPoseBasedCostCalculator> costCalculator)
{
    Vertex_t start_vt;
    if(!graph.getVertex(start_vt_id, start_vt)){
        logger().printError(__FUNCTION__, "Start vertex " + std::to_string(goal_vt_id) + " not found in graph");
        return false;
    }

    Vertex_t goal_vt;
    if(!graph.getVertex(goal_vt_id, goal_vt)){
        logger().printError(__FUNCTION__, "Goal vertex " + std::to_string(goal_vt_id) + " not found in graph");
        return false;
    }

    return plan( graph, start_vt, goal_vt, params, costCalculator );
}

std::vector<Point> PoseBasedAstar::getPathToNode(std::shared_ptr<const PoseBasedAstar::NodeInfo> node, double maxLength, bool startAtNode)
{
    std::vector<Point> path;
    double len = 0;
    while(node){
        if(!node->predecessor || node->connection_path.empty()){//only add the point
            path.emplace_back( node->vertex.pose );
            if( maxLength > 0 && node->predecessor ){
                len += arolib::geometry::calc_dist(node->vertex.pose, node->predecessor->vertex.pose);
                if(len >= maxLength)
                    break;
            }
        }
        else{
            path.reserve( path.size() + node->connection_path.size() - 1);//the point of the predecessor is not added
            bool maxLengthReached = false;
            for(size_t i = 0 ; i+1 < node->connection_path.size() ; ++i){
                const Point& p1 = r_at(node->connection_path, i).p;
                path.emplace_back( p1 );
                if( maxLength > 0 ){
                    const Point& p2 = r_at(node->connection_path, i+1).p;
                    len += arolib::geometry::calc_dist(p1, p2);
                    if(len >= maxLength){
                        path.emplace_back( p2 );
                        maxLengthReached = true;
                        break;
                    }
                }
            }
            if(maxLengthReached)
                break;

        }
        node = node->predecessor;
    }

    if(!startAtNode)
        std::reverse(path.begin(), path.end());

    return path;
}

bool PoseBasedAstar::plan(const PoseBasedGraph &graph,
                          const Vertex_t &start_vt,
                          const Vertex_t &goal_vt,
                          const PoseBasedAstar::PlanParameters &params,
                          std::shared_ptr<IPoseBasedCostCalculator> costCalculator)
{    
    if(!params.vertexFilterFct(goal_vt)){
        logger().printError(__FUNCTION__, "Goal vertex was filtered out by vertexFilterFct");
        return false;
    }

    SearchInternalData sd;
    sd.current_node = std::make_shared<NodeInfo>();
    sd.bestFieldExit.type = AccessVertexTypeValidity_notCheched;//for init


    std::shared_ptr<NodeInfo> goal_node = nullptr;

    m_result = Plan();

    // insert start node into open list
    sd.current_node->vertex = start_vt;
    sd.current_node->h = costCalculator->calcHeuristic(graph, start_vt, goal_vt, params.machine);
    sd.current_node->g = 0.0;
    sd.current_node->time = params.start_time;
    sd.current_node->waitTime = 0.0;
    sd.current_node->predecessor = nullptr;

    addToOpenList(sd.open_list, sd.current_node);

    while(!sd.open_list.sortedNodes.empty()){
        sd.current_node = removeTopFromOpenList(sd.open_list);// get first vertex from open_list (i.e. vertex with minimum f value)

        // copy current_node to closed_list
        sd.closed_list.insert( std::make_pair(sd.current_node->vertex.id, sd.current_node ));

        if(sd.current_node->vertex.id == goal_vt.id){//if the current node is the goal, no need to expand
            goal_node = sd.current_node;
            break;
        }

        //debug!
        std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

        // call function expandNode to find valid successors of current_node
        expandNode(graph,
                   sd,
                   goal_vt,
                   params,
                   costCalculator);

        //debug!
        double duration = 0.001*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
        std::cout << "*** duration expandNode = " << duration << std::endl;
    }


    if(goal_node && m_settings.doFinalSearch)
        checkForSmootherSolution(graph, goal_node, sd.closed_list, params, costCalculator);

    //update results, inlc. building the resulting route if a plan was found
    updateResult(goal_node, sd.closed_list, start_vt, goal_vt, params);

    logger().printDebug(__FUNCTION__, 10, "Planning finished with ",
                        sd.closed_list.size(), " nodes in the closed list and ",
                        sd.open_list.nodes.size(), " in the open list (total visited nodes = ",
                        sd.closed_list.size() + sd.open_list.nodes.size(), ")");

    return m_result.valid;

}

void PoseBasedAstar::expandNode(const PoseBasedGraph &graph,
                                SearchInternalData &sd,
                                const PoseBasedAstar::Vertex_t &goal_vt,
                                const PoseBasedAstar::PlanParameters &params,
                                std::shared_ptr<IPoseBasedCostCalculator> costCalculator)
{

    //debug!
    std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

    auto successors = getValidSuccessors(graph, sd, goal_vt, params, costCalculator);
    appendBestGuessSuccessors(successors, graph, sd, goal_vt, params, costCalculator);

    //debug!
    double duration = 0.001*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
    //std::cout << "\t*** duration getValidSuccessors = " << duration << " : #successors = " << successors.size() << std::endl;
    time_start = std::chrono::steady_clock::now();

    for(auto& suc_it : successors){
        auto& suc = suc_it.second;
        auto it_ol = sd.open_list.nodes.find( suc->vertex.id );
        if( it_ol != sd.open_list.nodes.end() ){
            if( it_ol->second->g + it_ol->second->h <= suc->g + suc->h )//in open list and better
                continue;
            replaceInOpenList(sd.open_list, suc);
        }
        else
            addToOpenList(sd.open_list, suc);
    }

    //debug!
    duration = 0.001*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
    //std::cout << "\t*** duration add/repalce in open list = " << duration << std::endl;
    time_start = std::chrono::steady_clock::now();
}


bool PoseBasedAstar::addToOpenList(OpenList &open_list, std::shared_ptr<NodeInfo> node)
{
    if(open_list.nodes.find(node->vertex.id) != open_list.nodes.end()){
        return false;
    }

    open_list.sortedNodes.insert( std::make_pair( node->g + node->h, node->vertex.id ) );
    open_list.nodes.insert( std::make_pair( node->vertex.id, node ) );
    return true;
}

std::shared_ptr<PoseBasedAstar::NodeInfo> PoseBasedAstar::removeTopFromOpenList(OpenList &open_list)
{
    auto top_node_id = open_list.sortedNodes.begin()->second;
    auto top_node_it = open_list.nodes.find(top_node_id);
    auto top_node = top_node_it->second;
    open_list.sortedNodes.erase(open_list.sortedNodes.begin());
    open_list.nodes.erase(top_node_it);
    return top_node;
}

bool PoseBasedAstar::replaceInOpenList(PoseBasedAstar::OpenList &open_list, std::shared_ptr<PoseBasedAstar::NodeInfo> node)
{
    auto node_it = open_list.nodes.find(node->vertex.id);
    if(node_it == open_list.nodes.end())
        return false;


    Cost_t cost = node_it->second->g + node_it->second->h;
    auto it_ol = open_list.sortedNodes.find( cost );
    if( it_ol == open_list.sortedNodes.end() ){//do hardish search, starting arround where the node should be
        if(open_list.sortedNodes.empty())
            return false;
        auto it1 = open_list.sortedNodes.lower_bound( cost );
        auto it2 = it1;
        if( it1 == open_list.sortedNodes.end() )
            it1--;
        else
            it2++;
        if( it2 == open_list.sortedNodes.end() )
            it2--;

        bool checkIt1 = true, checkIt2 = true;
        while( checkIt1 || checkIt2 ){
            if( checkIt1 && it1->second == node->vertex.id ){
                it_ol = it1;
                break;
            }
            if( it1 != open_list.sortedNodes.begin() )
                it1--;
            else
                checkIt1 = false;

            if( checkIt2 && it2->second == node->vertex.id ){
                it_ol = it2;
                break;
            }
            if( it2 != open_list.sortedNodes.begin() )
                it2--;
            else
                checkIt2 = false;
        }
    }
    if( it_ol == open_list.sortedNodes.end() )
        return false;

    open_list.sortedNodes.erase(it_ol);
    open_list.sortedNodes.insert( std::make_pair( node->g + node->h, node->vertex.id ) );
    node_it->second = node;

    return true;
}

std::unordered_map<PoseBasedAstar::VertexId_t, std::shared_ptr<PoseBasedAstar::NodeInfo> > PoseBasedAstar::getValidSuccessors(const PoseBasedGraph &graph,
                                                                                                                              SearchInternalData &sd,
                                                                                                                              const PoseBasedAstar::Vertex_t &goal_vt,
                                                                                                                              const PoseBasedAstar::PlanParameters &params,
                                                                                                                              std::shared_ptr<IPoseBasedCostCalculator> costCalculator)
{

    //debug!
    auto time_start = std::chrono::steady_clock::now();


    std::unordered_map<PoseBasedAstar::VertexId_t, std::shared_ptr<PoseBasedAstar::NodeInfo> > successors;
    if( sd.current_node->vertex.isOfOutfieldType(false)
            || sd.current_node->vertex.isFieldExitType() ){//use OF connections

        const auto& edges = graph.getOFConnections();
        auto it_start = edges.find(sd.current_node->vertex.id);
        if(it_start == edges.end())
            return successors;

        //debug!
        double duration = 0.001*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
        //std::cout << "\t\t*** duration getOFConnections = " << duration << std::endl;
        time_start = std::chrono::steady_clock::now();

        for(const auto& it_suc : it_start->second){

            if( sd.closed_list.find(it_suc.first) != sd.closed_list.end() )
                continue;

            auto it_m = it_suc.second.find( params.machine.id );
            if( it_m == it_suc.second.end() )//no specific connection for this machine
                it_m = it_suc.second.find( OutFieldInfo::AllMachines );
            if( it_m == it_suc.second.end() )//no connection for this machine
                continue;

            Vertex_t vt;
            if(!graph.getVertex(it_suc.first, vt))
                continue;

            if(!params.vertexFilterFct(vt))
                continue;

            auto cost = costCalculator->calcCost(it_m->second, params.machine, sd.current_node->time, params.initial_bunker_mass);
            if(cost.valid != IPoseBasedCostCalculator::CostExtended_t::VALID)
                continue;

            //debug!
            duration = 0.001*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
            //std::cout << "\t\t\t*** duration calcCost = " << duration << std::endl;
            time_start = std::chrono::steady_clock::now();

            std::shared_ptr<NodeInfo> successor = std::make_shared<NodeInfo>();
            successor->updateFrom( sd.current_node, cost );
            successor->h = costCalculator->calcHeuristic(graph, vt, goal_vt, params.machine);
            successor->vertex = vt;
            successor->connection_path.clear();

            successors.insert( std::make_pair(successor->vertex.id, successor) );

            //debug!
            duration = 0.001*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
            //std::cout << "\t\t\t*** duration calcHeuristic = " << duration << std::endl;
            time_start = std::chrono::steady_clock::now();
        }

        return successors;
    }

    //search for IF vertices

    float search_radius = params.searchRadius;
    if(search_radius < 1e-6)
        search_radius = 2 * params.machine.getTurningRadius();

    auto vertices = graph.getVerticesInRadius(sd.current_node->vertex.pose, search_radius, [&](const Vertex_t& vt)->bool{
                                                                                                    if( vt.id == sd.current_node->vertex.id
                                                                                                            || is_nan(vt.pose.angle)
                                                                                                            || vt.isOfOutfieldType(false)
                                                                                                            || vt.isFieldEntryType()
                                                                                                            || sd.closed_list.find(vt.id) != sd.closed_list.end()
                                                                                                            || sd.invalid_vts.find(vt.id) != sd.invalid_vts.end()
                                                                                                            || arolib::geometry::calc_dist(vt.pose, sd.current_node->vertex.pose) < params.noSearchRadius
                                                                                                            || !params.vertexFilterFct(vt) )
                                                                                                        return false;
                                                                                                    if( costCalculator->isVertexValid(vt, &params.machine, sd.current_node->time, params.restrictedMachineIds) != IPoseBasedCostCalculator::CostExtended_t::VALID ){
                                                                                                        sd.invalid_vts.insert(vt.id);
                                                                                                        return false;
                                                                                                    }
                                                                                                    return true;
                                                                                               });

    //debug!
    double duration = 0.001*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
    //std::cout << "\t\t*** duration getVerticesInRadius = " << duration << std::endl;
    time_start = std::chrono::steady_clock::now();

    if(!goal_vt.isOfOutfieldType(false) && !graph.vertexIdExists(goal_vt.id)){//check also the goal_vt if it is a pseudo vertex created from a pose
        if( arolib::geometry::calc_dist(goal_vt.pose, sd.current_node->vertex.pose) <= search_radius )
            vertices.push_back(goal_vt);
    }

    return getIFSuccessors(graph, sd, goal_vt, vertices, params, costCalculator);

}

void PoseBasedAstar::appendBestGuessSuccessors(std::unordered_map<VertexId_t, std::shared_ptr<NodeInfo> >& successors,
                                               const PoseBasedGraph &graph,
                                               SearchInternalData &sd,
                                               const Vertex_t &goal_vt,
                                               const PoseBasedAstar::PlanParameters &params,
                                               std::shared_ptr<IPoseBasedCostCalculator> costCalculator)
{
    if(sd.current_node->vertex.isOfOutfieldType(false)
            || sd.current_node->vertex.isFieldExitType() )//only infield search
        return;

    bool checkForBestGuess = !sd.current_node->predecessor//start node
                             || !sd.current_node->predecessor->predecessor//to allow manouvers out of the start node
                             || (sd.current_node->vertex.isFieldEntryType() && !goal_vt.isOfOutfieldType(true)
                             || params.probDirectSearch >= gen_random_double(0, 1));

    if(!checkForBestGuess)
        return;

    std::unordered_map<VertexId_t, Vertex_t> suc_vts;

    if(goal_vt.type == Vertex_t::TYPE_OF_RESOURCE_POINT){
        auto exit_vts = graph.getExitVertices();

        //add the exit vertex with lowest OF cost to reach the OF goal vt
        if(sd.bestFieldExit.type != Vertex_t::TYPE_ACCESSPOINT && sd.bestFieldExit.type != AccessVertexTypeValidity_notFound){
            //@todo search for best

            std::unordered_set<VertexId_t> exit_vts_ids;
            for(auto& vt_it : exit_vts)
                exit_vts_ids.insert(vt_it.second.begin(), vt_it.second.end());

            VertexId_t dummy;
            Cost_t bestCost;
            if(!costCalculator->getBestOutfieldConnection(graph,
                                                          exit_vts_ids,
                                                          {goal_vt.id},
                                                          params.machine.id,
                                                          sd.bestFieldExit.id,
                                                          dummy,
                                                          bestCost)
                || !graph.getVertex(sd.bestFieldExit.id, sd.bestFieldExit) ){
                sd.bestFieldExit.type = AccessVertexTypeValidity_notFound;
            }
        }
        if(sd.bestFieldExit.type == Vertex_t::TYPE_ACCESSPOINT)
            suc_vts[sd.bestFieldExit.id] = sd.bestFieldExit;


        //add the exit vertex closest to the current node
        Vertex_t closestFieldExit;
        closestFieldExit.type = AccessVertexTypeValidity_notFound;
        double minDist = std::numeric_limits<double>::max();
        for(auto& fap_it : exit_vts){
            for(auto vtId : fap_it.second){
                Vertex_t vtTmp;
                if( !graph.getVertex(vtId, vtTmp) )
                    continue;
                double dist = arolib::geometry::calc_dist(sd.current_node->vertex.pose, vtTmp.pose);
                if(minDist > dist){
                    minDist = dist;
                    closestFieldExit = vtTmp;
                }
            }
        }
        if(closestFieldExit.type == Vertex_t::TYPE_ACCESSPOINT)
            suc_vts[closestFieldExit.id] = closestFieldExit;

    }
    else if (!goal_vt.isOfOutfieldType(true))
        suc_vts[goal_vt.id] = goal_vt;


    for(auto& vt_it : suc_vts){
        if(successors.find(vt_it.second.id) != successors.end())
            continue;

        auto cost = getCostOfShortestValidPath(sd.current_node, vt_it.second, params, costCalculator);
        if(cost.valid == IPoseBasedCostCalculator::CostExtended_t::VALID){
            std::shared_ptr<NodeInfo> successor = std::make_shared<NodeInfo>();
            successor->updateFrom( sd.current_node, cost );
            successor->h = costCalculator->calcHeuristic(graph, vt_it.second, goal_vt, params.machine);
            successor->vertex = vt_it.second;
            successors[successor->vertex.id] = successor;
        }
    }
}

std::unordered_map<PoseBasedAstar::VertexId_t, std::shared_ptr<PoseBasedAstar::NodeInfo> > PoseBasedAstar::getIFSuccessors(const PoseBasedGraph &graph,
                                                                                                                           const SearchInternalData &sd,
                                                                                                                           const PoseBasedAstar::Vertex_t &goal_vt,
                                                                                                                           const std::vector<PoseBasedAstar::Vertex_t> &vertices,
                                                                                                                           const PoseBasedAstar::PlanParameters &params,
                                                                                                                           std::shared_ptr<IPoseBasedCostCalculator> costCalculator)
{
    //debug!
    auto time_start = std::chrono::steady_clock::now();
    auto time_start2 = std::chrono::steady_clock::now();

    std::unordered_map<PoseBasedAstar::VertexId_t, std::shared_ptr<PoseBasedAstar::NodeInfo> > successors;
    for(auto& vt : vertices){
        auto h =  costCalculator->calcHeuristic(graph, vt, goal_vt, params.machine);
        auto g_min = costCalculator->calcHeuristic(graph, sd.current_node->vertex, vt, params.machine);

        auto it_ol = sd.open_list.nodes.find( vt.id );
        if( it_ol != sd.open_list.nodes.end() ){
            if( h + g_min > it_ol->second->g + it_ol->second->h )//in open list and better
                continue;
        }

        auto cost = getCostOfShortestValidPath(sd.current_node, vt, params, costCalculator);

        //debug!
        double duration = 0.001*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
        //std::cout << "\t\t\t*** duration getCostOfShortestValidPath = " << duration << std::endl;
        time_start = std::chrono::steady_clock::now();

        if(cost.valid != IPoseBasedCostCalculator::CostExtended_t::VALID)
            continue;

        std::shared_ptr<NodeInfo> successor = std::make_shared<NodeInfo>();
        successor->updateFrom( sd.current_node, cost );
        successor->h = h;
        successor->vertex = vt;

        successors.insert( std::make_pair(successor->vertex.id, successor) );

        //debug!
        duration = 0.001*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
        //std::cout << "\t\t\t*** duration calcHeuristic = " << duration << std::endl;
        time_start = std::chrono::steady_clock::now();
    }

    //debug!
    double duration = 0.001*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start2).count();
    //std::cout << "\t\t*** duration getIFSuccessors = " << duration << std::endl;

    return successors;
}

IPoseBasedCostCalculator::CostExtended_t PoseBasedAstar::getCostOfShortestValidPath(std::shared_ptr<const NodeInfo> node_from,
                                                                                   const PoseBasedAstar::Vertex_t &vt_to,
                                                                                   const PlanParameters &params,
                                                                                   std::shared_ptr<IPoseBasedCostCalculator> costCalculator)
{
    IPoseBasedCostCalculator::CostExtended_t cost;
    cost.valid = IPoseBasedCostCalculator::CostExtended_t::INVALID;

    float turningRadius = std::max(1e-6, params.machine.turning_radius);

    const auto& vt_from = node_from->vertex;

    arolib::geometry::DubinsParams dp;
    dp.p1 = vt_from.pose.point();
    dp.p2 = vt_to.pose.point();
    dp.rho1 = vt_from.pose.angle;
    dp.rho2 = vt_to.pose.angle;

    double res = arolib::geometry::calc_dist(vt_from.pose, vt_to.pose) / 20;
    if(params.machine.length > 0)
        res = std::min(res, 0.5 * params.machine.length);


    //debug!
    auto time_start = std::chrono::steady_clock::now();
    auto time_start2 = std::chrono::steady_clock::now();

    int iPathType = 0;

    std::multimap<double, std::vector<Point>> sortedPaths;

    while(1){
        try{
            dp.type = arolib::geometry::DubinsParams::intToPathType(iPathType++);
        }
        catch(...){
            break;
        }
        if(dp.type == arolib::geometry::DubinsParams::SHORTEST)
            continue;

        double len;
        auto path = calcDubinsPath(dp, turningRadius, res, &len);

        //debug!
        double duration = 0.001*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
        //std::cout << "\t\t\t\t\t*** duration calc dubins path = " << duration << " : # samples = " << path.size() << std::endl;
        time_start = std::chrono::steady_clock::now();

        if(path.size() < 2)
            continue;

        sortedPaths.insert( std::make_pair(len, path) );
    }

    for(auto& it_path : sortedPaths){
        auto& path = it_path.second;

        arolib::geometry::unsample_linestring(path);

        //debug!
        double duration = 0.001*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
        //std::cout << "\t\t\t\t\t*** unsample = " << duration << " : # samples = " << path.size() << std::endl;
        time_start = std::chrono::steady_clock::now();

        if(path.size() < 2)
            continue;

        //check if geometry of the path is valid

        //check for intersections with path (incl. previous path)
        auto pathTmp = getPathToNode(node_from, -1, false);
        pathTmp.insert(pathTmp.end(), path.begin()+1, path.end());

        bool selfInstersects = arolib::geometry::intersects(pathTmp);

        //debug!
        duration = 0.001*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
        //std::cout << "\t\t\t\t\t*** duration check path intersections (prev) = " << duration << std::endl;
        time_start = std::chrono::steady_clock::now();

        if( selfInstersects )
            continue;

        cost = costCalculator->calcCost(path, params.machine, node_from->time, params.initial_bunker_mass, params.restrictedMachineIds );

        //debug!
        duration = 0.001*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count();
        //std::cout << "\t\t\t\t\t*** duration calcCost = " << duration << std::endl;
        time_start = std::chrono::steady_clock::now();

        if( cost.valid == IPoseBasedCostCalculator::CostExtended_t::VALID )
            break;
    }

    //debug!
    double duration = 0.001*std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start2).count();
    //std::cout << "\t\t\t\t*** duration check all dubins paths = " << duration << std::endl;
    time_start = std::chrono::steady_clock::now();

    if( cost.valid != IPoseBasedCostCalculator::CostExtended_t::VALID ){
        cost = IPoseBasedCostCalculator::CostExtended_t();
        cost.valid = IPoseBasedCostCalculator::CostExtended_t::INVALID;
    }

    return cost;
}

void PoseBasedAstar::updateResult(std::shared_ptr<const NodeInfo> goal_node, const ClosedList_t &closed_list, const Vertex_t &start_vt, const Vertex_t &goal_vt, const PlanParameters &params)
{
    m_result.route_points.clear();

    m_result.start_vt = start_vt;
    m_result.goal_vt = goal_vt;
    m_result.closed_list = closed_list;
    m_result.searchRadius = params.searchRadius;
    m_result.noSearchRadius = params.noSearchRadius;

    if(!goal_node){
        m_result.valid = false;
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Astar could not find a plan.");
        return;
    }

    m_result.waitTime = goal_node->waitTime;
    m_result.numCrossings = goal_node->numCrossings;
    m_result.numCrossings_HL = goal_node->numCrossings_HL;
    m_result.cost = goal_node->g;

    auto current_node = goal_node;

    std::vector< std::shared_ptr<const NodeInfo> > nodes = {current_node};
    while(current_node->predecessor){
        current_node = current_node->predecessor;
        nodes.emplace_back(current_node);
    }
    std::reverse(nodes.begin(), nodes.end());

    for(auto node : nodes)
        appendRouteSegment(m_result.route_points, node, params);

    m_result.valid = true;

}

bool PoseBasedAstar::appendRouteSegment(std::vector<RoutePoint> &route_points,
                                        std::shared_ptr<const NodeInfo> node,
                                        const PoseBasedAstar::PlanParameters &params) {

    auto buildRoutePoint = [&params](const Point& p, double time_stamp, RoutePoint::RoutePointType type)->RoutePoint{
        RoutePoint rp;
        rp.point() = p;
        rp.type = type;
        rp.time_stamp = time_stamp;
        rp.track_id = -1;
        rp.bunker_mass = params.initial_bunker_mass;
        return rp;
    };

    auto RPType = params.defaultRPType;
    if(node->vertex.type == Vertex_t::TYPE_OF_INITIAL_LOCATION)
        RPType = RoutePoint::INITIAL_POSITION;
    else if(node->vertex.type == Vertex_t::TYPE_OF_RESOURCE_POINT)
        RPType = RoutePoint::RESOURCE_POINT;
    else if(node->vertex.type == Vertex_t::TYPE_ACCESSPOINT)
        RPType = node->vertex.direction == Vertex_t::DIR_FIELD_ENTRY ? RoutePoint::FIELD_ENTRY : RoutePoint::FIELD_EXIT;

    if(!node->predecessor || node->connection_path.empty()){//only add the point
        route_points.emplace_back( buildRoutePoint(node->vertex.pose, node->time, RPType) );
        return true;
    }

    route_points.reserve( route_points.size() + node->connection_path.size() - 1);//the route point of the predecessor is not added
    for(size_t i = 1 ; i < node->connection_path.size() ; ++i){
        const auto& p = node->connection_path.at(i);
        route_points.emplace_back( buildRoutePoint(p.p, p.time_stamp, RPType) );
    }
    return true;
}

void PoseBasedAstar::checkForSmootherSolution(const PoseBasedGraph &graph,
                                              std::shared_ptr<PoseBasedAstar::NodeInfo>& goal_node,
                                              PoseBasedAstar::ClosedList_t &closed_list,
                                              const PoseBasedAstar::PlanParameters &params,
                                              std::shared_ptr<IPoseBasedCostCalculator> costCalculator)
{
    if(!goal_node)
        return;

    bool isOFsegment = goal_node->vertex.isOfOutfieldType(false)
                            || (goal_node->vertex.type == Vertex_t::TYPE_ACCESSPOINT && goal_node->vertex.direction == Vertex_t::DIR_FIELD_ENTRY);

    //segment the plan into IF and OF segments
    std::vector< std::pair<VertexId_t, Vertex_t> > segmentsBreakpoints = { std::make_pair(goal_node->vertex.id, goal_node->vertex) };
    std::shared_ptr<const PoseBasedAstar::NodeInfo> predecessor = goal_node->predecessor;
    size_t countNodes = 1;
    while(predecessor){
        countNodes++;
        bool added = false;
        if(isOFsegment){
            if( !predecessor->vertex.isOfOutfieldType(false) ){
                segmentsBreakpoints.emplace_back( std::make_pair( predecessor->vertex.id, predecessor->vertex ) );
                isOFsegment = false;
                added = true;
            }
        }
        else{
            if( predecessor->vertex.isOfOutfieldType(true) ){
                segmentsBreakpoints.emplace_back( std::make_pair( predecessor->vertex.id, predecessor->vertex ) );
                isOFsegment = true;
                added = true;
            }
        }

        if(!predecessor->predecessor && !added)
            segmentsBreakpoints.emplace_back( std::make_pair( predecessor->vertex.id, predecessor->vertex ) );

        predecessor = predecessor->predecessor;
    }

    if(segmentsBreakpoints.size() < 3 || countNodes == segmentsBreakpoints.size())
        return;

    std::unordered_map<VertexId_t, VertexId_t> originalVtIdsMap;
    PoseBasedGraph minimizeGraph(loggerPtr());
    for(auto & vt_pair : segmentsBreakpoints){
        auto &vt = vt_pair.second;

        //NOTE: the id of the vertices will be updated when adding them to the new graph. The vertex id in the original graph is in vt_pair.first

        if( vt.type == Vertex_t::TYPE_HL_POINT || vt.type == Vertex_t::TYPE_IF_POINT )
            minimizeGraph.addTrackVertex(vt);
        else if( vt.type == Vertex_t::TYPE_ACCESSPOINT ){
            FieldAccessPointId_t fapId;
            if(!graph.getAccessPointId(vt.id, fapId))
                return;
            minimizeGraph.addAccessPointVertex(vt, fapId);
        }
        else if( vt.type == Vertex_t::TYPE_OF_RESOURCE_POINT){
            ResourcePointId_t rp_id;
            if(!graph.getResourcePointId(vt.id, rp_id))
                return;
            minimizeGraph.addResourcePointVertex(vt, rp_id);
        }
        else if( vt.type == Vertex_t::TYPE_OF_INITIAL_LOCATION){
            MachineId_t m_id;
            if(!graph.getMachineIdFromInitLoc(vt.id, m_id))
                return;
            minimizeGraph.addInitialLocationVertex(vt, m_id);
        }
        originalVtIdsMap[vt.id] = vt_pair.first;
    }

    auto ofConnections = graph.getOFConnections();
    for(auto i = 0 ; i+1 < segmentsBreakpoints.size() ; i++){
        //check them in reverse, since the order of the vertices is from goal to start
        const auto& vt1 = segmentsBreakpoints.at(i+1).second;
        const auto& vt2 = segmentsBreakpoints.at(i).second;
        const auto& vt1_originalId = segmentsBreakpoints.at(i+1).first;
        const auto& vt2_originalId = segmentsBreakpoints.at(i).first;
        if(!vt1.isOfOutfieldType(true) || !vt2.isOfOutfieldType(true))
            continue;

        auto it1 = ofConnections.find(vt1_originalId);
        if(it1 == ofConnections.end())
            continue;
        auto it2 = it1->second.find(vt2_originalId);
        if(it2 == it1->second.end())
            continue;
        minimizeGraph.connectOFVertices(vt1.id, vt2.id, false, it2->second);
    }

    auto settings2 = m_settings;
    settings2.doFinalSearch = false;
    PoseBasedAstar astar(settings2, loggerPtr());
    auto params2 = params;
    params2.searchRadius = std::numeric_limits<float>::max();
    const auto& vt_start_new = segmentsBreakpoints.back().second;
    const auto& vt_goal_new = segmentsBreakpoints.front().second;
    if(!astar.plan(minimizeGraph, vt_start_new.id, vt_goal_new.id, params2, costCalculator))
        return;
    const auto& plan = astar.getPlan();
    if( plan.cost <= goal_node->g ){
        auto it_cl = plan.closed_list.find( vt_goal_new.id );
        if(it_cl == plan.closed_list.end())
            return;
        std::shared_ptr<const PoseBasedAstar::NodeInfo> current_node = it_cl->second;

        std::shared_ptr<PoseBasedAstar::NodeInfo> prev_node = nullptr;

        while(current_node){
            auto vt_id_it = originalVtIdsMap.find(current_node->vertex.id);
            if(vt_id_it == originalVtIdsMap.end()){
                logger().printCritic(__FUNCTION__, "Original vertex id not found");
                goal_node = nullptr;
                return;
            }
            auto node_it = closed_list.find(vt_id_it->second);
            if(node_it == closed_list.end()){
                logger().printCritic(__FUNCTION__, "Node id not found in closed list");
                goal_node = nullptr;
                return;
            }

            std::shared_ptr<PoseBasedAstar::NodeInfo>& node = node_it->second;
            Vertex_t vt_original = node->vertex;
            *node = *current_node;
            node->vertex = vt_original;

            if(prev_node)//update predecessor (the current node) of previous node;
                prev_node->predecessor = node;

            current_node = current_node->predecessor;
        }
    }

}



}

