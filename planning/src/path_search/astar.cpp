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
 
#include "arolib/planning/path_search/astar.hpp"


namespace arolib{

const std::string Astar::m_filename_expansionAttempts = "expansionAttempts.txt";

Astar::CollisionAvoidanceOption Astar::intToCollisionAvoidanceOption(int value)
{
    if(value == CollisionAvoidanceOption::WITHOUT_COLLISION_AVOIDANCE)
        return CollisionAvoidanceOption::WITHOUT_COLLISION_AVOIDANCE;
    else if(value == CollisionAvoidanceOption::COLLISION_AVOIDANCE__INFIELD)
        return CollisionAvoidanceOption::COLLISION_AVOIDANCE__INFIELD;
    else if(value == CollisionAvoidanceOption::COLLISION_AVOIDANCE__OVERALL)
        return CollisionAvoidanceOption::COLLISION_AVOIDANCE__OVERALL;

    throw std::invalid_argument( "The given value does not correspond to any Astar::CollisionAvoidanceOption" );

}

Astar::ISuccesorChecker::IsSuccessorValidParams::IsSuccessorValidParams(const Machine& _machine,
                                                                        double _vt_to_timestamp,
                                                                        double _waiting_time,
                                                                        const Astar::search_property_struct& _current_search_node,
                                                                        const DirectedGraph::vertex_t& _vt_from,
                                                                        const DirectedGraph::vertex_property& _vt_from_prop,
                                                                        const DirectedGraph::vertex_t& _vt_to,
                                                                        const DirectedGraph::vertex_property& _vt_to_prop,
                                                                        const DirectedGraph::vertex_t& _vt_start,
                                                                        const DirectedGraph::vertex_property& _vt_start_prop,
                                                                        const DirectedGraph::vertex_t& _vt_goal,
                                                                        const DirectedGraph::vertex_property& _vt_goal_prop,
                                                                        const DirectedGraph::edge_t& _edge,
                                                                        const DirectedGraph::edge_property& _edge_prop,
                                                                        arolib::Astar::CollisionAvoidanceOption _collisionAvoidanceOption,
                                                                        const DirectedGraph::Graph& _graph):
    machine(_machine),
    target_timestamp(_vt_to_timestamp),
    waiting_time(_waiting_time),
    current_search_node(_current_search_node),
    vt_from(_vt_from),
    vt_from_prop(_vt_from_prop),
    vt_to(_vt_to),
    vt_to_prop(_vt_to_prop),
    vt_start(_vt_start),
    vt_start_prop(_vt_start_prop),
    vt_goal(_vt_goal),
    vt_goal_prop(_vt_goal_prop),
    edge(_edge),
    edge_prop(_edge_prop),
    collisionAvoidanceOption(_collisionAvoidanceOption),
    graph(_graph)

{

}

Astar::ISuccesorChecker::GetMinDurationAtEdgeParams::GetMinDurationAtEdgeParams(const Machine& _machine,
                                                                                double _clearance_time,
                                                                                const Astar::search_property_struct& _current_search_node,
                                                                                const DirectedGraph::vertex_t& _vt_from,
                                                                                const DirectedGraph::vertex_property& _vt_from_prop,
                                                                                const DirectedGraph::vertex_t& _vt_to,
                                                                                const DirectedGraph::vertex_property& _vt_to_prop,
                                                                                const DirectedGraph::vertex_t& _vt_start,
                                                                                const DirectedGraph::vertex_property& _vt_start_prop,
                                                                                const DirectedGraph::vertex_t& _vt_goal,
                                                                                const DirectedGraph::vertex_property& _vt_goal_prop,
                                                                                const DirectedGraph::edge_t& _edge,
                                                                                const DirectedGraph::edge_property& _edge_prop,
                                                                                arolib::Astar::CollisionAvoidanceOption _collisionAvoidanceOption,
                                                                                const DirectedGraph::Graph& _graph):
    machine(_machine),
    clearance_time(_clearance_time),
    current_search_node(_current_search_node),
    vt_from(_vt_from),
    vt_from_prop(_vt_from_prop),
    vt_to(_vt_to),
    vt_to_prop(_vt_to_prop),
    vt_start(_vt_start),
    vt_start_prop(_vt_start_prop),
    vt_goal(_vt_goal),
    vt_goal_prop(_vt_goal_prop),
    edge(_edge),
    edge_prop(_edge_prop),
    collisionAvoidanceOption(_collisionAvoidanceOption),
    graph(_graph)

{

}

Astar::AStarSettings::AStarSettings(double _clearanceTime,
                                    CollisionAvoidanceOption _collisionAvoidanceOption,
                                    bool _includeWaitInCost)
    : clearanceTime(_clearanceTime),
      collisionAvoidanceOption(_collisionAvoidanceOption),
      includeWaitInCost(_includeWaitInCost)
{
}

bool Astar::AStarSettings::parseFromStringMap(Astar::AStarSettings &params, const std::map<std::string, std::string> &map, bool strict)
{
    Astar::AStarSettings tmp;

    int collisionAvoidanceOption;
    std::map<std::string, double*> dMap = { {"clearanceTime" , &tmp.clearanceTime} };
    std::map<std::string, bool*> bMap = { {"includeWaitInCost" , &tmp.includeWaitInCost} };
    std::map<std::string, int*> enumMap = { {"collisionAvoidanceOption" , &collisionAvoidanceOption} };

    if( !setValuesFromStringMap( map, dMap, strict)
            || !setValuesFromStringMap( map, bMap, strict)
            || !setValuesFromStringMap( map, enumMap, strict) )
        return false;

    tmp.collisionAvoidanceOption = intToCollisionAvoidanceOption( collisionAvoidanceOption );

    params = tmp;
    return true;
}

std::map<std::string, std::string> Astar::AStarSettings::parseToStringMap(const Astar::AStarSettings &params)
{
    std::map<std::string, std::string> ret;
    ret["clearanceTime"] = double2string( params.clearanceTime );
    ret["includeWaitInCost"] = std::to_string( params.includeWaitInCost );
    ret["collisionAvoidanceOption"] = std::to_string( params.collisionAvoidanceOption );
    return ret;

}

Astar::ExpansionExclussionReason Astar::intToExpansionExclussionReason(int value)
{
    if(value == ExpansionExclussionReason::IN_OPEN_LIST_AND_BETTER)
        return ExpansionExclussionReason::IN_OPEN_LIST_AND_BETTER;
    else if(value == ExpansionExclussionReason::IN_VISITED_LIST_AND_BETTER)
        return ExpansionExclussionReason::IN_VISITED_LIST_AND_BETTER;
    else if(value == ExpansionExclussionReason::OK)
        return ExpansionExclussionReason::OK;
    else if(value == ExpansionExclussionReason::TENTATIVE_TIME_OVER_MAX_TIME_GOAL)
        return ExpansionExclussionReason::TENTATIVE_TIME_OVER_MAX_TIME_GOAL;
    else if(value == ExpansionExclussionReason::TOWARDS_CURRENT_VERTEX)
        return ExpansionExclussionReason::TOWARDS_CURRENT_VERTEX;
    else if(value == ExpansionExclussionReason::IN_SUCC_EXCLUDE_SET)
        return ExpansionExclussionReason::IN_SUCC_EXCLUDE_SET;
    else if(value == ExpansionExclussionReason::IN_EDGE_EXCLUDE_SET)
        return ExpansionExclussionReason::IN_EDGE_EXCLUDE_SET;
    else if(value == ExpansionExclussionReason::IN_CLOSED_LIST)
        return ExpansionExclussionReason::IN_CLOSED_LIST;
    else if(value == ExpansionExclussionReason::GRAPH_LOCATION)
        return ExpansionExclussionReason::GRAPH_LOCATION;
    else if(value == ExpansionExclussionReason::SUCCESSOR_TIMESTAMP_OVER_MAX_TIME_VISIT)
        return ExpansionExclussionReason::SUCCESSOR_TIMESTAMP_OVER_MAX_TIME_VISIT;
    else if(value == ExpansionExclussionReason::CURRENT_VERTEX_GOT_BUSY)
        return ExpansionExclussionReason::CURRENT_VERTEX_GOT_BUSY;
    else if(value == ExpansionExclussionReason::COST_LIMIT)
        return ExpansionExclussionReason::COST_LIMIT;

    throw std::invalid_argument( "The given value does not correspond to any Astar::ExpansionExclussionReason" );

}

Astar::ExpansionExclussionReason Astar::toExpansionExclussionReason(ISuccesorChecker::Validity value){
    if(value == ISuccesorChecker::VALID)
        return ExpansionExclussionReason::OK;
    if(value == ISuccesorChecker::INVALID__SUCCESSOR_TIMESTAMP)
        return ExpansionExclussionReason::SUCCESSOR_TIMESTAMP_OVER_MAX_TIME_VISIT;
    if(value == ISuccesorChecker::INVALID__GOAL_TIME_LIMIT)
        return ExpansionExclussionReason::TENTATIVE_TIME_OVER_MAX_TIME_GOAL;
    if(value == ISuccesorChecker::INVALID__SUCCESSOR_EXCLUDED)
        return ExpansionExclussionReason::IN_SUCC_EXCLUDE_SET;
    if(value == ISuccesorChecker::INVALID__EDGE_EXCLUDED)
        return ExpansionExclussionReason::IN_EDGE_EXCLUDE_SET;
    if(value == ISuccesorChecker::INVALID__CURRENT_VERTEX_GOT_BUSY)
        return ExpansionExclussionReason::CURRENT_VERTEX_GOT_BUSY;
    if(value == ISuccesorChecker::INVALID__COST_LIMIT)
        return ExpansionExclussionReason::COST_LIMIT;

    return ExpansionExclussionReason::OTHER;
}

Astar::Astar(const Astar::PlanParameters &parameters,
             const Astar::AStarSettings &settings,
             RoutePoint::RoutePointType defaultRPType,
             const std::string &outputFolder,
             std::shared_ptr<Logger> _logger):
    LoggingComponent(_logger, __FUNCTION__),
    m_parameters(parameters),
    m_settings(settings),
    m_outputFolder(outputFolder),
    m_defaultRPType(defaultRPType)
{

    if(!m_outputFolder.empty() && m_outputFolder.back() != '/')
        m_outputFolder += "/";
}

Astar::~Astar(){}


bool Astar::plan(const DirectedGraph::Graph &graph, std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator)
{
    if(!edgeCostCalculator){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid edgeCostCalculator." );
        return false;
    }

    std::multimap<double, Astar::search_property_struct> open_list;
    std::map<DirectedGraph::vertex_t, Astar::search_property_struct> closed_list;
    Astar::search_property_struct current_node;
    Astar::search_property_struct node_property;

    m_result = AstarPlan();
    m_search_id_cnt = 1;
    m_min_g_in_open_list = std::numeric_limits<double>::max();

    //m_settings.clearanceTime = std::max(0.0, clearanceTime);

    DirectedGraph::vertex_property start_prop = graph[m_parameters.start_vt];
    DirectedGraph::vertex_property goal_prop = graph[m_parameters.goal_vt];

    DirectedGraph::vertex_property::GraphLocation goal_location = goal_prop.graph_location;

    /// insert start vertex into open list
    node_property.current_node = m_parameters.start_vt;
    node_property.h = edgeCostCalculator->calcHeuristic(m_parameters.machine, start_prop, goal_prop);
    node_property.time = m_parameters.start_time;
    node_property.waitTime = 0;
    node_property.g = 0.0;
    node_property.search_id = m_search_id_cnt;
    add_to_open_list(open_list, 0, node_property);

    unsigned long start_id = m_search_id_cnt; //search_id of the start node

    Astar::search_property_struct goal_node;

    m_expandNodeCount = 0;

    bool found_plan = false;
    while(!open_list.empty()){
        current_node = remove_top_from_open_list(open_list);// get first vertex from open_list (i.e. vertex with minimum f value)

        if(current_node.current_node == m_parameters.goal_vt){//if the current node is the goal, no need to expand
            goal_node = current_node;
            found_plan = true;
            break;
        }


        // copy current_node to closed_list
        closed_list[current_node.current_node] = current_node;

        // call function expandNode to find valid successors of current_node
        expandNode(graph,
                   current_node,
                   open_list,
                   closed_list,
                   edgeCostCalculator,
                   goal_prop,
                   start_prop,
                   start_id);


//        if( closed_list.find(m_parameters.goal_vt) != closed_list.end() ){
//            break;
//        }

    }


    //save expantion attempts in file (if applicable)
    saveExpansionAttempts(graph);

    if(found_plan)//build the resulting route if a plan was found
        buildPath(graph, closed_list, goal_node, m_parameters.start_vt, start_id);
    else {//@TODO: remove this if the search is moved to another method
        //auto vPropStart = graph[parameters.start_vt];
        //auto vPropGoal = graph[parameters.goal_vt];
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Astar could not find a plan.");
        return false;
    }

    m_result.isOK = true;
    return true;

}

bool Astar::hasPlan() const {
    return m_result.isOK;
}

const AstarPlan &Astar::getPlan() const {
    return m_result;
}

const std::string &Astar::filename_expansionAttempts() {
    return m_filename_expansionAttempts;
}

bool Astar::readFile_expansionAttempts(const std::string &filename,
                                       std::map<DirectedGraph::vertex_t, DirectedGraph::vertex_property> &vertices,
                                       std::map< std::pair<DirectedGraph::vertex_t,DirectedGraph::vertex_t>, DirectedGraph::edge_property> &edges,
                                       Astar::PlanParameters &planParameters,
                                       std::vector<Astar::ExpansionAttemptData> &expansionAttempts,
                                       std::shared_ptr<Logger> _logger)
{
    const std::string sep = ";";

    Logger logger(LogLevel::CRITIC, __FUNCTION__);
    logger.setParent(_logger);

    expansionAttempts.clear();
    vertices.clear();

    std::ifstream file(filename);
    if(!file.is_open()){
        logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error opening file '" + filename + "'");
        return false;
    }

    const std::vector<std::string> params = { "start_vt",
                                              "goal_vt",
                                              "start_time",
                                              "machine_id",
                                              "machine_weight",
                                              "machine_speed",
                                              "initial_olv_bunker_mass",
                                              "includeWaitInCost"};

    std::string line;
    while( std::getline(file, line) && line != "+VERTICES" );
    if(line != "+VERTICES"){
        logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error reading vertices from '" + filename + "'");
        file.close();
        return false;
    }
    while( std::getline(file, line) && line != "-VERTICES" ){
        size_t ref = 0;
        try{
            std::vector<std::string> sValue;
            boost::split(sValue, line, boost::is_any_of(sep));
            DirectedGraph::vertex_property v_prop;
            RoutePoint& rp = v_prop.route_point;
            DirectedGraph::vertex_t vt = std::stoi( sValue.at(ref++) );
            rp.x = string2double( sValue.at(ref++) );
            rp.y = string2double( sValue.at(ref++) );
            rp.time_stamp = string2double( sValue.at(ref++) );
            rp.type = RoutePoint::intToRoutePointType( std::stoi( sValue.at(ref++) ) );
            v_prop.graph_location = DirectedGraph::vertex_property::intToGraphLocation( std::stoi( sValue.at(ref++) ) );
            vertices.insert( vertices.end(), std::make_pair(vt, v_prop) );
        }
        catch(std::exception &e){
            logger.printOut(LogLevel::ERROR, __FUNCTION__,
                              "Error reading vertex '" + std::to_string( vertices.size()+1 )
                             + "' from '" + filename + "': " + e.what());
            file.close();
            return false;
        }

    }

    while( std::getline(file, line) && line != "+EDGES" );
    if(line != "+EDGES"){
        logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error reading edges from '" + filename + "'");
        file.close();
        return false;
    }
    while( std::getline(file, line) && line != "-EDGES" ){
        size_t ref = 0;
        try{
            std::vector<std::string> sValue;
            boost::split(sValue, line, boost::is_any_of(sep));
            DirectedGraph::edge_property prop;
            DirectedGraph::vertex_t vt0 = std::stoi( sValue.at(ref++) );
            DirectedGraph::vertex_t vt1 = std::stoi( sValue.at(ref++) );
            prop.edge_type = DirectedGraph::intToEdgeType( std::stoi( sValue.at(ref++) ) );
            prop.defWidth = string2double( sValue.at(ref++) );
            prop.distance = string2double( sValue.at(ref++) );
            prop.overruns.resize( std::stoi( sValue.at(ref++) ) );
            edges.insert( edges.end(), std::make_pair( std::make_pair(vt0, vt1) , prop) );
        }
        catch(std::exception &e){
            logger.printOut(LogLevel::ERROR, __FUNCTION__,
                              "Error reading edge '" + std::to_string( edges.size()+1 )
                             + "' from '" + filename + "': " + e.what());
            file.close();
            return false;
        }

    }

    while( std::getline(file, line) && line != "+PARAMETERS" );
    if(line != "+PARAMETERS"){
        logger.printOut(LogLevel::ERROR, __FUNCTION__,
                          "Error reading plan parameters from '" + filename + "'");
        file.close();
        return false;
    }
    for(size_t i = 0 ; i < params.size() ; ++i){
        if( !std::getline(file, line) ){
            logger.printOut(LogLevel::ERROR, __FUNCTION__,
                              "Error reading plan parameter '" + params.at(i) + "' from '" + filename + "'");
            file.close();
            return false;
        }

        try{
            std::vector<std::string> sValue;
            boost::split(sValue, line, boost::is_any_of(":"));
            if( ( params.at(i) != "exclude" && sValue.size() != 2 )
                    || ( params.at(i) == "exclude" && sValue.size() != 1 && sValue.size() != 2 )
                    || sValue.front() != params.at(i)){
                logger.printOut(LogLevel::ERROR, __FUNCTION__,
                                  "Error reading plan parameter '" + params.at(i) + "' from '" + filename + "'");
                file.close();
                return false;
            }
            if(params.at(i) == "start_vt")
                planParameters.start_vt = std::stoi(sValue.back());
            else if(params.at(i) == "goal_vt")
                planParameters.goal_vt = std::stoi(sValue.back());
            else if(params.at(i) == "start_time")
                planParameters.start_time = string2double(sValue.back());
            else if(params.at(i) == "machine_id")
                planParameters.machine.id = std::stoi(sValue.back());
            else if(params.at(i) == "machine_weight")
                planParameters.machine.weight = string2double(sValue.back());
            else if(params.at(i) == "machine_speed")
                planParameters.machine_speed = string2double(sValue.back());
            else if(params.at(i) == "initial_olv_bunker_mass")
                planParameters.initial_bunker_mass = string2double(sValue.back());
            else if(params.at(i) == "includeWaitInCost")
                planParameters.includeWaitInCost = std::stoi(sValue.back());
        }
        catch(std::exception &e){
            logger.printOut(LogLevel::ERROR, __FUNCTION__,
                              "Error reading plan parameter '" + params.at(i) + "' from '" + filename + "': " + e.what());
            file.close();
            return false;
        }
    }

    while( std::getline(file, line) && line != "+ATTEMPTS" );
    if(line != "+ATTEMPTS"){
        logger.printOut(LogLevel::ERROR, __FUNCTION__,
                          "Error reading plan attempts from '" + filename + "'");
        file.close();
        return false;
    }
    while( std::getline(file, line) && line != "-ATTEMPTS" ){
        expansionAttempts.emplace_back( ExpansionAttemptData() );
        auto& ead = expansionAttempts.back();
        std::vector<std::string> sValue;
        boost::split(sValue, line, boost::is_any_of(sep));
        if( sValue.size() != 13 ){
            logger.printOut(LogLevel::ERROR, __FUNCTION__,
                              "Error reading expansion attempt data '{"
                              + std::to_string(expansionAttempts.size())
                             + "}' from '" + filename + "' (invalid size)");
            file.close();
            return false;
        }
        size_t count_easd = 0;
        size_t ref = 0;
        try{
            ead.count = std::stoi( sValue.at(ref++) );
            ead.current_vt_data.node.current_node = std::stoi( sValue.at(ref++) );
            ead.current_vt_data.node.predecessor = std::stoll( sValue.at(ref++) );
            ead.current_vt_data.node.time = string2double( sValue.at(ref++) );
            ead.current_vt_data.node.g = string2double( sValue.at(ref++) );
            ead.current_vt_data.node.h = string2double( sValue.at(ref++) );
            ead.current_vt_data.node.totalCost = string2double( sValue.at(ref++) );
            ead.current_vt_data.node.numCrossings = std::stoi( sValue.at(ref++) );
            ead.current_vt_data.node.numCrossings_HL = std::stoi( sValue.at(ref++) );
            ead.current_vt_data.node.waitTime = string2double( sValue.at(ref++) );
            ead.current_vt_data.point.x = string2double( sValue.at(ref++) );
            ead.current_vt_data.point.y = string2double( sValue.at(ref++) );
            count_easd = std::stoi( sValue.at(ref++) );
        }
        catch(std::exception &e){
            logger.printOut(LogLevel::ERROR, __FUNCTION__,
                              "Error reading expansion attempt data '{"
                              + std::to_string(expansionAttempts.size()) + "-"
                              + std::to_string(ref) + "}' from '" + filename + "': " + e.what());
            file.close();
            return false;
        }

        ead.successor_vts_data.resize(count_easd);
        for(size_t i = 0 ; i < count_easd ; ++i){
            auto& easd = ead.successor_vts_data.at(i);
            if(!std::getline(file, line)){
                logger.printOut(LogLevel::ERROR, __FUNCTION__,
                                 "Error reading expansion attempt successors-data '{"
                                  + std::to_string(expansionAttempts.size()) + "-"
                                  + std::to_string(i) + "}' from '" + filename + "'");
                file.close();
                return false;
            }
            boost::split(sValue, line, boost::is_any_of(sep));
            if( sValue.size() != 10 ){
                logger.printOut(LogLevel::ERROR, __FUNCTION__,
                                  "Error reading expansion attempt successors-data '{"
                                  + std::to_string(expansionAttempts.size()) + "-"
                                  + std::to_string(i) + "}' from '" + filename + "' (invalid size)");
                file.close();
                return false;
            }
            try{
                ref = 0;
                easd.vt = std::stoi( sValue.at(ref++) );
                easd.point.x = string2double( sValue.at(ref++) );
                easd.point.y = string2double( sValue.at(ref++) );
                easd.exclussionReason = intToExpansionExclussionReason( std::stoi( sValue.at(ref++) ) );
                easd.tentative_g = string2double( sValue.at(ref++) );
                easd.tentative_h = string2double( sValue.at(ref++) );
                easd.calc_time = string2double( sValue.at(ref++) );
                easd.min_time = string2double( sValue.at(ref++) );
                easd.busy_time = string2double( sValue.at(ref++) );
                easd.tentative_time = string2double( sValue.at(ref++) );
            }
            catch(std::exception &e){
                logger.printOut(LogLevel::ERROR, __FUNCTION__,
                                  "Error reading expansion attempt successors-data '{"
                                  + std::to_string(expansionAttempts.size()) + "-"
                                  + std::to_string(i) + "}' from '" + filename + "': " + e.what());
                file.close();
                return false;
            }
        }
    }
    file.close();
    return true;

}

RoutePoint Astar::buildRoutePoint(const DirectedGraph::vertex_property &vp, double time) {
    RoutePoint ret;
    ret.type = m_defaultRPType;
    if(isOfSpecialType(vp.route_point))
       ret.type =  vp.route_point.type;

    ret.point() = vp.route_point.point();
    if(vp.route_point.type == RoutePoint::RoutePointType::HEADLAND)
        ret.track_id = -1;
    else
        ret.track_id = vp.route_point.track_id;
    ret.time_stamp = time;
    ret.bunker_mass = m_parameters.initial_bunker_mass;
    return ret;
}

bool Astar::buildPath(const DirectedGraph::Graph &graph,
                      const std::map<DirectedGraph::vertex_t, search_property_struct> &closed_list,
                      const Astar::search_property_struct &_current_node,
                      const DirectedGraph::vertex_t &start_vertex,
                      unsigned long start_id){
    m_result.route_points_.clear();
    m_result.edge_overruns_.clear();
    m_result.visit_periods.clear();

    m_result.waitTime = _current_node.waitTime;
    m_result.plan_cost_total = _current_node.totalCost;
    m_result.numCrossings = _current_node.numCrossings;
    m_result.numCrossings_HL = _current_node.numCrossings_HL;

    Astar::search_property_struct current_node = _current_node;
    std::vector<DirectedGraph::vertex_t> rp_nodes;

    m_result.visited_vertices.clear();
    m_result.visited_vertices.clear();

    DirectedGraph::vertex_property current_v_prop = graph[current_node.current_node];

    m_result.route_points_.push_back(buildRoutePoint(current_v_prop, current_node.time));
    rp_nodes.push_back(current_node.current_node);
    m_result.visited_vertices.push_back(current_node.current_node);

    double prevWaitingTime = current_node.waitTime;

    double plan_cost = current_node.g;

    DirectedGraph::overroll_property overrun;
    overrun.machine_id = m_parameters.machine.id;
    overrun.weight = m_parameters.machine.weight + m_parameters.initial_bunker_mass;

    while (current_node.current_node != start_vertex){

        DirectedGraph::edge_t edge = current_node.edge;
        m_result.visited_edges.push_back(edge);

        auto it_cl = closed_list.find(current_node.predecessor);
        if(it_cl == closed_list.end()){
            logger().printError(__FUNCTION__, "Predecessor node " + std::to_string(current_node.predecessor) + " not found in closed list");
            return false;
        }

        current_node = it_cl->second;
        m_result.visited_vertices.push_back(current_node.current_node);

//        while (current_node.search_id != start_id) {
//               for (int i = 0; i < closed_list.size(); ++i) {
//                   if (current_node.predecessor_search_id == closed_list.at(i).search_id) {
//                       current_node = closed_list.at(i);
//                       break;
//                   }
//               }

        current_v_prop = graph[current_node.current_node];

        double waitingTime = prevWaitingTime - current_node.waitTime;
        prevWaitingTime = current_node.waitTime;

        auto rp = buildRoutePoint(current_v_prop, current_node.time);
        bool addRoutePoint = true;
        if (!m_result.route_points_.empty() && isSameRoutePoint(rp, m_result.route_points_.back()) ){
            bool isLastSpecial = isOfSpecialType(m_result.route_points_.back());
            bool isNewSpecial = isOfSpecialType(rp);
            addRoutePoint = false;

            if( isNewSpecial && !isLastSpecial ){//replace
                m_result.route_points_.back() = rp;
            }
            else if( isLastSpecial ){//leave both?
                addRoutePoint = true;
            }

        }
        if(addRoutePoint){
            if(waitingTime > 1e-2){
                bool addExtraPoint = true;
                if(rp.type != RoutePoint::RESOURCE_POINT || rp.type == RoutePoint::INITIAL_POSITION){
                    if(m_result.route_points_.empty())
                        addExtraPoint = false;
                    else{
                        auto rpPrev = m_result.route_points_.back();
                        addExtraPoint = !rpPrev.isFieldAccess();
                    }
                }
                if(addExtraPoint){
                    auto rpTmp = rp;
                    rpTmp.time_stamp += waitingTime;
                    rpTmp.type = m_defaultRPType;
                    m_result.route_points_.push_back(rpTmp);
                    rp_nodes.push_back(current_node.current_node);
                }
            }
            m_result.route_points_.push_back(rp);
            rp_nodes.push_back(current_node.current_node);
            if ( m_result.route_points_.size() > 1) {
                overrun.duration = m_result.route_points_.back().time_stamp - r_at(m_result.route_points_, 1).time_stamp;
                m_result.edge_overruns_.push_back(std::make_pair(edge, overrun));
            }
        }


    }

    std::reverse(m_result.route_points_.begin(), m_result.route_points_.end());
    std::reverse(rp_nodes.begin(), rp_nodes.end());
    std::reverse(m_result.edge_overruns_.begin(), m_result.edge_overruns_.end());
    std::reverse(m_result.visited_vertices.begin(), m_result.visited_vertices.end());
    std::reverse(m_result.visited_edges.begin(), m_result.visited_edges.end());


    double deltaTimeDef = std::max(0.0, m_settings.clearanceTime);
    for(size_t i = 0 ; i < m_result.route_points_.size() ; ++i){
        auto &rp = m_result.route_points_.at(i);
        if(rp.type == RoutePoint::RESOURCE_POINT
                ||rp.type == RoutePoint::INITIAL_POSITION )
            continue;

        DirectedGraph::VisitPeriod vp(m_parameters.machine.id,
                                      rp.time_stamp - deltaTimeDef,
                                      rp.time_stamp + deltaTimeDef,
                                      rp.time_stamp);

        if(i+1 < m_result.route_points_.size()){
            const auto& rp2 = m_result.route_points_.at(i+1);
            if( !rp.isFieldAccess() || !rp2.isFieldAccess() )//do not add next_vt if it is transit between access points
                vp.next_vt.emplace_back( std::make_pair( rp_nodes.at(i+1) , rp2 ) );
        }


        if(rp.type != m_parameters.machine_speed > 0){

            if( i > 0
                    && m_result.route_points_.at(i-1).type != RoutePoint::RESOURCE_POINT  )
                vp.time_in  = rp.time_stamp - 0.5 * std::max ( 0.5 * deltaTimeDef,
                                                               arolib::geometry::calc_dist( rp, m_result.route_points_.at(i-1) ) / m_parameters.machine_speed );
            if( i+1 < m_result.route_points_.size()
                    && m_result.route_points_.at(i+1).type != RoutePoint::RESOURCE_POINT )
                vp.time_out = rp.time_stamp + 0.5 * std::max ( 0.5 * deltaTimeDef,
                                                               arolib::geometry::calc_dist( rp, m_result.route_points_.at(i+1) ) / m_parameters.machine_speed );

        }
        m_result.visit_periods.emplace_back( std::make_pair( rp_nodes.at(i) , vp ) );
    }

    m_result.plan_cost_ = plan_cost;
    return true;
}

bool Astar::isSameRoutePoint(const RoutePoint &rp1, const RoutePoint &rp2) const
{
    return rp1.point() == rp2.point()
            && std::fabs(rp1.time_stamp - rp2.time_stamp) < 1e-6
            && std::fabs(rp1.bunker_mass - rp2.bunker_mass) < 1e-6
            && std::fabs(rp1.bunker_volume - rp2.bunker_volume) < 1e-6;
}

bool Astar::isOfSpecialType(const RoutePoint &route_point){
    return route_point.isOfType({ RoutePoint::RESOURCE_POINT, RoutePoint::FIELD_ENTRY, RoutePoint::FIELD_EXIT, RoutePoint::INITIAL_POSITION });
}

void Astar::expandNode(const DirectedGraph::Graph &graph,
                       const search_property_struct &current_node,
                       std::multimap<double, search_property_struct> &open_list,
                       std::map<DirectedGraph::vertex_t, search_property_struct> &closed_list,
                       std::shared_ptr<IEdgeCostCalculator> edgeCostCalculator,
                       const DirectedGraph::vertex_property &goal_prop,
                       const DirectedGraph::vertex_property &start_prop,
                       unsigned long start_id)
{
    m_expandNodeCount++;
    std::pair<DirectedGraph::out_edge_iterator, DirectedGraph::out_edge_iterator> out_edges;
    Astar::search_property_struct node_property;
    DirectedGraph::vertex_t successor;

    DirectedGraph::vertex_property current_prop = graph[current_node.current_node];
    DirectedGraph::vertex_property::GraphLocation goal_location = goal_prop.graph_location;

    ExpansionAttemptData ead_tmp;
    ExpansionAttemptData* p_ead = &ead_tmp;

    if(!m_outputFolder.empty()){
        m_expansionAttempts.emplace_back( ExpansionAttemptData());
        p_ead = &m_expansionAttempts.back();
    }

    ExpansionAttemptData& ead = *p_ead;
    ead.count = m_expandNodeCount;
    ead.current_vt_data.node = current_node;
    ead.current_vt_data.point = current_prop.route_point.point();

    out_edges= boost::out_edges(current_node.current_node, graph);
    for(;out_edges.first != out_edges.second; out_edges.first++){

        successor = target(*out_edges.first, graph);

        DirectedGraph::vertex_property successor_prop = graph[successor];

        ead.successor_vts_data.push_back(ExpansionAttemptSuccessorData());
        ExpansionAttemptSuccessorData &easd = ead.successor_vts_data.back();
        easd.vt = successor;
        easd.point = successor_prop.route_point.point();
        easd.exclussionReason = ExpansionExclussionReason::OK;

        if(goal_location != DirectedGraph::vertex_property::GraphLocation::DEFAULT &&
               successor_prop.graph_location != DirectedGraph::vertex_property::GraphLocation::DEFAULT &&
                goal_location != successor_prop.graph_location ){
            easd.exclussionReason = ExpansionExclussionReason::GRAPH_LOCATION;
            continue;
        }

        if(Track::isHeadlandTrack(successor_prop.route_point.track_id)
                && ( successor_prop.route_point.type == RoutePoint::TRACK_START || successor_prop.route_point.type == RoutePoint::TRACK_END )){

            graph.getVerticesInRadius( successor_prop.route_point, 1e-1,
                                       [&successor_prop](const DirectedGraph::vertex_t&, const DirectedGraph::vertex_property& vertex_prop)->bool{
                if(vertex_prop.route_point.track_id != successor_prop.route_point.track_id
                        || ( vertex_prop.route_point.type != RoutePoint::TRACK_START &&  vertex_prop.route_point.type != RoutePoint::TRACK_END) )
                    return false;
                successor_prop.route_point.time_stamp = std::min(successor_prop.route_point.time_stamp, vertex_prop.route_point.time_stamp);
                return true;
            });
        }

        if(closed_list.find(successor) != closed_list.end()){
            easd.exclussionReason = ExpansionExclussionReason::IN_CLOSED_LIST;
            continue;
        }

        const DirectedGraph::edge_t edge = *out_edges.first;
        DirectedGraph::edge_property edge_prop = graph[edge];

        double clearanceTime = m_settings.clearanceTime;
        if(m_parameters.machine.id == successor_prop.harvester_id)
            clearanceTime = 0;

        double min_edge_time = 0;//minimum time that transversing the edge can take

        for(auto checker: m_parameters.successorCheckers){
            if(!checker)
                continue;
            double minDuration = checker->getMinDurationAtEdge(ISuccesorChecker::GetMinDurationAtEdgeParams(m_parameters.machine,
                                                                                                            clearanceTime,
                                                                                                            current_node,
                                                                                                            current_node.current_node, current_prop, successor, successor_prop,
                                                                                                            m_parameters.start_vt, start_prop, m_parameters.goal_vt, goal_prop,
                                                                                                            edge, edge_prop,
                                                                                                            m_settings.collisionAvoidanceOption,
                                                                                                            graph));
            if(minDuration > 0.0)
                min_edge_time = std::max(min_edge_time, minDuration);
        }

        double edge_start_time = current_node.time;
        //double edge_start_time = std::max(successor_prop.route_point.time_stamp, current_node.time);  /// earliest time when the olv can enter the edge

        double distance = edge_prop.distance;
        if(distance < -1e-3){//it is machine dependant
            distance = arolib::geometry::calc_dist(successor_prop.route_point, current_prop.route_point);
        }

        double calc_edge_time;//time that the machine would take to transverse the edge if harvesting times, other machines in the field, etc. are not taken into account
        if(edge_prop.edge_type == DirectedGraph::FAP_TO_RP
                || edge_prop.edge_type == DirectedGraph::RP_TO_FAP
                || edge_prop.edge_type == DirectedGraph::FAP_TO_FAP){//get the data data from the out-of-field information
            OutFieldInfo::MachineBunkerState bunkerState = OutFieldInfo::MACHINE_LOADED;
            if(m_parameters.initial_bunker_mass - std::max(0.0, m_parameters.machine.weight) < 1e-9 + 0.4*std::max(0.0, m_parameters.machine.bunker_mass) )
                bunkerState = OutFieldInfo::MACHINE_EMPTY;
            OutFieldInfo::TravelCosts tc;
            if( OutFieldInfo::getTravelCost( edge_prop.travelCosts, m_parameters.machine.id, bunkerState, tc ) ){
                calc_edge_time = tc.time;
                if(tc.distance > -1e-9)
                    distance = tc.distance;
            }
            else{
                calc_edge_time = distance / m_parameters.machine_speed;
            }
        }
        else if(edge_prop.edge_type == DirectedGraph::INIT)
            calc_edge_time = edge_prop.arrivalCosts.time;
        else
            calc_edge_time = distance / m_parameters.machine_speed;

        double tentative_time = edge_start_time + std::max(calc_edge_time, min_edge_time);// the timestamp the the route point would have
        tentative_time = std::max(tentative_time, 1e-3);//just in case it is zero

        bool incWaitInCost = m_parameters.includeWaitInCost;

        //update expansion-attempt info
        easd.calc_time = calc_edge_time;
        easd.min_time = min_edge_time;
        easd.busy_time = 0;

        if(m_settings.collisionAvoidanceOption != CollisionAvoidanceOption::WITHOUT_COLLISION_AVOIDANCE){//if collision avoidance
            //check if the node is occupied, and if it is, get the time that the machine would have to wait for it to be free and update the corresponding times (min_edge_time, tentative_time)

            //set the maximum number of machines allowed in the node depending on the nodes's location and route-point type
            size_t maxMachinesInNode = 1;//unless it is a special case (e.g. headland point during infield harvesting), only the node has space only for one machine
            if( successor_prop.graph_location == DirectedGraph::vertex_property::INFIELD
                    && successor_prop.route_point.type == RoutePoint::HEADLAND){
                if(m_settings.collisionAvoidanceOption == CollisionAvoidanceOption::COLLISION_AVOIDANCE__OVERALL)
                    maxMachinesInNode = 1;
                else
                    maxMachinesInNode = -1;//if no collition voidance
            }

            double busyWaitingTime;
            bool towardsCurrentVt;
            std::vector<MachineId_t> busyMachines;

            double visitPeriodCheck_startTime = edge_start_time + calc_edge_time; //tentative_time;
            double visitPeriodCheck_duration = clearanceTime + tentative_time - edge_start_time - calc_edge_time; //clearanceTime;  //we cannot know how much time will the olv spend here without knowing the next node :/

            if( DirectedGraph::VisitPeriod::isBusy( successor_prop.visitPeriods,
                                                    visitPeriodCheck_startTime,
                                                    visitPeriodCheck_duration,
                                                    maxMachinesInNode,
                                                    busyWaitingTime,
                                                    current_node.current_node,
                                                    towardsCurrentVt,
                                                    busyMachines,
                                                    {m_parameters.machine.id}) ){
                if(towardsCurrentVt){
                    easd.exclussionReason = ExpansionExclussionReason::TOWARDS_CURRENT_VERTEX;
                    continue;
                }


                incWaitInCost |= ( busyWaitingTime > clearanceTime );//include the wait time in cost if the node is busy with a machine other that the harvester

                busyWaitingTime += clearanceTime*0.5;

//                min_edge_time += busyWaitingTime;
//                tentative_time += busyWaitingTime;
//                easd.busy_time = busyWaitingTime;


                min_edge_time = std::max(min_edge_time, calc_edge_time + busyWaitingTime);
                tentative_time = edge_start_time + std::max(calc_edge_time, min_edge_time);

                easd.busy_time = busyWaitingTime;
            }
        }

        easd.tentative_time = tentative_time;

        double edge_time = std::max(calc_edge_time, min_edge_time);
        double edge_wait_time = edge_time - calc_edge_time;

        for(auto checker: m_parameters.successorCheckers){
            if(!checker)
                continue;
            auto val = checker->isSuccessorValid(ISuccesorChecker::IsSuccessorValidParams(m_parameters.machine,
                                                                               tentative_time,
                                                                               edge_wait_time,
                                                                               current_node,
                                                                               current_node.current_node, current_prop, successor, successor_prop,
                                                                               m_parameters.start_vt, start_prop, m_parameters.goal_vt, goal_prop,
                                                                               edge, edge_prop,
                                                                               m_settings.collisionAvoidanceOption,
                                                                               graph));
            if(val != ISuccesorChecker::VALID){
                easd.exclussionReason = toExpansionExclussionReason(val);
                break;
            }
        }
        if(easd.exclussionReason != ExpansionExclussionReason::OK)
            continue;


        double edge_cost = edgeCostCalculator->calcCost(m_parameters.machine,
                                                        edge,
                                                        edge_prop,
                                                        edge_time,
                                                        m_parameters.includeWaitInCost ? edge_wait_time : 0,
                                                        m_parameters.initial_bunker_mass);//edge cost used for the search
        double edge_cost_total = edgeCostCalculator->calcCost(m_parameters.machine,
                                                              edge,
                                                              edge_prop,
                                                              edge_time,
                                                              edge_wait_time,
                                                              m_parameters.initial_bunker_mass);//edge cost used for the analytics
        node_property.h = edgeCostCalculator->calcHeuristic(m_parameters.machine, successor_prop, goal_prop);


        double tentative_g = current_node.g + edge_cost;

        node_property.current_node = successor;
        node_property.predecessor = current_node.current_node;
        node_property.g = tentative_g;
        node_property.totalCost = current_node.totalCost + edge_cost_total;
        node_property.time = tentative_time;
        node_property.search_id = ++m_search_id_cnt;
        node_property.predecessor_search_id = current_node.search_id;
        node_property.edge = *out_edges.first;

        node_property.numCrossings = current_node.numCrossings;
        node_property.numCrossings_HL = current_node.numCrossings_HL;
        if(edge_prop.edge_type == DirectedGraph::EdgeType::CROSS || edge_prop.edge_type == DirectedGraph::EdgeType::CROSS_HL || edge_prop.edge_type == DirectedGraph::EdgeType::BOUNDARY_CONN){
            if(Track::isHeadlandTrack(successor_prop.route_point.track_id))
                ++node_property.numCrossings_HL;
            else
                ++node_property.numCrossings;
        }

        if(min_edge_time > calc_edge_time)
            node_property.waitTime = current_node.waitTime + (min_edge_time-calc_edge_time);
        else
            node_property.waitTime = current_node.waitTime;

        m_openListCandidates[successor].push_back( std::tuple<unsigned int, DirectedGraph::vertex_t, double, double>
                                                   (m_expandNodeCount,
                                                    current_node.current_node,
                                                    node_property.g,
                                                    node_property.h) );

        easd.tentative_g = node_property.g;
        easd.tentative_h = node_property.h;


        if(m_parameters.max_cost >= 0 && m_parameters.max_cost < node_property.g + node_property.h){
            easd.exclussionReason = ExpansionExclussionReason::COST_LIMIT;
            continue;
        }

        bool in_open_list = false;
        bool in_open_list_and_better = false;
        auto it_ol = open_list.begin();
        for( ; it_ol != open_list.end() ; it_ol++){
            if( it_ol->second.current_node == successor ){
                if(tentative_g > it_ol->second.g)
                    in_open_list_and_better = true;
                else if(tentative_g == it_ol->second.g && tentative_time > it_ol->second.time)
                    in_open_list_and_better = true;
                in_open_list = true;
                break;
//                if(successor == (*it).second.current_node
//                  && tentative_g >=(*it).second.g
//                  && tentative_time >= (*it).second.time)
            }
        }
        if(in_open_list_and_better){//in open list and better
            easd.exclussionReason = ExpansionExclussionReason::IN_OPEN_LIST_AND_BETTER;
            continue;
        }
        if(in_open_list)
            open_list.erase(it_ol);
        add_to_open_list(open_list, node_property.g + node_property.h, node_property);

    }

}

void Astar::add_to_open_list(std::multimap<double, Astar::search_property_struct> &open_list,
                             double f,
                             const Astar::search_property_struct &node_property)
{
    open_list.insert(std::make_pair(f, node_property));
    update_min_g_in_open_list(open_list,
                              std::numeric_limits<double>::max(),
                              node_property.g);
}

Astar::search_property_struct Astar::remove_top_from_open_list(std::multimap<double, Astar::search_property_struct> &open_list)
{
    Astar::search_property_struct top_node = open_list.begin()->second;
    open_list.erase(open_list.begin());
    update_min_g_in_open_list(open_list,
                              top_node.g,
                              std::numeric_limits<double>::max());
    return top_node;
}

void Astar::update_min_g_in_open_list(const std::multimap<double, Astar::search_property_struct> &open_list,
                                      const double &removed_g,
                                      const double &new_g)
{
    if(removed_g <= m_min_g_in_open_list+0.000001)
        update_min_g_in_open_list(open_list);
    if(m_min_g_in_open_list > new_g)
        m_min_g_in_open_list = new_g;
}

void Astar::update_min_g_in_open_list(const std::multimap<double, search_property_struct> &open_list)
{
    m_min_g_in_open_list = std::numeric_limits<double>::max();
    for(const auto &it : open_list)
        m_min_g_in_open_list = std::min(m_min_g_in_open_list, it.second.g);
}

void Astar::saveExpansionAttempts(const DirectedGraph::Graph &graph)
{
    if(m_outputFolder.empty())
        return;

    if(!io::create_directory(m_outputFolder))
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error opening/creating output folder " + m_outputFolder);

    std::string filename = m_outputFolder + m_filename_expansionAttempts;

    try{
        std::ofstream of(filename);
        if(!of.is_open()){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error opening output file " + filename);
            return;
        }

        const std::string sep = ";";

        of << "+VERTICES" << std::endl;
        for(DirectedGraph::vertex_iter vp = vertices(graph); vp.first != vp.second; vp.first++){
            const DirectedGraph::vertex_property& v_prop = graph[*vp.first];
            const RoutePoint &rp = v_prop.route_point;
            of  << std::setprecision(10)
                << *vp.first << sep
                << rp.x << sep
                << rp.y << sep
                << rp.time_stamp << sep
                << rp.type << sep
                << v_prop.graph_location << sep
                << std::endl;
        }
        of << "-VERTICES" << std::endl;

        of << "+EDGES" << std::endl;
        for(DirectedGraph::edge_iter ed = edges(graph); ed.first != ed.second; ed.first++){
            const DirectedGraph::edge_property& prop = graph[*ed.first];
            auto vt1 = source(*ed.first, graph);
            auto vt2 = target(*ed.first, graph);
            of  << vt1 << sep
                << vt2 << sep
                << prop.edge_type << sep
                << prop.defWidth << sep
                << prop.distance << sep
                << prop.overruns.size() << sep
                << std::endl;
        }
        of << "-EDGES" << std::endl;

        of << "+PARAMETERS" << std::endl;
        of << "start_vt:" << m_parameters.start_vt << std::endl;
        of << "goal_vt:" << m_parameters.goal_vt << std::endl;
        of << "start_time:" << m_parameters.start_time << std::endl;
        of << "machine_id:" << m_parameters.machine.id << std::endl;
        of << "machine_weight:" << m_parameters.machine.weight << std::endl;
        of << "machine_speed:" << m_parameters.machine_speed << std::endl;
        of << "initial_olv_bunker_mass:" << m_parameters.initial_bunker_mass << std::endl;
        of << "includeWaitInCost:" << m_parameters.includeWaitInCost << std::endl;
        of << "-PARAMETERS" << std::endl;

        of << "+ATTEMPTS" << std::endl;
        for(const auto& ead : m_expansionAttempts){
            of << std::setprecision(10)
               << ead.count << sep
               << (int) ead.current_vt_data.node.current_node << sep
               << (int) ead.current_vt_data.node.predecessor << sep
               << ead.current_vt_data.node.time << sep
               << ead.current_vt_data.node.g << sep
               << ead.current_vt_data.node.h << sep
               << ead.current_vt_data.node.totalCost << sep
               << ead.current_vt_data.node.numCrossings << sep
               << ead.current_vt_data.node.numCrossings_HL << sep
               << ead.current_vt_data.node.waitTime << sep
               << ead.current_vt_data.point.x << sep
               << ead.current_vt_data.point.y << sep
               << ead.successor_vts_data.size()
               << std::endl;
            for(const auto& easd : ead.successor_vts_data){
                of << std::setprecision(10)
                   << easd.vt << sep
                   << easd.point.x << sep
                   << easd.point.y << sep
                   << easd.exclussionReason << sep
                   << easd.tentative_g << sep
                   << easd.tentative_h << sep
                   << easd.calc_time << sep
                   << easd.min_time << sep
                   << easd.busy_time << sep
                   << easd.tentative_time
                   << std::endl;
            }
        }
        of << "-ATTEMPTS" << std::endl;

        of.close();

    }
    catch(std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error saving in output file '" + filename + "': " + e.what());
    }


}

void Astar::writeToDebugFile(const DirectedGraph::Graph &graph,
                             const Astar::search_property_struct &current,
                             const DirectedGraph::vertex_t &start,
                             const DirectedGraph::vertex_t &goal,
                             const std::multimap<double, Astar::search_property_struct> &open_list,
                             const std::map<DirectedGraph::vertex_t, Astar::search_property_struct> &closed_list)
{
    std::ofstream ofStart("start.dat");
    ofStart << std::setprecision(12);
    ofStart << graph[start].route_point.x << " "
            << graph[start].route_point.y << std::endl;
    ofStart.close();

    std::ofstream ofGoal("goal.dat");
    ofGoal << std::setprecision(12);
    ofGoal << graph[goal].route_point.x << " "
           << graph[goal].route_point.y << std::endl;
    ofGoal.close();

    std::ofstream ofCurrent("current.dat");
    ofCurrent << std::setprecision(12);
    ofCurrent << graph[current.current_node].route_point.x << " "
                                                                 << graph[current.current_node].route_point.y << std::endl;
    ofCurrent.close();

    std::ofstream ofOpenList("openList.dat");
    ofOpenList << std::setprecision(12);
    for (auto &node : open_list) {
        ofOpenList << graph[node.second.current_node].route_point.x << " "
                                                                          << graph[node.second.current_node].route_point.y << " "
                                                                          << node.first << std::endl;
    }
    ofOpenList.close();

    std::ofstream ofClosedList("closedList.dat");
    ofClosedList << std::setprecision(12);
    for (auto &node : closed_list) {
        ofClosedList << graph[node.second.current_node].route_point.x << " "
                                                                     << graph[node.second.current_node].route_point.y << " "
                                                                     << node.second.g+node.second.h << " "
                                                                     << node.second.g << " "
                                                                     << node.second.h << " " << std::endl;
    }
    ofClosedList.close();
}



}
