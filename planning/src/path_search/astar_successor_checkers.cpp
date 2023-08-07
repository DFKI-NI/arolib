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
 
#include "arolib/planning/path_search/astar_successor_checkers.hpp"


namespace arolib{

AstarCustomSuccessorChecker::AstarCustomSuccessorChecker(const CallbackFunct1Type &cb_isSuccessorValid, const CallbackFunct2Type &cb_getMinDurationAtEdge):
    m_cb_isSuccessorValid(cb_isSuccessorValid),
    m_cb_getMinDurationAtEdge(cb_getMinDurationAtEdge)
{

}

Astar::ISuccesorChecker::Validity AstarCustomSuccessorChecker::isSuccessorValid(const Astar::ISuccesorChecker::IsSuccessorValidParams &params) const
{
    return m_cb_isSuccessorValid(params);
}

double AstarCustomSuccessorChecker::getMinDurationAtEdge(const GetMinDurationAtEdgeParams &params) const
{
    return m_cb_getMinDurationAtEdge(params);
}

AstarSuccessorChecker_SuccessorTimestamp::AstarSuccessorChecker_SuccessorTimestamp(double timestamp, CheckOption checkOption, const std::set<MachineId_t> &restrictedMachineIds):
    m_timestamp(timestamp), m_checkOption(checkOption), m_restrictedMachineIds(restrictedMachineIds)
{

}

Astar::ISuccesorChecker::Validity AstarSuccessorChecker_SuccessorTimestamp::isSuccessorValid(const Astar::ISuccesorChecker::IsSuccessorValidParams& params) const
{
    const float eps = 1e-6;
    if( m_timestamp > -eps
            && params.vt_to_prop.route_point.time_stamp > -eps
            && ( m_restrictedMachineIds.find(params.vt_to_prop.harvester_id) != m_restrictedMachineIds.end()
                 || m_restrictedMachineIds.find(Machine::AllMachineIds) != m_restrictedMachineIds.end() )
            /*&& vt_to != vt_goal*/){
        if(m_checkOption == INVALID_IF_LOWER_THAN_SUCC_TIMESPAMP
                && params.vt_to_prop.route_point.time_stamp - eps > m_timestamp)
            return Astar::ISuccesorChecker::INVALID__SUCCESSOR_TIMESTAMP;
        if(m_checkOption == INVALID_IF_HIGHER_THAN_SUCC_TIMESPAMP
                && params.vt_to_prop.route_point.time_stamp + eps < m_timestamp)
            return Astar::ISuccesorChecker::INVALID__SUCCESSOR_TIMESTAMP;
    }
    return Astar::ISuccesorChecker::VALID;

}

double AstarSuccessorChecker_SuccessorTimestamp::getMinDurationAtEdge(const GetMinDurationAtEdgeParams &params) const
{
    if(m_checkOption == INVALID_IF_LOWER_THAN_SUCC_TIMESPAMP
            && params.vt_to_prop.route_point.time_stamp > -1e-3){
        double clearanceTime = std::max(0.0, params.clearance_time);
        return std::max(0.0, params.vt_to_prop.route_point.time_stamp)
                + clearanceTime
                - std::max(0.0, params.current_search_node.time);//minimum time that transversing the edge can take based on whether the route point was worked already (and when) or not (disregarding distance)
    }

    return -1;
}

AstarSuccessorChecker_GoalMaxTime::AstarSuccessorChecker_GoalMaxTime(float max_time_goal):
    m_max_time_goal(max_time_goal)
{

}

Astar::ISuccesorChecker::Validity AstarSuccessorChecker_GoalMaxTime::isSuccessorValid(const Astar::ISuccesorChecker::IsSuccessorValidParams &params) const
{
    if ( m_max_time_goal >= 0 && params.target_timestamp > m_max_time_goal)
            return Astar::ISuccesorChecker::INVALID__GOAL_TIME_LIMIT;
    return Astar::ISuccesorChecker::VALID;
}


AstarSuccessorChecker_VertexExcludeSet::AstarSuccessorChecker_VertexExcludeSet(const std::set<DirectedGraph::vertex_t> &excludeSet):
    m_excludeSet(excludeSet),
    m_hasExceptions(false),
    m_except ([](const Astar::ISuccesorChecker::IsSuccessorValidParams&){return false;})
{

}

AstarSuccessorChecker_VertexExcludeSet::AstarSuccessorChecker_VertexExcludeSet(const std::set<DirectedGraph::vertex_t> &excludeSet, const ExceptFct &exceptIf):
    m_excludeSet(excludeSet),
    m_hasExceptions(true),
    m_except(exceptIf)
{

}

Astar::ISuccesorChecker::Validity AstarSuccessorChecker_VertexExcludeSet::isSuccessorValid(const IsSuccessorValidParams &params) const
{
    if (m_excludeSet.find(params.vt_to) == m_excludeSet.end())
        return Astar::ISuccesorChecker::VALID;

    if(m_hasExceptions && m_except(params))
        return Astar::ISuccesorChecker::VALID;

    return Astar::ISuccesorChecker::INVALID__SUCCESSOR_EXCLUDED;
}


AstarSuccessorChecker_VertexExcludeSet_Exceptions1::AstarSuccessorChecker_VertexExcludeSet_Exceptions1(const std::set<DirectedGraph::vertex_t> &excludeSet,
                                                                                                       bool exceptIfVtFromIsInitPoint,
                                                                                                       bool exceptIfVtFromIsAccessPoint,
                                                                                                       bool exceptIfVtFromIsResourcePoint):
    AstarSuccessorChecker_VertexExcludeSet(excludeSet, std::bind(&AstarSuccessorChecker_VertexExcludeSet_Exceptions1::except,
                                                                 std::placeholders::_1,
                                                                 exceptIfVtFromIsInitPoint,
                                                                 exceptIfVtFromIsAccessPoint,
                                                                 exceptIfVtFromIsResourcePoint))
{

}

bool AstarSuccessorChecker_VertexExcludeSet_Exceptions1::except(const IsSuccessorValidParams &params, bool exceptIfVtFromIsInitPoint, bool exceptIfVtFromIsAccessPoint, bool exceptIfVtFromIsResourcePoint)
{
    if(params.vt_to_prop.route_point.isOfType({RoutePoint::RESOURCE_POINT, RoutePoint::FIELD_ENTRY, RoutePoint::FIELD_EXIT}))
        return false;

    if(params.vt_to_prop.graph_location != DirectedGraph::vertex_property::HEADLAND
            || params.vt_to_prop.route_point.type != RoutePoint::HEADLAND)
        return false;

    //temporary workaround: dont exclude it if it is being accessed from a init vertex
    if(params.vt_from_prop.route_point.type == RoutePoint::INITIAL_POSITION)
        return exceptIfVtFromIsInitPoint;

    //temporary workaround: dont exclude it if it is being accessed from a FAP: this is needed for the planning of the headland, where it can happen that the FAP connection to the boundary is exacly the excluded vertex neighbor from the next route point after the overload start
    if(exceptIfVtFromIsAccessPoint){
        for(auto &it : params.graph.accesspoint_vertex_map()){
            if(it.second == params.vt_from)
                return true;
        }
    }

    //temporary workaround: dont exclude it if it is being accessed from a resource point: this is needed for the planning of the headland, where it can happen that the resource point connection to the boundary is exacly the excluded vertex neighbor from the next route point after the overload start
    if(exceptIfVtFromIsResourcePoint){
        for(auto &it : params.graph.resourcepoint_vertex_map()){
            if(it.second == params.vt_from)
                return true;
        }
    }

    return false;

}


AstarSuccessorChecker_EdgeExcludeSet::AstarSuccessorChecker_EdgeExcludeSet(const std::set<DirectedGraph::edge_t> &excludeSet):
    m_excludeSet(excludeSet), m_hasExceptions(false), m_except ([](const Astar::ISuccesorChecker::IsSuccessorValidParams&){return false;})
{

}

AstarSuccessorChecker_EdgeExcludeSet::AstarSuccessorChecker_EdgeExcludeSet(const std::set<DirectedGraph::edge_t> &excludeSet, const ExceptFct &exceptIf):
    m_excludeSet(excludeSet), m_hasExceptions(true), m_except(exceptIf)
{

}

Astar::ISuccesorChecker::Validity AstarSuccessorChecker_EdgeExcludeSet::isSuccessorValid(const IsSuccessorValidParams &params) const
{
    if (m_excludeSet.find(params.edge) == m_excludeSet.end())
        return Astar::ISuccesorChecker::VALID;

    if(m_hasExceptions && m_except(params))
        return Astar::ISuccesorChecker::VALID;

    return Astar::ISuccesorChecker::INVALID__EDGE_EXCLUDED;
}

AstarSuccessorChecker_FutureVisits::AstarSuccessorChecker_FutureVisits(const std::map<MachineId_t, double> &restrictedMachineIds):
    m_restrictedMachineIds(restrictedMachineIds)
{

}

Astar::ISuccesorChecker::Validity AstarSuccessorChecker_FutureVisits::isSuccessorValid(const Astar::ISuccesorChecker::IsSuccessorValidParams &params) const
{

    if(params.collisionAvoidanceOption == Astar::CollisionAvoidanceOption::WITHOUT_COLLISION_AVOIDANCE //only if collision avoidance
            || params.vt_from == params.vt_start //do not check if vt_from is the start vertex
            || params.vt_from_prop.route_point.type == RoutePoint::RESOURCE_POINT //do not check if vt_from is corresponds to a resource point
            || params.vt_from_prop.route_point.isFieldAccess()) //do not check if vt_from is corresponds to an access point
        return Astar::ISuccesorChecker::VALID;


    for(auto &it_rm : m_restrictedMachineIds){
        if(it_rm.first == params.machine.id)
            continue;

        //check future visit periods for the current node
        if( DirectedGraph::VisitPeriod::isMachineVisiting(params.vt_from_prop.visitPeriods,
                                                          it_rm.first,
                                                          params.current_search_node.time+1e-5,
                                                          it_rm.second) ){

            //check future visit periods for the succesor
            if( DirectedGraph::VisitPeriod::isMachineVisiting(params.vt_to_prop.visitPeriods,
                                                              it_rm.first,
                                                              params.target_timestamp+1e-5,
                                                              it_rm.second) )
                return Astar::ISuccesorChecker::INVALID__CURRENT_VERTEX_GOT_BUSY;
        }
    }
    return Astar::ISuccesorChecker::VALID;
}

AstarSuccessorChecker_FutureVisits2::AstarSuccessorChecker_FutureVisits2()
{

}

Astar::ISuccesorChecker::Validity AstarSuccessorChecker_FutureVisits2::isSuccessorValid(const IsSuccessorValidParams &params) const
{
    if(params.collisionAvoidanceOption == Astar::CollisionAvoidanceOption::WITHOUT_COLLISION_AVOIDANCE //only if collision avoidance
            || params.vt_from == params.vt_start //do not check if vt_from is the start vertex
            || params.vt_from_prop.route_point.type == RoutePoint::RESOURCE_POINT //do not check if vt_from is corresponds to a resource point
            || params.vt_from_prop.route_point.isFieldAccess()) //do not check if vt_from is corresponds to an access point
        return Astar::ISuccesorChecker::VALID;



    //check if the time the machine needs to remain in the current nod to achieve this successor generates a collition with other machines already planned to visit the current node (i.e. check if the node is still free)

    //set the maximum number of machines allowed in the node depending on the nodes's location and route-point type
    size_t maxMachinesInNode = 1;//unless it is a special case (e.g. headland point during infield harvesting), only the node has space only for one machine
//    if( params.vt_from_prop.graph_location == DirectedGraph::vertex_property::INFIELD
//            && params.vt_from_prop.route_point.type == RoutePoint::HEADLAND){
//        if(params.collisionAvoidanceOption == Astar::CollisionAvoidanceOption::COLLISION_AVOIDANCE__OVERALL)
//            maxMachinesInNode = params.graph.numHeadlandTracks();//the headland track (for infield harvesting) is in the middle of the headland has space for one ore more machines (assume the number of tracks is the number of machines that fit in the headland)
//        else
//            maxMachinesInNode = -1;//if no collition avoidance
//    }

    double busyWaitingTime;
    bool towardsCurrentVt;
    std::vector<MachineId_t> busyMachines;
    double dTime = params.target_timestamp - params.current_search_node.time;
    //if (min_edge_time < 0)//if the machine does not have restrictions regarding harvesting time, use the the timestamp of the middle point
    //    dTime *= 0.5;

    if( DirectedGraph::VisitPeriod::isBusy( params.vt_from_prop.visitPeriods,
                                            params.current_search_node.time,
                                            dTime,
                                            maxMachinesInNode,
                                            busyWaitingTime,
                                            params.current_search_node.predecessor,
                                            towardsCurrentVt,
                                            busyMachines,
                                            {params.machine.id}) )
        return Astar::ISuccesorChecker::INVALID__CURRENT_VERTEX_GOT_BUSY;

    return Astar::ISuccesorChecker::VALID;
}

}
