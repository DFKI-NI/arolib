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
 

#include "arolib/planning/track_connectors/infieldtracksconnectorgraphbased.hpp"

namespace arolib{

namespace ag = arolib::geometry;


void InfieldTracksConnectorGraphBased::VertexFunctionParams::fromSuccessorCheckerParams(const Astar::ISuccesorChecker::IsSuccessorValidParams &scParams)
{
    vt = scParams.vt_to;
    vt_prop = scParams.vt_to_prop;
    vt_start = scParams.vt_start;
    vt_start_prop = scParams.vt_start_prop;
    vt_goal = scParams.vt_goal;
    vt_goal_prop = scParams.vt_goal_prop;

    vt_start_valid = true;
    vt_goal_valid = true;
}

void InfieldTracksConnectorGraphBased::EdgeFunctionParams::fromSuccessorCheckerParams(const Astar::ISuccesorChecker::IsSuccessorValidParams &scParams)
{
    edge = scParams.edge;
    edge_prop = scParams.edge_prop;
    vt_from = scParams.vt_from;
    vt_from_prop = scParams.vt_from_prop;
    vt_to = scParams.vt_to;
    vt_to_prop = scParams.vt_to_prop;
    vt_start = scParams.vt_start;
    vt_start_prop = scParams.vt_start_prop;
    vt_goal = scParams.vt_goal;
    vt_goal_prop = scParams.vt_goal_prop;

}

InfieldTracksConnectorGraphBased::InfieldTracksConnectorGraphBased(std::shared_ptr<Logger> parentLogger)
    : IInfieldTracksConnector(__FUNCTION__, parentLogger)
{
    m_excludeVertexFunc = [](...){return false;};
    m_excludeEdgeFunc = [](...){return false;};
    m_allowVertexFunc = [](...){return false;};
    m_allowEdgeFunc = [](...){return false;};

    setCostCalculator(nullptr);
    setViaMaxEdges(0);
    setMaxCountCloseVts(-1);
}

std::vector<Point> InfieldTracksConnectorGraphBased::getConnection(const Machine &machine,
                                                                   const Pose2D &_pose_start,
                                                                   const Pose2D &_pose_end,
                                                                   double turningRad,
                                                                   const std::pair<double, double> &extraDist,
                                                                   const Polygon &limitBoundary,
                                                                   const Polygon &infieldBoundary,
                                                                   const Headlands &headlands,
                                                                   double speed_start,
                                                                   double maxConnectionLength) const
{

    std::vector<Point> ret;

    //adjust start/end poses
    Pose2D pose_start, pose_end;
    getExtendedPoses(_pose_start, _pose_end, extraDist, pose_start, pose_end);

    if(!checkForMinPossibleLength(pose_start, pose_end, turningRad, limitBoundary, infieldBoundary, maxConnectionLength))
        return ret;

    VertexFunctionParams vtParams;
    vtParams.vt_start_valid = vtParams.vt_goal_valid = false;

    std::vector<DirectedGraph::vertex_t> vts0, vtsn;

    if(turningRad > 1e-9){
        std::map<DirectedGraph::vertex_t, double> distMap;
        vts0 = m_graph.getVerticesInRadius(pose_start, turningRad,
                                           [this, &infieldBoundary, &pose_start, &vtParams, &distMap]
                                           (const DirectedGraph::vertex_t& vt, const DirectedGraph::vertex_property& vertex_prop)->bool{
                                               vtParams.vt = vt;
                                               vtParams.vt_prop = vertex_prop;
                                               if (!isStartVtValid(vtParams, pose_start, infieldBoundary))
                                                   return false;
                                               distMap[vt] = geometry::calc_dist(pose_start, vertex_prop.route_point);
                                               return true;
                                           });
        if(m_maxCountCloseVts < vts0.size())//sort by distance to the pose
            std::sort(vts0.begin(), vts0.end(), [&distMap](const DirectedGraph::vertex_t& a, const DirectedGraph::vertex_t& b)->bool{
                return distMap[a] < distMap[b];
            });
    }
    if(vts0.empty()){
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Could not find a valid starting vertex corresponding to pose_start within the given turning radius. Seaching closest...");
        vts0 = getClosestValidVertices(m_graph, pose_start,
                                       [this, &infieldBoundary, &pose_start, &vtParams]
                                       (const DirectedGraph::vertex_t& vt, const DirectedGraph::vertex_property& vertex_prop)->bool{
                                            vtParams.vt = vt;
                                            vtParams.vt_prop = vertex_prop;
                                            return isStartVtValid(vtParams, pose_start, infieldBoundary);
                                        },
                                        1);
    }

    if(vts0.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Could not find a valid starting vertex corresponding to pose_start");
        return ret;
    }

    if(turningRad > 1e-9){
        std::map<DirectedGraph::vertex_t, double> distMap;
        vtsn = m_graph.getVerticesInRadius(pose_end, turningRad,
                                           [this, &infieldBoundary, &pose_end, &vtParams, &distMap]
                                           (const DirectedGraph::vertex_t& vt, const DirectedGraph::vertex_property& vertex_prop)->bool{
                                               vtParams.vt = vt;
                                               vtParams.vt_prop = vertex_prop;
                                               if (!isEndVtValid(vtParams, pose_end, infieldBoundary))
                                                   return false;
                                               distMap[vt] = geometry::calc_dist(pose_end, vertex_prop.route_point);
                                               return true;
                                           });
        if(m_maxCountCloseVts < vtsn.size())//sort by distance to the pose
            std::sort(vtsn.begin(), vtsn.end(), [&distMap](const DirectedGraph::vertex_t& a, const DirectedGraph::vertex_t& b)->bool{
                return distMap[a] < distMap[b];
            });
    }
    if(vtsn.empty()){
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Could not find a valid goal vertex corresponding to pose_end within the given turning radius. Seaching closest...");
        vtsn = getClosestValidVertices(m_graph, pose_end,
                                       [this, &infieldBoundary, &pose_end, &vtParams]
                                       (const DirectedGraph::vertex_t& vt, const DirectedGraph::vertex_property& vertex_prop)->bool{
                                            vtParams.vt = vt;
                                            vtParams.vt_prop = vertex_prop;
                                            return isEndVtValid(vtParams, pose_end, infieldBoundary);
                                        },
                                        1);
    }
    if(vtsn.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Could not find a valid goal vertex corresponding to pose_end");
        return ret;
    }



    if(m_maxCountCloseVts < vts0.size()){//remove farest vertices
        vts0.erase(vts0.begin()+m_maxCountCloseVts, vts0.end());
    }

    if(m_maxCountCloseVts < vtsn.size()){//remove farest vertices
        vtsn.erase(vtsn.begin()+m_maxCountCloseVts, vtsn.end());
    }


    Astar::PlanParameters astarParamsBase;
    astarParamsBase.start_time = 0.0;
    astarParamsBase.machine = machine;
    astarParamsBase.machine_speed = machine.calcSpeed(0);
    astarParamsBase.initial_bunker_mass = 0;
    astarParamsBase.includeWaitInCost = false;

    Astar::AStarSettings astarSettings;
    astarSettings.clearanceTime = 0;
    astarSettings.collisionAvoidanceOption = Astar::CollisionAvoidanceOption::WITHOUT_COLLISION_AVOIDANCE;
    astarSettings.includeWaitInCost = false;

    double minCost = std::numeric_limits<double>::max();

    std::vector<Point> initPath, finishPath;

    m_costCalculator->m_boundary = &limitBoundary;
    m_costCalculator->m_turningRad = turningRad;

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Planning connection with " + std::to_string(vts0.size()) + " potential start-vertices and " + std::to_string(vtsn.size()) + " potential end-vertices");

//    getConnectionSingleThread(machine, pose_start, pose_end, infieldBoundary, maxConnectionLength,
//                              vts0, vtsn,
//                              astarParamsBase, astarSettings,
//                              minCost, initPath, finishPath);

    getConnectionMultiThread(machine, pose_start, pose_end, infieldBoundary, maxConnectionLength,
                             vts0, vtsn,
                             astarParamsBase, astarSettings,
                             minCost, initPath, finishPath);


    if(!initPath.empty()){
        ret.reserve(initPath.size() + finishPath.size() + 4);

        if( ag::calc_dist(pose_start, _pose_start) > 1e-3 )
            ret.push_back(_pose_start.point());
        if( ag::calc_dist(initPath.front(), pose_start) > 1e-3 )
            ret.push_back(pose_start.point());

        ret.insert(ret.end(), initPath.begin(), initPath.end());

        if(!finishPath.empty())
            ret.insert(ret.end(), finishPath.begin()+1, finishPath.end());

        if( ag::calc_dist(ret.back(), pose_end) > 1e-3 )
            ret.push_back(pose_end.point());
        if( ag::calc_dist(ret.back(), _pose_end) > 1e-3 )
            ret.push_back(_pose_end.point());
    }

    ag::remove_repeated_points(ret);

    return ret;
}

Polygon InfieldTracksConnectorGraphBased::getExtendedLimitBoundary(const Subfield &sf, double turningRad) const
{
    Polygon ret;
    auto boundary = sf.boundary_outer;
    ag::correct_polygon(ret);
    if(ag::isPolygonValid(boundary) == ag::PolygonValidity::INVALID_POLYGON)
        return ret;

    if(!ag::offsetPolygon(boundary, ret, 0.01, true, 0))
        ret = boundary;
    return ret;
}

void InfieldTracksConnectorGraphBased::setGraph(const DirectedGraph::Graph &graph)
{
    m_graph = graph;
}

void InfieldTracksConnectorGraphBased::setCostCalculator(std::shared_ptr<IEdgeCostCalculator> costCalculator)
{
    if(costCalculator)
        m_costCalculator->m_baseCostCalculator = costCalculator;
    else{//default

        auto& baseCostCalculator = m_costCalculator->m_baseCostCalculator;
        baseCostCalculator = std::make_shared<ECC_distanceOptimization>();
        baseCostCalculator->logger().setParent(m_costCalculator->loggerPtr());

        IEdgeCostCalculator::GeneralParameters params;
        params.headlandCrossCostMult = 0;
        baseCostCalculator->setGeneralParameters(params);
    }
}

void InfieldTracksConnectorGraphBased::setExcludeFunctions(const ExcludeVertexFunc &vtFunc, const ExcludeEdgeFunc &edgeFunc)
{
    m_excludeVertexFunc = vtFunc;
    m_excludeEdgeFunc = edgeFunc;
}

void InfieldTracksConnectorGraphBased::setAllowFunctions(const AllowVertexFunc &vtFunc, const AllowEdgeFunc &edgeFunc)
{
    m_allowVertexFunc = vtFunc;
    m_allowEdgeFunc = edgeFunc;
}

void InfieldTracksConnectorGraphBased::setViaMaxEdges(int val)
{
    if(val > 0)
        m_viaMaxEdges = val;
    else
        m_viaMaxEdges = 3;
}

void InfieldTracksConnectorGraphBased::setMaxCountCloseVts(int val)
{
    if(val > 0)
        m_maxCountCloseVts = val;
    else if (val < 0)
        m_maxCountCloseVts = 3;
    else
        m_maxCountCloseVts = std::numeric_limits<size_t>::max();
}

void InfieldTracksConnectorGraphBased::setOutputSearchFolder(const std::string &outputFolder)
{
    m_outputFolder = outputFolder;
    if(!m_outputFolder.empty() && m_outputFolder.back() != '/')
        m_outputFolder += "/";
}

bool InfieldTracksConnectorGraphBased::checkForMinPossibleLength(const Pose2D &pose_start, const Pose2D &pose_end, double turningRad, const Polygon &limitBoundary, const Polygon &infieldBoundary, double maxConnectionLength) const
{
    return IInfieldTracksConnector::checkForMinPossibleLength( pose_start,
                                                               pose_end,
                                                               -1,
                                                               limitBoundary,
                                                               infieldBoundary,
                                                               maxConnectionLength );
}

void InfieldTracksConnectorGraphBased::getConnectionSingleThread(const Machine& machine,
                                                                 const Pose2D& pose_start,
                                                                 const Pose2D& pose_end,
                                                                 const Polygon& infieldBoundary,
                                                                 double maxConnectionLength,
                                                                 const std::vector<DirectedGraph::vertex_t> &vts0,
                                                                 const std::vector<DirectedGraph::vertex_t> &vtsn,
                                                                 const Astar::PlanParameters &astarParamsBase,
                                                                 const Astar::AStarSettings &astarSettings,
                                                                 double &minCost,
                                                                 std::vector<Point> &initPath, std::vector<Point> &finishPath) const
{
    auto astarParams = astarParamsBase;

    size_t countStartVts = 0;
    for(auto vt_start : vts0)
    {
        if(countStartVts > m_maxCountCloseVts)
            break;

        auto via_vts = getReachableVertices(m_graph, vt_start, m_viaMaxEdges);
        if(via_vts.empty())
            continue;

        ++countStartVts;

        double costStart = 0;

        DirectedGraph::vertex_property vt_start_prop = m_graph[vt_start];
        double dist_start = geometry::calc_dist(pose_start, vt_start_prop.route_point);
        if( dist_start > 1e-3 ){
            double speed = machine.calcSpeed( astarParamsBase.initial_bunker_mass );
            double dTime = speed > 0 ? dist_start / speed : 0;
            costStart += m_costCalculator->calcCost(machine, pose_start, vt_start_prop.route_point, dTime, 0, astarParams.initial_bunker_mass, {});
        }

        size_t countEndVts = 0;
        for(auto vt_end : vtsn){
            if(countEndVts > m_maxCountCloseVts)
                break;

            ++countEndVts;

            astarParams.successorCheckers.clear();

            //sets to hold invalid vts and edges already excluded in previous attempts
            std::set<DirectedGraph::vertex_t> invalidVts;
            std::set<DirectedGraph::edge_t> invalidEdges;

            std::vector<DirectedGraph::vertex_t> prevVtsInit;

            std::shared_ptr<const Astar::ISuccesorChecker> successorChecker = std::make_shared<AstarCustomSuccessorChecker>(
                        [this, &infieldBoundary, &invalidVts, &invalidEdges, &vt_start, &vt_end, &pose_start, &pose_end, &prevVtsInit]
                        (const Astar::ISuccesorChecker::IsSuccessorValidParams& params)->Astar::ISuccesorChecker::Validity
            {
                if(invalidVts.find(params.vt_to) != invalidVts.end())
                    return Astar::ISuccesorChecker::INVALID__SUCCESSOR_EXCLUDED;

                if(invalidEdges.find(params.edge) != invalidEdges.end())
                    return Astar::ISuccesorChecker::INVALID__EDGE_EXCLUDED;

                VertexFunctionParams vtParams;
                vtParams.fromSuccessorCheckerParams(params);

                if(!isVertexValid(vtParams, infieldBoundary, false)){
                    invalidVts.insert(params.vt_to);
                    return Astar::ISuccesorChecker::INVALID__SUCCESSOR_EXCLUDED;
                }

                EdgeFunctionParams edgeParams;
                edgeParams.fromSuccessorCheckerParams(params);

                if(!isEdgeValid(edgeParams, vt_start, vt_end, pose_start, pose_end, prevVtsInit)){
                    invalidEdges.insert(params.edge);
                    return Astar::ISuccesorChecker::INVALID__EDGE_EXCLUDED;
                }

                return Astar::ISuccesorChecker::VALID;

            } );

            double costEnd = 0;

            DirectedGraph::vertex_property vt_end_prop = m_graph[vt_end];
            double dist_end = geometry::calc_dist(pose_end, vt_end_prop.route_point);
            if( dist_end > 1e-3 ){
                double speed = machine.calcSpeed( astarParams.initial_bunker_mass );
                double dTime = speed > 0 ? dist_end / speed : 0;
                costEnd += m_costCalculator->calcCost(machine, pose_end, vt_end_prop.route_point, dTime, 0, astarParams.initial_bunker_mass, {});
            }

            astarParams.successorCheckers.push_back( successorChecker );

            VertexFunctionParams vtParams;
            vtParams.vt_start = vt_start;
            vtParams.vt_start_prop = m_graph[vt_start];
            vtParams.vt_goal = vt_end;
            vtParams.vt_goal_prop = m_graph[vt_end];
            vtParams.vt_start_valid = vtParams.vt_goal_valid = true;
            for(auto& via_vt : via_vts){

                if(via_vt != vt_end){
                    vtParams.vt = via_vt;
                    vtParams.vt_prop = m_graph[via_vt];
                    vtParams.vt_start_valid = vtParams.vt_goal_valid = true;
                    if(!isVertexValid(vtParams, infieldBoundary, false))
                        continue;
                }

                prevVtsInit.clear();
                invalidVts.clear();
                invalidEdges.clear();

                astarParams.start_vt = vt_start;
                astarParams.goal_vt = via_vt;

                Astar astarInit(astarParams,
                                astarSettings,
                                RoutePoint::TRANSIT,
                                getOutputSearchFolder(vt_start, vt_end, via_vt, true),
                                loggerPtr());

                if( !astarInit.plan(m_graph, m_costCalculator) || astarInit.getPlan().route_points_.size() < 2 ){
                    logger().printOut(LogLevel::ERROR, __FUNCTION__, "AStar init search failed ["
                                      + std::to_string(astarParams.start_vt) + "->" + std::to_string(astarParams.goal_vt) + "] ");
                    continue;
                }

                double cost = costStart + costEnd + astarInit.getPlan().plan_cost_total;

                if(cost > minCost)
                    continue;

                double pathLength;

                if(maxConnectionLength > 1e-9){
                    pathLength = ag::getGeometryLength(astarInit.getPlan().route_points_);
                    if(pathLength > maxConnectionLength)
                        continue;
                }

                if(via_vt == vt_end)
                    finishPath.clear();
                else{
                    prevVtsInit = m_graph.getVerticesInRadius( r_at(astarInit.getPlan().route_points_, 1), 1e-3 );
                    invalidVts.clear();
                    invalidEdges.clear();

                    astarParams.start_vt = via_vt;
                    astarParams.goal_vt = vt_end;

                    Astar astar(astarParams,
                                astarSettings,
                                RoutePoint::TRANSIT,
                                getOutputSearchFolder(vt_start, vt_end, via_vt, false),
                                loggerPtr());

                    if( !astar.plan(m_graph, m_costCalculator) || astar.getPlan().route_points_.empty() ){
                        logger().printOut(LogLevel::ERROR, __FUNCTION__, "AStar search failed ["
                                          + std::to_string(astarParams.start_vt) + "->" + std::to_string(astarParams.goal_vt) + "] ");
                        continue;
                    }

                    cost += astar.getPlan().plan_cost_total;

                    if(cost > minCost)
                        continue;

                    if(maxConnectionLength > 1e-9){
                        pathLength += ag::getGeometryLength(astarInit.getPlan().route_points_);
                        if(pathLength > maxConnectionLength)
                            continue;
                    }
                    finishPath = Point::toPoints( astar.getPlan().route_points_ );
                }

                logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Obtained new connection ["
                                  + std::to_string(vt_start) + "->" + std::to_string(vt_end) + "] "
                                  + " via vertex " + std::to_string(via_vt)
                                  + ": cost " + std::to_string(minCost) + " -> " + std::to_string(cost));

                minCost = cost;

                initPath = Point::toPoints( astarInit.getPlan().route_points_ );
            }
        }
    }

}



void InfieldTracksConnectorGraphBased::getConnectionMultiThread(const Machine& machine,
                                                                const Pose2D& pose_start,
                                                                const Pose2D& pose_end,
                                                                const Polygon& infieldBoundary,
                                                                double maxConnectionLength,
                                                                const std::vector<DirectedGraph::vertex_t> &vts0,
                                                                const std::vector<DirectedGraph::vertex_t> &vtsn,
                                                                const Astar::PlanParameters &astarParamsBase,
                                                                const Astar::AStarSettings &astarSettings,
                                                                double &minCost,
                                                                std::vector<Point> &initPath, std::vector<Point> &finishPath) const
{


    std::vector< std::future< void > > futures_start( vts0.size() );
    std::mutex mutex;

    for(size_t i_s = 0 ; i_s < futures_start.size() ; ++i_s){
        futures_start.at(i_s) = getFutureStart(mutex,
                                               machine, infieldBoundary, maxConnectionLength,
                                               pose_start, pose_end,
                                               vts0.at(i_s), vtsn,
                                               astarParamsBase, astarSettings,
                                               minCost, initPath, finishPath);


    }

    for(auto& fu_s : futures_start)
        fu_s.wait();

}

bool InfieldTracksConnectorGraphBased::isVertexValid(const VertexFunctionParams &params,
                                                     const Polygon &infieldBoundary,
                                                     bool allowInitLocationVts) const
{
    const auto& rp = params.vt_prop.route_point;

    if( infieldBoundary.points.size() > 3 && ag::in_polygon(rp, infieldBoundary) )
        return m_allowVertexFunc(params) && isVertexValid(params, Polygon(), allowInitLocationVts);

    if(rp.type == RoutePoint::RESOURCE_POINT)
        return false;
    if(rp.type == RoutePoint::INITIAL_POSITION)
        return allowInitLocationVts && !m_excludeVertexFunc(params);
    return !m_excludeVertexFunc(params);
}

bool InfieldTracksConnectorGraphBased::isStartVtValid(const VertexFunctionParams &params,
                                                      const Pose2D &pose_start,
                                                      const Polygon &infieldBoundary) const
{
    const auto& rp = params.vt_prop.route_point;

    if(ag::calc_dist(pose_start, rp) < 1e-3)
        return true;

    Point ptNext = ag::getPointAtDist(pose_start, 1);

    if( std::fabs( ag::get_angle(ptNext, pose_start, rp, true) ) > 100 ) //do not drive in reverse
        return false;

    return isVertexValid(params, infieldBoundary, true);
}

bool InfieldTracksConnectorGraphBased::isEndVtValid(const VertexFunctionParams &params, const Pose2D &pose_end, const Polygon &infieldBoundary) const
{
    const auto& rp = params.vt_prop.route_point;
    if(ag::calc_dist(pose_end, rp) < 1e-3)
        return true;

    Point ptNext = ag::getPointAtDist(pose_end, 1);

    if( std::fabs( ag::get_angle(ptNext, pose_end, rp, true) ) < 80 ) //do not drive in reverse
        return false;

    return isVertexValid(params, infieldBoundary, false);

}

bool InfieldTracksConnectorGraphBased::isEdgeValid(const EdgeFunctionParams &params,
                                                   const DirectedGraph::vertex_t &vt_start, const DirectedGraph::vertex_t &vt_end,
                                                   const Pose2D &pose_start, const Pose2D &pose_end,
                                                   const std::vector<DirectedGraph::vertex_t>& prev_vts_init) const
{
    //check for reverse driving at the start, end, and via locations
    if(params.vt_from == params.vt_start && params.vt_start != vt_start){//if the search start_vt is a via_vt, do not drive towards the vertex from which you reached the via_vt
        for(auto& vt : prev_vts_init){
            if(vt == params.vt_to)
                return false;
        }
    }
    if(params.vt_from == vt_start && params.vt_start == vt_start){//if the search start_vt is the overall start_vt, check the pose_start angle and the riving direction to reach from pose_start
        if(ag::calc_dist(params.vt_from_prop.route_point, pose_start) < 1e-3){//vt_start is at the pose_start
            Point ptNext = ag::getPointAtDist(pose_start, 1);
            if( std::fabs( ag::get_angle(ptNext, pose_start, params.vt_to_prop.route_point, true) ) > 100 ) //do not drive in reverse
                return false;
        }
        else{
            if( std::fabs( ag::get_angle(pose_start, params.vt_from_prop.route_point, params.vt_to_prop.route_point, true) ) < 30 ) //do not drive in reverse
                return false;
        }
    }
    if(params.vt_to == vt_end){//if the search goal_vt is the overall end_vt, check the pose_end angle and the driving direction to reach pose_end
        if(ag::calc_dist(params.vt_to_prop.route_point, pose_end) < 1e-3){//vt_end is at the pose_end
            Point ptNext = ag::getPointAtDist(pose_end, 1);
            if( std::fabs( ag::get_angle(ptNext, pose_end, params.vt_from_prop.route_point, true) ) < 80 ) //do not drive in reverse
                return false;
        }
        else{
            if( std::fabs( ag::get_angle(params.vt_from_prop.route_point, params.vt_to_prop.route_point, pose_end, true) ) < 30 ) //do not drive in reverse
                return false;
        }
    }

    switch(params.edge_prop.edge_type){

    case DirectedGraph::RP_TO_FAP:
    case DirectedGraph::FAP_TO_RP:
        return false;
    case DirectedGraph::FAP_TO_FAP:
        return m_allowEdgeFunc(params);
    default:
        return !m_excludeEdgeFunc(params);

    }

}

std::string InfieldTracksConnectorGraphBased::getOutputSearchFolder(const DirectedGraph::vertex_t &vt_start, const DirectedGraph::vertex_t &vt_end, const DirectedGraph::vertex_t &vt_via, bool isInit) const
{
    if(m_outputFolder.empty())
        return "";

    std::string folder = m_outputFolder + "P_" + std::to_string(vt_start) + "_to_" + std::to_string(vt_end) + "_via_" + std::to_string(vt_via);
    if(isInit)
        folder += "_init/";
    else
        folder += "_finish/";

    if(!io::create_directory(folder, true))
        return "";
    return folder;
}

std::future<void> InfieldTracksConnectorGraphBased::getFutureStart(std::mutex &mutex,
                                                                   const Machine &machine, const Polygon &infieldBoundary, const double& maxConnectionLength,
                                                                   const Pose2D &pose_start, const Pose2D &pose_end,
                                                                   const DirectedGraph::vertex_t &vt_start,
                                                                   const std::vector<DirectedGraph::vertex_t> &vtsn,
                                                                   const Astar::PlanParameters &astarParamsBase,
                                                                   const Astar::AStarSettings &astarSettings,
                                                                   double &minCost,
                                                                   std::vector<Point> &initPath, std::vector<Point> &finishPath) const
{

    return std::async( std::launch::async,
                       [this,
                       &mutex,
                       &machine, &infieldBoundary, &maxConnectionLength,
                       &pose_start, &pose_end,
                       vt_start, &vtsn,
                       &astarParamsBase, &astarSettings,
                       &minCost, &initPath, &finishPath](){

        auto via_vts = getReachableVertices(m_graph, vt_start, m_viaMaxEdges);
        if(via_vts.empty())
            return;


        double costStart = 0;

        DirectedGraph::vertex_property vt_start_prop = m_graph[vt_start];
        double dist_start = geometry::calc_dist(pose_start, vt_start_prop.route_point);
        if( dist_start > 1e-3 ){
            double speed = machine.calcSpeed( astarParamsBase.initial_bunker_mass );
            double dTime = speed > 0 ? dist_start / speed : 0;
            costStart += m_costCalculator->calcCost(machine, pose_start, vt_start_prop.route_point, dTime, 0, astarParamsBase.initial_bunker_mass, {});
        }

        std::vector< std::future< void > > futures_end( vtsn.size() );

        for(size_t i_e = 0 ; i_e < futures_end.size() ; ++i_e){
            futures_end.at(i_e) = getFutureEnd(mutex,
                                               machine, infieldBoundary, maxConnectionLength,
                                               pose_start, pose_end,
                                               vt_start, vtsn.at(i_e), via_vts,
                                               astarParamsBase, astarSettings,
                                               minCost, initPath, finishPath, costStart);

        }

        for(auto& fu_e : futures_end)
            fu_e.wait();

    });
}

std::future<void> InfieldTracksConnectorGraphBased::getFutureEnd(std::mutex &mutex,
                                                                 const Machine &machine, const Polygon &infieldBoundary, const double& maxConnectionLength,
                                                                 const Pose2D &pose_start, const Pose2D &pose_end,
                                                                 const DirectedGraph::vertex_t &vt_start,
                                                                 const DirectedGraph::vertex_t &vt_end,
                                                                 const std::set<DirectedGraph::vertex_t>& via_vts,
                                                                 const Astar::PlanParameters &astarParamsBase,
                                                                 const Astar::AStarSettings &astarSettings,
                                                                 double &minCost,
                                                                 std::vector<Point> &initPath, std::vector<Point> &finishPath,
                                                                 const double& costStart) const
{

    return std::async( std::launch::async,
                       [this,
                       &mutex,
                       &machine, &infieldBoundary, maxConnectionLength,
                       &pose_start, &pose_end,
                       vt_start, vt_end, &via_vts,
                       &astarParamsBase, &astarSettings,
                       &minCost, &initPath, &finishPath, &costStart](){


        double costEnd = 0;

        DirectedGraph::vertex_property vt_end_prop = m_graph[vt_end];
        double dist_end = geometry::calc_dist(pose_end, vt_end_prop.route_point);
        if( dist_end > 1e-3 ){
            double speed = machine.calcSpeed( astarParamsBase.initial_bunker_mass );
            double dTime = speed > 0 ? dist_end / speed : 0;
            costEnd += m_costCalculator->calcCost(machine, pose_end, vt_end_prop.route_point, dTime, 0, astarParamsBase.initial_bunker_mass, {});
        }



        {
            std::lock_guard<std::mutex> guard(mutex);
        }

        VertexFunctionParams vtParams;
        vtParams.vt_start = vt_start;
        vtParams.vt_start_prop = m_graph[vt_start];
        vtParams.vt_goal = vt_end;
        vtParams.vt_goal_prop = m_graph[vt_end];
        vtParams.vt_start_valid = vtParams.vt_goal_valid = true;


        std::vector< std::future< void > > futures_via( via_vts.size() );

        size_t i_v = 0;
        for(auto& via_vt : via_vts){
            futures_via.at(i_v) = getFutureVia(mutex,
                                               machine, infieldBoundary, maxConnectionLength,
                                               pose_start, pose_end,
                                               vt_start, via_vt, vt_end,
                                               astarParamsBase, astarSettings,
                                               minCost, initPath, finishPath, costStart, costEnd,
                                               vtParams);
            i_v++;

        }

        for(auto& fu_v : futures_via)
            fu_v.wait();

    });

}

std::future<void> InfieldTracksConnectorGraphBased::getFutureVia(std::mutex &mutex,
                                                                 const Machine &machine, const Polygon &infieldBoundary, const double& maxConnectionLength,
                                                                 const Pose2D &pose_start, const Pose2D &pose_end,
                                                                 const DirectedGraph::vertex_t &vt_start,
                                                                 const DirectedGraph::vertex_t &vt_via,
                                                                 const DirectedGraph::vertex_t &vt_end,
                                                                 const Astar::PlanParameters &astarParamsBase,
                                                                 const Astar::AStarSettings &astarSettings,
                                                                 double &minCost,
                                                                 std::vector<Point> &initPath, std::vector<Point> &finishPath,
                                                                 const double& costStart, const double& costEnd,
                                                                 const VertexFunctionParams& vtParamsBase) const
{

    return std::async( std::launch::async,
                       [this,
                       &mutex,
                       &machine, &infieldBoundary, &maxConnectionLength,
                       &pose_start, &pose_end,
                       vt_start, vt_via, vt_end,
                       &astarParamsBase, &astarSettings,
                       &minCost, &initPath, &finishPath,
                       costStart, costEnd,
                       &vtParamsBase](){

        if(vt_via != vt_end){
            auto vtParams = vtParamsBase;
            vtParams.vt = vt_via;
            vtParams.vt_prop = m_graph[vt_via];
            if(!isVertexValid(vtParams, infieldBoundary, false))
                return;
        }


        //sets to hold invalid vts and edges already excluded in previous attempts
        std::set<DirectedGraph::vertex_t> invalidVts;
        std::set<DirectedGraph::edge_t> invalidEdges;

        std::vector<DirectedGraph::vertex_t> prevVtsInit;

        std::shared_ptr<const Astar::ISuccesorChecker> successorChecker = std::make_shared<AstarCustomSuccessorChecker>(
                    [this, &infieldBoundary, &invalidVts, &invalidEdges, &vt_start, &vt_end, &pose_start, &pose_end, &prevVtsInit]
                    (const Astar::ISuccesorChecker::IsSuccessorValidParams& params)->Astar::ISuccesorChecker::Validity
        {
            if(invalidVts.find(params.vt_to) != invalidVts.end())
                return Astar::ISuccesorChecker::INVALID__SUCCESSOR_EXCLUDED;

            if(invalidEdges.find(params.edge) != invalidEdges.end())
                return Astar::ISuccesorChecker::INVALID__EDGE_EXCLUDED;

            VertexFunctionParams vtParams;
            vtParams.fromSuccessorCheckerParams(params);

            if(!isVertexValid(vtParams, infieldBoundary, false)){
                invalidVts.insert(params.vt_to);
                return Astar::ISuccesorChecker::INVALID__SUCCESSOR_EXCLUDED;
            }

            EdgeFunctionParams edgeParams;
            edgeParams.fromSuccessorCheckerParams(params);

            if(!isEdgeValid(edgeParams, vt_start, vt_end, pose_start, pose_end, prevVtsInit)){
                invalidEdges.insert(params.edge);
                return Astar::ISuccesorChecker::INVALID__EDGE_EXCLUDED;
            }

            return Astar::ISuccesorChecker::VALID;

        } );


        prevVtsInit.clear();
        invalidVts.clear();
        invalidEdges.clear();

        auto astarParams = astarParamsBase;
        astarParams.start_vt = vt_start;
        astarParams.goal_vt = vt_via;
        astarParams.successorCheckers.push_back( successorChecker );

        Astar astarInit(astarParams,
                        astarSettings,
                        RoutePoint::TRANSIT,
                        getOutputSearchFolder(vt_start, vt_end, vt_via, true),
                        loggerPtr());

        if( !astarInit.plan(m_graph, m_costCalculator) || astarInit.getPlan().route_points_.size() < 2 ){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "AStar init search failed ["
                              + std::to_string(astarParams.start_vt) + "->" + std::to_string(astarParams.goal_vt) + "] ");
            return;
        }

        double cost = costStart + costEnd + astarInit.getPlan().plan_cost_total;
        double pathLength;

        {
            std::lock_guard<std::mutex> guard(mutex);
            if(cost > minCost)
                return;

            if(maxConnectionLength > 1e-9){
                pathLength = geometry::getGeometryLength(astarInit.getPlan().route_points_);
                if(pathLength > maxConnectionLength)
                    return;
            }

            if(vt_via == vt_end){
                double oldCost = minCost;
                minCost = cost;
                initPath = Point::toPoints( astarInit.getPlan().route_points_ );
                finishPath.clear();

                logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Obtained new connection ["
                                  + std::to_string(vt_start) + "->" + std::to_string(vt_end) + "] "
                                  + " via vertex " + std::to_string(vt_via)
                                  + ": cost " + std::to_string(oldCost) + " -> " + std::to_string(minCost));
                return;
            }
        }

        prevVtsInit = m_graph.getVerticesInRadius( r_at(astarInit.getPlan().route_points_, 1), 1e-3 );
        invalidVts.clear();
        invalidEdges.clear();

        astarParams.start_vt = vt_via;
        astarParams.goal_vt = vt_end;

        Astar astar(astarParams,
                    astarSettings,
                    RoutePoint::TRANSIT,
                    getOutputSearchFolder(vt_start, vt_end, vt_via, false),
                    loggerPtr());

        if( !astar.plan(m_graph, m_costCalculator) || astar.getPlan().route_points_.empty() ){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "AStar search failed ["
                              + std::to_string(astarParams.start_vt) + "->" + std::to_string(astarParams.goal_vt) + "] ");
            return;
        }

        cost += astar.getPlan().plan_cost_total;

        {
            std::lock_guard<std::mutex> guard(mutex);

            if(cost > minCost)
                return;

            if(maxConnectionLength > 1e-9){
                pathLength += geometry::getGeometryLength(astarInit.getPlan().route_points_);
                if(pathLength > maxConnectionLength)
                    return;
            }

            double oldCost = minCost;
            minCost = cost;
            initPath = Point::toPoints( astarInit.getPlan().route_points_ );
            finishPath = Point::toPoints( astar.getPlan().route_points_ );

            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Obtained new connection ["
                              + std::to_string(vt_start) + "->" + std::to_string(vt_end) + "] "
                              + " via vertex " + std::to_string(vt_via)
                              + ": cost " + std::to_string(oldCost) + " -> " + std::to_string(minCost));
        }

    } );
}

InfieldTracksConnectorGraphBased::InternalEdgeCostCalculator::InternalEdgeCostCalculator()
    : IEdgeCostCalculator(__FUNCTION__)
{
}

void InfieldTracksConnectorGraphBased::InternalEdgeCostCalculator::generateInternalParameters(DirectedGraph::Graph &graph)
{
    m_baseCostCalculator->generateInternalParameters(graph);
}

double InfieldTracksConnectorGraphBased::InternalEdgeCostCalculator::calcCost(const Machine& machine,
                                                                              const Point& p1,
                                                                              const Point& p2,
                                                                              double time,
                                                                              double waitingTime,
                                                                              double bunkerMass,
                                                                              const std::vector<DirectedGraph::overroll_property>& overruns)
{
    return m_baseCostCalculator->calcCost(machine, p1, p2, time, waitingTime, bunkerMass, overruns);
}

double InfieldTracksConnectorGraphBased::InternalEdgeCostCalculator::calcCost(const Machine& machine,
                                                                              const DirectedGraph::edge_t& edge,
                                                                              const DirectedGraph::edge_property& edge_prop,
                                                                              double time,
                                                                              double waitingTime,
                                                                              double bunkerMass)
{
    if(edge_prop.edge_type == DirectedGraph::BOUNDARY_CONN
            || !m_boundary || m_boundary->points.size() < 4)
        return m_baseCostCalculator->calcCost(machine, edge, edge_prop, time, waitingTime, bunkerMass);

    double distComp = m_turningRad > 1e-9 ? 0.02 : 0;

    double distToBoundary0 = ag::calc_dist_to_linestring(m_boundary->points, edge_prop.p0, false);
    double distToBoundary1 = ag::calc_dist_to_linestring(m_boundary->points, edge_prop.p1, false);
    if(distToBoundary0 > distComp && distToBoundary1 > distComp)
        return m_baseCostCalculator->calcCost(machine, edge, edge_prop, time, waitingTime, bunkerMass);

    //the point is too close to the boundary -> treat the edge as a boundary_connection
    auto edge_prop_ed = edge_prop;
    edge_prop_ed.edge_type = DirectedGraph::BOUNDARY_CONN;

    return m_baseCostCalculator->calcCost(machine, edge, edge_prop_ed, time, waitingTime, bunkerMass);

}

double InfieldTracksConnectorGraphBased::InternalEdgeCostCalculator::calcHeuristic(const Machine &machine, const DirectedGraph::vertex_property &v_prop_current, const DirectedGraph::vertex_property &v_prop_goal)
{
    return m_baseCostCalculator->calcHeuristic(machine, v_prop_current, v_prop_goal);
}

bool InfieldTracksConnectorGraphBased::InternalEdgeCostCalculator::parseOtherParametersFromStringMap(const std::map<std::string, std::string> &strMap, bool strict)
{
    return true;
}

void InfieldTracksConnectorGraphBased::InternalEdgeCostCalculator::parseAndAppendOtherParametersToStringMap(std::map<std::string, std::string> &strMap) const
{
}



}

