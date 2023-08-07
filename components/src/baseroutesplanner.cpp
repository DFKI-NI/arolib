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
 
#include "arolib/components/baseroutesplanner.h"

namespace arolib {


BaseRoutesPlanner::PlannerParameters::PlannerParameters()
{
    //start with HL and IF planner parameters defaults
    fromHeadlandPlannerParameters( HeadlandBaseRoutesPlanner::PlannerParameters() );
    fromInfieldPlannerParameters( InfieldBaseRoutesPlanner::PlannerParameters() );
}

bool BaseRoutesPlanner::PlannerParameters::parseFromStringMap(BaseRoutesPlanner::PlannerParameters &params, const std::map<std::string, std::string> &map, bool strict)
{
    try{
        BaseRoutesPlanner::PlannerParameters tmp;

        if( !FieldGeneralParameters::parseFromStringMap(tmp, map, strict) )
            return false;
        if( !GridComputationSettings::parseFromStringMap(tmp, map, strict) )
            return false;

        int workedAreaTransitRestriction = tmp.workedAreaTransitRestriction;
        int headlandMachineOrderStrategy = tmp.headlandMachineOrderStrategy;

        std::map<std::string, double*> dMap = { {"headlandSpeedMultiplier" , &tmp.headlandSpeedMultiplier} };
        std::map<std::string, bool*> bMap = { {"workHeadlandFirst" , &tmp.workHeadlandFirst},
                                              {"startHeadlandFromOutermostTrack" , &tmp.startHeadlandFromOutermostTrack},
                                              {"finishHeadlandWithOutermostTrack" , &tmp.finishHeadlandWithOutermostTrack},
                                              {"headlandClockwise" , &tmp.headlandClockwise},
                                              {"restrictToBoundary" , &tmp.restrictToBoundary},
                                              {"monitorPlannedAreasInHeadland" , &tmp.monitorPlannedAreasInHeadland},
                                              {"limitStartToExtremaTracks" , &tmp.limitStartToExtremaTracks},
                                              {"useMachineTurningRadInTrackSequencer" , &tmp.useMachineTurningRadInTrackSequencer},
                                              {"infieldInverseTrackOrder" , &tmp.infieldInverseTrackOrder},
                                              {"infieldInversePointsOrder" , &tmp.infieldInversePointsOrder} };
        std::map<std::string, int*> enumMap = { {"workedAreaTransitRestriction" , &workedAreaTransitRestriction},
                                                {"headlandMachineOrderStrategy" , &headlandMachineOrderStrategy} };


        if( !setValuesFromStringMap( map, dMap, strict)
                || !setValuesFromStringMap( map, bMap, strict)
                || !setValuesFromStringMap( map, enumMap, strict) )
            return false;

        tmp.workedAreaTransitRestriction = HeadlandBaseRoutesPlanner::intToWorkedAreaTransitRestriction( workedAreaTransitRestriction );
        tmp.headlandMachineOrderStrategy = HeadlandBaseRoutesPlanner::intToMachineOrderStrategy( headlandMachineOrderStrategy );


        params = tmp;
        return true;
    }
    catch(...){
        return false;
    }
}

std::map<std::string, std::string> BaseRoutesPlanner::PlannerParameters::parseToStringMap(const BaseRoutesPlanner::PlannerParameters &params)
{
    std::map<std::string, std::string> ret, subMap;
    subMap = FieldGeneralParameters::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );
    subMap = GridComputationSettings::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );

    ret["headlandSpeedMultiplier"] = double2string( params.headlandSpeedMultiplier );
    ret["workHeadlandFirst"] = std::to_string( params.workHeadlandFirst );
    ret["startHeadlandFromOutermostTrack"] = std::to_string( params.startHeadlandFromOutermostTrack );
    ret["finishHeadlandWithOutermostTrack"] = std::to_string( params.finishHeadlandWithOutermostTrack );
    ret["headlandClockwise"] = std::to_string( params.headlandClockwise );
    ret["restrictToBoundary"] = std::to_string( params.restrictToBoundary );
    ret["monitorPlannedAreasInHeadland"] = std::to_string( params.monitorPlannedAreasInHeadland );
    ret["infieldInverseTrackOrder"] = std::to_string( params.infieldInverseTrackOrder );
    ret["limitStartToExtremaTracks"] = std::to_string( params.limitStartToExtremaTracks );
    ret["useMachineTurningRadInTrackSequencer"] = std::to_string( params.useMachineTurningRadInTrackSequencer );
    ret["infieldInversePointsOrder"] = std::to_string( params.infieldInversePointsOrder );
    ret["workedAreaTransitRestriction"] = std::to_string( params.workedAreaTransitRestriction );
    ret["headlandMachineOrderStrategy"] = std::to_string( params.headlandMachineOrderStrategy );

    return ret;
}

HeadlandBaseRoutesPlanner::PlannerParameters BaseRoutesPlanner::PlannerParameters::toHeadlandPlannerParameters() const
{
    HeadlandBaseRoutesPlanner::PlannerParameters params;
    *((FieldGeneralParameters*)(&params)) = *((FieldGeneralParameters*)(this));
    *((GridComputationSettings*)(&params)) = *((GridComputationSettings*)(this));

    params.startFromOutermostTrack = startHeadlandFromOutermostTrack;
    params.finishWithOutermostTrack = finishHeadlandWithOutermostTrack;
    params.clockwise = headlandClockwise;
    params.speedMultiplier = headlandSpeedMultiplier;
    params.removeInitialWorkedSegments = true;
    params.restrictToBoundary = restrictToBoundary;
    params.monitorPlannedAreas = monitorPlannedAreasInHeadland;
    params.machineOrderStrategy = headlandMachineOrderStrategy;
    params.workedAreaTransitRestriction = workedAreaTransitRestriction;

    return params;
}

InfieldBaseRoutesPlanner::PlannerParameters BaseRoutesPlanner::PlannerParameters::toInfieldPlannerParameters() const
{
    InfieldBaseRoutesPlanner::PlannerParameters params;
    *((FieldGeneralParameters*)(&params)) = *((FieldGeneralParameters*)(this));
    *((GridComputationSettings*)(&params)) = *((GridComputationSettings*)(this));

    params.limitStartToExtremaTracks = limitStartToExtremaTracks;
    params.useMachineTurningRad = useMachineTurningRadInTrackSequencer;
    params.inverseTrackOrder = infieldInverseTrackOrder;
    params.inversePointsOrder = infieldInversePointsOrder;
    params.sampleResolutionHeadland = 0;
    params.removeInitialWorkedSegments = true;

    return params;
}

void BaseRoutesPlanner::PlannerParameters::fromHeadlandPlannerParameters(const HeadlandBaseRoutesPlanner::PlannerParameters &params)
{
    *((FieldGeneralParameters*)(this)) = *((FieldGeneralParameters*)(&params));
    *((GridComputationSettings*)(this)) = *((GridComputationSettings*)(&params));

    startHeadlandFromOutermostTrack = params.startFromOutermostTrack;
    finishHeadlandWithOutermostTrack = params.finishWithOutermostTrack;
    headlandClockwise = params.clockwise;
    restrictToBoundary = params.restrictToBoundary;
    monitorPlannedAreasInHeadland = params.monitorPlannedAreas;
    headlandSpeedMultiplier = params.speedMultiplier;
    headlandMachineOrderStrategy = params.machineOrderStrategy;
    workedAreaTransitRestriction = params.workedAreaTransitRestriction;
}

void BaseRoutesPlanner::PlannerParameters::fromInfieldPlannerParameters(const InfieldBaseRoutesPlanner::PlannerParameters &params)
{
    *((FieldGeneralParameters*)(this)) = *((FieldGeneralParameters*)(&params));
    *((GridComputationSettings*)(this)) = *((GridComputationSettings*)(&params));

    limitStartToExtremaTracks = params.limitStartToExtremaTracks;
    useMachineTurningRadInTrackSequencer = params.useMachineTurningRad;
    infieldInverseTrackOrder = params.inverseTrackOrder;
    infieldInversePointsOrder = params.inversePointsOrder;
}

BaseRoutesPlanner::BaseRoutesPlanner(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{
    if(logger().logLevel() == LogLevel::DEBUG)
        logger().includeElapsedTime(true);

    setDefInfieldTrackSequencer();
    setDefTrackConnector_headland2infield();
    setDefTrackConnector_infield();
}

AroResp BaseRoutesPlanner::plan(const Subfield &subfield,
                                const std::vector<Machine> &workinggroup,
                                const PlannerParameters &plannerParameters,
                                std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorHeadland,
                                std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorInfield,
                                std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit,
                                std::vector<Route> &routes,
                                std::shared_ptr<ArolibGrid_t> massFactorMap,
                                const std::map<MachineId_t, MachineDynamicInfo> *machineCurrentStates,
                                const Pose2D *initRefPose,
                                const OutFieldInfo *outFieldInfo,
                                std::shared_ptr<const ArolibGrid_t> remainingAreaMap)
{
    //@todo: the LoggersHandler is causing segfault
    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction

    routes.clear();
    std::vector<Route> routes_hl, routes_if;

    auto machines = Machine::toMachineIdMap(workinggroup);

    bool headlandFirst = plannerParameters.workHeadlandFirst;
    auto workedAreaTransitRestriction = HeadlandBaseRoutesPlanner::getWorkedAreaTransitRestriction(plannerParameters.workedAreaTransitRestriction, machines);
    if(workedAreaTransitRestriction == HeadlandBaseRoutesPlanner::WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_WORKED_AREA)
        headlandFirst = true;
    else if(workedAreaTransitRestriction == HeadlandBaseRoutesPlanner::WorkedAreaTransitRestriction::TRANSIT_ONLY_OVER_UNWORKED_AREA)
        headlandFirst = false;

    AroResp aroResp;
    HeadlandBaseRoutesPlanner headlandPlanner(logger().logLevel());
    InfieldBaseRoutesPlanner infieldPlanner(logger().logLevel());

    headlandPlanner.logger().setParent(loggerPtr());
    infieldPlanner.logger().setParent(loggerPtr());

    headlandPlanner.setGridCellsInfoManager(m_cim);
    infieldPlanner.setGridCellsInfoManager(m_cim);

    infieldPlanner.setInfieldTrackSequencer(m_tracksSequencer);
    infieldPlanner.setInfieldTrackConnector(m_tracksConnector_if);

    if(headlandFirst){
        aroResp = headlandPlanner.plan(subfield,
                                       workinggroup,
                                       plannerParameters.toHeadlandPlannerParameters(),
                                       edgeMassCalculator,
                                       edgeSpeedCalculatorHeadland,
                                       routes_hl,
                                       massFactorMap,
                                       machineCurrentStates,
                                       initRefPose,
                                       outFieldInfo,
                                       remainingAreaMap);
        if(aroResp.isError())
            return AroResp::LoggingResp(1, "Error obtaining the headland base routes", ": " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);

        removeWorkedPoints(routes_hl);

        auto refPose = getRefPose(routes_hl);
        auto machineStates = machineCurrentStates? *machineCurrentStates : std::map<MachineId_t, MachineDynamicInfo>();
        updatedMachineStates(routes_hl, machineStates);
        auto massFactorMapEd = getUpdatedMassFactorMap_infield(subfield, machines, routes_hl, massFactorMap);

        aroResp = infieldPlanner.plan(subfield,
                                      workinggroup,
                                      plannerParameters.toInfieldPlannerParameters(),
                                      edgeMassCalculator,
                                      edgeSpeedCalculatorInfield,
                                      edgeSpeedCalculatorTransit,
                                      routes_if,
                                      !machineStates.empty() ? &machineStates : nullptr,
                                      refPose.isValid() ? &refPose : initRefPose,
                                      massFactorMapEd,
                                      remainingAreaMap);

        if(aroResp.isError())
            return AroResp::LoggingResp(1, "Error obtaining the infield base routes", ": " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);

        removeWorkedTracks(routes_if);

        aroResp = connectHeadlandAndInfieldRoutes(subfield, routes_hl, routes_if, edgeSpeedCalculatorTransit, machines, routes);
        if(aroResp.isError())
            return AroResp::LoggingResp(1, "Error connecting headland and infield base routes", ": " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);
    }
    else{
        aroResp = infieldPlanner.plan(subfield,
                                      workinggroup,
                                      plannerParameters.toInfieldPlannerParameters(),
                                      edgeMassCalculator,
                                      edgeSpeedCalculatorInfield,
                                      edgeSpeedCalculatorTransit,
                                      routes_if,
                                      machineCurrentStates,
                                      initRefPose,
                                      massFactorMap,
                                      remainingAreaMap);
        if(aroResp.isError())
            return AroResp::LoggingResp(1, "Error obtaining the infield base routes", ": " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);

        removeWorkedPoints(routes_if);

        auto refPose = getRefPose(routes_if);
        auto machineStates = machineCurrentStates? *machineCurrentStates : std::map<MachineId_t, MachineDynamicInfo>();
        updatedMachineStates(routes_if, machineStates);

        auto massFactorMapEd = getUpdatedMassFactorMap_headland(subfield, machines, routes_if, massFactorMap);

        aroResp = headlandPlanner.plan(subfield,
                                       workinggroup,
                                       plannerParameters.toHeadlandPlannerParameters(),
                                       edgeMassCalculator,
                                       edgeSpeedCalculatorHeadland,
                                       routes_hl,
                                       massFactorMapEd,
                                       !machineStates.empty() ? &machineStates : nullptr,
                                       refPose.isValid() ? &refPose : initRefPose,
                                       outFieldInfo,
                                       remainingAreaMap);
        if(aroResp.isError())
            return AroResp::LoggingResp(1, "Error obtaining the headland base routes", ": " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);

        removeWorkedPoints(routes_hl);

        aroResp = connectInfieldAndHeadlandRoutes(subfield, routes_hl, routes_if, edgeSpeedCalculatorTransit, machines, routes);
        if(aroResp.isError())
            return AroResp::LoggingResp(1, "Error connecting infield and headland base routes", ": " + aroResp.msg, m_logger, LogLevel::ERROR, __FUNCTION__);
    }

    return AroResp::ok();
}

void BaseRoutesPlanner::setInfieldTrackSequencer(std::shared_ptr<ITrackSequencer> track_sequencer) {
    if(track_sequencer)
        m_tracksSequencer = track_sequencer;
    else
        setDefInfieldTrackSequencer();
}

void BaseRoutesPlanner::setTrackConnector_headland2infield(std::shared_ptr<IInfieldTracksConnector> connector)
{
    if(connector)
        m_tracksConnector_hl2if = connector;
    else
        setDefTrackConnector_headland2infield();
}

void BaseRoutesPlanner::setTrackConnector_infield(std::shared_ptr<IInfieldTracksConnector> connector)
{
    if(connector)
        m_tracksConnector_if = connector;
    else
        setDefTrackConnector_infield();
}

void BaseRoutesPlanner::setGridCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim)
{
    m_cim = cim;
}

void BaseRoutesPlanner::setDefInfieldTrackSequencer()
{
    m_tracksSequencer = std::make_shared<SimpleTrackSequencer>();
    m_tracksSequencer->logger().setParent(loggerPtr());
}

void BaseRoutesPlanner::setDefTrackConnector_headland2infield()
{
    m_tracksConnector_hl2if = std::make_shared<InfieldTracksConnectorDef>();
    m_tracksConnector_hl2if->logger().setParent(loggerPtr());
}

void BaseRoutesPlanner::setDefTrackConnector_infield()
{
    m_tracksConnector_if = std::make_shared<InfieldTracksConnectorDef>();
    m_tracksConnector_if->logger().setParent(loggerPtr());
}

void BaseRoutesPlanner::removeWorkedTracks(std::vector<Route> &routes)
{
    for(auto& route : routes){
        auto& points = route.route_points;
        for(int i = 0; i < points.size() ; ++i){
            auto& rp = points.at(i);
            if(rp.time_stamp > -1e-6)
                break;
            if(rp.type == RoutePoint::TRACK_START && i > 0){
                points.erase(points.begin(), points.begin()+i);
                i = -1;
            }
        }
    }
}

Route BaseRoutesPlanner::removeWorkedPoints(Route &route)
{
    Route removedPoints;
    auto& points = route.route_points;
    int indCut = -1;
    for(int i = 0; i < points.size() ; ++i){
        auto& rp = points.at(i);
        if(rp.time_stamp > -1e-6){
            indCut = i;
            break;
        }
    }
    if(indCut < 0){
        removedPoints = route;
        route.route_points.clear();
        return removedPoints;
    }

    route.copyToWithoutPoints( removedPoints );

    if(indCut > 0){
        auto& rps = removedPoints.route_points;
        rps.insert(rps.end(), points.begin(), points.begin()+indCut);
        points.erase(points.begin(), points.begin()+indCut);
    }
    return removedPoints;
}

std::vector<Route> BaseRoutesPlanner::removeWorkedPoints(std::vector<Route> &routes)
{
    std::vector<Route> removedPoints;
    for(auto& route : routes)
        removedPoints.push_back( removeWorkedPoints(route) );
    return removedPoints;
}

Pose2D BaseRoutesPlanner::getRefPose(const std::vector<Route> &routes)
{
    Pose2D ret( Point::invalidPoint() );
    int ind = -1;
    double minTimestamp = std::numeric_limits<double>::max();
    for(size_t i = 0 ; i < routes.size() ; ++i){
        auto& rps = routes.at(i).route_points;
        if(rps.empty())
            continue;
        if(rps.back().time_stamp < minTimestamp){
            minTimestamp = rps.back().time_stamp;
            ind = i;
        }
    }
    if(ind >= 0){
        auto& rps = routes.at(ind).route_points;
        ret.point() = rps.back();
        if(rps.size() > 1)
            ret.angle = geometry::get_angle( r_at(rps, 1), rps.back() );
    }

    return ret;
}

void BaseRoutesPlanner::updatedMachineStates(const std::vector<Route> &routes, std::map<MachineId_t, MachineDynamicInfo> &machineStates)
{
    for(auto& route : routes){
        auto& rps = route.route_points;
        if(rps.empty())
            continue;
        MachineDynamicInfo mdi;
        auto it_mdi = machineStates.find(route.machine_id);
        if(it_mdi != machineStates.end())
            mdi = it_mdi->second;
        else
            mdi.bunkerMass = mdi.bunkerVolume = 0;
        mdi.position = rps.back();
        if(rps.size() > 1)
            mdi.theta = geometry::get_angle( r_at(rps, 1), rps.back() );
        machineStates[route.machine_id] = mdi;
    }
}

std::shared_ptr<ArolibGrid_t> BaseRoutesPlanner::getUpdatedMassFactorMap_headland(const Subfield &subfield, const std::map<MachineId_t, Machine> &machines, const std::vector<Route> &routes_if, std::shared_ptr<const ArolibGrid_t> massFactorMap)
{
    std::shared_ptr<ArolibGrid_t> ret = std::make_shared<ArolibGrid_t>();
    double minWidth = std::numeric_limits<double>::max();
    for(auto& route : routes_if){
        auto it_m = machines.find(route.machine_id);
        if(it_m == machines.end())
            continue;
        minWidth = std::min(minWidth, it_m->second.working_width);
    }

    double cellsize = std::min(2.0, 0.5 * minWidth);

    if(massFactorMap && massFactorMap->isAllocated()){
        ret->copyFrom(*massFactorMap, false);
        if(ret->getCellsize() < 0.5 * minWidth){
            int scale = ret->getCellsize() / cellsize + 1;
            ret->scaleResolution(scale);
        }
    }
    else
        ret->convertPolygonToGrid(subfield.boundary_outer.points, cellsize, 1.0);

    ret->updatePolygonProportionally(subfield.boundary_inner, 0.0, true);
    return ret;
}

std::shared_ptr<ArolibGrid_t> BaseRoutesPlanner::getUpdatedMassFactorMap_infield(const Subfield &subfield, const std::map<MachineId_t, Machine> &machines, const std::vector<Route> &routes_hl, std::shared_ptr<const ArolibGrid_t> massFactorMap)
{
    std::shared_ptr<ArolibGrid_t> ret = std::make_shared<ArolibGrid_t>();
    double minWidth = std::numeric_limits<double>::max();
    for(auto& route : routes_hl){
        auto it_m = machines.find(route.machine_id);
        if(it_m == machines.end())
            continue;
        minWidth = std::min(minWidth, it_m->second.working_width);
    }

    double cellsize = std::min(2.0, 0.5 * minWidth);

    if(massFactorMap && massFactorMap->isAllocated()){
        ret->createGrid(massFactorMap->getLayout(), nullptr);
        if(ret->getCellsize() < 0.5 * minWidth){
            int scale = ret->getCellsize() / cellsize + 1;
            ret->scaleResolution(scale);
        }
        ret->updatePolygonProportionally(subfield.boundary_inner, 1.0, false);

        for(size_t x = 0 ; x < ret->getSizeX() ; x++){
            for(size_t y = 0 ; y < ret->getSizeY() ; y++){
                if(!ret->hasValue(x, y))
                    continue;
                Point center;
                ret->getCellCenter(x, y, center);
                if(massFactorMap->hasValue(center))
                    ret->multValue(x, y, massFactorMap->getValue(center));
                else
                    ret->setNoValue(x, y); //assuming that the cells with no-value are read as 0 in the planners
            }
        }
    }
    else{
        ret->convertPolygonToGrid(subfield.boundary_outer.points, cellsize, nullptr);
        ret->updatePolygonProportionally(subfield.boundary_inner, 1.0, false);
    }

    return ret;
}

AroResp BaseRoutesPlanner::connectHeadlandAndInfieldRoutes(const Subfield &subfield,
                                                           std::vector<Route> &routes_hl,
                                                           std::vector<Route> &routes_if,
                                                           std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit,
                                                           const std::map<MachineId_t, Machine> &machines,
                                                           std::vector<Route> &routes)
{
    routes = routes_hl;
    std::map<MachineId_t, size_t> idsMap;
    for(size_t i = 0 ; i < routes.size() ; ++i){
        auto& route = routes.at(i);
        if(route.route_points.empty())
            continue;

        auto it_m = machines.find(route.machine_id);
        if(it_m == machines.end())
            return AroResp(1, "The machine assigned to one of the routes is not in the working group");

        //adjust worked mass/volume
        auto rp0 = route.route_points.front();
        if(rp0.worked_mass > 1e-6 || rp0.worked_volume > 1e-6){
            for(auto& rp : route.route_points){
                rp.worked_mass -= rp0.worked_mass;
                rp.worked_volume -= rp0.worked_volume;
            }
        }
        idsMap[route.machine_id] = i;
    }

    for(auto& route_if : routes_if){
        auto workedSegment = Point::toPoints( removeWorkedPoints(route_if).route_points );

        if(route_if.route_points.empty())
            continue;

        auto it_m = machines.find(route_if.machine_id);
        if(it_m == machines.end())
            return AroResp(1, "The machine assigned to one of the routes is not in the working group");
        const Machine& machine = it_m->second;

        RoutePoint rpRef;
        auto it_r = idsMap.find(route_if.machine_id);
        if(it_r == idsMap.end()){
            routes.push_back(route_if);
            rpRef = route_if.route_points.front();

            //adjust worked mass/volume
            if(rpRef.worked_mass > 1e-6 || rpRef.worked_volume > 1e-6){
                for(auto& rp : routes.back().route_points){
                    rp.worked_mass -= rpRef.worked_mass;
                    rp.worked_volume -= rpRef.worked_volume;
                }
            }
            continue;
        }

        auto& route = routes.at(it_r->second);
        rpRef = route.route_points.back();

        double turningRad = machine.getTurningRadius();

        Polygon boundary = m_tracksConnector_hl2if->getExtendedLimitBoundary(subfield, turningRad);
//        double boundaryTH = 2*turningRad;
//        Polygon boundary = subfield.boundary_outer;
//        float distToBoundary = geometry::calc_dist_to_linestring(boundary.points, route.route_points.back(), false);
//        if( distToBoundary < boundaryTH )
//            geometry::offsetPolygon(subfield.boundary_outer, boundary, boundaryTH - distToBoundary, true);

        Pose2D pose0( route.route_points.back() );
        if(route.route_points.size() > 1)
            pose0.angle = geometry::get_angle( r_at(route.route_points, 1), pose0 );
        Pose2D posen;
        if(workedSegment.empty()){
            posen.point() = route_if.route_points.front();
            if(route_if.route_points.size() > 1)
                posen.angle = geometry::get_angle( posen, route_if.route_points.at(1) );
        }
        else{
            posen.point() = workedSegment.front();
            if(workedSegment.size() > 1)
                posen.angle = geometry::get_angle( posen, workedSegment.at(1) );
            else
                posen.angle = geometry::get_angle( posen, route_if.route_points.front() );
        }

        double extraDist_hl2if0 = subfield.headlands.hasCompleteHeadland() ? -1 : /*0.5 **/ turningRad;
        double extraDist_hl2ifn = -1;

        auto path = m_tracksConnector_hl2if->getConnection(machine, pose0, posen, turningRad,
                                                           std::make_pair(extraDist_hl2if0, extraDist_hl2ifn),
                                                           boundary, subfield.boundary_inner, subfield.headlands);
        if(path.size() < 2 && extraDist_hl2if0 > 1e-9)
            path = m_tracksConnector_hl2if->getConnection(machine, pose0, posen, turningRad, std::make_pair(-1, -1), boundary, subfield.boundary_inner, subfield.headlands);

        if(path.size() < 2){
            logger().printWarning("Error connecting routes of machine with id " + std::to_string(route.machine_id) + " using the turning radius. Trying again without turning radius... ");
            path = m_tracksConnector_hl2if->getConnection(machine, pose0, posen, 0.0, std::make_pair(-1, -1), boundary, subfield.boundary_inner, subfield.headlands);
            if(path.size() < 2){
                return AroResp(1, "Error connecting routes of machine with id " + std::to_string(route.machine_id));
            }
        }

        path.pop_back();
        if(workedSegment.size() > 1)
            path.insert(path.end(), workedSegment.begin(), workedSegment.end());

        for(size_t i = 0 ; i < path.size() ; ++i){
            Point p0 = path.at(i);
            RoutePoint rp = route.route_points.back();
            rp.track_id = -1;
            if(i+1 < path.size())
                rp.point() = path.at(i+1);
            else
                rp.point() = route.route_points.front().point();
            rp.type = RoutePoint::TRANSIT;

            double speed = edgeSpeedCalculatorTransit->calcSpeed(p0, rp, rp.bunker_mass, machine);
            if (speed < 1e-9)
                return AroResp(1, "Invalid speed for machine with id " + std::to_string(machine.id));

            double dtime = geometry::calc_dist(p0, rp) / speed;
            rp.time_stamp += dtime;

            if(i+1 < path.size())
                route.route_points.push_back(rp);
            else
                rpRef = rp;
        }

        route.route_points.insert(route.route_points.end(), route_if.route_points.begin(), route_if.route_points.end());

        //adjust timestamps and worked mass/volume
        RoutePoint rp0 = route_if.route_points.front();
        for(size_t i = 0 ; i < route_if.route_points.size() ; ++i){
            RoutePoint& rp = r_at(route.route_points, i);
            rp.time_stamp = rp.time_stamp - rp0.time_stamp + rpRef.time_stamp;
            rp.worked_mass = rp.worked_mass - rp0.worked_mass + rpRef.worked_mass;
            rp.worked_volume = rp.worked_volume - rp0.worked_volume + rpRef.worked_volume;
        }
    }

    return AroResp::ok();
}

AroResp BaseRoutesPlanner::connectInfieldAndHeadlandRoutes(const Subfield &subfield, std::vector<Route> &routes_if, std::vector<Route> &routes_hl, std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit, const std::map<MachineId_t, Machine> &machines, std::vector<Route> &routes)
{
    // @todo
    return AroResp(1, "Infield-to-headland connection not supported at the moment");
}

}

