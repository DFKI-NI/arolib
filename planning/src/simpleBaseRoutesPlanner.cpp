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
 
#include "arolib/planning/simpleBaseRoutesPlanner.hpp"
#include "arolib/geometry/geometry_helper.hpp"

namespace arolib{
SimpleBaseRoutesPlanner::SimpleBaseRoutesPlanner(const LogLevel &logLevel) :
    LoggingComponent(logLevel, __FUNCTION__),
    m_has_subfield (false),
    m_has_machine (false)
{

    m_track_sequencer = std::make_shared<SimpleTrackSequencer>();
    m_track_sequencer->logger().setParent(loggerPtr());
}

AroResp SimpleBaseRoutesPlanner::setSubfield(const Subfield& subfield) {
    if(subfield.tracks.empty())
        return AroResp(1, "No tracks given");
    for(auto& track : subfield.tracks){
        if(track.points.empty())
            return AroResp(1, "One or more tracks are empty");
    }
    m_subfield = subfield;
    m_has_subfield = true;
    return AroResp::ok();
}

AroResp SimpleBaseRoutesPlanner::addMachine(const Machine &machine) {
    if( m_machinesMap.find(machine.id) != m_machinesMap.end() )
        return AroResp(1, "Machine with the same id already added");
    m_machinesMap[machine.id] = machine;
    m_machines.push_back(machine);
    m_has_machine = true;
    return AroResp::ok();
}

AroResp SimpleBaseRoutesPlanner::plan(std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                      std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                      std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit) {
    if(!m_has_subfield || !m_has_machine)
        return AroResp(1, "Subfield or machines missing");

    Pose2D initRefPose = getInitRefPoseForSequencer();
    if(!initRefPose.isValid())
        return AroResp(1, "Error obtaining initial reference point for sequencer");

    std::map<MachineId_t, std::vector<ITrackSequencer::TrackInfo>> sequences;

    auto aroResp = m_track_sequencer->computeSequences(m_subfield, m_machines, m_track_sequencer_settings, sequences, &initRefPose, m_excludeTrackIndexes);
    if(aroResp.isError())
        return AroResp(1, "Error obtaining tracks' sequence: " + aroResp.msg);

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Sequence computed!");

    std::map< MachineId_t, RouteAssembler* > routeAssemblers;
    std::vector< RouteAssembler > routeAss_v;//for some reason, the routeAssemblers.at() tries to call the RouteAssembler constructur. this is a workarround
    int route_id = 0;
    for(auto& it_m : sequences){
        const auto & machineId = it_m.first;
        if(it_m.second.empty())
            continue;
        auto it_m2 = m_machinesMap.find(machineId);
        if(it_m2 == m_machinesMap.end())
            return AroResp(1, "Sequence obtained for a non-existing machine");
        const Machine& machine = it_m2->second;

        routeAss_v.push_back( RouteAssembler(machine, route_id++, RoutePoint::getDefaultRPType(machine), logger().logLevel()) );
        routeAssemblers[it_m.first] = &(routeAss_v.back());

        RouteAssembler& routeAss = *( routeAssemblers.at(machineId) );
        routeAss.logger().setParent(loggerPtr());
        routeAss.setMassCalculator(edgeMassCalculator);
        routeAss.setWorkingSpeedCalculator(edgeSpeedCalculator);
        routeAss.setTransitSpeedCalculator(edgeSpeedCalculatorTransit);
        routeAss.setTracksConnectionsMap( m_track_sequencer->getPathsMap() );
    }

    double limBoundaryDist_turningRadFactor = 0.1;//@todo we should either put this as a setting of the planner or simple not allow transit outside the subfield boundary
    if(!m_subfield.headlands.hasCompleteHeadland())//workaround for partial headlands (some connections will very likelly lie outside the boundary)
        limBoundaryDist_turningRadFactor = 2.0; //1.5;

    std::map<MachineId_t, Polygon> limitBoundaries;
    for(auto& it_m : sequences){//generate the routes for each machine based on the assigned tracks and their sequence

        //get the sequence
        const auto & machineId = it_m.first;
        const auto & sequence = it_m.second;

        logger().printOut(LogLevel::INFO, __FUNCTION__, "Sequence gotten for machine with id " + std::to_string(it_m.first)
                                                         + " ( " + std::to_string(sequence.size()) + " tracks )");

        const Machine& machine = m_machinesMap[machineId];

        //get and initialize the route assembler for this machine/route
        RouteAssembler& routeAss = *( routeAssemblers.at(machineId) );

//        const Polygon *limitBoundary = &m_subfield.boundary_outer;//limit boundary used for inter-track connections
//        if(limBoundaryDist_turningRadFactor > 1e-9){
//            if( limitBoundaries.find( machine.id ) == limitBoundaries.end() ){
//                Polygon limitBoundaryTmp;
//                if( arolib::geometry::offsetPolygon(m_subfield.boundary_outer, limitBoundaryTmp, limBoundaryDist_turningRadFactor * machine.getTurningRadius(), true, 0) )
//                    limitBoundaries[machine.id] = limitBoundaryTmp;
//                else
//                    limitBoundaries[machine.id] = m_subfield.boundary_outer;
//            }
//            limitBoundary = &(limitBoundaries[machine.id]);
//        }

        auto it_bound = limitBoundaries.find( machine.id );
        if( it_bound == limitBoundaries.end() ){
            limitBoundaries[machine.id] = routeAss.getTracksConnector()->getExtendedLimitBoundary(m_subfield, machine.getTurningRadius());
            it_bound = limitBoundaries.find( machine.id );
        }
        const Polygon *limitBoundary = &(it_bound->second);

        //build the route by adding all assigned tracks in the given order, adding the corresponding inter-track (headland) connections in between
        for (int k = 0; k < sequence.size(); ++k) {
            const ITrackSequencer::TrackInfo& trackInfo = sequence.at(k);
            auto points = m_subfield.tracks.at(trackInfo.trackIndex).points;//get a copy of the next track in the sequence

            if(trackInfo.trackPointsDirection == ITrackSequencer::REVERSE)//reverse the track points if necesary
                std::reverse(points.begin(), points.end());

            if( !routeAss.addTrack(m_subfield,
                                   points,
                                   m_subfield.tracks.at(trackInfo.trackIndex).id,
                                   trackInfo.trackPointsDirection == ITrackSequencer::UNDEF,
                                   *limitBoundary) ){
                return AroResp::LoggingResp(1, "Error adding track " + std::to_string(k) + " (track index = " + std::to_string(trackInfo.trackIndex) + ") of sequence for machine with id " + std::to_string(machineId), m_logger, LogLevel::ERROR, __FUNCTION__);
            }

            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Track " + std::to_string(k) + " (track index = " + std::to_string(trackInfo.trackIndex) + ") of sequence for machine with id " + std::to_string(machineId) + " added.");

        }
        m_machines_routes[machineId] = routeAss.getRoute();//retrieve the routes
    }

    return AroResp::ok();
}

Pose2D SimpleBaseRoutesPlanner::getInitRefPoseForSequencer()
{
    if(m_initRefPose.isValid())
        return m_initRefPose;

    int ind0 = -1;
    for(size_t i = 0 ; i < m_subfield.tracks.size() ; ++i){
        if( m_excludeTrackIndexes.find(i) == m_excludeTrackIndexes.end() ){
            ind0 = i;
            break;
        }
    }
    if(ind0 < 0)
        return Pose2D(Point::invalidPoint(), 0);

    int indn;
    for(int i = m_subfield.tracks.size() - 1 ; i >= ind0 ; --i){
        if( m_excludeTrackIndexes.find(i) == m_excludeTrackIndexes.end() ){
            indn = i;
            break;
        }
    }

    auto& track = m_tracks_in_reverse ? m_subfield.tracks.at(indn).points : m_subfield.tracks.at(ind0).points;
    if(m_first_track_inverse)
        return Pose2D( track.back(),
                       geometry::get_angle( track.back(), r_at(track,1) ) );
    return Pose2D( track.front(),
                   geometry::get_angle( track.front(), track.at(1)) );

}

AroResp SimpleBaseRoutesPlanner::getRoute(int machine_id,
                                          Route &route,
                                          std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                          std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                          std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculatorTransit) {

    if(m_machinesMap.find(machine_id) == m_machinesMap.end()){
        route.route_points.clear();
        return AroResp(1, "The machine with id " + std::to_string(machine_id) + " is not part of the working group");
    }

    auto it_m = m_machines_routes.find(machine_id);
    if(it_m == m_machines_routes.end()) {//either no plan hos been computed, or this machine was added after computing the last plan --> plan (for the first time or again)
        if(m_machines_routes.empty())
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "No plan has been computed. Planning for all machines...");
        else
            logger().printOut(LogLevel::DEBUG, __FUNCTION__, "The computed plan might be outdated. Planning for all machines...");

        auto aroResp = plan(edgeMassCalculator, edgeSpeedCalculator, edgeSpeedCalculatorTransit);
        if( aroResp.isError() ){
            m_machines_routes.clear();
            route.route_points.clear();
            return aroResp;
        }
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Plan was obtained");

        it_m = m_machines_routes.find(machine_id);
        if(it_m == m_machines_routes.end()) {
            route.route_points.clear();
            return AroResp(1, "The machine with id " + std::to_string(machine_id) + " is not part of the working group");
        }
    }
    route = it_m->second;
    return AroResp::ok();
}


}
