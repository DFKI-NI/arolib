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
 
#include "arolib/planning/simpleBaseRoutesPlanner.hpp"
#include "arolib/geometry/geometry_helper.hpp"

namespace arolib{
SimpleBaseRoutesPlanner::SimpleBaseRoutesPlanner(const LogLevel &logLevel) :
    LoggingComponent(logLevel, __FUNCTION__),
    m_has_subfield (false),
    m_has_machine (false)
{
}

void SimpleBaseRoutesPlanner::setSubfield(Subfield subfield) {
    m_subfield = subfield;
    m_has_subfield = true;
}

void SimpleBaseRoutesPlanner::addMachine(Machine machine) {
    m_machines[machine.id] = machine;
    m_has_machine = true;
    m_track_sequencer.addMachine(machine);
}

bool SimpleBaseRoutesPlanner::plan(IEdgeMassCalculator &edgeMassCalculator, IEdgeSpeedCalculator &edgeSpeedCalculator) {
    if(!m_has_subfield || !m_has_machine)
        return false;

    m_track_sequencer.setSubfield(m_subfield);
    m_track_sequencer.computeSequence();//compute the tracks assigned to each machine and the sequence in which the machines should drive them

    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Sequence computed!");

    std::vector< RouteAssembler > routeAss_v;
    for(size_t i = 0 ; i < m_track_sequencer.getNumberMachines() ; ++i){
        int id = m_track_sequencer.getID(i);
        Machine machine = m_machines[id];
        routeAss_v.emplace_back( RouteAssembler(RoutePoint::getDefaultRPType(machine), m_logger.logLevel()) );
        routeAss_v.back().logger().setParent(&m_logger);
    }

    const double limBoundaryDist_turningRadFactor = 0.1;//@todo we should either put this as a setting of the planner or simple not allow transit outside the subfield boundary
    std::map<MachineId_t, Polygon> limitBoundaries;
    for (int j = 0; j < m_track_sequencer.getNumberMachines(); j++) {//generate the routes for each machine based on the assigned tracks and their sequence

        bool invPoints = m_first_track_inverse;//flag used to know if the points of the next track should be added in reverse order

        //get the sequence
        int id = m_track_sequencer.getID(j);
        std::vector<int> sequence = m_track_sequencer.getSequence(id);

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Sequence gotten for machine " + std::to_string(j)
                                                         + " ( " + std::to_string(sequence.size()) + " tracks )");

        Machine machine = m_machines[id];

        const Polygon *limitBoundary = &m_subfield.boundary_outer;//limit boundary used for inter-track connections
        if(limBoundaryDist_turningRadFactor > 1e-9){
            if( limitBoundaries.find( machine.id ) == limitBoundaries.end() ){
                Polygon limitBoundaryTmp;
                if( arolib::geometry::offsetPolygon(m_subfield.boundary_outer, limitBoundaryTmp, limBoundaryDist_turningRadFactor * machine.getTurningRadius(), true, 0) )
                    limitBoundaries[machine.id] = limitBoundaryTmp;
                else
                    limitBoundaries[machine.id] = m_subfield.boundary_outer;
            }
            limitBoundary = &(limitBoundaries[machine.id]);
        }

        //get and initialize the route assembler for this machine/route
        RouteAssembler& routeAss = routeAss_v.at(j);
        routeAss.setMachine( machine );
        routeAss.setIDs(id, j);

        double machine_def_speed = machine.def_working_speed;

        //build the route by adding all assigned tracks in the given order, adding the corresponding inter-track (headland) connections in between
        for (int k = 0; k < sequence.size(); ++k) {

            auto l = m_subfield.tracks.at(sequence.at(k));//get a copy of the next track in the sequence

            if(invPoints)//reverse the track points if necesary
                std::reverse(l.points.begin(), l.points.end());

            bool reverse = false;//flag stating if the points of the current track should be added in reverse order
            if(routeAss.getRouteSize() > 0) {//the route has already points --> check if the points of the current track should be added in reverse order
                double distFirst = arolib::geometry::calc_dist(routeAss.getRoutePoint( routeAss.getRouteSize()-1 ), l.points.front());
                double distLast  = arolib::geometry::calc_dist(routeAss.getRoutePoint( routeAss.getRouteSize()-1 ), l.points.back());
                if (distFirst > distLast) {//the last point of the track is closer to the last point in the route than the first point of the track --> add track points in reverse order (i.e. the route will be connected to the last point in the track)
                    reverse = true;
                }
            }

            routeAss.addTrack(m_subfield,
                              l.points,
                              sequence.at(k),
                              reverse,
                              machine,
                              edgeMassCalculator,
                              edgeSpeedCalculator);

            m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Track " + std::to_string(k)
                                                              + " of sequence " + std::to_string(j) + " added.");

            //add the inter-track (headland) connection, i.e. the segment that connects the current route's end with the next track
            if(k+1 < sequence.size()) {//add the connection only if this is not the last track to be added
                Point next_track_point;
                if(reverse) {//the last track was added in reverse order --> the new track should be adde in forward order (iif TrackConnectionStrategy!=MEANDER)
                    next_track_point = m_subfield.tracks.at(sequence.at(k+1)).points.at(0);
                } else {
                    next_track_point = m_subfield.tracks.at(sequence.at(k+1)).points.back();
                }
                //note: the next_track_point is not used if the TrackConnectionStrategy=MEANDER (in this case, the best point of the next track is automatically selected)


                if(! addHeadlandConnection(routeAss,
                                           sequence.at(k+1),
                                           !reverse, //the last track was added in reverse order --> the new track should be adde in forward order (for TrackConnectionStrategy!=MEANDER)
                                           machine_def_speed,
                                           *limitBoundary) )
                    return false;

                m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Headland part of track " + std::to_string(k)
                                                                  + " of sequence " + std::to_string(j) + " added.");
            }
        }
        m_machines_routes[id] = routeAss.getRoute();//retrieve the routes
    }

    return true;
}

bool SimpleBaseRoutesPlanner::plan(PlanningWorkspace &pw, IEdgeMassCalculator &edgeMassCalculator, IEdgeSpeedCalculator &edgeSpeedCalculator)
{
    m_calcGridValueOption = CALC_PW;
    m_planningWorkspace = &pw;

    bool ret = plan(edgeMassCalculator, edgeSpeedCalculator);

    m_planningWorkspace = nullptr;
    m_calcGridValueOption = CALC_DIRECT;
    return ret;
}


bool SimpleBaseRoutesPlanner::addHeadlandConnection(RouteAssembler &routeAss,
                                                    int nextTrackId,
                                                    bool reverseTrack,
                                                    double speed,
                                                    const Polygon& limitBoundary)
{
    //get the index of the corresponding next track
    int indNextTrack = -1;
    for(size_t i = 0 ; i < m_subfield.tracks.size() ; ++i){
        if(m_subfield.tracks.at(i).id == nextTrackId){
            indNextTrack = i;
            break;
        }
    }
    if(indNextTrack < 0){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Track with given id not found");
        return false;
    }

    Track track = m_subfield.tracks.at(indNextTrack);

    if( m_track_sequencer.getStrategy() != SimpleTrackSequencer::MEANDER && reverseTrack )
        std::reverse(track.points.begin(), track.points.end());

    return routeAss.addConnectionToTrack(track,
                                         m_subfield,
                                         speed,
                                         m_track_sequencer.getStrategy() == SimpleTrackSequencer::MEANDER,
                                         limitBoundary);

}

bool SimpleBaseRoutesPlanner::getRoute(int machine_id,
                                       Route &route,
                                       IEdgeMassCalculator & edgeMassCalculator,
                                       IEdgeSpeedCalculator & edgeSpeedCalculator) {

    if(m_machines.find(machine_id) == m_machines.end()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, 10, "The machine with id ", machine_id, " is not part of the working group");
        route.route_points.clear();
        return false;
    }

    auto it_m = m_machines_routes.find(machine_id);
    if(it_m == m_machines_routes.end()) {//either no plan hos been computed, or this machine was added after computing the last plan --> plan (for the first time or again)
        if(m_machines_routes.empty())
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "No plan has been computed. Planning for all harvesters...");
        else
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "The computed plan might be outdated. Planning for all harvesters...");

        if( !plan(edgeMassCalculator, edgeSpeedCalculator) ){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error planning harvester route(s)");
            m_machines_routes.clear();
            route.route_points.clear();
            return false;
        }
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Plan was obtained");

        it_m = m_machines_routes.find(machine_id);
        if(it_m == m_machines_routes.end()) {
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, 10, "No route was planned for the machine with id ", machine_id);
            route.route_points.clear();
            return false;
        }
    }
    route = it_m->second;
    return true;
}

bool SimpleBaseRoutesPlanner::getRoute(int machine_id,
                                       PlanningWorkspace &pw,
                                       Route &route,
                                       IEdgeMassCalculator & edgeMassCalculator,
                                       IEdgeSpeedCalculator & edgeSpeedCalculator)
{
    if(m_machines.find(machine_id) == m_machines.end()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, 10, "The machine with id ", machine_id, " is not part of the working group");
        route.route_points.clear();
        return false;
    }

    auto it_m = m_machines_routes.find(machine_id);
    if(it_m == m_machines_routes.end()) {//either no plan hos been computed, or this machine was added after computing the last plan --> plan (for the first time or again)
        if(m_machines_routes.empty())
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "No plan has been computed. Planning for all harvesters...");
        else
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "The computed plan might be outdated. Planning for all harvesters...");

        if( !plan(pw, edgeMassCalculator, edgeSpeedCalculator) ){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error planning harvester route(s)");
            m_machines_routes.clear();
            return false;
        }
        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Plan was obtained");

        it_m = m_machines_routes.find(machine_id);
        if(it_m == m_machines_routes.end()) {
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, 10, "No route was planned for the machine with id ", machine_id);
            route.route_points.clear();
            return false;
        }
    }
    route = it_m->second;
    return true;
}

}
