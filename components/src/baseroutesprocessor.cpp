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
 
#include "arolib/components/baseroutesprocessor.h"

namespace arolib {

BaseRoutesProcessor::BaseRoutesProcessor(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

AroResp BaseRoutesProcessor::processRoutes(const Subfield& subfield,
                                                 const std::vector<arolib::HeadlandRoute> &baseRoutes_headland,
                                                 const std::vector<arolib::Route> &baseRoutes_infield,
                                                 const std::vector<arolib::Machine>& machines,
                                                 const Settings& settings,
                                                 std::vector<arolib::Route> &connectedRoutes,
                                                 std::vector<arolib::HeadlandRoute> *pProcessedBaseRoutes_headland,
                                                 std::vector<arolib::Route> *pProcessedBaseRoutes_infield) const
{

    std::vector<arolib::HeadlandRoute> processedBaseRoutes_headland_tmp;
    std::vector<arolib::Route> processedBaseRoutes_infield_tmp;

    std::vector<arolib::HeadlandRoute> & processedBaseRoutes_headland = pProcessedBaseRoutes_headland ? *pProcessedBaseRoutes_headland : processedBaseRoutes_headland_tmp;
    std::vector<arolib::Route> & processedBaseRoutes_infield = pProcessedBaseRoutes_infield ? *pProcessedBaseRoutes_infield : processedBaseRoutes_infield_tmp;

    connectedRoutes.clear();
    processedBaseRoutes_infield.clear();
    processedBaseRoutes_headland.clear();

    try {
        if (baseRoutes_infield.empty() && baseRoutes_headland.empty()) {
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Routes missing, needed by the planner" );
            return AroResp(1, "Routes missing, needed by the planner" );
        }

        processedBaseRoutes_infield = baseRoutes_infield;
        processedBaseRoutes_headland = baseRoutes_headland;

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Pre-processing base routes...");
        preProcessRoutes(processedBaseRoutes_infield,
                         processedBaseRoutes_headland);


        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Connecting the headland and infield base routes...");
        connectedRoutes = connectRoutes(processedBaseRoutes_infield,
                                        processedBaseRoutes_headland,
                                        subfield,
                                        machines);

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Removing base route points where there is nothing to process...");
        auto removedHarvPoints = removeInitialUselessPoints(connectedRoutes);

        if(connectedRoutes.empty()){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No base routes remain after pre-planning processing. Posible reason: there is nothing to harvest.");
            return AroResp(1, "No base routes remain after pre-planning processing. Posible reason: there is nothing to harvest." );
        }

        m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Finished processing routes");
        return AroResp(0, "OK" );

    }
    catch (std::exception &e) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what() );
        return AroResp(1, "Exception cought: ", e.what() );
    }

}

AroResp BaseRoutesProcessor::processRoutes(PlanningWorkspace &pw, size_t subfieldIdx, const BaseRoutesProcessor::Settings &settings) const
{
    if(subfieldIdx >= getField(pw).subfields.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfield index");
        return AroResp(1, "Invalid subfield index");
    }
    const auto &subfield = getField(pw).subfields.at(subfieldIdx);
    const auto &machines = getMachines(pw);
    const auto &baseRoutes_headland = getBaseRoutes_headland(pw)[subfieldIdx];
    const auto &baseRoutes_infield = getBaseRoutes_infield(pw)[subfieldIdx];
    auto &processedBaseRoutes_headland = getBaseRoutesProcessed_headland(pw)[subfieldIdx];
    auto &processedBaseRoutes_infield = getBaseRoutesProcessed_infield(pw)[subfieldIdx];
    auto &connectedRoutes = getConnectedBaseRoutes(pw)[subfieldIdx];

    m_planningWorkspace = &pw;
    m_pw_subfieldIdx = subfieldIdx;

    auto ret = processRoutes(subfield,
                             baseRoutes_headland,
                             baseRoutes_infield,
                             machines,
                             settings,
                             connectedRoutes,
                             &processedBaseRoutes_headland,
                             &processedBaseRoutes_infield);

    m_planningWorkspace = nullptr;

    return ret;

}


void BaseRoutesProcessor::preProcessRoutes(std::vector<Route> &baseRoutes_infield,
                                                std::vector<HeadlandRoute> &baseRoutes_headland) const
{

    std::map<MachineId_t, std::pair< size_t, size_t > > firstGoodIndexMap_HL; //map< machine_id , < good_index, route_index > >
    std::map<MachineId_t, std::pair< size_t, size_t > > firstGoodIndexMap_IF; //map< machine_id , < good_index, route_index > >

    //check if there is anything to harvest in the routes. if not, delete the route
    for(size_t r = 0 ; r < baseRoutes_headland.size() ; ++r){

        HeadlandRoute &route = baseRoutes_headland.at(r);
        size_t firstGoodIndex = route.route_points.size();

        //the base routes have been preprocessed already, and the timestamps of the initial segment/edge with nothing to harvest is = -1
        for(size_t i = 0 ; i < route.route_points.size() ; ++i){
            if(route.route_points.at(i).time_stamp >= -1e-6){
                firstGoodIndex = i;
                break;
            }
        }

        if(firstGoodIndex == route.route_points.size() ){//nothing to harvest in this route
            baseRoutes_headland.erase( baseRoutes_headland.begin() + r );
            --r;
            continue;
        }

        //adjust timestamps and harvested mass/volume
        auto rpRef = route.route_points.at(firstGoodIndex);
        for(size_t i = firstGoodIndex ; i < route.route_points.size() ; ++i){
            route.route_points.at(i).time_stamp -= rpRef.time_stamp;
            route.route_points.at(i).harvested_mass -= rpRef.harvested_mass;
            route.route_points.at(i).harvested_volume -= rpRef.harvested_volume;
        }

        firstGoodIndexMap_HL[route.machine_id] = std::make_pair(firstGoodIndex, r);

    }
    //check if there is anything to harvest in the routes. if not, delete the route
    for(size_t r = 0 ; r < baseRoutes_infield.size() ; ++r){
        Route &route = baseRoutes_infield.at(r);
        size_t firstGoodIndex = route.route_points.size();

        //the base routes have been preprocessed already, and the timestamps of the initial segment/edge with nothing to harvest is = -1
        for(size_t i = 0 ; i < route.route_points.size() ; ++i){
            if(route.route_points.at(i).time_stamp >= -0.00001){
                firstGoodIndex = i;
                break;
            }
        }
        if(firstGoodIndex == route.route_points.size() ){
            baseRoutes_infield.erase( baseRoutes_infield.begin() + r );
            --r;
            continue;
        }

        //adjust timestamps and harvested mass/volume
        auto rpRef = route.route_points.at(firstGoodIndex);
        for(size_t i = firstGoodIndex ; i < route.route_points.size() ; ++i){
            route.route_points.at(i).time_stamp -= rpRef.time_stamp;
            route.route_points.at(i).harvested_mass -= rpRef.harvested_mass;
            route.route_points.at(i).harvested_volume -= rpRef.harvested_volume;
        }

        firstGoodIndexMap_IF[route.machine_id] = std::make_pair(firstGoodIndex, r);

    }


}


std::vector<Route> BaseRoutesProcessor::connectRoutes(std::vector<Route> &baseRoutes_infield,
                                                           std::vector<HeadlandRoute> &baseRoutes_headland,
                                                           const Subfield &subfield,
                                                           const std::vector<Machine> &machines) const
{
    std::vector<Point> headland_points = subfield.headlands.complete.middle_track.points;

    std::vector<Route> routes;

    std::map<int, Machine> machinesMap;//for easier access
    for(auto &m : machines)
        machinesMap[m.id] = m;

    //check if there only exists headland OR inner-field routes (-> no connection is needed)
    if(baseRoutes_headland.empty()){
        routes = baseRoutes_infield;
        return routes;
    }
    if(baseRoutes_infield.empty()){
        routes.resize( baseRoutes_headland.size() );
        for(size_t i = 0 ; i < baseRoutes_headland.size(); ++i){
            routes[i].route_id = baseRoutes_headland[i].route_id;
            routes[i].machine_id = baseRoutes_headland[i].machine_id;
            routes[i].route_points = baseRoutes_headland[i].route_points;
        }
        return routes;
    }

    std::set<int> routeIdsIF, routeIdsIF_added;
    for(auto &ifr : baseRoutes_infield)//the base route id will be the ids of the infield routes. this is done so that, if there are headland routes with no corresponding infield routes to be connected, its id does not conflict with other routes (i.e. avoid repeated route ids)
        routeIdsIF.insert(ifr.route_id);

    //connect the headland routes with the inner-field routes of the corresponding machine
    for(auto &hlr : baseRoutes_headland){
        bool connected = false;
        Route route;
        route.route_id = hlr.route_id;
        route.machine_id = hlr.machine_id;
        route.route_points = hlr.route_points;
        auto it_machine = machinesMap.find(route.machine_id);
        if(it_machine != machinesMap.end()){
            Machine machine = it_machine->second;

            //find and connect with the inner-field route of the same machine
            for(auto &ifr : baseRoutes_infield){
                if(ifr.route_points.empty())
                    continue;
                if(route.machine_id == ifr.machine_id){//is the route from the same machine?
                    if(!route.route_points.empty()){
                        //points to be used for the connection (initially, the last point of the headland route and the first point of the infield route)
                        Point cp0 = route.route_points.back().point();
                        Point cpn = ifr.route_points.front().point();

                        //check if some extra points must be added to get a smoother conection (i.e. trying to perserve the direction of harvesting before/after the connection path)
                        bool extra0 = false, extran = false;
                        if(route.route_points.size() > 1
                                && route.route_points.back().type == RoutePoint::TRACK_END){
                            //extend the last segment of the headland route so that when the base finishes harvesting, it keeps the driving direction for a short distance (i.e avoid turns immediatly after finishing harvesting the headland).
                            double extDist = std::min(machine.working_width, arolib::geometry::calc_dist_to_linestring(headland_points, cp0));
                            cp0 = arolib::geometry::extend_line( r_at(route.route_points, 1), cp0, extDist );//update cp0
                            extra0 = true;
                        }
                        if(ifr.route_points.size() > 1){
                            //(reverse) extend the first segment of the infield route so that the last segment of the connection is in the same direction of the first segment in the infield route (i.e. avoid turns immediatly before starting harvesting the infield).
                            double extDist = std::min(machine.working_width, arolib::geometry::calc_dist_to_linestring(headland_points, cpn));
                            cpn = arolib::geometry::extend_line( ifr.route_points.at(1), cpn, extDist );//update cpn
                            extran = true;
                        }

                        Pose2D pose_start, pose_finish;
                        double speed_start = 0;
                        pose_start.point() = cp0;
                        if( route.route_points.size() > 1 ){
                            pose_start.angle = arolib::geometry::get_angle( r_at(route.route_points, 1), cp0 );
                            double delta_time = route.route_points.back().time_stamp - r_at(route.route_points, 1).time_stamp;
                            if( delta_time > 1e-9 )
                                speed_start = arolib::geometry::calc_dist( route.route_points.back(), r_at(route.route_points, 1) ) / delta_time;
                        }
                        pose_finish.point() = cpn;
                        if( ifr.route_points.size() > 1 )
                            pose_finish.angle = arolib::geometry::get_angle( cpn, ifr.route_points.at(1) );
                        std::vector<Point> connection = m_tracksConnector->getConnection(subfield,
                                                                                         machine,
                                                                                         pose_start,
                                                                                         pose_finish,
                                                                                         -1,
                                                                                         0,
                                                                                         speed_start,
                                                                                         -1);

                        if(connection.empty())//get the shortest connection between cp0 and cpn though the planned headland ('track' in the middle of the headland)
                            connection = arolib::geometry::getShortestGeometryPart(headland_points,
                                                                                   cp0,
                                                                                   cpn,
                                                                                   true);

                        //update the start and end of the connections in case extensions had to be performed
                        if(extra0){
                            if (connection.size() > 1 &&
                                    std::fabs( arolib::geometry::get_angle(connection.front(), connection.at(1), connection.front(), cp0) ) < M_PI_2 )
                                connection.front() = cp0;
                            else
                                push_front(connection, cp0);
                        }
                        if(extran){
                            if (connection.size() > 1 &&
                                    std::fabs( arolib::geometry::get_angle(connection.back(), r_at(connection,1), connection.back(), cpn) ) < M_PI_2 )
                                connection.back() = cpn;
                            else
                                connection.emplace_back(cpn);
                        }

                        //build the connection route-segment and add it to both the final (connected) route and the (original) headland route
                        for(auto &cp : connection){
                            if(cp == route.route_points.back().point())//do not add repeated points
                                continue;

                            RoutePoint rp;
                            rp.point() = cp;
                            rp.type = RoutePoint::TRANSIT;
                            rp.track_id = -1;
                            rp.bunker_mass = route.route_points.back().bunker_mass;
                            rp.bunker_volume = route.route_points.back().bunker_volume;
                            rp.harvested_mass = route.route_points.back().harvested_mass;
                            rp.harvested_volume = route.route_points.back().harvested_volume;
                            rp.time_stamp = route.route_points.back().time_stamp;

                            double machine_speed = machine.calcSpeed(rp.bunker_mass);
                            if(machine_speed > 0){
                                double dist = arolib::geometry::calc_dist(rp, route.route_points.back());
                                rp.time_stamp += dist/machine_speed;
                            }

                            route.route_points.push_back( rp );
                            hlr.route_points.push_back(rp);
                        }

                        //calculate the time that the infield route has to be shifted (i.e. its timestamps)
                        double dTime = 0;
                        {
                            double machine_speed = machine.calcSpeed(ifr.route_points.front().bunker_mass);
                            if(machine_speed > 0){
                                double dist = arolib::geometry::calc_dist(ifr.route_points.front(), route.route_points.back());
                                dTime = route.route_points.back().time_stamp + dist/machine_speed;
                            }
                        }

//                            for(auto &rp : ifr.route_points){
//                                route.route_points.push_back(rp);
//                                route.route_points.back().time_stamp += dTime;
//                            }

                        //apply the shift-time to the timestamps of the (original) infield route
                        for(auto &rp : ifr.route_points){
                            rp.time_stamp += dTime;
                            rp.harvested_mass += route.route_points.back().harvested_mass;
                            rp.harvested_volume += route.route_points.back().harvested_volume;
                        }

                        //add the adjusted infield route to the final (connected) route
                        route.route_points.insert( route.route_points.end(), ifr.route_points.begin(), ifr.route_points.end() );

                        route.route_id = ifr.route_id;
                        routeIdsIF_added.insert(ifr.route_id);
                        connected = true;
                    }
                }
            }
        }
        if(!connected){//check if the headland route was connected to its corresponding infield route
            while( routeIdsIF.find(route.route_id) != routeIdsIF.end() ){//check if the routes id must be changed not to be repeated, and change it if needed
                route.route_id++;
                hlr.route_id++;
            }
        }
        routes.push_back(route);
    }

    //add the remaining IF routes that had no corresponding headland route to be conected with
    for(auto ifr : baseRoutes_infield){
        if(ifr.route_points.empty())
            continue;
        if(routeIdsIF_added.find(ifr.route_id) != routeIdsIF_added.end())
            continue;
        routes.push_back(ifr);
    }

    //sort final routes by route id
    std::sort(routes.begin(), routes.end(), [](const Route & r1, const Route & r2) -> bool { return r1.route_id < r2.route_id; } );

    return routes;
}

size_t BaseRoutesProcessor::removeInitialUselessPoints(std::vector<Route> &baseRoutes) const
{
    size_t count = 0;
    for(auto &route : baseRoutes){
        for(size_t i = 0 ; i < route.route_points.size() ; ++i){
            if(route.route_points.at(i).time_stamp >= -0.0001){
                count += i;
                route.route_points.erase( route.route_points.begin(), route.route_points.begin()+i );
                break;
            }
        }
        //route.route_points.front().type = RoutePoint::TRACK_START;
    }
    return count;
}

void BaseRoutesProcessor::setHeadlandInfieldConnector(std::shared_ptr<IInfieldTracksConnector> tc)
{
    m_tracksConnector = tc;
}


}
