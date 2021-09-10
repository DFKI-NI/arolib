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
 
#include "arolib/planning/routeassembler.hpp"


namespace arolib{

RouteAssembler::RouteAssembler(RoutePoint::RoutePointType workingRPType, LogLevel logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_workingRPType(workingRPType)
{
    clear();
}

RouteAssembler::RouteAssembler(const Route &route,
                               RoutePoint::RoutePointType workingRPType,
                               LogLevel logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_route(route),
    m_workingRPType(workingRPType)
{

}

RouteAssembler::RouteAssembler(MachineId_t machine_id, int route_id, RoutePoint::RoutePointType workingRPType, LogLevel logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_workingRPType(workingRPType)
{
    m_route.machine_id = machine_id;
    m_route.route_id = route_id;
    m_route.route_points.clear();
}

void RouteAssembler::addTrack(const std::vector<Point> &_points,
                              int track_id,
                              bool reverse,
                              const Machine &machine,
                              IEdgeMassCalculator &edgeMassCalculator,
                              IEdgeSpeedCalculator &edgeSpeedCalculator)
{

    addTrack(Subfield(), _points, track_id, reverse, machine, edgeMassCalculator, edgeSpeedCalculator);

}

void RouteAssembler::addTrack(const Subfield &sf, const std::vector<Point> &_points, int track_id, bool reverse, const Machine &machine, IEdgeMassCalculator &edgeMassCalculator, IEdgeSpeedCalculator &edgeSpeedCalculator)
{
    double minSpeed = 0.01;//@TODO hardcoded. should we get this as a parameter?

    double maxSpeed = machine.max_speed_empty;
    if(machine.bunker_mass > 1e-5 || machine.bunker_volume > 1e-5)//@TODO tempotaty workaround. This should be computed based on the amount of yield in the bunker, but this is planned afterwards
        maxSpeed = 0.5 * (machine.max_speed_empty + machine.max_speed_full);

    double defSpeed = (machine.def_working_speed > 0 ? machine.def_working_speed : maxSpeed);

    std::vector<Point> points = _points;

    if(reverse)
        std::reverse(points.begin(),points.end());

    std::set<Point> pointsOutside;
    if(sf.boundary_outer.points.size() > 3 && sf.boundary_inner.points.size() > 3)
        adjustTracksPointsBasedOnBoundaries(sf, points, machine, pointsOutside);

    for (size_t i = 0; i < points.size(); ++i) {
        RoutePoint rp;
        rp.point() = points.at(i);
        rp.bunker_mass = 0.0;//@TODO should we update the bunker_mass of the harvester routepoints when the harvester has a bunker?
        if (m_route.route_points.empty()) {
            rp.time_stamp = -1;//set to zero if necessary in the next step
            rp.harvested_mass = 0.0;
            rp.harvested_volume = 0.0;
        }
        else {
            double calcSpeed = defSpeed;
            double mass = 0;
            if(pointsOutside.find( m_route.route_points.back().point() ) != pointsOutside.end()
                    || pointsOutside.find( rp.point() ) != pointsOutside.end()){//do not add mass

                calcSpeed = edgeSpeedCalculator.calcSpeed(m_route.route_points.back(),
                                                          rp,
                                                          0,
                                                          machine);
                if (calcSpeed > maxSpeed)
                    calcSpeed = maxSpeed;
                if (calcSpeed < minSpeed){
                    m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Calculated harvester speed in track " + std::to_string(track_id)
                                       + " (index = " + std::to_string(i) + ") is lower than " + std::to_string(minSpeed) + " m/s");
                    calcSpeed = minSpeed;
                }
            }
            else if ( m_route.route_points.back().type != RoutePoint::HEADLAND ){//between the last point in the route and this point, some yield might be harvested

                mass = edgeMassCalculator.calcMass(m_route.route_points.back(),
                                                     rp,
                                                     m_machine.working_width);

                calcSpeed = edgeSpeedCalculator.calcSpeed(m_route.route_points.back(),
                                                          rp,
                                                          0,
                                                          machine);
                if (calcSpeed > maxSpeed)
                    calcSpeed = maxSpeed;
                if (calcSpeed < minSpeed){
                    m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Calculated harvester speed in track " + std::to_string(track_id)
                                       + " (index = " + std::to_string(i) + ") is lower than " + std::to_string(minSpeed) + " m/s");
                    calcSpeed = minSpeed;
                }

            }
            else{//add (to the current route) a connection between the last headland point to the first point to be added with the same sample resolution as the last headland-connection path
                double hlRes = -1;
                double hlVel = -1;
                for(size_t j = 0 ; j+1 < m_route.route_points.size() ; ++j){
                    const auto& p0 = *( m_route.route_points.end()-2-j );
                    const auto& p1 = *( m_route.route_points.end()-1-j );
                    if(p1.type != RoutePoint::HEADLAND)
                        break;
                    if(hlRes < arolib::geometry::calc_dist(p0, p1)){
                        hlRes = arolib::geometry::calc_dist(p0, p1);
                        hlVel = hlRes / (p1.time_stamp - p0.time_stamp);
                    }
                }
                if(hlRes > 1e-5){
                    std::vector<Point> connPoints = {m_route.route_points.back().point(), rp.point()};
                    connPoints = arolib::geometry::sample_geometry(connPoints, hlRes);
                    if(hlVel < 1e-5)
                        hlVel = maxSpeed;
                    for(size_t j = 1 ; j+1 < connPoints.size() ; ++j){
                        RoutePoint hl_rp = m_route.route_points.back();
                        hl_rp.point() = connPoints.at(j);
                        hl_rp.time_stamp += arolib::geometry::calc_dist( m_route.route_points.back(), hl_rp ) / hlVel;
                        m_route.route_points.push_back(hl_rp);
                    }
                    calcSpeed = hlVel;
                }
            }

            if(mass < 1e-5 && m_route.route_points.back().time_stamp < 0)
                rp.time_stamp = -1; //the route points will have a timestamp of -1 until some yield to be harvested is found
            else{
                if(m_route.route_points.back().time_stamp < 0)
                    m_route.route_points.back().time_stamp = 0;

                double segment_length = arolib::geometry::calc_dist(m_route.route_points.back(), rp);
                rp.time_stamp = m_route.route_points.back().time_stamp + segment_length / calcSpeed;
            }

            rp.harvested_mass = m_route.route_points.back().harvested_mass;
            rp.harvested_volume = m_route.route_points.back().harvested_volume;
            if(i != 0){
                rp.harvested_mass += mass;
                rp.harvested_volume += 0.0;//@TODO
            }

        }


        rp.type = m_workingRPType;
        if(reverse == false) {
            rp.track_idx = i;
        } else {
            rp.track_idx = points.size() - i;
        }
        if(i == 0/* && track_id > 0*/) {
            rp.type = RoutePoint::TRACK_START;
        } else if(i == points.size() - 1/* && track_id > 0*/) {
            rp.type = RoutePoint::TRACK_END;
        }
        rp.track_id = track_id;
        m_route.route_points.push_back(rp);
    }

}

void RouteAssembler::addTrack(double speed,
                              double default_yield,
                              const std::vector<Point> &_points,
                              int track_id,
                              bool reverse,
                              const Polygon &boundary)
{
    std::vector<Point> points = _points;
    if (speed < 0.01) {
        throw std::invalid_argument("Harvester speed must not be < 0.01");
    }
    if(reverse) {
        std::reverse(points.begin(),points.end());
    }
    for (size_t i = 0; i < points.size(); ++i) {
        RoutePoint rp;
        rp.point() = points.at(i);
        rp.bunker_mass = 0.0;//@TODO should we update the bunker_mass of the harvester routepoints when the harvester has a bunker?
        if (m_route.route_points.size() > 0) {

            if(m_route.route_points.back().type == RoutePoint::HEADLAND){//add (to the current route) a connection between the last headland point to the first point to be added with the same sample resolution as the last headland-connection path
                double hlRes = -1;
                double hlVel = -1;
                for(size_t j = 0 ; j+1 < m_route.route_points.size() ; ++j){
                    const auto& p0 = *( m_route.route_points.end()-2-j );
                    const auto& p1 = *( m_route.route_points.end()-1-j );
                    if(p1.type != RoutePoint::HEADLAND)
                        break;
                    if(hlRes < arolib::geometry::calc_dist(p0, p1)){
                        hlRes = arolib::geometry::calc_dist(p0, p1);
                        hlVel = hlRes / (p1.time_stamp - p0.time_stamp);
                    }
                }
                if(hlRes > 1e-5){
                    std::vector<Point> connPoints = {m_route.route_points.back().point(), rp.point()};
                    connPoints = arolib::geometry::sample_geometry(connPoints, hlRes);
                    if(hlVel < 1e-5)
                        hlVel = speed;
                    for(size_t j = 1 ; j+1 < connPoints.size() ; ++j){
                        RoutePoint hl_rp = m_route.route_points.back();
                        hl_rp.point() = connPoints.at(j);
                        hl_rp.time_stamp += arolib::geometry::calc_dist( m_route.route_points.back(), hl_rp ) / hlVel;
                        m_route.route_points.push_back(hl_rp);
                    }
                }
            }

            double segment_length = arolib::geometry::calc_dist(m_route.route_points.back(), rp);
            rp.time_stamp = m_route.route_points.back().time_stamp + segment_length / speed;//@TODO: or should we use the default or (input)

            rp.harvested_mass = m_route.route_points.back().harvested_mass;
            rp.harvested_volume = m_route.route_points.back().harvested_volume;
            if(i != 0){
                double edgeMult = 1;//for the cases where part of the edge lies outside of the boundary
                if (boundary.points.size() > 2 && m_machine.working_width > 1e-6){
                    double dist = std::min( m_machine.working_width , arolib::geometry::calc_dist(rp, m_route.route_points.back()) );
                    Polygon edge = arolib::geometry::createRectangleFromLine(rp, m_route.route_points.back(), m_machine.working_width);
                    double edgeArea = segment_length * m_machine.working_width;
                    auto intersections = arolib::geometry::get_likely_intersection(boundary, edge, dist * 1e-3);
                    double areaTotal = 0;
                    for(auto &intersection : intersections)
                        areaTotal += std::fabs( arolib::geometry::calc_area(intersection) );
                    edgeMult = std::min(1.0, areaTotal / edgeArea);
                }

                rp.harvested_mass += t_ha2Kg_sqrm(default_yield * edgeMult) * segment_length * m_machine.working_width; //now in Kg;
                rp.harvested_volume += 0.0;//@TODO
            }

        } else {
            rp.time_stamp = 0.0;
            rp.harvested_mass = 0.0;
            rp.harvested_volume = 0.0;
        }
        rp.type = m_workingRPType;
        if(reverse == false) {
            rp.track_idx = i;
        } else {
            rp.track_idx = points.size() - i;
        }
        if(i == 0/* && track_id > 0*/) {
            rp.type = RoutePoint::TRACK_START;
        } else if(i == points.size() - 1/* && track_id > 0*/) {
            rp.type = RoutePoint::TRACK_END;
        }
        rp.track_id = track_id;
        m_route.route_points.push_back(rp);
    }

}

void RouteAssembler::addTrack(double speed,
                              double default_yield,
                              const Track &l,
                              int track_id,
                              bool reverse,
                              const Polygon &boundary) {
    return addTrack(speed, default_yield, l.points, track_id, reverse, boundary);
}

bool RouteAssembler::addConnectionToTrack(const Track &nextTrack,
                                          const Subfield &sf,
                                          double speed,
                                          bool checkBothSides,
                                          const Polygon& limitBoundary)
{
    if (speed < 1e-9) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid speed");
        return false;
    }
    if (nextTrack.points.size() < 2) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid track");
        return false;
    }
    if (m_route.route_points.size() < 2) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Current route does not have enough points");
        return false;
    }

    Pose2D pose_start;
    pose_start.point() = m_route.route_points.back().point();
    pose_start.angle = arolib::geometry::get_angle( r_at(m_route.route_points, 1 ), m_route.route_points.back() );

    double delta_time = m_route.route_points.back().time_stamp - r_at(m_route.route_points, 1).time_stamp;
    double speed_start = std::fabs(delta_time) > 1e-9 ? arolib::geometry::calc_dist( r_at(m_route.route_points, 1 ), m_route.route_points.back() ) / delta_time  : 0;

    auto connection = m_tracksConnector->getConnection(m_machine,
                                                       pose_start,
                                                       nextTrack.points,
                                                       checkBothSides,
                                                       -1,
                                                       std::max(0.0, 0.5*m_machine.working_width),
                                                       limitBoundary,
                                                       sf.boundary_inner,
                                                       sf.headlands,
                                                       speed_start);

    if(connection.empty())
        return false;

    m_route.route_points.reserve( m_route.route_points.size() + connection.size() - 2 );
    for (int i = 1; i+1 < connection.size(); ++i) {
        RoutePoint rp;
        rp.point() = connection.at(i);
        rp.type = RoutePoint::HEADLAND;
        rp.track_idx = -1;
        rp.track_id = -1;

        const RoutePoint &rp_prev = m_route.route_points.back();
        rp.bunker_mass = rp_prev.bunker_mass;
        rp.bunker_volume = rp_prev.bunker_volume;
        rp.harvested_mass = rp_prev.harvested_mass;
        rp.harvested_volume = rp_prev.harvested_volume;
        double segment_length = arolib::geometry::calc_dist(rp_prev, rp);
        rp.time_stamp = rp_prev.time_stamp + segment_length / speed;

        m_route.route_points.push_back( rp );
    }


    return true;

}

bool RouteAssembler::addHeadlandPart(double speed,
                                     std::vector<Point> headland_part,
                                     const Track &nextTrack,
                                     size_t headland_size,
                                     size_t start_index,
                                     bool reverseIndexes)
{
    if (speed < 0.01) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid speed");
        return false;
    }
    if (headland_part.empty()) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No headland points given");
        return false;
    }

    int mult = 1;
    if(reverseIndexes)
        mult = -1;

    double hlRes = -1;
    for(size_t j = 0 ; j+1 < headland_part.size() ; ++j)
        hlRes = std::max( hlRes, arolib::geometry::calc_dist( headland_part.at(j) , headland_part.at(j+1) ) );

    if(hlRes > 1e-5){
        headland_part.insert( headland_part.begin(), m_route.route_points.back().point() );
        headland_part = arolib::geometry::sample_geometry(headland_part, hlRes, hlRes*0.25);
        headland_part.erase( headland_part.begin() );
    }

    for (int i = 0; i < headland_part.size(); ++i) {
        RoutePoint rp;

        rp.point() = headland_part.at(i);
        rp.type = RoutePoint::HEADLAND;
        rp.track_idx = (start_index + i*mult + headland_size)%headland_size;
        rp.track_id = -1; //@TODO or should we keep the original track_id?
        //rp.track_id = headland_part.at(i).track_id;

        if(m_route.route_points.empty()){
            rp.time_stamp
                    = rp.bunker_mass
                    = rp.bunker_volume
                    = rp.harvested_mass
                    = rp.harvested_volume = 0;
        }
        else{
            const RoutePoint &rp_last = m_route.route_points.back();
            rp.bunker_mass = rp_last.bunker_mass;
            rp.bunker_volume = rp_last.bunker_volume;
            rp.harvested_mass = rp_last.harvested_mass;
            rp.harvested_volume = rp_last.harvested_volume;
            double segment_length = arolib::geometry::calc_dist(rp_last, rp);
            rp.time_stamp = rp_last.time_stamp + segment_length / speed;
        }

        m_route.route_points.push_back(rp);
    }
    return true;

}


bool RouteAssembler::getClosestPointInNextTrack(const Subfield &sf,
                                                const Track& nextTrack,
                                                Point &p_out)
{
    if(nextTrack.points.empty()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Next track is empty");
        return false;
    }

    if(m_route.route_points.empty()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Route is currently empty");
        return false;
    }

    if(m_route.route_points.back().type != RoutePoint::TRACK_END){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Last route point in route is not a track end");
        return false;
    }

    //get all possible conections (through the headland track) between the last (infield track) routepoint in the current route, to the first and last point of the given track, and select the point corresponding to the shortest path
    //  * possible connection 1: connecting to the first point of the next track in forward (headland track) direction
    //  * possible connection 2: connecting to the first point of the next track in backwards (headland track) direction
    //  * possible connection 1: connecting to the last point of the next track in forward (headland track) direction
    //  * possible connection 2: connecting to the last point of the next track in backwards (headland track) direction

    const Point& p0 = m_route.route_points.back().point();

    std::vector<Point> h_points = sf.headlands.complete.middle_track.points;

    int start_index = geometry::addSampleToGeometryClosestToPoint(h_points, p0, 1);
    if(start_index < 0 || start_index >= h_points.size() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error getting headland point corresponding to the start point");
        return false;
    }

    size_t sizePrev = h_points.size();
    int end_index__start = geometry::addSampleToGeometryClosestToPoint(h_points, nextTrack.points.front(), 1);
    if(end_index__start < 0 || end_index__start >= h_points.size() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error getting headland point corresponding to the next point");
        return false;
    }
    if(h_points.size() > sizePrev && end_index__start <= start_index)
        start_index++;

    sizePrev = h_points.size();
    int end_index__end = geometry::addSampleToGeometryClosestToPoint(h_points, nextTrack.points.front(), 1);
    if(end_index__end < 0 || end_index__end >= h_points.size() ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error getting headland point corresponding to the next point");
        return false;
    }
    if(h_points.size() > sizePrev && end_index__end <= start_index)
        start_index++;
    if(h_points.size() > sizePrev && end_index__end <= end_index__start)
        end_index__start++;

    std::vector<Point> part__start = arolib::geometry::getGeometryPart(h_points, start_index, end_index__start);
    std::vector<Point> part_reverse__start = arolib::geometry::getGeometryPart(h_points, end_index__start, start_index);
    std::vector<Point> part__end = arolib::geometry::getGeometryPart(h_points, start_index, end_index__end);
    std::vector<Point> part_reverse__end = arolib::geometry::getGeometryPart(h_points, end_index__end, start_index);

    double l_start = std::min(arolib::geometry::getGeometryLength(part__start), arolib::geometry::getGeometryLength(part_reverse__start));
    double l_end = std::min(arolib::geometry::getGeometryLength(part__end), arolib::geometry::getGeometryLength(part_reverse__end));

    if(l_start < l_end)
        p_out = nextTrack.points.front();
    else
        p_out = nextTrack.points.back();

    return true;
}

void RouteAssembler::clear()
{
    m_route.machine_id = -1;
    m_route.route_id = -1;
    m_route.route_points.clear();

}

RoutePoint RouteAssembler::getRoutePoint(size_t index) const {
    return m_route.route_points.at(index);
}

void RouteAssembler::setIDs(int machine_id, int route_id)
{
    m_route.machine_id = machine_id;
    m_route.route_id = route_id;
}

bool RouteAssembler::setMachine(const Machine &m)
{
    if(m.working_width <= 0)
        return false;
    m_machine = m;
    return true;
}

void RouteAssembler::setTracksConnector(std::shared_ptr<IInfieldTracksConnector> tc)
{
    m_tracksConnector = tc;
}

std::vector<Point> RouteAssembler::adjustHeadlandPoints(const std::vector<Point> &hl_points,
                                                        const std::vector<Point> &prevPoints,
                                                        const std::vector<Point> &nextPoints){
    if(hl_points.size() < 2)
        return hl_points;
    auto ret = hl_points;

    //This method assumes that the hl_points were obtained from headland-points that were created following a MIN_DIST_TO_HEADLAND_SEGMENT strategy
    //The way to see if there is a better way to connect a segment (prevPoints or nextPoints) is to see if when the machine drives in the same direction of the segment, it intersects the hl_points linestring. This direct connection (to the intersection point) will therefore be shorter than using the original hl_points.

    //check if there is a shorter way to connect the first (previous) segment. If so, adjust the connection points accordingly
    if(prevPoints.size() > 1 && prevPoints.front() != prevPoints.at(1)){
        if(!nextPoints.empty())
            ret.emplace_back(nextPoints.front());
        std::vector<Point> intersections;
        if( !arolib::geometry::is_line(std::vector<Point>{prevPoints.front(), prevPoints.at(1), ret.front()}) ){
            Point p_ext = arolib::geometry::extend_line(prevPoints.front(), prevPoints.at(1), 1000);
            intersections = arolib::geometry::get_intersection(ret, prevPoints.front(), p_ext);
        }
        if( !intersections.empty() ){
            auto& p_int = intersections.back();
            size_t ind = 0;
            for( ; ind+1 < ret.size() ; ++ind ){
                if( arolib::geometry::is_line( std::vector<Point>{ ret.at(ind), p_int , ret.at(ind+1) } )
                        && arolib::geometry::getLocationInLine(ret.at(ind), ret.at(ind+1), intersections.back()) == 0
                        && ret.at(ind+1) != intersections.back())
                    break;
            }
            if(ind+1 < ret.size()){
                ret.erase( ret.begin(), ret.begin()+ind );
                ret.front() = p_int;
            }
        }
        if(!nextPoints.empty())
            ret.pop_back();
    }
    if(ret.size() < 2)//if the connection has only one point, it wouldn't be a very smooth connection to follow by a machine --> leave the connection as it was originally
        return hl_points;

    if (m_machine.working_width > 1e-3 && arolib::geometry::getGeometryLength(ret) < m_machine.working_width*0.99 )//if the resulting connection is too short, leave the connection as it was originally
        return hl_points;

    //check if there is a shorter way to connect the second (next) segment. If so, adjust the connection points accordingly
    if(nextPoints.size() > 1 && nextPoints.front() != nextPoints.at(1)){
        if(!prevPoints.empty()){
            if(prevPoints.size() == 1)
                ret.insert(ret.begin(), prevPoints.front());
            else
                ret.insert(ret.begin(), prevPoints.at(1));
        }
        std::vector<Point> intersections;
        if( !arolib::geometry::is_line(std::vector<Point>{nextPoints.at(1), nextPoints.front(), *(ret.end()-2)}) ){
            Point p_ext = arolib::geometry::extend_line(nextPoints.at(1), nextPoints.front(), 1000);
            intersections = arolib::geometry::get_intersection(ret, nextPoints.front(), p_ext, false);
        }
        if( !intersections.empty() ){
            auto& p_int = intersections.back();
            size_t ind = 0;
            for( ; ind+1 < ret.size() ; ++ind ){
                if( arolib::geometry::is_line( std::vector<Point>{ *(ret.end()-1-ind), p_int , *(ret.end()-2-ind) } )
                        && arolib::geometry::getLocationInLine(*(ret.end()-1-ind), *(ret.end()-2-ind), *(ret.end()-2-ind)) == 0
                        && *(ret.end()-2-ind) != intersections.back())
                    break;
            }
            if(ind+1 < ret.size()){
                ret.erase( ret.end()-ind, ret.end() );
                ret.back() = p_int;
            }
        }
        if(!prevPoints.empty())
            ret.erase(ret.begin());
    }
    if(ret.size() < 2)//if the connection has only one point, it wouldn't be a very smooth connection to follow by a machine --> leave the connection as it was originally
        return hl_points;

    if (m_machine.working_width > 1e-3 && arolib::geometry::getGeometryLength(ret) < m_machine.working_width*0.6)//if the resulting connection is too short, leave the connection as it was originally
        return hl_points;

    return ret;
}

bool RouteAssembler::adjustTracksPointsBasedOnBoundaries(const Subfield &sf, std::vector<Point> &points, const Machine& machine, std::set<Point>& pointsOutside)
{
    //@todo: pending tests!

    pointsOutside.clear();
    if(sf.boundary_outer.points.size() > 3 && sf.boundary_inner.points.size() > 3){

        auto intersection_points = geometry::get_intersection(points, sf.boundary_outer.points);

        size_t ind_ref_int = 0;
        for(size_t i = 0 ; i < intersection_points.size() ; i++){
            if(intersection_points.at(i) == points.front() || intersection_points.at(i) == points.back()){
                intersection_points.erase(intersection_points.begin()+i);
                --i;
            }
        }
        while(ind_ref_int+1 < intersection_points.size()){
            auto points_0 = points;
            auto points_1 = points;
            int ind_0_ob = geometry::addSampleToGeometryClosestToPoint(points_0, intersection_points.at(ind_ref_int), 1);
            int ind_1_ob = geometry::addSampleToGeometryClosestToPoint(points_1, intersection_points.at(ind_ref_int+1), 1);
            ind_ref_int += 2;
            if(ind_0_ob <= 0 || ind_0_ob >= points_0.size())
                continue;
            if(ind_1_ob <= 0 || ind_1_ob >= points_1.size())
                continue;
            points_0.erase( points_0.begin() + ind_0_ob + 1, points_0.end());
            points_1.erase( points_1.begin(), points_1.begin()+ ind_1_ob-1);

            int ind_0_ib = points_0.size()-1;
            double width = std::max(machine.working_width, machine.working_width);
            if(width < 0)
                width = 0.1;
            while (ind_0_ib >= 0){
                Polygon rectTmp = geometry::createRectangleFromLine( points_0.at(ind_0_ib-1), points_0.at(ind_0_ib), width );
                if(geometry::intersects(rectTmp.points, sf.boundary_inner))
                    break;
                --ind_0_ib;
            }
            int ind_1_ib = 0;
            while (ind_1_ib+1 < points_1.size() ){
                Polygon rectTmp = geometry::createRectangleFromLine( points_1.at(ind_1_ib), points_1.at(ind_1_ib+1), width );
                if(geometry::intersects(rectTmp.points, sf.boundary_inner))
                    break;
                ++ind_1_ib;
            }
            if( ind_0_ib <= 0 || ind_1_ib >= points_1.size()-1 )
                continue;

            Pose2D poseStart( points_0.at(ind_0_ib), geometry::get_angle(points_0.at(ind_0_ib-1), points_0.at(ind_0_ib)) );
            Pose2D poseFinish( points_1.at(ind_1_ib), geometry::get_angle(points_1.at(ind_1_ib), points_1.at(ind_1_ib+1)) );
            auto connection = m_tracksConnector->getConnection(sf,
                                                               machine,
                                                               poseStart,
                                                               poseFinish);
            if(connection.size() < 2)
                continue;

            points.clear();
            points.insert(points.end(), points_0.begin(), points_0.begin()+ind_0_ib+1);
            points.insert(points.end(), connection.begin()+1, connection.end()-1);
            points.insert(points.end(), points_1.begin()+ind_1_ib, points_1.end());

            pointsOutside.insert( connection.begin()+1, connection.end()-1 );
        }
    }
    return true;
}

}
