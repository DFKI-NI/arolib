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
 
#include "arolib/planning/routeassembler.hpp"


namespace arolib{

RouteAssembler::RouteAssembler(const Machine& machine, int route_id, RoutePoint::RoutePointType workingRPType, LogLevel logLevel):
    LoggingComponent(logLevel, __FUNCTION__),
    m_machine(machine),
    m_workingRPType(workingRPType)
{
    m_route.machine_id = m_machine.id;
    m_route.route_id = route_id;
    m_route.route_points.clear();

    m_tracksConnector->logger().setParent(loggerPtr());
}

bool RouteAssembler::addTrack(const std::vector<Point> &_points,
                              int track_id,
                              bool checkBothSides,
                              const Polygon &limitBoundary)
{

    return addTrack(Subfield(), _points, track_id, checkBothSides, limitBoundary);

}

bool RouteAssembler::addTrack(const Subfield &sf,
                              const std::vector<Point> &_points,
                              int track_id,
                              bool checkBothSides,
                              const Polygon &limitBoundary)
{
    const double minSpeed = 0.01;//@TODO hardcoded. should we get this as a parameter?

    if(m_machine.working_width < 1e-6){
        logger().printWarning(__FUNCTION__, "The given machine has an invalid working width");
        return false;
    }

    double maxSpeed = m_machine.max_speed_empty;
    if(m_machine.bunker_mass > 1e-5 || m_machine.bunker_volume > 1e-5)//@TODO temporary workaround. This should be computed based on the amount of yield in the bunker, but this is planned afterwards
        maxSpeed = 0.5 * (m_machine.max_speed_empty + m_machine.max_speed_full);

    double defSpeed = (m_machine.def_working_speed > 0 ? m_machine.def_working_speed : maxSpeed);

    std::vector<Point> points = _points;

    std::set<Point> pointsOutside;
    if(sf.boundary_outer.points.size() > 3 && sf.boundary_inner.points.size() > 3){
        if(!adjustTracksPointsBasedOnBoundaries(sf, points, m_machine, pointsOutside)){
            logger().printError(__FUNCTION__, "Error adjusting track points intersection with the field boundary");
            return false;
        }
    }

    if(points.size() < 2){
        logger().printWarning(__FUNCTION__, "No track points left to be added.");
        return false;
    }

    if (!m_route.route_points.empty()){
        if(!addConnectionToTrack(points,
                                 sf,
                                 checkBothSides,
                                 limitBoundary)){
            logger().printError(__FUNCTION__, "Error obtaining connection to new track");
            return false;
        }

        if( geometry::calc_dist(m_route.route_points.back(), points.front()) > geometry::calc_dist(m_route.route_points.back(), points.back()) )
            std::reverse(points.begin(),points.end());
    }

    for (size_t i = 0; i < points.size(); ++i) {
        RoutePoint rp;
        rp.point() = points.at(i);
        rp.bunker_mass = 0.0;//@TODO should we update the bunker_mass of the harvester routepoints when the harvester has a bunker?
        rp.bunker_volume = 0.0;//@TODO should we update the bunker_volume of the harvester routepoints when the harvester has a bunker?
        if (m_route.route_points.empty()) {
            rp.time_stamp = -1;//set to zero if necessary in the next step
            rp.worked_mass = 0.0;
            rp.worked_volume = 0.0;
        }
        else {
            double calcSpeed = defSpeed;
            double mass = 0;
            if(pointsOutside.find( m_route.route_points.back().point() ) != pointsOutside.end()
                    || pointsOutside.find( rp.point() ) != pointsOutside.end()){//do not add mass

                calcSpeed = m_workingSpeedCalculator->calcSpeed(m_route.route_points.back(),
                                                                rp,
                                                                0,
                                                                m_machine);
                if (calcSpeed > maxSpeed)
                    calcSpeed = maxSpeed;
                if (calcSpeed < minSpeed){
                    logger().printOut(LogLevel::WARNING, __FUNCTION__, "Calculated harvester speed in track " + std::to_string(track_id)
                                       + " (index = " + std::to_string(i) + ") is lower than " + std::to_string(minSpeed) + " m/s");
                    calcSpeed = minSpeed;
                }
            }
            else if ( m_route.route_points.back().type != RoutePoint::HEADLAND ){//between the last point in the route and this point, some yield might be harvested

                mass = m_massCalculator->calcMass(m_route.route_points.back(),
                                                  rp,
                                                  m_machine.working_width);

                calcSpeed = m_workingSpeedCalculator->calcSpeed(m_route.route_points.back(),
                                                                rp,
                                                                0,
                                                                m_machine);
                if (calcSpeed > maxSpeed)
                    calcSpeed = maxSpeed;
                if (calcSpeed < minSpeed){
                    logger().printOut(LogLevel::WARNING, __FUNCTION__, "Calculated harvester speed in track " + std::to_string(track_id)
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

            rp.worked_mass = m_route.route_points.back().worked_mass;
            rp.worked_volume = m_route.route_points.back().worked_volume;
            if(i != 0){
                rp.worked_mass += mass;
                rp.worked_volume += 0.0;//@TODO
            }

        }

        rp.track_id = track_id;
        rp.type = m_workingRPType;
        if(i == 0/* && track_id > 0*/) {
            rp.type = RoutePoint::TRACK_START;
        }
        else if(i == points.size() - 1/* && track_id > 0*/) {
            rp.type = RoutePoint::TRACK_END;
        }
        else if(i > 0 && i+1 < points.size() && rp.time_stamp > -1e-9){
            const auto& p1 = points.at(i-1);
            const auto& p2 = points.at(i);
            const auto& p3 = points.at(i+1);
            if(p1 != p2 && p2 != p3){
                double ang = geometry::get_angle(p1, p2, p3);
                double turningTime = m_workingSpeedCalculator->calcTurningTime(ang, 0, m_machine);
                if(turningTime > 1){
                    m_route.route_points.push_back(rp);
                    rp.time_stamp += turningTime;
                }
            }
        }
        m_route.route_points.push_back(rp);
    }
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

void RouteAssembler::setTracksConnector(std::shared_ptr<IInfieldTracksConnector> tc)
{
    if(tc)
        m_tracksConnector = tc;
}

void RouteAssembler::setMassCalculator(std::shared_ptr<IEdgeMassCalculator> mc)
{
    if(mc)
        m_massCalculator = mc;
}

void RouteAssembler::setWorkingSpeedCalculator(std::shared_ptr<IEdgeSpeedCalculator> sc)
{
    if(sc)
        m_workingSpeedCalculator = sc;
}

void RouteAssembler::setTransitSpeedCalculator(std::shared_ptr<IEdgeSpeedCalculator> sc)
{
    if(sc)
        m_transitSpeedCalculator = sc;
}

std::shared_ptr<const IInfieldTracksConnector> RouteAssembler::getTracksConnector() const
{
    return m_tracksConnector;
}

void RouteAssembler::setTracksConnectionsMap(ITrackSequencer::PathsMapConstPtr_t conns)
{
    m_tracksConnectionsMap = conns;
}
bool RouteAssembler::addConnectionToTrack(const std::vector<Point> &nextTrack,
                                          const Subfield &sf,
                                          bool checkBothSides,
                                          const Polygon& limitBoundary)
{
    if (nextTrack.size() < 2) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid track");
        return false;
    }
    if (m_route.route_points.size() < 2) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Current route does not have enough points");
        return false;
    }

    Pose2D pose_start;
    pose_start.point() = m_route.route_points.back().point();
    pose_start.angle = arolib::geometry::get_angle( r_at(m_route.route_points, 1 ), m_route.route_points.back() );

    double delta_time = m_route.route_points.back().time_stamp - r_at(m_route.route_points, 1).time_stamp;
    double speed_start = std::fabs(delta_time) > 1e-9 ? arolib::geometry::calc_dist( r_at(m_route.route_points, 1 ), m_route.route_points.back() ) / delta_time  : 0;

    PointVec connection;
    if(m_tracksConnectionsMap){
        double turningRad = IInfieldTracksConnector::getTurningRad(m_machine, -1);
        Pose2D poseNext = Pose2D(nextTrack.front(), geometry::get_angle(nextTrack.front(), nextTrack.at(1)));
        PointVec conn1 = ITrackSequencer::getPathFromMap(m_tracksConnectionsMap, pose_start, poseNext, turningRad, true);
        PointVec conn2;
        if(checkBothSides){
            poseNext = Pose2D(nextTrack.back(), geometry::get_angle(nextTrack.back(), r_at(nextTrack,1)));
            conn2 = ITrackSequencer::getPathFromMap(m_tracksConnectionsMap, pose_start, poseNext, turningRad, true);
        }
        if(!conn1.empty() && !conn2.empty()){
            if( geometry::getGeometryLength(conn1) > geometry::getGeometryLength(conn2) )
                std::swap(connection, conn2);
            else
                std::swap(connection, conn1);
        }
        else if(!conn1.empty())
            std::swap(connection, conn1);
        else if(!conn2.empty())
            std::swap(connection, conn2);
    }

    if(connection.empty())
        connection = m_tracksConnector->getConnection(m_machine,
                                                      pose_start,
                                                      nextTrack,
                                                      checkBothSides,
                                                      -1,
                                                      std::make_pair(0.0, 0.0),
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
        rp.track_id = -1;

        const RoutePoint &rp_prev = m_route.route_points.back();
        rp.bunker_mass = rp_prev.bunker_mass;
        rp.bunker_volume = rp_prev.bunker_volume;
        rp.worked_mass = rp_prev.worked_mass;
        rp.worked_volume = rp_prev.worked_volume;

        double segment_length = arolib::geometry::calc_dist(rp_prev, rp);
        double speed = m_transitSpeedCalculator->calcSpeed(rp_prev, rp, rp.bunker_mass, m_machine);

        rp.time_stamp = rp_prev.time_stamp + segment_length / speed;

        m_route.route_points.push_back( rp );
    }


    return true;

}


bool RouteAssembler::adjustTracksPointsBasedOnBoundaries(const Subfield &sf, std::vector<Point> &points, const Machine& machine, std::set<Point>& pointsOutside)
{
    //@todo: pending tests!

    auto getIntersectionPoints = [&sf, &points]()->std::vector<Point>{
        auto intersection_points = geometry::get_intersection(points, sf.boundary_outer.points);
        for(size_t i = 0 ; i < intersection_points.size() ; i++){
            if(intersection_points.at(i) == points.front() || intersection_points.at(i) == points.back()){
                intersection_points.erase(intersection_points.begin()+i);
                --i;
            }
        }
        return intersection_points;
    };

    auto getIntersectionWithInnerBoundary = [&sf](std::vector<Point> points, bool inReverse, double width) -> Point{
        Point ret = Point::invalidPoint();
        Polygon poly;
        geometry::unsample_linestring(points);
        if(!geometry::offsetLinestring(points, poly, 0.5*width, 0.5*width, true, 0))
            return ret;
        auto intersection_points_ib = geometry::get_intersection(poly.points, sf.boundary_inner.points);

        double minDist = std::numeric_limits<double>::max();
        for(auto& ip : intersection_points_ib){
            auto ind = geometry::addSampleToGeometryClosestToPoint(points, ip, 1);
            if(ind < 0)
                continue;
            double dist = inReverse ? geometry::getGeometryLength(points, ind, -1) :
                                      geometry::getGeometryLength(points, 0, ind);
            if(minDist > dist){
                minDist = dist;
                ret = points.at(ind);
            }
        }
        return ret;
    };

    pointsOutside.clear();
    if(sf.boundary_outer.points.size() > 3 && sf.boundary_inner.points.size() > 3){

        auto intersection_points = getIntersectionPoints();

        size_t ind_ref_int = 0;
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
            points_1.erase( points_1.begin(), points_1.begin() + ind_1_ob);

            double width = std::max(machine.working_width, machine.working_width);
            if(width < 0)
                width = 0.1;

            int ind_0_ib = -1;
            Point pointInt_ib = getIntersectionWithInnerBoundary(points_0, true, width);
            if(pointInt_ib.isValid())
                ind_0_ib = geometry::addSampleToGeometryClosestToPoint(points_0, pointInt_ib, 1);
            if(ind_0_ib < 0){//second try
                ind_0_ib = points_0.size()-1;
                while (ind_0_ib > 0){
                    std::vector<Point> lineTmp;
                    lineTmp.push_back( geometry::getPointInLineAtDist( points_0.at(ind_0_ib), geometry::rotate(points_0.at(ind_0_ib), points_0.at(ind_0_ib-1), M_PI_2), 0.5 * width ) );
                    lineTmp.push_back( points_0.at(ind_0_ib) );
                    lineTmp.push_back( geometry::getPointInLineAtDist( points_0.at(ind_0_ib), geometry::rotate(points_0.at(ind_0_ib), points_0.at(ind_0_ib-1), -M_PI_2), 0.5 * width ) );
                    if(geometry::in_polygon(lineTmp.front(), sf.boundary_inner)
                            || geometry::in_polygon(lineTmp.back(), sf.boundary_inner)
                            || geometry::in_polygon(lineTmp.at(1), sf.boundary_inner)
                            || geometry::intersects(lineTmp, sf.boundary_inner))
                        break;
                    --ind_0_ib;
                }
            }


            int ind_1_ib = -1;
            pointInt_ib = getIntersectionWithInnerBoundary(points_1, false, width);
            if(pointInt_ib.isValid())
                ind_1_ib = geometry::addSampleToGeometryClosestToPoint(points_1, pointInt_ib, 1);
            if(ind_1_ib < 0){//second try
                ind_1_ib = 0;
                while (ind_1_ib+1 < points_1.size() ){
                    std::vector<Point> lineTmp;
                    lineTmp.push_back( geometry::getPointInLineAtDist( points_1.at(ind_1_ib), geometry::rotate(points_1.at(ind_1_ib), points_1.at(ind_1_ib+1), M_PI_2), 0.5 * width ) );
                    lineTmp.push_back( points_1.at(ind_1_ib) );
                    lineTmp.push_back( geometry::getPointInLineAtDist( points_1.at(ind_1_ib), geometry::rotate(points_1.at(ind_1_ib), points_1.at(ind_1_ib+1), -M_PI_2), 0.5 * width ) );
                    if(geometry::in_polygon(lineTmp.front(), sf.boundary_inner)
                            || geometry::in_polygon(lineTmp.back(), sf.boundary_inner)
                            || geometry::in_polygon(lineTmp.at(1), sf.boundary_inner)
                            || geometry::intersects(lineTmp, sf.boundary_inner))
                        break;
                    ++ind_1_ib;
                }
            }


            if( ind_0_ib < 0 || ind_1_ib >= points_1.size() ||
                    (ind_0_ib == 0 && ind_1_ib+1 == points_1.size() ) ){//remove track
                points.clear();
                pointsOutside.clear();
                return true; //false;
            }
            if(ind_0_ib == 0){
                points.clear();
                points.insert(points.end(), points_1.begin()+ind_1_ib, points_1.end());
                continue;
            }
            if(ind_1_ib+1 == points_1.size()){
                points.clear();
                points.insert(points.end(), points_0.begin(), points_0.begin()+ind_0_ib+1);
                continue;
            }

            Pose2D poseStart( points_0.at(ind_0_ib), geometry::get_angle(points_0.at(ind_0_ib-1), points_0.at(ind_0_ib)) );
            Pose2D poseFinish( points_1.at(ind_1_ib), geometry::get_angle(points_1.at(ind_1_ib), points_1.at(ind_1_ib+1)) );

            auto connection = m_tracksConnector->getConnection(sf,
                                                               machine,
                                                               poseStart,
                                                               poseFinish);

            if(connection.size() < 2){//try again without turning radius
                connection = m_tracksConnector->getConnection(sf,
                                                              machine,
                                                              poseStart,
                                                              poseFinish,
                                                              0.0);

                if(connection.size() < 2)
                    return false;
            }

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
