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
 
#include "arolib/components/planpostprocessor.h"
namespace arolib {
using namespace arolib::geometry;

std::vector<Point> adjustHeadlandPoints(std::vector<Point> nodes, const Point& p0, const Point& pn){
    if(nodes.size() < 3)
        return nodes;
    if(p0 != nodes.front()){
        std::vector<Point> intersections;
        if( !arolib::geometry::is_line(std::vector<Point>{p0, nodes.front(), nodes.at(1)}) ){
            Point p_ext = arolib::geometry::extend_line(p0, nodes.front(), 1000);
            intersections = arolib::geometry::get_intersection(nodes, p0, p_ext);
        }
        while(!intersections.empty()
              && ( intersections.back() == nodes.front() || intersections.back() == nodes.back() ) )
            intersections.pop_back();
        if( !intersections.empty() ){
            auto& p_int = intersections.back();
            size_t ind = 1;
            for( ; ind+1 < nodes.size() ; ++ind ){
                if( arolib::geometry::is_line( std::vector<Point>{ nodes.at(ind), p_int , nodes.at(ind+1) } )
                    && arolib::geometry::getLocationInLine(nodes.at(ind), nodes.at(ind+1), intersections.back()) == 0 )
                    break;
            }
            if(ind+1 < nodes.size()){
                nodes.erase( nodes.begin()+1, nodes.begin()+ind );
                nodes.at(1) = p_int;
            }
        }
    }
    if(nodes.size() < 3)
        return nodes;

    if(pn != nodes.back()){
        std::vector<Point> intersections;
        if( !arolib::geometry::is_line(std::vector<Point>{pn, nodes.back(), *(nodes.end()-2)}) ){
            Point p_ext = arolib::geometry::extend_line(pn, nodes.back(), 1000);
            intersections = arolib::geometry::get_intersection(nodes, pn, p_ext, false);
        }
        while(!intersections.empty()
              && ( intersections.back() == nodes.front() || intersections.back() == nodes.back() ) )
            intersections.pop_back();
        if( !intersections.empty() ){
            auto& p_int = intersections.back();
            size_t ind = 1;
            for( ; ind+1 < nodes.size() ; ++ind ){
                if( arolib::geometry::is_line( std::vector<Point>{ *(nodes.end()-1-ind), p_int , *(nodes.end()-2-ind) } )
                        && arolib::geometry::getLocationInLine(*(nodes.end()-1-ind), *(nodes.end()-2-ind), *(nodes.end()-2-ind)) == 0)
                    break;
            }
            if(ind+1 < nodes.size()){
                nodes.erase( nodes.end()-1-ind, nodes.end()-1 );
                *(nodes.end()-2) = p_int;
            }
        }
    }
    return nodes;
}

PlanPostProcessor::PlanPostProcessor(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

AroResp PlanPostProcessor::getSegmentedRoute(const Route &route,
                                                   std::set<RoutePoint::RoutePointType> stamps,
                                                   std::vector<Route> &segments,
                                                   bool includeRelatedMachines)
{
    segments.clear();
    size_t start = 0;

    if( !stamps.empty() ){
        for(size_t i = 1 ; i < route.route_points.size() ; ++i){
            bool addSegment = false;
            if( stamps.find( route.route_points.at(i).type ) != stamps.end() || i == route.route_points.size()-1 )
                addSegment = true;
            else if(includeRelatedMachines){
                for(auto &mri : route.route_points.at(i).machineRelations){
                    if( stamps.find( mri.routePointType ) != stamps.end() ){
                        addSegment = true;
                        break;
                    }
                }
            }
            if(addSegment){
                Route segment;
                segment.machine_id = route.machine_id;
                segment.route_id = route.route_id;
                segment.route_points.insert( segment.route_points.end() , route.route_points.begin()+start, route.route_points.begin()+i+1 );
                segments.push_back( segment );
                start = i;
            }
        }
    }

    if(segments.empty())
        segments.push_back(route);

    return AroResp(0, "OK");
}

AroResp PlanPostProcessor::addOutfieldSegments(std::vector<Route> &routes, const std::vector<Linestring> &external_roads, double distanceLimit, double res)
{
    if(external_roads.empty())
        return AroResp::ok();

    for(auto& route : routes){
        int ind1 = 0;
        while(ind1 < route.route_points.size()){//check segments connecting to a field entry
            ind1 = route.getNextIndex(ind1, {RoutePoint::FIELD_ENTRY});
            if(ind1 < 0)
                break;
            if(ind1 == 0 || !route.route_points.at(ind1-1).isOfType( {RoutePoint::RESOURCE_POINT, RoutePoint::INITIAL_POSITION, RoutePoint::FIELD_EXIT} )){
                ++ind1;
                continue;
            }

            auto connection = getBestRoadConnection( route.route_points.at(ind1-1),
                                                     route.route_points.at(ind1),
                                                     external_roads,
                                                     res);
            if(connection.empty()){
                ++ind1;
                continue;
            }
            double dist1 = arolib::geometry::calc_dist_to_linestring( connection, route.route_points.at(ind1-1) );
            double dist2 = arolib::geometry::calc_dist_to_linestring( connection, route.route_points.at(ind1) );
            double dist3 = arolib::geometry::calc_dist( route.route_points.at(ind1-1), route.route_points.at(ind1) );
            if( (distanceLimit >= 0 && (dist1 > distanceLimit || dist2 > distanceLimit))
                    || dist3 < dist1 || dist3 < dist2){
                ++ind1;
                continue;
            }

            size_t sizePrev = route.route_points.size();
            addConnectionToRoute(route, ind1-1, connection, RoutePoint::TRANSIT_OF);

            ind1 = ind1 + 1 + route.route_points.size() - sizePrev;
        }

        ind1 = 0;
        while(ind1 < route.route_points.size()){//check segments connecting to a field exit
            ind1 = route.getNextIndex(ind1, {RoutePoint::FIELD_EXIT});
            if(ind1 < 0 || ind1+1 >= route.route_points.size())
                break;
            if(ind1 == 0 || !route.route_points.at(ind1+1).isOfType( {RoutePoint::RESOURCE_POINT} )){
                ++ind1;
                continue;
            }

            auto connection = getBestRoadConnection( route.route_points.at(ind1),
                                                     route.route_points.at(ind1+1),
                                                     external_roads,
                                                     res);
            if(connection.empty()){
                ++ind1;
                continue;
            }

            double dist1 = arolib::geometry::calc_dist_to_linestring( connection, route.route_points.at(ind1) );
            double dist2 = arolib::geometry::calc_dist_to_linestring( connection, route.route_points.at(ind1+1) );
            double dist3 = arolib::geometry::calc_dist( route.route_points.at(ind1), route.route_points.at(ind1+1) );
            if( (distanceLimit >= 0 && (dist1 > distanceLimit || dist2 > distanceLimit))
                    || dist3 < dist1 || dist3 < dist2){
                ++ind1;
                continue;
            }

            size_t sizePrev = route.route_points.size();
            addConnectionToRoute(route, ind1, connection, RoutePoint::TRANSIT_OF);

            ind1 = ind1 + 1 + route.route_points.size() - sizePrev;
        }

    }
    return AroResp::ok();
}

bool PlanPostProcessor::addConnectionToRoute(Route &route,
                                             size_t ind,
                                             std::vector<Point> connection,
                                             RoutePoint::RoutePointType RPType)
{
    auto& rps = route.route_points;
    if(ind+1 >= rps.size())
        return false;

    if(!connection.empty() && connection.front() == rps.at(ind).point())
        pop_front(connection, 1);
    if(!connection.empty() && connection.back() == rps.at(ind+1).point())
        pop_back(connection);
    if(connection.empty())
        return true;

    push_front(connection, rps.at(ind).point());
    connection.push_back(rps.at(ind+1).point());

    std::vector<RoutePoint> connectionRPs(connection.size());
    double connlength = arolib::geometry::getGeometryLength(connection);
    double deltaTime = rps.at(ind+1).time_stamp - rps.at(ind).time_stamp;
    double l = 0;

    if(connlength == 0)
        return true;

    for(size_t i = 1 ; i+1 < connectionRPs.size() ; ++i ){
        connectionRPs.at(i) = rps.at(ind+1);
        connectionRPs.at(i).point() = connection.at(i);
        connectionRPs.at(i).type = RPType;
        if( deltaTime == 0 )
            continue;
        l += arolib::geometry::calc_dist( connection.at(i-1), connection.at(i) );
        connectionRPs.at(i).time_stamp = rps.at(ind).time_stamp + deltaTime * l / connlength;
    }


    route.route_points.insert( route.route_points.begin() + ind + 1,
                               connectionRPs.begin() + 1,
                               connectionRPs.end() - 1);

    return true;
}


}
