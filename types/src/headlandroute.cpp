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
 
#include "arolib/types/headlandroute.hpp"


namespace arolib{

RoutePoint HeadlandRoute::calcPoint(double time, int *refIndex) const
{

    arolib::RoutePoint ret;
    if(route_points.empty()){
        if(refIndex)
            *refIndex = -1;
        return ret;
    }

    if(time < route_points.front().time_stamp){
        if(refIndex)
            *refIndex = 0;
        ret = route_points.front();
        ret.time_stamp = time;
        return ret;
    }
    if(time > route_points.back().time_stamp){
        if(refIndex)
            *refIndex = route_points.size()-1;
        ret = route_points.back();
        ret.time_stamp = time;
        return ret;
    }

    if(route_points.size() == 1){
        if(refIndex)
            *refIndex = 0;
        ret = route_points.front();
        ret.time_stamp = time;
        return ret;
    }

    auto it = std::upper_bound( route_points.begin(), route_points.end(), time,
                                [](const double& t, const arolib::RoutePoint& rp){
                                      return rp.time_stamp > t;
                                } );

    if(it == route_points.end() || it+1 == route_points.end()){
        if(refIndex)
            *refIndex = route_points.size()-1;
        ret = route_points.back();
        ret.time_stamp = time;
        return ret;
    }

    if(it == route_points.begin()){
        if(refIndex)
            *refIndex = 0;
        ret = route_points.front();
        ret.time_stamp = time;
        return ret;
    }


    if(refIndex)
        *refIndex = it-route_points.begin()-1;

    arolib::RoutePoint rp1 = *(it-1);
    arolib::RoutePoint rp2 = *it;

    double dTime1 = time - rp1.time_stamp;
    double dTime2 = rp2.time_stamp - rp1.time_stamp;
    double a = dTime1 / dTime2;

    ret = rp1;
    ret.time_stamp = time;
    ret.x = (1 - a) * rp1.x + a * rp2.x;
    ret.y = (1 - a) * rp1.y + a * rp2.y;
    ret.harvested_mass = (1 - a) * rp1.harvested_mass + a * rp2.harvested_mass;
    ret.harvested_volume = (1 - a) * rp1.harvested_volume + a * rp2.harvested_volume;
    ret.bunker_mass = (1 - a) * rp1.bunker_mass + a * rp2.bunker_mass;
    ret.bunker_volume = (1 - a) * rp1.bunker_volume + a * rp2.bunker_volume;

    return ret;
}

RoutePoint HeadlandRoute::calcPoint2(double time, int &refIndex) const
{
    arolib::RoutePoint ret;

    if(route_points.empty()){
        refIndex = -1;
        return ret;
    }

    if(time < route_points.front().time_stamp){
        refIndex = 0;
        ret = route_points.front();
        ret.time_stamp = time;
        return ret;
    }
    if(time > route_points.back().time_stamp){
        refIndex = route_points.size()-1;
        ret = route_points.back();
        ret.time_stamp = time;
        return ret;
    }

    if(route_points.size() == 1){
        refIndex = 0;
        ret = route_points.front();
        ret.time_stamp = time;
        return ret;
    }

    if(refIndex >= route_points.size()){
        refIndex = route_points.size()-1;
        ret = route_points.back();
        ret.time_stamp = time;
        return ret;
    }

    arolib::RoutePoint rp1 = route_points.at(refIndex);
    arolib::RoutePoint rp2 = rp1;


    for(int i = refIndex ; i >= 0 ; --i){
        rp1 = route_points.at(i);
        refIndex = i;
        if( rp1.time_stamp < time )
            break;
    }
    for(size_t i = refIndex+1 ; i < route_points.size() ; ++i){
        rp2 = route_points.at(i);
        if( rp2.time_stamp > time )
            break;
        rp1 = rp2;
        refIndex = i;
    }

    double dTime1 = time - rp1.time_stamp;
    double dTime2 = rp2.time_stamp - rp1.time_stamp;

    ret = rp1;
    ret.time_stamp = time;
    if(fabs(dTime2) < 0.000001)
        return ret;

    double a = dTime1 / dTime2;
    ret.x = (1 - a) * rp1.x + a * rp2.x;
    ret.y = (1 - a) * rp1.y + a * rp2.y;
    ret.harvested_mass = (1 - a) * rp1.harvested_mass + a * rp2.harvested_mass;
    ret.harvested_volume = (1 - a) * rp1.harvested_volume + a * rp2.harvested_volume;
    ret.bunker_mass = (1 - a) * rp1.bunker_mass + a * rp2.bunker_mass;
    ret.bunker_volume = (1 - a) * rp1.bunker_volume + a * rp2.bunker_volume;

    return ret;

}

double HeadlandRoute::calcSpeed(double time) const
{
    int refIndex = 0;
    return calcSpeed(time, refIndex);
}

double HeadlandRoute::calcSpeed(double time, int &refIndex) const
{
    RoutePoint rp0, rp1;
    rp1 = calcPoint2(time, refIndex);
    rp0 = route_points.at(refIndex);

    if( fabs(rp1.time_stamp - rp0.time_stamp) < 0.001 && refIndex == 0)
        return 0;

    if( fabs(rp1.time_stamp - rp0.time_stamp) < 0.05 && refIndex != 0)
        rp0 = route_points.at(refIndex-1);

    return Point::dist(rp1, rp0) / (rp1.time_stamp - rp0.time_stamp);
}

std::vector<Point> HeadlandRoute::getPoints() const
{
    return RoutePoint::toPoints(route_points);
}

bool HeadlandRoute::syncRoutes(std::vector<HeadlandRoute> &routes)
{
    std::vector<HeadlandRoute *> pRoutes(routes.size());
    for(size_t i = 0 ; i < routes.size() ; ++i)
        pRoutes[i] = &routes[i];
    return syncRoutes(pRoutes);
}

bool HeadlandRoute::syncRoutes(const std::vector<HeadlandRoute *> &routes)
{
    for(auto& r : routes){
        if(r->baseDateTime.empty())
            return false;
    }
    size_t indBase = 0;
    std::vector<DateTime> times(routes.size());
    for(size_t i = 0 ; i < routes.size() ; ++i){
        if(!times[i].fromISO8601(routes.at(i)->baseDateTime) ){
            std::cout << "Parse failed [" << i << "]: " << routes.at(i)->baseDateTime << std::endl;
            return false;
        }
        if(i == 0)
            continue;

        if(times[i] - times[indBase] < 0)
            indBase = i;
    }

    for(size_t i = 0 ; i < routes.size() ; ++i){
        if(i == indBase)
            continue;
        routes[i]->baseDateTime = routes[indBase]->baseDateTime;
        double deltaTime = times[i] - times[indBase];
        for(auto& rp : routes[i]->route_points)
            rp.time_stamp += deltaTime;
    }

    return true;

}

}
