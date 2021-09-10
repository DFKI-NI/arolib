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
 
#include "arolib/types/route.hpp"


namespace arolib{

const std::set<RoutePoint::RoutePointType> Route::m_nonConsecutiveTypes = { RoutePoint::RoutePointType::FIELD_ENTRY,
                                                                            RoutePoint::RoutePointType::FIELD_EXIT,
                                                                            RoutePoint::RoutePointType::INITIAL_POSITION,
                                                                            RoutePoint::RoutePointType::OVERLOADING_START,
                                                                            RoutePoint::RoutePointType::OVERLOADING_FINISH,
                                                                            RoutePoint::RoutePointType::TRACK_START,
                                                                            RoutePoint::RoutePointType::TRACK_END };

RoutePoint Route::calcPoint(double time, int *refIndex) const
{
    return calcPoint(route_points ,time, refIndex);
}

RoutePoint Route::calcPoint(const std::vector<RoutePoint> &route_points, double time, int *refIndex)
{
    arolib::RoutePoint ret;
    if(route_points.empty()){//invalid route
        if(refIndex)
            *refIndex = -1;
        return ret;
    }

    if(time < route_points.front().time_stamp){//return the first route-point with the given timestamp
        if(refIndex)
            *refIndex = 0;
        ret = route_points.front();
        ret.time_stamp = time;
        return ret;
    }
    if(time > route_points.back().time_stamp){//return the last route-point with the given timestamp
        if(refIndex)
            *refIndex = route_points.size()-1;
        ret = route_points.back();
        ret.time_stamp = time;
        return ret;
    }

    if(route_points.size() == 1){//return the single route-point with the given timestamp
        if(refIndex)
            *refIndex = 0;
        ret = route_points.front();
        ret.time_stamp = time;
        return ret;
    }

    //get the the iterator pointing to the first route-point whose timestamp is greater than the given timestamp
    auto it = std::upper_bound( route_points.begin(), route_points.end(), time,
                                [](const double& t, const arolib::RoutePoint& rp){
                                      return rp.time_stamp > t;
                                } );

    if(it == route_points.end()){//return the last route-point with the given timestamp
        if(refIndex)
            *refIndex = route_points.size()-1;
        ret = route_points.back();
        ret.time_stamp = time;
        return ret;
    }

    if(it == route_points.begin()){//return the first route-point with the given timestamp
        if(refIndex)
            *refIndex = 0;
        ret = route_points.front();
        ret.time_stamp = time;
        return ret;
    }

    //set the reference index
    if(refIndex)
        *refIndex = it-route_points.begin()-1;

    //get the route-points located (in the time domain) immediatly before and after the desired point
    arolib::RoutePoint rp1 = *(it-1);
    arolib::RoutePoint rp2 = *it;

    double dTime1 = time - rp1.time_stamp;
    double dTime2 = rp2.time_stamp - rp1.time_stamp;
    double a = dTime1 / dTime2;

    //interpolate timestamp, location, and other parameters for the desired route-point
    ret = rp1;
    ret.time_stamp = time;
    ret.x = (1 - a) * rp1.x + a * rp2.x;
    ret.y = (1 - a) * rp1.y + a * rp2.y;
    ret.harvested_mass = (1 - a) * rp1.harvested_mass + a * rp2.harvested_mass;
    ret.harvested_volume = (1 - a) * rp1.harvested_volume + a * rp2.harvested_volume;
    ret.bunker_mass = (1 - a) * rp1.bunker_mass + a * rp2.bunker_mass;
    ret.bunker_volume = (1 - a) * rp1.bunker_volume + a * rp2.bunker_volume;
    ret.type = getIntermediateRPType(rp1, rp2, RoutePoint::DEFAULT, &ret.point());

    return ret;

}

RoutePoint Route::calcPoint2(double time, int &refIndex) const
{
    return calcPoint2(route_points, time, refIndex);
}

RoutePoint Route::calcPoint2(const std::vector<RoutePoint> &route_points, double time, int &refIndex)
{
    arolib::RoutePoint ret;

    if(route_points.empty()){//invalid route
        refIndex = -1;
        return ret;
    }

    if(time < route_points.front().time_stamp){//return the first route-point with the given timestamp
        refIndex = 0;
        ret = route_points.front();
        ret.time_stamp = time;
        return ret;
    }
    if(time > route_points.back().time_stamp){//return the last route-point with the given timestamp
        refIndex = route_points.size()-1;
        ret = route_points.back();
        ret.time_stamp = time;
        return ret;
    }

    if(route_points.size() == 1){//return the single route-point with the given timestamp
        refIndex = 0;
        ret = route_points.front();
        ret.time_stamp = time;
        return ret;
    }

    if(refIndex >= route_points.size()){
        refIndex = route_points.size()-1;
    }

    //get the route-points located (in the time domain) immediatly before and after the desired point
    arolib::RoutePoint rp1 = route_points.at(refIndex);
    arolib::RoutePoint rp2 = rp1;


    //starting from the reference index, find the route-point located (in the time domain) immediatly before the give timestamp (updating the refIndex)
    for(int i = refIndex ; i >= 0 ; --i){
        rp1 = route_points.at(i);
        refIndex = i;
        if( rp1.time_stamp < time )
            break;
    }

    //starting from the (updated) reference index, find the route-point located (in the time domain) immediatly after the give timestamp (updating the refIndex with its right value)
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
    if(fabs(dTime2) < 0.000001)//the immidiatly previous and next points have the same timestamp --> return the immidiatly previous one (note that these two points have practically the same timestamp, but they are not necesarily equal (other parameters such as type might be different))
        return ret;

    //interpolate timestamp, location, and other parameters for the desired route-point

    double a = dTime1 / dTime2;
    ret.x = (1 - a) * rp1.x + a * rp2.x;
    ret.y = (1 - a) * rp1.y + a * rp2.y;
    ret.harvested_mass = (1 - a) * rp1.harvested_mass + a * rp2.harvested_mass;
    ret.harvested_volume = (1 - a) * rp1.harvested_volume + a * rp2.harvested_volume;
    ret.bunker_mass = (1 - a) * rp1.bunker_mass + a * rp2.bunker_mass;
    ret.bunker_volume = (1 - a) * rp1.bunker_volume + a * rp2.bunker_volume;
    ret.type = getIntermediateRPType(rp1, rp2, RoutePoint::DEFAULT, &ret.point());

    return ret;

}

double Route::calcSpeed(double time) const
{
    int refIndex = 0;
    return calcSpeed(time, refIndex);
}

double Route::calcSpeed(const std::vector<RoutePoint> &route_points, double time)
{
    int refIndex = 0;
    return calcSpeed(route_points, time, refIndex);
}

double Route::calcSpeed(double time, int &refIndex) const
{
    return calcSpeed(route_points, time, refIndex);
}

double Route::calcSpeed(const std::vector<RoutePoint> &route_points, double time, int &refIndex)
{
    RoutePoint rp0, rp1;
    rp1 = calcPoint2(route_points, time, refIndex);//(interpolated) route-point at the given time
    rp0 = route_points.at(refIndex);//Route-point in the route  located immidieatly before (in time domain) the given time

    if( fabs(rp1.time_stamp - rp0.time_stamp) < 0.001 && refIndex == 0)//No time has elapsed between the 2 points and both correspond to the first route-point (or the given time is before the first route timestamp) --> speed = 0
        return 0;

    if( fabs(rp1.time_stamp - rp0.time_stamp) < 0.05 && refIndex != 0)//No (significant) time has elapsed between the 2 points and there exists a route point before rp0
        rp0 = route_points.at(refIndex-1);//set rp0 to its previous point to to avoid very high speeds in the speed calculation

    return Point::dist(rp1, rp0) / (rp1.time_stamp - rp0.time_stamp);

}

double Route::calcBearing(double time) const
{
    int refIndex = 0;
    return calcBearing(time, refIndex);
}

double Route::calcBearing(const std::vector<RoutePoint> &route_points, double time)
{
    int refIndex = 0;
    return calcBearing(route_points, time, refIndex);
}

double Route::calcBearing(double time, int &refIndex) const
{
    return calcBearing(route_points, time, refIndex);
}

double Route::calcBearing(const std::vector<RoutePoint> &route_points, double time, int &refIndex)
{
    RoutePoint rp0, rp1;
    rp1 = calcPoint2(route_points, time, refIndex);//(interpolated) route-point at the given time
    int indTmp = refIndex+1;

    do{
        indTmp--;
        rp0 = route_points.at(indTmp);
    } while(indTmp > 0 && Point::dist(rp0, rp1) < 1e-6);

    if(indTmp < 0 && refIndex + 1 < route_points.size()){
        indTmp = refIndex;
        rp0 = rp1;
        do{
            indTmp++;
            rp1 = route_points.at(indTmp);
        } while(indTmp+1 < route_points.size() && Point::dist(rp0, rp1) < 1e-6);
    }

    if( indTmp < 0 || indTmp >= route_points.size() )
        return 0;

    Point p0_geo, p1_geo;
    CoordTransformer::GetInstance().convert_to_geodetic(rp0, p0_geo);
    CoordTransformer::GetInstance().convert_to_geodetic(rp1, p1_geo);

    return Point::bearing(p0_geo, p1_geo);

}

std::vector<Point> Route::getPoints() const
{
    return RoutePoint::toPoints(route_points);
}

void Route::copyToWithoutPoints(Route &other, bool clearPoints) const
{
    other.route_id = route_id;
    other.machine_id = machine_id;
    if(clearPoints)
        other.route_points.clear();
}

bool Route::syncRoutes(std::vector<Route> &routes)
{
    std::vector<Route *> pRoutes(routes.size());
    for(size_t i = 0 ; i < routes.size() ; ++i)
        pRoutes[i] = &routes[i];
    return syncRoutes(pRoutes);
}

bool Route::syncRoutes(const std::vector<Route *> &routes)
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

int Route::getNextIndex(size_t startIdx, const std::function<bool (const RoutePoint &)> rule)
{
    for( ; startIdx < route_points.size() ; ++startIdx ){
        if( rule(route_points.at(startIdx)) )
            return startIdx;
    }
    return -1;
}

int Route::getNextIndex(size_t startIdx, const std::function<bool (const RoutePoint &, const RoutePoint &)> rule)
{
    for( ; startIdx+1 < route_points.size() ; ++startIdx ){
        if( rule(route_points.at(startIdx), route_points.at(startIdx+1)) )
            return startIdx;
    }
    return -1;
}

int Route::getNextIndex(size_t startIdx, const std::set<RoutePoint::RoutePointType>& RPTypes)
{
    return getNextIndex(startIdx, [&RPTypes](const RoutePoint &rp)->bool{
        return RPTypes.find(rp.type) != RPTypes.end();
    });
}

RoutePoint::RoutePointType Route::getIntermediateRPType(const RoutePoint &rp0, const RoutePoint &rp1, RoutePoint::RoutePointType defaultWorkingType, const Point *p)
{

    RoutePoint::RoutePointType rpType = rp0.type;
    bool rp0IsSpecial = rp0.isOfType( m_nonConsecutiveTypes );
    bool rp1IsSpecial = rp1.isOfType( m_nonConsecutiveTypes );
    if(rp0IsSpecial && rp1IsSpecial){
        if( rp0.type == RoutePoint::TRACK_START && rp1.type == RoutePoint::TRACK_END )
            rpType = defaultWorkingType;
        else if( rp0.type == RoutePoint::OVERLOADING_START && rp1.type == RoutePoint::OVERLOADING_FINISH )
            rpType = RoutePoint::OVERLOADING;
        else if( rp1.type == RoutePoint::TRACK_START
                && Track::isInfieldTrack(rp0.track_id)
                && Track::isInfieldTrack(rp1.track_id) )
            rpType = RoutePoint::HEADLAND;
        else
            rpType = RoutePoint::TRANSIT;
    }
    else if(rp0IsSpecial)
        rpType = rp1.type;
    else if(p && !rp0IsSpecial && !rp1IsSpecial && rp0.type != rp1.type){
        if( Point::dist(*p, rp1) < Point::dist(*p, rp0) )
            rpType = rp1.type;
    }

    return rpType;
}

}
