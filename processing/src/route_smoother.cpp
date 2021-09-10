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
 
#include "arolib/processing/route_smoother.hpp"


namespace arolib{

using namespace arolib::geometry;

const double RouteSmoother::m_extraDistMult = 1;
const double RouteSmoother::m_curveDistMult = 2.5;
const std::set<RoutePoint::RoutePointType> m_nonConsecutiveTypes = { RoutePoint::RoutePointType::FIELD_ENTRY,
                                                                     RoutePoint::RoutePointType::FIELD_EXIT,
                                                                     RoutePoint::RoutePointType::INITIAL_POSITION,
                                                                     RoutePoint::RoutePointType::OVERLOADING_START,
                                                                     RoutePoint::RoutePointType::OVERLOADING_FINISH,
                                                                     RoutePoint::RoutePointType::TRACK_START,
                                                                     RoutePoint::RoutePointType::TRACK_END };

RouteSmoother::TimeHandlingStrategy RouteSmoother::intToTimeHandlingStrategy(int value)
{

    if(value == TimeHandlingStrategy::KEEP_ELAPSED_TIME)
        return TimeHandlingStrategy::KEEP_ELAPSED_TIME;
    else if(value == TimeHandlingStrategy::RECALC_PROP_TO_LENGTH)
        return TimeHandlingStrategy::RECALC_PROP_TO_LENGTH;
    throw std::invalid_argument( "The given value does not correspond to any RouteSmoother::TimeHandlingStrategy" );
}

RouteSmoother::RouteSmoother(LogLevel logLevel)
    : LoggingComponent(logLevel, __FUNCTION__)
{
}

void RouteSmoother::setWorkingGroup(const std::vector<Machine> &machines, bool removeCurrent)
{
    if(removeCurrent)
        m_machines.clear();

    for(auto& m : machines)
        m_machines[m.id] = m;
}


bool RouteSmoother::smoothenOLVRoute(Route &route, RouteSmoother::TimeHandlingStrategy THStrategy, bool onlyNonOverloadingSegments) const
{
    auto it = m_machines.find(route.machine_id);
    if(it == m_machines.end()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Machine not found");
        return false;
    }
    return smoothenOLVRoute(it->second, route, THStrategy, onlyNonOverloadingSegments);
}

bool RouteSmoother::smoothenOLVRoute(const Machine &machine, Route &route, RouteSmoother::TimeHandlingStrategy THStrategy, bool onlyNonOverloadingSegments) const
{

    if( machine.machinetype != Machine::OLV ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The route does not belong to an OLV machine");
        return false;
    }

    return smoothenRoute(route, machine, THStrategy, onlyNonOverloadingSegments);

}

bool RouteSmoother::smoothenHarvesterRoute(Route &route, const Polygon &boundary, RouteSmoother::TimeHandlingStrategy THStrategy, bool smoothenHLSegments) const
{
    auto it = m_machines.find(route.machine_id);
    if(it == m_machines.end()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Machine not found");
        return false;
    }
    return smoothenHarvesterRoute(it->second, route, boundary, THStrategy, smoothenHLSegments);
}

bool RouteSmoother::smoothenHarvesterRoute(const Machine &machine,
                                           Route &route,
                                           const Polygon &boundary,
                                           RouteSmoother::TimeHandlingStrategy THStrategy,
                                           bool smoothenHLSegments) const
{

    if( !machine.isOfWorkingType(true) ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The route does not belong to a 'working-type' machine");
        return false;
    }

    if(smoothenHLSegments){//smoothen the inner-field intertrack (headland) connections to be ear-liked shaped
        if(! smoothenHeadlandSegments( machine, route, boundary, THStrategy ) )
            return false;
    }

    auto& route_points = route.route_points;

    if(route_points.size() < 2)
        return true;

    size_t ind0 = 0;//this will hold the index corresponding to the first point of the segment to be smoothen

    //do not smoothen initial outfield segment
    if( route_points.at(1).type == RoutePoint::FIELD_ENTRY )//since we are dealing with planned routes, if the harvester is outside the field, the second point of its rout will be a FIELD_ENTRY
        ++ind0;

    size_t ind1 = ind0; //this will hold the index corresponding to the last point of the segment to be smoothen
    size_t delta_ind1 = 0;//used to hold the difference between the index of the point (if existent) before the original ind1 and the one used for the smoothening (after smoothing a segment, the check for the next segment to smoothen should start from the point before the original ind1)

    size_t prevSize;//the size of the route before smoothing a segment

    //get the indexes corresponding to the initial segment (prior to harvesting)
    while( ind1 < route_points.size() ){
        auto& rp = route_points.at(ind1);
        if(rp.track_id >= 0)//the route point belongs to a harvesting track
            break;
        ++ind1;
        delta_ind1 = 1;
    }
//    while( ind1 < route_points.size() ){
//        auto& rp = route_points.at(ind1);
//        if(rp.isOfType({RoutePoint::DEFAULT,RoutePoint::TRACK_START}) )
//            break;
//        ++ind1;
//    }

    //Note: at this point, when talking about 'segment' to smoothen, it doesn't refer to a single 'sharp' segment, but rather a part of the route which may have several 'sharp' (sub) segments

    if( ind1 < route_points.size() ){//smoothen the route from FIELD_ENTRY (or from beginning if the harvester starts inside the field) until the first inner-fiel 'harvesting' point

        if( ind1 > ind0 ){ //there exists a initial segment (prior to harvesting)

            //Check if extra points are needed to be added in order not to have undesired transitions between the initial segment (prior to harvesting) and the next segment (harvesting)
            //The idea here is to extend the (original) initial segment to be smoothen with points subsequent to the segment (harvesting points) so that the transition to the remaining part of the route is also smooth but ensuring that the harvestng points are driven over (at least close enought)
            //The initial segment keeps extending with the next point as long as the distance between the last point of the original segment and the subsequent point to the extended segment is lower than extraRefDist
            double extraRefDist = machine.getTurningRadius()*m_extraDistMult;
            while(ind1+1 < route_points.size()){
                auto extra = route_points.at(ind1+1);//temporarilly holding the point that comes after the segment to be smoothen
                double dist = arolib::geometry::calc_dist(route_points.at(ind1), extra);//distance between the last point of the (extended) segment to be smoothen and the subsequent point
                if( dist - extraRefDist > 1e-2 ){//obtain an intermidiate (interpolated) point between points at ind1 and ind1+1 so that the distance between the interpolated point and the point at ind1 is equal to extraRefDist. Update ind1 and stop extending
                    int iTmp = ind1;
                    double dTime = extra.time_stamp - route_points.at(ind1).time_stamp;
                    dTime *= (extraRefDist/dist);
                    extra = route.calcPoint2(route_points.at(ind1).time_stamp + dTime, iTmp);
                    extra.type = getIntermediateRPType(route_points.at(ind1), route_points.at(ind1+1), RoutePoint::getDefaultRPType(machine), &extra.point());
                    route_points.insert(route_points.begin()+ind1+1, extra);
                    ++ind1;
                    ++delta_ind1;
                    break;
                }
                else if( extraRefDist - dist > 1e-2 ){//update extraRefDist and ind1, and keep extending
                    ++ind1;
                    ++delta_ind1;
                    extraRefDist -= dist;
                }
                else{//the distance between points is almost extraRefDist --> update extraRefDist and ind1, and stop extending
                    ++ind1;
                    ++delta_ind1;
                    break;
                }
            }

            //at this point, ind0 and ind1 correspond to the indexes of the first and last points of the EXTENDED segment to be smoothen

            prevSize = route_points.size();//the size of the route before smoothing the current segment

            //smoothen the route's extended initial segment (prior to harvesting)
            if( !smoothenRoutePart(route, 0, ind1, machine, THStrategy, false) ){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error smoothing the initial segment (prior to harvesting)");
                return false;
            }

            ind1 += (route_points.size() - prevSize);//adjust ind1 based on the amount of points that were added/removed during smoothing
        }

        //-------------

        if( ind1 < route_points.size() && Track::isHeadlandTrack(route_points.at(ind1).track_id )){//there are headland tracks (i.e. headland harvesting) --> smoothen the segment corresponding to headland harvesting
            ind0 = ind1-delta_ind1;
            delta_ind1 = 0;

            while( ind1 < route_points.size() ){
                auto& rp = route_points.at(ind1);
                if(!Track::isHeadlandTrack(rp.track_id))//the route point does NOT belong to a headland-harvesting track
                    break;
                ++ind1;
                delta_ind1 = 1;
            }

            if( ind1 < route_points.size() ){
                //smoothen headland-harvesting route segment (@TODO: temporary solution. at some point it would be good to adjust more realistically the headland-track corners)

                prevSize = route_points.size();

                if( !smoothenRoutePart(route, ind0, ind1, machine, THStrategy, false) ){
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error smoothing the headland-harvesting route segment");
                    return false;
                }

                ind1 += (route_points.size() - prevSize);//adjust ind1 based on the amount of points that were added/removed during smoothing
            }
        }


        if( ind1 < route_points.size() && route_points.at(ind1).track_id < 0 ){//there exists a segment connecting the headland harvesting and the infield harvesting segments
            ind0 = ind1-delta_ind1;
            delta_ind1 = 0;

            while( ind1 < route_points.size() ){
                auto& rp = route_points.at(ind1);
                if(rp.track_id >= 0)//the route point belongs to a harvesting track
                    break;
                ++ind1;
                delta_ind1 = 1;
            }

            if( ind1 < route_points.size() ){
                //smoothen headland-infield connection segment

                prevSize = route_points.size();

                if( !smoothenRoutePart(route, ind0, ind1, machine, THStrategy, false) ){
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error smoothing the headland-infield connection segment");
                    return false;
                }

                ind1 += (route_points.size() - prevSize);//adjust ind1 based on the amount of points that were added/removed during smoothing
            }
        }

    }

    //note: no smoothening is done to the infield harvesting (we assume that the infield tracks do not have sharp corners)
    //@todo: this doesn't apply when the harvester must go through the headland in the middle of the track

    //-------------

    //now smoothen the last segment (after harvesting)

    //in the next lines, ind0 and ind1 correspond to REVERSE indexes (i.e. ind=0 corresponds to the last point)

    ind1 = 0;

    //do not smoothen last outfield segment --> find the index corresponding to the FIELD_EXIT
    if( r_at(route_points, 1).type == RoutePoint::FIELD_EXIT )//at the moment, if the harvester is sent to an exit point, the last point is a FIELD_EXIT (i.e. there is no final outfield segment). But lets check in case in the future the harvester has to be sent elsewhere after exiting the field...
        ++ind1;

    ind0 = ind1;

    while( ind0 < route_points.size() ){
        RoutePoint& rp = r_at(route_points, ind0);
        if( rp.isOfTypeWorking_InTrack(true) || rp.type == RoutePoint::TRACK_END )//last working point
            break;
        ++ind0;
    }

    if( ind0 < route_points.size() && ind0 > ind1 ){//there exists a segment to exit the field (after working)

        //set again the indexes to be normal (forward) indexes (they were reverse indexes)
        ind0 = route_points.size()-1-ind0;
        ind1 = route_points.size()-1-ind1;

        //Check if extra points are needed to be added in order not to have undesired transitions between the last harvesting segment and the final segment (to the exit after harvesting)
        //Similar to what it was done for the initial segment, but now the extension is done at the beginning of the segment
        double extraRefDist = machine.getTurningRadius()*m_extraDistMult;
        while(ind0 > 0) {
            auto extra = route_points.at(ind0-1);
            double dist = arolib::geometry::calc_dist(route_points.at(ind0), extra);
            if( dist - extraRefDist > 1e-2 ){
                int iTmp = ind0-1;
                double dTime = route_points.at(ind0).time_stamp - extra.time_stamp;
                dTime *= (extraRefDist/dist);
                extra = route.calcPoint2(route_points.at(ind0).time_stamp - dTime, iTmp);
                extra.type = getIntermediateRPType(route_points.at(ind0-1), route_points.at(ind0), RoutePoint::getDefaultRPType(machine), &extra.point());
                route_points.insert(route_points.begin()+ind0, extra);
                ++ind1;
            }
            else if( extraRefDist - dist > 1e-2 ){
                --ind0;
                extraRefDist -= dist;
            }
            else{
                --ind0;
                break;
            }
        }

        //smoothen the last segment (after harvesting)
        if( !smoothenRoutePart(route, ind0, route_points.size()-1, machine, THStrategy, false) ){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error smoothing the final segment (after harvesting)");
            return false;
        }
    }

    return true;
}

bool RouteSmoother::smoothenHeadlandSegments(Route &route, const Polygon &boundary, RouteSmoother::TimeHandlingStrategy THStrategy) const
{
    auto it = m_machines.find(route.machine_id);
    if(it == m_machines.end()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Machine not found");
        return false;
    }
    return smoothenHeadlandSegments(it->second, route, boundary, THStrategy);
}

bool RouteSmoother::smoothenHeadlandSegments(const Machine &machine, Route &route, const Polygon &boundary, RouteSmoother::TimeHandlingStrategy THStrategy) const
{
    if( !machine.isOfWorkingType(true) ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "The route does not belong to a working machine");
        return false;
    }

    double width = machine.working_width;
    if(!machine.isOfWorkingType(true) && machine.width > 0)
        width = 0.5 * ( machine.working_width + machine.width );

    auto & points = route.route_points;
    size_t ind_TS = 0, ind_TE = 0;//indexes corresponding to the TRACK_START and TRACK_END points (the headland connection will be the route segment between (and including) ind_TE and ind_TS)
    while (ind_TS < points.size() && ind_TE < points.size()) {

        //get the index of the next TRACK_END
        while(ind_TE < points.size() && points.at(ind_TE).type != RoutePoint::TRACK_END)
            ++ind_TE;

        //get the index of the next TRACK_START (after the previously obtained TRACK_END)
        ind_TS = ind_TE + 1;
        while(ind_TS < points.size() && points.at(ind_TS).type != RoutePoint::TRACK_START)
            ++ind_TS;

        if(ind_TS >= points.size())//no more headland connections
            break;

        std::vector<Point> hl_points_0 ( ind_TS-ind_TE+1 );//(original) headland connection
        for(size_t i = 0 ; i < hl_points_0.size() ; ++i)
            hl_points_0[i] = points.at(i + ind_TE).point();

        if(hl_points_0.size() <3){//not enough points (unexpected case), leave the  headland connection as it is
            ind_TE = ind_TS + 1;
            continue;
        }

        auto hl_points = hl_points_0;

        auto hl_points_us = hl_points;//unsampled headland connection
        arolib::geometry::unsample_linestring(hl_points_us, std::max( 1e-6, 0.01 * machine.getTurningRadius() ));

        std::pair<int, Point> restore_point = std::make_pair(0, Point());//holds the points to be restored in case hl_points_us is not rectangle-like shaped

        auto hl_points_tmp = hl_points_us;
        if(hl_points_tmp.size() > 2){//adjust the points of hl_points_us so that they are 'rectagle-like' shaped (remember, This method was design for harvester routes that were generated following the TrackConnectionStrategy=OPTIMAL_INTERTRACK_CONNECTION)
            double d0 = arolib::geometry::calc_dist( hl_points_tmp.front(), hl_points_tmp.at(1) );
            double d1 = arolib::geometry::calc_dist( hl_points_tmp.back() , r_at(hl_points_tmp,1) );
            if( std::fabs(d0-d1) > 0.2 * width ){ //the connection is not rectangle-like (one of the sides was extended during the inter-track connection generation)
                Point ps = hl_points_tmp.front();
                Point pe = hl_points_tmp.front();
                if(d0 > d1){
                    //restore_point = std::make_pair(1, hl_points_tmp.front());
                    Point p_new;
                    arolib::geometry::getParallelVector(hl_points_tmp.at(1), hl_points_tmp.front(), p_new, d1);
                    p_new += hl_points_tmp.at(1);
                    hl_points_tmp.front() = p_new;
                }
                else{
                    //restore_point = std::make_pair(-1, hl_points_tmp.back());
                    Point p_new;
                    arolib::geometry::getParallelVector(r_at(hl_points_tmp,1), hl_points_tmp.back(), p_new, d0);
                    p_new += r_at(hl_points_tmp,1);
                    hl_points_tmp.back() = p_new;
                }
            }
        }

        bool useNormalSmoothing = false;
        std::vector<Point> nodes, smoothPoints;

        if(hl_points_tmp.size() == 4){//check it the points form a rectangle
            double ang1 = arolib::geometry::get_angle( hl_points_tmp.at(0), hl_points_tmp.at(1), hl_points_tmp.at(2));
            double ang2 = arolib::geometry::get_angle( hl_points_tmp.at(1), hl_points_tmp.at(2), hl_points_tmp.at(3));
            double ang3 = arolib::geometry::get_angle( hl_points_tmp.at(2), hl_points_tmp.at(3), hl_points_tmp.at(1));
            if(ang1 * ang2 > 0 && ang1 * ang3 > 0
                    && std::fabs( std::fabs(ang1) - M_PI_2) < 1e-2
                    && std::fabs( std::fabs(ang2) - M_PI_2) < 1e-2){//it is a rectanglisch connetion --> smoothen using the radius
                if( getEarFromRightAnglesSegment(hl_points_us, machine, smoothPoints) ){
                    if(restore_point.first > 0)//restore the saved point at the beginning of the smoothen connection
                        smoothPoints.insert( smoothPoints.begin(), restore_point.second );
                    else if(restore_point.first < 0)//restore the saved point at the end of the smoothen connection
                        smoothPoints.emplace_back( restore_point.second );
                    smoothPoints = sample_geometry_ends(smoothPoints, width);//sample the first and last (2-point-)subsegments of the smoothen connection
                }
                else
                    smoothPoints.clear();
            }
        }

        if(smoothPoints.empty()){
            if(hl_points_us.size() > 4){
                useNormalSmoothing = true;
                double angRef = arolib::geometry::get_angle( hl_points_us.at(0), hl_points_us.at(1), hl_points_us.at(2), false, false);
                for(size_t i = 2 ; i+1 < hl_points_us.size() ; ++i){
                    if ( angRef * arolib::geometry::get_angle( hl_points_us.at(i-1), hl_points_us.at(i), hl_points_us.at(i+1), false, false) < 0 ){//direction changed
                        useNormalSmoothing = true;
                        break;
                    }
                }
            }

            if(!useNormalSmoothing){
                nodes = getBSplineAdjustedNodes(hl_points_us, width);//obtain the nodes for the BSpline computation

                size_t samples = (hl_points.size() > 5? hl_points.size()*2 : hl_points.size()*3);//number of samples for the BSpline computation
                smoothPoints = getBSplinePoints(nodes, samples, nodes.size()-1);//obtain the smooth connection using BSpline

                if(restore_point.first > 0)//restore the saved point at the beginning of the smoothen connection
                    smoothPoints.insert( smoothPoints.begin(), restore_point.second );
                else if(restore_point.first < 0)//restore the saved point at the end of the smoothen connection
                    smoothPoints.emplace_back( restore_point.second );
                smoothPoints = sample_geometry_ends(smoothPoints, width);//sample the first and last (2-point-)subsegments of the smoothen connection
            }
        }

        if( useNormalSmoothing ||
                ( boundary.points.size() > 3
                  && !arolib::geometry::in_polygon(smoothPoints, boundary, true) ) ){//if the smoothen connection is outside the boundary, make a normal smoothing

            size_t size_prev = route.route_points.size();
            if( !smoothenRoutePart(route, ind_TE, ind_TS, machine, THStrategy, false) ){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error smoothing the headland segment");
                return false;
            }

            ind_TE = ind_TS + 1 + (route.route_points.size() - size_prev);
            continue;
        }

        //now we map/project the (new) smooth HL connection points into the original HL connection points based on the length from the connection points to the first point of the connection (relative to the total length of the respective connection)

        double length = arolib::geometry::getGeometryLength(hl_points);//length of the original headland connection
        double length2 = arolib::geometry::getGeometryLength(smoothPoints);//length of the smoothen headland connection
        std::vector<double> rl(hl_points.size(), 0);//normalized lengths of the original headland connection points w.r.t. the total length of the original headland connection (values = [0,1])

        //obtain the normalized length from the first point to each one of the other points of the original HL connection, relative to the total length of the original HL connection
        double d = 0;
        for(size_t i = 1 ; i < rl.size() ; ++i){
            double d2 = arolib::geometry::calc_dist( hl_points.at(i-1), hl_points.at(i) );
            d += d2;
            rl[i] = d / length;
        }

        std::vector<RoutePoint> newRPs(smoothPoints.size());
        newRPs.front() = points.at(ind_TE);//the first route-point of the smoothen headland connection is the same as before
        d = 0;
        size_t ind1 = 0, ind2 = 1;//indexes of the subsegment of the original HL connection that 'overlap' (based on the normalized lengths) with the subsegment of the smoothen HL connection under analysis
        double refTime = 0;
        for(size_t i = 1 ; i < smoothPoints.size() ; ++i){
            double d2 = arolib::geometry::calc_dist( smoothPoints.at(i-1), smoothPoints.at(i) );
            double rl0 = d / length2;//normalized length from the first point to point at index i-1 of the smoothen HL connection, relative to the total length of the smoothen HL connection
            d += d2;
            double rl1 = d / length2;//normalized length, from the first point to point at index i of the smoothen HL connection, relative to the total length of the smoothen HL connection

            //increase ind2 until it reaches the right location w.r.t. to the normalized lengths
            while(ind2 < rl.size() && rl.at(ind2) < rl1)
                ++ind2;
            if(ind2 >= rl.size())
                ind2 = rl.size()-1;

            //now, based on the normalized lengths, we can map/project the subsegment [i-1, i] of the smoothen HL connection, into the corresponding ('overlaping') subsegment [ind1, ind2] of the original HL connection, in order to obtain the (relative) route-point parameters (timestamp, etc..)
            //for example, the middle point of the smooth HL connection (i.e. the point (or interpolated point) located half of the way) will be projected to the middle point of the original HL connection, i.e. they will have (for now) the same route-poin parameters (inc. timestamp)

            double dTime = 0;
            if(ind2-ind1 < 2){
                if(rl.at(ind2) != rl.at(ind1)){
                    dTime += ( points.at(ind_TE + ind2).time_stamp - points.at(ind_TE + ind1).time_stamp )
                            * ( rl1 - rl0 ) / ( rl.at(ind2) - rl.at(ind1) );
                }
            }
            else{
                if(rl.at(ind1+1) != rl.at(ind1)){
                    dTime += ( points.at(ind_TE + ind1+1).time_stamp - points.at(ind_TE + ind1).time_stamp )
                            * ( rl.at(ind1+1) - rl0 ) / ( rl.at(ind1+1) - rl.at(ind1) );
                }
                if(rl.at(ind2) != rl.at(ind2-1)){
                    dTime += ( points.at(ind_TE + ind2).time_stamp - points.at(ind_TE + ind2-1).time_stamp )
                            * ( rl1 - rl.at(ind2-1) ) / ( rl.at(ind2) - rl.at(ind2-1) );
                }
                for(size_t j = ind1+1 ; j+1 < ind2 ; ++j)
                    dTime += ( points.at(ind_TE + j+1).time_stamp - points.at(ind_TE + j).time_stamp );
            }
            refTime += dTime;

            auto &rp = newRPs.at(i);

            if(i+1 == smoothPoints.size())
                rp = points.at(ind_TS);
            else{//get the (interpolated) route-point at the given time
                int tmp = ind_TE;
                rp = route.calcPoint2( points.at(ind_TE).time_stamp + refTime, tmp );
                if(tmp == ind_TE)
                    rp = points.at(ind_TE+1);
                rp.type = RoutePoint::HEADLAND;
            }

            rp.time_stamp = newRPs.at(i-1).time_stamp + dTime;
            rp.point() = smoothPoints.at(i);

            ind1 = ind2-1;//set ind1 for the next mapping/projection

        }

        if(THStrategy == RECALC_PROP_TO_LENGTH){//recalculate the timestamps of the smooth HL connection based on the difference between the lengths of the smoothen and original segments (for example, if the smoothen connection is twice as long as the original connection, the time needed to drive over the smooth connection must be twice the time originally needed to drive over the connection)
            double dTime0 = newRPs.back().time_stamp - newRPs.front().time_stamp;
            if(dTime0 != 0){
                double dTimeMult = ( dTime0 * length2 / length ) / dTime0;
                double dTimeRef = 0;
                double tPrevOld = newRPs.front().time_stamp;
                for(size_t i = 1 ; i < newRPs.size() ; ++i){
                    double dTime = newRPs.at(i).time_stamp - tPrevOld;
                    tPrevOld = newRPs.at(i).time_stamp;
                    newRPs.at(i).time_stamp = newRPs.at(i-1).time_stamp + dTime*dTimeMult;
                    dTimeRef = (newRPs.at(i).time_stamp-tPrevOld);
                }

                for(size_t i = ind_TS+1 ; i < points.size() ; ++i)
                    points.at(i).time_stamp += dTimeRef;
            }
        }

        //replace the original HL connection with the smooth (ear-like) connection
        points.erase( points.begin()+ind_TE , points.begin()+ind_TS+1 );
        points.insert( points.begin()+ind_TE, newRPs.begin(), newRPs.end() );

        //update ind_TS and ind_TE based on the amount of points that were inserted/removed during smoothening
        ind_TS += (newRPs.size() - hl_points.size());
        ind_TE = ind_TS + 1;
    }
    return true;
}

bool RouteSmoother::smoothenRoute(Route &route, const std::set<RoutePoint::RoutePointType> &filter, bool filterOut, size_t indFrom, int indTo, const Polygon &boundary, RouteSmoother::TimeHandlingStrategy THStrategy) const
{

    auto it = m_machines.find(route.machine_id);
    if(it == m_machines.end()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Machine not found");
        return false;
    }
    return smoothenRoute(it->second, route, filter, filterOut, indFrom, indTo, boundary, THStrategy);
}

bool RouteSmoother::smoothenRoute(const Machine &machine, Route &route, const std::set<RoutePoint::RoutePointType> &filter, bool filterOut, size_t indFrom, int indTo, const Polygon &boundary, RouteSmoother::TimeHandlingStrategy THStrategy) const
{
    return smoothenRoute(machine,
                         route,
                         [filter, filterOut](const RoutePoint& rp)->bool{
                             auto it = filter.find(rp.type);
                             return ( filterOut && it != filter.end() ) || ( !filterOut && it == filter.end() );
                         },
                         indFrom, indTo, boundary, THStrategy);
}

bool RouteSmoother::smoothenRoute(const Machine &machine, Route &route, const std::function<bool(const RoutePoint&)>& cutFunct, size_t indFrom, int indTo, const Polygon &boundary, RouteSmoother::TimeHandlingStrategy THStrategy) const
{
    struct DebugFileHandler{//just to close always in return
        std::ofstream& file;
        DebugFileHandler(std::ofstream& _file) : file(_file) {}
        ~DebugFileHandler(){
            if(file.is_open())
                file.close();
        }
    } debugFileHandler(m_debugFile);

    auto& route_points = route.route_points;
    if(indTo < 0)
        indTo = route_points.size()-1;
    if(indFrom > indTo
            || indFrom >= route_points.size()
            || indTo >= route_points.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid indexes");
        return false;
    }
    if(indTo - indFrom < 2)
        return true;//not enough points to smoothen

    if(!m_debugFilename.empty()){
        m_debugFile.open( m_debugFilename );
        if(m_debugFile.is_open())
            m_debugFile << "radius=" << double2string( machine.getTurningRadius() ) << std::endl;
    }

    size_t ind0 = indFrom;

    do {

        if(indTo - ind0 < 2)
            return true;//not enough remaining points --> nothing (more) to smoothen

        auto ind1 = ind0+1;

        while( ind1 <= indTo ){//search for the next cut
            if( cutFunct( route_points.at(ind1) ) )
                break;
            ++ind1;
        }

        if(ind1 > indTo)
            ind1 = indTo;//all remaining points

        if(ind1 - ind0 < 2){//not enough points to smoothen
            ind0 = ind1;
            continue;
        }

        size_t sizePrev = route.route_points.size();

        if(!smoothenRoutePart(route,
                              machine,
                              ind0,
                              ind1,
                              THStrategy))
            return false;

        ind0 = ind1+route.route_points.size()-sizePrev;
        indTo += route.route_points.size()-sizePrev;

    }while(1);


    return true;

}

void RouteSmoother::setDebugFile(const std::string &filename)
{
    m_debugFilename = filename;
}

bool RouteSmoother::smoothenRoutePart(Route &route, size_t route_ind0, size_t route_indn, const Machine &machine, RouteSmoother::TimeHandlingStrategy THStrategy, bool onlyNonOverloadingSegments) const
{
    auto& route_points = route.route_points;

    if(route_ind0 > route_indn
            || route_ind0 >= route_points.size()
            || route_indn >= route_points.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid route points' indexes");
        return false;
    }

    if(route_indn - route_ind0 < 2)//nothing to smoothen
        return true;


    size_t ind0_IF = route_ind0, ind1_IF = route_indn;//indexes holding the first and last point of a (sub)segment inside the field (from FIELD_ENTRY (if existent) to FIELD_EXIT (if existent))

    //find the first entry/exit point in the given part. If FIELD_ENTRY, assign the index to ind0_IF
    for(size_t i = route_ind0 ; i <= ind1_IF ; ++i){
        auto &rp = route_points.at(i);
        if( rp.type == RoutePoint::FIELD_ENTRY || rp.type == RoutePoint::FIELD_EXIT ){
            if( rp.type == RoutePoint::FIELD_ENTRY)
                ind0_IF = i;
            break;
        }
    }

    while(ind0_IF <= route_indn ){//smoothen all (sub)segments inside the field (from FIELD_ENTRY (if existent) to FIELD_EXIT (if existent))
        ind1_IF = std::min( RoutePoint::getNextIndByType(route_points, {RoutePoint::FIELD_EXIT}, ind0_IF ), route_indn );//set ind1_IF to the next FIELD_EXIT point index


        while(1){//smoothe all 'sharp' (sub-)subsegments in the current (inside-the-field) subsegment

            //get the indexes corresponding to the next sharp (sub-)subsegment (i.e. the subsegment that must be smoothen)
            size_t ind0, ind1, indnSeg;
            double ang0_s, ang1_s;
            getNextCornersIndexes( route_points, machine, ind0_IF, ind1_IF, onlyNonOverloadingSegments, ind0, ind1, indnSeg, ang0_s, ang1_s );
            if( ind0 > route_indn || ind1 > route_indn ){//no more subsegments to smoothen
                //ind1_IF = route_points.size();
                break;
            }

            //Check if extra points are needed to be added at the beginning of the current 'sharp' subsegment in order not to have undesired transitions between the current 'sharp' subsegment and the previous part of the route
            //The idea here is to extend the (original) initial segment to be smoothen with points previous to the segment so that the transition from the previous part of the route is also smooth
            //The 'sharp' segment keeps extending with the previous point as long as the distance between the first point of the original 'sharp' segment and the previous point to the extended ''sharp segment is lower than extraRefDist

            double distMult = 1;
            if(ang0_s < 90)
                distMult += ( 1-(ang0_s/90) );
            double extraRefDist = machine.getTurningRadius() * distMult;

            while( ind0 > route_ind0
                   && route_points.at(ind0).type != RoutePoint::FIELD_ENTRY
                   && route_points.at(ind0).type != RoutePoint::FIELD_EXIT){ //only extend with points of the selected route part and that lie inside the field
                auto extra = route_points.at(ind0-1);
                double dist = arolib::geometry::calc_dist( extra, route_points.at(ind0) );//distance between the first point of the (extended) segment to be smoothen and the previous point
                if( dist - extraRefDist > 1e-2 ){//obtain an intermidiate (interpolated) point between points at ind0 and ind0-1 so that the distance between the interpolated point and the point at ind0 is equal to extraRefDist. Update indexes and stop extending
                    int iTmp = ind0-1;
                    double dTime = route_points.at(ind0).time_stamp - extra.time_stamp;
                    if(dTime != 0){
                        dTime *= (extraRefDist/dist);
                        extra = route.calcPoint2( route_points.at(ind0).time_stamp-dTime, iTmp);
                    }
                    else
                        extra.point() = arolib::geometry::getPointInLineAtDist(route_points.at(ind0), extra, extraRefDist);

                    extra.type = getIntermediateRPType(route_points.at(ind0-1), route_points.at(ind0), RoutePoint::getDefaultRPType(machine), &extra.point());
                    route_points.insert( route_points.begin() + ind0, extra );

                    //new point added at the beginning --> update the other indexes
                    ++route_indn;
                    ++ind1;
                    ++ind1_IF;
                    break;
                }
                else if( extraRefDist - dist > 1e-2 ){//update extraRefDist and ind0, and keep extending
                    --ind0;
                    extraRefDist -= dist;
                }
                else //the distance between points is almost extraRefDist --> stop extending
                    break;
            }

            //Check if extra points are needed to be added at the end of the current 'sharp' subsegment in order not to have undesired transitions between the current 'sharp' subsegment and the next part of the route
            //Similar to what was done before, but extending at the end of the 'sharp' segment
            extraRefDist = machine.getTurningRadius() * m_extraDistMult;

            while( ind1+1 <= route_indn
                   && route_points.at(ind1).type != RoutePoint::FIELD_ENTRY
                   && route_points.at(ind1).type != RoutePoint::FIELD_EXIT ){
                auto extra = route_points.at(ind1+1);
                double dist = arolib::geometry::calc_dist( extra, route_points.at(ind1) );
                if( dist - extraRefDist > 1e-2 ){
                    int iTmp = ind1;
                    double dTime = extra.time_stamp - route_points.at(ind1).time_stamp;
                    if(dTime != 0){
                        dTime *= (extraRefDist/dist);
                        extra = route.calcPoint2( route_points.at(ind1).time_stamp+dTime, iTmp);
                    }
                    else
                        extra.point() = arolib::geometry::getPointInLineAtDist(route_points.at(ind1), extra, extraRefDist);

                    extra.type = getIntermediateRPType(route_points.at(ind1), route_points.at(ind1+1), RoutePoint::getDefaultRPType(machine), &extra.point());
                    route_points.insert( route_points.begin() + ind1+1, extra );
                    ++route_indn;
                    ++ind1_IF;
                    ++ind1;
                    break;
                }
                else if( extraRefDist - dist > 1e-2 ){
                    ++ind1;
                    extraRefDist -= dist;
                }
                else{
                    ++ind1;
                    break;
                }
            }

            //smoothen the extended 'sharp' subsegment
            std::vector<RoutePoint> newRPs;
            double delta_time = 0;
            if( ! smoothenSegment( route,
                                   ind0,
                                   ind1,
                                   machine,
                                   THStrategy,
                                   newRPs,
                                   delta_time ) ){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, 10, "Error smoothing segment with ind0 = ", ind0, " and ind1 = ", ind1);
                return false;
            }

            //replace the extended 'sharp' subsegment with the smooth subsegment and adjust timestamps accordingly
            size_t delta_ind = newRPs.size() - (1 + ind1 - ind0);
            route_points.erase( route_points.begin() + ind0, route_points.begin() + ind1+1 );
            for(size_t i = ind0 ; i < route_points.size() ; ++i)
                route_points.at(i).time_stamp += delta_time;
            route_points.insert( route_points.begin() + ind0 , newRPs.begin(), newRPs.end() );

            //update indexes for next iteration
            ind0_IF = ind1 + delta_ind;
            ind1_IF += delta_ind;
            route_indn += delta_ind;
        }

        ind0_IF = RoutePoint::getNextIndByType(route_points, {RoutePoint::FIELD_ENTRY}, ind1_IF+1 );//set ind0_IF to the index of the next FIELD_ENTRY point for next iteration
    }

    return true;
}

bool RouteSmoother::smoothenRoutePart(Route &route, const Machine &machine, size_t route_ind0, size_t route_indn, RouteSmoother::TimeHandlingStrategy THStrategy) const
{
    auto& route_points = route.route_points;

    if(route_ind0 > route_indn
            || route_ind0 >= route_points.size()
            || route_indn >= route_points.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid route points' indexes");
        return false;
    }

    if(route_indn - route_ind0 < 2)//nothing to smoothen
        return true;


    size_t ind0_ref = route_ind0;

    while(1){//smoothen all 'sharp' subsegments in the current segment


        //get the indexes corresponding to the next sharp subsegment (i.e. the subsegment that must be smoothen)
        size_t ind0, ind1, indnSeg;
        double ang0_s, ang1_s;
        getNextCornersIndexes( route_points, machine, ind0_ref, route_indn, ind0, ind1, indnSeg, ang0_s, ang1_s );
        if( ind0 > route_indn || ind1 > route_indn ){//no more subsegments to smoothen
            break;
        }


        //Check if extra points are needed to be added at the beginning of the current 'sharp' subsegment in order not to have undesired transitions between the current 'sharp' subsegment and the previous part of the route
        //The idea here is to extend the (original) initial segment to be smoothen with points previous to the segment so that the transition from the previous part of the route is also smooth
        //The 'sharp' segment keeps extending with the previous point as long as the distance between the first point of the original 'sharp' segment and the previous point to the extended ''sharp segment is lower than extraRefDist

        double distMultRef = ( ang0_s < 90 || ang1_s < 90 || ind0 != ind1 ? 2 : 1);
        double distMult = distMultRef;
        if(ang0_s < 90)
            distMult += ( 1-(ang0_s/90) );

        double extraRefDist = machine.getTurningRadius() * distMult;
        while( ind0 > route_ind0 ){ //only extend with points of the selected route part and that lie inside the field
            auto extra = route_points.at(ind0-1);
            double dist = arolib::geometry::calc_dist( extra, route_points.at(ind0) );//distance between the first point of the (extended) segment to be smoothen and the previous point
            if( dist - extraRefDist > 1e-2 ){//obtain an intermidiate (interpolated) point between points at ind0 and ind0-1 so that the distance between the interpolated point and the point at ind0 is equal to extraRefDist. Update indexes and stop extending
                int iTmp = ind0-1;
                double dTime = route_points.at(ind0).time_stamp - extra.time_stamp;
                if(dTime != 0){
                    dTime *= (extraRefDist/dist);
                    extra = route.calcPoint2( route_points.at(ind0).time_stamp-dTime, iTmp);
                }
                else
                    extra.point() = arolib::geometry::getPointInLineAtDist(route_points.at(ind0), extra, extraRefDist);

                extra.type = getIntermediateRPType(route_points.at(ind0-1), route_points.at(ind0), RoutePoint::getDefaultRPType(machine), &extra.point());
                route_points.insert( route_points.begin() + ind0, extra );

                //new point added at the beginning --> update the other indexes
                ++route_indn;
                ++ind1;
                ++indnSeg;
                break;
            }
            else if( extraRefDist - dist > 1e-2 ){//update extraRefDist and ind0, and keep extending
                --ind0;
                extraRefDist -= dist;
            }
            else //the distance between points is almost extraRefDist --> stop extending
                break;
        }

        //Check if extra points are needed to be added at the end of the current 'sharp' subsegment in order not to have undesired transitions between the current 'sharp' subsegment and the next part of the route
        //Similar to what was done before, but extending at the end of the 'sharp' segment

        distMult = distMultRef;
        if(ang1_s < 90)
            distMult += 1.5*( 1-(ang1_s/90) );

        extraRefDist = machine.getTurningRadius() * distMult;
        while( ind1+1 <= indnSeg ){
            auto extra = route_points.at(ind1+1);
            double dist = arolib::geometry::calc_dist( extra, route_points.at(ind1) );
            if( dist - extraRefDist > 1e-2 ){
                int iTmp = ind1;
                double dTime = extra.time_stamp - route_points.at(ind1).time_stamp;
                if(dTime != 0){
                    dTime *= (extraRefDist/dist);
                    extra = route.calcPoint2( route_points.at(ind1).time_stamp+dTime, iTmp);
                }
                else
                    extra.point() = arolib::geometry::getPointInLineAtDist(route_points.at(ind1), extra, extraRefDist);

                extra.type = getIntermediateRPType(route_points.at(ind1), route_points.at(ind1+1), RoutePoint::getDefaultRPType(machine), &extra.point());
                route_points.insert( route_points.begin() + ind1+1, extra );
                ++route_indn;
                ++ind1;
                break;
            }
            else if( extraRefDist - dist > 1e-2 ){
                ++ind1;
                extraRefDist -= dist;
            }
            else{
                ++ind1;
                break;
            }
        }

        //smoothen 'sharp' subsegment
        std::vector<RoutePoint> newRPs;
        double delta_time = 0;
        if( ! smoothenSegment( route,
                               machine,
                               ind0,
                               ind1,
                               THStrategy,
                               newRPs,
                               delta_time ) ){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, 10, "Error smoothing segment with ind0 = ", ind0, " and ind1 = ", ind1);
            return false;
        }

        //check timestamps
        if(ind0 != 0 && route_points.at(ind0-1).time_stamp > newRPs.front().time_stamp){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, 10, "Error in timestamps (prev) after smoothing segment with ind0 = ", ind0, " and ind1 = ", ind1);
            return false;
        }
        for(size_t i = 1 ; i < newRPs.size() ; ++i){
            if(newRPs.at(i-1).time_stamp > newRPs.at(i).time_stamp){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, 10, "Error in timestamps after smoothing segment with ind0 = ", ind0, " and ind1 = ", ind1);
                return false;
            }
        }

        //replace the extended 'sharp' subsegment with the smooth subsegment and adjust timestamps accordingly
        size_t delta_ind = newRPs.size() - (1 + ind1 - ind0);
        route_points.erase( route_points.begin() + ind0, route_points.begin() + ind1+1 );
        if(delta_time > 1e-9){
            for(size_t i = ind0 ; i < route_points.size() ; ++i)
                route_points.at(i).time_stamp += delta_time;
        }
        route_points.insert( route_points.begin() + ind0 , newRPs.begin(), newRPs.end() );

        //update indexes for next iteration
        ind0_ref = ind1 + delta_ind - ( ind1==indnSeg );
        route_indn += delta_ind;
    }

    return true;

}

bool RouteSmoother::smoothenRoute(Route &route, const Machine &machine, RouteSmoother::TimeHandlingStrategy THStrategy, bool onlyNonOverloadingSegments) const
{
    if(route.route_points.empty())
        return true;
    return smoothenRoutePart(route, 0, route.route_points.size()-1, machine, THStrategy, onlyNonOverloadingSegments);
}

bool RouteSmoother::smoothenSegment(const Route &route,
                                    size_t ind_0,
                                    size_t ind_n,
                                    const Machine &machine,
                                    TimeHandlingStrategy THStrategy,
                                    std::vector<RoutePoint> &smoothSegment, double &delta_time) const
{
    smoothSegment.clear();
    if(ind_0 > ind_n)
        return false;
    if(ind_n - ind_0 < 2){//nothing to smoothen
        smoothSegment.insert( smoothSegment.begin(), route.route_points.begin()+ind_0, route.route_points.begin()+ind_n+1);
        return true;
    }

    Route segmentRoute;//copy of the segment to be smoothen
    auto & segment = segmentRoute.route_points;
    segment.insert( segment.begin(), route.route_points.begin()+ind_0, route.route_points.begin()+ind_n+1);

    auto segPoints_us = RoutePoint::toPoints(segment);//unsampled segment
    arolib::geometry::unsample_linestring(segPoints_us, std::max( 1e-6, 0.01 * machine.getTurningRadius() ));

    if(segPoints_us.size() < 3){//nothing to smoothen
        smoothSegment.insert( smoothSegment.begin(), route.route_points.begin()+ind_0, route.route_points.begin()+ind_n+1);
        return true;
    }

    //auto nodes = getBSplineAdjustedNodes(segPoints_us, machine.working_width);

    std::vector<Point> nodes = segPoints_us;
    bool useBezier = segPoints_us.size() <= BezierMaxSampleSize;//if possible, use bezier
    if(useBezier){
        const size_t BezierSizeSweetSpot = 3;//from trial and error
        int refTmp = BezierSizeSweetSpot;
        do{//the bezier nodes will be a (suitable) sampled version of the unsampled segment
            nodes = arolib::geometry::sample_geometry(segPoints_us, arolib::geometry::getGeometryLength(segPoints_us)/refTmp);
            --refTmp;
        }
        while(nodes.size() > BezierSizeSweetSpot && segPoints_us.size() < nodes.size() && refTmp > 0);
    }
    else{
        double refTmp = 0.5;
        while(nodes.size() < 4){//the BSpline nodes will be a (suitable) sampled version of the unsampled segment
            nodes = arolib::geometry::sample_geometry(nodes, machine.working_width*refTmp);
            refTmp *= 0.5;
        }
    }

    for(size_t i = 0 ; i+1 < segment.size() ; ++i){//remove repeated consecutive points (in the route, it might happen that two consecutive points have th same location, but different timestamp)
        if( segment.at(i) == segment.at(i+1) ){
            segment.erase( segment.begin() + i+1 );
            --i;
        }
    }

    double length = arolib::geometry::getGeometryLength( RoutePoint::toPoints(segment) );//total length of the original segment
    double d = 0;//will hold the length between the first point of the originl segment and the current point under analysis

    //sample the original segment points (sampled points = refSamples)
    std::vector< std::pair<double, RoutePoint> > refSamples;//vector holding the sampled points and their respective normalized length from it to the first pointof the orginal segment (<norm. length, sample>)
    refSamples.emplace_back( std::make_pair(0, segment.front()) );//the first sample corresponds to the first routepoint of the original segment (with normalized length = 0)
    for(size_t i = 1 ; i < segment.size() ; ++i){//get the samples for each one of the (2-points) subsegment of the original segment
        auto & rp0 = segment.at(i-1);
        auto & rp1 = segment.at(i);
        double rl_0 = d / length; //normalized length from the first point to point at index i-1 of the orginal segment, relative to the total length of the original segment
        double d2 = arolib::geometry::calc_dist( rp0, rp1 );//distance between the two consecutive points
        d += d2;//length between the first point of the originl segment and point at index i
        const double segRes = machine.working_width*0.25;//resulution for the (2-points) subsegment sampling
        double rl_1 = d / length; //normalized length from the first point to point at index i of the orginal segment, relative to the total length of the original segment
        if( d2 > segRes ){//we need to sample the line formed by points i-1 and 1 (i.e. the (2-points) subsegment)

            RoutePoint rpRef = rp0;
            rpRef.type = getIntermediateRPType(rp0, rp1, RoutePoint::getDefaultRPType(machine));//all the sampled routepoints (excluding the last point = rp1) will have this route-point type

            size_t numSamples = d2 / segRes;
            double deltaRef = ( rl_1 - rl_0 ) / (numSamples-1);//normlized-length difference between the samples
            double deltaTime = ( rp1.time_stamp - rp0.time_stamp ) / (numSamples-1);//timestamp-difference between the samples
            for(size_t i = 1 ; i+1 < numSamples ; ++i){//add the normalized-length and sample routepoints, which hold their timestamp and type (note that only the timestamp and type are saved for now, but not the location, bunker_mass, etc...)
                rpRef.time_stamp = rp0.time_stamp + i*deltaTime;
                refSamples.emplace_back( std::make_pair(rl_0 + i*deltaRef, rpRef) );
            }
        }
        refSamples.emplace_back( std::make_pair(rl_1, rp1 ) );//add sample info corresponding to rp1
    }
    refSamples.back().first = 1;//the last sample corresponds to the last routepoint of the original segment (with normalized length = 1)

    std::multiset<double> refSamplesSet;//hold the sample relative-lengths (including repeated values) to be used to compute the smooth segment (this way, the smoothen segment can be mapped/projected to the computed samples)
    for(auto& item : refSamples)
        refSamplesSet.insert( refSamplesSet.end(), item.first );

    //compute the smooth segment
    std::vector<Point> smoothPoints;
    if(useBezier)
        smoothPoints = getBezierPoints(nodes, refSamplesSet);
    if(smoothPoints.empty())
        smoothPoints = getBSplinePoints(nodes, refSamplesSet, nodes.size()-1);

    //update the smooth segment, where the route-point type and timestamp correspond to the ones from the refSample, and the location coresponds to the one from the smoothen points
    smoothSegment.resize(smoothPoints.size());
    for(size_t i = 0 ; i < refSamples.size() ; ++i){
        smoothSegment.at(i) = refSamples.at(i).second;
        smoothSegment.at(i).point() = smoothPoints.at(i);
    }

    double length2 = arolib::geometry::getGeometryLength(smoothPoints);

    delta_time = 0;

    if(THStrategy == RECALC_PROP_TO_LENGTH){//recalculate the timestamps of the smooth segment based on the difference between the lengths of the smoothen and original segments (for example, if the smooth segment is twice as long as the original segment, the time needed to drive over the smooth segment must be twice the time originally needed to drive over the segment)
        double dTime0 = smoothSegment.back().time_stamp - smoothSegment.front().time_stamp;
        if(dTime0 != 0){
            double dTimeMult = ( dTime0 * length2 / length ) / dTime0;
            double tPrevOld = smoothSegment.front().time_stamp;
            for(size_t i = 1 ; i < smoothSegment.size() ; ++i){
                double dTime = smoothSegment.at(i).time_stamp - tPrevOld;
                tPrevOld = smoothSegment.at(i).time_stamp;
                smoothSegment.at(i).time_stamp = smoothSegment.at(i-1).time_stamp + dTime*dTimeMult;
                delta_time = (smoothSegment.at(i).time_stamp-tPrevOld);
            }
        }
    }

    return true;

}

bool RouteSmoother::smoothenSegment(const Route &route,
                                    const Machine &machine,
                                    size_t ind_0,
                                    size_t ind_n,
                                    RouteSmoother::TimeHandlingStrategy THStrategy,
                                    std::vector<RoutePoint> &smoothSegment,
                                    double &delta_time) const
{
    smoothSegment.clear();
    if(ind_0 > ind_n)
        return false;
    if(ind_n - ind_0 < 2){//nothing to smoothen
        smoothSegment.insert( smoothSegment.begin(), route.route_points.begin()+ind_0, route.route_points.begin()+ind_n+1);
        return true;
    }

    Route segmentRoute;//copy of the segment to be smoothen
    auto & segment = segmentRoute.route_points;
    segment.insert( segment.begin(), route.route_points.begin()+ind_0, route.route_points.begin()+ind_n+1);

    auto segPoints_us = RoutePoint::toPoints(segment);//unsampled segment
    arolib::geometry::unsample_linestring(segPoints_us, std::max( 1e-6, 0.01 * machine.getTurningRadius() ));

    if(segPoints_us.size() < 3){//nothing to smoothen
        smoothSegment.insert( smoothSegment.begin(), route.route_points.begin()+ind_0, route.route_points.begin()+ind_n+1);
        return true;
    }


    for(size_t i = 0 ; i+1 < segment.size() ; ++i){//remove repeated consecutive points (in the route, it might happen that two consecutive points have th same location, but different timestamp)
        if( segment.at(i) == segment.at(i+1) ){
            segment.erase( segment.begin() + i+1 );
            --i;
        }
    }


    //compute the smooth segment
    std::vector<Point> smoothPoints;
    if(smoothenSegmentBasedOnRadius(segPoints_us, smoothPoints, machine) && !smoothPoints.empty())
        smoothSegment =  projectRouteSegment(segment, smoothPoints, THStrategy);

    if(smoothSegment.empty()){

        double length = arolib::geometry::getGeometryLength( RoutePoint::toPoints(segment) );//total length of the original segment
        double d = 0;//will hold the length between the first point of the originl segment and the current point under analysis

        //sample the original segment points (sampled points = refSamples)
        std::vector< std::pair<double, RoutePoint> > refSamples;//vector holding the sampled points and their respective normalized length from it to the first pointof the orginal segment (<norm. length, sample>)
        refSamples.emplace_back( std::make_pair(0, segment.front()) );//the first sample corresponds to the first routepoint of the original segment (with normalized length = 0)
        for(size_t i = 1 ; i < segment.size() ; ++i){//get the samples for each one of the (2-points) subsegment of the original segment
            auto & rp0 = segment.at(i-1);
            auto & rp1 = segment.at(i);
            double rl_0 = d / length; //normalized length from the first point to point at index i-1 of the orginal segment, relative to the total length of the original segment
            double d2 = arolib::geometry::calc_dist( rp0, rp1 );//distance between the two consecutive points
            d += d2;//length between the first point of the originl segment and point at index i
            const double segRes = machine.working_width*0.25;//resulution for the (2-points) subsegment sampling
            double rl_1 = d / length; //normalized length from the first point to point at index i of the orginal segment, relative to the total length of the original segment
            if( d2 > segRes ){//we need to sample the line formed by points i-1 and 1 (i.e. the (2-points) subsegment)

                RoutePoint rpRef = rp0;
                rpRef.type = getIntermediateRPType(rp0, rp1, RoutePoint::getDefaultRPType(machine));//all the sampled routepoints (excluding the last point = rp1) will have this route-point type

                size_t numSamples = d2 / segRes;
                double deltaRef = ( rl_1 - rl_0 ) / (numSamples-1);//normlized-length difference between the samples
                double deltaTime = ( rp1.time_stamp - rp0.time_stamp ) / (numSamples-1);//timestamp-difference between the samples
                for(size_t i = 1 ; i+1 < numSamples ; ++i){//add the normalized-length and sample routepoints, which hold their timestamp and type (note that only the timestamp and type are saved for now, but not the location, bunker_mass, etc...)
                    rpRef.time_stamp = rp0.time_stamp + i*deltaTime;
                    refSamples.emplace_back( std::make_pair(rl_0 + i*deltaRef, rpRef) );
                }
            }
            refSamples.emplace_back( std::make_pair(rl_1, rp1 ) );//add sample info corresponding to rp1
        }
        refSamples.back().first = 1;//the last sample corresponds to the last routepoint of the original segment (with normalized length = 1)

        std::multiset<double> refSamplesSet;//hold the sample relative-lengths (including repeated values) to be used to compute the smooth segment (this way, the smoothen segment can be mapped/projected to the computed samples)
        for(auto& item : refSamples)
            refSamplesSet.insert( refSamplesSet.end(), item.first );
        //auto nodes = getBSplineAdjustedNodes(segPoints_us, machine.working_width);
        std::vector<Point> nodes = segPoints_us;
        bool useBezier = segPoints_us.size() <= BezierMaxSampleSize;//if possible, use bezier
        if(useBezier){
            const size_t BezierSizeSweetSpot = 3;//from trial and error
            int refTmp = BezierSizeSweetSpot;
            do{//the bezier nodes will be a (suitable) sampled version of the unsampled segment
                nodes = arolib::geometry::sample_geometry(segPoints_us, arolib::geometry::getGeometryLength(segPoints_us)/refTmp);
                --refTmp;
            }
            while(nodes.size() > BezierSizeSweetSpot && segPoints_us.size() < nodes.size() && refTmp > 0);
            smoothPoints = getBezierPoints(nodes, refSamplesSet);
        }
        else{
            double refTmp = 0.5;
            while(nodes.size() < 4){//the BSpline nodes will be a (suitable) sampled version of the unsampled segment
                nodes = arolib::geometry::sample_geometry(nodes, machine.working_width*refTmp);
                refTmp *= 0.5;
            }
        }
        if(smoothPoints.empty())
            smoothPoints = getBSplinePoints(nodes, refSamplesSet, nodes.size()-1);

        if(smoothPoints.empty())
            return false;

        //update the smooth segment, where the route-point type and timestamp correspond to the ones from the refSample, and the location coresponds to the one from the smoothen points
        smoothSegment.resize(smoothPoints.size());
        for(size_t i = 0 ; i < refSamples.size() ; ++i){
            smoothSegment.at(i) = refSamples.at(i).second;
            smoothSegment.at(i).point() = smoothPoints.at(i);
        }

        double length2 = arolib::geometry::getGeometryLength(smoothPoints);

        delta_time = 0;

        if(THStrategy == RECALC_PROP_TO_LENGTH){//recalculate the timestamps of the smooth segment based on the difference between the lengths of the smoothen and original segments (for example, if the smooth segment is twice as long as the original segment, the time needed to drive over the smooth segment must be twice the time originally needed to drive over the segment)
            double dTime0 = smoothSegment.back().time_stamp - smoothSegment.front().time_stamp;
            if(dTime0 != 0){
                double dTimeMult = ( dTime0 * length2 / length ) / dTime0;
                double tPrevOld = smoothSegment.front().time_stamp;
                for(size_t i = 1 ; i < smoothSegment.size() ; ++i){
                    double dTime = smoothSegment.at(i).time_stamp - tPrevOld;
                    tPrevOld = smoothSegment.at(i).time_stamp;
                    smoothSegment.at(i).time_stamp = smoothSegment.at(i-1).time_stamp + dTime*dTimeMult;
                    delta_time = (smoothSegment.at(i).time_stamp-tPrevOld);
                }
            }
        }

    }

    return true;

}

bool RouteSmoother::getEarFromRightAnglesSegment(const std::vector<Point> &points_in, const Machine &machine, std::vector<Point> &points_out) const
{
    points_out.clear();
    if(points_in.size() != 4)
        return false;

    double working_width = std::max( {0.0, machine.working_width, machine.width} );
    double radius = machine.getTurningRadius();

    Point refAng = arolib::geometry::extend_line(points_in.at(1), points_in.at(0), 1);

    double angBoundary = arolib::geometry::get_angle( points_in.at(1), points_in.at(2), points_in.at(0), points_in.at(3) );
    double angBoundary_abs = std::fabs( angBoundary );
    double angBoundary2_abs = std::fabs( arolib::geometry::get_angle( refAng, points_in.at(0), points_in.at(3) ) );

    Point p1 = points_in.at(0);
    Point p4 = points_in.at(3);
    Point ref1, ref2;

    if( fabs( arolib::geometry::calc_dist(points_in.at(1), points_in.at(2)) - arolib::geometry::calc_dist(p1, p4) ) > 1e-3 ){//not a rectangle --> find the points that make a rectangle
        Point tmp, extra;
        if( !arolib::geometry::getNormVector(p1, points_in.at(1), tmp) )
            return false;
        tmp = tmp + p1;
        if( arolib::geometry::get_intersection(p1, tmp, p4, points_in.at(2), extra, true, false) )
            p4 = extra;
        else{
            if( !arolib::geometry::getNormVector(p4, points_in.at(2), tmp) )
                return false;
            tmp = tmp + p4;
            if( !arolib::geometry::get_intersection(p4, tmp, p1, points_in.at(1), extra, true, false) )
                return false;
            p1 = extra;
        }
    }

    //double extraDist0 = machine.length > 0 ? 0.25 * machine.length : 0; //distance to ensure that the machine front (working tool) is outside the IF during turn
    double extraDist0 = 0.5 * working_width; //distance to ensure that the machine front (working tool) is outside the IF during turn

    double angBoundary_tmp = std::min( std::fabs(angBoundary), M_PI_4 );//@todo temporary workarround for when the angle for the next track changes drastically from angBoundary
    double extraDist = extraDist0 + 0.5 * working_width * tan( std::fabs(angBoundary_tmp) );
    extraDist = std::min(extraDist , extraDist0 + 2 * working_width);//@todo temporary workarround for when the angle for the next track changes drastically from angBoundary

    if(!arolib::geometry::getParallelVector(p1, points_in.at(1), ref1, extraDist))
        return false;
    if(!arolib::geometry::getParallelVector(p4, points_in.at(2), ref2, extraDist))
        return false;

    ref1 = ref1 + p1;
    ref2 = ref2 + p4;


    double mult_angle = 1;
    if( arolib::geometry::get_angle(p1, ref1, ref2) > 0 )
        mult_angle = -1;

    double dist = arolib::geometry::calc_dist(ref1, ref2);
    std::vector<Point> c1, c2, connection;
    if(dist >= 2*radius){
        Point pivot1 = arolib::geometry::getPointInLineAtDist(ref1, ref2, radius );
        Point pivot2 = arolib::geometry::getPointInLineAtDist(ref2, ref1, radius );
        auto ang1 = arolib::geometry::get_angle(ref2, ref1, false, false);
        c1 = arolib::geometry::create_circle(pivot1, radius, 9, ang1, M_PI_2 * mult_angle).points;
        c2 = arolib::geometry::create_circle(pivot2, radius, 9, ang1 + M_PI_2 * mult_angle, M_PI_2 * mult_angle).points;

        if(c1.front() == pivot1)
            pop_front(c1);
        if(c1.back() == pivot1)
            c1.pop_back();
        if(c2.front() == pivot2)
            pop_front(c2);
        if(c2.back() == pivot2)
            c2.pop_back();

        if( arolib::geometry::calc_dist( c1.back(), c2.front() ) > radius ){
            connection.emplace_back(c1.back());
            connection.emplace_back(c2.front());
            connection = arolib::geometry::sample_geometry(connection, radius);
            pop_front(connection);
            connection.pop_back();
        }

        points_out.emplace_back(p1);
        points_out.insert( points_out.end(), c1.begin(), c1.end() );
        points_out.insert( points_out.end(), connection.begin(), connection.end() );
        points_out.insert( points_out.end(), c2.begin(), c2.end() );
        points_out.emplace_back(p4);

    }
    else {//ear-like
        Point pivot, pCut1 = ref1, pCut2 = ref2;
        double g = angBoundary_abs / M_PI_2;
        double dist1 = (radius - 0.5 * dist) * g;

        if(dist1 < 0.1 * dist){//use simple dubbins
            arolib::geometry::DubinsParams dp1;

            dp1.p1 = ref1;
            dp1.p2 = ref2;

            dp1.rho1 = arolib::geometry::get_angle( p1, ref1 );
            dp1.rho2 = arolib::geometry::get_angle( ref2, p4 );

            dp1.type = arolib::geometry::DubinsParams::SHORTEST;

            points_out = calcDubinsPath(30, dp1, radius);

            if(points_out.size() < 3){
                points_out.clear();
                return false;
            }
        }
        else{
            Point connection12;
            double connDistHor = 0.5*dist + dist1;
            double distConnection12 = radius;
            if(angBoundary2_abs > M_PI_2){
                connection12 = arolib::geometry::getPointInLineAtDist(ref2, ref1, connDistHor);
            }
            else{
                connection12 = arolib::geometry::getPointInLineAtDist(ref1, ref2, connDistHor);
            }
            connDistHor = dist - connDistHor;
            if(connDistHor <= -radius)
                distConnection12 += 2 * radius;
            else
                distConnection12 += std::max(2.0, 1.01 * std::sqrt( 4 * radius * radius - std::pow(radius + connDistHor, 2) ) );

            Point vDir;
            if( !arolib::geometry::getParallelVector(points_in.at(0), points_in.at(1), vDir, distConnection12) )
                return false;

            arolib::geometry::DubinsParams dp1, dp2;

            dp1.p1 = ref1;
            dp1.p2 = connection12 + vDir;

            dp2.p1 = connection12 + vDir;
            dp2.p2 = ref2;

            dp1.rho1 = arolib::geometry::get_angle( p1, ref1 );
            dp1.rho2 = arolib::geometry::get_angle( points_in.at(1), points_in.at(2) );

            dp2.rho1 = dp1.rho2;
            dp2.rho2 = arolib::geometry::get_angle( ref2, p4 );

            dp1.type = dp2.type = arolib::geometry::DubinsParams::SHORTEST;

            auto pts1 = calcDubinsPath(15, dp1, radius);
            auto pts2 = calcDubinsPath(15, dp2, radius);

            if(pts1.size() < 2 || pts2.size() < 2)
                return false;

            points_out.reserve( pts1.size() + pts2.size() );
            points_out = pts1;
            points_out.insert(points_out.end(), pts2.begin()+1, pts2.end());
        }


        if( arolib::geometry::intersects(points_out) ){//@todo is this really necessary?
            points_out.clear();

            Point vDir, vDir2;
            if(!arolib::geometry::getParallelVector(points_in.at(0), points_in.at(1), vDir, radius ))
                return false;
            if(!arolib::geometry::getParallelVector(ref1, ref2, vDir2, radius ))
                return false;

            Point pivot;
            if(angBoundary2_abs > M_PI_2){
                pivot = arolib::geometry::getPointInLineAtDist(ref2, ref1, 0.5*dist + dist1);
            }
            else{
                pivot = arolib::geometry::getPointInLineAtDist(ref1, ref2, 0.5*dist + dist1);
            }

            std::vector<Point> controlPts = {p1};
            if(angBoundary > 0)
                controlPts.push_back( arolib::geometry::getPointInLineAtDist( p1, ref1, 0.5*extraDist ) );
            else
                controlPts.push_back( ref1 );

            //controlPts.push_back( pivot - vDir2);
            controlPts.push_back( pivot - vDir2 + vDir );
            controlPts.push_back( pivot + vDir2 + vDir );
            //controlPts.push_back( pivot + vDir2 );

            if(angBoundary > 0)
                controlPts.push_back( ref2 );
            else
                controlPts.push_back( arolib::geometry::getPointInLineAtDist( p4, ref2, 0.5*extraDist ) );
            controlPts.push_back( p4 );

            PathSmoother ppss(10, 170);
            points_out = ppss.smoothenPath(controlPts, radius);
            if(points_out.empty())
                return false;
        }
    }

    if(points_in.front() != p1)
        push_front(points_out, points_in.front());
    if(points_in.back() != p4)
        points_out.emplace_back(points_in.back());

    return true;
}

bool RouteSmoother::getEarFromRightAnglesSegment_old(const std::vector<Point> &points_in, const Machine &machine, std::vector<Point> &points_out) const
{
    points_out.clear();
    if(points_in.size() != 4)
        return false;

    double radius = std::max( {machine.getTurningRadius(), machine.working_width, machine.width} );

    Point p1 = points_in.at(0);
    Point p4 = points_in.at(3);
    Point ref1, ref2;

    if( fabs( arolib::geometry::calc_dist(points_in.at(1), points_in.at(2)) - arolib::geometry::calc_dist(p1, p4) ) > 1e-3 ){//not a rectangle --> find the points that make a rectangle
        Point tmp, extra;
        if( !arolib::geometry::getNormVector(p1, points_in.at(1), tmp) )
            return false;
        tmp = tmp + p1;
        if( arolib::geometry::get_intersection(p1, tmp, p4, points_in.at(2), extra, true, false) )
            p4 = extra;
        else{
            if( !arolib::geometry::getNormVector(p4, points_in.at(2), tmp) )
                return false;
            tmp = tmp + p4;
            if( !arolib::geometry::get_intersection(p4, tmp, p1, points_in.at(1), extra, true, false) )
                return false;
            p1 = extra;
        }
    }

    if(!arolib::geometry::getParallelVector(p1, points_in.at(1), ref1, radius))
        return false;
    if(!arolib::geometry::getParallelVector(p4, points_in.at(2), ref2, radius))
        return false;

    ref1 = ref1 + p1;
    ref2 = ref2 + p4;


    double mult_angle = 1;
    if( arolib::geometry::get_angle(p1, ref1, ref2) > 0 )
        mult_angle = -1;

    double dist = arolib::geometry::calc_dist(ref1, ref2);
    std::vector<Point> c1, c2, connection;
    if(dist >= 2*radius){
        Point pivot1 = arolib::geometry::getPointInLineAtDist(ref1, ref2, radius );
        Point pivot2 = arolib::geometry::getPointInLineAtDist(ref2, ref1, radius );
        auto ang1 = arolib::geometry::get_angle(ref2, ref1, false, false);
        c1 = arolib::geometry::create_circle(pivot1, radius, 9, ang1, M_PI_2 * mult_angle).points;
        c2 = arolib::geometry::create_circle(pivot2, radius, 9, ang1 + M_PI_2 * mult_angle, M_PI_2 * mult_angle).points;

        if(c1.front() == pivot1)
            pop_front(c1);
        if(c1.back() == pivot1)
            c1.pop_back();
        if(c2.front() == pivot2)
            pop_front(c2);
        if(c2.back() == pivot2)
            c2.pop_back();

        if( arolib::geometry::calc_dist( c1.back(), c2.front() ) > radius ){
            connection.emplace_back(c1.back());
            connection.emplace_back(c2.front());
            connection = arolib::geometry::sample_geometry(connection, radius);
            pop_front(connection);
            connection.pop_back();
        }
    }
    else {//ear-like
        Point pivot;
        double dist1 = std::sqrt( radius*radius - (0.5*dist)*(0.5*dist));
        if(!arolib::geometry::getParallelVector(p1, points_in.at(1), pivot, dist1))
            return false;
        pivot = pivot + arolib::geometry::getCentroid(ref1, ref2);
        auto ang1 = arolib::geometry::get_angle(pivot, ref1, false, false);
        auto angle_range =  2 * M_PI - std::fabs( arolib::geometry::get_angle(ref1, pivot, ref2, false, false) );
        c1 = arolib::geometry::create_circle(pivot, radius, std::max(9.0, angle_range * 18 * M_1_PI), ang1, angle_range * mult_angle).points;
        if(c1.front() == pivot)
            pop_front(c1);
        if(c1.back() == pivot)
            c1.pop_back();
    }


    points_out.emplace_back(p1);
    points_out.insert( points_out.end(), c1.begin(), c1.end() );
    points_out.insert( points_out.end(), connection.begin(), connection.end() );
    points_out.insert( points_out.end(), c2.begin(), c2.end() );
    points_out.emplace_back(p4);

    int side = 0;
    while (side++ < 2){
        double ang_prev = 0;
        int ind;
        for(ind = 2; ind < points_out.size()-2 ; ++ind){//smoothen the connection to the circle(s)
            double ang;
            if(side == 1)
                ang = arolib::geometry::get_angle(points_out.at(0), points_out.at(1), points_out.at(ind), true, false);
            else
                ang = arolib::geometry::get_angle(r_at(points_out, 0), r_at(points_out, 1), r_at(points_out, ind), true, false);

            if( ang * ang_prev < 0 ){//no point in the circle was found that creates an angle higher that 150°
                ind = -1;
                break;
            }
            if( fabs(ang) > 150 ){
                if(ind == 2)//nothing to smoothen
                    ind = -1;
                break;
            }
        }
        if(ind < 0)
            continue;
        std::vector<Point> nodes;
        if(side == 1)
           nodes.insert( nodes.end(), points_out.begin(), points_out.begin()+ind );
        else
            nodes.insert( nodes.end(), points_out.rbegin(), points_out.rbegin()+ind );
        auto smoothPoints = getBezierPoints(nodes, nodes.size());
        for(size_t i = 0 ; i < smoothPoints.size(); ++i){
            if(side == 1)
                points_out.at(i) = smoothPoints.at(i);
            else
                r_at(points_out, i) = smoothPoints.at(i);
        }

    }

    if(points_in.front() != p1)
        push_front(points_out, points_in.front());
    if(points_in.back() != p4)
        points_out.emplace_back(points_in.back());

    return true;
}

bool RouteSmoother::getNextCornersIndexes(const std::vector<RoutePoint> &route_points,
                                          const Machine &machine,
                                          size_t ind0, size_t ind1,
                                          bool onlyNonOverloadingSegments,
                                          size_t &ind0_s, size_t &ind1_s, size_t &indn,
                                          double &ang0_s, double &ang1_s) const
{
    if(ind0 >= route_points.size() || ind1 >= route_points.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Indexes out of range");
        return false;
    }
    indn = ind1;

    if(onlyNonOverloadingSegments){
        for(; ind0 < ind1 ; ++ind0){
            if( route_points.at(ind0).type != RoutePoint::HEADLAND )
                break;
        }
    }

    bool gotCorner = false;

    //find first corner
    for(; ind0+2 <= ind1 ; ++ind0){
        auto& rp0 = route_points.at(ind0);
        auto& rp1 = route_points.at(ind0+1);
        auto& rp2 = route_points.at(ind0+2);

        if(onlyNonOverloadingSegments
            && ( rp0.type == RoutePoint::HEADLAND
              || rp1.type == RoutePoint::HEADLAND
              || rp2.type == RoutePoint::HEADLAND ) )
            continue;

        double angle = std::fabs( arolib::geometry::get_angle( rp1, rp0, rp1, rp2, true ) );
        if( angle < 175 ){
            ind0_s = ind0+1;
            gotCorner = true;
            ang0_s = angle;
            break;
        }
    }

    if(!gotCorner){
        ind0_s = route_points.size();
        ind1_s = route_points.size();
        return true;
    }
    if(onlyNonOverloadingSegments){
        for(size_t i = ind0+2 ; i <= ind1 ; ++i){
            if( route_points.at(i).type == RoutePoint::HEADLAND ){
                ind1 = i-1;
                break;
            }
        }
    }

    ind1_s = ind0 = ind0_s;//from now on, ind1_s holds the index of the last 'sharp' corner found

    double lastAngle = 0;

    //find last corner
    for(; ind0+2 <= ind1 ; ++ind0){
        auto& rp0 = route_points.at(ind0);
        auto& rp1 = route_points.at(ind0+1);//pivot point / potential corner
        auto& rp2 = route_points.at(ind0+2);
        double angle = std::fabs( arolib::geometry::get_angle( rp1, rp0, rp1, rp2, true ) );//rp1 is the pivot
        if( angle < 175 ){//'sharp' corner
            ind1_s = ind0+1;//update the last found 'sharp' corner
            lastAngle = angle;
        }

        //check if the last corner found is "the one"
        double distMult = 2;
        if(lastAngle < 90)
            distMult += ( 1-(lastAngle/90) );

        //it is a sharp corner -> check if the length between the last corner found and the current corner is long enough (i.e. the route-regment following the previously found corner is smooth and long enough to 'cut' here)
        auto segPoints = RoutePoint::toPoints( std::vector<RoutePoint>(route_points.begin()+ind1_s, route_points.begin()+ind0+2) );//points from the last sharp corner found (ind1_s), to the current point
        if( arolib::geometry::getGeometryLength(segPoints) > distMult*machine.getTurningRadius() ){
            ang1_s = lastAngle;
            indn = ind0+2;
            return true;
        }

        //keep searching
    }
    return true;
}

bool RouteSmoother::getNextCornersIndexes(const std::vector<RoutePoint> &route_points, const Machine &machine,
                                          size_t ind0, size_t ind1,
                                          size_t &ind0_s, size_t &ind1_s, size_t &indn,
                                          double &ang0_s, double &ang1_s) const
{
    static auto lastSegmentIsLongEnough =
            [](const Machine &machine,
            double angle,
            const std::vector<Point>& segment)->bool{

        double distMult = 2;
        if(angle < 90)
            distMult += ( 1-(angle/90) );
        return arolib::geometry::getGeometryLength(segment) > distMult*machine.getTurningRadius();
    };

    if(ind0 >= route_points.size() || ind1 >= route_points.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Indexes out of range");
        return false;
    }
    indn = ind1;

    bool gotCorner = false;

    size_t deltaInd1 = 1, deltaInd2 = 1;

    //find first corner
    while(ind0+deltaInd1+deltaInd2 <= ind1){
        auto& rp0 = route_points.at(ind0);
        auto& rp1 = route_points.at(ind0+deltaInd1);
        auto& rp2 = route_points.at(ind0+deltaInd1+deltaInd2);

        //check for repeated points
        if( arolib::geometry::calc_dist(rp0, rp1) < 1e-9 ){
            ++deltaInd1;
            continue;
        }
        if( arolib::geometry::calc_dist(rp1, rp2) < 1e-9 ){
            ++deltaInd2;
            continue;
        }

        ind0+=deltaInd1;
        deltaInd1 = deltaInd2;
        deltaInd2 = 1;

        double angle = std::fabs( arolib::geometry::get_angle( rp1, rp0, rp1, rp2, true ) );
        if( angle < 175 ){
            ind0_s = ind0;
            ang0_s = angle;
            gotCorner = true;

            if( lastSegmentIsLongEnough(machine, angle, {rp1.point(), rp2.point()}) ){
                ind1_s = ind0_s;
                ang1_s = angle;
                indn = ind0+deltaInd1;
                return true;
            }
            break;
        }
    }

    if(!gotCorner){
        ind0_s = route_points.size();
        ind1_s = route_points.size();
        return true;
    }

    ind1_s = ind0 = ind0_s;//from now on, ind1_s holds the index of the last 'sharp' corner found
    double lastAngle = 0;

    //find last corner
    deltaInd1 = deltaInd2 = 1;
    while(ind0+deltaInd1+deltaInd2 <= ind1){
        auto& rp0 = route_points.at(ind0);
        auto& rp1 = route_points.at(ind0+deltaInd1);//pivot point / potential corner
        auto& rp2 = route_points.at(ind0+deltaInd1+deltaInd2);

        //check for repeated points
        if( arolib::geometry::calc_dist(rp0, rp1) < 1e-9 ){
            ++deltaInd1;
            continue;
        }
        if( arolib::geometry::calc_dist(rp1, rp2) < 1e-9 ){
            ++deltaInd2;
            continue;
        }

        ind0+=deltaInd1;
        deltaInd1 = deltaInd2;
        deltaInd2 = 1;

        double angle = std::fabs( arolib::geometry::get_angle( rp1, rp0, rp1, rp2, true ) );//rp1 is the pivot
        if( angle < 175 ){//'sharp' corner
            ind1_s = ind0;//update the last found 'sharp' corner
            lastAngle = angle;
        }

        //check if the last corner found is "the one"
        if( lastSegmentIsLongEnough( machine, angle, RoutePoint::toPoints( std::vector<RoutePoint>(route_points.begin()+ind1_s, route_points.begin()+ind0+deltaInd1) ) ) ){
            ang1_s = lastAngle;
            indn = ind0+deltaInd1;
            return true;
        }
    }
    return true;

}

bool RouteSmoother::smoothenSegmentBasedOnRadius(const std::vector<Point> &points_us, std::vector<Point> &smoothPoints, const Machine &machine) const
{

    if(m_debugFile.is_open()){
        for(auto& p : points_us){
            m_debugFile << double2string(p.x) << "," << double2string(p.y) << ";";
        }
        m_debugFile << std::endl;
    }


    PathSmoother ppss(15, 150);
    smoothPoints = ppss.smoothenPath(points_us, machine.getTurningRadius());

    return !smoothPoints.empty() || points_us.empty();
}

std::vector<Point> RouteSmoother::getBSplineAdjustedNodes(std::vector<Point> nodes, double workingwidth) const{
    if(nodes.size() < 3)
        return nodes;

    arolib::geometry::unsample_linestring(nodes, 0.001 * workingwidth);

    std::vector<Point> ret = nodes;
    const double m = workingwidth * 1;

    std::vector< std::pair<size_t, Point> > extra;
    struct {
        bool add = false;
        double angle;
        double angle_r;
        double r;
    } extra0;

    for(size_t i = 1 ; i+1 < nodes.size() ; ++i){//for each of the points of the unsampled segment (excluding the first and last points)...
        auto &p0 = nodes.at(i-1);
        auto &p1 = nodes.at(i);
        auto &p2 = nodes.at(i+1);
        //        if( arolib::geometry::calc_dist(p1, p2) > 2*workingwidth )
        //            continue;
        double angle_r = arolib::geometry::get_angle(p1, p0, p1, p2);
        double angle = std::fabs( angle_r );

        //double mult = std::min( 0.0, 0.85*angle*angle - 3.35*angle + 2.125 );
        //double mult = std::min( 0.0, 0.68*angle*angle - 2.87*angle + 1.84 );
        //double mult = std::min( 0.0, std::max( -1.0, angle / M_PI_2 - 2 ) );
        double mult = std::min( 0.0, std::max( -1.5, angle / M_PI_2 - 2 ) );

        //        double mult;
        //        if(angle < M_PI_2)
        //            //mult = std::min( 0.0, std::max( -1.0, -0.5 * angle / M_PI_2 - 0.5 ) );
        //            mult = -1;
        //        else
        //            mult = std::min( 0.0, std::max( -1.0, angle / M_PI_2 - 2 ) );

        if(angle < M_PI_2)
            mult = std::min( -0.2, std::max( -1.5, angle / M_PI_2 - 2 ) );
        else
            mult = std::min( -0.2, std::max( -1.0, 1.27*angle - 3 ) );


        double r = m * mult;

        if(angle < M_PI_2){
            double b = 3;

            auto r_extra = b * r * (M_PI_2-angle) / M_PI_2;
            //r -= r_extra;
            //r = std::min(-0.5*r, r-r_extra);
            r = std::min(0.0, r-r_extra);

            auto p_rot = arolib::geometry::rotate(p1, p0, angle_r/2);
            arolib::geometry::getParallelVector(p1, p_rot, ret.at(i), r);
            ret.at(i) += p1;

            if(i != 1){
                auto d_ext = std::min( 2*workingwidth, 0.5 * arolib::geometry::calc_dist(ret.at(i), ret.at(i-1)) );
                auto p_extra = arolib::geometry::getPointInLineAtDist(ret.at(i), ret.at(i-1), d_ext);
                auto v_norm = arolib::geometry::rotate(ret.at(i), ret.at(i-1), M_PI_2 * angle_r/angle) - ret.at(i);
                arolib::geometry::setVectorLength(v_norm, r_extra);
                p_extra += v_norm;
                extra.emplace_back( std::make_pair(i, p_extra) );
            }
            else{
                extra0.add = true;
                extra0.angle = angle;
                extra0.angle_r = angle_r;
                extra0.r = r_extra;
            }
        }
        else{
            auto p_rot = arolib::geometry::rotate(p1, p0, angle_r/2);
            arolib::geometry::getParallelVector(p1, p_rot, ret.at(i), r);
            ret.at(i) += p1;
        }

    }

    if(extra0.add){
        auto l = 0.5 * arolib::geometry::calc_dist(ret.at(1), ret.at(2));
        if( l > 2.1*workingwidth || extra.empty() || extra.front().first != 2 ){
            auto d_ext = std::min( 2*workingwidth, l );
            auto p_extra = arolib::geometry::getPointInLineAtDist(ret.at(1), ret.at(2), d_ext);
            auto v_norm = arolib::geometry::rotate(ret.at(2), ret.at(1), M_PI_2 * extra0.angle_r/extra0.angle) - ret.at(2);
            arolib::geometry::setVectorLength(v_norm, extra0.r);
            p_extra += v_norm;
            extra.insert( extra.begin(), std::make_pair(2, p_extra) );
        }

    }

    for(size_t i = 0 ; i < extra.size() ; ++i)
        ret.insert( ret.begin() + extra.at(i).first + i, extra.at(i).second );

    return arolib::geometry::sample_geometry(ret, workingwidth);
}

RoutePoint::RoutePointType RouteSmoother::getIntermediateRPType(const RoutePoint &rp0, const RoutePoint &rp1, RoutePoint::RoutePointType defaultWorkingType, const Point *p)
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
        if( arolib::geometry::calc_dist(*p, rp1) < arolib::geometry::calc_dist(*p, rp0) )
            rpType = rp1.type;
    }

    return rpType;
}

std::vector<RoutePoint> RouteSmoother::projectRouteSegment(const std::vector<RoutePoint> &segment, const std::vector<Point> &newPoints, TimeHandlingStrategy THStrategy)
{
    std::vector<RoutePoint> points_out;

    if(segment.size() < 2 || newPoints.size() < 2)
        return points_out;

    double length_in = arolib::geometry::getGeometryLength( RoutePoint::toPoints(segment) );//total length of the original segment
    double length_out = arolib::geometry::getGeometryLength( newPoints );//total length of the new segment

    if(length_in <= 0 || length_out <= 0)
        return points_out;

    double d = 0;//will hold the length between the first point of the originl segment and the current point under analysis


    double minDist_in = std::numeric_limits<double>::max();
    //sample the original segment points (sampled points = refSamples)
    std::vector< std::pair<double, RoutePoint> > refSamples;//vector holding the sampled points and their respective normalized length from it to the first pointof the orginal segment (<norm. length, sample>)
    refSamples.emplace_back( std::make_pair(0, segment.front()) );//the first sample corresponds to the first routepoint of the original segment (with normalized length = 0)
    for(size_t i = 1 ; i < segment.size() ; ++i){//get the samples for each one of the (2-points) subsegment of the original segment
        auto & rp0 = segment.at(i-1);
        auto & rp1 = segment.at(i);
        double d2 = arolib::geometry::calc_dist( rp0, rp1 );//distance between the two consecutive points
        d += d2;//length between the first point of the originl segment and point at index i
        double rl_1 = d / length_in; //normalized length from the first point to point at index i of the orginal segment, relative to the total length of the original segment
        refSamples.emplace_back( std::make_pair(rl_1, rp1 ) );//add sample info corresponding to rp1
        minDist_in = std::min(minDist_in, d2);
    }
    refSamples.back().first = 1;//the last sample corresponds to the last routepoint of the original segment (with normalized length = 1)


    points_out.push_back( segment.front() );
    points_out.back().point() = newPoints.front();

    d = 0;//will hold the length between the first point of the new segment and the current point under analysis
    size_t ind_ref = 1;
    double deltaTime_in = segment.back().time_stamp - segment.front().time_stamp;
    double deltaTime_out = deltaTime_in * length_out / length_in;
    const double distTH = minDist_in * 0.05;

    for(size_t i = 1 ; i < newPoints.size() && ind_ref < refSamples.size() ; ++i){//get the samples for each one of the (2-points) subsegment of the original segment
        auto & p0 = newPoints.at(i-1);
        auto & p1 = newPoints.at(i);
        double d2 = arolib::geometry::calc_dist( p0, p1 );//distance between the two consecutive points
        d += d2;//length between the first point of the segment and point at index i
        double rl_1 = d / length_out; //normalized length from the first point to point at index i of the orginal segment, relative to the total length of the original segment

        RoutePoint rpNew;

        double distToLast = arolib::geometry::calc_dist( points_out.back(), p1 );
        double distToRef = arolib::geometry::calc_dist( points_out.back(), refSamples.at(ind_ref).second );

        if(rl_1 <= refSamples.at(ind_ref).first){
//            double distToRef_rel = refSamples.at(ind_ref).first - rl_1;
//            double distToRef2 = distToRef_rel * length_out;
//            if(distToRef2 < distTH){//add a route point at the location of p1 with the timestamp (and other properties) of the refSample
            if(distToRef < distTH){//add a route point at the location of p1 with the timestamp (and other properties) of the refSample
                rpNew = refSamples.at(ind_ref++).second;
                rpNew.point() = p1;
            }
            else if( distToLast < distTH )//too close to the previous route point. do not add
                continue;
            else{//add a route point at the location of p1 with the relative timestamp (and other properties)
                int iTmp = ind_ref;

                double time_stamp = refSamples.at(ind_ref).second.time_stamp;
                if(ind_ref > 0){
                    iTmp = ind_ref-1;
                    double deltaRL = refSamples.at(ind_ref).first - refSamples.at(ind_ref-1).first;
                    if(deltaRL != 0){
                        deltaRL = (refSamples.at(ind_ref).first - rl_1) / deltaRL;
                        if(deltaRL > 1)//a previous point was forced to a previous refSample --> do not add
                            continue;
                        time_stamp = deltaRL * refSamples.at(ind_ref-1).second.time_stamp + (1-deltaRL) * refSamples.at(ind_ref).second.time_stamp;
                    }
                }

                rpNew = Route::calcPoint2(segment, time_stamp, iTmp);
                rpNew.point() = p1;
            }

        }
        else if(i+1 < newPoints.size()){
            d -= d2;
            --i;

            if(distToRef < distTH){//replace last route point type with the one of the refSample
                points_out.back().type = refSamples.at(ind_ref++).second.type;
                continue;
            }

            //add sample between p0 and p1 with the timestamp (and other properties) of the refSample
            double dist = ( rl_1 - refSamples.at(ind_ref).first ) * length_out;
            rpNew = refSamples.at(ind_ref++).second;
            rpNew.point() = arolib::geometry::getPointInLineAtDist( p1, p0, dist );
        }
        else{
            rpNew = refSamples.back().second;
            rpNew.point() = p1;
            i = newPoints.size();
        }

        if(THStrategy == RECALC_PROP_TO_LENGTH && deltaTime_in != 0){//@todo the time recalculation should be done with respect to each of the (sub) segments lengths and not to the complete lengths, since the duration of the subsegments is not necesarily proportional to the lengths
            double dist = arolib::geometry::calc_dist( rpNew, points_out.back() );
            rpNew.time_stamp = points_out.front().time_stamp + deltaTime_out * dist / length_out;
        }

        points_out.push_back(rpNew);
    }


    if(points_out.back().point() != newPoints.back()){
        points_out.push_back( segment.back() );
        points_out.back().point() = newPoints.back();
    }

    return points_out;
}

}
