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

#include "arolib/geometry/field_geometry_processing.hpp"

namespace arolib{

namespace geometry{

std::vector<Point> getBestRoadConnection(const Point &pStart, const Point &pFinish, const std::vector<Linestring> &roads, double res )
{
    std::vector<Point> ret;

    double minDist_start = std::numeric_limits<double>::max();
    double minDist_finish = std::numeric_limits<double>::max();
    for(size_t i = 0 ; i < roads.size() ; ++i){
        const auto&road = roads.at(i);
        if(road.points.size() < 2)
            continue;
        double dist_start = calc_dist_to_linestring( road.points, pStart, false );
        double dist_finish = calc_dist_to_linestring( road.points, pFinish, false );

        if( minDist_start + minDist_finish > dist_start + dist_finish ){
            minDist_start = dist_start;
            minDist_finish = dist_finish;
            ret = road.points;
        }
    }
    if(ret.empty())
        return ret;

    int indStart = addSampleToGeometryClosestToPoint(ret, pStart, 1);
    size_t sizePrev = ret.size();
    int indFinish = addSampleToGeometryClosestToPoint(ret, pFinish, 1);

    if(sizePrev < ret.size() && indStart >= indFinish )//update indStart if a point was added before it
        indStart++;

    if(indStart == indFinish)
        ret = { ret.at(indStart) };

    if(ret.size() == 1)
        return ret;


    bool reverse = indStart > indFinish;
    if(reverse)
        std::swap(indStart, indFinish);

    if( calc_dist(ret.front(), ret.back()) < 1e-3  )//closed road/polygon
    {
        double l1 = getGeometryLength(ret, indStart, indFinish);
        double l2 = getGeometryLength(ret) - l1;
        if(l1 <= l2){
            ret.erase( ret.begin()+indFinish+1, ret.end() );
            ret.erase( ret.begin(), ret.begin()+indStart );
        }
        else{
            std::vector<Point> tmp(ret.begin(), ret.begin()+indStart+1);
            ret.erase( ret.begin(), ret.begin()+indFinish );
            ret.insert( ret.end(), tmp.begin()+1, tmp.end() );
            reverse = !reverse;
        }
    }
    else{
        ret.erase( ret.begin()+indFinish+1, ret.end() );
        ret.erase( ret.begin(), ret.begin()+indStart );
    }

    if(reverse)
        std::reverse(ret.begin(), ret.end());

    if(res > 0)
        ret = sample_geometry(ret, res);

    return ret;
}

bool getConnectionToHeadland(std::vector<Point> &headland,
                             const Point &p0_1,
                             const Point &p1_1,
                             const Point &p0_2,
                             const Point &p1_2,
                             std::vector<Point> &connection,
                             const HeadlandConnectionStrategy &strategy,
                             const bool &addConnectionToHeadland,
                             const double unsamplingTolerance)
{
    if(headland.size() < 2)
        return false;

    auto headland_tmp = headland;

    size_t indStart = 0, indEnd = 0;
    size_t indPrev1, indNext1, indPrev2, indNext2;
    Point pStart, pEnd;
    connection.clear();

    int count = 0;
    while(count < 2){
        const Point *p0 = &p0_1;
        const Point *p1 = &p1_1;
        size_t *ind = &indStart;
        size_t *indPrev = &indPrev1;
        size_t *indNext = &indNext1;
        Point *pnt = &pStart;
        if(count == 1){
            p0 = &p1_2;
            p1 = &p0_2;
            ind = &indEnd;
            indPrev = &indPrev2;
            indNext = &indNext2;
            pnt = &pEnd;
        }
        double minDist = std::numeric_limits<double>::max();

        if(strategy == HeadlandConnectionStrategy::MIN_DIST_TO_HEADLAND_SEGMENT){
            addSampleToGeometryClosestToPoint(headland_tmp, *p1);
            *pnt = *p1;
        }
        else{
            for(size_t ind1 = 0 ; ind1 < headland_tmp.size() ; ++ind1){
                size_t ind2 = ind1+1;
                if(ind2 == headland_tmp.size())
                    ind2 = 0;
                Point intersection;
                if( !get_intersection(*p0,
                                      *p1,
                                      headland_tmp.at(ind1),
                                      headland_tmp.at(ind2),
                                      intersection,
                                      true,
                                      false) )
                    continue;
                if(getLocationInLine(*p0, *p1, intersection) <= 0)
                    continue;
                double dist = calc_dist(intersection, *p1);
                double dist2 = calc_dist(intersection, *p0);
                if( dist2 < dist )
                    continue;
                if(dist > minDist)
                    continue;
                minDist = dist;
                *pnt = intersection;
            }
            addSampleToGeometryClosestToPoint(headland_tmp, *pnt);
        }

        ++count;
    }

    connection = getShortestGeometryPart(headland_tmp, pStart, pEnd);

    if(addConnectionToHeadland)
        headland = headland_tmp;

    unsample_linestring(connection, unsamplingTolerance);

    return true;
}

bool getConnectionToHeadland(std::vector<Point> &headland,
                             const Point &p0,
                             const Point &p1,
                             Point &connection,
                             const HeadlandConnectionStrategy &strategy,
                             const bool &addConnectionToHeadland)
{
    if(headland.size() < 2)
        return false;

    bool isClosed = ( headland.front() == headland.back() );
    if(isClosed && headland.size() == 2){
        connection = headland.front();
        return true;
    }

    size_t ind = 0;
    size_t indPrev, indNext;
    double minDist = std::numeric_limits<double>::max();
    for(size_t ind1 = 0 ; ind1 + isClosed < headland.size() ; ++ind1){
        size_t ind2 = ind1+1;
        if(ind2 == headland.size())
            ind2 = 0;

        if(strategy == HeadlandConnectionStrategy::KEEP_TRACK_SEGMENT_DIRECTION){
            Point intersection;
            if( !get_intersection(p0,
                                  p1,
                                  headland.at(ind1),
                                  headland.at(ind2),
                                  intersection,
                                  true,
                                  false) )
                continue;
            if(getLocationInLine(p0, p1, intersection) <= 0)
                continue;
            double dist = calc_dist(intersection, p1);
            if( calc_dist(intersection, p0) < dist )
                continue;
            if(dist > minDist)
                continue;
            minDist = dist;
            connection = intersection;
        }
        else{

            double dist = calc_dist_to_line(headland.at(ind1),
                                            headland.at(ind2),
                                            p1,
                                            false,
                                            false);

            if( std::fabs(dist) > minDist)
                continue;
            minDist = std::fabs(dist);


            connection = p1 + calc_vector_to_line(headland.at(ind1),
                                                  headland.at(ind2),
                                                  p1,
                                                  false);
        }
        if( calc_dist(connection, headland.at(ind1)) < calc_dist(connection, headland.at(ind2)))
            ind = ind1;
        else
            ind = ind2;
        indPrev = ind1;
        indNext = ind2;
    }


    if(addConnectionToHeadland){
        const double eps = calc_dist(headland.at(indPrev), headland.at(indNext)) * 0.00001;
        if( calc_dist( headland.at(indPrev), connection ) < eps ){
            headland.at(indPrev) = connection;
            if(isClosed && indPrev == 0)
                headland.back() = connection;
        }
        else if ( calc_dist( headland.at(indNext), connection ) < eps ){
            headland.at(indNext) = connection;
            if(isClosed && (indPrev+1) == headland.size())
                headland.front() = connection;
        }
        else{
            headland.insert( headland.begin()+indPrev+1, connection );
        }

    }

    return true;
}

HeadlandConnectionStrategy intToHeadlandConnectionStrategy(int value)
{
    if(value == HeadlandConnectionStrategy::MIN_DIST_TO_HEADLAND_SEGMENT)
        return HeadlandConnectionStrategy::MIN_DIST_TO_HEADLAND_SEGMENT;
    else if(value == HeadlandConnectionStrategy::KEEP_TRACK_SEGMENT_DIRECTION)
        return HeadlandConnectionStrategy::KEEP_TRACK_SEGMENT_DIRECTION;

    throw std::invalid_argument( "The given value does not correspond to any HeadlandConnectionStrategy" );
}

Subfield getSubieldFromRoutePoints(std::vector<RoutePoint> route_points, int subfield_id)
{
    Subfield sf;
    sf.id = subfield_id;

    double minX = std::numeric_limits<double>::max(), maxX = std::numeric_limits<double>::lowest();
    double minY = minX, maxY = maxX;
    int count = 0;

    std::set<RoutePoint> resourcePoints;
    std::set<RoutePoint> faps;

    for(auto &rp : route_points){
        if( rp.type == RoutePoint::INITIAL_POSITION )
            continue;
        if( rp.type == RoutePoint::RESOURCE_POINT ){
            resourcePoints.insert(rp);
            continue;
        }
        if( rp.type == RoutePoint::FIELD_ENTRY ||
                rp.type == RoutePoint::FIELD_EXIT ){
            faps.insert(rp);
            continue;
        }
        ++count;

        minX = std::min( minX , rp.x );
        maxX = std::max( maxX , rp.x );
        minY = std::min( minY , rp.y );
        maxY = std::max( maxY , rp.y );
    }
    if( count < 2 )
        return Subfield();

    sf.boundary_outer.points.emplace_back( Point(minX, minY) );
    sf.boundary_outer.points.emplace_back( Point(minX, maxY) );
    sf.boundary_outer.points.emplace_back( Point(maxX, maxY) );
    sf.boundary_outer.points.emplace_back( Point(maxX, minY) );
    sf.boundary_outer.points.emplace_back( Point(minX, minY) );

    count = 0;
    sf.resource_points.clear();
    for(auto& rp : resourcePoints){
        ResourcePoint p;
        p.id = count;
        p.point() = rp.point();
        sf.resource_points.emplace_back(p);
    }

    count = 0;
    sf.access_points.clear();
    for(auto& rp : faps){
        FieldAccessPoint p;
        p.id = count;
        p.point() = rp.point();
        sf.access_points.emplace_back(p);
    }

    return sf;
}

Field getFieldFromSubfields(std::vector<Subfield> const& subfields)
{
    Field f;
    f.subfields = subfields;

    if(subfields.empty())
        return f;
    if(subfields.size() == 1){
        f.outer_boundary = subfields.front().boundary_outer;
        return f;
    }

    double minX_f = std::numeric_limits<double>::max(), maxX_f = std::numeric_limits<double>::lowest();
    double minY_f = minX_f, maxY_f = maxX_f;

    for(auto& sf : subfields){
        double minX, maxX, minY, maxY;
        getPolygonLimits(sf.boundary_outer, minX, maxX, minY, maxY);

        minX_f = std::min( minX_f , minX );
        maxX_f = std::max( maxX_f , maxX );
        minY_f = std::min( minY_f , minY );
        maxY_f = std::max( maxY_f , maxY );
    }

    f.outer_boundary.points.clear();
    f.outer_boundary.points.emplace_back( Point(minX_f, minY_f) );
    f.outer_boundary.points.emplace_back( Point(minX_f, maxY_f) );
    f.outer_boundary.points.emplace_back( Point(maxX_f, maxY_f) );
    f.outer_boundary.points.emplace_back( Point(maxX_f, minY_f) );
    f.outer_boundary.points.emplace_back( Point(minX_f, minY_f) );

    return f;

}

}

}
