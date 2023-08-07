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

std::map<size_t, std::set<size_t> > getAdjacentTracksList(const std::vector<Track> &tracks, bool withRepeatedEntries)
{
    std::map<size_t, std::set<size_t> > ret;

    for(size_t i = 0 ; i+1 < tracks.size() ; ++i){
        for(size_t j = i+1 ; j < tracks.size() ; ++j){
            if(areTracksAdjacent(tracks.at(i), tracks.at(j))){
                ret[i].insert(j);
                if(withRepeatedEntries)
                    ret[j].insert(i);
            }
        }
    }

    return ret;

}

bool areTracksAdjacent(const std::map<size_t, std::set<size_t> > &list, size_t indTrack1, size_t indTrack2)
{
    if(indTrack1 == indTrack2)
        return false;

    if(indTrack1 > indTrack2)
        std::swap(indTrack1, indTrack2);

    auto it = list.find(indTrack1);
    if(it != list.end()){
        if( it->second.find(indTrack2) != it->second.end() )
            return true;
    }

    it = list.find(indTrack2);
    if(it != list.end()){
        if( it->second.find(indTrack1) != it->second.end() )
            return true;
    }

    return false;
}

bool areTracksAdjacent(const Track &t1, const Track &t2)
{

    if(t1.points.size() < 2 || t2.points.size() < 2)
        return false;

    if( (t1.width < 1e-9 && t1.boundary.points.size() < 4) ||
            (t2.width < 1e-9 && t2.boundary.points.size() < 4) )
        return false;

    if( t1.boundary.points.size() > 3 && t2.boundary.points.size() > 3 ){
        auto b1 = t1.boundary.points;
        auto b2 = t2.boundary.points;

        unsample_linestring(b1);
        unsample_linestring(b2);

        for(auto& p : b1)
            addSampleToGeometryClosestToPoint(b2, p);
        for(auto& p : b2)
            addSampleToGeometryClosestToPoint(b1, p);

        for(auto& p1 : b1){
            for(auto& p2 : b2){
                if(calc_dist(p1, p2) < 1e-2)
                    return true;
            }
        }
    }

    auto pts1 = t1.points;
    auto pts2 = t2.points;

    unsample_linestring(pts1);
    unsample_linestring(pts2);

    addSampleToGeometryClosestToPoint(pts1, pts2.front());
    addSampleToGeometryClosestToPoint(pts1, pts2.back());
    addSampleToGeometryClosestToPoint(pts2, pts1.front());
    addSampleToGeometryClosestToPoint(pts2, pts1.back());

    std::vector<Point> compPts1, compPts2;

    for(size_t t = 1 ; t <= 2 ; ++t){
        double width = ( t == 1 ? t1.width : t2.width );
        double width2 = ( t == 1 ? t2.width : t1.width );
        auto& pts = ( t == 1 ? pts1 : pts2 );
        auto& compPts = ( t == 1 ? compPts1 : compPts2 );
        if(width < 1e-9)
            continue;
        double eps = 0.02 * (width2 < 1e-9 ? width : width2);
        for(int side = 1 ; side >= -1 ; --side){
            double ang = M_PI_2 * side;
            for(size_t i = 0 ; i < pts.size() ; ++i){
                Point& p1 = ( i+1 < pts.size() ? pts.at(i) : pts.back() );
                Point& p2 = ( i+1 < pts.size() ? pts.at(i+1) : r_at(pts, 1) );
                auto pr = rotate(p1, p2, ang);
                compPts.push_back( getPointInLineAtDist(p1, pr, 0.5 * width + eps) );
            }
        }
    }

    //check if extended points are inside the boundary of the other track
    for(size_t t = 1 ; t <= 2 ; ++t){
        auto& boundary = ( t == 1 ? t2.boundary : t1.boundary );
        auto& compPts = ( t == 1 ? compPts1 : compPts2 );
        if(compPts.empty() || boundary.points.size() < 4)
            continue;
        if(in_polygon(compPts, boundary, false))
            return true;
    }

    if(!compPts1.empty() && !compPts2.empty()){//compare distances between extended points
        for(auto& p1 : compPts1){
            for(auto& p2 : compPts2){
                if(calc_dist(p1, p2) < 0.01)
                    return true;
            }
        }
    }


    //check if extended points are close enough to the points the other track
    for(size_t t = 1 ; t <= 2 ; ++t){
        double width2 = ( t == 1 ? t2.width : t1.width );
        auto& pts = ( t == 1 ? pts2 : pts1 );
        auto& compPts = ( t == 1 ? compPts1 : compPts2 );
        if(compPts.empty() || width2 < 1e-9)
            continue;
        for(auto& p1 : compPts){
            for(auto& p2 : pts){
                if(calc_dist(p1, p2) < 0.5 * width2)
                    return true;
            }
        }
    }

    return false;



//    const double eps = 0.01;
//    if(t1.boundary.points.size() > 3){
//        auto& t_ref = t1;
//        auto& t_comp = t2;
//        for(auto& p : t_comp.points){
//            if( calc_dist_to_linestring(t_ref.boundary.points, p) < t_comp.width + eps )
//                return true;
//        }
//        for(auto& p : t_comp.boundary.points){
//            if( calc_dist_to_linestring(t_ref.boundary.points, p) < eps )
//                return true;
//        }
//        return false;
//    }
//    if(t2.boundary.points.size() > 3){
//        auto& t_ref = t2;
//        auto& t_comp = t1;
//        for(auto& p : t_comp.points){
//            if( calc_dist_to_linestring(t_ref.boundary.points, p) < t_comp.width + eps )
//                return true;
//        }
//        for(auto& p : t_comp.boundary.points){
//            if( calc_dist_to_linestring(t_ref.boundary.points, p) < eps )
//                return true;
//        }
//        return false;
//    }

//    for(auto& p : t1.points){
//        if( calc_dist_to_linestring(t2.points, p) < t1.width + t2.width + eps )
//            return true;
//    }
//    return false;

}

std::vector<size_t> getInfieldExtremaTracksIndexes(const Subfield &sf,
                                                   const std::set<size_t> &excludeTrackIndexes)
{
    std::vector<size_t> ret;
    std::vector< std::vector<double> > adjAngles( sf.tracks.size() );

    for(size_t i = 0 ; i+1 < sf.tracks.size() ; ++i){
        if(excludeTrackIndexes.find(i) != excludeTrackIndexes.end())
            continue;
        for(size_t j = i+1 ; j < sf.tracks.size() ; ++j){
            if(excludeTrackIndexes.find(j) != excludeTrackIndexes.end())
                continue;
            if(areTracksAdjacent(sf.tracks.at(i), sf.tracks.at(j))){
                auto dir1 = getDirectionBetweenTracks( sf.tracks.at(i).points, sf.tracks.at(j).points );
                auto dir2 = rotate( Point(0,0), dir1, M_PI );
                adjAngles.at(i).push_back( get_angle(Point(0,0), dir1) );
                adjAngles.at(j).push_back( get_angle(Point(0,0), dir2) );
            }
        }
    }

    for(size_t i = 0 ; i < adjAngles.size() ; ++i){
        if(excludeTrackIndexes.find(i) != excludeTrackIndexes.end())
            continue;
        if(adjAngles.at(i).size() < 2){
            ret.push_back(i);
            continue;
        }
        Point refDir = rotate( Point(0,0), Point(1,0), adjAngles.at(i).front() );
        bool found = false;
        for(size_t j = 1 ; j < adjAngles.at(i).size() ; ++j){
            Point dir = rotate( Point(0,0), Point(1,0), adjAngles.at(i).at(j) );
            double ang = get_angle(refDir, Point(0,0), dir);
            if( std::fabs(ang) > M_PI_2 ){
                found = true;
                break;
            }
        }
        if(!found)
            ret.push_back(i);
    }

    return ret;
}

Point getDirectionBetweenTracks(std::vector<Point> trackFrom, std::vector<Point> trackTo)
{
    const double eps = 0.01;

    geometry::unsample_linestring(trackFrom);
    geometry::unsample_linestring(trackTo);

    for(auto& p : trackFrom)
        geometry::addSampleToGeometryClosestToPoint(trackTo, p, 1);

    for(auto& p : trackTo)
        geometry::addSampleToGeometryClosestToPoint(trackFrom, p, 1);

    std::multimap<double, std::pair<size_t, size_t>> distances;

    double minDist = std::numeric_limits<double>::max();
    for( size_t i = 0 ; i < trackFrom.size() ; ++i ){
        for( size_t j = 0 ; j < trackTo.size() ; ++j ){
            double dist = geometry::calc_dist( trackFrom.at(i), trackTo.at(j) );
            if(minDist > dist)
                minDist = dist;
            else if(dist > minDist * (1+eps))
                continue;
            distances.insert( std::make_pair(dist, std::make_pair(i, j)) );
        }
    }
    Point dir = Point(0,0);
    double compDist = minDist * (1+eps);
    for(auto& it_d : distances){
        if(it_d.first > compDist)
            break;
        size_t indFrom = it_d.second.first;
        size_t indTo = it_d.second.second;
        Point dirTmp = trackTo.at(indTo) - trackFrom.at(indFrom);
        geometry::setVectorLength(dirTmp, 1);

        dir = dir + dirTmp;
        geometry::setVectorLength(dir, 1);
    }

    return dir;
}

bool rotateField(Field &field, double angle_rad, Point pivot)
{
    if(!pivot.isValid()){
        if(!getCentroid(field.outer_boundary, pivot))
            return false;
    }

    field.outer_boundary = rotate(pivot, field.outer_boundary, angle_rad);
    for(auto& sf : field.subfields)
        rotateSubfield(sf, angle_rad, pivot);
    for(auto& road : field.external_roads)
        road.points = rotate(pivot, road.points, angle_rad);

    return true;
}

bool rotateSubfield(Subfield &sf, double angle_rad, Point pivot)
{
    if(!pivot.isValid()){
        if(!getCentroid(sf.boundary_outer, pivot))
            return false;
    }

    bool ok = true;

    sf.boundary_inner = rotate(pivot, sf.boundary_inner, angle_rad);
    sf.boundary_outer = rotate(pivot, sf.boundary_outer, angle_rad);
    sf.reference_line_A = rotate(pivot, sf.reference_line_A, angle_rad);
    sf.reference_line_B = rotate(pivot, sf.reference_line_B, angle_rad);
    for(auto& rl : sf.reference_lines)
        rl = rotate(pivot, rl, angle_rad);
    sf.access_points = rotate(pivot, sf.access_points, angle_rad);
    sf.resource_points = rotate(pivot, sf.resource_points, angle_rad);
    rotateHeadlands(sf.headlands, angle_rad, pivot);
    for(auto& track : sf.tracks)
        ok &= rotateTrack(track, angle_rad, pivot);
    for(auto& obs : sf.obstacles)
        ok &= rotateObstacle(obs, angle_rad, pivot);

    sf.working_direction += angle_rad;

    return ok;
}

bool rotateHeadlands(Headlands &hls, double angle_rad, const Point &pivot)
{
    if(!pivot.isValid())
        return false;

    bool ok = true;

    ok &= rotateHeadland(hls.complete, angle_rad, pivot);
    for(auto& hl : hls.partial)
        ok &= rotateHeadland(hl, angle_rad, pivot);

    return ok;
}

bool rotateHeadland(CompleteHeadland &hl, double angle_rad, const Point &pivot)
{
    if(!pivot.isValid())
        return false;

    bool ok = true;

    hl.boundaries = std::make_pair( rotate(pivot, hl.boundaries.first, angle_rad),
                                    rotate(pivot, hl.boundaries.second, angle_rad));
    hl.middle_track = rotate(pivot, hl.middle_track, angle_rad);
    for(auto& track : hl.tracks)
        ok &= rotateTrack(track, angle_rad, pivot);

    return ok;

}

bool rotateHeadland(PartialHeadland &hl, double angle_rad, const Point &pivot)
{
    if(!pivot.isValid())
        return false;

    bool ok = true;

    hl.boundary = rotate(pivot, hl.boundary, angle_rad);
    for(auto& track : hl.tracks)
        ok &= rotateTrack(track, angle_rad, pivot);

    return ok;
}

bool rotateTrack(Track &track, double angle_rad, const Point &pivot)
{
    if(!pivot.isValid())
        return false;

    bool ok = true;

    track.boundary = rotate(pivot, track.boundary, angle_rad);
    track.points = rotate(pivot, track.points, angle_rad);

    return ok;

}

bool rotateObstacle(Obstacle &obs, double angle_rad, const Point &pivot)
{
    if(!pivot.isValid())
        return false;

    bool ok = true;

    obs.boundary = rotate(pivot, obs.boundary, angle_rad);

    return ok;

}

}

}
