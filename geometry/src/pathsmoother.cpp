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

#include "arolib/geometry/pathsmoother.hpp"

namespace arolib{

namespace geometry{

PathSmoother::SmoothenPathCornerStrategy PathSmoother::intToSmoothePathCornerStrategy(int value)
{
    if(value == SmoothenPathCornerStrategy::SPCS_DO_NOT_REACH_CORNER_POINT)
        return SmoothenPathCornerStrategy::SPCS_DO_NOT_REACH_CORNER_POINT;
    else if(value == SmoothenPathCornerStrategy::SPCS_DO_NOT_REACH_CORNER_POINT_IF_POSSIBLE)
        return SmoothenPathCornerStrategy::SPCS_DO_NOT_REACH_CORNER_POINT_IF_POSSIBLE;
    else if(value == SmoothenPathCornerStrategy::SPCS_REACH_CORNER_POINT_IF_POSSIBLE)
        return SmoothenPathCornerStrategy::SPCS_REACH_CORNER_POINT_IF_POSSIBLE;
    else if(value == SmoothenPathCornerStrategy::SPCS_REACH_CORNER_POINT)
        return SmoothenPathCornerStrategy::SPCS_REACH_CORNER_POINT;
    throw std::invalid_argument( "The given value does not correspond to any SmoothenPathCornerStrategy" );

}

PathSmoother::PathSmoother(double angRes,
                           double angThreshold,
                           bool allowMultiCornerSmoothening,
                           bool allowDubins,
                           bool checkPiecewiseReverse,
                           int checkReverseOption,
                           PathSmoother::SmoothenPathCornerStrategy cornerStrategy_sharpCorners,
                           PathSmoother::SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners)
    : m_angRes(angRes)
    , m_angThreshold(angThreshold)
    , m_allowMultiCornerSmoothening(allowMultiCornerSmoothening)
    , m_allowDubins(allowDubins)
    , m_checkPiecewiseReverse(checkPiecewiseReverse)
    , m_checkReverseOption(checkReverseOption)
    , m_cornerStrategy_sharpCorners(cornerStrategy_sharpCorners)
    , m_cornerStrategy_nonSharpCorners(cornerStrategy_nonSharpCorners)
{

}

PathSmoother::~PathSmoother()
{

}

std::vector<Point> PathSmoother::smoothenPath(const std::vector<Point> &points, double radius)
{
    std::vector<Point> ret;

    if(radius < -1e-9)
        return ret;

    auto points_us = points;
    if(! unsample_linestring(points_us) )
        return ret;

    if(radius < 1e-9){
        ret = points_us;
        return ret;
    }

    bool checkReverse = points_us.size() > 2;
    if(m_checkReverseOption == 0)
        checkReverse = false;
    else if(m_checkReverseOption > 0)
        checkReverse &= points_us.size() <= m_checkReverseOption;

    if(!m_checkPiecewiseReverse){
        ret = smoothenPath_internal(points_us, radius, checkReverse);
        return ret;
    }

    ret = smoothenPath_piecewise(points_us, radius);

    if(ret.empty() || checkReverse){
        std::reverse(points_us.begin(), points_us.end());
        auto rev = smoothenPath_piecewise(points_us, radius);

        if( !rev.empty() ){
            std::reverse(rev.begin(), rev.end());

            if(!ret.empty()){
                double diff_fwd = computeDifferenceBetweenPaths(points_us, ret);
                double diff_rev = computeDifferenceBetweenPaths(points_us, rev);
                if(diff_fwd > diff_rev)
                    std::swap(ret, rev);
            }
            else
                std::swap(ret, rev);
        }

    }

    return ret;
}


bool PathSmoother::checkCornerGeometryForPathSmoothening ( const Point& p0,
                                                           const Point& p1,
                                                           const Point& p2,
                                                           double radius,
                                                           double angThreshold,
                                                           SmoothenPathCornerStrategy cornerStrategy_sharpCorners,
                                                           SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners,
                                                           double& cutDist,
                                                           double& minDist,
                                                           double& d2centroid,
                                                           bool& needsExtraSmoothening ){

    static auto getMinDist = [](double radius, double cutDist, double d2centroid, double angle_abs, bool isSharp, bool needsExtraSmoothening)->double{//@todo obtain extra needed distance in a more accurate manner
        double minDist = cutDist;
        if(needsExtraSmoothening){
//            double ang1 = angle_abs/2;
//            double ang2 = asin( std::min( 1.0, radius * sin(ang1) / d2centroid ) );
//            double alpha = 0.5 * (M_PI_2 - ang2);
//            double cutDistNew = radius * sin(alpha) / sin(M_PI_2 - alpha);
//            minDist += cutDistNew;

            double alpha = angle_abs/2;
            double a = d2centroid * cos(alpha);
            double b = d2centroid * sin(alpha);
            double x = sqrt( 4*radius*radius - (radius+b)*(radius+b) );
            minDist = x + a;
        }
        return minDist;
    };

    double angle = get_angle( p0, p1, p2 );
    double angle_abs = std::fabs(angle);
    double ang_comp = M_PI - angle_abs;
    if(ang_comp <= 0)
        return true;

    bool isSharp = ang_comp > M_PI_2;
    SmoothenPathCornerStrategy cornerStrategy = isSharp ? cornerStrategy_sharpCorners : cornerStrategy_nonSharpCorners;

    double dist1 = calc_dist(p0, p1);
    double dist2 = calc_dist(p1, p2);

    double d2centroid1, cutDist1, minDist1;//do not reach corner
    double d2centroid2, cutDist2, minDist2;//reach corner
    double alpha = ang_comp * 0.5;
    if(!isSharp){
        cutDist1 = radius * sin(alpha) / sin(M_PI_2 - alpha);
        d2centroid1 = std::sqrt( radius*radius + cutDist1*cutDist1 );
    }
    else{
        d2centroid1 = radius * std::sqrt(2.0);
        double d1 = cos(alpha) * d2centroid1;
        double l1 = sin(alpha) * d2centroid1;
        double l2 = std::sqrt( radius*radius - d1*d1 );
        cutDist1 = l1 + l2;
    }

    d2centroid2 = radius;
    alpha = 0.5 * std::fabs( angle );
    double beta = M_PI - 2*alpha;
    cutDist2 = radius * sin(beta) / sin(alpha);

    minDist1 = getMinDist(radius, cutDist1, d2centroid1, angle_abs, isSharp, isSharp);
    minDist2 = getMinDist(radius, cutDist2, d2centroid2, angle_abs, isSharp, true);

    if( cornerStrategy == SPCS_DO_NOT_REACH_CORNER_POINT || cornerStrategy == SPCS_DO_NOT_REACH_CORNER_POINT_IF_POSSIBLE ){
        cutDist = cutDist1;
        d2centroid = d2centroid1;
        minDist = minDist1;
        needsExtraSmoothening = isSharp;
    }
    else if( cornerStrategy == SPCS_REACH_CORNER_POINT || cornerStrategy == SPCS_REACH_CORNER_POINT_IF_POSSIBLE){
        cutDist = cutDist2;
        d2centroid = d2centroid2;
        minDist = minDist2;
        needsExtraSmoothening = true;
    }


    if( dist1 < minDist || dist2 < minDist ){
        if( cornerStrategy == SPCS_DO_NOT_REACH_CORNER_POINT_IF_POSSIBLE || cornerStrategy == SPCS_REACH_CORNER_POINT_IF_POSSIBLE){
            if( minDist1 < minDist2 ){
                cutDist = cutDist1;
                d2centroid = d2centroid1;
                minDist = minDist1;
                needsExtraSmoothening = isSharp;
            }
            else {
                cutDist = cutDist2;
                d2centroid = d2centroid2;
                minDist = minDist2;
                needsExtraSmoothening = true;
            }
        }
    }

    return (minDist <= dist1 && minDist <= dist2)/* || angle_abs > angThreshold*/;
}

double PathSmoother::computeDifferenceBetweenPaths(std::vector<Point> path1, std::vector<Point> path2, double radius){
    if( path1.empty() || path2.empty() )
        return std::numeric_limits<double>::max();

    const size_t minSamples = 20;

    unsample_linestring(path1);
    unsample_linestring(path2);
    int numSamples1 = std::max( 2 * path1.size(), minSamples );
    int numSamples2 = std::max( 2 * path2.size(), minSamples );
    std::vector<Point> path1_s{path1.front()};
    std::vector<Point> path2_s{path2.front()};

    double step1 = 1.0 / numSamples1;
    for(double d = step1 ; d <= 1-0.95*step1 ; d += step1 )
        path1_s.push_back( getPointAtRelativeDist(path1, d).first );
    path1_s.push_back( path1.back() );

    double step2 = 1.0 / numSamples2;
    for(double d = step2 ; d <= 1-0.95*step2 ; d += step2 )
        path2_s.push_back( getPointAtRelativeDist(path2, d).first );
    path2_s.push_back( path2.back() );

    double dist1 = 0, dist2 = 0;
    for(auto& p : path1_s)
        dist1 += std::pow( calc_dist_to_linestring(path2, p, false) , 2);
    for(auto& p : path2_s)
        dist2 += std::pow( calc_dist_to_linestring(path1, p, false) , 2);


    double dist3 = 0;
    double numSamples3 = std::max(numSamples1, numSamples2);
    double step3 = 1.0 / numSamples3;
    for(double d = step3 ; d <= 1-0.95*step3 ; d += step3 ){
        dist3 += std::pow( calc_dist(getPointAtRelativeDist(path1, d).first, getPointAtRelativeDist(path2, d).first) , 2);
    }


    double dist4 = std::pow( ( getGeometryLength(path1) - getGeometryLength(path2) ) / minSamples , 2);

    dist1 /= numSamples1;
    dist2 /= numSamples2;
    dist3 /= numSamples3;

    double diff = 1*(dist1 + dist2) + dist3 + dist4;

    return diff;



//    double dist = 0;
//    double resComp = std::min( radius * 0.25, 0.1 * getGeometryLength(path1) );
//    auto path1_s = sample_geometry(path1, resComp);
//    auto path2_s = sample_geometry(path1, resComp);
//    for(auto& p : path1_s)
//        dist += calc_dist_to_linestring(path2_s, p, false);
//    for(auto& p : path2_s)
//        dist += calc_dist_to_linestring(path1_s, p, false);
//    return dist;

//    return calc_discrete_frechet_distance(path1, path2);

//    return PathSmoother__calc_discrete_frechet_distance(path1, path2);
}

double PathSmoother::getAngRes() const
{
    return m_angRes;
}

bool PathSmoother::setAngRes(double value)
{
    if(value < 1e-6 || value > 180)
        return false;
    m_angRes = value;
    return true;
}

double PathSmoother::getAngThreshold() const
{
    return m_angThreshold;
}

bool PathSmoother::setAngThreshold(double value)
{
    if(value < 1e-6 || value > 180+1e-3)
        return false;
    m_angThreshold = value;
    return true;
}

bool PathSmoother::getAllowDubins() const
{
    return m_allowDubins;
}

void PathSmoother::setAllowDubins(bool value)
{
    m_allowDubins = value;
}

bool PathSmoother::getCheckPiecewiseReverse() const
{
    return m_checkPiecewiseReverse;
}

void PathSmoother::setCheckPiecewiseReverse(bool value)
{
    m_checkPiecewiseReverse = value;
}

bool PathSmoother::isMultiCornerSmootheningEnabled() const
{
    return m_allowMultiCornerSmoothening;
}

void PathSmoother::enableMultiCornerSmoothening(bool value)
{
    m_allowMultiCornerSmoothening = value;
}

int PathSmoother::getCheckReverseOption() const
{
    return m_checkReverseOption;
}

void PathSmoother::setCheckReverseOption(int value)
{
    m_checkReverseOption = value;
}

PathSmoother::SmoothenPathCornerStrategy PathSmoother::getCornerStrategy_sharpCorners() const
{
    return m_cornerStrategy_sharpCorners;
}

void PathSmoother::setCornerStrategy_sharpCorners(const SmoothenPathCornerStrategy &value)
{
    m_cornerStrategy_sharpCorners = value;
}

PathSmoother::SmoothenPathCornerStrategy PathSmoother::getCornerStrategy_nonSharpCorners() const
{
    return m_cornerStrategy_nonSharpCorners;
}

void PathSmoother::setCornerStrategy_nonSharpCorners(const SmoothenPathCornerStrategy &value)
{
    m_cornerStrategy_nonSharpCorners = value;
}

std::vector<Point> PathSmoother::smoothenPath_internal(std::vector<Point> points_us, double radius, bool checkReverse)
{
    std::vector<Point> ret;

    double angRes_rad = deg2rad( std::fabs(m_angRes) );
    double angThreshold_rad = deg2rad( std::fabs(m_angThreshold) );

    bool okFwd = smoothenPath_recursive(points_us, 0, ret, radius, angRes_rad, angThreshold_rad, m_allowMultiCornerSmoothening,
                                        m_cornerStrategy_sharpCorners, m_cornerStrategy_nonSharpCorners);


    if(!okFwd){
        ret.clear();
    }

    if(!okFwd || checkReverse){
        std::vector<Point> rev;

        std::reverse(points_us.begin(), points_us.end());

        if( smoothenPath_recursive(points_us, 0, rev, radius, angRes_rad, angThreshold_rad, m_allowMultiCornerSmoothening,
                                    m_cornerStrategy_sharpCorners, m_cornerStrategy_nonSharpCorners) ){
            std::reverse(rev.begin(), rev.end());

            if(okFwd){
                double diff_fwd = computeDifferenceBetweenPaths(points_us, ret);
                double diff_rev = computeDifferenceBetweenPaths(points_us, rev);
                if(diff_fwd > diff_rev)
                    std::swap(ret, rev);
            }
            else
                std::swap(ret, rev);
        }

    }

    unsample_linestring(ret);
    return ret;

}

std::vector<Point> PathSmoother::smoothenPath_piecewise(std::vector<Point> points_us, double radius)
{
    std::vector<Point> ret;

    double angThreshold_rad = deg2rad( std::fabs(m_angThreshold) );
    size_t ind_start = 0;
    ret.push_back(points_us.front());
    double checkMult = 1.1;
    while(1){
        size_t ind_end = points_us.size();
        auto ind_from = ind_start;
        Point pEnd;
        while( ind_from+1 < points_us.size() ){
            auto ind_ok = findIndOfNextLongEnoughSegment(points_us,
                                                         ind_from,
                                                         radius,
                                                         angThreshold_rad,
                                                         m_cornerStrategy_sharpCorners,
                                                         m_cornerStrategy_nonSharpCorners,
                                                         false,
                                                         checkMult);

            if(ind_ok <= ind_from || ind_ok+2 >= points_us.size())
                break;

            double cutDist, minDist, d2centroid;
            bool needsExtraSmoothening;

            checkCornerGeometryForPathSmoothening ( points_us.at(ind_ok-1), points_us.at(ind_ok), points_us.at(ind_ok+1),
                                                    radius, angThreshold_rad,
                                                    m_cornerStrategy_sharpCorners, m_cornerStrategy_nonSharpCorners,
                                                    cutDist, minDist, d2centroid, needsExtraSmoothening );

            double dist_prev = calc_dist(points_us.at(ind_ok-1), points_us.at(ind_ok));
            double dist_check_prev = 4 * radius;//to ensure that the segment is long enough for the previous smoothing
            double dist_check_next = minDist*checkMult + needsExtraSmoothening * radius;//to ensure that the segment is long enough for the next smoothing
            if(dist_prev > dist_check_prev + dist_check_next){
                pEnd = getPointInLineAtDist( points_us.at(ind_ok-1), points_us.at(ind_ok), dist_check_prev );
                ind_end = ind_ok;//index of the last point of the long segment
                break;
            }

            pEnd = points_us.at(ind_ok);
            points_us.at(ind_ok) = getPointInLineAtDist( points_us.at(ind_ok), points_us.at(ind_ok+1), minDist*checkMult );

            if( checkCornerGeometryForPathSmoothening ( points_us.at(ind_ok), points_us.at(ind_ok+1), points_us.at(ind_ok+2),
                                                        radius, angThreshold_rad,
                                                        m_cornerStrategy_sharpCorners, m_cornerStrategy_nonSharpCorners,
                                                        cutDist, minDist, d2centroid, needsExtraSmoothening )
                    && calc_dist(points_us.at(ind_ok), points_us.at(ind_ok+1)) > minDist*checkMult
                    && calc_dist(points_us.at(ind_ok+1), points_us.at(ind_ok+2)) > minDist*checkMult ){
                std::swap(pEnd, points_us.at(ind_ok));
                ind_end = ind_ok+1;//index of the last point of the long segment
                break;
            }
            std::swap(pEnd, points_us.at(ind_ok));
            ind_from = ind_ok;
        }

        if(ind_end > ind_start+1 && ind_end+1 < points_us.size()){
            std::vector<Point> seg( points_us.begin()+ind_start, points_us.begin()+ind_end+1 );
            seg.back() = pEnd;
            auto seg_smooth = smoothenPath_internal(seg, radius, seg.size() > 3);
            if(seg_smooth.empty()){
                ret.clear();
                return ret;
            }
            ret.insert( ret.end(), seg_smooth.begin()+1, seg_smooth.end() );

            points_us.at(ind_end-1) = pEnd;
            ind_start = ind_end-1;
        }
        else{//remaining points

            std::vector<Point> seg( points_us.begin()+ind_start, points_us.end() );
            bool checkReverse = seg.size() > 2 && ind_start != 0;
            auto seg_smooth = smoothenPath_internal(seg, radius, checkReverse);
            if(seg_smooth.empty()){
                ret.clear();
                return ret;
            }
            ret.insert( ret.end(), seg_smooth.begin()+1, seg_smooth.end() );
            break;
        }
    }
    unsample_linestring(ret);
    return ret;
}

bool PathSmoother::smoothenPath_recursive(std::vector<Point> &points_us, size_t ind_from, std::vector<Point> &points_out,
                                          double radius, double angRes_rad, double angThreshold_rad, bool allowMultiCornerSmoothening,
                                          PathSmoother::SmoothenPathCornerStrategy cornerStrategy_sharpCorners, PathSmoother::SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners){

    if(ind_from >= points_us.size())
        return false;

    if(points_out.empty())
        points_out.push_back(points_us.at(ind_from));

    if(ind_from + 2 >= points_us.size()){
        points_out.insert( points_out.end(), points_us.begin()+ind_from + 1, points_us.end() );
        return true;
    }

    if(allowMultiCornerSmoothening){
        size_t size_prev = points_out.size();

        bool ok = getMultiCornerSmoothPath(points_us, ind_from, points_out,
                                           radius, angRes_rad, angThreshold_rad,
                                           cornerStrategy_sharpCorners,
                                           cornerStrategy_nonSharpCorners);
        if (ok)
            return ok;
        if(points_out.size() > size_prev)
            points_out.erase( points_out.begin()+size_prev, points_out.end() );
    }

    double angle = get_angle( points_us.at(ind_from), points_us.at(ind_from+1), points_us.at(ind_from+2) );
    double ang_comp = M_PI - std::fabs( angle );
    bool needsExtraSmoothening;

    if(ang_comp <= 0){
        if(ind_from + 3 < points_us.size()){
            return smoothenPath_recursive( points_us, ind_from+1, points_out, radius, angRes_rad, angThreshold_rad,
                                           allowMultiCornerSmoothening, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);
        }
        points_out.insert( points_out.end(), points_us.begin()+ind_from + 1, points_us.end() );
        return true;
    }

    double cutDist, d2centroid, minDist;
    bool okLengths = checkCornerGeometryForPathSmoothening(points_us.at(ind_from), points_us.at(ind_from+1), points_us.at(ind_from+2),
                                                           radius, angThreshold_rad,
                                                           cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners,
                                                           cutDist, minDist, d2centroid, needsExtraSmoothening);
    if( okLengths
            && points_us.size() - ind_from > 3
            && std::fabs(angle) >= angThreshold_rad){//check if the next segment is long enough to smoothen the first corner
        okLengths = checkNextSegments(points_us, ind_from+1, radius, angThreshold_rad, minDist,
                                      cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);

    }

    if(!okLengths){
        if( std::fabs(angle) < angThreshold_rad ){

            double dist0 = calc_dist(points_us.at(ind_from), points_us.at(ind_from+1));
            double dist1 = calc_dist(points_us.at(ind_from+1), points_us.at(ind_from+2));

            if( cutDist > dist0 || cutDist > dist1 || !needsExtraSmoothening ){// line not long enough to make a continuous connection
                if(!m_allowDubins)
                    return false;
                return smoothenPath_dubins(points_us, ind_from, points_out, radius, angRes_rad, angThreshold_rad, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);
            }
        }

        else {
            if(ind_from + 3 < points_us.size()){
                return smoothenPath_recursive( points_us, ind_from+1, points_out, radius, angRes_rad, angThreshold_rad,
                                               allowMultiCornerSmoothening, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners );
            }

            points_out.insert( points_out.end(), points_us.begin()+ind_from+1, points_us.end() );
            return true;
        }
    }

    Point centroid = rotate( points_us.at(ind_from+1), points_us.at(ind_from), 0.5*angle );
    centroid = getPointInLineAtDist( points_us.at(ind_from+1), centroid, d2centroid );

    Point pStart = getPointInLineAtDist( points_us.at(ind_from+1), points_us.at(ind_from), cutDist );
    Point pFinish = getPointInLineAtDist( points_us.at(ind_from+1), points_us.at(ind_from+2), cutDist );
    double angRange = get_angle( pStart, centroid, pFinish );
    if(needsExtraSmoothening && angle * angRange > 0 ){
        angRange = 2 * M_PI - std::fabs(angRange);
        if(angle > 0)
            angRange *= -1;
    }

//    std::vector<Point> connection = getSmoothCornerFromCircle(points_us, ind_from, ind_from+2, pStart, pFinish, centroid,
//                                                              radius, angRes_rad, angThreshold_rad, angRange < 0, true,
//                                                              cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);

    std::vector<Point> connection = getSmoothCornerFromCircles(points_us.at(ind_from), points_us.at(ind_from+2), pStart, pFinish, centroid,
                                                               radius, angRes_rad, angRange < 0, true );

    //std::vector<Point> connection = getSmoothCorner_dubins(points_us.at(ind_from), points_us.at(ind_from+1), points_us.at(ind_from+2), minDist, radius, angRes_rad);

    if(connection.empty())
        return smoothenPath_dubins(points_us, ind_from, points_out, radius, angRes_rad, angThreshold_rad, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);;

    points_out.insert(points_out.end(), connection.begin(), connection.end());


    if(ind_from+2 >= points_us.size()){
        points_out.push_back(points_us.back());
        return true;
    }

    Point pTmp = points_us.at(ind_from+1);
    points_us.at(ind_from+1) = points_out.back();
    bool ok = smoothenPath_recursive( points_us, ind_from+1, points_out, radius, angRes_rad, angThreshold_rad,
                                      allowMultiCornerSmoothening, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners );
    points_us.at(ind_from+1) = pTmp;

    return ok;
}

bool PathSmoother::smoothenPath_recursive_old(std::vector<Point> &points_us, size_t ind_from, std::vector<Point> &points_out,
                                          double radius, double angRes_rad, double angThreshold_rad, bool allowMultiCornerSmoothening,
                                          PathSmoother::SmoothenPathCornerStrategy cornerStrategy_sharpCorners, PathSmoother::SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners){

    if(ind_from >= points_us.size())
        return false;

    if(points_out.empty())
        points_out.push_back(points_us.at(ind_from));

    if(ind_from + 2 >= points_us.size()){
        points_out.insert( points_out.end(), points_us.begin()+ind_from + 1, points_us.end() );
        return true;
    }

    std::vector<Point> corner_0;
    bool isSmooth = isCornerSmooth(points_us.at(ind_from),
                                   points_us.at(ind_from+1),
                                   points_us.at(ind_from+2),
                                   radius,
                                   angRes_rad,
                                   M_PI, //angThreshold_rad,
                                   corner_0);
    if( isSmooth ){
        size_t size_prev = points_out.size();

        points_out.insert( points_out.end(), corner_0.begin(), corner_0.end() );
        Point pTmp = points_us.at(ind_from+1);
        points_us.at(ind_from+1) = points_out.back();
        bool ok = smoothenPath_recursive( points_us, ind_from+1, points_out, radius, angRes_rad, angThreshold_rad,
                                          allowMultiCornerSmoothening, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);
        points_us.at(ind_from+1) = pTmp;

        if(ok || !allowMultiCornerSmoothening)
            return ok;

        //it is possible that the lengths of the next segments are not long enough and we need to do MultiCornerSmoothening
        if(points_out.size() > size_prev)
            points_out.erase( points_out.begin()+size_prev, points_out.end() );

    }

    if(allowMultiCornerSmoothening){
        size_t size_prev = points_out.size();

        bool ok = getMultiCornerSmoothPath(points_us, ind_from, points_out,
                                           radius, angRes_rad, angThreshold_rad,
                                           cornerStrategy_sharpCorners,
                                           cornerStrategy_nonSharpCorners);
        if (ok || isSmooth)
            return ok;
        if(points_out.size() > size_prev)
            points_out.erase( points_out.begin()+size_prev, points_out.end() );
    }

    double angle = get_angle( points_us.at(ind_from), points_us.at(ind_from+1), points_us.at(ind_from+2) );
    double ang_comp = M_PI - std::fabs( angle );
    bool needsExtraSmoothening;

    if(ang_comp <= 0){
        if(ind_from + 3 < points_us.size()){
            return smoothenPath_recursive( points_us, ind_from+1, points_out, radius, angRes_rad, angThreshold_rad,
                                           allowMultiCornerSmoothening, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);
        }
        points_out.insert( points_out.end(), points_us.begin()+ind_from + 1, points_us.end() );
        return true;
    }

    double cutDist, d2centroid, minDist;
    bool okLengths = checkCornerGeometryForPathSmoothening(points_us.at(ind_from), points_us.at(ind_from+1), points_us.at(ind_from+2),
                                                           radius, angThreshold_rad,
                                                           cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners,
                                                           cutDist, minDist, d2centroid, needsExtraSmoothening);
    if( okLengths
            && points_us.size() - ind_from > 3
            && std::fabs(angle) >= angThreshold_rad){//check if the next segment is long enough to smoothen the first corner
        okLengths = checkNextSegments(points_us, ind_from+1, radius, angThreshold_rad, minDist,
                                      cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);

    }

    if(!okLengths){
        if( std::fabs(angle) < angThreshold_rad ){

            double dist0 = calc_dist(points_us.at(ind_from), points_us.at(ind_from+1));
            double dist1 = calc_dist(points_us.at(ind_from+1), points_us.at(ind_from+2));

            if( cutDist > dist0 || cutDist > dist1 || !needsExtraSmoothening ){// line not long enough to make a continuous connection
                if(!m_allowDubins)
                    return false;
                return smoothenPath_dubins(points_us, ind_from, points_out, radius, angRes_rad, angThreshold_rad, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);
            }

            //since the minDist is computed with the assuption that both connections to the main circle are done also with semi-circles,
            //there is a chance that those connections can be done directly without that extra semi-circles (i.e without the need of the extra distance needed for the connection semi-circles)
            //and so, the real minDist = cutDist
        }

        else {
            if(ind_from + 3 < points_us.size()){
                return smoothenPath_recursive( points_us, ind_from+1, points_out, radius, angRes_rad, angThreshold_rad,
                                               allowMultiCornerSmoothening, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners );
            }

            points_out.insert( points_out.end(), points_us.begin()+ind_from+1, points_us.end() );
            return true;
        }
    }

    Point centroid = rotate( points_us.at(ind_from+1), points_us.at(ind_from), 0.5*angle );
    centroid = getPointInLineAtDist( points_us.at(ind_from+1), centroid, d2centroid );

    Point pStart = getPointInLineAtDist( points_us.at(ind_from+1), points_us.at(ind_from), cutDist );
    Point pFinish = getPointInLineAtDist( points_us.at(ind_from+1), points_us.at(ind_from+2), cutDist );
    double angRange = get_angle( pStart, centroid, pFinish );
    if(needsExtraSmoothening && angle * angRange > 0 ){
        angRange = 2 * M_PI - std::fabs(angRange);
        if(angle > 0)
            angRange *= -1;
    }

//    std::vector<Point> connection = getSmoothCornerFromCircle(points_us, ind_from, ind_from+2, pStart, pFinish, centroid,
//                                                              radius, angRes_rad, angThreshold_rad, angRange < 0, true,
//                                                              cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);

    std::vector<Point> connection = getSmoothCornerFromCircles(points_us.at(ind_from), points_us.at(ind_from+2), pStart, pFinish, centroid,
                                                               radius, angRes_rad, angRange < 0, true );
    if(connection.empty())
        return false;

    points_out.insert(points_out.end(), connection.begin(), connection.end());


    if(ind_from+2 >= points_us.size()){
        points_out.push_back(points_us.back());
        return true;
    }

    Point pTmp = points_us.at(ind_from+1);
    points_us.at(ind_from+1) = points_out.back();
    bool ok = smoothenPath_recursive( points_us, ind_from+1, points_out, radius, angRes_rad, angThreshold_rad,
                                      allowMultiCornerSmoothening, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners );
    points_us.at(ind_from+1) = pTmp;

    return ok;
}

size_t PathSmoother::findIndOfNextLongEnoughSegment(std::vector<Point> &points_us, size_t ind_from, double radius, double angThreshold_rad,
                                                    PathSmoother::SmoothenPathCornerStrategy cornerStrategy_sharpCorners, PathSmoother::SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners, bool includeFirstSegment, double distMult){
    double minDistPrev = 0;
    for(size_t i = ind_from ; i+2 < points_us.size() ; ++i){
        double cutDist;
        double minDist;
        double d2centroid;
        bool needsExtraSmoothening;

        double d1 = calc_dist( points_us.at(i), points_us.at(i+1) );
        double d2 = calc_dist( points_us.at(i+1), points_us.at(i+2) );

        checkCornerGeometryForPathSmoothening(points_us.at(i), points_us.at(i+1), points_us.at(i+2),
                                              radius, M_PI,
                                              cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners,
                                              cutDist, minDist, d2centroid, needsExtraSmoothening);

        if(d1 > distMult*(minDist + minDistPrev) && ( includeFirstSegment || i > ind_from ) )//first subsegment is long enogh
            return i+1;

        if( i+3 == points_us.size() && d2 > distMult*minDist)//last segment of points_us is long enough
            return i+2;
        minDistPrev = minDist;
    }
    return 0;//fail
}

bool PathSmoother::isCornerSmooth(const Point &p0, const Point &p1, const Point &p2, double radius,
                                  double angRes_rad, double angThreshold_rad, std::vector<Point> &corner){

    double angle = get_angle( p0, p1, p2 );
    double angle_abs = std::fabs( angle );
    if( angle_abs < M_PI_2 )
        return false;

    Point centroid = rotate( p1, p0, 0.5 * angle );
    centroid = getPointInLineAtDist(p1, centroid, radius);

    double d1 = calc_dist(centroid, p0);
    double d2 = calc_dist(centroid, p2);

    double deltaRad = radius * 1e-2;
    if( d1 < radius - deltaRad ||  d2 < radius - deltaRad ){
        if(angle_abs > angThreshold_rad){
            corner = {p1};
            return true;
        }
        return false;
    }

    double deltaRes = angRes_rad * 1e-1;
    double angRes_actual = M_PI - angle_abs;

    if(angRes_actual < angRes_rad + deltaRes){
        corner = {p1};
        return true;
    }


    if(angRes_actual < angRes_rad * 1.5
            || ( angRes_actual < angRes_rad * 2 && d1 < 1.5*radius && d2 < 1.5*radius ) ){
        double dir = angle < 0 ? -1 : 1;
        Point pStart = rotate( centroid, p1, 0.5*angRes_actual * dir );

        double angRange = -angRes_actual*dir;
        int numSamples = std::ceil( std::fabs( angRange / angRes_rad ) );
        auto circle = create_circle( centroid, radius, numSamples, get_angle( centroid, pStart ), angRange );
        if(circle.points.size() < 4)
            return false;
        corner.clear();
        corner.insert( corner.end(), circle.points.begin()+1, circle.points.end()-1 );

        return true;
    }

    return false;

}

bool PathSmoother::checkNextSegments(std::vector<Point> &points_us, size_t ind_from, double radius,
                                     double angThreshold_rad, double dist0,
                                     PathSmoother::SmoothenPathCornerStrategy cornerStrategy_sharpCorners, PathSmoother::SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners){

    if(ind_from >= points_us.size())
        return false;

    if(ind_from + 2 >= points_us.size())
        return true;

    double minDistPrev = dist0;
    for(size_t j = ind_from ; j+2 < points_us.size() ; ++j ){
        double angleNext = std::fabs( get_angle(points_us.at(j), points_us.at(j+1), points_us.at(j+2)));
        double d1 = calc_dist( points_us.at(j), points_us.at(j+1) );
        double cutDistNext, d2centroidNext, minDistNext;
        bool needsExtraSmootheningNext;
        bool okLengthsNext = checkCornerGeometryForPathSmoothening(points_us.at(j), points_us.at(j+1), points_us.at(j+2),
                                                                   radius, angThreshold_rad,
                                                                   cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners,
                                                                   cutDistNext, minDistNext, d2centroidNext, needsExtraSmootheningNext);

        if(minDistNext + minDistPrev <= d1)
            return true;

        //not enough segment length for the next corner
        if(angleNext < angThreshold_rad)
            return false;

        if(!okLengthsNext)//second segment is not long enough
            return false;

        //keep cheking
        minDistPrev = minDistNext;

    }
    return true;
}

std::vector<Point> PathSmoother::getSmoothCornerFromCircle(std::vector<Point> &points_us, size_t ind_p0, size_t ind_pn,
                                                           Point pStart, Point pFinish, const Point &centroid,
                                                           double radius, double angRes_rad, double angThreshold_rad, bool negativeAngDir, bool checkNextSegs,
                                                           PathSmoother::SmoothenPathCornerStrategy cornerStrategy_sharpCorners, PathSmoother::SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners){

    static auto getAngularRange = [](const Point& pStart, const Point& pFinish, const Point& centroid, bool negativeAngDir)->double{
        double angRange = get_angle( pStart, centroid, pFinish );
        if( (angRange>0 && negativeAngDir) || (angRange<0 && !negativeAngDir) ){
            angRange = 2 * M_PI - std::fabs(angRange);
            if( (angRange>0 && negativeAngDir) || (angRange<0 && !negativeAngDir) )
                angRange *= -1;
        }
        return angRange;
    };

    std::vector<Point> ret;
    if( ind_p0 >= points_us.size() || ind_pn >= points_us.size() )
        return ret;

    double angRange = getAngularRange( pStart, pFinish, centroid, negativeAngDir );

    int numSamples = std::ceil( std::fabs( angRange / angRes_rad ) );
    auto circle = create_circle( centroid, radius, numSamples, get_angle( centroid, pStart ), angRange ).points;
    if(circle.size() < 4)
        return ret;

    numSamples = std::ceil( std::fabs( rad2deg(angRange) ) );//samples at 1 deg
    auto circle2 = create_circle( centroid, radius, numSamples, get_angle( centroid, pStart ), angRange ).points;
    if(circle2.size() < 4)
        return ret;

    std::vector<Point> connection_0, connection_n;
    for(int side = 0 ; side < 2 ; ++side){
        Point pRef1;
        Point* pRefC;
        std::vector<Point>* connection;
        if(side == 0){
            pRef1 = points_us.at(ind_p0);
            pRefC = &pStart;
            connection = &connection_0;
        }
        else{
            pRef1 = points_us.at(ind_pn);
            pRefC = &pFinish;
            connection = &connection_n;
        }

        double distRef = calc_dist(pRef1, *pRefC);
        double ang1 = std::fabs( get_angle(pRef1, *pRefC, centroid) );
        if(pRef1 != *pRefC && ang1-1e-5 > M_PI_2){//needs extra smoothening (start)
            Point goodThresholdOption;
            double goodThresholdOption_maxAngle = -1;
            int indRef = 2;
            do{
                if(indRef+1 >= circle2.size())
                    return ret;

                Point p2;
                if(side == 0)
                    p2 = circle2.at(indRef);
                else
                    p2 = r_at(circle2, indRef);

                double angC = std::fabs( get_angle(*pRefC, centroid, p2) );
                bool stopChecking = angC > M_PI_4;

                if(!stopChecking){

                    double ang2 = get_angle( pRef1, *pRefC, p2 );
                    double ang_comp2 = M_PI - std::fabs( ang2 );
                    double alpha = ang_comp2 * 0.5;
                    double cutDist2 = radius * sin(alpha) / sin(M_PI_2 - alpha);
                    double dist = calc_dist( *pRefC, p2 );

                    if(indRef+2 < circle2.size() && std::fabs(ang2) >= angThreshold_rad ){
                        Point p3;
                        if(side == 0)
                            p3 = circle2.at(indRef+1);
                        else
                            p3 = r_at(circle2, indRef+1);
                        double ang3 = std::fabs( get_angle(p3, p2, *pRefC) );

                        if( ang3 >= angThreshold_rad ){
                            double minAng = std::min( std::fabs(ang2), ang3);
                            if( goodThresholdOption_maxAngle < minAng ){
                                goodThresholdOption_maxAngle = minAng;
                                goodThresholdOption = p2;
                            }
                        }
                    }

                    if( cutDist2 <= dist){

                        bool okNextSegments = true;
                        if(side == 1 && checkNextSegs){
                            Point pTmp = *pRefC;
                            std::swap( points_us.at(ind_pn-1), pTmp );
                            okNextSegments = checkNextSegments(points_us, ind_pn-1, radius, angThreshold_rad, cutDist2,
                                                               cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);
                            std::swap( points_us.at(ind_pn-1), pTmp );
                        }

                        if( cutDist2 <= distRef && okNextSegments ){
                            //smoothen circle connection
                            std::vector<Point> connection_in = {pRef1, *pRefC, p2};
                            if(!smoothenPath_recursive(connection_in, 0, *connection, radius, angRes_rad, angThreshold_rad, false,
                                                       SPCS_DO_NOT_REACH_CORNER_POINT, SPCS_DO_NOT_REACH_CORNER_POINT))
                                return ret;

                            if(!connection->empty())
                                pop_front( *connection );

                            if(side != 0)
                                std::reverse(connection->begin(), connection->end());

                            *pRefC = p2;

                            break;
                        }

                    }
                }

                else{
                    if(goodThresholdOption_maxAngle > 0){
                        if(side == 0)
                            *connection = {*pRefC, goodThresholdOption};
                        else
                            *connection = {goodThresholdOption, *pRefC};
                        *pRefC = goodThresholdOption;
                        break;
                    }

                    double ang3;
                    if(side == 0)
                        ang3 = std::fabs( get_angle( pRef1, circle.at(1), circle.at(2) ) );
                    else
                        ang3 = std::fabs( get_angle( pRef1, r_at(circle2, 1), r_at(circle2, 2) ) );
                    if(ang3 >= angThreshold_rad)//the main circle generates a good angle --> no connection needed
                        break;

                    return ret;
                }
                indRef++;
            }
            while(1);
        }

    }

    if(connection_0.empty() && connection_n.empty()){
        ret.insert( ret.end(), circle.begin()+1, circle.end()-1 );
        return ret;
    }

    angRange = getAngularRange( pStart, pFinish, centroid, negativeAngDir );

    numSamples = std::ceil( std::fabs( angRange / angRes_rad ) );
    circle = create_circle( centroid, radius, numSamples, get_angle( centroid, pStart ), angRange ).points;
    if(circle.size() < 4)
        return ret;

    if(!connection_0.empty())
        ret.insert( ret.end(), connection_0.begin(), connection_0.end()-1 );
    ret.insert( ret.end(), circle.begin()+1, circle.end()-1 );
    if(!connection_n.empty())
        ret.insert( ret.end(), connection_n.begin()+1, connection_n.end() );

    return ret;

}

std::vector<Point> PathSmoother::getSmoothCornerFromCircles(Point &p0, Point &pn, Point pCutStart, Point pCutFinish, const Point &centroid, double radius, double angRes_rad, bool negativeAngDir, bool checkNextSegs)
{
    struct ConnInfo{
        Point centroid, pStart;
        double angRange = 0;
    }startConnInfo, finishConnInfo, mainConnInfo;

    static auto getAngularRange = [](const Point& pStart, const Point& pFinish, const Point& centroid, bool negativeAngDir)->double{
        double angRange = get_angle( pStart, centroid, pFinish );
        if( (angRange>0 && negativeAngDir) || (angRange<0 && !negativeAngDir) ){
            angRange = 2 * M_PI - std::fabs(angRange);
            if( (angRange>0 && negativeAngDir) || (angRange<0 && !negativeAngDir) )
                angRange *= -1;
        }
        return angRange;
    };

    static auto getConnInfo = [](const Point& pRef, const Point& pCut, const Point& centroid, const Point& pMidConnection, double radius, double dirRot)->ConnInfo{
        ConnInfo info;
        Point pIntersection;
        info.pStart = pCut;

        if( get_intersection(pRef, pCut, centroid, pMidConnection, pIntersection, true, true) ){
            double d2centroid = calc_dist( centroid, pIntersection );
            double alpha = d2centroid > 0 ? std::fabs( get_angle(pCut, pIntersection, centroid ) ) : 0;
            if( alpha > M_PI_2 ){
                d2centroid *= -1;
                alpha = M_PI - alpha;
            }

            double a = d2centroid * cos(alpha);
            double b = d2centroid * sin(alpha);
            double x = sqrt( 4*radius*radius - (radius+b)*(radius+b) );

            if( calc_dist(pCut, pIntersection) < calc_dist(pRef, pIntersection) ){
                double distStart = a+x;
                info.pStart = getPointInLineAtDist(pIntersection, pRef, distStart);
                info.centroid = rotate( info.pStart, pIntersection, M_PI_2 * dirRot );
            }
            else{
                double distStart = std::fabs( a-x );
                info.pStart = getPointInLineAtDist(pIntersection, pCut, distStart);
                info.centroid = rotate( info.pStart, pIntersection, -M_PI_2 * dirRot );
            }

            info.centroid = getPointInLineAtDist(info.pStart, info.centroid, radius);
            info.angRange = asin( x / (2*radius) ) * dirRot;
        }
        else{//parallel
            double b = calc_dist_to_line(pRef, pCut, centroid);
            double x = sqrt( 4*radius*radius - (radius+b)*(radius+b) );
            info.pStart = centroid + calc_vector_to_line(pRef, pCut, centroid, true);
            info.pStart = getPointInLineAtDist(centroid, info.pStart, b);
            info.pStart = getPointInLineAtDist(info.pStart, pRef, x);
            info.centroid = rotate( info.pStart, pCut, M_PI_2 * dirRot );
            info.centroid = getPointInLineAtDist(info.pStart, info.centroid, radius);
            info.angRange = asin( x / (2*radius) ) * dirRot;
        }

        return info;
    };

    static auto getSemiCircle = [](const ConnInfo& info, double radius, double angRes_rad)->std::vector<Point>{
        int numSamples = std::ceil( std::fabs( info.angRange / angRes_rad ) );
        auto ret = create_circle( info.centroid, radius, numSamples, get_angle( info.centroid, info.pStart ), info.angRange ).points;
        if(ret.size() < 4){
            ret.clear();
            return ret;
        }
        ret.pop_back();
        pop_front(ret);
        return ret;
    };

    std::vector<Point> ret;
    if(radius <= 0)
        return ret;

    mainConnInfo.angRange = getAngularRange( pCutStart, pCutFinish, centroid, negativeAngDir );
    mainConnInfo.pStart = pCutStart;
    mainConnInfo.centroid = centroid;

    Point pMidConnection = rotate(centroid, pCutStart, 0.5 * mainConnInfo.angRange);

    if(pCutStart != p0){
        double dirRot = mainConnInfo.angRange > 0 ? -1 : 1;
        startConnInfo = getConnInfo(p0, pCutStart, centroid, pMidConnection, radius, dirRot);
        if( calc_dist(pCutStart, p0) < calc_dist(pCutStart, startConnInfo.pStart) )//segment not long enough
            return ret;
    }
    if(pCutFinish != pn){
        double dirRot = mainConnInfo.angRange > 0 ? 1 : -1;
        finishConnInfo = getConnInfo(pn, pCutFinish, centroid, pMidConnection, radius, dirRot);
        if( calc_dist(pCutFinish, pn) < calc_dist(pCutFinish, finishConnInfo.pStart) )//segment not long enough
            return ret;
    }


    std::vector<Point> connectionMain, connectionFinish;
    Point pFinishMain = pCutFinish;
    if( std::fabs( startConnInfo.angRange ) > 1e-3 ){
        mainConnInfo.pStart = rotate( startConnInfo.centroid, startConnInfo.pStart, startConnInfo.angRange );
        ret = getSemiCircle(startConnInfo, radius, angRes_rad);
    }

    if( std::fabs( finishConnInfo.angRange ) > 1e-3 ){
        pFinishMain = rotate( finishConnInfo.centroid, finishConnInfo.pStart, finishConnInfo.angRange );
        connectionFinish = getSemiCircle(finishConnInfo, radius, angRes_rad);
        std::reverse(connectionFinish.begin(), connectionFinish.end());
    }

    mainConnInfo.angRange = getAngularRange( mainConnInfo.pStart, pFinishMain, centroid, negativeAngDir );
    connectionMain = getSemiCircle(mainConnInfo, radius, angRes_rad);

    if(!ret.empty())
        ret.pop_back();
    ret.insert( ret.end(), connectionMain.begin(), connectionMain.end() );
    if(!connectionFinish.empty())
        ret.insert( ret.end(), connectionFinish.begin()+1, connectionFinish.end() );

    return ret;
}

std::vector<Point> PathSmoother::getSmoothCorner_dubins(Point &p0, Point &p1, Point &p2, double distFromCorner, double radius, double angRes_rad)
{
    std::vector<Point> ret;
    distFromCorner += radius * 1e-3;

    if(angRes_rad == 0)
        return ret;

    if( calc_dist(p0, p1) < distFromCorner || calc_dist(p1, p2) < distFromCorner )//segments not long enough
        return ret;

    DubinsParams dp;
    dp.p1 = getPointInLineAtDist(p1, p0, distFromCorner);
    dp.p2 = getPointInLineAtDist(p1, p2, distFromCorner);
    dp.rho1 = get_angle(p0, p1);
    dp.rho2 = get_angle(p1, p2);
    dp.type = DubinsParams::SHORTEST;

    double ang_abs = std::fabs( get_angle(p0, p1, p2) );
    size_t numSamples;
    if(ang_abs < M_PI_2)
        numSamples = 2 * M_PI / std::fabs(angRes_rad) + 1;
    else
        numSamples = std::max( 2.0, (M_PI - ang_abs) / std::fabs(angRes_rad) + 1 );


    ret = calcDubinsPath(dp, radius, angRes_rad, false);

    return ret;
}

bool PathSmoother::smoothenPath_dubins(std::vector<Point> &points_us, size_t ind_from, std::vector<Point> &points_out,
                                                   double radius, double angRes_rad, double angThreshold_rad,
                                                   PathSmoother::SmoothenPathCornerStrategy cornerStrategy_sharpCorners, PathSmoother::SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners){
    if(ind_from + 2 >= points_us.size())
        return false;
//    if(ind_from + 3 >= points_us.size())
//        return false;

    if(points_out.empty())
        points_out.push_back(points_us.at(ind_from));

    size_t indConsCornersEnd = findIndOfNextLongEnoughSegment(points_us, ind_from, radius, angThreshold_rad,
                                                              cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners,
                                                              false, 1);
    if(indConsCornersEnd == 0)
        indConsCornersEnd = points_us.size() -1;

    double distCmpr1 = 1.2*radius;
    double distCmpr2 = radius;
    for(; indConsCornersEnd+1 < points_us.size() ; ++indConsCornersEnd){
        double dist_tmp = calc_dist(points_us.at(indConsCornersEnd-1), points_us.at(indConsCornersEnd));
        if( calc_dist(points_us.at(indConsCornersEnd-1), points_us.at(indConsCornersEnd)) > distCmpr1
                && calc_dist(points_us.at(indConsCornersEnd), points_us.at(indConsCornersEnd+1)) > distCmpr2 )
            break;
    }

//    double distCmpr = 2 * radius;
//    size_t indConsCornersEnd = points_us.size() -1;
//    for(size_t i = ind_from ; i+1 < points_us.size() ; ++i){
//        if( calc_dist(points_us.at(i), points_us.at(i+1)) > distCmpr ){
//            indConsCornersEnd = i+1;
//            break;
//        }
//    }

    if(ind_from + 1 >= indConsCornersEnd)//no consecutive corners exist
        return false;

//    if(ind_from + 2 >= indConsCornersEnd)//no consecutive corners exist
//        return false;


    bool segmentsIntersect = false, segmentsIntersect_prev = false, segmentsIntersect_next = false;
    std::vector<Point> seg(points_us.begin()+ind_from, points_us.begin()+indConsCornersEnd+1), seg_prev, seg_next;

    double distCheckIntersections = 2*radius;

    if(points_out.size() == 1){//extend first segment of points_us backwards
        seg_prev.push_back( points_us.at(ind_from) );
        seg_prev.push_back( extend_line( points_us.at(ind_from+1), points_us.at(ind_from), distCheckIntersections ) );
    }
    else{
        int ind_tmp;
        double dist_tmp = 0;
        for(ind_tmp = points_out.size()-1 ; ind_tmp > 0 ; ind_tmp--){
            dist_tmp += calc_dist( points_out.at(ind_tmp-1), points_out.at(ind_tmp) );
            if(dist_tmp >= distCheckIntersections){
                ind_tmp--;
                break;
            }
        }
        seg_prev.insert( seg_prev.end(), points_out.begin()+ind_tmp, points_out.end() );
        if(dist_tmp < distCheckIntersections)
            seg_prev.front() = extend_line( seg_prev.at(1), seg_prev.front(), distCheckIntersections-dist_tmp );

    }

    if(indConsCornersEnd+1 >= points_us.size()){//extend last segment of points_us
        seg_next.push_back( points_us.at(indConsCornersEnd) );
        seg_next.push_back( extend_line( points_us.at(indConsCornersEnd-1), points_us.at(indConsCornersEnd), distCheckIntersections ) );
    }
    else{
        int ind_tmp;
        double dist_tmp = 0;
        for(ind_tmp = indConsCornersEnd ; ind_tmp+1 < points_us.size() ; ind_tmp++){
            dist_tmp += calc_dist( points_us.at(ind_tmp), points_us.at(ind_tmp+1) );
            if(dist_tmp >= distCheckIntersections){
                ind_tmp++;
                break;
            }
        }
        seg_next.insert( seg_next.end(), points_us.begin()+indConsCornersEnd, points_us.begin()+ind_tmp+1 );
        if(dist_tmp < distCheckIntersections)
            seg_next.back() = extend_line( r_at(seg_next, 1), seg_next.back(), distCheckIntersections-dist_tmp );

    }

    segmentsIntersect = intersects(seg);

    seg.pop_back();
    pop_front(seg);

    segmentsIntersect_prev = intersects( seg, seg_prev );
    segmentsIntersect_next = intersects( seg, seg_next );

    std::vector<DubinsParams> dubParams(2);

    double rho1 = get_angle(points_us.at(ind_from), points_us.at(ind_from+1));
    double rho2 = get_angle(points_us.at(indConsCornersEnd-1), points_us.at(indConsCornersEnd));

    {//include first and last points
        dubParams.front().p1 = points_us.at(ind_from);
        dubParams.back().p1 = points_us.at(ind_from+1);
        dubParams.front().p2 = points_us.at(indConsCornersEnd);
        dubParams.back().p2 = points_us.at(indConsCornersEnd-1);
        //        if( ind_from > 0 )
        //            dubParams.front().rho1 = get_angle(points_us.at(ind_from-1), points_us.at(ind_from));
        //        else
        dubParams.front().rho1 = rho1;
        dubParams.back().rho1 = rho1;
        if( indConsCornersEnd + 1 < points_us.size() )
            dubParams.front().rho2 = get_angle(points_us.at(indConsCornersEnd), points_us.at(indConsCornersEnd+1));
        else
            dubParams.front().rho2 = rho2;
        dubParams.back().rho2 = rho2;
    }

    {//for intermediate points
        double d_ref = 1*radius;
        std::vector<Point> v_p0 = {points_us.at(ind_from), points_us.at(ind_from+1)};
        std::vector<Point> v_pn = {points_us.at(indConsCornersEnd), points_us.at(indConsCornersEnd-1)};
        if( calc_dist(points_us.at(ind_from), points_us.at(ind_from+1)) > d_ref )
            v_p0.push_back( getPointInLineAtDist(points_us.at(ind_from+1), points_us.at(ind_from), d_ref) );
        //v_p0.push_back( getPointInLineAtDist(points_us.at(ind_from+1), points_us.at(ind_from), -d_ref) );
        if( calc_dist(points_us.at(indConsCornersEnd), points_us.at(indConsCornersEnd-1)) > d_ref )
            v_pn.push_back( getPointInLineAtDist(points_us.at(indConsCornersEnd-1), points_us.at(indConsCornersEnd), d_ref) );
        //v_pn.push_back( getPointInLineAtDist(points_us.at(indConsCornersEnd-1), points_us.at(indConsCornersEnd), -d_ref) );

        for(size_t i_0 = 0 ; i_0 < v_p0.size() ; i_0++){
            for(size_t i_n = 0 ; i_n < v_pn.size() ; i_n++){
                if( (i_0 == 0 && i_n == 0) || (i_0 == 1 && i_n == 1) )//added already
                    continue;
                DubinsParams dp;
                dp.p1 = v_p0.at(i_0);
                dp.p2 = v_pn.at(i_n);
                i_0 == 0 ? dp.rho1 = dubParams.front().rho1: dp.rho1 = rho1;
                i_n == 0 ? dp.rho2 = dubParams.front().rho2: dp.rho2 = rho2;
                dubParams.push_back(dp);
            }
        }
    }


    double minDiff = std::numeric_limits<double>::max();

    std::vector<Point> points_0_comp( points_us.begin()+ind_from , points_us.begin()+indConsCornersEnd+1 );

    std::pair<DubinsParams, double> bestDP {DubinsParams(), -1};

    double radius_d = radius * 0.995;//in case millimessimal differences cause dubins to generate long paths
    for(auto& dp : dubParams){
        for(int i = 0 ; i < DubinsParams::SHORTEST ; ++i){
            dp.type = DubinsParams::SHORTEST;

            if(i == DubinsParams::LSL) dp.type = DubinsParams::LSL;
            else if(i == DubinsParams::LSR) dp.type = DubinsParams::LSR;
            else if(i == DubinsParams::RSL) dp.type = DubinsParams::RSL;
            else if(i == DubinsParams::RSR) dp.type = DubinsParams::RSR;
            else if(i == DubinsParams::RLR) dp.type = DubinsParams::RLR;
            else if(i == DubinsParams::LRL) dp.type = DubinsParams::LRL;

            double len;
            std::vector<Point> points_tmp = calcDubinsPath(dp, radius_d, angRes_rad, false, &len);
            if(points_tmp.empty())
                continue;

            unsample_linestring(points_tmp);

            auto seg_d = points_tmp;
            if( seg_d.front() != points_us.at(ind_from) )
                push_front(seg_d, points_us.at(ind_from));
            if( seg_d.back() != points_us.at(indConsCornersEnd) )
                seg_d.push_back( points_us.at(indConsCornersEnd) );

            if( !segmentsIntersect && intersects(seg_d) )
                continue;

            seg_d.pop_back();
            pop_front(seg_d);
            if( !segmentsIntersect_prev && intersects(seg_d, seg_prev) )
                continue;
            if( !segmentsIntersect_next && intersects(seg_d, seg_next) )
                continue;

            std::swap(points_0_comp.front(), dp.p1);
            std::swap(points_0_comp.back(), dp.p2);
            double diffTmp = computeDifferenceBetweenPaths(points_0_comp, points_tmp, radius);
            std::swap(points_0_comp.front(), dp.p1);
            std::swap(points_0_comp.back(), dp.p2);

            if( minDiff > diffTmp ){
                minDiff = diffTmp;
                bestDP.first = dp;
                bestDP.second = len;
            }

        }

    }

    if(bestDP.second <= 0)
        return false;

    std::vector<Point> points_d = calcDubinsPath(bestDP.first, radius_d, angRes_rad, false);

    if(points_d.empty())
        return false;

    //unsample_linestring(points_d);

    bool lastIncluded = points_d.back() == points_us.at(indConsCornersEnd);
    points_out.insert( points_out.end(), points_d.begin(), points_d.end() );
    if(indConsCornersEnd+1 >= points_us.size()){
        if(!lastIncluded)
            points_out.push_back( points_us.at(indConsCornersEnd) );
        return true;
    }

    if(lastIncluded){
        return smoothenPath_recursive( points_us, indConsCornersEnd, points_out, radius, angRes_rad, angThreshold_rad,
                                       true, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners );
    }

    Point pTmp = points_us.at(indConsCornersEnd-1);
    points_us.at(indConsCornersEnd-1) = points_out.back();
    bool ok = smoothenPath_recursive( points_us, indConsCornersEnd-1, points_out, radius, angRes_rad, angThreshold_rad,
                                      true, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners );
    points_us.at(indConsCornersEnd-1) = pTmp;
    return ok;
}

bool PathSmoother::getMultiCornerSmoothPath(std::vector<Point> &points_us, size_t ind_from, std::vector<Point> &points_out,
                                            double radius, double angRes_rad, double angThreshold_rad,
                                            PathSmoother::SmoothenPathCornerStrategy cornerStrategy_sharpCorners, PathSmoother::SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners){
    if(ind_from + 3 >= points_us.size())
        return false;

    if(points_out.empty())
        points_out.push_back(points_us.at(ind_from));

    std::vector<Point> seg = {points_us.at(ind_from)};
    double angRefConsCorners = get_angle( points_us.at(ind_from), points_us.at(ind_from+1), points_us.at(ind_from+2) );
    double angTotal = 0;
    bool includeIndConsCornersEndInRad = false;
    bool segmentsIntersect = false;

    size_t indConsCornersEnd, indConsCornersEnd__inclFirst;
    size_t ind_from_search = ind_from;

    indConsCornersEnd__inclFirst = findIndOfNextLongEnoughSegment(points_us, ind_from, radius, angThreshold_rad,
                                                                  cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners,
                                                                  true);
    if(indConsCornersEnd__inclFirst != ind_from+1)
        ind_from_search = ind_from+2;

    indConsCornersEnd = findIndOfNextLongEnoughSegment(points_us, ind_from_search, radius, angThreshold_rad,
                                                       cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners,
                                                       ind_from_search != ind_from);

    if(indConsCornersEnd == 0)
        indConsCornersEnd = points_us.size() -1;

    for(size_t i = ind_from+1; i+1 <= indConsCornersEnd ; ++i){
        Point p0 = points_us.at(i-1);
        Point p1 = points_us.at(i);
        Point p2 = points_us.at(i+1);
        double ang = get_angle(p0, p1, p2);

        //        if( ang * angRefConsCorners < 0 //change of turning direction
        //                //|| intersects( p1, p2, seg, false, false ) //segments intersect
        //           ){
        //            indConsCornersEnd = i;
        //            includeIndConsCornersEndInRad = true;
        //            break;
        //        }

        angTotal += (M_PI - std::fabs(ang) );

        if(angTotal >= 2 * M_PI){
            indConsCornersEnd = i+1;
            break;
        }

        if( i+2 <= indConsCornersEnd ){
            Point p3 = points_us.at(i+2);
            double ang2 = get_angle(p1, p2, p3);
            if( ang2 * angRefConsCorners < 0){//change of turning direction in next corner
                indConsCornersEnd = i+1;

                if(ind_from + 2 >= indConsCornersEnd)
                    return smoothenPath_dubins(points_us, ind_from, points_out, radius, angRes_rad, angThreshold_rad, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);

                break;
            }
        }

        if( intersects( p1, p2, seg, false, false ) ) {//segments intersect
            indConsCornersEnd = i+1;
            segmentsIntersect = true;
            break;
        }

        seg.push_back( p1 );
    }
    seg.clear();

    if(ind_from + 2 >= indConsCornersEnd)//no consecutive corners exist
        return false;

    if(angTotal >= 2 * M_PI){//@todo see how to deal with these cases
        //return false;

        return smoothenPath_dubins(points_us, ind_from, points_out, radius, angRes_rad, angThreshold_rad,
                                               cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);
    }

    if(angTotal <= M_PI_2){

        auto ind_prev = ind_from;
        double dist_prev = 0;
        while(dist_prev < 2*radius){
            if(ind_prev == 0)
                break;
            --ind_prev;
            dist_prev += calc_dist( points_us.at(ind_prev), points_us.at(ind_prev+1) );
        }

        Point pNew;
        get_intersection( points_us.at( ind_from ), points_us.at( ind_from+1 ),
                          points_us.at( indConsCornersEnd ), points_us.at( indConsCornersEnd-1 ),
                          pNew, true, true);
        std::vector<Point> pointsTmp(points_us.begin()+ind_prev, points_us.begin()+ind_from+1);
        auto ind_from_new = pointsTmp.size()-1;
        pointsTmp.push_back(pNew);
        pointsTmp.push_back(points_us.at(indConsCornersEnd));

        if(!smoothenPath_recursive(pointsTmp, ind_from_new, points_out,
                                   radius, angRes_rad, angThreshold_rad,
                                   false,
                                   SPCS_DO_NOT_REACH_CORNER_POINT_IF_POSSIBLE,
                                   SPCS_DO_NOT_REACH_CORNER_POINT_IF_POSSIBLE)){
            //return getMultiCornerSmoothPath_dubins(points_us, ind_from, points_out, radius, angRes_rad, angThreshold_rad, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);
            return false;
        }

        points_out.pop_back();

        Point pTmp = points_us.at(indConsCornersEnd-1);
        points_us.at(indConsCornersEnd-1) = points_out.back();
        bool ok = smoothenPath_recursive( points_us, indConsCornersEnd-1, points_out, radius, angRes_rad, angThreshold_rad,
                                          true, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners );
        points_us.at(indConsCornersEnd-1) = pTmp;
        return ok;
    }

    double rad = radius;
    Point centroid(0,0);

    enum CentriodType{
        GEOMETRIC_CENTROID,
        AVG_POINT,
        AVG_POINT_IN_BISECTORS
    } centriodType = AVG_POINT_IN_BISECTORS;

    if(centriodType == GEOMETRIC_CENTROID){
        Polygon polyTmp;
        polyTmp.points.insert( polyTmp.points.end(), points_us.begin()+ind_from+1, points_us.begin()+indConsCornersEnd+includeIndConsCornersEndInRad );
        if(polyTmp.points.size() > 2){
            if(!getCentroid(polyTmp, centroid))
                return false;
        }
        else
            centroid = getCentroid(polyTmp.points.front(), polyTmp.points.back());
    }
    else if(centriodType == AVG_POINT){
        int count = 0;
        for(size_t i = ind_from+1 ; i < indConsCornersEnd + includeIndConsCornersEndInRad ; ++i){
            centroid.x += points_us.at(i).x;
            centroid.y += points_us.at(i).y;
            ++count;
        }
        if(count == 0)
            return false;
        centroid.x /= count;
        centroid.y /= count;
    }
    else{
        int count = 0;
        for(size_t i = ind_from+1 ; i+1 <= indConsCornersEnd ; ++i){
            Point p0 = points_us.at(i-1);
            Point p1 = points_us.at(i);
            Point p2 = points_us.at(i+1);
            double ang = get_angle(p0, p1, p2);
            Point p = rotate(p1, p0, 0.5 * ang);
            p = getPointInLineAtDist(p1, p, radius);

            centroid.x += p.x;
            centroid.y += p.y;
            ++count;
        }
        if(count == 0)
            return false;
        centroid.x /= count;
        centroid.y /= count;

    }

    for(size_t i = ind_from ; i+1 <= indConsCornersEnd ; ++i){
        Point p1 = points_us.at(i);
        Point p2 = points_us.at(i+1);
        rad = std::max(rad, calc_dist_to_line(p1, p2, centroid, false));
    }
    if(includeIndConsCornersEndInRad)
        rad = std::max(rad, calc_dist(centroid, points_us.at(indConsCornersEnd)));

    //    rad = std::max(rad, calc_dist_to_line(points_us.at(ind_from), points_us.at(ind_from+1), centroid, false));
    //    rad = std::max(rad, calc_dist_to_line(points_us.at(indConsCornersEnd), points_us.at(indConsCornersEnd-1), centroid, false));

    //check if the circle with that radius intersects the first and las segments
    double d_0_0 = calc_dist( points_us.at(ind_from), centroid );
    double d_0_1 = calc_dist( points_us.at(ind_from+1), centroid );
    double d_n_0 = calc_dist( points_us.at(indConsCornersEnd+includeIndConsCornersEndInRad-1), centroid );
    double d_n_1 = calc_dist( points_us.at(indConsCornersEnd+includeIndConsCornersEndInRad), centroid );

    bool intersects_0 = d_0_0>=rad || d_0_1>=rad //beacause we calculated the rad with the distance to the segment
            || ( d_0_0>rad && d_0_1<rad ) || ( d_0_1>rad && d_0_0<rad );
    bool intersects_n = d_n_0>=rad || d_n_1>=rad //beacause we calculated the rad with the distance to the segment
            || ( d_n_0>rad && d_0_1<rad ) || ( d_n_1>rad && d_0_0<rad );
    if(!intersects_0 || !intersects_n){
        double rad0 = rad, radn = rad;
        if(!intersects_0)
            rad0 = std::max(d_0_0, d_0_1);
        if(!intersects_n)
            radn = std::max(d_n_0, d_n_1);
        rad = std::min(rad0, radn);

        if(rad < radius){//@todo: move the centroid so that the distance constraints are fulfilled
            //return false;

            return smoothenPath_dubins(points_us, ind_from, points_out, radius, angRes_rad, angThreshold_rad,
                                                   cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);
        }
    }

    //    double d_0 = calc_dist( points_us.at(ind_from), centroid );
    //    double d_n = calc_dist( points_us.at(indConsCornersEnd+includeIndConsCornersEndInRad), centroid );
    //    if( d_0 < rad || d_n < rad )//first and/or last segment are not long enough for the first and last connections
    //        return false;

    std::vector<Point> intersections_0, intersections_n;
    //    double d_l0 = calc_dist_to_line( points_us.at(ind_from), points_us.at(ind_from+1), centroid );
    //    double d_ln = calc_dist_to_line( points_us.at(indConsCornersEnd+includeIndConsCornersEndInRad+segmentsIntersect-1), points_us.at(indConsCornersEnd+includeIndConsCornersEndInRad), centroid );
    //    if( std::fabs(rad - d_l0) < rad*1e-3 ){//no need for extra smoothening in the initial connection with the (semi) circle
    //        std::vector<Point> pointsTmp = {points_us.at(ind_from), points_us.at(ind_from+1)};
    //        intersections_0.push_back( pointsTmp.at( addSampleToGeometryClosestToPoint(pointsTmp, centroid, 1) ) );
    //    }
    //    if( std::fabs(rad - d_ln) < rad*1e-3 ){//no need for extra smoothening in the last connection with the (semi) circle
    //        std::vector<Point> pointsTmp = {points_us.at(indConsCornersEnd-1), points_us.at(indConsCornersEnd)};
    //        intersections_n.push_back( pointsTmp.at( addSampleToGeometryClosestToPoint(pointsTmp, centroid, 1) ) );
    //    }

    //    if(intersections_0.empty() || intersections_n.empty()){//get intersections with the first and last segment (if needed)
    //        auto circleTmp = create_circle(centroid, rad*1.01, 360);
    //        if(intersections_0.empty())
    //            intersections_0 = get_intersection( points_us.at(ind_from), points_us.at(ind_from+1), circleTmp.points, true, false, false );
    //        if(intersections_n.empty())
    //            intersections_n = get_intersection( points_us.at(indConsCornersEnd+includeIndConsCornersEndInRad), points_us.at(indConsCornersEnd+includeIndConsCornersEndInRad-1), circleTmp.points, true, false, false );
    //    }

    if(intersections_0.empty() || intersections_n.empty()){//get intersections with the first and last segment
        Point p0 = points_us.at(ind_from);
        Point p1 = points_us.at(ind_from+1);
        double d = calc_dist( p0, centroid );
        double dSeg = calc_dist( p0, p1 );
        double ang1 = std::fabs( get_angle(centroid, p0, p1) );
        double distToP0;
        if(ang1 < M_PI && ang1 > 0){
            double ang2 = asin( std::min( 1.0, d * sin(ang1) / rad ) );
            double ang3_1 = M_PI - ang1 - ang2;
            double ang3_2 = ang2 - ang1;//arcsin can give the "wrong" angle
            double distToP0_1 = sin(ang3_1) * rad / sin(ang1);
            double distToP0_2 = sin(ang3_2) * rad / sin(ang1);
            if( distToP0_1<0 || distToP0_2<0 ){
                distToP0 = std::max(distToP0_1,distToP0_2 );
            }
            else{
                distToP0 = std::min(distToP0_1,distToP0_2 );
            }
        }
        else
            distToP0 = std::fabs(rad - d);
        intersections_0.clear();
        intersections_0.push_back( getPointInLineAtDist(p0, p1, distToP0) );

        p0 = points_us.at(indConsCornersEnd+includeIndConsCornersEndInRad);
        p1 = points_us.at(indConsCornersEnd+includeIndConsCornersEndInRad-1);
        d = calc_dist( p0, centroid );
        dSeg = calc_dist( p0, p1 );
        ang1 = std::fabs( get_angle(centroid, p0, p1) );
        if(ang1 < M_PI && ang1 > 0){
            double ang2 = asin( std::min( 1.0, d * sin(ang1) / rad ) );
            double ang3_1 = M_PI - ang1 - ang2;
            double ang3_2 = ang2 - ang1;//arcsin can give the "wrong" angle
            double distToP0_1 = sin(ang3_1) * rad / sin(ang1);
            double distToP0_2 = sin(ang3_2) * rad / sin(ang1);
            if( distToP0_1<0 || distToP0_2<0 )
                distToP0 = std::max(distToP0_1,distToP0_2 );
            else
                distToP0 = std::min(distToP0_1,distToP0_2 );
        }
        else
            distToP0 = std::fabs(rad - d);
        intersections_n.clear();
        intersections_n.push_back( getPointInLineAtDist(p0, p1, distToP0) );
    }


    if(intersections_0.empty() || intersections_n.empty()){
        return smoothenPath_dubins(points_us, ind_from, points_out, radius, angRes_rad, angThreshold_rad,
                                               cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);
        //return false;
    }

    Point pStart = intersections_0.front();
    Point pFinish = intersections_n.front();

    if(points_us.at(ind_from) == pStart){
        if( std::fabs( get_angle( points_us.at(ind_from), points_us.at(ind_from+1), points_us.at(ind_from+2) ) ) >= angThreshold_rad
                /*|| ind_from == 0*/ ){
            points_out.push_back( points_us.at(ind_from+1) );
            return smoothenPath_recursive( points_us, ind_from+1, points_out, radius, angRes_rad, angThreshold_rad,
                                           true, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);
        }
        return smoothenPath_dubins(points_us, ind_from, points_out, radius, angRes_rad, angThreshold_rad,
                                               cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);
        //return false;
    }

    if(points_us.at(indConsCornersEnd) == pFinish){
        /*if( indConsCornersEnd < points_us.size())*/{
            return smoothenPath_dubins(points_us, ind_from, points_out, radius, angRes_rad, angThreshold_rad,
                                                   cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);
            //return false;
        }
    }

    double angRange = get_angle( pStart, centroid, pFinish );
    if(angRefConsCorners * angRange > 0 ){
        angRange = 2 * M_PI - std::fabs(angRange);
        if(angRefConsCorners > 0)
            angRange *= -1;
    }

//    std::vector<Point> connection = getSmoothCornerFromCircle(points_us, ind_from, indConsCornersEnd, pStart, pFinish, centroid,
//                                                              rad, angRes_rad, angThreshold_rad, angRange < 0, true,
//                                                              cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners );

    std::vector<Point> connection = getSmoothCornerFromCircles(points_us.at(ind_from), points_us.at(indConsCornersEnd), pStart, pFinish, centroid,
                                                              rad, angRes_rad, angRange < 0, true );


    if(connection.empty()){
        return smoothenPath_dubins(points_us, ind_from, points_out, radius, angRes_rad, angThreshold_rad,
                                               cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners);
        //return false;
    }

    points_out.insert(points_out.end(), connection.begin(), connection.end());

    if(indConsCornersEnd == points_us.size() - 1){
        points_out.push_back( points_us.back() );
        return true;
    }

    Point pTmp = points_us.at(indConsCornersEnd-1);
    points_us.at(indConsCornersEnd-1) = points_out.back();
    bool ok = smoothenPath_recursive( points_us, indConsCornersEnd-1, points_out, radius, angRes_rad, angThreshold_rad,
                                      true, cornerStrategy_sharpCorners, cornerStrategy_nonSharpCorners );
    points_us.at(indConsCornersEnd-1) = pTmp;
    return ok;

}

std::vector<Point> PathSmoother::sampleLine(const Point &p0, const Point &p1, const std::multiset<double> &samplesRef){

    if(p0 == p1)
        return {p0};

    double dist = calc_dist(p0, p1);
    std::vector<Point> ret;
    for(auto& it : samplesRef){
        auto v = std::min( 1.0, std::max( 0.0, it ) );
        ret.emplace_back( getPointInLineAtDist(p0, p1, v*dist) );
    }
    return ret;
}

}

}
