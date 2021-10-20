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

#include "arolib/geometry/curves_helper.hpp"
#include "arolib/types/units.hpp"

namespace arolib{

namespace geometry{

namespace{

template<size_t S>
std::vector<Point> _getBezierPoints(const std::vector<Point> &nodes, size_t num_samples){
    try{
        std::vector<Bezier::Point> bz_nodes( nodes.size() );
        std::vector<Point> ret(num_samples);

        for(size_t i = 0 ; i < nodes.size() ; ++i){
            bz_nodes[i].x = nodes[i].x - nodes.front().x;
            bz_nodes[i].y = nodes[i].y - nodes.front().y;
        }

        Bezier::Bezier<S> cubicBezier(bz_nodes);
        for(size_t i = 0 ; i < num_samples ; ++i){
            auto p = cubicBezier.valueAt( (float)i / (num_samples-1) );
            ret[i] = Point( nodes.front().x + p.x, nodes.front().y + p.y);
        }
        return ret;
    }
    catch(...){
        return {};
    }

}

template<size_t S>
std::vector<Point> _getBezierPoints(const std::vector<Point> &nodes, const std::multiset<double> &samplesRef){
    try{
        std::vector<Bezier::Point> bz_nodes( nodes.size() );

        for(size_t i = 0 ; i < nodes.size() ; ++i){
            bz_nodes[i].x = nodes[i].x - nodes.front().x;
            bz_nodes[i].y = nodes[i].y - nodes.front().y;
        }

        Bezier::Bezier<S> cubicBezier(bz_nodes);
        std::vector<Point> ret;

        for(auto& it : samplesRef){
            auto v = std::min( 1.0, std::max( 0.0, it ) );
            auto p = cubicBezier.valueAt(v);
            ret.emplace_back( Point( nodes.front().x + p.x, nodes.front().y + p.y) );
        }

        return ret;
    }
    catch(...){
        return {};
    }

}

std::vector<Point> sampleLine(const Point &p0, const Point &p1, const std::multiset<double> &samplesRef){

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


DubinsParams::PathType DubinsParams::intToPathType(int value)
{

    if(value == PathType::LSL)
        return PathType::LSL;
    else if(value == PathType::LSR)
        return PathType::LSR;
    else if(value == PathType::RSL)
        return PathType::RSL;
    else if(value == PathType::RSR)
        return PathType::RSR;
    else if(value == PathType::RLR)
        return PathType::RLR;
    else if(value == PathType::LRL)
        return PathType::LRL;
    else if(value == PathType::SHORTEST)
        return PathType::SHORTEST;

    throw std::invalid_argument( "The given value does not correspond to any DubinsParams::PathType" );
}

std::vector<Point> calcDubinsPath(const DubinsParams &dp, double radius, double resolution, double *length)
{
    std::vector<Point> path_out;
    DubinsPath path_d;
    bool ok = false;

    double lengthTmp;
    double& len = length ? *length : lengthTmp;
    len = 0;

    if(radius < 0 || resolution < 1e-9)
        return path_out;

    if(radius == 0){
        path_out.push_back(dp.p1);
        path_out.push_back(dp.p2);
        path_out = sample_geometry(path_out, resolution);
        len = calc_dist(dp.p1, dp.p2);
        return path_out;
    }

    double q0[] = { dp.p1.x, dp.p1.y, dp.rho1 };
    double q1[] = { dp.p2.x, dp.p2.y, dp.rho2 };

    if(dp.type == DubinsParams::SHORTEST)
        ok = dubins_shortest_path( &path_d, q0, q1, radius) == 0;
    else{
        DubinsPathType dpt = LRL;
        if(dp.type == DubinsParams::LSL) dpt = LSL;
        else if(dp.type == DubinsParams::LSR) dpt = LSR;
        else if(dp.type == DubinsParams::RSL) dpt = RSL;
        else if(dp.type == DubinsParams::RSR) dpt = RSR;
        else if(dp.type == DubinsParams::RLR) dpt = RLR;

        ok = dubins_path( &path_d, q0, q1, radius, dpt) == 0;
    }

    if(!ok)
        return path_out;

    len = dubins_path_length(&path_d);

    if( len < resolution+1e-9 ){
        path_out.emplace_back( dp.p1 );
        path_out.emplace_back( dp.p2 );
        return path_out;
    }

    int numSamples = std::ceil(len/resolution) + 1;
    double delta_d = len / (numSamples-1);

    path_out.reserve(numSamples);

    path_out.emplace_back( dp.p1 );
    for(int i = 1 ; i+1 < numSamples ; ++i){
        double t = i * delta_d;
        double q[3];
        if( dubins_path_sample(&path_d, t, q) != 0 ){
            path_out.clear();
            return path_out;
        }
        path_out.emplace_back( Point(q[0], q[1]) );
    }
    path_out.emplace_back( dp.p2 );

    return path_out;

}

std::vector<Point> calcDubinsPath(const Pose2D &start, const Pose2D &finish, double radius, double resolution, DubinsParams::PathType pathType, double *length)
{
    DubinsParams dp;
    dp.p1 = start;
    dp.rho1 = start.angle;
    dp.p2 = finish;
    dp.rho2 = finish.angle;
    dp.type = pathType;
    return calcDubinsPath(dp, radius, resolution, length);
}


std::vector<Point> calcDubinsPath(const DubinsParams &dp, double radius, double angResolution, bool inDeg, double *length)
{
    std::vector<Point> path_out;
    DubinsPath path_d;
    bool ok = false;

    double lengthTmp;
    double& len = length ? *length : lengthTmp;
    len = 0;

    if(radius < 0 || angResolution <= 0)
        return path_out;

    if(radius == 0){
        path_out.push_back(dp.p1);
        path_out.push_back(dp.p2);
        len = calc_dist(dp.p1, dp.p2);
        return path_out;
    }

    if(inDeg)
        angResolution = deg2rad( angResolution );

    double q0[] = { dp.p1.x, dp.p1.y, dp.rho1 };
    double q1[] = { dp.p2.x, dp.p2.y, dp.rho2 };

    if(dp.type == DubinsParams::SHORTEST)
        ok = dubins_shortest_path( &path_d, q0, q1, radius) == 0;
    else{
        DubinsPathType dpt = LRL;
        if(dp.type == DubinsParams::LSL) dpt = LSL;
        else if(dp.type == DubinsParams::LSR) dpt = LSR;
        else if(dp.type == DubinsParams::RSL) dpt = RSL;
        else if(dp.type == DubinsParams::RSR) dpt = RSR;
        else if(dp.type == DubinsParams::RLR) dpt = RLR;

        ok = dubins_path( &path_d, q0, q1, radius, dpt) == 0;
    }

    if(!ok)
        return path_out;

    len = dubins_path_length(&path_d);

    path_out.emplace_back( dp.p1 );

    double lenSum = 0;
    for(size_t seg = 0 ; seg < 3 ; seg++){
        double q[3];
        double len_seg = dubins_segment_length(&path_d, seg);

        if(seg != 1 || ( path_d.type == DubinsPathType::RLR || path_d.type == DubinsPathType::LRL ) ){
            double ang_seg = len_seg / radius;
            int numSamples_seg = std::ceil(ang_seg / angResolution);
            int lastSeg = (seg==2);
            if(numSamples_seg > 1){
                double delta_d = len_seg / (numSamples_seg-1);
                for(int i = 1 ; i+lastSeg < numSamples_seg ; ++i){
                    double t = lenSum + i * delta_d;
                    if( dubins_path_sample(&path_d, t, q) != 0 ){
                        path_out.clear();
                        return path_out;
                    }
                    path_out.emplace_back( Point(q[0], q[1]) );
                }
            }
        }
        else{//staright line -> add only the last point of the middle segment
            if( dubins_path_sample(&path_d, lenSum + len_seg, q) != 0 ){
                path_out.clear();
                return path_out;
            }
            path_out.emplace_back( Point(q[0], q[1]) );
        }

        lenSum += len_seg;

    }


    path_out.emplace_back( dp.p2 );

    return path_out;

}

std::vector<Point> calcDubinsPath(const Pose2D &start, const Pose2D &finish, double radius, double angResolution, bool inDeg, DubinsParams::PathType pathType, double *length)
{
    DubinsParams dp;
    dp.p1 = start;
    dp.rho1 = start.angle;
    dp.p2 = finish;
    dp.rho2 = finish.angle;
    dp.type = pathType;
    return calcDubinsPath(dp, radius, angResolution, inDeg, length);
}

std::vector<Point> calcDubinsPath(size_t numSamples, const DubinsParams &dp, double radius, double* length)
{
    std::vector<Point> path_out;
    DubinsPath path_d;
    bool ok = false;

    if(radius < 0 || numSamples < 2)
        return path_out;

    if( radius == 0 || (numSamples == 2 && !length) ){
        path_out.push_back(dp.p1);
        path_out.push_back(dp.p2);
        return path_out;
    }

    double lengthTmp;
    double& len = length ? *length : lengthTmp;
    len = 0;

    double q0[] = { dp.p1.x, dp.p1.y, dp.rho1 };
    double q1[] = { dp.p2.x, dp.p2.y, dp.rho2 };

    if(dp.type == DubinsParams::SHORTEST)
        ok = dubins_shortest_path( &path_d, q0, q1, radius) == 0;
    else{
        DubinsPathType dpt = LRL;
        if(dp.type == DubinsParams::LSL) dpt = LSL;
        else if(dp.type == DubinsParams::LSR) dpt = LSR;
        else if(dp.type == DubinsParams::RSL) dpt = RSL;
        else if(dp.type == DubinsParams::RSR) dpt = RSR;
        else if(dp.type == DubinsParams::RLR) dpt = RLR;

        ok = dubins_path( &path_d, q0, q1, radius, dpt) == 0;
    }

    if(!ok)
        return path_out;

    len = dubins_path_length(&path_d);

    double delta_d = len / (numSamples-1);

    path_out.reserve(numSamples);

    path_out.emplace_back( dp.p1 );
    for(int i = 1 ; i+1 < numSamples ; ++i){
        double t = i * delta_d;
        double q[3];
        if( dubins_path_sample(&path_d, t, q) != 0 ){
            path_out.clear();
            return path_out;
        }
        path_out.emplace_back( Point(q[0], q[1]) );
    }
    path_out.emplace_back( dp.p2 );

    return path_out;
}

std::vector<Point> calcDubinsPath(size_t numSamples, const Pose2D &start, const Pose2D &finish, double radius, DubinsParams::PathType pathType, double *length)
{
    DubinsParams dp;
    dp.p1 = start;
    dp.rho1 = start.angle;
    dp.p2 = finish;
    dp.rho2 = finish.angle;
    dp.type = pathType;
    return calcDubinsPath(numSamples, dp, radius, length);
}

double calcDubinsPathLength(const DubinsParams &dp, double radius)
{
    DubinsPath path_d;
    bool ok = false;

    if(radius < 0)
        return -1;

    if( radius == 0 )
        return calc_dist(dp.p1, dp.p2);

    double q0[] = { dp.p1.x, dp.p1.y, dp.rho1 };
    double q1[] = { dp.p2.x, dp.p2.y, dp.rho2 };

    if(dp.type == DubinsParams::SHORTEST)
        ok = dubins_shortest_path( &path_d, q0, q1, radius) == 0;
    else{
        DubinsPathType dpt = LRL;
        if(dp.type == DubinsParams::LSL) dpt = LSL;
        else if(dp.type == DubinsParams::LSR) dpt = LSR;
        else if(dp.type == DubinsParams::RSL) dpt = RSL;
        else if(dp.type == DubinsParams::RSR) dpt = RSR;
        else if(dp.type == DubinsParams::RLR) dpt = RLR;

        ok = dubins_path( &path_d, q0, q1, radius, dpt) == 0;
    }

    if(!ok)
        return -1;

    return dubins_path_length(&path_d);
}

double calcDubinsPathLength(const Pose2D &start, const Pose2D &finish, double radius, DubinsParams::PathType pathType)
{
    DubinsParams dp;
    dp.p1 = start;
    dp.rho1 = start.angle;
    dp.p2 = finish;
    dp.rho2 = finish.angle;
    dp.type = pathType;
    return calcDubinsPathLength(dp, radius);
}

std::vector<Point> getBezierPoints(const std::vector<Point> &nodes, size_t num_samples){
    size_t size = nodes.size();
    if(size < 2)
        return nodes;

    if(size < 3)
        return sample_line(nodes.front(), nodes.back(), num_samples);

    if(size > BezierMaxSampleSize)
        return {};

    if(num_samples < 2)
        return {};

    std::multiset<double> samplesRef = {0};
    double deltaRef = 1.0/(num_samples-1);
    for(size_t i = 1 ; i+1 < num_samples ; ++i)
        samplesRef.insert( samplesRef.end(), i*deltaRef );
    samplesRef.insert( 1 );

    return getBezierPoints(nodes, samplesRef);
}


std::vector<Point> getBezierPoints(const std::vector<Point> &nodes, const std::multiset<double> &samplesRef)
{
    size_t size = nodes.size();
    if(size < 2)
        return nodes;

    if(size < 3)
        return sampleLine(nodes.front(), nodes.back(), samplesRef);

    if(size > BezierMaxSampleSize)
        return {};

    switch (size){
    case 3:
        return _getBezierPoints<2>(nodes, samplesRef);
    case 4:
        return _getBezierPoints<3>(nodes, samplesRef);
    case 5:
        return _getBezierPoints<4>(nodes, samplesRef);
    case 6:
        return _getBezierPoints<5>(nodes, samplesRef);
    case 7:
        return _getBezierPoints<6>(nodes, samplesRef);
    case 8:
        return _getBezierPoints<7>(nodes, samplesRef);
    case 9:
        return _getBezierPoints<8>(nodes, samplesRef);
    case 10:
        return _getBezierPoints<9>(nodes, samplesRef);
    case 11:
        return _getBezierPoints<10>(nodes, samplesRef);
    case 12:
        return _getBezierPoints<11>(nodes, samplesRef);
    case 13:
        return _getBezierPoints<12>(nodes, samplesRef);
    case 14:
        return _getBezierPoints<13>(nodes, samplesRef);
    case 15:
        return _getBezierPoints<14>(nodes, samplesRef);
    case 16:
        return _getBezierPoints<15>(nodes, samplesRef);
    case 17:
        return _getBezierPoints<16>(nodes, samplesRef);
    case 18:
        return _getBezierPoints<17>(nodes, samplesRef);
    case 19:
        return _getBezierPoints<18>(nodes, samplesRef);
    case 20:
        return _getBezierPoints<19>(nodes, samplesRef);
    default:
        return {};
    }

}

std::vector<Point> getBSplinePoints(const std::vector<Point> &nodes, size_t num_samples, size_t order)
{
    if(nodes.size() < 2)
        return nodes;

    if(nodes.size() < 3)
        return sample_line(nodes.front(), nodes.back(), num_samples);

    if(order < 2)
        return {};

    Spline<Point, double> s(order);
    s.set_ctrl_points(nodes);

    std::vector<Point> ret(num_samples);
    for(size_t i = 0 ; i < num_samples ; ++i){
        ret[i] = s.eval_f( (float)i / (num_samples-1) );
    }
    return ret;

}

std::vector<Point> getBSplinePoints(const std::vector<Point> &nodes, const std::multiset<double> &samplesRef, size_t order)
{
    if(nodes.size() < 2)
        return nodes;

    if(nodes.size() < 3)
        return sampleLine(nodes.front(), nodes.back(), samplesRef);

    if(order < 2)
        return {};

    Spline<Point, double> s(order);
    s.set_ctrl_points(nodes);

    std::vector<Point> ret;

    for(auto& it : samplesRef){
        auto v = std::min( 1.0, std::max( 0.0, it ) );
        ret.emplace_back( s.eval_f(v) );
    }
    return ret;
}



}

}
