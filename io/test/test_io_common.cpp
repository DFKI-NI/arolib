/*
 * Copyright 2021-2025 DFKI GmbH
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


#ifndef AROLIB_IO_TEST_COMMON_H
#define AROLIB_IO_TEST_COMMON_H

#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>

#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/tracksgenerator.h"

arolib::Field getTestField(const arolib::Point& trans = arolib::Point(0,0)){

    arolib::Point pRef = arolib::Point(564300, 5762400) + trans;
    const double dx = 200;
    const double dy = 100;
    const double hl_width = 20;
    arolib::Polygon polyTmp;

    arolib::Field field;
    field.id = 100;
    field.name = "TestField";

    field.outer_boundary = arolib::geometry::createRectangleFromLine(pRef, pRef + arolib::Point(2*dx, 0), dy);

    arolib::geometry::offsetPolygon(field.outer_boundary, polyTmp, 10, true, 0);
    field.external_roads.emplace_back(polyTmp.points);
    arolib::geometry::offsetPolygon(field.outer_boundary, polyTmp, 20, true, 0);
    field.external_roads.emplace_back(polyTmp.points);

    for(size_t s = 0 ; s < 2 ; ++s){
        field.subfields.emplace_back(arolib::Subfield());
        auto& sf = field.subfields.back();
        sf.id = 1000 + s;
        sf.boundary_outer = arolib::geometry::createRectangleFromLine(pRef + arolib::Point(s*dx, 0), pRef + arolib::Point(dx*(1+s), 0), dy);
        arolib::geometry::offsetPolygon(sf.boundary_outer, sf.boundary_inner, hl_width, false, 0);

        sf.headlands.complete.headlandWidth = hl_width;
        sf.headlands.complete.boundaries = std::make_pair(sf.boundary_outer, sf.boundary_inner);
        for(int i = 5 ; i < hl_width ; i += 5){
            arolib::geometry::offsetPolygon(sf.boundary_outer, polyTmp, i, false, 0);
            sf.headlands.complete.tracks.emplace_back(arolib::Track());
            sf.headlands.complete.tracks.back().id = 1000*s + i;
            sf.headlands.complete.tracks.back().points = polyTmp.points;
        }
        sf.reference_lines.emplace_back(arolib::Linestring({pRef, pRef + arolib::Point(dx*s, 50)}));

        arolib::geometry::TracksGenerator tg;
        arolib::geometry::TracksGenerator::TracksGeneratorParameters tgp;
        tgp.trackDistance = tgp.sampleResolution = 10;
        tg.generateTracks(sf, tgp, 0);

        for(size_t i = 0 ; i+1 < sf.boundary_outer.points.size() ; ++i)
            sf.access_points.emplace_back(arolib::FieldAccessPoint(sf.boundary_outer.points.at(i), i));

        arolib::Point center;
        arolib::geometry::getCentroid(sf.boundary_outer, center);
        sf.obstacles.emplace_back( arolib::geometry::create_circle(center, 5, 10, 0.0, -2*M_PI, false, true) );
        sf.obstacles.emplace_back( arolib::geometry::create_circle(center + arolib::Point(5, 0), 2, 10, 0.0, -2*M_PI, false, true) );

        auto updateResourcePoint = [&sf, &s](){
            auto& rp = sf.resource_points.back();
            rp.geometry = arolib::geometry::create_circle(rp, 1, 5, 0.0, -2*M_PI, false, true);
            rp.massCapacity = 10 * rp.id;
            rp.massCapacity = 100 * rp.id;
        };

        sf.resource_points.emplace_back( arolib::ResourcePoint(field.external_roads.back().points.front(), 10*s + 1 ) );
        updateResourcePoint();
        sf.resource_points.emplace_back( arolib::ResourcePoint( arolib::geometry::getPointAtHalfLength(field.external_roads.back().points).first, 10*s + 2 ) );
        updateResourcePoint();
    }


    return field;
}

#endif // AROLIB_IO_TEST_COMMON_H
