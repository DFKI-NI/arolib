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
 
#include <boost/test/unit_test.hpp>


#include "arolib/components/drivenmassareamanager.h"
#include "arolib/misc/filesystem_helper.h"

BOOST_AUTO_TEST_SUITE(test_drivenmassareamanager)

const float cellSize = 1;
const float machineWidth = 3;

const auto _output_dir = arolib::io::create_path( arolib::io::get_temp_dir(), "arolib", "test", "test_components", "test_drivenmassareamanager" );
const std::string& getOutputDir()
{
    arolib::io::create_directory(_output_dir, false);
    return _output_dir;
}

arolib::Field getTestField()
{
    arolib::Field field;
    field.outer_boundary.points.push_back( arolib::Point( 564200, 5762400 ) );
    field.outer_boundary.points.push_back( arolib::Point( 564300, 5762400 ) );
    field.outer_boundary.points.push_back( arolib::Point( 564300, 5762500 ) );
    field.outer_boundary.points.push_back( arolib::Point( 564200, 5762500 ) );
    field.outer_boundary.points.push_back(field.outer_boundary.points.front());
    return field;
}

std::vector<arolib::Point> getPathWithoutCrossing(const arolib::Point& refPoint){
    std::vector<arolib::Point> path;

    auto p = refPoint;

    p.x += 20*cellSize;
    p.y += 40*cellSize;
    path.push_back(p);

    p.x += 20*cellSize;
    path.push_back(p);

    p.x += 40*cellSize;
    p.y += 20*cellSize;
    path.push_back(p);

    p.x += -50*cellSize;
    path.push_back(p);

    //
    //      ----------
    //               /
    //              /
    //             /
    //            /
    //   ---------
    //
    //

    return path;
}

std::vector<arolib::Point> getPathWithCrossing(const arolib::Point& refPoint){
    std::vector<arolib::Point> path = getPathWithoutCrossing(refPoint);

    auto p = path.back();
    p.y += -30*cellSize;
    path.push_back(p);

    //
    //      ----------
    //     |         /
    //     |        /
    //     |       /
    //     |      /
    //   --|------
    //     |
    //

    return path;
}

std::vector<arolib::Point> getCloseCircularPath(const arolib::Point& center, size_t cycles){

    assert(cycles > 0);

    auto circle = arolib::geometry::create_circle(center, machineWidth/2, 9);

    std::vector<arolib::Point> path;
    for(size_t i = 0 ; i < cycles ; ++i)
        path.insert(path.end(), circle.points.begin(), circle.points.end()-1);
    path.emplace_back(circle.points.back());

    return path;
}

void test_drivenmassareamanager_1_()
{
    auto field = getTestField();

    arolib::Machine m;
    m.weight = 1000;
    m.width = machineWidth;
    m.length = 2 * m.width;

    auto path1 = getPathWithoutCrossing(field.outer_boundary.points.front());
    auto path2 = getPathWithCrossing(field.outer_boundary.points.front());

    auto cim = std::make_shared<arolib::gridmap::GridCellsInfoManager>();
    arolib::DrivenMassAreaManager dmam;
    dmam.setGridCellsInfoManager(cim);

    std::cout << std::endl << "Testing path without crossings..." << std::endl << std::endl;
    dmam.init(field, cellSize);
    for(auto& p : path1)
        dmam.addData(m, p, 0.0);
    auto map = dmam.getDrivenMassAreaMap();
//    map->saveGridAsGeoTiff( arolib::io::create_path(getOutputDir, "gridmap_DrivenMassArea_noCrossings.tif" );
    arolib::DrivenMassAreaManager::GridType::GridStatistics stats;
    BOOST_CHECK( map->getStatistics(stats) );
    BOOST_CHECK_SMALL(stats.min, 1e-6f);
    BOOST_CHECK_CLOSE(stats.max, m.weight, 0.01);
    for(size_t i = 0 ; i+1 < path1.size() ; ++i){
        auto p0 = path1.at(i);
        auto pn = path1.at(i+1);
        double d = arolib::geometry::calc_dist(p0, pn);
        auto p1 = arolib::geometry::getPointInLineAtDist(p0, pn, d/3);
        auto p2 = arolib::geometry::getPointInLineAtDist(p0, pn, d*2/3);

        p0 = arolib::geometry::getPointInLineAtDist(p0, pn, cellSize);
        pn = arolib::geometry::getPointInLineAtDist(pn, p0, cellSize);

        auto val = map->getLineComputedValue(p0, pn, cellSize, true);
        BOOST_CHECK_CLOSE( val, m.weight, 0.1);
        BOOST_CHECK_CLOSE( map->getValue(p1) , m.weight, 0.01);
        BOOST_CHECK_CLOSE( map->getValue(p2) , m.weight, 0.01);
    }


    std::cout << std::endl << "Testing path with crossings..." << std::endl << std::endl;
    dmam.init(field, cellSize);
    for(auto& p : path2)
        dmam.addData(m, p, 0.0);
    map = dmam.getDrivenMassAreaMap();
//    map->saveGridAsGeoTiff( arolib::io::create_path(getOutputDir, "gridmap_DrivenMassArea_withCrossings.tif" );
    BOOST_CHECK( map->getStatistics(stats) );
    BOOST_CHECK_SMALL(stats.min, 1e-6f);
    BOOST_CHECK_CLOSE(stats.max, 2*m.weight, 0.01);
    for(size_t i = 1 ; i+2 < path2.size() ; ++i){
        auto p0 = path2.at(i);
        auto pn = path2.at(i+1);
        double d = arolib::geometry::calc_dist(p0, pn);
        auto p1 = arolib::geometry::getPointInLineAtDist(p0, pn, d/3);
        auto p2 = arolib::geometry::getPointInLineAtDist(p0, pn, d*2/3);

        p0 = arolib::geometry::getPointInLineAtDist(p0, pn, cellSize);
        pn = arolib::geometry::getPointInLineAtDist(pn, p0, cellSize);

        auto val = map->getLineComputedValue(p0, pn, cellSize, true);
        BOOST_CHECK_CLOSE( val, m.weight, 0.1);
        BOOST_CHECK_CLOSE( map->getValue(p1) , m.weight, 0.01);
        BOOST_CHECK_CLOSE( map->getValue(p2) , m.weight, 0.01);
    }
    std::vector<arolib::Point> s0 = {path2.front(), path2.at(1)};
    std::vector<arolib::Point> sn = {path2.back(), r_at(path2, 1)};
    auto intersection = arolib::geometry::get_intersection(s0, sn).front();
    auto poly1 = arolib::geometry::create_circle(intersection, 1, 10);
    auto val = map->getPolygonComputedValue(poly1, arolib::DrivenMassAreaManager::GridType::AVERAGE_TOTAL, true);
    BOOST_CHECK_CLOSE(val, 2*m.weight, 0.01);

}

void test_drivenmassareamanager_2_()
{
    auto field = getTestField();

    arolib::Machine m;
    m.weight = 1000;
    m.width = machineWidth * 0.999;
    m.length = 2 * m.width;

    arolib::Point center = field.outer_boundary.points.front();
    center.x += 20.5 * cellSize;
    center.y += 20.5 * cellSize;

    auto cim = std::make_shared<arolib::gridmap::GridCellsInfoManager>();
    arolib::DrivenMassAreaManager dmam;
    dmam.setGridCellsInfoManager(cim);

    for(size_t i = 1 ; i <= 3 ; ++i){
        std::cout << std::endl << "Testing circular path with " << i << " cycles..." << std::endl << std::endl;
        auto path = getCloseCircularPath(center, i);
        dmam.init(field, cellSize);
        for(auto& p : path)
            dmam.addData(m, p, 0.0);
        auto map = dmam.getDrivenMassAreaMap();
//        map->saveGridAsGeoTiff( arolib::io::create_path(getOutputDir, "gridmap_DrivenMassArea_closeCircularPath_" + std::to_string(i) + ".tif" );
        arolib::DrivenMassAreaManager::GridType::GridStatistics stats;
        BOOST_CHECK( map->getStatistics(stats) );
        BOOST_CHECK_SMALL(stats.min, 1e-6f);
        auto val = map->getValue(center);
        BOOST_CHECK_CLOSE( val, m.weight, 0.01);
    }

}

BOOST_AUTO_TEST_CASE(test_drivenmassareamanager_1){ test_drivenmassareamanager_1_(); }
BOOST_AUTO_TEST_CASE(test_drivenmassareamanager_2){ test_drivenmassareamanager_2_(); }

BOOST_AUTO_TEST_SUITE_END()
