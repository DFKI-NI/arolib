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
 
#include <boost/test/unit_test.hpp>
#include "arolib/geometry/geometry.hpp"
#include "arolib/types/polygon.hpp"
#include "arolib/types/point.hpp"

BOOST_AUTO_TEST_SUITE(test_polygon)
BOOST_AUTO_TEST_CASE(test_intersection_line_polygon)
{
    using namespace arolib;

    Polygon poly;
    Point p0(0.5, 2), p1(5.5, 2);
    poly.points = {Point(0, 0),
                   Point(0, 5),
                   Point(1, 5),
                   Point(1, 1),
                   Point(2, 1),
                   Point(2, 5),
                   Point(3, 5),
                   Point(3, 1),
                   Point(4, 1),
                   Point(4, 5),
                   Point(5, 5),
                   Point(5, 0),
                   Point(0, 0)};
    std::vector<Point> expected = {p0,
                                   Point(1, 2),
                                   Point(2, 2),
                                   Point(3, 2),
                                   Point(4, 2),
                                   Point(5, 2)};
    auto intersections = geometry::get_intersection(p0, p1, poly);
    BOOST_CHECK_EQUAL(intersections.size(), 6);
    for (auto &p : expected)
    {
        bool found = false;
        for (auto &pint : intersections)
        {
            if (arolib::geometry::calc_dist(p, pint) < 1e-9)
            {
                found = true;
                break;
            }
        }

        BOOST_TEST(found, "Intersection Point " + p.toString() + " not found!");
    }
}

BOOST_AUTO_TEST_CASE(test_intersection_polygon_polygon)
{
    auto poly1 = arolib::geometry::createRectangleFromLine(arolib::Point(0, 0), arolib::Point(0, 10), 20);
    auto poly2 = arolib::geometry::createRectangleFromLine(arolib::Point(10, 0), arolib::Point(10, 5), 10);

    auto intersections = arolib::geometry::get_intersection(poly1, poly2);
    BOOST_CHECK_EQUAL(intersections.size(), 1);

    arolib::Point minCorner, maxCorner;
    arolib::geometry::getPolygonLimits(intersections.front(), minCorner.x, maxCorner.x, minCorner.y, maxCorner.y);

    BOOST_CHECK(arolib::geometry::calc_dist(minCorner, arolib::Point(5, 0)) <= 1e-6);
    BOOST_CHECK(arolib::geometry::calc_dist(maxCorner, arolib::Point(10, 5)) <= 1e-6);
    // if (arolib::geometry::calc_dist(minCorner, Point(5, 0)) > 1e-6 || arolib::geometry::calc_dist(maxCorner, Point(10, 5)) > 1e-6)
    //     logger.printError("Unexpeted intersection polygon bounding box");

    double area = arolib::geometry::calc_area(intersections.front());
    BOOST_CHECK(std::fabs(area - 25) <= 1e-3);
    // if (std::fabs(area - 25) > 1e-3)
    //     logger.printError("Unexpeted intersection polygon area");
}
BOOST_AUTO_TEST_SUITE_END()
