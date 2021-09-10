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
 
#include <boost/test/unit_test.hpp>

#include "arolib/types/point.hpp"
#include "arolib/types/route_point.hpp"
#include "arolib/types/route.hpp"
#include "arolib/types/coordtransformer.hpp"
#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <cmath>

using namespace arolib;

float round_to_two_digits(float x)
{
    int y = round(x*100);
    return (y / 100.);
}

bool floats_within(float a, float b, float tolerance)
{
    float diff = a - b;
    if (diff < 0)
        diff *= -1;

    return diff <= tolerance;
}

Route init_route()
{
    using namespace arolib;
    Route route;

    route.route_points = std::vector<RoutePoint>{
        RoutePoint(0, 0),   // time 0 (speed 10)
        RoutePoint(10, 0),  // time 1 (speed 10 / 3)
        RoutePoint(10, 10), // time 4 (speed 2)
        RoutePoint(0, 10),  // time 9 (speed 10)
        RoutePoint(0, 0),   // time 10 (speed 1)
        RoutePoint(30, 16)  // time 44
    };
    std::vector<double> timestamps{0, 1, 4, 9, 10, 44};
    BOOST_REQUIRE_EQUAL(route.route_points.size(), timestamps.size());

    for (int i = 0; i < timestamps.size(); i++)
    {
        route.route_points[i].time_stamp = timestamps[i];
    }
    return route;
}

BOOST_AUTO_TEST_SUITE(test_route)
BOOST_AUTO_TEST_CASE(calcPoint)
{
    Route route = init_route();
    std::map<double, Point> correct_points{
        {0.0, Point(0, 0)},
        {0.5, Point(5, 0)},
        {0.7, Point(7, 0)},
        {4.0, Point(10, 10)},
        {4.5, Point(9, 10)},
        {6.5, Point(5, 10)},
        {7.0, Point(4, 10)},
        {9.0, Point(0, 10)},
        {10.0, Point(0, 0)},
        {27.0, Point(15, 8)},
        {44.0, Point(30, 16)}};

    std::cout << route << std::endl;

    int counting_refindex = 0;
    for (const auto &correct_point : correct_points)
    {
        std::cout << "Time: " << correct_point.first << "; Correct Point = " << correct_point.second << std::endl;
        BOOST_CHECK_EQUAL(route.calcPoint(correct_point.first).point(), correct_point.second);

        int reset_refindex = 0;
        BOOST_CHECK_EQUAL(route.calcPoint2(correct_point.first, reset_refindex).point(), correct_point.second);
        BOOST_CHECK_EQUAL(route.calcPoint2(correct_point.first, counting_refindex).point(), correct_point.second);
        std::cout << "---" << std::endl;
    }

}

BOOST_AUTO_TEST_CASE(calcSpeed)
{
    Route route = init_route();
    std::map<double, double> correct_speeds{
        {0.1, 10.0},
        {0.777, 10.0},
        {1.1, round_to_two_digits(10./3.)},
        {8.99, 2},
        {9.1, 10},
        {10, 10},
        {10.1, 1},
        {16, 1},
        {43, 1}};

    std::cout << route << std::endl;

    int counting_refindex = 0;
    for (const auto &correct : correct_speeds)
    {
        std::cout << "Time: " << correct.first << "; Correct Speed = " << correct.second << std::endl;
        BOOST_CHECK_EQUAL(round_to_two_digits(route.calcSpeed(correct.first)), correct.second);

        int reset_refindex = 0;
        BOOST_CHECK_EQUAL(round_to_two_digits(route.calcSpeed(correct.first, reset_refindex)), correct.second);
        BOOST_CHECK_EQUAL(round_to_two_digits(route.calcSpeed(correct.first, counting_refindex)), correct.second);
        std::cout << "---" << std::endl;
    }

}

BOOST_AUTO_TEST_CASE(calcBearing)
{
    Route route;
    
    std::map<float, Point> points {
        {0, Point(8, 52)},
        {1, Point(8, 53)},
        {2, Point(9, 53)},
        {5, Point(8, 52)}
    };

    int i = 0;
    for(auto& pair: points) {
        std::cout << "Converting " << pair.second << " to cartesian" << std::endl;
        RoutePoint out;
        BOOST_REQUIRE(CoordTransformer::GetInstance().convert_to_cartesian(pair.second, out));
        out.time_stamp = pair.first;
        route.route_points.push_back(out);
    }

    std::cout << route << std::endl;

    std::map<float, float> correct_bearings{
        {0.5, 0},
        {1.5, 90},
        {2.5, 210}
    };

    int counting_refindex = 0;
    for (const auto& p: correct_bearings)
    {
        BOOST_TEST(floats_within(route.calcBearing(p.first), p.second, 2));

        int reset_refindex = 0;
        BOOST_TEST(floats_within(route.calcBearing(p.first, counting_refindex), p.second, 2));
        BOOST_TEST(floats_within(route.calcBearing(p.first, reset_refindex), p.second, 2));
    }

}
BOOST_AUTO_TEST_SUITE_END()
