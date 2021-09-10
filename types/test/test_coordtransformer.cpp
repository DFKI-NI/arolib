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

#include <iostream>
#include <vector>
#include "arolib/types/point.hpp"
#include "arolib/types/coordtransformer.hpp"

using namespace arolib;

BOOST_AUTO_TEST_SUITE(test_coordtransformer)
BOOST_AUTO_TEST_CASE(test1)
{
    // trying out some points all over the world
    std::vector<Point> geodetic_points{
        Point(52, 8),          // Osnabrück 52 lat, 8 long
        Point(48.84, 2.42),    // Paris
        Point(47.39, 8.52),    // Zürich
        Point(42, 12.48),      // Rome
        Point(38.71, -9.12),   // Lisbon
        Point(40.34, -3.67),   // Madrid
        Point(50.47, 30.48),   // Kiev
        Point(8, -10),         // Senegal
        Point(-31, 28),        // South Africa
        // Point(-24, 150),       // Australia
        // Point(32, 100),        // China
        // Point(37.75, -122.48), // San Francisco
        // Point(42.33, -83),     // Detroit
        Point(-23, -43.13),    // Rio
    };

    for (const auto &geo_original : geodetic_points)
    {
        Point cart, geo_new;
        CoordTransformer::GetInstance().convert_to_cartesian(geo_original, cart);
        CoordTransformer::GetInstance().convert_to_geodetic(cart, geo_new);
        std::cout << "Geo original: " << geo_original << ", Cartesian: " << cart << ", Geo new: " << geo_new << std::endl;
        BOOST_CHECK_EQUAL(geo_original, geo_new);
    }
}
BOOST_AUTO_TEST_SUITE_END()
