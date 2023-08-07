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
#include "arolib/types/machine.hpp"

BOOST_AUTO_TEST_SUITE(test_machine)
BOOST_AUTO_TEST_CASE(test1)
{
    using namespace arolib;

    Machine machine;
    machine.max_speed_empty = 110;
    machine.max_speed_full = 10;
    machine.bunker_mass = 1000;

    auto speed = machine.calcSpeed(500);

    BOOST_TEST(speed == (100 * 0.5 + 10));
}
BOOST_AUTO_TEST_SUITE_END()
