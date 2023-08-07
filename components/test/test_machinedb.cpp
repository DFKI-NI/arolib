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
#include <string>
#include <set>
#include <iostream>

#include "arolib/types/machine.hpp"
#include "arolib/components/machinedb.h"

BOOST_AUTO_TEST_SUITE(test_machinedb)
BOOST_AUTO_TEST_CASE(test1)
{
    using namespace arolib;

    const std::string arolib_src(getenv("AROLIB_AROLIB_ROOT"));
    std::string machine_dir = arolib_src + "/testdata/machines";
    MachineDB machinedb(machine_dir);
    machinedb.setCSVHeaderTags({{MachineDB::EntryTag::TAG_MACHINE_TYPE, "type"}});
    machinedb.readFiles();
    machinedb.printList();

    // if something in the machine CSV changes or new files are added this unit test must be adapted

    std::vector<Machine> machines;
    auto resp = machinedb.getAllMachines(machines);
    BOOST_REQUIRE(!resp.errorID);
    BOOST_TEST(machines.size() == 24);

    std::vector<Machine> pseudo_machines;
    resp = machinedb.getMachines(std::set<std::string>{"PseudoMachines"}, std::set<std::string>{"Pseudo plough", "Pseudo sowing winter wheat"}, std::set<Machine::MachineType>(), pseudo_machines);
    BOOST_REQUIRE(!resp.errorID);
    BOOST_TEST(pseudo_machines.size() == 2);

    // Check if it parses all fields correctly
    auto machine = pseudo_machines.at(0);
    BOOST_TEST(machine.num_axis == 2);
    BOOST_TEST(machine.width == 2.);
    BOOST_TEST(machine.length == 7.2);
    BOOST_TEST(machine.height <= 0);
    BOOST_TEST(machine.weight == 19600);
    BOOST_TEST(machine.bunker_mass == 0);
    BOOST_TEST(machine.working_width == 2.);
    BOOST_TEST(machine.max_speed_empty == 2.5);
    BOOST_TEST(machine.max_speed_full == 2.5);
    BOOST_TEST(machine.def_working_speed == 2.5);
    BOOST_TEST(machine.turning_radius <= 0);

}
BOOST_AUTO_TEST_SUITE_END()
