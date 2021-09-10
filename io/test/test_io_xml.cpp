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
 
#include "arolib/io/io_xml.hpp"
#include <string>
#include <iostream>
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(io_xml_test)
BOOST_AUTO_TEST_CASE(field_loading_test)
{
    const std::string arolib_src = std::string(getenv("AROLIB_AROLIB_ROOT"));
    const std::string field_file_path = arolib_src + "/testdata/fields/field_example_1.xml";

    arolib::Field field;

    auto res = arolib::io::readFieldXML(field_file_path, field);
    BOOST_REQUIRE(res);

    BOOST_CHECK_EQUAL(field.subfields.size(), 1);

    BOOST_TEST(field.outer_boundary == field.subfields.at(0).boundary_outer);
}
BOOST_AUTO_TEST_SUITE_END()
