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
 
#include "arolib/misc/mappable_parameters.h"
#include <boost/test/unit_test.hpp>
#include <boost/test/test_tools.hpp>
#include <string>
#include <iostream>

using namespace arolib;

enum TestEnum
{
    A,
    B,
    C
};

class TestParams : public MappableParameters
{
public:
    double a_double = 42.0;
    float a_float = 1337.1337;
    std::string a_string = "hello world nice";
    bool a_bool = false;
    bool a_bool2 = true;
    TestEnum a_enum = TestEnum::C;

    inline ParameterList get_parameters()
    {
        return ParameterList{make_parameter("a_double", a_double),
                             make_parameter("a_float", a_float),
                             make_parameter("a_string", a_string),
                             make_parameter("a_bool", a_bool),
                             make_parameter("a_bool2", a_bool2),
                             make_parameter("a_enum", a_enum)
                            };
    };

    void print()
    {
        std::cout << "double: " << a_double;
        std::cout << " float: " << a_float;
        std::cout << " string: " << a_string;
        std::cout << " bool: " << a_bool;
        std::cout << " enum: " << a_enum;
        std::cout << std::endl;
    }
};

BOOST_AUTO_TEST_SUITE(test_mappable_parameters)

void compare_objects(TestParams &A, TestParams &B)
{
    BOOST_CHECK_CLOSE(A.a_double, B.a_double, 0.001);
    BOOST_CHECK_CLOSE(A.a_float, B.a_float, 0.001);
    BOOST_CHECK_EQUAL(A.a_string, B.a_string);
    BOOST_CHECK_EQUAL(A.a_bool, B.a_bool);
    BOOST_CHECK_EQUAL(A.a_bool2, B.a_bool2);
    BOOST_CHECK_EQUAL(A.a_enum, B.a_enum);
}

BOOST_AUTO_TEST_CASE(mappable_parameters1)
{
    double my_double = 123.123;
    float my_float = -24.2;
    std::string my_string = "halldf kasdfj lkakj";
    bool my_bool = true;
    bool my_bool2 = false;
    TestEnum my_enum = TestEnum::B;
    TestParams original, parsed;

    BOOST_TEST_CHECKPOINT("All variables initialized");
    auto map = original.parseToStringMap();
    BOOST_TEST_CHECKPOINT("After the first call to parseToStringMap()");
    BOOST_CHECK(parsed.parseFromStringMap(map, false));
    compare_objects(original, parsed);
    original.print();
    parsed.print();

    original.a_double = my_double;
    original.a_float = my_float;
    original.a_string = my_string;
    original.a_bool = my_bool;
    original.a_bool2 = my_bool2;
    original.a_enum = my_enum;

    map.clear();
    map = original.parseToStringMap();
    BOOST_CHECK(parsed.parseFromStringMap(map, false));
    compare_objects(original, parsed);
    original.print();
    parsed.print();

    // check what happens if exact
    TestParams parsed2, parsed3, neu1;
    map.erase("a_float");
    map["something"] = "test";
    BOOST_CHECK(parsed2.parseFromStringMap(map, false));
    BOOST_CHECK(!parsed3.parseFromStringMap(map, true));

    parsed2.a_float = my_float;
    compare_objects(parsed2, original);
    // compare_objects(neu1, original);
}
BOOST_AUTO_TEST_SUITE_END()
