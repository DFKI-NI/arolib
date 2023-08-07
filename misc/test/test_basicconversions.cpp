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
#include <iostream>
#include <limits>

#include "arolib/misc/basicconversions.hpp"
using namespace arolib;

BOOST_AUTO_TEST_SUITE(test_basicconversions)

// not floating point nums
template <typename T>
typename std::enable_if<!std::is_floating_point<T>::value, void>::type compare_nums(const T a, const T b)
{
    BOOST_CHECK_EQUAL(a, b);
}

// floating point nums
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, void>::type compare_nums(const T a, const T b)
{
    BOOST_CHECK_CLOSE(a, b, 1e-3);
}

template <typename T>
void check_num(const T a)
{
    T b;
    const std::string s = type2string(a);
    BOOST_CHECK(string2type(b, s));
    std::cout << "a=" << a << ", s=" << s << ", b=" << b << std::endl;
    // BOOST_CHECK_CLOSE(a, b, 1e-3);
    compare_nums(a, b);
}

template <typename T>
void check_all_limits()
{
    check_num<T>(std::numeric_limits<T>::min());
    check_num<T>(std::numeric_limits<T>::max());
    check_num<T>(std::numeric_limits<T>::max() / 2);

    // outcommented because raises compiler warning for some types
    check_num<T>(-1 * (std::numeric_limits<T>::min() + 1));
    check_num<T>((-1 * std::numeric_limits<T>::max() + 1));
    check_num<T>((-1 * std::numeric_limits<T>::max() + 1) / 2);
}

BOOST_AUTO_TEST_CASE(test_int)
{
    const int a = 42;
    int b;
    const std::string s = type2string(a);
    BOOST_CHECK_EQUAL(s, "42");
    BOOST_CHECK(string2type(b, s));
    BOOST_CHECK_EQUAL(a, b);
}

BOOST_AUTO_TEST_CASE(test_negative_int)
{
    const int a = -42;
    int b;
    const std::string s = type2string(a);
    BOOST_CHECK_EQUAL(s, "-42");
    BOOST_CHECK(string2type(b, s));
    BOOST_CHECK_EQUAL(a, b);
}

BOOST_AUTO_TEST_CASE(test_number_types_limits)
{
    check_all_limits<float>();
    check_all_limits<double>();
    check_all_limits<int>();
    check_all_limits<unsigned int>();
    check_all_limits<long>();
    check_all_limits<unsigned long>();
    check_all_limits<short>();
}

BOOST_AUTO_TEST_CASE(test_string)
{
    const std::string a = "Hello world, this is a string \n \r äöü##";
    std::string b;
    const std::string s = type2string(a);
    BOOST_CHECK(string2type(b, s));
    BOOST_CHECK_EQUAL(a, b);
    BOOST_CHECK_EQUAL(a, s);
}

enum TestEnum
{
    A,
    B,
    C,
    D,
    E
};

BOOST_AUTO_TEST_CASE(test_enum)
{
    const TestEnum a = TestEnum::C;
    TestEnum b;
    const std::string s = type2string(a);
    BOOST_CHECK(string2type(b, s));
    BOOST_CHECK_EQUAL(a, b);
    BOOST_CHECK_EQUAL(s, "2");
}

enum TestEnum2
{
    A2 = 33,
    B2 = 235,
    C2,
    D2 = -23,
    E2 = 999,
};

BOOST_AUTO_TEST_CASE(test_enum2)
{
    const TestEnum2 a = TestEnum2::D2;
    TestEnum2 b;
    const std::string s = type2string(a);
    BOOST_CHECK(string2type(b, s));
    BOOST_CHECK_EQUAL(a, b);
    BOOST_CHECK_EQUAL(s, "-23");
}
BOOST_AUTO_TEST_SUITE_END()
