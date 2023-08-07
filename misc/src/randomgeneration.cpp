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
 
#include "arolib/misc/randomgeneration.hpp"

namespace arolib {

namespace{
std::once_flag flag_do_once;
}

/**
 * @brief Generate a random int between a range
 *
 * @param min Range min
 * @param max Range max
 * @return Random int
 */
int gen_random_int(int min, int max)
{
    std::call_once( flag_do_once, [](){ srand( std::time(NULL) ); } );

    if(min == max)
        return min;

    if(min > max)
        std::swap(min, max);

    int delta = max - min + 1;

    return rand() % delta + min;
}

double gen_random_double(double min, double max)
{
    static std::default_random_engine re ( std::chrono::system_clock::now().time_since_epoch().count() );

    if(min == max)
        return min;

    if(min > max)
        std::swap(min, max);

    std::uniform_real_distribution<double> unif(min, max);

    return unif(re);
}

std::string gen_random_string(size_t len) {
    std::call_once( flag_do_once, [](){ srand( std::time(NULL) ); } );

    static const char alphanum[] =
            "0123456789"
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            "abcdefghijklmnopqrstuvwxyz";

    std::string ret;
    ret.resize(len);

    for (int i = 0; i < len; ++i)
        ret[i] = alphanum[rand() % (sizeof(alphanum) - 1)];

    return ret;
}




}//end namespace arolib

