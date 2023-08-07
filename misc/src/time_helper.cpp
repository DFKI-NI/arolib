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
 
#include "arolib/misc/time_helper.hpp"


namespace arolib{

double getDeltaTime(const timespec &from, const timespec &to)
{
    double t_from = (double)from.tv_sec + (double)from.tv_nsec / 1000000000.0;
    double t_to = (double)to.tv_sec + (double)to.tv_nsec / 1000000000.0;
    return t_to - t_from;
}

double getDeltaTime(const clock_t &from, const clock_t &to)
{
    return ( (double) (to-from) ) / CLOCKS_PER_SEC;
}

bool getNow(timespec &timestamp)
{
    return clock_gettime(CLOCK_MONOTONIC, &timestamp) == 0;
}

bool getNow(clock_t &timestamp)
{
    timestamp = std::clock();
    return true;
}



}
