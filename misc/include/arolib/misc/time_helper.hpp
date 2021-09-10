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
 
#ifndef ARO_TIMEHELPER_HPP
#define ARO_TIMEHELPER_HPP

#include <ctime>
#include <chrono>

namespace arolib {

double getDeltaTime(const timespec &from, const timespec &to);

double getDeltaTime(const clock_t &from, const clock_t &to);

bool getNow(timespec &timestamp);

bool getNow(clock_t &timestamp);


}


#endif // ARO_DATETIME_HPP
