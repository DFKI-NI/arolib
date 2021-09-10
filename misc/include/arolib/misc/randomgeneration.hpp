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
 
#ifndef AROLIB_RANDOMGENERATION_HPP
#define AROLIB_RANDOMGENERATION_HPP

#include <cstdlib>
#include <string>
#include <mutex>
#include <chrono>
#include <random>

namespace arolib {

/**
 * @brief Generate a random int between a range
 *
 * @param min Range min
 * @param max Range max
 * @return Random int
 */
int gen_random_int(int min, int max);

/**
 * @brief Generate a random double between a range
 *
 * @param min Range min
 * @param max Range max
 * @return Random double
 */
double gen_random_double(double min, double max);


/**
 * @brief Generate a random string with defined length
 *
 * @param len Length
 * @return Random string
 */
std::string gen_random_string(size_t len);

}//end namespace arolib


#endif //AROLIB_RANDOMGENERATION_HPP

