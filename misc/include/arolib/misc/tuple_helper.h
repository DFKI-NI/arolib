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
 
#ifndef AROLIBTUPLEHELPER_H
#define AROLIBTUPLEHELPER_H

#include <vector>
#include <tuple>
#include <type_traits>
#include <iostream>

namespace arolib {

/**
 * @brief Call a user defined function in each one of the tupple elements
 *
 * @param tuple Tuple
 * @param F Function
 */
template <size_t SIZE, typename F, typename R, typename... Ts  ,
          typename = typename std::enable_if< (SIZE > 0), void >::type >
void for_each_tuple_element(std::tuple<Ts...> & tuple, F funct){

    static_assert( ( SIZE <= sizeof...(Ts) ), "The SIZE must be <= the size of the tuple");
    funct( std::get< sizeof...(Ts) - SIZE >(tuple), sizeof...(Ts) - SIZE );
    for_each_tuple_element<SIZE-1, F, R, Ts...>(tuple, funct);
}

/**
 * @brief Call a user defined function in each one of the tupple elements
 *
 * @param tuple Tuple
 * @param [out] resp (optional) Responses for ecah tuple element after calling the given function
 * @param F Function
 */
template <size_t SIZE, typename F, typename R, typename... Ts  ,
          typename = typename std::enable_if< (SIZE > 0), void >::type >
void for_each_tuple_element(std::tuple<Ts...> & tuple, F funct, std::vector<R>* resp){

    static_assert( ( SIZE <= sizeof...(Ts) ), "The SIZE must be <= the size of the tuple");

    resp->emplace_back( funct( std::get< sizeof...(Ts) - SIZE >(tuple), sizeof...(Ts) - SIZE ) );
    for_each_tuple_element<SIZE-1, F, R, Ts...>(tuple, funct, resp);
}

/**
 * @brief Call a user defined function in each one of the tupple elements (specific case for empty tuples)
 */
template <size_t SIZE, typename... Ts ,
          typename = typename std::enable_if< (SIZE == 0), void >::type >
void for_each_tuple_element(...){
    return;
}

}

#endif // AROLIBTUPLEHELPER_H
