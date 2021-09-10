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
 
#ifndef AROLIBCONTAINERHELPER_H
#define AROLIBCONTAINERHELPER_H

#include <vector>
#include <map>
#include <set>

namespace arolib {

/**
 * @brief Reverse at: obtain an element of a container located at the given 'reverse' index (similar to the 'at' but starting from the end of the container)
 * @param container Container
 * @param r_ind 'Reverse' index. For instance, if r_ind=0, it will refer to the last element in the container
 * @return Element of the container located at the given 'reverse' index.
 */
template< typename C >
auto r_at(C& container, size_t r_ind) -> typename C::value_type& {
    if(r_ind >= container.size())
        throw std::invalid_argument( "Index out of range. Size of container is " + std::to_string(container.size())
                                     + " and requested (reverse) index is " + std::to_string(r_ind) );
    return *( container.end()-1-r_ind );
}

/**
 * @brief Reverse at: obtain an element of a container located at the given 'reverse' index (similar to the 'at' but starting from the end of the container)
 * @param container Container
 * @param r_ind 'Reverse' index. For instance, if r_ind=0, it will refer to the last element in the container
 * @return Element of the container located at the given 'reverse' index.
 */
template< typename C >
auto r_at(const C& container, size_t r_ind) -> const typename C::value_type& {
    if(r_ind >= container.size())
        throw std::out_of_range( "Index out of range. Size of container is " + std::to_string(container.size())
                                     + " and requested (reverse) index is " + std::to_string(r_ind) );
    return *( container.cend()-1-r_ind );
}

/**
 * @brief Insert an item at the beginning of the container (similar to push_back, but at the beginning and with obvious lower performance)
 * @param [in/out] v Vector where the item will be inserted.
 * @param item Item to be inserted.
 */
template <typename T>
inline void push_front(std::vector<T>& v, const T& item){
    v.insert( v.begin(), item );
}

/**
 * @brief Remove the first n items of the container
 * @param [in/out] v Vector from where the item will be removed.
 * @param numElements Number of items to be removed.
 */
template <typename T>
inline void pop_front(std::vector<T>& v, size_t numElements = 1){
    if(v.size() < numElements)
        throw std::out_of_range( "The container has " + std::to_string(v.size()) + ", less than " + std::to_string(numElements) );
    v.erase( v.begin(), v.begin()+numElements );
}

/**
 * @brief Remove the last n items of the container
 * @param [in/out] v Vector from where the item will be removed.
 * @param numElements Number of items to be removed.
 */
template <typename T>
inline void pop_back(std::vector<T>& v, size_t numElements = 1){
    if(v.size() < numElements)
        throw std::out_of_range( "The container has " + std::to_string(v.size()) + ", less than " + std::to_string(numElements) );
    v.erase( v.begin() + v.size() - numElements, v.end() );
}

}

#endif // AROLIBCONTAINERHELPER_H
