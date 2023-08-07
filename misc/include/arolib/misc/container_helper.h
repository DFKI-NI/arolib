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
 
#ifndef AROLIBCONTAINERHELPER_H
#define AROLIBCONTAINERHELPER_H

#include <vector>
#include <map>
#include <set>

namespace arolib {

/**
 * @brief Returns the real index corresponding to a given pseudo-index for a ciclyc container, where container[i] := container[i+size]
 * @param v ciclyc container
 * @param ind Pseudo index. Can be <0 or >v.size
 * @return Real index
 */
template< typename C >
size_t get_index_from_cyclic_container(const C& container, int ind){
   if(container.empty())
       throw std::invalid_argument( std::string(__FUNCTION__) + ": the container is empty" );

   if(ind >= 0)
       ind = ind % container.size();
   else{
       ind = -1 * (ind + 1);
       ind = ind % container.size();
       ind = container.size() - 1 - ind;
   }
   return ind;
}

/**
 * @brief Reverse at: obtain an element of a (random access) container located at the given 'reverse' index (similar to the 'at' but starting from the end of the container)
 *
 * The container must use random access iterators, where the elements can be accessed as *(container.end()-1-r_ind)
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
 *
 * The container must use random access iterators, where the elements can be accessed as *(container.end()-1-r_ind)
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
 * @brief Obtain an element of a (random access) cyclic container located at the given 'pseudo' index, where v[i] := v[i+size] (similar to the 'at_cyclic' but starting from the end of the container)
 *
 * The container must use random access iterators, where the elements can be accessed as *(container.beging()+ind)
 * @param container Container
 * @param ind 'Pseudo' index. Can be <0 or >container.size. For instance, if ind=container.size, it will refer to the first element in the container (real index = 0)
 * @param [out] indReal (optional) Real index
 * @return Element of the container located at the given 'pseudo' index.
 */
template< typename C >
auto at_cyclic(C& container, int ind, size_t * realInd = nullptr) -> typename C::value_type& {
    ind = get_index_from_cyclic_container(container, ind);
    if(realInd)
        *realInd = ind;
    return *( container.begin()+ind );
}

/**
 * @brief Obtain an element of a (random access) cyclic container located at the given 'pseudo' index, where v[i] := v[i+size] (similar to the 'at_cyclic' but starting from the end of the container)
 *
 * The container must use random access iterators, where the elements can be accessed as *(container.beging()+ind)
 * @param container Container
 * @param ind 'Pseudo' index. Can be <0 or >container.size. For instance, if ind=container.size, it will refer to the first element in the container (real index = 0)
 * @param [out] indReal (optional) Real index
 * @return Element of the container located at the given 'pseudo' index.
 */
template< typename C >
auto at_cyclic(const C& container, int ind, size_t * realInd = nullptr) -> const typename C::value_type& {
    ind = get_index_from_cyclic_container(container, ind);
    if(realInd)
        *realInd = ind;
    return *( container.cbegin()+ind );
}

/**
 * @brief Obtain an element of a (random access) cyclic container located at the given 'reverse pseudo' index, where v[i] := v[i+size] (similar to the 'at_cyclic' but starting from the end of the container)
 *
 * The container must use random access iterators, where the elements can be accessed as *(container.end()+1-ind)
 * @param container Container
 * @param r_ind 'Reverse-pseudo' index. Can be <0 or >container.size. For instance, if ind=container.size, it will refer to the last element in the container (real index = container.size - 1)
 * @param [out] indReal (optional) Real index (not reversed)
 * @return Element of the container located at the given 'reverse-pseudo' index.
 */
template< typename C >
auto r_at_cyclic(C& container, int r_ind, size_t * realInd = nullptr) -> typename C::value_type& {
    r_ind = get_index_from_cyclic_container(container, r_ind);
    if(realInd)
        *realInd = container.size() - 1 - r_ind;
    return r_at(container, r_ind);
}

/**
 * @brief Obtain an element of a (random access) cyclic container located at the given 'reverse pseudo' index, where v[i] := v[i+size] (similar to the 'at_cyclic' but starting from the end of the container)
 *
 * The container must use random access iterators, where the elements can be accessed as *(container.end()+1-ind)
 * @param container Container
 * @param r_ind 'Reverse-pseudo' index. Can be <0 or >container.size. For instance, if ind=container.size, it will refer to the last element in the container (real index = container.size - 1)
 * @param [out] indReal (optional) Real index (not reversed)
 * @return Element of the container located at the given 'reverse-pseudo' index.
 */
template< typename C >
auto r_at_cyclic(const C& container, int r_ind, size_t * realInd = nullptr) -> const typename C::value_type& {
    r_ind = get_index_from_cyclic_container(container, r_ind);
    if(realInd)
        *realInd = container.size() - 1 - r_ind;
    return r_at(container, r_ind);
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
    if(numElements == 0)
        return;
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
    if(numElements == 0)
        return;
    if(v.size() < numElements)
        throw std::out_of_range( "The container has " + std::to_string(v.size()) + ", less than " + std::to_string(numElements) );
    v.erase( v.begin() + v.size() - numElements, v.end() );
}

}

#endif // AROLIBCONTAINERHELPER_H
