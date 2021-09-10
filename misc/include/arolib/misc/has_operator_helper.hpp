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
 
#ifndef AROLIB_HASOPERATORHELPER_HPP
#define AROLIB_HASOPERATORHELPER_HPP

#include <type_traits>

namespace arolib {


/**
 * @brief Struct used to know if there exists the "T1 == T2" operator for classes T1 and T2
 */
template <typename T1, typename T2 = T1, typename Tresult = bool>
struct has_is_equal_method
{
private:
    template<typename U, typename V> static auto test(char) -> decltype(std::declval<U>() == std::declval<V>());
    template<typename, typename> static std::false_type test(...);
public:
    static constexpr bool value = std::is_same<decltype(test<T1, T2>(0)), Tresult>::value;
};


/**
 * @brief Struct used to know if there exists the "T1 + T2" operator for classes T1 and T2
 */
template <typename T1, typename T2 = T1, typename Tresult = T1>
struct has_add_method
{
private:
    template<typename U, typename V> static auto test(char) -> decltype(std::declval<U>() + std::declval<V>());
    template<typename, typename> static std::false_type test(...);
public:
    static constexpr bool value = std::is_same<decltype(test<T1, T2>(0)), Tresult>::value
                                    || std::is_same<decltype(test<T1, T2>(0)), const Tresult>::value
                                    || std::is_same<decltype(test<T1, T2>(0)), Tresult&>::value
                                    || std::is_same<decltype(test<T1, T2>(0)), const Tresult&>::value;
};


}//end namespace arolib


#endif //AROLIB_HASOPERATORHELPER_HPP

