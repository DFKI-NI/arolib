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
 
#ifndef AROLIB_BASICCONVERSIONS_HPP
#define AROLIB_BASICCONVERSIONS_HPP

#include <string>
#include <map>
#include <algorithm>
#include <type_traits>
#include <cmath>
#include <sstream>

namespace arolib
{

    /**
     * @brief Converts a string to double
     *
     * Admits both '.' and ',' as decimal separators.
     */
    double string2double(std::string s);

    /**
     * @brief Converts a double to a string using '.' as decimal separator
     */
    std::string double2string(double n);

    /**
     * @brief Check if a value is "not a number"
     * @param value value
     */
    template <typename T,
              typename = typename std::enable_if<std::is_floating_point<T>::value, void>::type>
    bool is_nan(T value)
    {
        return std::isnan(value);
    }


    /**
     * @brief Check if a value is "not a number"
     * @param value value
     */
    template <typename T,
              typename = typename std::enable_if<!std::is_floating_point<T>::value, void>::type>
    bool is_nan(const T &)
    {
        return false;
    }

    /**
     * @brief Interface for (string) mappable parameters
     */
    struct IMappableParameters
    {
        /**
         * @brief Set the parameters from a map of strings
         * @param strMap Map containing the parameters' values as string
         * @param strict If false, it will disregard absent parameters, leaving them unchanged
         * @return True on success
         */
        virtual bool parseFromStringMap(const std::map<std::string, std::string> &strMap, bool strict) = 0;

        /**
         * @brief Get the parameters as a map of strings
         * @return Map containing the parameters' values as string
         */
        virtual std::map<std::string, std::string> parseToStringMap() const = 0;
    };


    /**
     * @brief Parses the values from a string map into a values map with matching keys
     * @param strMap Input map holding the values as strings
     * @param [in/out] valMap Parsed values map
     * @param If set to true, all values corresponding to the keys in valMap must be available in strMap to succeed.
     */
    template <typename T,
              typename = typename std::enable_if<std::is_arithmetic<T>::value>::type>
    bool setValuesFromStringMap(const std::map<std::string, std::string> &strMap, const std::map<std::string, T *> &valMap, bool strict)
    {
        for (auto &it : valMap)
        {
            auto it2 = strMap.find(it.first);
            if (it2 == strMap.end())
            {
                if (strict)
                    return false;
                continue;
            }
            if (std::is_floating_point<T>::value)
                *it.second = string2double(it2->second);
            else
                *it.second = std::stoi(it2->second);
        }
        return true;
    }

    /**
     * @brief Overload if type is string
     *
     * Because other serialize function has trouble with strings
     */
    static inline std::string type2string(const std::string obj) { return obj; }

    /**
     * @brief Can serialize Enums and all types with operator<< definition
     */
    template <typename T>
    static std::string type2string(const T &obj)
    {
        std::stringstream ss;
        ss << obj;
        return ss.str();
    }

    /**
     * @brief Overload if type is string
     */
    static bool string2type(std::string &obj, const std::string s)
    {
        obj = s;
        return true;
    }

    /**
     * @brief Tries to convert given string to type T
     *      this function should be used for everything except enums and strings 
     * 
     * @tparam T 
     * @param [out] obj Object
     * @param s the string
     * @return bool success 
     */
    template <typename T>
    static typename std::enable_if<!std::is_enum<T>::value, bool>::type string2type(T &obj, const std::string s)
    {
        T x;
        std::stringstream ss(s);
        ss >> x;
        obj = x;
        return true;
    }

    /**
     * @brief Tries to convert given string to type T
     *      this function should be used for enums 
     * 
     * @tparam T 
     * @param [out] obj Object
     * @param s the string
     * @return bool success 
     */
    template <typename T>
    static typename std::enable_if<std::is_enum<T>::value, bool>::type string2type(T &obj, const std::string s)
    {
        int i;
        std::stringstream ss(s);
        ss >> i;
        obj = static_cast<T>(i);
        return true;
    }

} //end namespace arolib

#endif //AROLIB_BASICCONVERSIONS_HPP
