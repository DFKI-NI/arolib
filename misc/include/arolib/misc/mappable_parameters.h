/*
 * Copyright 2021-2025 DFKI GmbH
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
 
#ifndef MAPPABLE_PARAMETERS_HPP
#define MAPPABLE_PARAMETERS_HPP

#include <map>
#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <sstream>
#include <type_traits>
#include <boost/serialization/nvp.hpp>
#include "arolib/misc/basicconversions.hpp"

namespace arolib
{
    struct MappableParameters // : public virtual IMappableParameters
    {
    public:
        struct ParameterBase
        {
            std::string name;
            virtual std::string serealize() const = 0;
            virtual bool deserealize(std::string s) = 0;
            ParameterBase(std::string parameter_name);
        };

        typedef std::vector<std::shared_ptr<ParameterBase>> ParameterList;

        template <typename T>
        struct ParameterImpl : ParameterBase
        {
            T &pointer;
            ParameterImpl(std::string name, T &obj) : ParameterBase(name), pointer(obj){}
            std::string serealize() const
            {
                return type2string(pointer);
            }
            bool deserealize(std::string s)
            {
                bool success = string2type(pointer, s);
                return success;
            }
        };

        /**
         * @brief Set the parameters from a map of strings
         * @param strMap Map containing the parameters' values as string
         * @param strict If false, it will disregard absent parameters, leaving them unchanged (= fail if some parameters not in strMap)
         * @return True on success
         */
        bool parseFromStringMap(const std::map<std::string, std::string> &strMap, bool strict);

        /**
         * @brief Get the parameters as a map of strings
         * @return Map containing the parameters' values as string
         */
        std::map<std::string, std::string> parseToStringMap();

        virtual ParameterList get_parameters() = 0;

    protected:
        template <typename T>
        std::shared_ptr<ParameterImpl<T>> make_parameter(std::string name, T &param)
        {
            return std::make_shared<ParameterImpl<T>>(name, param);
        }
    };
}
#endif // MAPPABLE_PARAMETERS
