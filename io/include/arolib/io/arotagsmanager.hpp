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
 
#ifndef AROLIB_IO_XMLDOCUMENT_HPP
#define AROLIB_IO_XMLDOCUMENT_HPP

#include <ostream>
#include <fstream>
#include <sstream>
#include <iterator>
#include <stdlib.h>
#include <typeinfo>
#include <typeindex>
#include <type_traits>

#include "arolib/types/coordtransformer.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/types/subfield.hpp"
#include "arolib/types/field.hpp"
#include "arolib/types/fieldaccesspoint.hpp"
#include "arolib/types/resourcepoint.hpp"
#include "arolib/types/outfieldinfo.hpp"
#include "arolib/types/machinedynamicinfo.hpp"
#include "arolib/types/route.hpp"
#include "arolib/cartography/common.hpp"
#include "arolib/planning/path_search/directedgraph.hpp"

namespace arolib {
namespace io {

/**
 * @brief Class to manage tags in Arolib documents
 */
class AroTagsManager{
protected:
    /**
     * @brief Default constructor.
     */
    explicit AroTagsManager() = default;

public:
    static const std::string UseDefaultTag; /**< String value to state that default tags must be used */

    /**
     * @brief Get the default tag for a type T
     * @return Default tag for a type T
     */
    template <typename T>
    static std::string getTag(){
        auto it = m_tags.find( std::type_index(typeid(T)) );
        if(it != m_tags.end())
            return it->second;
        return "";
    }

    /**
     * @brief Get the default tag for a type T
     * @param t Object of type T (used only to "guess" the type)
     * @return Default tag for a type T
     */
    template <typename T>
    static std::string getTag(const T&){
        return getTag<T>();
    }

protected:
    /**
     * @brief Get the tag based on the type (T) and input tag
     *
     * If tag == UseDefaultTag, the default tag for type T will be returned; otherwise, 'tag' will be returned.
     * @param tag Input tag
     * @return Tag to be used
     */
    template <typename T>
    static std::string getTag(const std::string& tag){
        if(tag != UseDefaultTag)
            return tag;
        auto it = m_tags.find( std::type_index(typeid(T)) );
        if(it != m_tags.end())
            return it->second;
        return "";
    }
    static const std::map<std::type_index, std::string> m_tags; /**< Default tags for different types */
};


}
}//end namespace arolib


#endif //AROLIB_IO_XMLDOCUMENT_HPP

