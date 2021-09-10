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
 
#ifndef AROLIB_IO_XMLOUTDOCUMENT_HPP
#define AROLIB_IO_XMLOUTDOCUMENT_HPP

#include <ostream>
#include <fstream>
#include <sstream>

#include "arodocument.hpp"
#include "arotagsmanager.hpp"
#include "io_common.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/misc/base64Utility.hpp"

namespace arolib {
namespace io {

/**
 * @brief General XML output document
 */
class XMLOutDocument : public AroOutDocument, public AroTagsManager{

public:

    /**
     * @brief Constructor.
     *
     * @param logLevel Log level
     */
    explicit XMLOutDocument(LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Destructor.
     */
    virtual ~XMLOutDocument();

    /**
     * @brief Open a tag in the current document
     *
     * @param tag Tag name
     * @param extra Extra information to add in the tag's XML header (disregarded if empty)
     * @return True on success
     */
    bool openTag(const std::string &tag, const std::string& extra = "");

    /**
     * @brief Close currently opened tag(s) in the current document
     *
     * @param numTags Number of tags to close
     * @return True on success
     */
    bool closeTag(int numTags = 1);


    /**
     * @brief Add/write a numeric value with a given tag
     *
     * @param value Value to be written
     * @param tag Tag
     * @return True on success
     */
    template< typename T,
              typename = typename std::enable_if< ( std::is_arithmetic<T>::value || std::is_enum<T>::value )
                                                    && !std::is_void<T>::value
                                                    && !std::is_same<T*, const char*>::value, void >::type >
    bool add(const T& value, std::string tag){
        if(!isReadyToWrite())
            return false;
        tag = getTag<T>(tag);
        tabs();
        *m_os << "<" << tag << ">" << value << "</" << tag << ">\n";
        return true;
    }

//    template< typename T >
//    bool add(const std::vector<T>& items, std::string tag){
//        return add(&items, tag);
//    }

//    template< typename T ,
//              typename = typename std::enable_if< ( !std::is_same<T*, const char*>::value ) >::type >
//    bool add(const std::vector<T>* items, const std::string& tag){
//        for(auto& item : *items){
//            if( !add(item, tag) )
//                return false;
//        }
//        return true;
//    }

//    template< typename T ,
//              typename = typename std::enable_if< ( !std::is_same<T*, const char*>::value ) >::type >
//    bool add(const std::pair<T*, std::string>& item){
//        return add(*item.first, item.second);
//    }

//    template< typename T >
//    bool add(const std::pair<T, std::string>& item){
//        return add(item.first, item.second);
//    }

//    template< typename T, typename... Types >
//    bool add(const std::pair<T, std::string> &item, const Types& ... items){
//        if(!add(item))
//            return false;
//        return add(items...);
//    }

    /**
     * @brief Add/write a char-array value with a given tag
     *
     * @param value Value to be written
     * @param tag Tag
     * @return True on success
     */
    bool add(const char* value, std::string tag);

    /**
     * @brief Add/write a string value with a given tag
     *
     * @param value Value to be written
     * @param tag Tag
     * @return True on success
     */
    bool add(const std::string& value, std::string tag);

    /**
     * @brief Add/write several string values with a given base tag and specific sub tags
     *
     * Will create a group with 'tag' (if not empty), then create sub-groups with tags values.first where each sub-group will have a < name , value > entry.
     * @param values Values to be written ( <sub-group tag , <value name, value> > )
     * @param tag Base (group) tag (disregrded if empty)
     * @return True on success
     */
    bool add(const std::map<std::string, std::map<std::string, std::string> > &values, // map<tag, map< name , value > >
                   std::string tag = "");


protected:

    /**
     * @brief Open document (AroDocument derived class implementation).
     *
     * @return True on success
     */
    virtual bool openDoc();

    /**
     * @brief Close document (AroDocument derived class implementation).
     *
     * @return True on success
     */
    virtual bool closeDoc();


    /**
     * @brief Add n "tabs" to the document
     */
    inline void tabs(int n);

    /**
     * @brief Add m_nTabs "tabs" to the document
     */
    void tabs();

    /**
     * @brief SFINAE test to know if a class T has a specific add(const P &, std::string)
     */
    template <typename P, typename T = XMLOutDocument>
    struct has_add_method
    {
    private:
        static const P _p;
        template<typename U> static auto test(char) -> decltype(std::declval<U>().add(_p, std::string()) == 1, std::true_type());
        template<typename> static std::false_type test(...);
    public:
        static constexpr bool value = std::is_same<decltype(test<T>(0)),std::true_type>::value;
    };

protected:
    int m_nTabs = 0; /**< Current count of "tabs" */
    std::vector<std::string> m_openTags; /**< Currently open tags */
};

}
}//end namespace arolib


#endif //AROLIB_IO_XMLOUTDOCUMENT_HPP

