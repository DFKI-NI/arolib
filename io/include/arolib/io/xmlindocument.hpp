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
 
#ifndef AROLIB_IO_XMLINDOCUMENT_HPP
#define AROLIB_IO_XMLINDOCUMENT_HPP

#include <ostream>
#include <fstream>
#include <sstream>
#include <functional>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/optional/optional.hpp>
#include <boost/algorithm/string.hpp>

#include "arodocument.hpp"
#include "arotagsmanager.hpp"
#include "io_common.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/misc/base64Utility.hpp"
#include "arolib/misc/tuple_helper.h"

namespace arolib {
namespace io {

/**
 * @brief General XML input document
 */
class XMLInDocument : public AroInDocument, public AroTagsManager{

public:

    /**
     * @brief Handler to manage branches and read values from them
     */
    struct ReadHandler
    {
        friend class XMLInDocument;

        /**
         * @brief Default constructor.
         */
        explicit ReadHandler() {}
    protected:

        /**
         * @brief Constructor (initialize from ptree).
         * @param _t ptree
         */
        explicit ReadHandler(const boost::property_tree::ptree& _t) : t(_t) {}
        boost::property_tree::ptree t; /**< ptree */
    };

    /**
     * @brief Constructor.
     *
     * @param logLevel Log level
     */
    explicit XMLInDocument(LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Destructor.
     */
    virtual ~XMLInDocument();

    /**
     * @brief Open document.
     *
     * @param rh ReadHandler (optional)
     * @return True on success
     */
    virtual bool openDocument(ReadHandler* rh = nullptr);


    /**
     * @brief Get the base ReadHandler from the document.
     *
     * @param [out] rh ReadHandler
     * @return True on success
     */
    bool getBaseHandler(ReadHandler& rh);

    /**
     * @brief Check if the branch coresponding to the ReadHandler has a branch with a given name.
     *
     * @param base Base ReadHandler
     * @param name Branch name
     * @return True if branch exists
     */
    static bool hasBranch(const ReadHandler& base, const std::string& name);

    /**
     * @brief Check if the base/root branch of the document has a branch with a given name.
     *
     * @param name Branch name
     * @return True if branch exists
     */
    bool hasBranch(const std::string& name);

    /**
     * @brief Get a (sub) branch ReadHandler with a given name from the branch coresponding to a ReadHandler.
     *
     * @param base Base ReadHandler
     * @param name Branch name
     * @param [out] branch ReadHandler for the output (sub) branch
     * @param _logger Logger
     * @return True on success
     */
    static bool getBranchHandler(const ReadHandler& base, const std::string& name, ReadHandler & branch, std::shared_ptr<Logger> _logger = nullptr);

    /**
     * @brief Get a (sub) branch ReadHandler with a given sequence of tags from the branch coresponding to a ReadHandler.
     *
     * @param base Base ReadHandler
     * @param tags Branch sequence of tags (relative to base)
     * @param [out] branch ReadHandler for the output (sub) branch
     * @param _logger Logger
     * @return True on success
     */
    static bool getBranchHandler(const ReadHandler& base, const std::vector< std::string >& tags, ReadHandler & branch, std::shared_ptr<Logger> _logger = nullptr);

    /**
     * @brief Get a (sub) branch ReadHandler with a given name from the base/root branch of the document.
     *
     * @param name Branch name
     * @param [out] branch ReadHandler for the output (sub) branch
     * @param _logger Logger
     * @return True on success
     */
    bool getBranchHandler(const std::string& name, ReadHandler & branch);

    /**
     * @brief Get a (sub) branch ReadHandler with a given sequence of tags from the base/root branch of the document.
     *
     * @param base Base ReadHandler
     * @param tags Branch sequence of tags (relative to base)
     * @param [out] branch ReadHandler for the output (sub) branch
     * @param _logger Logger
     * @return True on success
     */
    bool getBranchHandler(const std::vector< std::string >& tags, ReadHandler & branch);

    /**
     * @brief Get multiple (sub) branch ReadHandlers with a given name from the branch coresponding to a ReadHandler.
     *
     * @param base Base ReadHandler
     * @param tag Tag of the branches
     * @param funct Function called on the obtained (sub) branch ReadHandlers
     * @param parentTags Sequence of parent tags to reach the location where the (sub) branches will be read (relative to base)
     * @param _logger Logger
     * @return True on success
     */
    static bool getMultiBranchHandlers(const ReadHandler& base,
                                       const std::string& tag,
                                       const std::function<bool(const ReadHandler&)> funct,
                                       const std::vector<std::string>& parentTags = {},
                                       std::shared_ptr<Logger> _logger = nullptr);


    /**
     * @brief Get multiple (sub) branch ReadHandlers with a given name from the base/root branch of the document.
     *
     * @param tag Tag of the branches
     * @param funct Function called on the obtained (sub) branch ReadHandlers
     * @param parentTags Sequence of parent tags to reach the location where the (sub) branches will be read (relative to the base/root branch of the document)
     * @param _logger Logger
     * @return True on success
     */
    bool getMultiBranchHandlers(const std::string& tag,
                                const std::function<bool(const ReadHandler&)> funct,
                                const std::vector<std::string>& parentTags = {});

    /**
     * @brief Read (directly) a string value from a ReadHandler.
     *
     * @param base Base ReadHandler
     * @param [out] value Read value
     * @return True on success
     */
    virtual bool read( const ReadHandler & base, std::string& value);

    /**
     * @brief Read (directly) a string-map from a ReadHandler.
     *
     * @param base Base ReadHandler
     * @param [out] values Read string-map
     * @return True on success
     */
    virtual bool read( const ReadHandler & base,
                       std::map<std::string, std::map<std::string, std::string> > &values );

    /**
     * @brief Read (directly) a numeric value from a ReadHandler.
     *
     * @param base Base ReadHandler
     * @param [out] value Read value
     * @return True on success
     */
    template< typename T,
              typename = typename std::enable_if< std::is_arithmetic<T>::value, void >::type >
    bool read( const ReadHandler & base, T& value ){
        auto ovalue = RHTree(base).get_value_optional<T>();
        if(!ovalue)
            return false;
        value = *ovalue;
        return true;
    }

protected:

    /**
     * @brief Open document (AroDocument derived class implementation).
     *
     * @return True on success
     */
    virtual bool openDoc() override;

    /**
     * @brief Close document (AroDocument derived class implementation).
     *
     * @return True on success
     */
    virtual bool closeDoc() override;

    /**
     * @brief Check if a ptree::value_type has a value (parseable-to-T) with the given name.
     *
     * @param v ptree::value_type
     * @param name Name of the value to be read
     * @return True if (parseable) value exists
     */
    template <typename T>
    static bool hasValue(const boost::property_tree::ptree::value_type & v, const std::string& name){
        return (v.second.get_optional<T>(name));
    }


    /**
     * @brief Check if a ptree has a value (parseable-to-T) with the given name.
     *
     * @param v ptree
     * @param name Name of the value to be read
     * @return True if (parseable) value exists
     */
    template <typename T>
    static bool hasValue(const boost::property_tree::ptree & tree, const std::string& name){
        return (tree.get_optional<T>(name));
    }

    /**
     * @brief Get a value with the given name from a ptree.
     *
     * @param v ptree
     * @param name Name of the value to be read
     * @param [out] value Read value
     * @return True on success
     */
    template <typename T>
    static bool getValue(const boost::property_tree::ptree& tree, const std::string& name, T& value){
        try{
            auto ovalue = tree.get_optional<T>(name);
            if(!ovalue)
                return false;
            value = *ovalue;
            return true;
        }
        catch(...){
            return false;
        }
    }


    /**
     * @brief Get (directly) a value from a ptree.
     *
     * @param v ptree
     * @param [out] value Read value
     * @return True on success
     */
    template< typename T >
    static bool getValue( const boost::property_tree::ptree& tree, T& value ){
        auto ovalue = tree.get_value_optional<T>();
        if(!ovalue)
            return false;
        value = *ovalue;
        return true;
    }



    /**
     * @brief Get a (sub) branch (ptree) with a given name from a ptree.
     *
     * @param tree Base ptree
     * @param name Name of the (sub) branch
     * @param [out] branch (sub) Branch
     * @return True on success
     */
    static bool getBranch(const boost::property_tree::ptree& tree, const std::string& name, boost::property_tree::ptree& branch);

    /**
     * @brief Get a (sub) branch (ptree) with a given sequence of tags from a ptree.
     *
     * @param tree Base ptree
     * @param tags Branch sequence of tags (relative to tree)
     * @param [out] branch (sub) Branch
     * @param logLevel Log level
     * @return True on success
     */
    static bool getBranch(const boost::property_tree::ptree& tree, const std::vector< std::string >& tags, boost::property_tree::ptree& branch, LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Get a (sub) branch (ptree) with a given name from the base/root branch of the document.
     *
     * @param name Name of the (sub) branch
     * @param [out] branch (sub) Branch
     * @return True on success
     */
    bool getBranch(const std::string& name, boost::property_tree::ptree& branch);

    /**
     * @brief Get a (sub) branch (ptree) with a given sequence of tags from the base/root branch of the document.
     *
     * @param tags Branch sequence of tags (relative to the base/root branch of the document)
     * @param [out] branch (sub) Branch
     * @param logLevel Log level
     * @return True on success
     */
    bool getBranch(const std::vector< std::string >& tags, boost::property_tree::ptree& branch);

    /**
     * @brief Get a (sub) branch (ReadHandler) with a given name from a ReadHandler.
     *
     * @param base Base ReadHandler
     * @param name Name of the (sub) branch
     * @param [out] branch (sub) Branch
     * @return True on success
     */
    static bool getBranch(const ReadHandler & base, const std::string &name, ReadHandler &branch);

    /**
     * @brief Get a (sub) branch (ReadHandler) with a given sequence of tags from a ReadHandler.
     *
     * @param base Base ReadHandler
     * @param tags Branch sequence of tags (relative to base)
     * @param [out] branch (sub) Branch
     * @return True on success
     */
    static bool getBranch(const ReadHandler & base, const std::vector<std::string> &tags, ReadHandler &branch);

    /**
     * @brief Get a (sub) branch (ReadHandler) with a given name from the base/root branch of the document.
     *
     * @param name Name of the (sub) branch
     * @param [out] branch (sub) Branch
     * @return True on success
     */
    bool getBranch(const std::string &name, ReadHandler &branch);

    /**
     * @brief Get a (sub) branch (ReadHandler) with a given sequence of tags from the base/root branch of the document.
     *
     * @param tags Branch sequence of tags (relative to the base/root branch of the document)
     * @param [out] branch (sub) Branch
     * @return True on success
     */
    bool getBranch(const std::vector<std::string> &tags, ReadHandler &branch);

    /**
     * @brief Create a ReadHandler from a ptree .
     *
     * @param tree ptree
     * @return Created ReadHandler
     */
    static ReadHandler createRH(const boost::property_tree::ptree& tree);

    /**
     * @brief Get the ptree from a ReadHandler.
     *
     * @param rh ReadHandler
     * @return ReadHandler's ptree
     */
    static const boost::property_tree::ptree& RHTree(const ReadHandler &rh);

    /**
     * @brief Get the ptree from a ReadHandler.
     *
     * @param rh ReadHandler
     * @return ReadHandler's ptree
     */
    static boost::property_tree::ptree& RHTree(ReadHandler &rh);


    /**
     * @brief SFINAE test to know if a class T has a specific read(const ReadHandler &, P&)
     */
    template <typename P, typename T = XMLInDocument>
    struct has_read_method
    {
    private:
        static P _p;
        template<typename U> static auto test(char) -> decltype(std::declval<U>().read(ReadHandler(), _p) == 1, std::true_type());
        template<typename> static std::false_type test(...);
    public:
        static constexpr bool value = std::is_same<decltype(test<T>(0)),std::true_type>::value;
    };

protected:
    ReadHandler m_base; /**< Root ReadHandler for the document */
    bool m_docOpen = false; /**< Is the ducument open? */
};

}
}//end namespace arolib


#endif //AROLIB_IO_XMLINDOCUMENT_HPP

