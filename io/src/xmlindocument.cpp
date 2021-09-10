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
 
#include "arolib/io/xmlindocument.hpp"

namespace arolib {
namespace io {

XMLInDocument::XMLInDocument(LogLevel logLevel):
    AroInDocument(logLevel)
{
    m_logger = Logger(logLevel, __FUNCTION__);
}

XMLInDocument::~XMLInDocument()
{
}

bool XMLInDocument::openDocument(ReadHandler *rh)
{
    if(!AroInDocument::openDocument())
        return false;
    if(rh)
        *rh = m_base;
    return true;
}

bool XMLInDocument::getBaseHandler(XMLInDocument::ReadHandler &rh)
{
    rh = m_base;
    return true;
}

bool XMLInDocument::hasBranch(const ReadHandler &base, const std::string &name){
    if( base.t.get_child_optional(name) )
        return true;
    return false;
}

bool XMLInDocument::hasBranch(const std::string &name)
{
    if(!m_isDocOpen)
        return false;
    return hasBranch(m_base, name);
}

bool XMLInDocument::getBranchHandler(const XMLInDocument::ReadHandler &base, const std::string &name, XMLInDocument::ReadHandler &branch, Logger *_logger)
{
    try{
        auto ochild = base.t.get_child_optional(name);
        if(!ochild)
            return false;
        branch.t = *ochild;
        return true;
    }
    catch(...){
        return false;
    }

}

bool XMLInDocument::getBranchHandler(const ReadHandler &base, const std::vector<std::string> &tags, ReadHandler &branch, Logger* _logger)
{
    branch = base;
    Logger logger(LogLevel::CRITIC, "XMLInDocument");
    logger.setParent(_logger);
    for(size_t i = 0 ; i < tags.size() ; ++i){
        if(!getBranchHandler(branch, tags.at(i), branch, _logger)){
            logger.printOut(LogLevel::WARNING, __FUNCTION__, "Cannot open parent tag '" + std::to_string(i) + ": '" + tags.at(i) + "'");
            return false;
        }
    }

    return true;
}

bool XMLInDocument::getBranchHandler(const std::string &name, ReadHandler &branch)
{
    if(!m_isDocOpen)
        return false;
    return getBranchHandler(m_base, name, branch, &m_logger);

}

bool XMLInDocument::getBranchHandler(const std::vector<std::string> &tags, ReadHandler &branch)
{
    if(!m_isDocOpen)
        return false;
    return getBranchHandler(m_base, tags, branch, &m_logger);
}

bool XMLInDocument::getMultiBranchHandlers(const ReadHandler &base,
                                           const std::string &tag,
                                           const std::function<bool (const XMLInDocument::ReadHandler &)> funct,
                                           const std::vector<std::string> &parentTags,
                                           Logger *_logger)
{
    ReadHandler branch;
    if(!getBranchHandler(base, parentTags, branch, _logger))
        return false;

    BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, branch.t){
        if(v.first == tag){
            if (!funct( ReadHandler(v.second) ) )
                return false;
        }
    }
    return true;
}

bool XMLInDocument::getMultiBranchHandlers(const std::string &tag,
                                           const std::function<bool (const ReadHandler &)> funct,
                                           const std::vector<std::string> &parentTags)
{
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }
    return getMultiBranchHandlers(m_base, tag, funct, parentTags, &m_logger);
}

bool XMLInDocument::read(const ReadHandler &base, std::string &value){
    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    auto ovalue = base.t.get_value_optional<std::string>();
    if(!ovalue)
        return false;
    value = *ovalue;
    return true;
}

bool XMLInDocument::read( const ReadHandler & base, std::map<std::string, std::map<std::string, std::string> > &values){ // map<tag, map< name , value > >

    if(!m_isDocOpen){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
        return false;
    }

    try{
        BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, base.t){
            BOOST_FOREACH( boost::property_tree::ptree::value_type const& vv, v.second){
                values[v.first][vv.first] =  vv.second.get_value<std::string>();
            }
        }
        return true;
    }
    catch(std::exception &e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Values map: ") + e.what());
        return false;
    }
}

bool XMLInDocument::openDoc()
{
    if(!AroInDocument::isReadyToRead())
        return false;

    m_is->seekg(0, m_is->beg);

    try{
        read_xml(*m_is, m_base.t);

        if(!getBranchHandler(m_base, m_docTag, m_base)){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Cannot open document tag.");
            return false;
        }

        return true;
    }
    catch (std::exception& e){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Error reading input stream/file: '") + e.what() + "'");
        return false;
    }
}

bool XMLInDocument::closeDoc()
{
    if(!AroInDocument::isReadyToRead())
        return false;
    m_base = ReadHandler();
    return true;
}

bool XMLInDocument::getBranch(const boost::property_tree::ptree & base, const std::string &name, boost::property_tree::ptree &branch)
{
    try{
        auto ochild = base.get_child_optional(name);
        if(!ochild)
            return false;
        branch = *ochild;
        return true;
    }
    catch(...){
        return false;
    }

}

bool XMLInDocument::getBranch(const boost::property_tree::ptree & base, const std::vector<std::string> &tags, boost::property_tree::ptree &branch, LogLevel logLevel)
{
    branch = base;
    for(size_t i = 0 ; i < tags.size() ; ++i){
        if(!getBranch(branch, tags.at(i), branch)){
            Logger::printOut(logLevel, LogLevel::WARNING, "XMLInDocument", __FUNCTION__, "Cannot open tag '" + std::to_string(i) + ": '" + tags.at(i) + "'");
            return false;
        }
    }
    return true;

}

bool XMLInDocument::getBranch(const std::string &name, boost::property_tree::ptree &branch)
{
    return getBranch(m_base.t, name, branch);
}

bool XMLInDocument::getBranch(const std::vector<std::string> &tags, boost::property_tree::ptree &branch)
{
    return getBranch(m_base.t, tags, branch);
}

bool XMLInDocument::getBranch(const XMLInDocument::ReadHandler &base, const std::string &name, XMLInDocument::ReadHandler &branch)
{
    return getBranch(base.t, name, branch.t);
}

bool XMLInDocument::getBranch(const XMLInDocument::ReadHandler &base, const std::vector<std::string> &tags, XMLInDocument::ReadHandler &branch)
{
    return getBranch(base.t, tags, branch.t);
}

bool XMLInDocument::getBranch(const std::string &name, XMLInDocument::ReadHandler &branch)
{
    return getBranch(m_base.t, name, branch.t);
}


bool XMLInDocument::getBranch(const std::vector<std::string> &tags, ReadHandler &branch)
{
    return getBranch(m_base.t, tags, branch.t);
}

XMLInDocument::ReadHandler XMLInDocument::createRH(const boost::property_tree::ptree &tree){
    return ReadHandler(tree);
}

const boost::property_tree::ptree &XMLInDocument::RHTree(const XMLInDocument::ReadHandler &rh){
    return rh.t;
}

boost::property_tree::ptree &XMLInDocument::RHTree(XMLInDocument::ReadHandler &rh){
    return rh.t;
}

}
}//end namespace arolib

