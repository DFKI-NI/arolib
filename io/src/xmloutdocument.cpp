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
 
#include "arolib/io/xmloutdocument.hpp"

namespace arolib {
namespace io {

XMLOutDocument::XMLOutDocument(LogLevel logLevel):
    AroOutDocument(logLevel)
{
    m_logger = Logger(logLevel, __FUNCTION__);
}

XMLOutDocument::~XMLOutDocument()
{
    if(m_os)
        closeTag(-1);
}

bool XMLOutDocument::openTag(const std::string& tag, const std::string &extra){
    if(!isReadyToWrite())
        return false;
    m_openTags.emplace_back(tag);
    if(tag.empty())
        return true;
    tabs(m_nTabs++);
    if(extra.empty())
        *m_os << "<" << tag << ">\n";
    else
        *m_os << "<" << tag << " " << extra << ">\n";
    return true;
}

bool XMLOutDocument::closeTag(int numTags){
    if(!isReadyToWrite())
        return false;
    if(numTags < 0 || numTags >= m_openTags.size()){
        while (!m_openTags.empty()){
            if(m_openTags.back().empty()){
                m_openTags.pop_back();
                continue;
            }
            tabs(--m_nTabs);
            *m_os << "</" << m_openTags.back() << ">\n";
            m_openTags.pop_back();
        }
        return true;
    }

    for(; numTags > 0; --numTags){
        if(m_openTags.back().empty()){
            m_openTags.pop_back();
            continue;
        }
        tabs(--m_nTabs);
        *m_os << "</" << m_openTags.back() << ">\n";
        m_openTags.pop_back();
    }
    return true;
}

bool XMLOutDocument::add(const char *value, std::string tag)
{
    return add(std::string(value), tag);
}

bool XMLOutDocument::add(const std::string &value, std::string tag){
    if(!isReadyToWrite())
        return false;
    if(value.empty())
        return true;
    tabs();
    *m_os << "<" << tag << ">" << value << "</" << tag << ">\n";
    return true;
}

bool XMLOutDocument::add(const std::map<std::string, std::map<std::string, std::string> > &values, std::string tag){
    bool ok = true;
    if(!isReadyToWrite())
        return false;
    if(!tag.empty())
        openTag(tag);

    for(const auto &tags : values){
        openTag(tags.first);
        for(const auto &v : tags.second){
            ok &= add(v.second, v.first);
            if (!ok)
                break;
        }
        closeTag();
        if (!ok)
            break;
    }

    if(!tag.empty())
        closeTag();
    return ok;
}

bool XMLOutDocument::openDoc()
{
    if(!AroOutDocument::isReadyToWrite())
        return false;

    *m_os << std::setprecision(12)
          << "<?xml version='1.0'?>\n";

    return openTag(m_docTag);
}

bool XMLOutDocument::closeDoc()
{
    if(!AroOutDocument::isReadyToWrite())
        return false;
    closeTag(-1);
    return true;
}

void XMLOutDocument::tabs(int n){
    if(n <= 0)
        return;
    *m_os << std::string(n, '\t');
}

void XMLOutDocument::tabs()
{
    if(m_nTabs <= 0)
        return;
    *m_os << std::string(m_nTabs, '\t');
}

}
}//end namespace arolib

