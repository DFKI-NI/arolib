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
 
#include "arolib/io/arodocument.hpp"

namespace arolib {
namespace io {

const std::string AroDocument::m_docTag = "Document";
const std::string AroDocument::m_coordTypeTag = "coordinatesType";

AroDocument::AroDocument(LogLevel logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

AroDocument::~AroDocument()
{
}

bool AroDocument::openDocument()
{
    if(m_isDocOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document is already open.");
        return false;
    }

    m_isDocOpen = openDoc();
    return m_isDocOpen;
}

bool AroDocument::closeDocument()
{
    if(!m_isDocOpen){
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "Document is not open.");
        return false;
    }

    m_isDocOpen = !closeDoc();
    return !m_isDocOpen;
}



AroOutDocument::AroOutDocument(LogLevel logLevel):
    AroDocument(logLevel)
{
    logger() = Logger(logLevel, __FUNCTION__);
}

AroOutDocument::~AroOutDocument()
{
    if(m_isOpen)
        closeFile();
}

bool AroOutDocument::openFile(const std::string &filename)
{
    if(m_isOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document (file) is already open.");
        return false;
    }
    if(m_os){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The output stream was set externally already.");
        return false;
    }

    auto os = new std::ofstream();
    os->open(filename);
    if(!os->is_open()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error opening file '" + filename + "'.");
        delete os;
        return false;
    }
    m_isOpen = true;
    m_os = os;
    return true;
}

bool AroOutDocument::closeFile()
{
    if(!m_isOpen && m_os){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The output stream was set externally and cannot be closed.");
        return false;
    }
    if(!m_isOpen){
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "The output stream (file) is not open.");
        return false;
    }
    if(!m_os){
        logger().printOut(LogLevel::CRITIC, __FUNCTION__, "The output stream is open but not allocated!!!.");
        return false;
    }

    if(m_isDocOpen)
        closeDocument();

    std::ofstream* of = dynamic_cast<std::ofstream*>(m_os);
    of->close();
    delete m_os;
    m_os = nullptr;
    m_isOpen = false;
    return true;
}

bool AroOutDocument::setOutputStream(std::ostream *os)
{
    if(m_isOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The output stream (file) was set internally after callin 'open'. Cannot be (re)set externally.");
        return false;
    }
    if(m_os)
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "The output stream was already set externally. Resetting it...");

    m_os = os;
    return true;

}

bool AroOutDocument::unsetOutputStream()
{
    if(m_isOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The output stream (file) was set internally after callin 'open'. Cannot be unset externally.");
        return false;
    }
    if(!m_os){
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "The output stream is not set.");
        return false;
    }
    m_os = nullptr;
    return true;
}

const std::ostream *AroOutDocument::outputStream() const
{
    return m_os;
}

bool AroOutDocument::setCoordinatesTypes(Point::ProjectionType in, Point::ProjectionType out)
{
    m_coordinatesType_in = in;
    m_coordinatesType_out = out;
    return true;
}

bool AroOutDocument::isReadyToWrite() const
{
    if(!m_os){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The output stream is not ready.");
        return false;
    }
    return true;
}

AroInDocument::AroInDocument(LogLevel logLevel):
    AroDocument(logLevel)
{
    logger() = Logger(logLevel, __FUNCTION__);
}

AroInDocument::~AroInDocument()
{
    if(m_isOpen)
        closeFile();
}

bool AroInDocument::openFile(const std::string &filename)
{
    if(m_isOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Document (file) is already open.");
        return false;
    }
    if(m_is){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The input stream was set externally already.");
        return false;
    }

    auto is = new std::ifstream();
    is->open(filename);
    if(!is->is_open()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error opening file '" + filename + "'.");
        delete is;
        return false;
    }

    m_isOpen = true;
    m_is = is;
    return true;
}

bool AroInDocument::closeFile()
{
    if(!m_isOpen && m_is){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The input stream was set externally and cannot be closed.");
        return false;
    }
    if(!m_isOpen){
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "The input stream (file) is not open.");
        return false;
    }
    if(!m_is){
        logger().printOut(LogLevel::CRITIC, __FUNCTION__, "The input stream is open but not allocated!!!.");
        return false;
    }

    std::ifstream* is = dynamic_cast<std::ifstream*>(m_is);
    is->close();
    delete m_is;
    m_is = nullptr;
    m_isOpen = false;

    return true;

}

bool AroInDocument::setInputStream(std::istream *is)
{
    if(m_isOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The input stream (file) was set internally after callin 'open'. Cannot be (re)set externally.");
        return false;
    }
    if(m_is)
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "The input stream was already set externally. Resetting it...");

    m_is = is;
    return true;
}

bool AroInDocument::unsetInputStream()
{
    if(m_isOpen){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The input stream (file) was set internally after callin 'open'. Cannot be unset externally.");
        return false;
    }
    if(!m_is){
        logger().printOut(LogLevel::WARNING, __FUNCTION__, "The input stream is not set.");
        return false;
    }
    m_is = nullptr;
    return true;

}

const std::istream *AroInDocument::inputStream() const
{
    return m_is;
}

bool AroInDocument::setCoordinatesType(Point::ProjectionType out)
{
    m_coordinatesType_out = out;
    return true;
}

bool AroInDocument::isReadyToRead() const
{
    if(!m_is){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "The input stream is not ready.");
        return false;
    }
    return true;
}



}
}//end namespace arolib

