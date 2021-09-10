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
 
#include "arolib/misc/logger.h"

namespace arolib{

std::mutex Logger::m_mutex;

Logger::Logger(const arolib::LogLevel &logLevel, const std::string &baseName) :
    m_logLevel(logLevel),
    m_baseName(baseName)
{
    //m_timestamp = std::chrono::steady_clock::now();
}

Logger::Logger(Logger *parent, const std::string &baseName):
    m_parent(parent),
    m_baseName(baseName)
{
    if(parent){
        m_os = parent->os();
        m_logLevel = parent->logLevel();
        m_includeElapsedTime = parent->m_includeElapsedTime;
        m_includeElapsedTime = parent->m_includeTimestamp;
    }
    //m_timestamp = std::chrono::steady_clock::now();
}

Logger::Logger(const Logger &other)
{
    this->m_parent = other.m_parent;
    this->m_logLevel = other.logLevel();
    this->m_baseName = other.m_baseName;
    this->m_os = other.os();
}

Logger &Logger::operator=(const Logger &other){
    if (this != &other) {
        this->m_parent = other.m_parent;
        this->m_logLevel = other.logLevel();
        this->m_baseName = other.m_baseName;
        this->m_timestamp = other.m_timestamp;
        this->m_includeTimestamp = other.m_includeTimestamp;
        this->m_includeElapsedTime = other.m_includeElapsedTime;
        this->m_os = other.os();
    }
    return *this;
}

Logger::~Logger()
{
}

void Logger::printOut(LogLevel _logLevel,
                      const std::string &function,
                      const std::string &msg,
                      int precision,
                      bool endLine) const
{

    if (_logLevel > logLevel() || logLevel() == LogLevel::NONE)
        return;

    auto out = os();

    std::lock_guard<std::mutex> guard(m_mutex);

    if(precision < 0)
        precision = m_precision;

    if(precision >= 0)
        *out << std::setprecision(precision);
    *out << getHeader(_logLevel, function) << msg;
    if(endLine)
        *out << std::endl;
}

void Logger::printOut(LogLevel logLevel_base,
                      LogLevel logLevel_msg,
                      const std::string &baseName,
                      const std::string &function,
                      const std::string &msg,
                      int precision,
                      bool endLine,
                      std::ostream *os)
{
    Logger tmp(logLevel_base, baseName);
    tmp.setOutputStream(os);
    tmp.printOut(logLevel_msg, function, msg, precision, endLine);
}

void Logger::printOut(LogLevel _logLevel,
                      const std::string &msg,
                      int precision,
                      bool endLine) const
{
    if(precision < 0)
        precision = m_precision;
    printOut(logLevel(), _logLevel, msg, precision, endLine, os());
}

void Logger::printOut(LogLevel logLevel_base,
                      LogLevel logLevel_msg,
                      const std::string &msg,
                      int precision,
                      bool endLine,
                      std::ostream *os)
{
    if (logLevel_msg > logLevel_base || logLevel_base == LogLevel::NONE)
        return;

    std::lock_guard<std::mutex> guard(m_mutex);

    if(!os)
        os = &std::cout;

    if(precision >= 0)
        *os << std::setprecision(precision);
    *os << msg;
    if(endLine)
        *os << std::endl;
}

LogLevel Logger::logLevel() const
{
    if(m_parent)
        return m_parent->logLevel();
    return m_logLevel;
}

std::string Logger::baseName() const
{
    return m_baseName;
}

void Logger::setLogLevel(const LogLevel &logLevel)
{
    resetParent();
    m_logLevel = logLevel;
}

void Logger::setBaseName(const std::string &baseName)
{
    m_baseName = baseName;
}


void Logger::setParent(const Logger *parent)
{
    if(!parent)
        return;
    m_logLevel = parent->logLevel();
    m_os = parent->os();
}

void Logger::resetParent()
{
    if(!m_parent)
        return;
    m_parent = nullptr;
}

void Logger::setOutputStream(std::ostream *os)
{
    if(!os)
        return;
    resetParent();
    m_os = os;
}

void Logger::setPrecision(int precision)
{
    m_precision = precision;
}

void Logger::includeTimestamp(bool include)
{
    m_includeTimestamp = include;
}

void Logger::includeElapsedTime(bool include)
{
    m_includeElapsedTime = include;
}
std::ostream *Logger::os() const
{
    if(m_parent)
        return m_parent->os();
    return m_os;
}

std::string Logger::getHeader(LogLevel _logLevel, const std::string& function) const
{
    std::string hdr = "[";
    switch (_logLevel){
    case LogLevel::CRITIC:
        if(os() == &std::cout)
            hdr += "\033[1;31mCRITIC\033[0m ";
        else
            hdr += "CRITIC ";
        break;
    case LogLevel::ERROR:
        if(os() == &std::cout)
            hdr += "\033[1;31mERROR\033[0m  ";
        else
            hdr += "ERROR  ";
        break;
    case LogLevel::WARNING:
        if(os() == &std::cout)
            hdr += "\033[1;33mWARNING\033[0m";
        else
            hdr += "WARNING";
        break;
    case LogLevel::INFO:
        hdr += "INFO   ";
        break;
    case LogLevel::DEBUG:
        hdr += "DEBUG  ";
        break;
    default:
        hdr += "?????  ";
        break;
    }
    hdr += "]";

    if( !m_baseName.empty() ){
        hdr += "[" + m_baseName;
        if( !function.empty() )
            hdr += "::" + function;
        hdr += "] ";
    }
    else{
        if( !function.empty() )
            hdr += "[" + function + "] ";
    }

    auto timestamp = m_timestamp;

    //m_timestamp = std::chrono::steady_clock::now();
    m_timestamp.setNow();

    std::string ts_hdr;
    if(m_includeTimestamp){
        //ts_hdr += "{" + DateTime().getTimeStr(true) + "}";
        ts_hdr += "[" + m_timestamp.getTimeStr(true) + "]";
    }
    if(m_includeElapsedTime){
        //ts_hdr += "{" + std::to_string( std::chrono::duration_cast<std::chrono::microseconds>(m_timestamp - timestamp).count() ) + "us}";
        std::string timeElapsedStr;
        auto timeElapsed = m_timestamp.timeSince(timestamp);
        if(timeElapsed < 1)
            timeElapsedStr = std::to_string((int)(timeElapsed*1e6)) + "us";
        else
            timeElapsedStr = std::to_string(timeElapsed) + "s";
        if(timeElapsedStr.size() < 8)
            timeElapsedStr = std::string( 8 - timeElapsedStr.size(), ' ' ) + timeElapsedStr;
        ts_hdr += "[" + timeElapsedStr + "]";
    }

    return ts_hdr+hdr;
}

bool Logger::getIncludeTimestamp() const {
    if(m_parent)
        return m_parent->getIncludeTimestamp();
    return m_includeTimestamp;
}

bool Logger::getIncludeElapsedTime() const {

    if(m_parent)
        return m_parent->getIncludeElapsedTime();
    return m_includeElapsedTime;
}

}
