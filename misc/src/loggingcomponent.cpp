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
 
#include "arolib/misc/loggingcomponent.h"

namespace arolib{

LoggingComponent::LoggersHandler::LoggersHandler(bool _resetOnDestruction) : resetOnDestruction(_resetOnDestruction){}

LoggingComponent::LoggersHandler::~LoggersHandler()
{
    if(resetOnDestruction)
        resetLoggers();
}

void LoggingComponent::LoggersHandler::resetLoggers()
{
    for(size_t i = 0 ; i < loggers.size() ; ++i){
        auto& p = r_at(loggers, i);//in reverse order in case one logger was changed more than once
        *p.second = p.first;//Restore the logger (at p.second) with its original copy (p.first)
    }
    loggers.clear();
}

LoggingComponent::LoggingComponent(arolib::LogLevel logLevel, const std::string &baseName)
    : m_logger(logLevel, baseName){}

LoggingComponent::LoggingComponent(Logger *parentLogger, const std::string &baseName)
    : m_logger(parentLogger, baseName){}

LoggingComponent::LoggingComponent(const LoggingComponent &other)
    : m_logger(other.m_logger){}

void LoggingComponent::setTemporalLoggersParent(LoggingComponent::LoggersHandler &lh, const LoggingComponent &parent, const LoggingComponent &item){
    lh.loggers.emplace_back( std::make_pair(item.m_logger, &item.m_logger) );//save a coppy of the current logger of 'item' in the handler (so that it can be restored later)
    item.logger().setParent(&parent.m_logger);
}

void LoggingComponent::setTemporalLoggersParent(LoggingComponent::LoggersHandler &lh, Logger *parent, Logger &item){
    lh.loggers.emplace_back( std::make_pair(item, &item) );//save a coppy of the current logger 'item' in the handler (so that it can be restored later)
    item.setParent(parent);
}

void LoggingComponent::resetTemporalLoggersParent(LoggingComponent::LoggersHandler &lh){
    lh.resetLoggers();
}


}
