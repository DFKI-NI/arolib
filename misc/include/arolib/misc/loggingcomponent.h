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
 
#ifndef AROLIBLOGGINGCOMPONENT_H
#define AROLIBLOGGINGCOMPONENT_H

#include <vector>
#include <memory>

#include "logger.h"
#include "container_helper.h"

namespace arolib {

/**
 * @brief A class that logs
 */
class LoggingComponent {
public:
    /**
     * @brief Get a reference to the logger
     * @return Reference to the logger
     */
    inline Logger& logger() const {return *m_logger;}

    /**
     * @brief Get the logger
     * @return Logger
     */
    inline std::shared_ptr<Logger> loggerPtr() const {return m_logger;}

protected:
    /**
     * @brief Struct to manage the logger's temporal imheritance (to save and restore the original logger's attributes when its parent logger is temporally changed )
     */
    struct LoggersHandler{
        friend class LoggingComponent;

        /**
         * @brief Constructor
         * @param _resetOnDestruction If true, the saved loggers are restored to their original state on destruction
         */
        LoggersHandler(bool _resetOnDestruction = true);

        /**
         * @brief Destructor
         */
        ~LoggersHandler();
    private:
        /**
         * @brief Restore the saved loggers to their original state
         */
        void resetLoggers();

        bool resetOnDestruction = true; /**< If true, the saved loggers are restored to their original on destruction */
        std::vector< std::pair< const std::shared_ptr<Logger>, const std::shared_ptr<Logger> > > loggers; /**< Contains the original parents of the loggers. <pointer to the corresponding logger, original parent> */
    };

    /**
     * @brief Constructor
     * @sa Logger
     * @param logLevel Logger's log level.
     * @param baseName Logger's baseName.
     */
    LoggingComponent(LogLevel logLevel = LogLevel::INFO, const std::string &baseName = "");

    /**
     * @brief Constructor
     * @sa Logger
     * @param parentLogger Logger's parent logger.
     * @param baseName Logger's baseName.
     */
    LoggingComponent(std::shared_ptr<Logger> parentLogger, const std::string &baseName = "");

    /**
     * @brief Copy constructor
     * @param other Other logging component from which to copy the attributes
     */
    LoggingComponent(const LoggingComponent& other);

    /**
     * @brief Set one logging component as a (temporal) parent of another logging component (for Logger inheritance)
     * @sa Logger
     * @param [in/out] lh Handler
     * @param parent Logging component to be set as a (temporal) parent
     * @param [in/out] item Logging component to be set as a (temporal) child
     */
    static void setTemporalLoggersParent(LoggersHandler& lh, const LoggingComponent &parent, const LoggingComponent &item);

    /**
     * @brief Set (recursivelly) one logging component as a (temporal) parent of other logging components (for Logger inheritance)
     * @sa Logger
     * @param [in/out] lh Handler
     * @param parent Logging component to be set as a (temporal) parent
     * @param [in/out] item First logging component to be set as a (temporal) child
     * @param [in/out] items Remanining logging component to be set as a (temporal) children
     */
    template<typename ... Ts>
    static void setTemporalLoggersParent(LoggersHandler& lh, const LoggingComponent &parent, const LoggingComponent &item, const Ts & ...items){
        setTemporalLoggersParent(lh, parent, item);
        return setTemporalLoggersParent(lh, parent, items...);
    }

    /**
     * @brief Set one logger as a (temporal) parent of another logger
     * @sa Logger
     * @param [in/out] lh Handler
     * @param parent Logger to be set as a (temporal) parent
     * @param [in/out] item Loggert to be set as a (temporal) child
     */
    static void setTemporalLoggersParent(LoggersHandler& lh, std::shared_ptr<Logger> parent, std::shared_ptr<Logger> &item);

    /**
     * @brief Set (recursivelly) one logger as a (temporal) parent of other loggers
     * @sa Logger
     * @param [in/out] lh Handler
     * @param parent Logger to be set as a (temporal) parent
     * @param [in/out] item First logger to be set as a (temporal) child
     * @param [in/out] items Remanining loggers to be set as a (temporal) children
     */
    template<typename ... Ts>
    static void setTemporalLoggersParent(LoggersHandler& lh, std::shared_ptr<Logger> parent, std::shared_ptr<Logger> &item, Ts & ...items){
        setTemporalLoggersParent(lh, parent, item);
        return setTemporalLoggersParent(lh, parent, items...);
    }

    /**
     * @brief Restore the loggers saven in the handler to their original state
     */
    static void resetTemporalLoggersParent(LoggersHandler& lh);

protected:
    std::shared_ptr<Logger> m_logger;
};

}

#endif // AROLIBLOGGINGCOMPONENT_H
