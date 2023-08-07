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
 
#ifndef AROLIBLOGGER_H
#define AROLIBLOGGER_H

#include <string>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <memory>
#include <tuple>
#include <functional>
#include <chrono>

#include "arolib/misc/tuple_helper.h"
#include "arolib/misc/datetime.hpp"

namespace arolib {

/**
 * @brief Log level used to decide what messages to print
 */
enum class LogLevel{
    NONE, /**< None (highest importance) --> If logger has NON level, nothing gets printed */
    CRITIC, /**< Critic */
    ERROR, /**< Error */
    WARNING, /**< Warning */
    INFO, /**< Information */
    DEBUG /**< Debug */
};

/**
 * @brief Logger class
 *
 * If the logger has a parent, the logger's log level and output stream is inherited from the parent. Inheritance can come from several generations (i.e. the parent of the parent of the parent...)
 */
class Logger {

public:

    static LogLevel DefaultLogLevel;

    /**
     * @brief Constructor
     * @param logLevel Logger's log level. Only logs with this level or 'lower' (more important) will be printed by the logger
     * @param baseName Basename to be printed in the messages' header (disregarded if empty-string).
     */
    Logger(const LogLevel& logLevel = DefaultLogLevel, const std::string &baseName = "");

    /**
     * @brief Constructor
     *
     * If the logger has a parent, the logger's log level and output stream is inherited from the parent.
     * @param parent Pointer to the parent logger (this logger will have some same attributes (log level, output stream, etc) as the parent). If = null --> no parent.
     * @param baseName Basename to be printed in the messages (disregarded if empty-string).
     */
    Logger(std::shared_ptr<Logger> parent, const std::string &baseName = "");

    /**
     * @brief Copy constructor
     * @param other Other logger from which to copy the attributes
     */
    Logger(const Logger& other);

    /**
     * @brief Copy assignment
     * @param other Other logger from which to copy the attributes
     */
    virtual Logger& operator=(const Logger& other);

    /**
     * @brief Destructor
     */
    virtual ~Logger();

    /**
     * @brief Print a given message in the assigned output stream
     *
     * The message will only be printed if the log level of the message is equal or lower (more important) that the log level of the logger
     * If the logger has a parent, the logger's log level and output stream is inherited from the parent.
     * @param _logLevel Log level of the message
     * @param function Function from where the message is printed, to be added in the printed message's header. Disregarded id empty-string
     * @param msg Message to be printed
     * @param precision (number) precision to be set in the output stream before printing. If <0, it uses the precision stored in the logger.
     * @param endLine If set to true, the logger will print the message and end the line (std::endl)
     */
    void printOut(LogLevel _logLevel,
                  const std::string& function,
                  const std::string& msg,
                  int precision = 10,
                  bool endLine = true) const;

    /**
     * @brief Print a given message in the assigned output stream
     *
     * The message will only be printed if the log level of the message is equal or lower (more important) that the base log level
     * @param logLevel_base Base log level
     * @param logLevel_msg Log level of the message
     * @param baseName Basename to be printed in the message's header (disregarded if empty-string).
     * @param function Function from where the message is printed, to be added in the printed message's header. Disregarded id empty-string
     * @param msg Message to be printed
     * @param precision (number) precision to be set in the output stream before printing. Uses (logger) default if if <0.
     * @param endLine If set to true, the logger will print the message and end the line (std::endl)
     * @param os Output stream (if = null --> std::cout)
     */
    static void printOut(LogLevel logLevel_base,
                         LogLevel logLevel_msg,
                         const std::string &baseName,
                         const std::string &function,
                         const std::string &msg,
                         int precision = 10,
                         bool endLine = true,
                         std::ostream *os = nullptr);

    /**
     * @brief Print a given message without header in the assigned output stream
     *
     * The message will only be printed if the log level of the message is equal or lower (more important) that the log level of the logger
     * If the logger has a parent, the logger's log level and output stream is inherited from the parent.
     * @param _logLevel Log level of the message
     * @param msg Message to be printed
     * @param precision (number) precision to be set in the output stream before printing. If <0, it uses the precision stored in the logger.
     * @param endLine If set to true, the logger will print the message and end the line (std::endl)
     */
    void printOut(LogLevel _logLevel,
                  const std::string &msg,
                  int precision = 10,
                  bool endLine = true) const;

    /**
     * @brief Print a given message without header in the assigned output stream
     *
     * The message will only be printed if the log level of the message is equal or lower (more important) that the base log level
     * @param logLevel_base Base log level
     * @param logLevel_msg Log level of the message
     * @param msg Message to be printed
     * @param precision (number) precision to be set in the output stream before printing. Uses (logger) default if if <0.
     * @param endLine If set to true, the logger will print the message and end the line (std::endl)
     * @param os Output stream (if = null --> std::cout)
     */
    static void printOut(LogLevel logLevel_base,
                         LogLevel logLevel_msg,
                         const std::string &msg,
                         int precision = 10,
                         bool endLine = true,
                         std::ostream *os = nullptr);

    /**
     * @brief Print a given message containing several (multi-type) items in the assigned output stream
     *
     * The message will only be printed if the log level of the message is equal or lower (more important) that the log level of the logger
     * If the logger has a parent, the logger's log level and output stream is inherited from the parent.
     * @note Types accepted for the items are basic types, std::strings, or any type that has a definned 'std::ostream& operator<<'
     * @param _logLevel Log level of the message
     * @param function Function from where the message is printed, to be added in the printed message's header. Disregarded id empty-string
     * @param msg Tuple containing the message items/elements
     * @param precision (number) precision to be set in the output stream before printing. If <0, it uses the precision stored in the logger.
     * @param endLine If set to true, the logger will print the message and end the line (std::endl)
     */
    template<typename ... Ts>
    void printOut(LogLevel _logLevel,
                  const std::string& function,
                  const std::tuple<Ts...>& msg,
                  int precision = 10,
                  bool endLine = true) const{

        if (_logLevel > logLevel() || logLevel() == LogLevel::NONE)
            return;

        auto out = os();

        {
            std::lock_guard<std::mutex> guard(m_mutex);

            if(precision < 0)
                precision = m_precision;

            if(precision >= 0)
                *out << std::setprecision(precision);

            *out << getHeader(_logLevel, function);
        }

        printTupleElements<sizeof...(Ts)>(out, msg);

        {
            std::lock_guard<std::mutex> guard(m_mutex);
            if(endLine)
                *out << std::endl;
        }
    }

    /**
     * @brief Print a given message containing several (multi-type) items in the assigned output stream
     *
     * The message will only be printed if the log level of the message is equal or lower (more important) that the base log level
     * @note Types accepted for the items are basic types, std::strings, or any type that has a definned 'std::ostream& operator<<'
     * @param logLevel_base Base log level
     * @param logLevel_msg Log level of the message
     * @param baseName Basename to be printed in the message's header (disregarded if empty-string).
     * @param function Function from where the message is printed, to be added in the printed message's header. Disregarded id empty-string
     * @param msg Tuple containing the message items/elements
     * @param precision (number) precision to be set in the output stream before printing. Uses (logger) default if if <0.
     * @param endLine If set to true, the logger will print the message and end the line (std::endl)
     * @param os Output stream (if = null --> std::cout)
     */
    template<typename ... Ts>
    static void printOut(LogLevel logLevel_base,
                         LogLevel logLevel_msg,
                         const std::string &baseName,
                         const std::string &function,
                         const std::tuple<Ts...>& msg,
                         int precision = 10,
                         bool endLine = true,
                         std::ostream *os = nullptr){

        Logger tmp(logLevel_base, baseName);
        tmp.setOutputStream(os);
        tmp.printOut(logLevel_msg, function, msg, precision, endLine);
    }

    /**
     * @brief Print a given message containing several (multi-type) items in the assigned output stream
     *
     * The message will only be printed if the log level of the message is equal or lower (more important) that the log level of the logger
     * If the logger has a parent, the logger's log level and output stream is inherited from the parent.
     * @note Types accepted for the items are basic types, std::strings, or any type that has a definned 'std::ostream& operator<<'
     * @param _logLevel Log level of the message
     * @param function Function from where the message is printed, to be added in the printed message's header. Disregarded id empty-string
     * @param precision (number) precision to be set in the output stream before printing. If <0, it uses the precision stored in the logger.
     * @param msg Message items/elements (e.g. "This is an example with ", 4, " elements to print a point: " , Point(10,20) )
     */
    template<typename ... Ts>
    void printOut(LogLevel _logLevel,
                  const std::string &function,
                  int precision,
                  const Ts & ... items) const{

        if (_logLevel > logLevel() || logLevel() == LogLevel::NONE)
            return;

        auto out = os();
        {
            std::lock_guard<std::mutex> guard(m_mutex);

            if(precision < 0)
                precision = m_precision;

            if(precision >= 0)
                *out << std::setprecision(precision);

            *out << getHeader(_logLevel, function);
        }
        printItem(out, items...);

    }

    /**
     * @brief Print a given message containing several (multi-type) items in the assigned output stream
     *
     * The message will only be printed if the log level of the message is equal or lower (more important) that the base log level
     * @note Types accepted for the items are basic types, std::strings, or any type that has a definned 'std::ostream& operator<<'
     * @param logLevel_base Base log level
     * @param logLevel_msg Log level of the message
     * @param baseName Basename to be printed in the message's header (disregarded if empty-string).
     * @param function Function from where the message is printed, to be added in the printed message's header. Disregarded id empty-string
     * @param precision (number) precision to be set in the output stream before printing. Uses (logger) default if if <0.
     * @param msg Message items/elements (e.g. "This is an example with ", 4, " elements to print a point: " , Point(10,20) )
     * @param os Output stream (if = null --> std::cout)
     */
    template<typename ... Ts>
    static void printOut(LogLevel logLevel_base,
                         LogLevel logLevel_msg,
                         const std::string &baseName,
                         const std::string &function,
                         int precision,
                         std::ostream *os,
                         const Ts & ... items){

        Logger tmp(logLevel_base, baseName);
        tmp.setOutputStream(os);
        tmp.printOut(logLevel_msg, function, precision, items...);

    }


    /**
     * @brief Print a critic message
     */
    template<typename ... Ts>
    void printCritic(const Ts & ... items) const{
        printOut(LogLevel::CRITIC, items...);
    }

    /**
     * @brief Print an error message
     */
    template<typename ... Ts>
    void printError(const Ts & ... items) const{
        printOut(LogLevel::ERROR, items...);
    }

    /**
     * @brief Print a warning message
     */
    template<typename ... Ts>
    void printWarning(const Ts & ... items) const{
        printOut(LogLevel::WARNING, items...);
    }

    /**
     * @brief Print an info message
     */
    template<typename ... Ts>
    void printInfo(const Ts & ... items) const{
        printOut(LogLevel::INFO, items...);
    }

    /**
     * @brief Print a debug message
     */
    template<typename ... Ts>
    void printDebug(const Ts & ... items) const{
        printOut(LogLevel::DEBUG, items...);
    }

    /**
     * @brief Get the stored log level of the logger. If the logger has a parent, this method will return the log level inherited from the parent.
     * @return Stored log level of the logger or log level inherited from the parent
     */
    LogLevel logLevel() const;

    /**
     * @brief Get the stored basename of the logger
     * @return Stored basename of the logger
     */
    std::string baseName() const;

    /**
     * @brief Get the stored basename of the logger
     * @return Stored basename of the logger
     */
    inline const std::shared_ptr<Logger> parent() const { return m_parent; }

    /**
     * @brief Set the log level and reset/remove the parent if existent.
     * @param logLevel Log level
     */
    void setLogLevel(const LogLevel &logLevel);


    /**
     * @brief Set the basename
     * @param baseName Basename to be printed in the message's header (disregarded if empty-string).
     */
    void setBaseName(const std::string& baseName);

    /**
     * @brief Set the logger's parent logger
     *
     * If the logger has a parent, the logger's log level and output stream is inherited from the parent.
     * @param parent Pointer to the parent logger. If = null --> no parent
     */
    void setParent(const std::shared_ptr<Logger> parent);

    /**
     * @brief Remove the logger's parent logger (i.e. no parent)
     */
    void resetParent();

    /**
     * @brief Set the logger's output stream and reset/remove the parent if existent.
     * @param os Pointer to the output stream (if = null, nothing happens)
     */
    void setOutputStream(std::ostream* os = &std::cout);

    /**
     * @brief Set the logger's (number) precision
     * @param precision (number) precision
     */
    void setPrecision(int precision);

    /**
     * @brief Should the timestamp be included in the headers?
     * @param include If true, the timestamp will be included in the headers
     */
    void includeTimestamp(bool include);

    /**
     * @brief Should the elapsed time since the last message be included in the headers?
     * @param include If true, the elapsed time since the last message will be included in the headers
     */
    void includeElapsedTime(bool include);

protected:
    /**
     * @brief Obtain the loggers output stream (pointer). If the logger has a parent, this os is the one inherited from the parent.
     * @return Pointer to the output stream of the logger
     */
    std::ostream *os() const;

    /**
     * @brief Build a message header
     * @param Message log level
     * @param basename Basename. Disregarded if empty-string.
     * @param function Function from which the message is being printed. Disregarded if empty-string.
     * @return Message header
     */
    std::string getHeader(LogLevel _logLevel, const std::string &function) const;

    /**
     * @brief get m_includeTimestamp
     * @return m_includeTimestamp
     */
    bool getIncludeTimestamp() const;

    /**
     * @brief get m_includeElapsedTime
     * @return m_includeElapsedTime
     */
    bool getIncludeElapsedTime() const;

    /**
     * @brief Print recursivelly and consecutivelly the elements of a tupple in the given output stream.
     * @note Types accepted for the items are basic types, std::strings, or any type that has a definned 'std::ostream& operator<<'
     * @param os Pointer to the output stream
     * @param tuple Tuple containing the elements to be printed
     */
    template <size_t SIZE,  /**< Amount of elements to be printed (from the back of the tuple) */
              typename... Ts , /**< Tuple elements' types */
              typename = typename std::enable_if< (SIZE > 0), void >::type > /**< Called only when there are elements still to be printed */
    static void printTupleElements(std::ostream* os, const std::tuple<Ts...> & tuple){
        static_assert( ( SIZE <= sizeof...(Ts) ), "The SIZE must be <= the size of the tuple");
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            *os << std::get< sizeof...(Ts) - SIZE >(tuple); //print first element to be printed
        }
        printTupleElements<SIZE-1, Ts...>(os, tuple); //send remaining elements to be printed
    }

    /**
     * @brief Used to avoid errors when SIZE = 0 or to stop calling printTupleElements recursivelly when there are no more elements to be printed.
     */
    template <size_t SIZE, /**< Amount of elements to be printed (=0) */
              typename... Ts , /**< Tuple elements' types */
              typename = typename std::enable_if< (SIZE == 0), void >::type >/**< Called only when there are no elements to be printed */
    static void printTupleElements(...){
        return;//do nothing / stop printing
    }

    /**
     * @brief Prints an item in the given output stream and ends the line (std::endl).
     * @note Types accepted for the item are basic types, std::strings, or any type that has a definned 'std::ostream& operator<<'
     * @param os Pointer to the output stream
     * @param item Item to be printed
     */
    template<typename T> /**< Item's type */
    static void printItem(std::ostream* os, const T & item ){
        std::lock_guard<std::mutex> guard(m_mutex);
        *os << item << std::endl;
    }

    /**
     * @brief Prints recursivelly more than one item in the given output stream and ends the line (std::endl).
     * @note Types accepted for the item are basic types, std::strings, or any type that has a definned 'std::ostream& operator<<'
     * @param os Pointer to the output stream
     * @param item Next item to be printed
     * @param item Remaining items to be printed
     */
    template<typename T, /**< Type of the next item to be printed */
             typename ... Ts> /**< Types of the remaining items to be printed */
    static void printItem(std::ostream* os, const T & item, const Ts & ... items ){
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            *os << item;
        }
        printItem(os, items...);
    }

protected:
    static /*mutable*/ std::mutex m_mutex; /**< Mutex */
    LogLevel m_logLevel = LogLevel::INFO; /**< Logger's log level */
    std::string m_baseName = ""; /**< Logger's base name (to be printed in the message headers iif not an empty-string) */
    mutable std::ostream* m_os = &std::cout; /**< Pointer to the logger's output stream */
    std::shared_ptr<Logger> m_parent = nullptr; /**< Logger's parent (to inherit its log level and output stream) */
    int m_precision = 10; /**< Logger's (number) precision */
    //mutable std::chrono::steady_clock::time_point m_timestamp; /**< Timestamp of the last message */
    mutable DateTime m_timestamp; /**< Timestamp of the last message */
    bool m_includeTimestamp = true; /**< Include timestamp in message headers */
    bool m_includeElapsedTime = false; /**< Include elapsed time in message headers */

};

}

#endif // AROLIBLOGGER_H
