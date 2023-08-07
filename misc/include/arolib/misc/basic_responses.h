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
 
#ifndef AROLIB_BASICRESPONSES_H
#define AROLIB_BASICRESPONSES_H

#include <string>
#include <sstream>
#include "arolib/misc/logger.h"

namespace arolib {


/**
 * @brief Arolib standar response
 */
struct AroResp{
    int errorID = 0; /**< Error ID: 0: OK; >0; error; <0: warning */
    std::string msg = "";  /**< Message */

    /**
     * @brief Default constructor
     */
    AroResp() = default;

    /**
     * @brief Constructor
     * @param _errorID Error ID
     * @param _msg Message
     */
    explicit AroResp(int _errorID, const std::string& _msg);

    /**
     * @brief Constructor
     * @param _errorID Error ID
     * @param _msg Error message segments
     */
    template<typename ... Ts>
    explicit AroResp(int _errorID, const Ts& ... _msg):
        errorID(_errorID)
    {
        std::stringstream ss;
        appendToStream(ss, _msg...);
        msg = ss.str();
    }

    /**
     * @brief Check if the current error ID corresponds to an error
     * @return True if the current error ID corresponds to an error
     */
    bool isError() const;

    /**
     * @brief Check if the current error ID corresponds to a warning
     * @return True if the current error ID corresponds to a warning
     */
    bool isWarning() const;

    /**
     * @brief Check if the current error ID corresponds to "OK"
     * @return True if the current error ID corresponds to "OK"
     */
    bool isOK() const;

    /**
     * @brief Returns a response with error ID corresponding to "OK"
     * @param _msg Response message
     * @return True if the current error ID corresponds to "OK"
     */
    static AroResp ok(const std::string& _msg = "OK");

    /**
     * @brief Prints the message with the given logger and returns the coresponding AroResp
     * @param errorID Error ID
     * @param msg Message
     * @param logger Logger
     * @param logLevel Log level of the message
     * @param function Function from where the message is printed, to be added in the printed message's header. Disregarded id empty-string
     */
    static AroResp LoggingResp(int errorID,
                               const std::string& msg,
                               const Logger& logger,
                               LogLevel logLevel,
                               const std::string& function);

    /**
     * @brief Prints the message with the given logger and returns the coresponding AroResp
     * @param errorID Error ID
     * @param msg_print Initial part of the message, which will be printed
     * @param msg_no_print Last part of the message, which will not be printed
     * @param logger Logger
     * @param logLevel Log level of the message
     * @param function Function from where the message is printed, to be added in the printed message's header. Disregarded id empty-string
     */
    static AroResp LoggingResp(int errorID,
                               const std::string& msg_print,
                               const std::string& msg_no_print,
                               const Logger& logger,
                               LogLevel logLevel,
                               const std::string& function);

private:

    /**
     * @brief Append the string value of an item to a stringstream
     * @param item item
     */
    template<typename T>
    void appendToStream(std::stringstream &ss, const T& item){
        ss << item;
    }

    /**
     * @brief Append the string values of a set of items to a stringstream
     * @param item First item
     * @param item Rest of the items
     */
    template<typename T, typename ... Ts>
    void appendToStream(std::stringstream &ss, const T& item, const Ts& ... items ){
        ss << item;
        appendToStream(ss, items...);
    }
};

}

#endif // AROLIB_BASICRESPONSES_H
