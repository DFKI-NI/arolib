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
 
#ifndef AROLIB_PLANNINGEXCEPTION_HPP
#define AROLIB_PLANNINGEXCEPTION_HPP

#include <stdexcept>


namespace arolib{

/**
 * @brief Arolib planning exception
 */
class PlanningException : public std::runtime_error{

public:

    /**
     * @brief Constructor
     * @param msg Exception message/description
     */
    explicit PlanningException(const std::string &msg) : std::runtime_error(""), error_msg(msg) {}

    /**
     * @brief Get exception message/description
     * @return Exception message/description
     */
    virtual const char* what() const throw() {
        return error_msg.c_str();
    }

private:
    std::string error_msg;/**< Arolib planning exception message/description */

};

}

#endif // AROLIB_PLANNINGEXCEPTION_HPP
