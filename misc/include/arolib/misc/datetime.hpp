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
 
#ifndef ARO_DATETIME_HPP
#define ARO_DATETIME_HPP

#include <ctime>
#include <string>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <cctype>
#include <algorithm>
#include <iostream>
#include <chrono>

namespace arolib {


/**
 * @brief Date-time class
 */
class DateTime
{
public:
    /**
     * @brief Default constructor
     *
     * Initializes the datetime in 'now'
     */
    DateTime();


    /**
     * @brief Set the datetime from an ISO8601 formatter string
     *
     * @param dt_iso Datatime as ISO8601 formatter string
     * @return True on success
     */
    bool fromISO8601(const std::string& dt_iso);

    /**
     * @brief Get the datetime as an ISO8601 formatter string
     *
     * @return Datetime as an ISO8601 formatter string
     */
    std::string toISO8601() const;


    /**
     * @brief operator+ (adds seconds to a Datetime)
     *
     * @return Result Datetime
     */
    DateTime operator+(double sec) const;

    /**
     * @brief operator- (subtracts seconds to a Datetime)
     *
     * @return Result Datetime
     */
    DateTime operator-(double sec) const;


    /**
     * @brief operator- (get the seconds between this Datetime and another datetime)
     *
     * @param from Other datetime
     * @return Seconds between this Datetime and another datetime
     */
    long double operator-(const DateTime &from) const;

    /**
     * @brief Get the seconds between this Datetime and another datetime
     *
     * @param from Other datetime
     * @return Seconds between this Datetime and another datetime
     */
    long double timeSince(const DateTime &from) const;

    /**
     * @brief Convert seconds to a formatted string
     *
     * @param sec Seconds
     * @return Formatted string
     */
    static std::string secToTimeStr(double sec);

    /**
     * @brief Set this to 'now'
     *
     * @return this
     */
    DateTime& setNow();

    /**
     * @brief Get the time
     *
     * @return Time
     */
    const time_t& getTime() const;

    /**
     * @brief Get the time as string
     *
     * e.g., "HH:mm:ss.fffff"
     * @param incFracSec Include fractions of seconds?
     * @param sep Separator
     * @return Time as string
     */
    std::string getTimeStr(bool incFracSec = false, const std::string &sep = ":") const;


    /**
     * @brief Get the date as string
     *
     * e.g., "yyyy-MM-dd"
     * @param sep Separator
     * @return Time as string
     */
    std::string getDateStr(const std::string &sep = "-") const;

    /**
     * @brief Get the date-time as string
     *
     * e.g., "yyyy-MM-dd HH:mm:ss.fffff"
     * @param incFracSec Include fractions of seconds?
     * @param sepDate Separator for the date values
     * @param sepDateTime Separator between date and time
     * @param ssepTimeep Separator for the time values
     * @return Date-time as string
     */
    std::string getDateTimeStr(bool incFracSec = false, const std::string &sepDate = "-", const std::string &sepDateTime = " ", const std::string &sepTime = ":") const;

    /**
     * @brief Get the current (now) time as string
     *
     * e.g., "HH:mm:ss.fffff"
     * @param incFracSec Include fractions of seconds?
     * @param sep Separator
     * @return Time as string
     */
    static std::string getNowTimeStr(bool incFracSec = false, const std::string &sep = ":");

    /**
     * @brief Get the current (now) date as string
     *
     * e.g., "yyyy-MM-dd"
     * @param sep Separator
     * @return Time as string
     */
    static std::string getNowDateStr(const std::string &sep = ":");

    /**
     * @brief Get the current (now) date-time as string
     *
     * e.g., "yyyy-MM-dd HH:mm:ss.fffff"
     * @param incFracSec Include fractions of seconds?
     * @param sepDate Separator for the date values
     * @param sepDateTime Separator between date and time
     * @param ssepTimeep Separator for the time values
     * @return Date-time as string
     */
    static std::string getNowDateTimeStr(bool incFracSec = false, const std::string &sepDate = "-", const std::string &sepDateTime = " ", const std::string &sepTime = ":");

protected:

    /**
     * @brief Get the amount of seconds for a given "delta timezone" string
     * @param tz "delta timezone" string
     * @return Seconds
     */
    static double getDeltaTimeZone(std::string ts);

    /**
     * @brief Get the "delta timezone" string for a given amount of seconds
     * @param delta_s Seconds
     * @return "delta timezone" string
     */
    static std::string getDeltaTimeZone(double delta_s);

    /**
     * @brief Extract the microseconds from a time-string
     * @param [in/out] ts
     * @return Microseconds
     */
    static long extractMicroSeconds(std::string &ts);


    /**
     * @brief Convert microseconts to a string of fractions of a second
     * @param us Microseconds
     * @return String of fractions of a second
     */
    static std::string usTofracSec(double us);

    /**
     * @brief Convert microseconts to seconds (extracting the residual)
     * @param us Microseconds
     * @param [out] residual Residual microseconds
     * @return Seconds
     */
    static int usToSec(long us, long& residual);

protected:
    time_t m_time; /**< Time */
    double m_delta = 0; /**< Delta time (in seconds) */
    long m_us = 0; /**< Microseconds */
};

}


#endif // ARO_DATETIME_HPP
