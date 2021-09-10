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
 
#include "arolib/misc/datetime.hpp"

namespace{
std::string double2string(double n){
    std::string s = std::to_string(n);
    std::replace(s.begin(), s.end(), ',', '.');
    return s;
}
}

namespace arolib{


DateTime::DateTime():
    m_time( time(NULL) )
{
    setNow();
}

bool DateTime::fromISO8601(const std::string &dt_iso)
{
    std::tm tm = {};
    std::stringstream ss(dt_iso);
    ss >> std::get_time(&tm, "%Y-%m-%dT%T");
    if (ss.fail())
        return false;

    m_time = mktime(&tm);//mktime changes tm with the Daylight Saving Time (DST)
    m_delta = 0;
    m_us = 0;

    std::string tmp = dt_iso;
    auto ind = tmp.rfind("+");
    if(ind == std::string::npos){
        auto indT = tmp.find("T");
        if(indT != std::string::npos){
            ind = tmp.rfind("-");
            if(ind != std::string::npos && ind < indT)
                ind = std::string::npos;
        }
    }
    if(ind != std::string::npos){
        m_delta = getDeltaTimeZone( tmp.substr(ind) );
        tmp.erase(ind);
        tmp += "Z";
    }

    m_us = extractMicroSeconds(tmp);

    return true;

}

std::string DateTime::toISO8601() const
{
    std::tm *ptm = localtime ( &m_time );

    if(ptm->tm_isdst > 0){//if there is daylight saving, it puts the 'wrong' hour
        auto tmp = m_time - 3600;
        ptm = localtime ( &tmp );
    }

    std::string ret = std::to_string( ptm->tm_year + 1900 ) + "-"
                    + (ptm->tm_mon < 9 ? "0" : "") + std::to_string( ptm->tm_mon + 1 ) + "-"
                    + (ptm->tm_mday < 10 ? "0" : "") + std::to_string( ptm->tm_mday ) + "T"
                    + (ptm->tm_hour < 10 ? "0" : "") + std::to_string( ptm->tm_hour ) + ":"
                    + (ptm->tm_min < 10 ? "0" : "") + std::to_string( ptm->tm_min ) + ":"
                    + (ptm->tm_sec < 10 ? "0" : "") + std::to_string( ptm->tm_sec );
    ret += usTofracSec( m_us );
    ret += getDeltaTimeZone(m_delta);
    return ret;
}

DateTime DateTime::operator+(double sec) const
{
    DateTime ret = *this;
    int iSec = sec;
    ret.m_time += iSec;
    ret.m_us = m_us + (sec-iSec)*1e6;
    return ret;
}

DateTime DateTime::operator-(double sec) const
{
    DateTime ret = *this;
    int iSec = sec;
    ret.m_time -= iSec;
    ret.m_us = m_us - (sec-iSec)*1e6;
    if(ret.m_us < 0){
        ret.m_time -= 1;
        ret.m_us += 1e6;
    }
    return ret;
}

long double DateTime::operator-(const DateTime &from) const
{
    return timeSince(from);
}

long double DateTime::timeSince(const DateTime &from) const
{
    auto time_to = m_time - m_delta;
    auto time_from = from.m_time - from.m_delta;
    long double deltaTime = time_to - time_from;
    long double _us = m_us - from.m_us;
    return deltaTime + _us*1e-6;
}

std::string DateTime::secToTimeStr(double sec)
{

    std::string ret;
    if(sec > 0)
        ret += "+";
    else{
        ret += "-";
        sec *= -1;
    }
    int hh = sec / 3600;
    int mm = ( sec - (hh*3600) )/60;
    if(hh < 10)
        ret += "0";
    ret += std::to_string(hh) + ":";
    if(mm < 10)
        ret += "0";
    ret += std::to_string(mm) + ":";
    sec -= ( hh*3600 + mm*60 );
    if(sec < 10)
        ret += "0";
    ret += std::to_string( (int)sec );
    double us = ( sec - (int)sec ) * 1e6;
    if(us > 1e-6)
        ret += usTofracSec(us);
    return ret;
}

DateTime &DateTime::setNow()
{
    auto now = std::chrono::system_clock::now();
    m_time = std::chrono::system_clock::to_time_t(now);

    m_us = std::chrono::duration_cast<std::chrono::microseconds>(now - std::chrono::system_clock::from_time_t(m_time)).count();

    m_time = time(NULL);
    return *this;
}

const time_t &DateTime::getTime() const
{
    return m_time;
}

std::string DateTime::getTimeStr(bool incFracSec, const std::string& sep) const
{
    std::tm tm = *localtime ( &m_time );

//    if(tm.tm_isdst > 0){//if there is daylight saving, it puts the 'wrong' hour
//        auto tmp = m_time - 3600;
//        tm = *localtime ( &tmp );
//    }

//    std::string ret =  (tm.tm_hour < 10 ? "0" : "") + std::to_string( tm.tm_hour ) + sep
//                     + (tm.tm_min < 10 ? "0" : "") + std::to_string( tm.tm_min ) + sep
//                     + (tm.tm_sec < 10 ? "0" : "") + std::to_string( tm.tm_sec );

    char buffer [80];
    std::string format = "%H" + sep + "%M" + sep + "%S";
    strftime( buffer, sizeof(buffer), format.c_str(), &tm );
    std::string ret(buffer);

    if(incFracSec)
        ret += usTofracSec( m_us );
    return ret;

}

std::string DateTime::getDateStr(const std::string &sep) const
{
    std::tm tm = *localtime ( &m_time );

    if(tm.tm_isdst > 0){//if there is daylight saving, it puts the 'wrong' hour
        auto tmp = m_time - 3600;
        tm = *localtime ( &tmp );
    }

    char buffer [80];
    std::string format = "%Y" + sep + "%m" + sep + "%d";
    strftime( buffer, sizeof(buffer), format.c_str(), &tm );
    std::string ret(buffer);
    return ret;

}

std::string DateTime::getDateTimeStr(bool incFracSec, const std::string &sepDate, const std::string &sepDateTime, const std::string &sepTime) const
{
    return getDateStr(sepDate) + sepDateTime + getTimeStr(incFracSec, sepTime);
}

std::string DateTime::getNowTimeStr(bool incFracSec, const std::string &sep)
{
    return DateTime().getTimeStr(incFracSec, sep);
}

std::string DateTime::getNowDateStr(const std::string &sep)
{
    return DateTime().getDateStr(sep);
}

std::string DateTime::getNowDateTimeStr(bool incFracSec, const std::string &sepDate, const std::string &sepDateTime, const std::string &sepTime)
{
    return DateTime().getDateTimeStr(incFracSec, sepDate, sepDateTime, sepTime);
}

double DateTime::getDeltaTimeZone(std::string ts)
{
    try{
        auto mult = 1;
        if(ts.front() == '-')
            mult = -1;
        auto ind = ts.find(":");
        if(ind != std::string::npos)
            ts.erase(ind,1);
        double hh = std::stoi( ts.substr(1,2) );
        double mm = std::stoi( ts.substr(3,2) );
        return hh * 3600 + mm * 60 * mult;
    }
    catch(...){
        return 0;
    }

}

std::string DateTime::getDeltaTimeZone(double delta_s)
{
   if( std::fabs(delta_s) < 1e-6)
       return "";

   std::string ret;
   if(delta_s > 0)
       ret += "+";
   else{
       ret += "-";
       delta_s *= -1;
   }
   int hh = delta_s / 3600;
   int mm = ( delta_s - (hh*3600) )/60;
   if(hh < 10)
       ret += "0";
   ret += std::to_string(hh) + ":";
   if(mm < 10)
       ret += "0";
   ret += std::to_string(mm);
   return ret;
}

long DateTime::extractMicroSeconds(std::string &ts)
{
    try{
        auto ind = ts.rfind(".");
        if(ind == std::string::npos || ind == ts.size()-1)
            return 0;

        auto indn = ind+1;
        while(indn < ts.size() && isdigit( ts[indn] ) )
            ++indn;


        if(indn == ind+1)
            return 0;

        size_t digits = std::min((size_t)6, indn - ind - 1);
        long us = std::stoi( ts.substr(ind+1,digits) );
        for(size_t i = 0 ; i < digits ; ++i)
            ts.at(ind+1+i) = '0';

        if(digits != 6)
            us *= std::pow(10,6-digits);
        return us;
    }
    catch(...){
        return 0;
    }

}

std::string DateTime::usTofracSec(double us)
{
    if(us < 1e-6)
        return "";
    if(us > 1e7)
        return ".999999";

    std::string ret = std::to_string((int)us);
    while(ret.size() < 6)
        ret = "0" + ret;
    return "." + ret;
}

int DateTime::usToSec(long us, long &residual)
{
    int sec = us * 1e-6;
    residual = us - sec*1e6;
    if(residual < 0){
        sec--;
        residual += 1e6;
    }
    return sec;
}


}
