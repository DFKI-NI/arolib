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
 
#include "arolib/misc/basic_responses.h"


namespace arolib{

AroResp::AroResp(int _errorID, const std::string &_msg):
    errorID(_errorID), msg(_msg)
{

}

bool AroResp::isError() const
{
    return errorID > 0;
}

bool AroResp::isWarning() const
{
    return errorID < 0;
}

bool AroResp::isOK() const
{
    return errorID == 0;
}

AroResp AroResp::ok(const std::string& _msg)
{
    return AroResp(0, _msg);
}

AroResp AroResp::LoggingResp(int errorID, const std::string &msg, const Logger &logger, LogLevel logLevel, const std::string &function)
{
    logger.printOut(logLevel, function, msg);
    return AroResp(errorID, msg);
}

AroResp AroResp::LoggingResp(int errorID, const std::string &msg_print, const std::string &msg_no_print, const Logger &logger, LogLevel logLevel, const std::string &function)
{
    logger.printOut(logLevel, function, msg_print);
    return AroResp(errorID, msg_print+msg_no_print);
}

}
