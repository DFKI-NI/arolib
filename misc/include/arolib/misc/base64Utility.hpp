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
 
#ifndef ARO_BASE64UTILITY_HPP
#define ARO_BASE64UTILITY_HPP

#include <cryptopp/base64.h>

#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/algorithm/string.hpp>

namespace arolib {

/**
 * @brief Encode a string in base-64.
 * @param val String to be encoded.
 * @return String encoded in base-64.
 */
std::string base64_encode(const std::string &val);
/**
 * @brief Decode a string from base-64.
 * @param val String in base-64 to be decoded.
 * @return Decodes string
 */
std::string base64_decode(const std::string &val);

/**
 * @brief Encode a string in base-64.
 * @note Code extracted from https://stackoverflow.com/questions/7053538/how-do-i-encode-a-string-to-base64-using-only-boost
 * @warning Seems to be buggy
 * @param val String to be encoded.
 * @return String encoded in base-64.
 */
std::string base64_encode2(const std::string &val);

/**
 * @brief Decode a string from base-64.
 * @note Code extracted from https://stackoverflow.com/questions/7053538/how-do-i-encode-a-string-to-base64-using-only-boost
 * @warning Seems to be buggy
 * @param val String in base-64 to be decoded.
 * @param useEndDelimiter Use end-delimiter?
 * @return Decodes string
 */
std::string base64_decode2(const std::string &val, const bool &useEndDelimiter = false);

}

#endif // ARO_BASE64UTILITY_HPP
