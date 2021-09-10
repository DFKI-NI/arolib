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
 
#include "arolib/misc/base64Utility.hpp"

namespace arolib{


std::string base64_encode(const std::string &val)
{
    if(val.empty())
        return "";

    std::string ret;

    CryptoPP::Base64Encoder encoder;
    encoder.Put( (byte*)val.data(), val.size() );
    encoder.MessageEnd();

    CryptoPP::word64 size = encoder.MaxRetrievable();
    if(size && size <= SIZE_MAX)
    {
        ret.resize(size);
        encoder.Get((byte*)&ret[0], ret.size());
    }
    return ret;
}

std::string base64_decode(const std::string &val)
{
    if(val.empty())
        return "";

    std::string ret;

    CryptoPP::Base64Decoder decoder;
    decoder.Put( (byte*)val.data(), val.size() );
    decoder.MessageEnd();

    CryptoPP::word64 size = decoder.MaxRetrievable();
    if(size && size <= SIZE_MAX)
    {
        ret.resize(size);
        decoder.Get((byte*)&ret[0], ret.size());
    }
    return ret;
}

std::string base64_encode2(const std::string &val) {
    if(val.empty())
        return "";
    using namespace boost::archive::iterators;
    using It = base64_from_binary<transform_width<std::string::const_iterator, 6, 8>>;
    auto tmp = std::string(It(std::begin(val)), It(std::end(val)));
    return tmp.append((3 - val.size() % 3) % 3, '=');
}

std::string base64_decode2(const std::string &val, const bool & useEndDelimiter) {
    if(val.empty())
        return "";
    using namespace boost::archive::iterators;
    using It = transform_width<binary_from_base64<std::string::const_iterator>, 8, 6>;
    return boost::algorithm::trim_right_copy_if( std::string( It(std::begin(val)), It(std::end(val)) ),
                                                 [&useEndDelimiter](char c) {
        return (useEndDelimiter && c == '\0');
    });
}

}
