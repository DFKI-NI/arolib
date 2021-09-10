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
 
#include "arolib/misc/basicconversions.hpp"

namespace arolib {

double string2double(std::string s){
    if( atof("0.5") > 0 )
        return atof(s.c_str());
    int i = s.find_first_of(".");
    if(i == std::string::npos)
        return atof(s.c_str());
    s.at(i) = ',';
    return atof(s.c_str());
}

std::string double2string(double n){
    std::string s = std::to_string(n);
    std::replace(s.begin(), s.end(), ',', '.');
    return s;
}


}//end namespace arolib

