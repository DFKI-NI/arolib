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
 
#include "arolib/types/machinedynamicinfo.hpp"

namespace arolib {

bool MachineDynamicInfo::isValid() const{
    return position.isValid() && timestamp >= -1e-9;
}

bool MachineDynamicInfo::isValid(const std::map<MachineId_t, MachineDynamicInfo> &mdi)
{
    for(const auto& mdi_it : mdi)
        if(!mdi_it.second.isValid()) return false;
    return true;
}

bool MachineDynamicInfo::isValid(const std::vector<MachineDynamicInfo> &mdi)
{
    for(const auto& info : mdi)
        if(!info.isValid()) return false;
    return true;
}

}
