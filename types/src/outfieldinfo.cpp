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
 
#include "arolib/types/outfieldinfo.hpp"

namespace arolib {


const OutFieldInfo::MachineId_t OutFieldInfo::AllMachines = -255;
const ResourcePointId_t OutFieldInfo::AllResourcePoints = -255;
const FieldAccessPointId_t OutFieldInfo::AllFieldAccessPoints = -255;

OutFieldInfo::TravelCosts::TravelCosts():
    time(0),
    time_per_kg(0),
    distance(0){

}

OutFieldInfo::TravelCosts::TravelCosts(double _time, double _time_per_kg, double _distance):
    time(_time),
    time_per_kg(_time_per_kg),
    distance(_distance){

}


OutFieldInfo::UnloadingCosts::UnloadingCosts():
    time(0),
    time_per_kg(0){

}

OutFieldInfo::UnloadingCosts::UnloadingCosts(double _time, double _time_per_kg):
    time(_time),
    time_per_kg(_time_per_kg){

}


OutFieldInfo::MachineBunkerState OutFieldInfo::intToMachineBunkerState(int value){
    if(value == MachineBunkerState::MACHINE_EMPTY)
        return MachineBunkerState::MACHINE_EMPTY;
    else if(value == MachineBunkerState::MACHINE_LOADED)
        return MachineBunkerState::MACHINE_LOADED;
    else if(value == MachineBunkerState::ALL_MACHINE_STATES)
        return MachineBunkerState::ALL_MACHINE_STATES;

    throw std::invalid_argument( "The given value does not correspond to any OutFieldInfo::MachineBunkerState" );
}

OutFieldInfo::OutFieldInfo()
{

}

void OutFieldInfo::clearAll()
{
    clearFAP2RP();
    clearRP2FAP();
    clearFAP2FAP();
    clearUnloadingCosts();
    clearArrivalCosts();
}

void OutFieldInfo::clearFAP2RP()
{
    m_mapAccessPoint2ResourcePoint.clear();
}

void OutFieldInfo::clearRP2FAP()
{
    m_mapResourcePoint2AccessPoint.clear();
}

void OutFieldInfo::clearFAP2FAP()
{
    m_mapAccessPoint2AccessPoint.clear();
}

void OutFieldInfo::clearUnloadingCosts()
{
    m_mapUnloadingCosts.clear();
}

void OutFieldInfo::clearArrivalCosts()
{
    m_mapArrivalCosts.clear();
}

void OutFieldInfo::fromMatrix_FAP2RP(const std::vector<OutFieldInfo::TravelData> &matrixData)
{
    clearFAP2RP();
    for(const auto &item : matrixData){
        m_mapAccessPoint2ResourcePoint[item.fieldAccessPointId]
                                      [item.resourcePointId]
                                      [item.machineId]
                                      [item.machineBunkerState] = item.travelCosts;
    }
}

void OutFieldInfo::fromMatrix_RP2FAP(const std::vector<OutFieldInfo::TravelData> &matrixData)
{
    clearRP2FAP();
    for(const auto &item : matrixData){
        m_mapResourcePoint2AccessPoint[item.resourcePointId]
                                      [item.fieldAccessPointId]
                                      [item.machineId]
                                      [item.machineBunkerState] = item.travelCosts;
    }
}

void OutFieldInfo::fromMatrix_FAP2FAP(const std::vector<OutFieldInfo::TravelData2> &matrixData, bool bidirecctional)
{
    clearFAP2FAP();
    for(const auto &item : matrixData){
        m_mapAccessPoint2AccessPoint[item.fap_id_from]
                                    [item.fap_id_to]
                                    [item.machineId]
                                    [item.machineBunkerState] = item.travelCosts;

        if(bidirecctional)
            m_mapAccessPoint2AccessPoint[item.fap_id_to]
                                        [item.fap_id_from]
                                        [item.machineId]
                                        [item.machineBunkerState] = item.travelCosts;
    }

}

void OutFieldInfo::fromMatrix_unloadingCosts(const std::vector<OutFieldInfo::UnloadingData> &matrixData)
{
    clearUnloadingCosts();
    for(const auto &item : matrixData){
        m_mapUnloadingCosts[item.resourcePointId]
                           [item.machineId] = item.unloadingCosts;
    }
}

void OutFieldInfo::fromMatrix_arrivalCosts(const std::vector<OutFieldInfo::ArrivalData> &matrixData)
{
    clearArrivalCosts();
    for(const auto &item : matrixData){
        m_mapArrivalCosts[item.fieldAccessPointId]
                         [item.machineId]
                         [item.machineBunkerState] = item.arrivalCosts;
    }
}

void OutFieldInfo::add_FAP2RP(const OutFieldInfo::TravelData &travelData)
{
    m_mapAccessPoint2ResourcePoint[travelData.fieldAccessPointId]
                                  [travelData.resourcePointId]
                                  [travelData.machineId]
                                  [travelData.machineBunkerState] = travelData.travelCosts;
}

void OutFieldInfo::add_RP2FAP(const OutFieldInfo::TravelData &travelData)
{
    m_mapResourcePoint2AccessPoint[travelData.resourcePointId]
                                  [travelData.fieldAccessPointId]
                                  [travelData.machineId]
                                  [travelData.machineBunkerState] = travelData.travelCosts;

}

void OutFieldInfo::add_FAP2FAP(const OutFieldInfo::TravelData2 &travelData, bool bidirecctional)
{
    m_mapAccessPoint2AccessPoint[travelData.fap_id_from]
                                [travelData.fap_id_to]
                                [travelData.machineId]
                                [travelData.machineBunkerState] = travelData.travelCosts;
    if(bidirecctional)
        m_mapAccessPoint2AccessPoint[travelData.fap_id_to]
                                    [travelData.fap_id_from]
                                    [travelData.machineId]
                                    [travelData.machineBunkerState] = travelData.travelCosts;
}

void OutFieldInfo::add_unloadingCosts(const UnloadingData &unloadingData)
{
    m_mapUnloadingCosts[unloadingData.resourcePointId]
                       [unloadingData.machineId] = unloadingData.unloadingCosts;
}

void OutFieldInfo::add_arrivalCosts(const arolib::OutFieldInfo::ArrivalData &arrivalData)
{
    m_mapArrivalCosts[arrivalData.fieldAccessPointId]
                     [arrivalData.machineId]
                     [arrivalData.machineBunkerState] = arrivalData.arrivalCosts;
}

size_t OutFieldInfo::size_FAP2RP(FieldAccessPointId_t fieldAccessPointId,
                                  ResourcePointId_t resourcePointId,
                                  OutFieldInfo::MachineId_t machineId) const
{
    if(fieldAccessPointId == AllFieldAccessPoints)
        return m_mapAccessPoint2ResourcePoint.size();
    auto fieldAccessPointId_it = m_mapAccessPoint2ResourcePoint.find(fieldAccessPointId);
    if( fieldAccessPointId_it == m_mapAccessPoint2ResourcePoint.end() ){//no specific data --> search for default
        fieldAccessPointId_it = m_mapAccessPoint2ResourcePoint.find(AllFieldAccessPoints);
        if( fieldAccessPointId_it == m_mapAccessPoint2ResourcePoint.end() )
            return 0;
    }

    if(resourcePointId == AllResourcePoints)
        return fieldAccessPointId_it->second.size();
    auto resourcePointId_it = fieldAccessPointId_it->second.find(resourcePointId);
    if( resourcePointId_it == fieldAccessPointId_it->second.end() ){//no specific data --> search for default
        resourcePointId_it = fieldAccessPointId_it->second.find(AllResourcePoints);
        if( resourcePointId_it == fieldAccessPointId_it->second.end() )
            return 0;
    }

    if(machineId == AllMachines)
        return resourcePointId_it->second.size();
    auto machineId_it = resourcePointId_it->second.find(machineId);
    if( machineId_it == resourcePointId_it->second.end() ){//no specific data --> search for default
        machineId_it = resourcePointId_it->second.find(AllMachines);
        if( machineId_it == resourcePointId_it->second.end() )
            return 0;
    }

    return machineId_it->second.size();
}

size_t OutFieldInfo::size_RP2FAP(ResourcePointId_t resourcePointId,
                                  FieldAccessPointId_t fieldAccessPointId,
                                  OutFieldInfo::MachineId_t machineId) const
{
    if(resourcePointId == AllResourcePoints)
        return m_mapResourcePoint2AccessPoint.size();
    auto resourcePointId_it = m_mapResourcePoint2AccessPoint.find(resourcePointId);
    if( resourcePointId_it == m_mapResourcePoint2AccessPoint.end() ){//no specific data --> search for default
        resourcePointId_it = m_mapResourcePoint2AccessPoint.find(AllResourcePoints);
        if( resourcePointId_it == m_mapResourcePoint2AccessPoint.end() )
            return 0;
    }

    if(fieldAccessPointId == AllFieldAccessPoints)
        return resourcePointId_it->second.size();
    auto fieldAccessPointId_it = resourcePointId_it->second.find(fieldAccessPointId);
    if( fieldAccessPointId_it == resourcePointId_it->second.end() ){//no specific data --> search for default
        fieldAccessPointId_it = resourcePointId_it->second.find(AllFieldAccessPoints);
        if( fieldAccessPointId_it == resourcePointId_it->second.end() )
            return 0;
    }

    if(machineId == AllMachines)
        return fieldAccessPointId_it->second.size();
    auto machineId_it = fieldAccessPointId_it->second.find(machineId);
    if( machineId_it == fieldAccessPointId_it->second.end()){//no specific data --> search for default
        machineId_it = fieldAccessPointId_it->second.find(AllMachines);
        if( machineId_it == fieldAccessPointId_it->second.end())
            return 0;
    }

    return machineId_it->second.size();
}

size_t OutFieldInfo::size_FAP2FAP(FieldAccessPointId_t fap_id_from, FieldAccessPointId_t fap_id_to, OutFieldInfo::MachineId_t machineId) const
{
    if(fap_id_from == AllFieldAccessPoints)
        return m_mapAccessPoint2AccessPoint.size();
    auto fapid1_it = m_mapAccessPoint2AccessPoint.find(fap_id_from);
    if( fapid1_it == m_mapAccessPoint2AccessPoint.end() ){//no specific data --> search for default
        fapid1_it = m_mapAccessPoint2AccessPoint.find(AllFieldAccessPoints);
        if( fapid1_it == m_mapAccessPoint2AccessPoint.end() )
            return 0;
    }

    if(fap_id_to == AllFieldAccessPoints)
        return fapid1_it->second.size();
    auto fapid2_it = fapid1_it->second.find(fap_id_to);
    if( fapid2_it == fapid1_it->second.end() ){//no specific data --> search for default
        fapid2_it = fapid1_it->second.find(AllFieldAccessPoints);
        if( fapid2_it == fapid1_it->second.end() )
            return 0;
    }

    if(machineId == AllMachines)
        return fapid2_it->second.size();
    auto machineId_it = fapid2_it->second.find(machineId);
    if( machineId_it == fapid2_it->second.end()){//no specific data --> search for default
        machineId_it = fapid2_it->second.find(AllMachines);
        if( machineId_it == fapid2_it->second.end())
            return 0;
    }

    return machineId_it->second.size();

}

size_t OutFieldInfo::size_unloadingCosts(ResourcePointId_t resourcePointId,
                                          MachineId_t machineId) const
{
    if(resourcePointId == AllResourcePoints)
        return m_mapUnloadingCosts.size();
    auto resourcePointId_it = m_mapUnloadingCosts.find(resourcePointId);
    if( resourcePointId_it == m_mapUnloadingCosts.end() ){//no specific data --> search for default
        resourcePointId_it = m_mapUnloadingCosts.find(AllResourcePoints);
        if( resourcePointId_it == m_mapUnloadingCosts.end() )
            return 0;
    }

    if(machineId == AllMachines)
        return resourcePointId_it->second.size();
    auto machineId_it = resourcePointId_it->second.find(machineId);
    if( machineId_it == resourcePointId_it->second.end() ){//no specific data --> search for default
        machineId_it = resourcePointId_it->second.find(AllMachines);
        if( machineId_it == resourcePointId_it->second.end() )
            return 0;
    }

    return 1;
}

size_t OutFieldInfo::size_arrivalCosts(FieldAccessPointId_t fieldAccessPointId,
                                       OutFieldInfo::MachineId_t machineId) const
{
    if(fieldAccessPointId == AllFieldAccessPoints)
        return m_mapArrivalCosts.size();
    auto fieldAccessPointId_it = m_mapArrivalCosts.find(fieldAccessPointId);
    if( fieldAccessPointId_it == m_mapArrivalCosts.end() ){//no specific data --> search for default
        fieldAccessPointId_it = m_mapArrivalCosts.find(AllFieldAccessPoints);
        if( fieldAccessPointId_it == m_mapArrivalCosts.end() )
            return 0;
    }

    if(machineId == AllMachines)
        return fieldAccessPointId_it->second.size();
    auto machineId_it = fieldAccessPointId_it->second.find(machineId);
    if( machineId_it == fieldAccessPointId_it->second.end()){//no specific data --> search for default
        machineId_it = fieldAccessPointId_it->second.find(AllMachines);
        if( machineId_it == fieldAccessPointId_it->second.end())
            return 0;
    }

    return machineId_it->second.size();
}

bool OutFieldInfo::resourcePointBidirectionalConnectionExists(ResourcePointId_t resourcePointId,
                                                               const std::vector<FieldAccessPoint>& accessPoints) const
{
    if(accessPoints.empty())
        return false;

    bool found = false;

    //check if there exists a route from the RP to any of the input FAPs
    for(int allRPIds = 0 ; allRPIds < 2 ; ++allRPIds){//allRPIds = 0 -> search for specific data ; allRPIds = 1 --> search for default
        ResourcePointId_t RP_id = resourcePointId;
        if(allRPIds > 0)//allRPIds = 1 --> search for default
            RP_id = AllResourcePoints;

        auto resourcePointId_it = m_mapResourcePoint2AccessPoint.find(RP_id);
        if(resourcePointId_it == m_mapResourcePoint2AccessPoint.end())
            continue;

        //chech for travel data from any of the FAPs to the RP
        for(size_t FAP_ind = 0 ; FAP_ind < accessPoints.size() + 1 ; ++FAP_ind){
            FieldAccessPointId_t FAP_id = AllFieldAccessPoints;
            if(FAP_ind < accessPoints.size())
                FAP_id = accessPoints.at(FAP_ind).id;

            auto fieldAccessPointId_it = resourcePointId_it->second.find(FAP_id);
            if(fieldAccessPointId_it == resourcePointId_it->second.end())
                continue;

            for( auto &machineId_it : fieldAccessPointId_it->second ){
                if(!machineId_it.second.empty()){
                    found = true;
                    break;
                }
            }
            if(found)
                break;
        }
        if(found || RP_id==AllResourcePoints)
            break;
    }
    if(!found)
        return false;

    //check if there exists a route from any of the input FAPs to the RP
    for(size_t FAP_ind = 0 ; FAP_ind < accessPoints.size() + 1 ; ++FAP_ind){
        FieldAccessPointId_t FAP_id = AllFieldAccessPoints;
        if(FAP_ind < accessPoints.size())
            FAP_id = accessPoints.at(FAP_ind).id;

        auto fieldAccessPointId_it = m_mapAccessPoint2ResourcePoint.find(FAP_id);
        if(fieldAccessPointId_it == m_mapAccessPoint2ResourcePoint.end())
            continue;

        for(int allRPIds = 0 ; allRPIds < 2 ; ++allRPIds){//allRPIds = 0 -> search for specific data ; allRPIds = 1 --> search for default
            ResourcePointId_t RP_id = resourcePointId;
            if(allRPIds > 0)//allRPIds = 1 --> search for default
                RP_id = AllResourcePoints;

            auto resourcePointId_it = fieldAccessPointId_it->second.find(RP_id);
            if( resourcePointId_it != fieldAccessPointId_it->second.end() ){
                for( auto &machineId_it : resourcePointId_it->second )
                    if(!machineId_it.second.empty())
                        return true;
            }
            if(RP_id==AllResourcePoints)
                break;
        }
    }

    return false;
}

bool OutFieldInfo::getTravelCost_FAP2RP(FieldAccessPointId_t fieldAccessPointId,
                                         ResourcePointId_t resourcePointId,
                                         OutFieldInfo::MachineId_t machineId,
                                         OutFieldInfo::MachineBunkerState machineBunkerState,
                                         OutFieldInfo::TravelCosts &travelCosts) const
{
    for(size_t allFAPIds = 0 ; allFAPIds < (fieldAccessPointId==AllFieldAccessPoints?1:2) ; ++allFAPIds){//allFAPIds = 0 -> search for specific data ; allFAPIds = 1 --> search for default
        FieldAccessPointId_t FAP_id = fieldAccessPointId;
        if(allFAPIds > 0)//allFAPIds = 1 --> search for default
            FAP_id = AllFieldAccessPoints;
        auto it_fieldAccessPointId = m_mapAccessPoint2ResourcePoint.find(FAP_id);
        if( it_fieldAccessPointId == m_mapAccessPoint2ResourcePoint.end() )
            continue;

        for(int allRPIds = 0 ; allRPIds < (resourcePointId==AllResourcePoints?1:2) ; ++allRPIds){//allRPIds = 0 -> search for specific data ; allRPIds = 1 --> search for default
            ResourcePointId_t RP_id = resourcePointId;
            if(allRPIds > 0)//allRPIds = 1 --> search for default
                RP_id = AllResourcePoints;
            auto it_resourcePointId = it_fieldAccessPointId->second.find(RP_id);
            if(it_resourcePointId == it_fieldAccessPointId->second.end())
                continue;
            if( getTravelCost( it_resourcePointId->second,
                               machineId,
                               machineBunkerState,
                               travelCosts ) )
                return true;
        }
    }
    return false;
}

bool OutFieldInfo::getTravelCost_FAP2RP(FieldAccessPointId_t fieldAccessPointId, ResourcePointId_t resourcePointId, OutFieldInfo::MachineId_t machineId, OutFieldInfo::MapMachineStateTravelCosts_t &travelCosts) const
{

    for(size_t allFAPIds = 0 ; allFAPIds < (fieldAccessPointId==AllFieldAccessPoints?1:2) ; ++allFAPIds){//allFAPIds = 0 -> search for specific data ; allFAPIds = 1 --> search for default
        FieldAccessPointId_t FAP_id = fieldAccessPointId;
        if(allFAPIds > 0)//allFAPIds = 1 --> search for default
            FAP_id = AllFieldAccessPoints;
        auto it_fieldAccessPointId = m_mapAccessPoint2ResourcePoint.find(FAP_id);
        if( it_fieldAccessPointId == m_mapAccessPoint2ResourcePoint.end() )
            continue;

        for(int allRPIds = 0 ; allRPIds < (resourcePointId==AllResourcePoints?1:2) ; ++allRPIds){//allRPIds = 0 -> search for specific data ; allRPIds = 1 --> search for default
            ResourcePointId_t RP_id = resourcePointId;
            if(allRPIds > 0)//allRPIds = 1 --> search for default
                RP_id = AllResourcePoints;
            auto it_resourcePointId = it_fieldAccessPointId->second.find(RP_id);
            if(it_resourcePointId == it_fieldAccessPointId->second.end())
                continue;
            if( getTravelCost( it_resourcePointId->second,
                               machineId,
                               travelCosts ) )
                return true;
        }
    }
    return false;
}

bool OutFieldInfo::getTravelCost_FAP2RP(FieldAccessPointId_t fieldAccessPointId,
                                         ResourcePointId_t resourcePointId,
                                         MapMachineTravelCosts_t &travelCostsMap) const
{
    travelCostsMap.clear();

    for(size_t allFAPIds = 0 ; allFAPIds < (fieldAccessPointId==AllFieldAccessPoints?1:2) ; ++allFAPIds){//allFAPIds = 0 -> search for specific data ; allFAPIds = 1 --> search for default
        FieldAccessPointId_t FAP_id = fieldAccessPointId;
        if(allFAPIds > 0)//allFAPIds = 1 --> search for default
            FAP_id = AllFieldAccessPoints;
        auto it_fieldAccessPointId = m_mapAccessPoint2ResourcePoint.find(FAP_id);
        if( it_fieldAccessPointId == m_mapAccessPoint2ResourcePoint.end() )
            continue;

        for(int allRPIds = 0 ; allRPIds < (resourcePointId==AllResourcePoints?1:2) ; ++allRPIds){//allRPIds = 0 -> search for specific data ; allRPIds = 1 --> search for default
            ResourcePointId_t RP_id = resourcePointId;
            if(allRPIds > 0)//allRPIds = 1 --> search for default
                RP_id = AllResourcePoints;
            auto it_resourcePointId = it_fieldAccessPointId->second.find(RP_id);
            if(it_resourcePointId == it_fieldAccessPointId->second.end())
                continue;

            travelCostsMap = it_resourcePointId->second;
            return true;
        }
    }
    return false;
}

bool OutFieldInfo::getTravelCost_RP2FAP(ResourcePointId_t resourcePointId,
                                         FieldAccessPointId_t fieldAccessPointId,
                                         OutFieldInfo::MachineId_t machineId,
                                         OutFieldInfo::MachineBunkerState machineBunkerState,
                                         OutFieldInfo::TravelCosts &travelCosts) const
{

    for(int allRPIds = 0 ; allRPIds < (resourcePointId==AllResourcePoints?1:2) ; ++allRPIds){//allRPIds = 0 -> search for specific data ; allRPIds = 1 --> search for default
        ResourcePointId_t RP_id = resourcePointId;
        if(allRPIds > 0)//allRPIds = 1 --> search for default
            RP_id = AllResourcePoints;
        auto it_resourcePointId = m_mapResourcePoint2AccessPoint.find(RP_id);
        if( it_resourcePointId == m_mapResourcePoint2AccessPoint.end() )
            continue;

        for(size_t allFAPIds = 0 ; allFAPIds < (fieldAccessPointId==AllFieldAccessPoints?1:2) ; ++allFAPIds){//allFAPIds = 0 -> search for specific data ; allFAPIds = 1 --> search for default
            FieldAccessPointId_t FAP_id = fieldAccessPointId;
            if(allFAPIds > 0)//allFAPIds = 1 --> search for default
                FAP_id = AllFieldAccessPoints;
            auto it_fieldAccessPointId = it_resourcePointId->second.find(FAP_id);
            if(it_fieldAccessPointId == it_resourcePointId->second.end())
                continue;
            if( getTravelCost( it_fieldAccessPointId->second,
                               machineId,
                               machineBunkerState,
                               travelCosts ) )
                return true;
        }
    }
    return false;
}

bool OutFieldInfo::getTravelCost_RP2FAP(ResourcePointId_t resourcePointId,
                                         FieldAccessPointId_t fieldAccessPointId,
                                         OutFieldInfo::MachineId_t machineId,
                                         MapMachineStateTravelCosts_t &travelCosts) const
{

    for(int allRPIds = 0 ; allRPIds < (resourcePointId==AllResourcePoints?1:2) ; ++allRPIds){//allRPIds = 0 -> search for specific data ; allRPIds = 1 --> search for default
        ResourcePointId_t RP_id = resourcePointId;
        if(allRPIds > 0)//allRPIds = 1 --> search for default
            RP_id = AllResourcePoints;
        auto it_resourcePointId = m_mapResourcePoint2AccessPoint.find(RP_id);
        if( it_resourcePointId == m_mapResourcePoint2AccessPoint.end() )
            continue;

        for(size_t allFAPIds = 0 ; allFAPIds < (fieldAccessPointId==AllFieldAccessPoints?1:2) ; ++allFAPIds){//allFAPIds = 0 -> search for specific data ; allFAPIds = 1 --> search for default
            FieldAccessPointId_t FAP_id = fieldAccessPointId;
            if(allFAPIds > 0)//allFAPIds = 1 --> search for default
                FAP_id = AllFieldAccessPoints;
            auto it_fieldAccessPointId = it_resourcePointId->second.find(FAP_id);
            if(it_fieldAccessPointId == it_resourcePointId->second.end())
                continue;
            if( getTravelCost( it_fieldAccessPointId->second,
                               machineId,
                               travelCosts ) )
                return true;
        }
    }
    return false;
}

bool OutFieldInfo::getTravelCost_RP2FAP(ResourcePointId_t resourcePointId,
                                         FieldAccessPointId_t fieldAccessPointId,
                                         MapMachineTravelCosts_t &travelCostsMap) const
{
    travelCostsMap.clear();

    for(int allRPIds = 0 ; allRPIds < (resourcePointId==AllResourcePoints?1:2) ; ++allRPIds){//allRPIds = 0 -> search for specific data ; allRPIds = 1 --> search for default
        ResourcePointId_t RP_id = resourcePointId;
        if(allRPIds > 0)//allRPIds = 1 --> search for default
            RP_id = AllResourcePoints;
        auto it_resourcePointId = m_mapResourcePoint2AccessPoint.find(RP_id);
        if( it_resourcePointId == m_mapResourcePoint2AccessPoint.end() )
            continue;

        for(size_t allFAPIds = 0 ; allFAPIds < (fieldAccessPointId==AllFieldAccessPoints?1:2) ; ++allFAPIds){//allFAPIds = 0 -> search for specific data ; allFAPIds = 1 --> search for default
            FieldAccessPointId_t FAP_id = fieldAccessPointId;
            if(allFAPIds > 0)//allFAPIds = 1 --> search for default
                FAP_id = AllFieldAccessPoints;
            auto it_fieldAccessPointId = it_resourcePointId->second.find(FAP_id);
            if(it_fieldAccessPointId == it_resourcePointId->second.end())
                continue;

            travelCostsMap = it_fieldAccessPointId->second;
            return true;
        }
    }
    return false;

}

bool OutFieldInfo::getTravelCost_FAP2FAP(FieldAccessPointId_t fap_id_from, FieldAccessPointId_t fap_id_to, OutFieldInfo::MachineId_t machineId, OutFieldInfo::MachineBunkerState machineBunkerState, OutFieldInfo::TravelCosts &travelCosts) const
{
    for(int allFAPIds1 = 0 ; allFAPIds1 < (fap_id_from==AllFieldAccessPoints?1:2) ; ++allFAPIds1){//allFAPIds1 = 0 -> search for specific data ; allFAPIds1 = 1 --> search for default
        FieldAccessPointId_t fap_id_1 = fap_id_from;
        if(allFAPIds1 > 0)//allFAPIds1 = 1 --> search for default
            fap_id_1 = AllFieldAccessPoints;
        auto it_fapid_1 = m_mapAccessPoint2AccessPoint.find(fap_id_1);
        if( it_fapid_1 == m_mapAccessPoint2AccessPoint.end() )
            continue;

        for(size_t allFAPIds2 = 0 ; allFAPIds2 < (fap_id_to==AllFieldAccessPoints?1:2) ; ++allFAPIds2){//allFAPIds2 = 0 -> search for specific data ; allFAPIds2 = 1 --> search for default
            FieldAccessPointId_t FAP_id = fap_id_to;
            if(allFAPIds2 > 0)//allFAPIds = 1 --> search for default
                FAP_id = AllFieldAccessPoints;
            auto it_fapid_2 = it_fapid_1->second.find(FAP_id);
            if(it_fapid_2 == it_fapid_1->second.end())
                continue;
            if( getTravelCost( it_fapid_2->second,
                               machineId,
                               machineBunkerState,
                               travelCosts ) )
                return true;
        }
    }
    return false;

}

bool OutFieldInfo::getTravelCost_FAP2FAP(FieldAccessPointId_t fap_id_from, FieldAccessPointId_t fap_id_to, OutFieldInfo::MachineId_t machineId, OutFieldInfo::MapMachineStateTravelCosts_t &travelCosts) const
{
    for(int allFAPIds1 = 0 ; allFAPIds1 < (fap_id_from==AllFieldAccessPoints?1:2) ; ++allFAPIds1){//allFAPIds1 = 0 -> search for specific data ; allFAPIds1 = 1 --> search for default
        FieldAccessPointId_t fap_id_1 = fap_id_from;
        if(allFAPIds1 > 0)//allFAPIds1 = 1 --> search for default
            fap_id_1 = AllFieldAccessPoints;
        auto it_fapid_1 = m_mapAccessPoint2AccessPoint.find(fap_id_1);
        if( it_fapid_1 == m_mapAccessPoint2AccessPoint.end() )
            continue;

        for(size_t allFAPIds2 = 0 ; allFAPIds2 < (fap_id_to==AllFieldAccessPoints?1:2) ; ++allFAPIds2){//allFAPIds2 = 0 -> search for specific data ; allFAPIds2 = 1 --> search for default
            FieldAccessPointId_t FAP_id = fap_id_to;
            if(allFAPIds2 > 0)//allFAPIds = 1 --> search for default
                FAP_id = AllFieldAccessPoints;
            auto it_fapid_2 = it_fapid_1->second.find(FAP_id);
            if(it_fapid_2 == it_fapid_1->second.end())
                continue;
            if( getTravelCost( it_fapid_2->second,
                               machineId,
                               travelCosts ) )
                return true;
        }
    }
    return false;

}

bool OutFieldInfo::getTravelCost_FAP2FAP(ResourcePointId_t fap_id_from, FieldAccessPointId_t fap_id_to, MapMachineTravelCosts_t &travelCostsMap) const
{
    travelCostsMap.clear();

    for(int allFAPIds1 = 0 ; allFAPIds1 < (fap_id_from==AllResourcePoints?1:2) ; ++allFAPIds1){//allFAPIds1 = 0 -> search for specific data ; allFAPIds1 = 1 --> search for default
        FieldAccessPointId_t fap_id_1 = fap_id_from;
        if(allFAPIds1 > 0)//allFAPIds1 = 1 --> search for default
            fap_id_1 = AllResourcePoints;
        auto it_fapid_1 = m_mapAccessPoint2AccessPoint.find(fap_id_1);
        if( it_fapid_1 == m_mapAccessPoint2AccessPoint.end() )
            continue;

        for(size_t allFAPIds2 = 0 ; allFAPIds2 < (fap_id_to==AllFieldAccessPoints?1:2) ; ++allFAPIds2){//allFAPIds2 = 0 -> search for specific data ; allFAPIds2 = 1 --> search for default
            FieldAccessPointId_t fap_id_2 = fap_id_to;
            if(allFAPIds2 > 0)//allFAPIds2 = 1 --> search for default
                fap_id_2 = AllFieldAccessPoints;
            auto it_fapid_2 = it_fapid_1->second.find(fap_id_2);
            if(it_fapid_2 == it_fapid_1->second.end())
                continue;

            travelCostsMap = it_fapid_2->second;
            return true;
        }
    }
    return false;

}

bool OutFieldInfo::getUnloadingCosts(ResourcePointId_t resourcePointId,
                                     OutFieldInfo::MachineId_t machineId,
                                     OutFieldInfo::UnloadingCosts &unloadingCosts) const
{
    for(int allRPIds = 0 ; allRPIds < (resourcePointId==AllResourcePoints?1:2) ; ++allRPIds){//allRPIds = 0 -> search for specific data ; allRPIds = 1 --> search for default
        ResourcePointId_t RP_id = resourcePointId;
        if(allRPIds > 0)//allRPIds = 1 --> search for default
            RP_id = AllResourcePoints;
        auto it_resourcePointId = m_mapUnloadingCosts.find(RP_id);
        if( it_resourcePointId == m_mapUnloadingCosts.end() )
            continue;
        if ( getUnloadingCosts(it_resourcePointId->second,
                               machineId,
                               unloadingCosts) )
            return true;

    }
    return false;

}

bool OutFieldInfo::getUnloadingCosts(const ResourcePointId_t &resourcePointId,
                                      std::map<OutFieldInfo::MachineId_t, OutFieldInfo::UnloadingCosts> &unloadingCostsMap) const
{
    return getUnloadingCosts(m_mapUnloadingCosts, resourcePointId, unloadingCostsMap);
}

bool OutFieldInfo::getArrivalCost(FieldAccessPointId_t fieldAccessPointId,
                                  OutFieldInfo::MachineId_t machineId,
                                  OutFieldInfo::MachineBunkerState machineBunkerState,
                                  OutFieldInfo::TravelCosts &arrivalCosts) const
{
    for(size_t allFAPIds = 0 ; allFAPIds < (fieldAccessPointId==AllFieldAccessPoints?1:2) ; ++allFAPIds){//allFAPIds = 0 -> search for specific data ; allFAPIds = 1 --> search for default
        FieldAccessPointId_t FAP_id = fieldAccessPointId;
        if(allFAPIds > 0)//allFAPIds = 1 --> search for default
            FAP_id = AllFieldAccessPoints;
        auto it_fieldAccessPointId = m_mapArrivalCosts.find(FAP_id);
        if(it_fieldAccessPointId == m_mapArrivalCosts.end())
            continue;
        if( getArrivalCost( it_fieldAccessPointId->second,
                            machineId,
                            machineBunkerState,
                            arrivalCosts ) )
            return true;
    }
    return false;

}

bool OutFieldInfo::getArrivalCost(FieldAccessPointId_t fieldAccessPointId, OutFieldInfo::MachineId_t machineId, OutFieldInfo::MapMachineStateTravelCosts_t &arrivalCosts) const
{
    for(size_t allFAPIds = 0 ; allFAPIds < (fieldAccessPointId==AllFieldAccessPoints?1:2) ; ++allFAPIds){//allFAPIds = 0 -> search for specific data ; allFAPIds = 1 --> search for default
        FieldAccessPointId_t FAP_id = fieldAccessPointId;
        if(allFAPIds > 0)//allFAPIds = 1 --> search for default
            FAP_id = AllFieldAccessPoints;
        auto it_fieldAccessPointId = m_mapArrivalCosts.find(FAP_id);
        if(it_fieldAccessPointId == m_mapArrivalCosts.end())
            continue;
        if( getArrivalCost( it_fieldAccessPointId->second,
                            machineId,
                            arrivalCosts ) )
            return true;
    }
    return false;

}

bool OutFieldInfo::getArrivalCost(FieldAccessPointId_t fieldAccessPointId,
                                         MapMachineTravelCosts_t &arrivalCostsMap) const
{
    arrivalCostsMap.clear();

    for(size_t allFAPIds = 0 ; allFAPIds < (fieldAccessPointId==AllFieldAccessPoints?1:2) ; ++allFAPIds){//allFAPIds = 0 -> search for specific data ; allFAPIds = 1 --> search for default
        FieldAccessPointId_t FAP_id = fieldAccessPointId;
        if(allFAPIds > 0)//allFAPIds = 1 --> search for default
            FAP_id = AllFieldAccessPoints;
        auto it_fieldAccessPointId = m_mapArrivalCosts.find(FAP_id);
        if(it_fieldAccessPointId == m_mapArrivalCosts.end())
            continue;

        arrivalCostsMap = it_fieldAccessPointId->second;
        return true;
    }
    return false;
}

bool OutFieldInfo::getTravelCost(const MapMachineTravelCosts_t &travelCostsMap,
                                  OutFieldInfo::MachineId_t machineId,
                                  OutFieldInfo::MachineBunkerState machineBunkerState,
                                  OutFieldInfo::TravelCosts &travelCosts)
{
    for(int allIds = 0 ; allIds < (machineId==AllMachines?1:2) ; ++allIds){//allIds = 0 -> search for specific data ; allIds = 1 --> search for default
        MachineId_t m_id = machineId;
        if(allIds > 0)//allIds = 1 --> search for default
            m_id = AllMachines;
        auto it_machineId = travelCostsMap.find(m_id);
        if(it_machineId == travelCostsMap.end())
            continue;

        for(int allStates = 0 ; allStates < (machineBunkerState==MachineBunkerState::ALL_MACHINE_STATES?1:2) ; ++allStates){//allStates = 0 -> search for specific data ; allStates = 1 --> search for default
            OutFieldInfo::MachineBunkerState bunkerState = machineBunkerState;
            if(allStates > 0)//allStates = 1 --> search for default
                bunkerState = MachineBunkerState::ALL_MACHINE_STATES;

            auto it_machineBunkerState = it_machineId->second.find(bunkerState);
            if(it_machineBunkerState == it_machineId->second.end())
                continue;
            travelCosts = it_machineBunkerState->second;
            return true;
        }
    }
    return false;
}

bool OutFieldInfo::getTravelCost(const OutFieldInfo::MapMachineTravelCosts_t &travelCostsMap, OutFieldInfo::MachineId_t machineId, OutFieldInfo::MapMachineStateTravelCosts_t &travelCosts)
{
    for(int allIds = 0 ; allIds < (machineId==AllMachines?1:2) ; ++allIds){//allIds = 0 -> search for specific data ; allIds = 1 --> search for default
        MachineId_t m_id = machineId;
        if(allIds > 0)//allIds = 1 --> search for default
            m_id = AllMachines;
        auto it_machineId = travelCostsMap.find(m_id);
        if(it_machineId != travelCostsMap.end()){
            travelCosts = it_machineId->second;
            return true;
        }
    }
    return false;

}

bool OutFieldInfo::getTravelCost(const OutFieldInfo::MapMachineStateTravelCosts_t &travelCostsMap, OutFieldInfo::MachineBunkerState machineBunkerState, OutFieldInfo::TravelCosts &travelCosts)
{
    for(int allStates = 0 ; allStates < (machineBunkerState==MachineBunkerState::ALL_MACHINE_STATES?1:2) ; ++allStates){//allStates = 0 -> search for specific data ; allStates = 1 --> search for default
        OutFieldInfo::MachineBunkerState bunkerState = machineBunkerState;
        if(allStates > 0)//allStates = 1 --> search for default
            bunkerState = MachineBunkerState::ALL_MACHINE_STATES;

        auto it_machineBunkerState = travelCostsMap.find(bunkerState);
        if(it_machineBunkerState == travelCostsMap.end())
            continue;
        travelCosts = it_machineBunkerState->second;
        return true;
    }
    return false;
}

bool OutFieldInfo::getUnloadingCosts(const std::map<OutFieldInfo::MachineId_t, OutFieldInfo::UnloadingCosts> &unloadingCostsMap,
                                      OutFieldInfo::MachineId_t machineId,
                                      OutFieldInfo::UnloadingCosts &unloadingCosts)
{
    for(int allIds = 0 ; allIds < (machineId==AllMachines?1:2) ; ++allIds){//allIds = 0 -> search for specific data ; allIds = 1 --> search for default
        MachineId_t m_id = machineId;
        if(allIds > 0)//allIds = 1 --> search for default
            m_id = AllMachines;
        auto it_machineId = unloadingCostsMap.find(m_id);
        if(it_machineId == unloadingCostsMap.end())
            continue;
        unloadingCosts = it_machineId->second;
        return true;
    }
    return false;
}

bool OutFieldInfo::getUnloadingCosts(const MapUnloadingCosts_t &unloadingCostsMap_all,
                                     const ResourcePointId_t &resourcePointId,
                                     std::map<OutFieldInfo::MachineId_t, OutFieldInfo::UnloadingCosts> &unloadingCostsMap_rp)
{

    unloadingCostsMap_rp.clear();
    for(int allRPIds = 0 ; allRPIds < (resourcePointId==AllResourcePoints?1:2) ; ++allRPIds){//allRPIds = 0 -> search for specific data ; allRPIds = 1 --> search for default
        ResourcePointId_t RP_id = resourcePointId;
        if(allRPIds > 0)//allRPIds = 1 --> search for default
            RP_id = AllResourcePoints;
        auto it_resourcePointId = unloadingCostsMap_all.find(RP_id);
        if( it_resourcePointId == unloadingCostsMap_all.end() )
            continue;

        unloadingCostsMap_rp = it_resourcePointId->second;
        return true;

    }
    return false;
}

bool OutFieldInfo::getArrivalCost(const MapMachineTravelCosts_t &arrivalCostsMap,
                                  OutFieldInfo::MachineId_t machineId,
                                  OutFieldInfo::MachineBunkerState machineBunkerState,
                                  OutFieldInfo::TravelCosts &arrivalCosts)
{
    for(int allIds = 0 ; allIds < (machineId==AllMachines?1:2) ; ++allIds){//allIds = 0 -> search for specific data ; allIds = 1 --> search for default
        MachineId_t m_id = machineId;
        if(allIds > 0)//allIds = 1 --> search for default
            m_id = AllMachines;
        auto it_machineId = arrivalCostsMap.find(m_id);
        if(it_machineId == arrivalCostsMap.end())
            continue;

        for(int allStates = 0 ; allStates < (machineBunkerState==MachineBunkerState::ALL_MACHINE_STATES?1:2) ; ++allStates){//allStates = 0 -> search for specific data ; allStates = 1 --> search for default
            OutFieldInfo::MachineBunkerState bunkerState = machineBunkerState;
            if(allStates > 0)//allStates = 1 --> search for default
                bunkerState = MachineBunkerState::ALL_MACHINE_STATES;

            auto it_machineBunkerState = it_machineId->second.find(bunkerState);
            if(it_machineBunkerState == it_machineId->second.end())
                continue;
            arrivalCosts = it_machineBunkerState->second;
            return true;
        }
    }
    return false;
}

bool OutFieldInfo::getArrivalCost(const OutFieldInfo::MapMachineTravelCosts_t &arrivalCostsMap, OutFieldInfo::MachineId_t machineId, OutFieldInfo::MapMachineStateTravelCosts_t &arrivalCosts)
{
    for(int allIds = 0 ; allIds < (machineId==AllMachines?1:2) ; ++allIds){//allIds = 0 -> search for specific data ; allIds = 1 --> search for default
        MachineId_t m_id = machineId;
        if(allIds > 0)//allIds = 1 --> search for default
            m_id = AllMachines;
        auto it_machineId = arrivalCostsMap.find(m_id);
        if(it_machineId != arrivalCostsMap.end()){
            arrivalCosts = it_machineId->second;
            return true;
        }
    }
    return false;

}

bool OutFieldInfo::addDefaultUnloadingCosts(const ResourcePointId_t &resourcePointId,
                                             const OutFieldInfo::UnloadingCosts &unloadingCosts,
                                             bool overwrite)
{
    auto resourcePointId_it = m_mapUnloadingCosts.find(resourcePointId);
    if( resourcePointId_it == m_mapUnloadingCosts.end() ){
        m_mapUnloadingCosts[resourcePointId][AllMachines] = unloadingCosts;
        return true;
    }
    auto machineId_it = resourcePointId_it->second.find(AllMachines);
    if( machineId_it == resourcePointId_it->second.end() ){
        m_mapUnloadingCosts[resourcePointId][AllMachines] = unloadingCosts;
        return true;
    }
    if(overwrite){
        m_mapUnloadingCosts[resourcePointId][AllMachines] = unloadingCosts;
        return true;
    }
    return false;
}

bool OutFieldInfo::addDefaultArrivalCosts(const FieldAccessPointId_t &fieldAccessPointId,
                                          const OutFieldInfo::TravelCosts &arrivalCosts,
                                          bool overwrite)
{
    auto fieldAccessPointId_it = m_mapArrivalCosts.find(fieldAccessPointId);
    if( fieldAccessPointId_it == m_mapArrivalCosts.end() ){
        m_mapArrivalCosts[fieldAccessPointId][AllMachines][ALL_MACHINE_STATES] = arrivalCosts;
        return true;
    }
    auto machineId_it = fieldAccessPointId_it->second.find(AllMachines);
    if( machineId_it == fieldAccessPointId_it->second.end() ){
        m_mapArrivalCosts[fieldAccessPointId][AllMachines][ALL_MACHINE_STATES] = arrivalCosts;
        return true;
    }
    auto machineBunkerState_it = machineId_it->second.find(ALL_MACHINE_STATES);
    if( machineBunkerState_it == machineId_it->second.end() ){
        m_mapArrivalCosts[fieldAccessPointId][AllMachines][ALL_MACHINE_STATES] = arrivalCosts;
        return true;
    }
    if(overwrite){
        m_mapArrivalCosts[fieldAccessPointId][AllMachines][ALL_MACHINE_STATES] = arrivalCosts;
        return true;
    }
    return false;
}

std::vector<OutFieldInfo::TravelData> OutFieldInfo::getAllAccessPoint2ResourcePointData() const
{
    std::vector<TravelData> ret;
    for(auto &it_fap : m_mapAccessPoint2ResourcePoint){
        for(auto &it_rp : it_fap.second){
            for(auto &it_m : it_rp.second){
                for(auto &it_bs : it_m.second){
                    OutFieldInfo::TravelData td;
                    td.fieldAccessPointId = it_fap.first;
                    td.resourcePointId = it_rp.first;
                    td.machineId = it_m.first;
                    td.machineBunkerState = it_bs.first;
                    td.travelCosts = it_bs.second;
                    ret.emplace_back(td);
                }
            }
        }
    }
    return ret;
}

std::vector<OutFieldInfo::TravelData> OutFieldInfo::getAllResourcePoint2AccessPointData() const
{
    std::vector<OutFieldInfo::TravelData> ret;
    for(auto &it_rp : m_mapResourcePoint2AccessPoint){
        for(auto &it_fap : it_rp.second){
            for(auto &it_m : it_fap.second){
                for(auto &it_bs : it_m.second){
                    OutFieldInfo::TravelData td;
                    td.fieldAccessPointId = it_fap.first;
                    td.resourcePointId = it_rp.first;
                    td.machineId = it_m.first;
                    td.machineBunkerState = it_bs.first;
                    td.travelCosts = it_bs.second;
                    ret.emplace_back(td);
                }
            }
        }
    }
    return ret;
}

std::vector<OutFieldInfo::TravelData2> OutFieldInfo::getAllAccessPoint2AccessPointData() const
{
    std::vector<TravelData2> ret;
    for(auto &it_fap_1 : m_mapAccessPoint2AccessPoint){
        for(auto &it_fap_2 : it_fap_1.second){
            for(auto &it_m : it_fap_2.second){
                for(auto &it_bs : it_m.second){
                    TravelData2 td;
                    td.fap_id_from = it_fap_1.first;
                    td.fap_id_to = it_fap_2.first;
                    td.machineId = it_m.first;
                    td.machineBunkerState = it_bs.first;
                    td.travelCosts = it_bs.second;
                    ret.emplace_back(td);
                }
            }
        }
    }
    return ret;
}

std::vector<OutFieldInfo::UnloadingData> OutFieldInfo::getAllUnloadingData() const
{
    std::vector<OutFieldInfo::UnloadingData> ret;
    for(auto &it_rp : m_mapUnloadingCosts){
        for(auto &it_m : it_rp.second){
            OutFieldInfo::UnloadingData ud;
            ud.resourcePointId = it_rp.first;
            ud.machineId = it_m.first;
            ud.unloadingCosts = it_m.second;
            ret.emplace_back(ud);
        }
    }
    return ret;
}

std::vector<OutFieldInfo::ArrivalData> OutFieldInfo::getAllArrivalData() const
{
    std::vector<OutFieldInfo::ArrivalData> ret;
    for(auto &it_fap : m_mapArrivalCosts){
        for(auto &it_m : it_fap.second){
            for(auto &it_bs : it_m.second){
                OutFieldInfo::ArrivalData ad;
                ad.fieldAccessPointId = it_fap.first;
                ad.machineId = it_m.first;
                ad.machineBunkerState = it_bs.first;
                ad.arrivalCosts = it_bs.second;
                ret.emplace_back(ad);
            }
        }
    }
    return ret;
}

bool OutFieldInfo::empty() const
{
    return m_mapAccessPoint2ResourcePoint.empty()
            && m_mapResourcePoint2AccessPoint.empty()
            && m_mapAccessPoint2AccessPoint.empty()
            && m_mapUnloadingCosts.empty()
            && m_mapArrivalCosts.empty();
}

bool OutFieldInfo::empty_travelCosts_FAP_RP() const
{
    return m_mapAccessPoint2ResourcePoint.empty()
            && m_mapResourcePoint2AccessPoint.empty();
}




}
