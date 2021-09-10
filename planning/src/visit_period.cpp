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
 

#include "arolib/planning/visit_period.hpp"

namespace arolib{

using namespace arolib::geometry;

bool VisitPeriod::DelayInfo::operator <(const VisitPeriod::DelayInfo& other) const{
    if(time_from < other.time_from)
        return true;
    if(time_from > other.time_from)
        return false;
    if(delay < other.delay)
        return true;
    if(delay > other.delay)
        return false;
    return onlyTimeOut < other.onlyTimeOut;
}

VisitPeriod::VisitPeriod(MachineId_t _machineId, double _time_in, double _time_out, double _timestamp, std::vector<Polygon> _next_area):
    machineId(_machineId),
    time_in(_time_in),
    time_out(_time_out),
    timestamp(_timestamp),
    next_area(_next_area)
{

}

bool VisitPeriod::isBusy(const std::vector<VisitPeriod> &visitPeriods,
                         double time_in,
                         double duration,
                         int maxMachines,
                         double &waiting,
                         Point currentLocation,
                         bool &towardsCurrentLocation,
                         std::vector<MachineId_t> &machineIds,
                         const std::set<MachineId_t> &excludedMachines,
                         const std::map<MachineId_t, std::set<DelayInfo> > &delays)
{
    towardsCurrentLocation = false;
    waiting = 0;
    machineIds.clear();
    if(maxMachines <= 0)
        return false;

    auto sortedVisitPeriods = getSortedVisitPeriods(visitPeriods, delays);

    double time_out = time_in + duration;
    std::multimap<double, std::pair<MachineId_t, bool>, std::greater<double>> waitings;

    for(auto it : sortedVisitPeriods){
        if( it.first > time_out )
            break;
        const VisitPeriod& vp = it.second;

        if( time_in > vp.time_out )
            continue;

        if(excludedMachines.find(vp.machineId) != excludedMachines.end())
            continue;

        bool bNextArea = false;
        for(auto &next_area : vp.next_area){
            if( arolib::geometry::in_polygon(currentLocation, next_area) ){
                bNextArea = true;
                break;
            }
        }

        waitings.insert( std::make_pair( vp.time_out - time_in , std::make_pair(vp.machineId, bNextArea) ) );
    }


    if(waitings.size() >= maxMachines){
        //auto it = std::next(waitings.begin(), maxMachines-1);
        auto it = waitings.begin();
        machineIds.push_back( it->second.first );
        for(size_t i = 1 ; i < maxMachines ; ++i ){
            ++it;
            machineIds.push_back( it->second.first );
        }
        waiting = it->first;

        while( it != waitings.end() ){
            if(it->second.second){
                towardsCurrentLocation = true;
                break;
            }
            ++it;
        }

    }

    return waitings.size() >= maxMachines;
}

std::set<MachineId_t> VisitPeriod::getVisitingMachines(const std::vector<VisitPeriod> &visitPeriods,
                                                       double time_in,
                                                       double duration,
                                                       const std::set<MachineId_t>& excludedMachines,
                                                       const std::map<MachineId_t, std::set<DelayInfo> > &delays )
{
    std::set<MachineId_t> machineIds;
    double time_out = time_in + duration;
    auto sortedVisitPeriods = getSortedVisitPeriods(visitPeriods, delays);
    for(auto it : sortedVisitPeriods){
        if( it.first > time_out )
            break;
        const VisitPeriod& vp = it.second;

        if( time_in > vp.time_out )
            continue;

        if(excludedMachines.find(vp.machineId) != excludedMachines.end())
            continue;

        machineIds.insert( vp.machineId );
    }
    return machineIds;
}

bool VisitPeriod::isMachineVisiting(const std::vector<VisitPeriod> &visitPeriods,
                                    MachineId_t machineId,
                                    double time_in,
                                    double time_out,
                                    const std::map<MachineId_t, std::set<DelayInfo> > &delays)
{
    if(time_in >= time_out)
        return false;
    auto sortedVisitPeriods = getSortedVisitPeriods(visitPeriods, delays);
    for(auto it : sortedVisitPeriods){
        if( it.first > time_out )
            break;
        const VisitPeriod& vp = it.second;

        if( time_in > vp.time_out )
            continue;

        if(machineId == vp.machineId)
            return true;
    }
    return false;

}

std::multimap<double, VisitPeriod> VisitPeriod::getSortedVisitPeriods(const std::vector<VisitPeriod> &visitPeriods,
                                                                      const std::map<MachineId_t, std::set<VisitPeriod::DelayInfo> > &delays)
{
    std::multimap<double, VisitPeriod> ret;
    for(auto& vp0 : visitPeriods){
        auto vp = vp0;
        auto it_delay = delays.find(vp.machineId);
        if(it_delay != delays.end()){
            for(DelayInfo d : it_delay->second){
                if(vp.timestamp - 1e-9 > d.time_from){
                    if(!d.onlyTimeOut){
                        vp.time_in += d.delay;
                        vp.timestamp += d.delay;
                    }
                    vp.time_out += d.delay;
                }
            }
        }
        ret.insert( std::make_pair(vp.time_in, vp) );
    }

    return ret;
}

}

