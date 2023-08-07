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
 
#include "arolib/types/headland.hpp"
namespace arolib {

const int PartialHeadland::NoConnectingHeadlandId = std::numeric_limits<int>::lowest();

bool Headlands::hasCompleteHeadland() const{
    return !complete.tracks.empty() || !complete.middle_track.points.empty();
}

void Headlands::clear()
{
    complete = CompleteHeadland();
    partial.clear();
}

bool PartialHeadland::isConnectingHeadland(bool twoSided) const
{
    if(twoSided)
        return connectingHeadlandIds.first != NoConnectingHeadlandId && connectingHeadlandIds.second != NoConnectingHeadlandId;
    return connectingHeadlandIds.first != NoConnectingHeadlandId || connectingHeadlandIds.second != NoConnectingHeadlandId;
}

std::vector<std::vector<size_t>> PartialHeadland::getHeadlandConnectionSequences(const std::vector<PartialHeadland> &hls, size_t ind_from, size_t ind_to)
{
    std::vector<std::vector<size_t>> ret;
    if(ind_from >= hls.size() || ind_to >= hls.size())
        return ret;

    std::vector<size_t> visitLimits(hls.size(), 1);

    if(ind_from == ind_to){
        ret.push_back({ind_from});
        visitLimits.at(ind_from) = 2;
    }

    std::set<size_t> connectingHLs;
    std::map<int, size_t> idsMap;

    for(size_t i = 0 ; i < hls.size() ; ++i){
        const auto& hl = hls.at(i);
        if(hl.isConnectingHeadland()){
            connectingHLs.insert(i);
            if(hl.connectingHeadlandIds.first == hl.connectingHeadlandIds.second)
                visitLimits.at(i) = 2;
        }
        idsMap[hls.at(i).id] = i;
    }

    auto checkVisits = [visitLimits](const std::vector<size_t>& visited, size_t ind) -> bool{
        return visited.at(ind) < visitLimits.at(ind);
    };

    for(int side = 0; side < 2 ; ++side ){

        std::vector<size_t> visited(hls.size(), 0);
        visited.at(ind_from) = 1;

        std::vector<size_t> conn = {ind_from};
        size_t ind = ind_from;
        size_t ind_prev = hls.size();
        do{
            int ind_next = -1;
            if(connectingHLs.find(ind) != connectingHLs.end()){
                const auto& hl = hls.at(ind);
                if(ind == ind_from){//first iteration
                    int hl_id = ( side == 0 ? hl.connectingHeadlandIds.first : hl.connectingHeadlandIds.second );
                    auto it = idsMap.find( hl_id );
                    if(it != idsMap.end())
                        ind_next = it->second;

                    if(hl.connectingHeadlandIds.first == hl.connectingHeadlandIds.second)
                        ++side;
                }
                else{
                    if(hl.connectingHeadlandIds.first == hl.connectingHeadlandIds.second){//special case where the connecting headland interconnects the same headland
                        auto it = idsMap.find( hl.connectingHeadlandIds.first );
                        if(it != idsMap.end() && checkVisits(visited, it->second))
                            ind_next = it->second;
                    }
                    else{
                        auto it = idsMap.find( hl.connectingHeadlandIds.first );
                        if(it != idsMap.end() && it->second != ind_prev && checkVisits(visited, it->second))
                            ind_next = it->second;
                        if(ind_next < 0){
                            it = idsMap.find( hl.connectingHeadlandIds.second );
                            if(it != idsMap.end() && it->second != ind_prev && checkVisits(visited, it->second))
                                ind_next = it->second;
                        }
                    }
                }
            }
            else{
                int count = 0;
                for(auto& ch_ind : connectingHLs){
                    if( !checkVisits(visited, ch_ind) )
                        continue;

                    const auto& hl = hls.at(ch_ind);
                    bool sameConnection = (hl.connectingHeadlandIds.first == hl.connectingHeadlandIds.second);

                    if( ( ch_ind == ind_prev && !sameConnection ) )
                        continue;

                    if(hl.connectingHeadlandIds.first == ind || hl.connectingHeadlandIds.second == ind){
                        if(ind == ind_from){//first iteration -> check search for 2 different connecting hls
                            if(count == side){
                                ind_next = ch_ind;
                                break;
                            }
                            if(sameConnection){
                                ++side;
                                break;
                            }
                            ++count;
                        }
                        else{
                            ind_next = ch_ind;
                            break;
                        }
                    }
                }
            }
            if(ind_next < 0)
                break;
            ind_prev = ind;
            ind = ind_next;
            visited.at(ind) = visited.at(ind) + 1;
            conn.push_back(ind);

        }while(ind != ind_to);
        if(conn.size() > 1 && conn.back() == ind_to)
            ret.push_back(conn);
    }
    return ret;
}


}
