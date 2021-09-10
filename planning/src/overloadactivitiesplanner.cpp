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
 
#include "arolib/planning/overloadactivitiesplanner.h"

namespace arolib{

const double OverloadActivitiesPlanner::InitialOlvMaxCapacityMultiplier = 0.7;

bool OverloadActivitiesPlanner::PlannerSettings::parseFromStringMap(OverloadActivitiesPlanner::PlannerSettings &params, const std::map<std::string, std::string> &map, bool strict)
{
    return ASP_GeneralSettings::parseFromStringMap(params, map, strict);
}

std::map<std::string, std::string> OverloadActivitiesPlanner::PlannerSettings::parseToStringMap(const OverloadActivitiesPlanner::PlannerSettings &params)
{
    return ASP_GeneralSettings::parseToStringMap(params);
}

OverloadActivitiesPlanner::OverloadActivitiesPlanner(LogLevel logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}

std::vector<OLVPlan::OverloadInfo> OverloadActivitiesPlanner::computeOverloadActivities(PlannerSettings settings, const Route &harvester_route, const std::vector<Machine> &overloadMachines, const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates, double harvestedMassLimit, Logger *_logger)
{
    if(settings.switchOnlyAtTrackEnd)
        return computeTracksEndsOverloadPoints(harvester_route, overloadMachines, machineCurrentStates, harvestedMassLimit, _logger);

    return computeFieldOverloadPoints(harvester_route, overloadMachines, machineCurrentStates, harvestedMassLimit, _logger);
}

std::vector<OLVPlan::OverloadInfo> OverloadActivitiesPlanner::computeFieldOverloadPoints(const Route &harvester_route,
                                                                                         const std::vector<Machine> &_overloadMachines,
                                                                                         const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                                                         double harvestedMassLimit,
                                                                                         Logger *_logger)
{
    Logger logger(LogLevel::CRITIC, __FUNCTION__);
    logger.setParent(_logger);
    std::vector<OLVPlan::OverloadInfo> ret;

    std::vector<Machine> overloadMachines = _overloadMachines;
    // we need the first routepoint that is on the track to get the initial harvested_mass
    // this is required in case of replanning
    int first_non_headland_index = 0;
    RoutePoint first_rp = harvester_route.route_points.at(first_non_headland_index);
    while (first_rp.type == RoutePoint::RoutePointType::HEADLAND ||
           first_rp.type == RoutePoint::RoutePointType::TRANSIT ||
           first_rp.time_stamp < -0.0001) {
        first_non_headland_index++;
        if(first_non_headland_index >= harvester_route.route_points.size())
            return ret;
        first_rp = harvester_route.route_points.at(first_non_headland_index);
    }

    RoutePoint second_rp = first_rp;
    if(first_non_headland_index+1 < harvester_route.route_points.size())
        second_rp = harvester_route.route_points.at(first_non_headland_index+1);
    double distDisregardInitialLimit = arolib::geometry::calc_dist(first_rp, second_rp) ;
    distDisregardInitialLimit = std::max( 25.0, std::min(50.0, distDisregardInitialLimit*5) );

    std::vector<double> olvCurrentBunkerLevels( overloadMachines.size() );
    for(size_t i = 0 ; i < overloadMachines.size() ; ++i){
        auto it_m = machineCurrentStates.find(overloadMachines.at(i).id);
        if(it_m != machineCurrentStates.end()){
            double dist = arolib::geometry::calc_dist(it_m->second.position, first_rp);
            bool disregardInitialLimit = (i == 0 && dist < distDisregardInitialLimit);
            if(disregardInitialLimit
                    || it_m->second.bunkerMass < overloadMachines.at(i).bunker_mass * InitialOlvMaxCapacityMultiplier)
                olvCurrentBunkerLevels.at(i) = it_m->second.bunkerMass;
            else
                olvCurrentBunkerLevels.at(i) = overloadMachines.at(i).bunker_mass * 2;//make them go tho a resource point first
        }
        else
            olvCurrentBunkerLevels.at(i) = 0;
    }

    int olv_index = 0;
    Machine current_olv = overloadMachines.at(olv_index);
    double current_harvester_mass = first_rp.harvested_mass;
    double max_harvested_mass = current_harvester_mass + current_olv.bunker_mass * OLVPlan::OlvMaxCapacityMultiplier - olvCurrentBunkerLevels.at(olv_index);  /// harvester_mass when the next olv will be full
    int overload_start_index = first_non_headland_index;
    int overload_end_index = first_non_headland_index;

    bool toResourcePointFirst = false;
    bool olvCurrentBunkerLevel_prev;
    bool updateCurrentMass = true;
    bool updateLastIndex = true;

    for (int i = first_non_headland_index; i+1 < harvester_route.route_points.size(); ++i) {
        RoutePoint next_rp = harvester_route.route_points.at(i+1);

//        if(overload_start_index >= i &&
//                ( next_rp.type == RoutePoint::RoutePointType::HEADLAND
//                  || next_rp.type == RoutePoint::RoutePointType::TRACK_END
//                  || next_rp.type == RoutePoint::RoutePointType::TRACK_START ) ){
//            ++overload_start_index;
//            //updateLastIndex = false;
//            continue;
//        }

        if (!next_rp.isOfType({RoutePoint::HEADLAND, RoutePoint::TRACK_START, RoutePoint::TRANSIT})) {
            double next_harvester_mass = next_rp.harvested_mass;

            if (next_harvester_mass > max_harvested_mass) {

                if(overload_start_index < overload_end_index){
                    OLVPlan::OverloadInfo olvinfo;
                    olvinfo.start_index = overload_start_index;
                    olvinfo.end_index = overload_end_index;
                    olvinfo.machine = current_olv;
                    olvinfo.toResourcePointFirst = toResourcePointFirst;
                    toResourcePointFirst = false;
                    ret.push_back(olvinfo);

                    if(harvestedMassLimit > 1e-6
                            && harvester_route.route_points.at(olvinfo.end_index).harvested_mass > harvestedMassLimit)
                        return ret;

                    if(next_rp.type == RoutePoint::RoutePointType::TRACK_END){//is this necesary?
                        overload_start_index = i+1;
                        while(overload_start_index < harvester_route.route_points.size()
                              && harvester_route.route_points.at(overload_start_index).type != RoutePoint::RoutePointType::TRACK_START)
                            ++overload_start_index;
                    }
                    else{
                        if(updateLastIndex)
                            overload_start_index = i+1;
                        else
                            overload_start_index = i;
                    }

                    olvCurrentBunkerLevel_prev = olvCurrentBunkerLevels.at(olv_index);
                    olvCurrentBunkerLevels.at(olv_index) = 0;

                    // next olv machine:
                    olv_index = (olv_index + 1) % overloadMachines.size();
                    current_olv = overloadMachines.at(olv_index);
                }
                else if(!toResourcePointFirst){
                    toResourcePointFirst = true;
                    olvCurrentBunkerLevels.at(olv_index) = 0;
                    i--;
                    updateCurrentMass = false;
                }
                else{
                    toResourcePointFirst = false;
                    olvCurrentBunkerLevels.at(olv_index) = olvCurrentBunkerLevel_prev;

                    // next olv machine:
                    olv_index = (olv_index + 1) % overloadMachines.size();
                    current_olv = overloadMachines.at(olv_index);
                }


                /// next maximum
                max_harvested_mass = current_harvester_mass + current_olv.bunker_mass * OLVPlan::OlvMaxCapacityMultiplier - olvCurrentBunkerLevels.at(olv_index);

            }
            else /*if(updateLastIndex)*/
                overload_end_index = i+1;

            // update variables for next iteration
            if(updateCurrentMass)
                current_harvester_mass = next_harvester_mass;
            else
                updateCurrentMass = true;

            updateLastIndex = true;
        }
        else
            updateLastIndex = false;
    }


    /// add last overloading activity
    OLVPlan::OverloadInfo olvinfo;
    olvinfo.start_index = overload_start_index;
    olvinfo.end_index = harvester_route.route_points.size()-1;
    olvinfo.machine = current_olv;
    olvinfo.toResourcePointFirst = toResourcePointFirst;
    if(olvinfo.start_index < olvinfo.end_index){
        ret.push_back(olvinfo);

    }

    return ret;

}

std::vector<OLVPlan::OverloadInfo> OverloadActivitiesPlanner::computeTracksEndsOverloadPoints(const Route &harvester_route, std::vector<Machine> overloadMachines,
                                                                                              const std::map<MachineId_t, MachineDynamicInfo> &machineCurrentStates,
                                                                                              double harvestedMassLimit,
                                                                                              Logger *_logger)
{
    Logger logger(LogLevel::CRITIC, __FUNCTION__);
    logger.setParent(_logger);

    std::vector<OLVPlan::OverloadInfo> ret;
    int olv_index = 0;  /// index of the current overloading vehicle in overloadMachines
    int overload_start_index = 0;
    int old_index = 0;  // index of the previous track end

    int first_non_headland_index = 0;
    RoutePoint first_rp = harvester_route.route_points.at(first_non_headland_index);
    while (first_rp.type == RoutePoint::RoutePointType::HEADLAND ||
           first_rp.type == RoutePoint::RoutePointType::TRANSIT ||
           first_rp.time_stamp < -0.0001) {
        first_non_headland_index++;
        if(first_non_headland_index >= harvester_route.route_points.size())
            return ret;
        first_rp = harvester_route.route_points.at(first_non_headland_index);
    }
    RoutePoint second_rp = first_rp;
    if(first_non_headland_index+1 < harvester_route.route_points.size())
        second_rp = harvester_route.route_points.at(first_non_headland_index+1);
    double distDisregardInitialLimit = arolib::geometry::calc_dist(first_rp, second_rp) ;
    distDisregardInitialLimit = std::max( 25.0, std::min(50.0, distDisregardInitialLimit*5) );

    std::vector<double> olvCurrentBunkerLevels( overloadMachines.size() );
    for(size_t i = 0 ; i < overloadMachines.size() ; ++i){
        auto it_m = machineCurrentStates.find(overloadMachines.at(i).id);
        if(it_m != machineCurrentStates.end()){
            double dist = arolib::geometry::calc_dist(it_m->second.position, first_rp);
            bool disregardInitialLimit = (i == 0 && dist < distDisregardInitialLimit);
            if(disregardInitialLimit
                    || it_m->second.bunkerMass < overloadMachines.at(i).bunker_mass * InitialOlvMaxCapacityMultiplier)
                olvCurrentBunkerLevels.at(i) = it_m->second.bunkerMass;
            else
                olvCurrentBunkerLevels.at(i) = overloadMachines.at(i).bunker_mass * 2;//make them go tho a resource point first
        }
        else
            olvCurrentBunkerLevels.at(i) = 0;
    }

    double max_harvested_mass = first_rp.harvested_mass + overloadMachines.at(olv_index).bunker_mass * OLVPlan::OlvMaxCapacityMultiplier - olvCurrentBunkerLevels.at(olv_index);  /// harvester_mass when the next olv will be full
    while (old_index < harvester_route.route_points.size()-1) {
        int next_end_id = run_to_next_type(harvester_route, old_index+1, RoutePoint::RoutePointType::TRACK_END);
        RoutePoint next_end_rp = harvester_route.route_points.at(old_index);
        double next_end_harvester_mass = next_end_rp.harvested_mass;
        if (next_end_harvester_mass > max_harvested_mass) {
            OLVPlan::OverloadInfo olvinfo;
            olvinfo.start_index = overload_start_index;
            olvinfo.end_index = old_index;
            olvinfo.machine = overloadMachines.at(olv_index);
            ret.push_back(olvinfo);

            if(harvestedMassLimit > 1e-6
                    && harvester_route.route_points.at(olvinfo.end_index).harvested_mass > harvestedMassLimit)
                return ret;

            /// next start:
            overload_start_index = run_to_next_type(harvester_route, old_index, RoutePoint::RoutePointType::TRACK_START);
            /// next maximum:
            RoutePoint start_rp = harvester_route.route_points.at(overload_start_index);

            olvCurrentBunkerLevels.at(olv_index) = 0;

            /// next olv machine:
            olv_index = (olv_index + 1) % overloadMachines.size();
            max_harvested_mass = start_rp.harvested_mass + overloadMachines.at(olv_index).bunker_mass * OLVPlan::OlvMaxCapacityMultiplier - olvCurrentBunkerLevels.at(olv_index); /// TODO should not be completely filled (maybe 90%)

            logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Found next overload switching point: next_bunker_sum: " + std::to_string(max_harvested_mass));
        }
        old_index = next_end_id;
    }

    /// add last overloading activity
    OLVPlan::OverloadInfo olvinfo;
    olvinfo.start_index = overload_start_index;
    olvinfo.end_index = harvester_route.route_points.size()-1;
    olvinfo.machine = overloadMachines.at(olv_index);
    ret.push_back(olvinfo);
    return ret;

}

int OverloadActivitiesPlanner::run_to_next_type(const Route &route, int start, RoutePoint::RoutePointType type) {
    for (int i = start; i < route.route_points.size(); ++i) {
        RoutePoint rp = route.route_points.at(i);
        if (rp.type == type) {
            return i;
        }
    }
    return route.route_points.size()-1;  //TODO or -1???
}

}
