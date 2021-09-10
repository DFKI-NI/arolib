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
 
#include "arolib/components/componentparametershelper.h"

namespace arolib{
namespace {

std::string getParameterString(const std::map<std::string, std::map<std::string, std::string> > &map,
                               const std::string& tag1,
                               const std::string& tag2){
    try{
        return map.at(tag1).at(tag2);
    }
    catch(const std::exception& e){
        return "";
    }
}

void setParameterFromMap(const std::map<std::string, std::map<std::string, std::string> > &map,
                         const std::string& tag1,
                         const std::string& tag2,
                         double& param){
    std::string sParam = getParameterString(map, tag1, tag2);
    if(sParam.empty())
        return;
    try{
        param = string2double(sParam);
    }
    catch(const std::exception&){}
}

void setParameterFromMap(const std::map<std::string, std::map<std::string, std::string> > &map,
                         const std::string& tag1,
                         const std::string& tag2,
                         int& param){
    std::string sParam = getParameterString(map, tag1, tag2);
    if(sParam.empty())
        return;
    try{
        param = std::stoi(sParam);
    }
    catch(const std::exception&){}
}

void setParameterFromMap(const std::map<std::string, std::map<std::string, std::string> > &map,
                         const std::string& tag1,
                         const std::string& tag2,
                         size_t& param){
    std::string sParam = getParameterString(map, tag1, tag2);
    if(sParam.empty())
        return;
    try{
        param = std::stoi(sParam);
    }
    catch(const std::exception&){}
}

void setParameterFromMap(const std::map<std::string, std::map<std::string, std::string> > &map,
                         const std::string& tag1,
                         const std::string& tag2,
                         bool& param){
    std::string sParam = getParameterString(map, tag1, tag2);
    if(sParam.empty())
        return;
    try{
        param = std::stoi(sParam);
    }
    catch(const std::exception&){}
}

void setParameterFromMap(const std::map<std::string, std::map<std::string, std::string> > &map,
                         const std::string& tag1,
                         const std::string& tag2_x,
                         const std::string& tag2_y,
                         Point& param){
    std::string sParam_x = getParameterString(map, tag1, tag2_x);
    std::string sParam_y = getParameterString(map, tag1, tag2_y);
    if(sParam_x.empty() || sParam_y.empty())
        return;
    try{
        param = Point( string2double( sParam_x ),
                       string2double( sParam_y ) );
    }
    catch(const std::exception&){}
}

void setParameterFromMap(const std::map<std::string, std::map<std::string, std::string> > &map,
                         const std::string& tag1,
                         const std::string& tag2,
                         HeadlandPlanner::TracksSamplingStrategy& param){
    std::string sParam = getParameterString(map, tag1, tag2);
    if(sParam.empty())
        return;
    try{
        param = HeadlandPlanner::intToTracksSamplingStrategy( std::stoi(sParam) );
    }
    catch(const std::exception&){}
}

void setParameterFromMap(const std::map<std::string, std::map<std::string, std::string> > &map,
                         const std::string& tag1,
                         const std::string& tag2,
                         TracksGenerator::ShiftingStrategy& param){
    std::string sParam = getParameterString(map, tag1, tag2);
    if(sParam.empty())
        return;
    try{
        param = TracksGenerator::intToShiftingStrategy( std::stoi(sParam) );
    }
    catch(const std::exception&){}
}

void setParameterFromMap(const std::map<std::string, std::map<std::string, std::string> > &map,
                         const std::string& tag1,
                         const std::string& tag2,
                         TracksGenerator::TrackOrderStrategy& param){
    std::string sParam = getParameterString(map, tag1, tag2);
    if(sParam.empty())
        return;
    try{
        param = TracksGenerator::intToTrackOrderStrategy( std::stoi(sParam) );
    }
    catch(const std::exception&){}
}

void setParameterFromMap(const std::map<std::string, std::map<std::string, std::string> > &map,
                         const std::string& tag1,
                         const std::string& tag2,
                         SimpleTrackSequencer::SequenceStrategy& param){
    std::string sParam = getParameterString(map, tag1, tag2);
    if(sParam.empty())
        return;
    try{
        param = SimpleTrackSequencer::intToSequenceStrategy( std::stoi(sParam) );
    }
    catch(const std::exception&){}
}

void setParameterFromMap(const std::map<std::string, std::map<std::string, std::string> > &map,
                         const std::string& tag1,
                         const std::string& tag2,
                         MultiOLVPlanner::OlvOrderStrategy& param){
    std::string sParam = getParameterString(map, tag1, tag2);
    if(sParam.empty())
        return;
    try{
        param = MultiOLVPlanner::intToOlvOrderStrategy( std::stoi(sParam) );
    }
    catch(const std::exception&){}
}

void setParameterFromMap(const std::map<std::string, std::map<std::string, std::string> > &map,
                         const std::string& tag1,
                         const std::string& tag2,
                         Astar::CollisionAvoidanceOption& param){
    std::string sParam = getParameterString(map, tag1, tag2);
    if(sParam.empty())
        return;
    try{
        param = Astar::intToCollisionAvoidanceOption( std::stoi(sParam) );
    }
    catch(const std::exception&){}
}

}

std::map<std::string, std::map<std::string, std::string> >
    getParametersAsMap(const HeadlandPlanner::PlannerParameters &headlandPlannerParameters,
                       const TracksGenerator::TracksGeneratorParameters &tracksGeneratorParameters,
                       const BaseRoutesInfieldPlanner::PlannerParameters &baseRoutesPlannerParameters,
                       const FieldProcessPlanner::PlannerParameters &fieldProcessPlannerParameters)
{
    std::map<std::string, std::map<std::string, std::string> > map;
    std::map<std::string, std::string> subMap;
    std::string tag1;

    tag1 = "headlandPlannerParameters";
    subMap = HeadlandPlanner::PlannerParameters::parseToStringMap(headlandPlannerParameters);
    map[tag1].insert(subMap.begin(), subMap.end());

    tag1 = "tracksGeneratorParameters";
    subMap = TracksGenerator::TracksGeneratorParameters::parseToStringMap(tracksGeneratorParameters);
    map[tag1].insert(subMap.begin(), subMap.end());

    tag1 = "baseRoutesPlannerParameters";
    subMap = BaseRoutesInfieldPlanner::PlannerParameters::parseToStringMap(baseRoutesPlannerParameters);
    map[tag1].insert(subMap.begin(), subMap.end());

    tag1 = "fieldProcessPlannerParameters";
    subMap = FieldProcessPlanner::PlannerParameters::parseToStringMap(fieldProcessPlannerParameters);
    map[tag1].insert(subMap.begin(), subMap.end());

    return map;
}

bool setParametersFromMap(const std::map<std::string, std::map<std::string, std::string> > &map,
                          HeadlandPlanner::PlannerParameters &headlandPlannerParameters,
                          TracksGenerator::TracksGeneratorParameters &tracksGeneratorParameters,
                          BaseRoutesInfieldPlanner::PlannerParameters &baseRoutesPlannerParameters,
                          FieldProcessPlanner::PlannerParameters &fieldProcessPlannerParameters)
{
    try{
        std::string tag1;
        bool ok = true;

        tag1 = "headlandPlannerParameters";
        if(map.find(tag1) != map.end())
            ok &= HeadlandPlanner::PlannerParameters::parseFromStringMap(headlandPlannerParameters, map.at(tag1), false);

        tag1 = "tracksGeneratorParameters";
        if(map.find(tag1) != map.end())
            ok &= TracksGenerator::TracksGeneratorParameters::parseFromStringMap(tracksGeneratorParameters, map.at(tag1), false);

        tag1 = "baseRoutesPlannerParameters";
        if(map.find(tag1) != map.end())
            ok &= BaseRoutesInfieldPlanner::PlannerParameters::parseFromStringMap(baseRoutesPlannerParameters, map.at(tag1), false);

        tag1 = "fieldProcessPlannerParameters";
        if(map.find(tag1) != map.end())
            ok &= FieldProcessPlanner::PlannerParameters::parseFromStringMap(fieldProcessPlannerParameters, map.at(tag1), false);

        return ok;

    }
    catch(...){
        return false;
    }


}

}
