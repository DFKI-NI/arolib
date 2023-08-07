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
 
#include "arolib/analysis/harvesterplananalyser.hpp"
namespace arolib {

using namespace arolib::geometry;

const double HarvesterPlanAnalyser::m_harvestYieldRateThreshold = 0.1;


HarvesterPlanAnalyser::HarvesterPlanAnalyser(const LogLevel &logLevel):
    PlanAnalyserBase(logLevel, __FUNCTION__)
{

}

bool HarvesterPlanAnalyser::runAnalysis()
{
    if(!m_fieldReady){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "No valid field has been set");
        return false;
    }
    if(m_routes.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "No routes have been set");
        return false;
    }

    m_analysisReady = false;
    m_analysisResult.clear();

    for(auto &it_sf : m_routes){
        if(it_sf.first >= m_field.subfields.size()){//for now, the key is the index in the vector
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Saved routes have an invalid subfield id/key");
            return false;
        }
        for(auto &it_m : it_sf.second){
            AnalysisResult analysisResult;
            if( !getRouteSegmentsIndexes( m_field.subfields.at(it_sf.first), it_m.second.first, analysisResult.routeSegmentation_1 ) ){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining the segments' indexes from route 1 (subfield " + std::to_string(it_sf.first) + ", machineId " + std::to_string(it_m.first) + ")");
                return false;
            }
            if( !getRouteSegmentsIndexes( m_field.subfields.at(it_sf.first), it_m.second.second, analysisResult.routeSegmentation_2 ) ){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining the segments' indexes from route 2 (subfield " + std::to_string(it_sf.first) + ", machineId " + std::to_string(it_m.first) + ")");
                return false;
            }

            if( !analyseSegments( it_m.second.first, it_m.second.second, analysisResult ) ){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error running analysis of segments (subfieldId " + std::to_string(it_sf.first) + ", machineId " + std::to_string(it_m.first) + ")");
                return false;
            }

            m_analysisResult[it_sf.first][it_m.first] = analysisResult;
        }
    }

    m_analysisReady = true;
    return true;
}


const std::map<size_t, std::map<MachineId_t, HarvesterPlanAnalyser::AnalysisResult> > &HarvesterPlanAnalyser::getResults()
{
    return m_analysisResult;
}

const std::map<MachineId_t, HarvesterPlanAnalyser::AnalysisResult> &HarvesterPlanAnalyser::getResults(size_t subfieldId)
{
    return m_analysisResult[subfieldId];
}

const HarvesterPlanAnalyser::AnalysisResult &HarvesterPlanAnalyser::getResults(size_t subfieldId, const MachineId_t &machineId)
{
    return m_analysisResult[subfieldId][machineId];
}

bool HarvesterPlanAnalyser::saveResults_CSV(const std::string &filename) const
{
    std::ofstream of(filename);
    if (!of.is_open()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error opening file " + filename);
        return false;
    }

    //of << std::setprecision(10);
    of << std::fixed;

    for(auto &it_sf : m_analysisResult){
        of << "+ Subfield " << it_sf.first << std::endl;
        for(auto &it_m : it_sf.second){
            const AnalysisResult& results = it_m.second;
            of << sep << "+ Machine " << it_m.first << std::endl;

            of << sep << sep << sep
               << "r1" << sep << "r2" << sep << "r1-r2" << sep
               << "r1" << sep << "r2" << sep << "r1-r2" << sep
               << "r1" << sep << "r2" << sep << "r1-r2" << sep
               << "r1" << sep << "r2" << sep << "r1-r2" << sep
               << "r1" << sep << "r2" << sep << "r1-r2" << sep
               << std::endl << std::endl;

            of << sep << sep << "SEGMENTS RESULTS" << sep
               << "Init" << sep << sep << sep
               << "End" << sep << sep << sep
               << "Field" << sep  << sep << sep
               << "Headland" << sep  << sep << sep
               << "Infield" << sep  << sep << sep
               << std::endl;

            of << sep << sep << "Duration [s]" << sep;
            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::DURATION);

            of << sep << sep << "Time start [s]" << sep;
            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::TIME_START);

            of << sep << sep << "Time end [s]" << sep;
            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::TIME_END);

            of << sep << sep << "Distance [m]" << sep;
            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::DISTANCE);

            of << sep << sep << "Speed avg [m/s]" << sep;
            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::SPEED_AVG);

            of << sep << sep << "Speed min [m/s]" << sep;
            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::SPEED_MIN);

            of << sep << sep << "Speed max [m/s]" << sep;
            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::SPEED_MAX);

            of << sep << sep << "Speed (harv) min [m/s]" << sep;
            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::SPEED_HARV_MIN);

            of << sep << sep << "Speed (harv) max [m/s]" << sep;
            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::SPEED_HARV_MAX);

            of << sep << sep << "Mass total [Kg]" << sep;
            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::MASS_TOTAL);

            of << sep << sep << "Mass/Time avg [Kg/s]" << sep;
            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::MASS_PER_TIME);

            of << sep << sep << "Mass/Distance avg [Kg/m]" << sep;
            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::MASS_PER_DISTANCE);

            of << std::endl;

            of << sep << sep << "INFIELD SUB-SEGMENTS RESULTS" << std::endl << std::endl;
            of << sep << sep << "GENERAL" << sep
               << "Infield" << sep << sep << sep
               << "Headland" << sep << sep << sep
               << std::endl;

            of << sep << sep << "# tracks" << sep
                             << results.infieldAnalysis_1.infieldSubSegments.size() << sep
                             << results.infieldAnalysis_2.infieldSubSegments.size() << sep
                             << (int)results.infieldAnalysis_1.infieldSubSegments.size() - results.infieldAnalysis_2.infieldSubSegments.size() << sep
                             << results.infieldAnalysis_1.headlandSubSegments.size() << sep
                             << results.infieldAnalysis_2.headlandSubSegments.size() << sep
                             << (int)results.infieldAnalysis_1.headlandSubSegments.size() - results.infieldAnalysis_2.headlandSubSegments.size() << std::endl;

            of << sep << sep << "Duration total [s]" << sep;
            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DURATION_TOTAL);

            of << sep << sep << "Duration min [s]" << sep;
            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DURATION_MIN);

            of << sep << sep << "Duration max [s]" << sep;
            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DURATION_MAX);

            of << sep << sep << "Duration avg [s]" << sep;
            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DURATION_AVG);

            of << sep << sep << "Distance total [m]" << sep;
            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DISTANCE_TOTAL);

            of << sep << sep << "Distance min [m]" << sep;
            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DISTANCE_MIN);

            of << sep << sep << "Distance max [m]" << sep;
            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DISTANCE_MAX);

            of << sep << sep << "Distance avg [m]" << sep;
            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DISTANCE_AVG);

            of << sep << sep << "Speed min [m/s]" << sep;
            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::SPEED_MIN);

            of << sep << sep << "Speed max [m/s]" << sep;
            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::SPEED_MAX);

            of << sep << sep << "Speed avg [m/s]" << sep;
            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::SPEED_AVG);

            of << sep << sep << "Speed (harv) min [m/s]" << sep;
            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::SPEED_HARV_MIN);

            of << sep << sep << "Speed (harv) max [m/s]" << sep;
            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::SPEED_HARV_MAX);

            of << std::endl;

            of << sep << sep << "INFIELD TRACKS" << sep
               << "Infield" << sep << sep << sep
               << "Headland" << sep << sep << sep
               << std::endl;

            size_t maxSize = std::max( results.infieldAnalysis_2.infieldSubSegments.size() ,
                                       std::max( results.infieldAnalysis_1.infieldSubSegments.size() ,
                                                     std::max( results.infieldAnalysis_2.headlandSubSegments.size() ,
                                                               results.infieldAnalysis_1.headlandSubSegments.size() ) ) );
            for(size_t i = 0; i < maxSize; ++i){

                of << sep << sep << i+1 << std::endl;

                of << sep << sep << "Duration [s]" << sep;
                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::DURATION);

                of << sep << sep << "Time start [s]" << sep;
                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::TIME_START);

                of << sep << sep << "Time end [s]" << sep;
                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::TIME_END);

                of << sep << sep << "Distance [m]" << sep;
                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::DISTANCE);

                of << sep << sep << "Speed avg [m/s]" << sep;
                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::SPEED_AVG);

                of << sep << sep << "Speed min [m/s]" << sep;
                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::SPEED_MIN);

                of << sep << sep << "Speed max [m/s]" << sep;
                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::SPEED_MAX);

                of << sep << sep << "Speed (harv) min [m/s]" << sep;
                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::SPEED_HARV_MIN);

                of << sep << sep << "Speed (harv) max [m/s]" << sep;
                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::SPEED_HARV_MAX);

                of << sep << sep << "Mass total [Kg]" << sep;
                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::MASS_TOTAL);

                of << sep << sep << "Mass/Time avg [Kg/s]" << sep;
                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::MASS_PER_TIME);

                of << sep << sep << "Mass/Distance avg [Kg/m]" << sep;
                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::MASS_PER_DISTANCE);
            }

            of << sep << "- Machine " << it_m.first << std::endl;
        }
        of << "- Subfield " << it_sf.first << std::endl;
    }


    of.close();
    return true;
}

bool HarvesterPlanAnalyser::getResults(std::map<size_t, std::map<MachineId_t, AnalysisResult> > &results)
{
    results.clear();
    if(!m_analysisReady)
        return false;
    results = m_analysisResult;
    return true;
}

bool HarvesterPlanAnalyser::getResults(size_t subfieldId, std::map<MachineId_t, HarvesterPlanAnalyser::AnalysisResult> &results)
{
    results.clear();
    if(!m_analysisReady){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Results not ready");
        return false;
    }
    auto it_sf = m_analysisResult.find(subfieldId);
    if(it_sf == m_analysisResult.end()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfieldId");
        return false;
    }
    results = it_sf->second;
    return true;
}

bool HarvesterPlanAnalyser::getResults(size_t subfieldId, const MachineId_t &machineId, HarvesterPlanAnalyser::AnalysisResult &results)
{
    if(!m_analysisReady){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Results not ready");
        return false;
    }
    auto it_sf = m_analysisResult.find(subfieldId);
    if(it_sf == m_analysisResult.end()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfieldId");
        return false;
    }
    auto it_m = it_sf->second.find(machineId);
    if(it_m == it_sf->second.end()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid machineId for the given subfield");
        return false;
    }
    results = it_m->second;
    return true;

}

bool HarvesterPlanAnalyser::getRouteSegmentsIndexes(const Subfield& subfield, const Route &route, RouteSegmentationResults &segResults) const
{

    for(size_t i = 0 ; i < route.route_points.size() ; ++i){
        auto &rp = route.route_points.at(i);
        if( rp.type == RoutePoint::TRACK_START )//the routes come from a plan --> use the routepoint types for the segmentation
            return getRouteSegmentsIndexes_informed(subfield, route, segResults);
    }

    segResults.indexRange_init.min = segResults.indexRange_init.max = -1;
    segResults.indexRange_end.min = segResults.indexRange_end.max = -1;
    segResults.indexRange_field.min = segResults.indexRange_field.max = -1;
    segResults.indexRange_headland.min = segResults.indexRange_headland.max = -1;
    segResults.indexRange_infield.min = segResults.indexRange_infield.max = -1;
    segResults.indexRange_infield__headland.clear();
    segResults.indexRange_infield__infield.clear();

    if(route.route_points.empty()){
        //logger().printOut(LogLevel::ERROR, __FUNCTION__, "Route is empty");
        //return false;
        return true;
    }

    Polygon innerBoundary_ed_in;
    if(! arolib::geometry::offsetPolygon( subfield.boundary_inner, innerBoundary_ed_in, 2, false)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting subfield inner boundary (inwards)");
        return false;
    }


    segResults.indexRange_init.min = segResults.indexRange_end.min = 0;
    segResults.indexRange_init.max = segResults.indexRange_end.max = route.route_points.size()-1;

    for(size_t i = 0 ; i < route.route_points.size() ; ++i){
        auto &rp = route.route_points.at(i);
        if( arolib::geometry::in_polygon( rp, subfield.boundary_outer) ){
            segResults.indexRange_init.max = i;
            break;
        }
    }

    for(int i = route.route_points.size()-1 ; i >= 0 ; --i){
        auto &rp = route.route_points.at(i);
        if( arolib::geometry::in_polygon( rp, subfield.boundary_outer) ){
            segResults.indexRange_end.min = i;
            break;
        }
    }

    for(size_t i = segResults.indexRange_init.max ; i <= segResults.indexRange_end.min ; ++i){
        auto &rp = route.route_points.at(i);
        if( arolib::geometry::in_polygon( rp, innerBoundary_ed_in) ){
            segResults.indexRange_infield.min = i;
            break;
        }
    }
    if(segResults.indexRange_infield.min >= 0){
        for(int i = segResults.indexRange_end.min ; i >= segResults.indexRange_infield.min ; --i){
            auto &rp = route.route_points.at(i);
            if( arolib::geometry::in_polygon( rp, innerBoundary_ed_in) ){
                segResults.indexRange_infield.max = i;
                break;
            }
        }
    }


    if(segResults.indexRange_infield.min < 0){
        if( segResults.indexRange_end.min == 0 ){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "No (in-) field route points were obtained from the route)");
            return false;
        }
        segResults.indexRange_headland.min = segResults.indexRange_init.max;
        segResults.indexRange_headland.max = segResults.indexRange_end.min;
        return true;
    }
    if(segResults.indexRange_infield.min != segResults.indexRange_init.max){
        segResults.indexRange_headland.min = segResults.indexRange_init.max;
        segResults.indexRange_headland.max = segResults.indexRange_infield.min;
    }

    segResults.indexRange_field.min = segResults.indexRange_init.max;
    segResults.indexRange_field.max = segResults.indexRange_end.min;


    if(! arolib::geometry::offsetPolygon( subfield.boundary_inner, innerBoundary_ed_in, 1, false)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting subfield inner boundary (inwards-2)");
        return false;
    }
    Polygon innerBoundary_ed_out;
    if(! arolib::geometry::offsetPolygon( subfield.boundary_inner, innerBoundary_ed_out, 1, true)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting subfield inner boundary (outwards)");
        return false;
    }

    IndexRange subRange;
    subRange.min = segResults.indexRange_infield.min;
    bool checkIF = true;
    for(size_t i = segResults.indexRange_infield.min ; i <= segResults.indexRange_infield.max ; ++i){
        auto &rp = route.route_points.at(i);
        if(checkIF){
            if( !arolib::geometry::in_polygon( rp, innerBoundary_ed_out) ){
                subRange.max = i-1;
                if(subRange.min <= subRange.max)
                    segResults.indexRange_infield__infield.push_back(subRange);
                subRange.min = i-1;
                checkIF = false;
            }
        }
        else{
            if( arolib::geometry::in_polygon( rp, innerBoundary_ed_in) ){
                subRange.max = i-1;
                if(subRange.min <= subRange.max)
                    segResults.indexRange_infield__headland.push_back(subRange);
                subRange.min = i-1;
                checkIF = true;
            }
        }
    }

    subRange.max = segResults.indexRange_infield.max;
    if(subRange.min <= subRange.max){
        if(checkIF)
            segResults.indexRange_infield__infield.push_back(subRange);
        else
            segResults.indexRange_end.min = subRange.min;
    }

    return true;
}

bool HarvesterPlanAnalyser::getRouteSegmentsIndexes_informed(const Subfield &subfield, const Route &route, HarvesterPlanAnalyser::RouteSegmentationResults &segResults) const
{
    bool useInnerBoundary = false;

    segResults.indexRange_init.min = segResults.indexRange_init.max = -1;
    segResults.indexRange_end.min = segResults.indexRange_end.max = -1;
    segResults.indexRange_field.min = segResults.indexRange_field.max = -1;
    segResults.indexRange_headland.min = segResults.indexRange_headland.max = -1;
    segResults.indexRange_infield.min = segResults.indexRange_infield.max = -1;
    segResults.indexRange_infield__headland.clear();
    segResults.indexRange_infield__infield.clear();

    //Inflate a bit the inner boundary so that inner-field route points (except inter-track connections through the headland) are inside the boundary for sure
    Polygon innerBoundary_ed_out;
    if(useInnerBoundary){
        if(! arolib::geometry::offsetPolygon( subfield.boundary_inner, innerBoundary_ed_out, 1, true)){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting subfield inner boundary (outwards)");
            return false;
        }
    }

    segResults.indexRange_init.min = segResults.indexRange_end.min = 0;
    segResults.indexRange_init.max = segResults.indexRange_end.max = route.route_points.size()-1;

    for(size_t i = 0 ; i < route.route_points.size() ; ++i){
        auto &rp = route.route_points.at(i);
        if( rp.type == RoutePoint::TRACK_START || rp.type == RoutePoint::HARVESTING ){
            if(useInnerBoundary){
                if( !arolib::geometry::in_polygon( rp, innerBoundary_ed_out) )//there is headland harvesting
                    segResults.indexRange_headland.min = i;
                else//there is NO headland harvesting, only inner-field harvesting
                    segResults.indexRange_infield.min = i;
            }
            else{
                if( Track::isHeadlandTrack(rp.track_id) )//there is headland harvesting
                    segResults.indexRange_headland.min = i;
                else//there is NO headland harvesting, only inner-field harvesting
                    segResults.indexRange_infield.min = i;
            }

            segResults.indexRange_init.max = i;
            break;
        }
    }

    for(int i = route.route_points.size()-1 ; i >= 0 ; --i){
        auto &rp = route.route_points.at(i);
        if( rp.type == RoutePoint::TRACK_END || rp.type == RoutePoint::HARVESTING ){
            if(useInnerBoundary){
                if( !arolib::geometry::in_polygon( rp, innerBoundary_ed_out) )
                    segResults.indexRange_headland.max = i;
                else
                    segResults.indexRange_infield.max = i;
            }
            else{
                if( Track::isHeadlandTrack(rp.track_id) )
                    segResults.indexRange_headland.max = i;
                else
                    segResults.indexRange_infield.max = i;
            }

            segResults.indexRange_end.min = i;
            break;
        }
    }

    if(segResults.indexRange_infield.max < 0){//plan has only headland harvesting
        if(segResults.indexRange_headland.min < 0){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing headland/infield segmentation using TRACK_START (no headland and no infield harvesting?)");
            return false;
        }
        return true;
    }

    if(segResults.indexRange_headland.min >= 0){//plan has headland harvesting
        for(size_t i = segResults.indexRange_headland.min ; i < route.route_points.size() ; ++i){
            auto &rp = route.route_points.at(i);

            if(useInnerBoundary){
                if( i != 0
                        && rp.type == RoutePoint::TRACK_START
                        && route.route_points.at(i-1).type != RoutePoint::TRACK_END
                        && arolib::geometry::in_polygon( rp, innerBoundary_ed_out) ){
                    segResults.indexRange_headland.max = i;
                    segResults.indexRange_infield.min = i;
                    break;
                }
            }
            else{
                if( i != 0
                        && rp.type == RoutePoint::TRACK_START
                        && Track::isInfieldTrack(rp.track_id) ){
                    segResults.indexRange_headland.max = i;
                    segResults.indexRange_infield.min = i;
                    break;
                }

            }
        }
    }

    segResults.indexRange_field.min = segResults.indexRange_init.max;
    segResults.indexRange_field.max = segResults.indexRange_end.min;

    IndexRange subRange;
    subRange.min = segResults.indexRange_infield.min;
    bool checkIF = true;
    for(size_t i = segResults.indexRange_infield.min ; i <= segResults.indexRange_infield.max ; ++i){
        auto &rp = route.route_points.at(i);
        if(checkIF && rp.type == RoutePoint::TRACK_END ){
            subRange.max = i;
            if(subRange.min <= subRange.max)
                segResults.indexRange_infield__infield.push_back(subRange);
            subRange.min = i;
            checkIF = false;
        }
        else if(!checkIF && rp.type == RoutePoint::TRACK_START ){
            subRange.max = i;
            if(subRange.min <= subRange.max)
                segResults.indexRange_infield__headland.push_back(subRange);
            subRange.min = i;
            checkIF = true;
        }
    }
    if(checkIF && subRange.min <= segResults.indexRange_infield.max){
        subRange.max = segResults.indexRange_infield.max;
        segResults.indexRange_infield__infield.push_back(subRange);
    }

    return true;
}

bool HarvesterPlanAnalyser::analyseSegments(const Route &route1, const Route &route2, AnalysisResult &analysisResult)
{
    std::string routeNr = "1";
    if(!analyseSegment(route1, analysisResult.routeSegmentation_1.indexRange_field, analysisResult.fieldAnalysis_1)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the (complete) field segment from route " + routeNr);
        return false;
    }
    if(!analyseSegment(route1, analysisResult.routeSegmentation_1.indexRange_init, analysisResult.initAnalysis_1)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the init segment from route " + routeNr);
        return false;
    }
    if(!analyseSegment(route1, analysisResult.routeSegmentation_1.indexRange_end, analysisResult.endAnalysis_1)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the end segment from route " + routeNr);
        return false;
    }
    if(!analyseSegment(route1, analysisResult.routeSegmentation_1.indexRange_headland, analysisResult.headlandAnalysis_1)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the headland segment from route " + routeNr);
        return false;
    }
    if(!analyseSegment(route1, analysisResult.routeSegmentation_1.indexRange_infield, analysisResult.infieldAnalysis_1)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the infield segment from route " + routeNr);
        return false;
    }
    if(!analyseSubSegments(route1,
                           analysisResult.routeSegmentation_1.indexRange_infield__headland,
                           analysisResult.infieldAnalysis_1.headlandSegments,
                           analysisResult.infieldAnalysis_1.headlandSubSegments)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the infield-headland subsegments from route " + routeNr);
        return false;
    }
    if(!analyseSubSegments(route1,
                           analysisResult.routeSegmentation_1.indexRange_infield__infield,
                           analysisResult.infieldAnalysis_1.infieldSegments,
                           analysisResult.infieldAnalysis_1.infieldSubSegments)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the infield-infield subsegments from route " + routeNr);
        return false;
    }

    routeNr = "2";
    if(!analyseSegment(route2, analysisResult.routeSegmentation_2.indexRange_field, analysisResult.fieldAnalysis_2)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the (complete) field segment from route " + routeNr);
        return false;
    }
    if(!analyseSegment(route2, analysisResult.routeSegmentation_2.indexRange_init, analysisResult.initAnalysis_2)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the init segment from route " + routeNr);
        return false;
    }
    if(!analyseSegment(route2, analysisResult.routeSegmentation_2.indexRange_end, analysisResult.endAnalysis_2)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the end segment from route " + routeNr);
        return false;
    }
    if(!analyseSegment(route2, analysisResult.routeSegmentation_2.indexRange_headland, analysisResult.headlandAnalysis_2)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the headland segment from route " + routeNr);
        return false;
    }
    if(!analyseSegment(route2, analysisResult.routeSegmentation_2.indexRange_infield, analysisResult.infieldAnalysis_2)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the infield segment from route " + routeNr);
        return false;
    }
    if(!analyseSubSegments(route2,
                           analysisResult.routeSegmentation_2.indexRange_infield__headland,
                           analysisResult.infieldAnalysis_2.headlandSegments,
                           analysisResult.infieldAnalysis_2.headlandSubSegments)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the infield-headland subsegments from route " + routeNr);
        return false;
    }
    if(!analyseSubSegments(route2,
                           analysisResult.routeSegmentation_2.indexRange_infield__infield,
                           analysisResult.infieldAnalysis_2.infieldSegments,
                           analysisResult.infieldAnalysis_2.infieldSubSegments)){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the infield-infield subsegments from route " + routeNr);
        return false;
    }

    return true;
}

bool HarvesterPlanAnalyser::analyseSegment(const Route &route, const HarvesterPlanAnalyser::IndexRange &indexRange, HarvesterPlanAnalyser::FieldSegmentAnalysisResult &analysisResult)
{

    analysisResult.distance = 0;
    analysisResult.duration = 0;
    analysisResult.speed_avg = 0;
    analysisResult.speed_max = std::numeric_limits<double>::lowest();
    analysisResult.speed_min = std::numeric_limits<double>::max();
    analysisResult.speed_harv_max = std::numeric_limits<double>::lowest();
    analysisResult.speed_harv_min = std::numeric_limits<double>::max();
    analysisResult.yieldMass_total = 0;
    analysisResult.yieldMassPerTime_avg = 0;
    if(indexRange.min < 0 && indexRange.max < 0){
        analysisResult.timeStart = -1;
        analysisResult.timeEnd = -1;
        return true;
    }

    if(indexRange.max < indexRange.min
            || indexRange.min < 0
            || indexRange.max >= route.route_points.size()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid index-range");
        return false;
    }

    auto &rp1 = route.route_points.at(indexRange.min);
    auto &rp2 = route.route_points.at(indexRange.max);

    analysisResult.timeStart = rp1.time_stamp;
    analysisResult.timeEnd = rp2.time_stamp;

    if(indexRange.min != indexRange.max){
        for(size_t i = indexRange.min ; i < indexRange.max ; ++i){
            auto &rp1 = route.route_points.at(i);
            auto &rp2 = route.route_points.at(i+1);
            double dist = arolib::geometry::calc_dist(rp1, rp2);
            double deltaTime = rp2.time_stamp - rp1.time_stamp;
            if(deltaTime != 0){
                double speed = dist / deltaTime;
                analysisResult.speed_max = std::max(analysisResult.speed_max, speed);
                analysisResult.speed_min = std::min(analysisResult.speed_min, speed);

                if(dist > 0.05){
                    double harvYield = std::fabs(rp2.worked_mass - rp1.worked_mass);
                    double harvYieldRate = harvYield / dist;

                    if(harvYieldRate > m_harvestYieldRateThreshold){
                        analysisResult.speed_harv_max = std::max(analysisResult.speed_harv_max, speed);
                        analysisResult.speed_harv_min = std::min(analysisResult.speed_harv_min, speed);
                    }
                }
            }
            analysisResult.distance += dist;
        }

        analysisResult.duration = rp2.time_stamp - rp1.time_stamp;
        if(analysisResult.duration != 0)
            analysisResult.speed_avg = analysisResult.distance/analysisResult.duration;
    }
    if(analysisResult.speed_min > analysisResult.speed_max)
        analysisResult.speed_min = analysisResult.speed_max = 0;
    if(analysisResult.speed_harv_min > analysisResult.speed_harv_max)
        analysisResult.speed_harv_min = analysisResult.speed_harv_max = 0;

    analysisResult.yieldMass_total = route.route_points.at(indexRange.max).worked_mass - route.route_points.at(indexRange.min).worked_mass;
    if(analysisResult.duration != 0)
        analysisResult.yieldMassPerTime_avg = analysisResult.yieldMass_total /(analysisResult.duration);
    if(analysisResult.distance != 0)
        analysisResult.yieldMassPerTime_avg = analysisResult.yieldMass_total /(analysisResult.distance);


    return true;
}

bool HarvesterPlanAnalyser::analyseSubSegments(const Route &route,
                                               const std::vector<HarvesterPlanAnalyser::IndexRange> &indexRanges,
                                               InfieldAnalysisResult::InfieldSegmentsAnalysisResult  &analysisSegment,
                                               std::vector<FieldSegmentAnalysisResult> &analysisSubSegments)
{
    analysisSubSegments = std::vector<FieldSegmentAnalysisResult>( indexRanges.size() );
    analysisSegment = InfieldAnalysisResult::InfieldSegmentsAnalysisResult();

    if(indexRanges.empty())
        return true;

    for(size_t i = 0 ; i < indexRanges.size() ; ++i){
        if(!analyseSegment(route, indexRanges.at(i), analysisSubSegments.at(i))){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing subsegment " + std::to_string(i) );
            analysisSubSegments.clear();
            return false;
        }
    }

    analysisSegment.distance_total = analysisSegment.duration_total = 0;
    analysisSegment.distance_avg = analysisSegment.duration_avg = analysisSegment.speed_avg = 0;
    analysisSegment.distance_min = analysisSegment.duration_min = analysisSegment.speed_min = std::numeric_limits<double>::max();
    analysisSegment.distance_max = analysisSegment.duration_max = analysisSegment.speed_max = std::numeric_limits<double>::lowest();

    for(auto &s : analysisSubSegments){
        analysisSegment.distance_min = std::min( analysisSegment.distance_min, s.distance );
        analysisSegment.distance_max = std::max( analysisSegment.distance_max, s.distance );
        analysisSegment.distance_total += s.distance;

        analysisSegment.duration_min = std::min( analysisSegment.duration_min, s.duration );
        analysisSegment.duration_max = std::max( analysisSegment.duration_max, s.duration );
        analysisSegment.duration_total += s.duration;

        analysisSegment.speed_min = std::min( analysisSegment.speed_min, s.speed_min );
        analysisSegment.speed_max = std::max( analysisSegment.speed_max, s.speed_max );
        analysisSegment.speed_avg += s.speed_avg;

        if(s.speed_harv_min > 0)
            analysisSegment.speed_harv_min = std::min( analysisSegment.speed_harv_min, s.speed_harv_min );
        if(s.speed_harv_max > 0)
            analysisSegment.speed_harv_max = std::max( analysisSegment.speed_harv_max, s.speed_harv_max );
    }
    analysisSegment.distance_avg = analysisSegment.distance_total / analysisSubSegments.size();
    analysisSegment.duration_avg = analysisSegment.duration_total / analysisSubSegments.size();
    analysisSegment.speed_avg = analysisSegment.speed_avg / analysisSubSegments.size();

    if(analysisSegment.distance_min > analysisSegment.distance_max)
        analysisSegment.distance_min = analysisSegment.distance_max = 0;

    if(analysisSegment.duration_min > analysisSegment.duration_max)
        analysisSegment.duration_min = analysisSegment.duration_max = 0;

    if(analysisSegment.speed_min > analysisSegment.speed_max)
        analysisSegment.speed_min = analysisSegment.speed_max = 0;

    if(analysisSegment.speed_harv_min > analysisSegment.speed_harv_max)
        analysisSegment.speed_harv_min = analysisSegment.speed_harv_max = 0;

    return true;
}

void HarvesterPlanAnalyser::addSegmentResultsToCSV(std::ofstream &of, const AnalysisResult &results, FieldSegmentAnalysisResult::ValueType valueType) const
{
    of /*<< std::setprecision(10)*/
       << results.initAnalysis_1.getValue(valueType) << sep
       << results.initAnalysis_2.getValue(valueType) << sep
       << results.initAnalysis_1.getValue(valueType) - results.initAnalysis_2.getValue(valueType) << sep
       << results.endAnalysis_1.getValue(valueType) << sep
       << results.endAnalysis_2.getValue(valueType) << sep
       << results.endAnalysis_1.getValue(valueType) - results.endAnalysis_2.getValue(valueType) << sep
       << results.fieldAnalysis_1.getValue(valueType) << sep
       << results.fieldAnalysis_2.getValue(valueType) << sep
       << results.fieldAnalysis_1.getValue(valueType) - results.fieldAnalysis_2.getValue(valueType) << sep
       << results.headlandAnalysis_1.getValue(valueType) << sep
       << results.headlandAnalysis_2.getValue(valueType) << sep
       << results.headlandAnalysis_1.getValue(valueType) - results.headlandAnalysis_2.getValue(valueType) << sep
       << results.infieldAnalysis_1.getValue(valueType) << sep
       << results.infieldAnalysis_2.getValue(valueType) << sep
       << results.infieldAnalysis_1.getValue(valueType) - results.infieldAnalysis_2.getValue(valueType) << std::endl;

}

void HarvesterPlanAnalyser::addInfieldSubsegmentResultsToCSV(std::ofstream &of, const AnalysisResult &results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::ValueType valueType) const
{
    of /*<< std::setprecision(10)*/
       << results.infieldAnalysis_1.infieldSegments.getValue(valueType) << sep
       << results.infieldAnalysis_2.infieldSegments.getValue(valueType) << sep
       << results.infieldAnalysis_1.infieldSegments.getValue(valueType) - results.infieldAnalysis_2.infieldSegments.getValue(valueType) << sep
       << results.infieldAnalysis_1.headlandSegments.getValue(valueType) << sep
       << results.infieldAnalysis_2.headlandSegments.getValue(valueType) << sep
       << results.infieldAnalysis_1.headlandSegments.getValue(valueType) - results.infieldAnalysis_2.headlandSegments.getValue(valueType) << std::endl;

}

void HarvesterPlanAnalyser::addInfieldSubsegmentResultsToCSV(std::ofstream &of, const HarvesterPlanAnalyser::AnalysisResult &results, size_t ind, FieldSegmentAnalysisResult::ValueType valueType) const
{
//    of << std::setprecision(10);
    double val1 = 0 , val2 = 0;
    if(ind < results.infieldAnalysis_1.infieldSubSegments.size()){
        val1 = results.infieldAnalysis_1.infieldSubSegments.at(ind).getValue(valueType);
        of << val1;
    }
    of << sep;
    if(ind < results.infieldAnalysis_2.infieldSubSegments.size()){
        val2 = results.infieldAnalysis_2.infieldSubSegments.at(ind).getValue(valueType);
        of << val2;
    }
    of << sep
       << val1 - val2 << sep;
    val1 = val2 = 0;
    if(ind < results.infieldAnalysis_1.headlandSubSegments.size()){
        val1 = results.infieldAnalysis_1.headlandSubSegments.at(ind).getValue(valueType);
        of << val1;
    }
    of << sep;
    if(ind < results.infieldAnalysis_2.headlandSubSegments.size()){
        val2 = results.infieldAnalysis_2.headlandSubSegments.at(ind).getValue(valueType);
        of << val2;
    }
    of << sep
       << val1 - val2 << std::endl;

}

double HarvesterPlanAnalyser::FieldSegmentAnalysisResult::getValue(ValueType type) const
{
    switch (type) {
    case ValueType::TIME_START:
        return timeStart;
    case ValueType::TIME_END:
        return timeEnd;
    case ValueType::DURATION:
        return duration;
    case ValueType::DISTANCE:
        return distance;
    case ValueType::SPEED_AVG:
        return speed_avg;
    case ValueType::SPEED_MIN:
        return speed_min;
    case ValueType::SPEED_MAX:
        return speed_max;
    case ValueType::SPEED_HARV_MIN:
        return speed_harv_min;
    case ValueType::SPEED_HARV_MAX:
        return speed_harv_max;
    case ValueType::MASS_TOTAL:
        return yieldMass_total;
    case ValueType::MASS_PER_TIME:
        return yieldMassPerTime_avg;
    case ValueType::MASS_PER_DISTANCE:
        return yieldMassPerDistance_avg;
    default:
        return 0;
    }

}

double HarvesterPlanAnalyser::InfieldAnalysisResult::InfieldSegmentsAnalysisResult::getValue(ValueType type) const
{
    switch (type) {
    case ValueType::DURATION_TOTAL:
        return duration_total;
    case ValueType::DURATION_MIN:
        return duration_min;
    case ValueType::DURATION_MAX:
        return duration_max;
    case ValueType::DURATION_AVG:
        return duration_avg;
    case ValueType::DISTANCE_TOTAL:
        return distance_total;
    case ValueType::DISTANCE_MIN:
        return distance_min;
    case ValueType::DISTANCE_MAX:
        return distance_max;
    case ValueType::DISTANCE_AVG:
        return distance_avg;
    case ValueType::SPEED_MIN:
        return speed_min;
    case ValueType::SPEED_MAX:
        return speed_max;
    case ValueType::SPEED_AVG:
        return speed_avg;
    case ValueType::SPEED_HARV_MIN:
        return speed_harv_min;
    case ValueType::SPEED_HARV_MAX:
        return speed_harv_max;
    default:
        return 0;
    }

}


}
