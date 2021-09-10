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
 
#include "arolib/analysis/olvplananalyser.hpp"
namespace arolib {

using namespace arolib::geometry;

OlvPlanAnalyser::OlvPlanAnalyser(const LogLevel &logLevel):
    PlanAnalyserBase(logLevel, __FUNCTION__)
{

}

bool OlvPlanAnalyser::runAnalysis()
{
    if(!m_fieldReady){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No valid field has been set");
        return false;
    }
    if(m_routes.empty()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No routes have been set");
        return false;
    }

    m_analysisReady = false;
    m_analysisResult.clear();

    for(auto &it_sf : m_routes){
        if(it_sf.first >= m_field.subfields.size()){//for now, the key is the index in the vector
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Saved routes have an invalid subfield id/key");
            return false;
        }
        for(auto &it_m : it_sf.second){
            AnalysisResult analysisResult;
            if( !getRouteSegmentsIndexes( m_field.subfields.at(it_sf.first), it_m.second.first, analysisResult.routeSegmentation_1 ) ){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining the segments' indexes from route 1 (subfield " + std::to_string(it_sf.first) + ", machineId " + std::to_string(it_m.first) + ")");
                return false;
            }
            if( !getRouteSegmentsIndexes( m_field.subfields.at(it_sf.first), it_m.second.second, analysisResult.routeSegmentation_2 ) ){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error obtaining the segments' indexes from route 2 (subfield " + std::to_string(it_sf.first) + ", machineId " + std::to_string(it_m.first) + ")");
                return false;
            }

            if( !analyseSegments( it_m.second.first, it_m.second.second, analysisResult ) ){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error running analysis of segments (subfieldId " + std::to_string(it_sf.first) + ", machineId " + std::to_string(it_m.first) + ")");
                return false;
            }

            m_analysisResult[it_sf.first][it_m.first] = analysisResult;
        }
    }

    m_analysisReady = true;
    return true;
}


const std::map<size_t, std::map<MachineId_t, OlvPlanAnalyser::AnalysisResult> > &OlvPlanAnalyser::getResults()
{
    return m_analysisResult;
}

const std::map<MachineId_t, OlvPlanAnalyser::AnalysisResult> &OlvPlanAnalyser::getResults(size_t subfieldId)
{
    return m_analysisResult[subfieldId];
}

const OlvPlanAnalyser::AnalysisResult &OlvPlanAnalyser::getResults(size_t subfieldId, const MachineId_t &machineId)
{
    return m_analysisResult[subfieldId][machineId];
}

bool OlvPlanAnalyser::saveResults_CSV(const std::string &filename) const
{
    std::ofstream of(filename);
    if (!of.is_open()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error opening file " + filename);
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

//            of << sep << sep << "SEGMENTS RESULTS" << sep
//               << "Init" << sep << sep << sep
//               << "End" << sep << sep << sep
//               << "Field" << sep  << sep << sep
//               << "Headland" << sep  << sep << sep
//               << "Infield" << sep  << sep << sep
//               << std::endl;

//            of << sep << sep << "Duration [s]" << sep;
//            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::DURATION);

//            of << sep << sep << "Time start [s]" << sep;
//            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::TIME_START);

//            of << sep << sep << "Time end [s]" << sep;
//            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::TIME_END);

//            of << sep << sep << "Distance [m]" << sep;
//            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::DISTANCE);

//            of << sep << sep << "Speed avg [m/s]" << sep;
//            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::SPEED_AVG);

//            of << sep << sep << "Speed min [m/s]" << sep;
//            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::SPEED_MIN);

//            of << sep << sep << "Speed max [m/s]" << sep;
//            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::SPEED_MAX);

//            of << sep << sep << "Mass total [Kg]" << sep;
//            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::MASS_TOTAL);

//            of << sep << sep << "Mass/Time avg [Kg/s]" << sep;
//            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::MASS_PER_TIME);

//            of << sep << sep << "Mass/Distance avg [Kg/m]" << sep;
//            addSegmentResultsToCSV(of, results, FieldSegmentAnalysisResult::MASS_PER_DISTANCE);

//            of << std::endl;

//            of << sep << sep << "INFIELD SUB-SEGMENTS RESULTS" << std::endl << std::endl;
//            of << sep << sep << "GENERAL" << sep
//               << "Infield" << sep << sep << sep
//               << "Headland" << sep << sep << sep
//               << std::endl;

//            of << sep << sep << "# tracks" << sep
//                             << results.infieldAnalysis_1.infieldSubSegments.size() << sep
//                             << results.infieldAnalysis_2.infieldSubSegments.size() << sep
//                             << (int)results.infieldAnalysis_1.infieldSubSegments.size() - results.infieldAnalysis_2.infieldSubSegments.size() << sep
//                             << results.infieldAnalysis_1.headlandSubSegments.size() << sep
//                             << results.infieldAnalysis_2.headlandSubSegments.size() << sep
//                             << (int)results.infieldAnalysis_1.headlandSubSegments.size() - results.infieldAnalysis_2.headlandSubSegments.size() << std::endl;

//            of << sep << sep << "Duration total [s]" << sep;
//            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DURATION_TOTAL);

//            of << sep << sep << "Duration min [s]" << sep;
//            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DURATION_MIN);

//            of << sep << sep << "Duration max [s]" << sep;
//            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DURATION_MAX);

//            of << sep << sep << "Duration avg [s]" << sep;
//            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DURATION_AVG);

//            of << sep << sep << "Distance total [m]" << sep;
//            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DISTANCE_TOTAL);

//            of << sep << sep << "Distance min [m]" << sep;
//            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DISTANCE_MIN);

//            of << sep << sep << "Distance max [m]" << sep;
//            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DISTANCE_MAX);

//            of << sep << sep << "Distance avg [m]" << sep;
//            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::DISTANCE_AVG);

//            of << sep << sep << "Speed min [m/s]" << sep;
//            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::SPEED_MIN);

//            of << sep << sep << "Speed max [m/s]" << sep;
//            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::SPEED_MAX);

//            of << sep << sep << "Speed avg [m/s]" << sep;
//            addInfieldSubsegmentResultsToCSV(of, results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::SPEED_AVG);

//            of << std::endl;

//            of << sep << sep << "INFIELD TRACKS" << sep
//               << "Infield" << sep << sep << sep
//               << "Headland" << sep << sep << sep
//               << std::endl;

//            size_t maxSize = std::max( results.infieldAnalysis_2.infieldSubSegments.size() ,
//                                       std::max( results.infieldAnalysis_1.infieldSubSegments.size() ,
//                                                     std::max( results.infieldAnalysis_2.headlandSubSegments.size() ,
//                                                               results.infieldAnalysis_1.headlandSubSegments.size() ) ) );
//            for(size_t i = 0; i < maxSize; ++i){

//                of << sep << sep << i+1 << std::endl;

//                of << sep << sep << "Duration [s]" << sep;
//                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::DURATION);

//                of << sep << sep << "Time start [s]" << sep;
//                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::TIME_START);

//                of << sep << sep << "Time end [s]" << sep;
//                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::TIME_END);

//                of << sep << sep << "Distance [m]" << sep;
//                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::DISTANCE);

//                of << sep << sep << "Speed avg [m/s]" << sep;
//                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::SPEED_AVG);

//                of << sep << sep << "Speed min [m/s]" << sep;
//                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::SPEED_MIN);

//                of << sep << sep << "Speed max [m/s]" << sep;
//                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::SPEED_MAX);

//                of << sep << sep << "Mass total [Kg]" << sep;
//                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::MASS_TOTAL);

//                of << sep << sep << "Mass/Time avg [Kg/s]" << sep;
//                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::MASS_PER_TIME);

//                of << sep << sep << "Mass/Distance avg [Kg/m]" << sep;
//                addInfieldSubsegmentResultsToCSV(of, results, i, FieldSegmentAnalysisResult::MASS_PER_DISTANCE);
//            }

            of << sep << "- Machine " << it_m.first << std::endl;
        }
        of << "- Subfield " << it_sf.first << std::endl;
    }


    of.close();
    return true;
}

bool OlvPlanAnalyser::getResults(std::map<size_t, std::map<MachineId_t, AnalysisResult> > &results)
{
    results.clear();
    if(!m_analysisReady)
        return false;
    results = m_analysisResult;
    return true;
}

bool OlvPlanAnalyser::getResults(size_t subfieldId, std::map<MachineId_t, OlvPlanAnalyser::AnalysisResult> &results)
{
    results.clear();
    if(!m_analysisReady){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Results not ready");
        return false;
    }
    auto it_sf = m_analysisResult.find(subfieldId);
    if(it_sf == m_analysisResult.end()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfieldId");
        return false;
    }
    results = it_sf->second;
    return true;
}

bool OlvPlanAnalyser::getResults(size_t subfieldId, const MachineId_t &machineId, OlvPlanAnalyser::AnalysisResult &results)
{
    if(!m_analysisReady){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Results not ready");
        return false;
    }
    auto it_sf = m_analysisResult.find(subfieldId);
    if(it_sf == m_analysisResult.end()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfieldId");
        return false;
    }
    auto it_m = it_sf->second.find(machineId);
    if(it_m == it_sf->second.end()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid machineId for the given subfield");
        return false;
    }
    results = it_m->second;
    return true;

}

bool OlvPlanAnalyser::getRouteSegmentsIndexes(const Subfield& subfield, const Route &route, RouteSegmentationResults &segResults) const
{

    for(size_t i = 0 ; i < route.route_points.size() ; ++i){
        auto &rp = route.route_points.at(i);
        if( rp.type == RoutePoint::OVERLOADING )//the routes come from a plan --> use the routepoint types for the segmentation
            return getRouteSegmentsIndexes_informed(subfield, route, segResults);
    }

    segResults.indexRange_init.min = segResults.indexRange_init.max = -1;
    segResults.indexRange_end.min = segResults.indexRange_end.max = -1;
    segResults.indexRanges_field.clear();
    segResults.indexRanges_outfield.clear();

    if(route.route_points.empty()){
        //m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Route is empty");
        //return false;
        return true;
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

    int ind = segResults.indexRange_init.max;
    while(ind < segResults.indexRange_end.min){
        IndexRange fieldIndexes(ind, -1);
        bool searchNext = true;
        while( arolib::geometry::in_polygon( route.route_points.at(ind), subfield.boundary_outer) ){
            ++ind;
            if(ind > segResults.indexRange_end.min){
                searchNext = false;
                break;
            }
        }
        if(searchNext)
            fieldIndexes.max = ind;
        else
            fieldIndexes.max = ind-1;
        if(fieldIndexes.max <= fieldIndexes.min)
            break;

        //add segment indexes
        std::vector<IndexRange> subIndexes = getFieldSubsegmentIndexes(route, fieldIndexes);
        segResults.indexRanges_field.emplace_back( std::make_pair(fieldIndexes, subIndexes) );

        if(!searchNext)
            break;

        IndexRange OutFieldIndexes(ind, -1);

        while( !arolib::geometry::in_polygon( route.route_points.at(ind), subfield.boundary_outer) ){
            ++ind;
            if(ind > segResults.indexRange_end.min){
                searchNext = false;
                break;
            }
        }
        if(searchNext)
            OutFieldIndexes.max = ind;
        else
            OutFieldIndexes.max = ind-1;
        if(OutFieldIndexes.max <= OutFieldIndexes.min)
            break;

        //add segment indexes
        subIndexes = getOutFieldSubsegmentIndexes(route, OutFieldIndexes);
        segResults.indexRanges_outfield.emplace_back( std::make_pair(fieldIndexes, subIndexes) );

        if(!searchNext)
            break;
    }

    return true;
}

bool OlvPlanAnalyser::getRouteSegmentsIndexes_informed(const Subfield &subfield, const Route &route, OlvPlanAnalyser::RouteSegmentationResults &segResults) const
{
    segResults.indexRange_init.min = segResults.indexRange_init.max = -1;
    segResults.indexRange_end.min = segResults.indexRange_end.max = -1;
    segResults.indexRanges_field.clear();
    segResults.indexRanges_outfield.clear();

    segResults.indexRange_init.min = segResults.indexRange_end.min = 0;
    segResults.indexRange_init.max = segResults.indexRange_end.max = route.route_points.size()-1;

    if(route.route_points.size() < 2)
        return false;

    segResults.indexRange_init.min = segResults.indexRange_init.max = 0;
    if( route.route_points.front().type == RoutePoint::INITIAL_POSITION
            && route.route_points.at(1).type == RoutePoint::FIELD_ENTRY ){//machine starts outside the field
        segResults.indexRange_init.max = 1;
    }

    segResults.indexRange_end.min = segResults.indexRange_end.max = route.route_points.size()-1;
    if( route.route_points.back().type == RoutePoint::RESOURCE_POINT ){
        segResults.indexRange_end.min--;
    }


    int ind = segResults.indexRange_init.max;
    while(ind < segResults.indexRange_end.min){
        IndexRange fieldIndexes(ind, -1);
        bool searchNext = true;
        while( route.route_points.at(ind).type != RoutePoint::FIELD_EXIT ){
            ++ind;
            if(ind > segResults.indexRange_end.min){
                searchNext = false;
                break;
            }
        }
        if(searchNext)
            fieldIndexes.max = ind;
        else
            fieldIndexes.max = ind-1;
        if(fieldIndexes.max <= fieldIndexes.min)
            break;

        //add segment indexes
        std::vector<IndexRange> subIndexes = getFieldSubsegmentIndexes_informed(route, fieldIndexes);
        segResults.indexRanges_field.emplace_back( std::make_pair(fieldIndexes, subIndexes) );

        if(!searchNext)
            break;

        IndexRange OutFieldIndexes(ind, -1);

        while( route.route_points.at(ind).type != RoutePoint::FIELD_ENTRY ){
            ++ind;
            if(ind > segResults.indexRange_end.min){
                searchNext = false;
                break;
            }
        }
        if(searchNext)
            OutFieldIndexes.max = ind;
        else
            OutFieldIndexes.max = ind-1;
        if(OutFieldIndexes.max <= OutFieldIndexes.min)
            break;

        //add segment indexes
        subIndexes = getOutFieldSubsegmentIndexes_informed(route, OutFieldIndexes);
        segResults.indexRanges_outfield.emplace_back( std::make_pair(fieldIndexes, subIndexes) );

        if(!searchNext)
            break;
    }

    return true;
}

std::vector<PlanAnalyserBase::IndexRange> OlvPlanAnalyser::getFieldSubsegmentIndexes(const Route &route, const IndexRange &range) const
{
    return getFieldSubsegmentIndexes_informed(route, range);
}

std::vector<PlanAnalyserBase::IndexRange> OlvPlanAnalyser::getFieldSubsegmentIndexes_informed(const Route &route, const IndexRange &range) const
{
    std::vector<PlanAnalyserBase::IndexRange> ret(3);

    size_t indOLStart = range.max + 1;
    for(size_t i = range.min ; i <= range.max ; ++i){
        if(route.route_points.at(i).type == RoutePoint::OVERLOADING_START){
            indOLStart = i;
            break;
        }
    }
    if(indOLStart > range.max)//we do not have enough information to segment
        return {};

    size_t indOLEnd = range.max + 1;
    for(size_t i = indOLStart ; i <= range.max ; ++i){
        if(route.route_points.at(i).type == RoutePoint::OVERLOADING_FINISH){
            indOLEnd = i;
            break;
        }
    }
    if(indOLEnd > range.max)//we do not have enough information to segment
        return {};

    ret.at(0).min = range.min;
    ret.at(0).max = indOLStart;
    ret.at(1).min = indOLStart;
    ret.at(1).max = indOLEnd;
    ret.at(2).min = indOLEnd;
    ret.at(2).max = range.max;

    return ret;

}

std::vector<PlanAnalyserBase::IndexRange> OlvPlanAnalyser::getOutFieldSubsegmentIndexes(const Route &route, const IndexRange &range) const
{
    return getOutFieldSubsegmentIndexes_informed(route, range);
}

std::vector<PlanAnalyserBase::IndexRange> OlvPlanAnalyser::getOutFieldSubsegmentIndexes_informed(const Route &route, const IndexRange &range) const
{
    std::vector<PlanAnalyserBase::IndexRange> ret(3);

    size_t indRPStart = range.max + 1;
    for(size_t i = range.min ; i <= range.max ; ++i){
        if(route.route_points.at(i).type == RoutePoint::RESOURCE_POINT){
            indRPStart = i;
            break;
        }
    }
    if(indRPStart > range.max)//we do not have enough information to segment
        return {};

    size_t indRPEnd = range.max + 1;
    for(int i = range.max ; i >= range.min ; --i){
        if(route.route_points.at(i).type == RoutePoint::RESOURCE_POINT){
            indRPEnd = i;
            break;
        }
    }
    if(indRPEnd > range.max)//we do not have enough information to segment
        return {};

    ret.at(0).min = range.min;
    ret.at(0).max = indRPStart;
    ret.at(1).min = indRPStart;
    ret.at(1).max = indRPEnd;
    ret.at(2).min = indRPEnd;
    ret.at(2).max = range.max;

    return ret;

}

bool OlvPlanAnalyser::analyseSegments(const Route &route1, const Route &route2, AnalysisResult &analysisResult) const
{
    std::string routeNr = "1";
    if(!analyseSegment(route1, analysisResult.routeSegmentation_1.indexRange_init, analysisResult.initAnalysis_1)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the init segment from route " + routeNr);
        return false;
    }
    if(!analyseSegment(route1, analysisResult.routeSegmentation_1.indexRange_end, analysisResult.endAnalysis_1)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the end segment from route " + routeNr);
        return false;
    }
    if(!analyseFieldSegments(route1, analysisResult.routeSegmentation_1.indexRanges_field, analysisResult.fieldAnalysis_1)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing 'in-the-field driving' segments from route " + routeNr);
        return false;
    }
    if(!analyseOutFieldSegments(route1, analysisResult.routeSegmentation_1.indexRanges_outfield, analysisResult.outFieldAnalysis_1)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing 'out-of-field driving' segments from route " + routeNr);
        return false;
    }

    routeNr = "2";
    if(!analyseSegment(route2, analysisResult.routeSegmentation_2.indexRange_init, analysisResult.initAnalysis_2)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the init segment from route " + routeNr);
        return false;
    }
    if(!analyseSegment(route2, analysisResult.routeSegmentation_2.indexRange_end, analysisResult.endAnalysis_2)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing the end segment from route " + routeNr);
        return false;
    }
    if(!analyseFieldSegments(route2, analysisResult.routeSegmentation_2.indexRanges_field, analysisResult.fieldAnalysis_2)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing 'in-the-field driving' segments from route " + routeNr);
        return false;
    }
    if(!analyseOutFieldSegments(route2, analysisResult.routeSegmentation_2.indexRanges_outfield, analysisResult.outFieldAnalysis_2)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing 'out-of-field driving' segments from route " + routeNr);
        return false;
    }

    return true;
}

bool OlvPlanAnalyser::analyseSegment(const Route &route, const OlvPlanAnalyser::IndexRange &indexRange, SegmentAnalysisResult &analysisResult) const
{

    analysisResult.distance = 0;
    analysisResult.duration = 0;
    analysisResult.speed_avg = 0;
    analysisResult.speed_max = std::numeric_limits<double>::lowest();
    analysisResult.speed_min = std::numeric_limits<double>::max();
    if(indexRange.min < 0 && indexRange.max < 0){
        analysisResult.timeStart = -1;
        analysisResult.timeEnd = -1;
        return true;
    }

    if(indexRange.max < indexRange.min
            || indexRange.min < 0
            || indexRange.max >= route.route_points.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid index-range");
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
                analysisResult.speed_max = std::max(analysisResult.speed_max, dist / deltaTime);
                analysisResult.speed_min = std::min(analysisResult.speed_min, dist / deltaTime);
            }
            analysisResult.distance += dist;
        }

        analysisResult.duration = rp2.time_stamp - rp1.time_stamp;
        if(analysisResult.duration != 0)
            analysisResult.speed_avg = analysisResult.distance/analysisResult.duration;
    }
    if(analysisResult.speed_min > analysisResult.speed_max)
        analysisResult.speed_min = analysisResult.speed_max = 0;

    return true;
}

bool OlvPlanAnalyser::analyseFieldSegments(const Route &route,
                                           const std::vector<std::pair<PlanAnalyserBase::IndexRange, std::vector<PlanAnalyserBase::IndexRange> > > &indexRanges,
                                           FieldAnalysisResult &analysisSegments) const
{
    auto & overall = analysisSegments.overall;
    auto & segments = analysisSegments.segments;
    overall.duration_total = overall.distance_total = 0;
    overall.duration_avg = overall.distance_avg = overall.speed_avg = 0;
    overall.duration_min = overall.distance_min = overall.speed_min = std::numeric_limits<double>::max();
    overall.duration_max = overall.distance_max = overall.speed_max = std::numeric_limits<double>::lowest();

    analysisSegments.segments.resize( indexRanges.size() );

    for(size_t i = 0 ; i < indexRanges.size() ; ++i){
        const auto & ir = indexRanges.at(i);
        if(!analyseFieldSegment(route, ir, segments.at(i))){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, 10, "Error analysing segment ", i );
            analysisSegments.segments.clear();
            return false;
        }
    }

    for(auto &s : segments){
        overall.distance_min = std::min( overall.distance_min, s.distance );
        overall.distance_max = std::max( overall.distance_max, s.distance );
        overall.distance_total += s.distance;

        overall.duration_min = std::min( overall.duration_min, s.duration );
        overall.duration_max = std::max( overall.duration_max, s.duration );
        overall.duration_total += s.duration;

        overall.speed_min = std::min( overall.speed_min, s.speed_min );
        overall.speed_max = std::max( overall.speed_max, s.speed_max );
        overall.speed_avg += s.speed_avg;
    }
    overall.distance_avg = overall.distance_total / segments.size();
    overall.duration_avg = overall.duration_total / segments.size();
    overall.speed_avg = overall.speed_avg / segments.size();

    if(overall.distance_min > overall.distance_max)
        overall.distance_min = overall.distance_max = 0;

    if(overall.duration_min > overall.duration_max)
        overall.duration_min = overall.duration_max = 0;

    if(overall.speed_min > overall.speed_max)
        overall.speed_min = overall.speed_max = 0;

    return true;
}

bool OlvPlanAnalyser::analyseFieldSegment(const Route &route,
                                          const std::pair<PlanAnalyserBase::IndexRange, std::vector<PlanAnalyserBase::IndexRange> > &indexRanges,
                                          OlvPlanAnalyser::FieldAnalysisResult::FieldSegmentAnalysisResult &analysisSegment) const
{
    if(!analyseSegment(route, indexRanges.first, analysisSegment)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing segment." );
        return false;
    }
    if(!analyseSegment(route, indexRanges.second.at(0), analysisSegment.toOverload)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing 'to overload' sub-segment." );
        return false;
    }
    if(!analyseSegment(route, indexRanges.second.at(1), analysisSegment.overload)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing 'overload' sub-segment." );
        return false;
    }
    if(!analyseSegment(route, indexRanges.second.at(2), analysisSegment.fromOverload)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing 'from overload' sub-segment." );
        return false;
    }

    return true;
}

bool OlvPlanAnalyser::analyseOutFieldSegments(const Route &route,
                                              const std::vector<std::pair<PlanAnalyserBase::IndexRange, std::vector<PlanAnalyserBase::IndexRange> > > &indexRanges,
                                              OlvPlanAnalyser::OutFieldAnalysisResult &analysisSegments) const
{
    auto & overall = analysisSegments.overall;
    auto & segments = analysisSegments.segments;
    overall.duration_total = overall.distance_total = 0;
    overall.duration_avg = overall.distance_avg = overall.speed_avg = 0;
    overall.duration_min = overall.distance_min = overall.speed_min = std::numeric_limits<double>::max();
    overall.duration_max = overall.distance_max = overall.speed_max = std::numeric_limits<double>::lowest();

    analysisSegments.segments.resize( indexRanges.size() );

    for(size_t i = 0 ; i < indexRanges.size() ; ++i){
        const auto & ir = indexRanges.at(i);
        if(!analyseOutFieldSegment(route, ir, segments.at(i))){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, 10, "Error analysing segment ", i );
            analysisSegments.segments.clear();
            return false;
        }
    }

    for(auto &s : segments){
        overall.distance_min = std::min( overall.distance_min, s.distance );
        overall.distance_max = std::max( overall.distance_max, s.distance );
        overall.distance_total += s.distance;

        overall.duration_min = std::min( overall.duration_min, s.duration );
        overall.duration_max = std::max( overall.duration_max, s.duration );
        overall.duration_total += s.duration;

        overall.speed_min = std::min( overall.speed_min, s.speed_min );
        overall.speed_max = std::max( overall.speed_max, s.speed_max );
        overall.speed_avg += s.speed_avg;
    }
    overall.distance_avg = overall.distance_total / segments.size();
    overall.duration_avg = overall.duration_total / segments.size();
    overall.speed_avg = overall.speed_avg / segments.size();

    if(overall.distance_min > overall.distance_max)
        overall.distance_min = overall.distance_max = 0;

    if(overall.duration_min > overall.duration_max)
        overall.duration_min = overall.duration_max = 0;

    if(overall.speed_min > overall.speed_max)
        overall.speed_min = overall.speed_max = 0;

    return true;

}

bool OlvPlanAnalyser::analyseOutFieldSegment(const Route &route,
                                             const std::pair<PlanAnalyserBase::IndexRange, std::vector<PlanAnalyserBase::IndexRange> > &indexRanges,
                                             OlvPlanAnalyser::OutFieldAnalysisResult::OutFieldSegmentAnalysisResult &analysisSegment) const
{
    if(!analyseSegment(route, indexRanges.first, analysisSegment)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing segment." );
        return false;
    }
    if(!analyseSegment(route, indexRanges.second.at(0), analysisSegment.toOverload)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing 'to overload' sub-segment." );
        return false;
    }
    if(!analyseSegment(route, indexRanges.second.at(1), analysisSegment.overload)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing 'overload' sub-segment." );
        return false;
    }
    if(!analyseSegment(route, indexRanges.second.at(2), analysisSegment.fromOverload)){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error analysing 'from overload' sub-segment." );
        return false;
    }

    return true;

}


//void OlvPlanAnalyser::addSegmentResultsToCSV(std::ofstream &of, const AnalysisResult &results, FieldSegmentAnalysisResult::ValueType valueType) const
//{
//    of /*<< std::setprecision(10)*/
//       << results.initAnalysis_1.getValue(valueType) << sep
//       << results.initAnalysis_2.getValue(valueType) << sep
//       << results.initAnalysis_1.getValue(valueType) - results.initAnalysis_2.getValue(valueType) << sep
//       << results.endAnalysis_1.getValue(valueType) << sep
//       << results.endAnalysis_2.getValue(valueType) << sep
//       << results.endAnalysis_1.getValue(valueType) - results.endAnalysis_2.getValue(valueType) << sep
//       << results.fieldAnalysis_1.getValue(valueType) << sep
//       << results.fieldAnalysis_2.getValue(valueType) << sep
//       << results.fieldAnalysis_1.getValue(valueType) - results.fieldAnalysis_2.getValue(valueType) << sep
//       << results.headlandAnalysis_1.getValue(valueType) << sep
//       << results.headlandAnalysis_2.getValue(valueType) << sep
//       << results.headlandAnalysis_1.getValue(valueType) - results.headlandAnalysis_2.getValue(valueType) << sep
//       << results.infieldAnalysis_1.getValue(valueType) << sep
//       << results.infieldAnalysis_2.getValue(valueType) << sep
//       << results.infieldAnalysis_1.getValue(valueType) - results.infieldAnalysis_2.getValue(valueType) << std::endl;

//}

//void OlvPlanAnalyser::addInfieldSubsegmentResultsToCSV(std::ofstream &of, const AnalysisResult &results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::ValueType valueType) const
//{
//    of /*<< std::setprecision(10)*/
//       << results.infieldAnalysis_1.infieldSegments.getValue(valueType) << sep
//       << results.infieldAnalysis_2.infieldSegments.getValue(valueType) << sep
//       << results.infieldAnalysis_1.infieldSegments.getValue(valueType) - results.infieldAnalysis_2.infieldSegments.getValue(valueType) << sep
//       << results.infieldAnalysis_1.headlandSegments.getValue(valueType) << sep
//       << results.infieldAnalysis_2.headlandSegments.getValue(valueType) << sep
//       << results.infieldAnalysis_1.headlandSegments.getValue(valueType) - results.infieldAnalysis_2.headlandSegments.getValue(valueType) << std::endl;

//}

//void OlvPlanAnalyser::addInfieldSubsegmentResultsToCSV(std::ofstream &of, const OlvPlanAnalyser::AnalysisResult &results, size_t ind, FieldSegmentAnalysisResult::ValueType valueType) const
//{
////    of << std::setprecision(10);
//    double val1 = 0 , val2 = 0;
//    if(ind < results.infieldAnalysis_1.infieldSubSegments.size()){
//        val1 = results.infieldAnalysis_1.infieldSubSegments.at(ind).getValue(valueType);
//        of << val1;
//    }
//    of << sep;
//    if(ind < results.infieldAnalysis_2.infieldSubSegments.size()){
//        val2 = results.infieldAnalysis_2.infieldSubSegments.at(ind).getValue(valueType);
//        of << val2;
//    }
//    of << sep
//       << val1 - val2 << sep;
//    val1 = val2 = 0;
//    if(ind < results.infieldAnalysis_1.headlandSubSegments.size()){
//        val1 = results.infieldAnalysis_1.headlandSubSegments.at(ind).getValue(valueType);
//        of << val1;
//    }
//    of << sep;
//    if(ind < results.infieldAnalysis_2.headlandSubSegments.size()){
//        val2 = results.infieldAnalysis_2.headlandSubSegments.at(ind).getValue(valueType);
//        of << val2;
//    }
//    of << sep
//       << val1 - val2 << std::endl;

//}

double OlvPlanAnalyser::SegmentAnalysisResult::getValue(ValueType type) const
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
    default:
        return 0;
    }

}

double OlvPlanAnalyser::FieldAnalysisResult::FieldSegmentsAnalysisResult::getValue(ValueType type) const
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
    default:
        return 0;
    }

}

double OlvPlanAnalyser::OutFieldAnalysisResult::OutFieldSegmentsAnalysisResult::getValue(ValueType type) const
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
    default:
        return 0;
    }

}


}
