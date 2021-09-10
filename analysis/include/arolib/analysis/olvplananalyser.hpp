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
 
#ifndef AROLIB_OLV_PLAN_ANALYSER_H
#define AROLIB_OLV_PLAN_ANALYSER_H

#include <map>
#include <fstream>

#include "plananalyserbase.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/types/field.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/types/route.hpp"

namespace arolib {

/**
 * @brief Class to analyze plans of overload vehicles
 */
class OlvPlanAnalyser : public PlanAnalyserBase
{
public:

    /**
     * @brief Indexes resulting from obtaining the diferent segments of a route (initial segment, in-the-field segments, out-of-field segments, final segment)
     */
    struct RouteSegmentationResults{
        IndexRange indexRange_init;/**< Index range for the routepoints corresponding to the initial segment (from inial point, until entering the field) */
        IndexRange indexRange_end;/**< Index range for the routepoints corresponding to the final segment (from exiting the field for the last time, until the last route point) */
        std::vector< std::pair< IndexRange, std::vector<IndexRange> > > indexRanges_field;/**< Index ranges for the routepoints corresponding to in-the-field driving, and the index ranges of thei subsegments */
        std::vector< std::pair< IndexRange, std::vector<IndexRange> > > indexRanges_outfield;/**< Index ranges for the routepoints corresponding to out-of-field driving, and the index ranges of thei subsegments */
    };

    /**
     * @brief Route-segment analytic results (general for all segments segments)
     */
    struct SegmentAnalysisResult{

        /**
         * @brief Type of analytic result
         *
         * In the case of uninformed routes, the distance and speeds for out-of-field segments are NAN
         */
        enum ValueType{
            TIME_START,/**< Start time of the segment */
            TIME_END,/**< End time of the segment */
            DURATION,/**< Duration of the segment */
            DISTANCE,/**< Distance covered by the segment */
            SPEED_AVG,/**< Average speed in the segment */
            SPEED_MIN,/**< Minimum speed in the segment */
            SPEED_MAX,/**< Maximum speed in the segment */
        };
        double timeStart = -1;/**< Start time of the segment [s] */
        double timeEnd = -1;/**< End time of the segment [s] */
        double duration = -1;/**< Duration of the segment [s] */
        double distance = -1;/**< Distance covered by the segment [m] */
        double speed_avg = -1;/**< Average speed in the segment [m/s] */
        double speed_min = -1;/**< Minimum speed in the segment [m/s] */
        double speed_max = -1;/**< Maximum speed in the segment [m/s] */

        /**
         * @brief Get a certain analytics value
         * @param type Type of value to retrieve
         * @return Retrieved value
         */
        double getValue(ValueType type) const;
    };

    /**
     * @brief Analytic results corresponding to in-the-field driving
     */
    struct FieldAnalysisResult{

        /**
         * @brief Route-segment analytic results corresponding to in-the-field driving
         */
        struct FieldSegmentAnalysisResult : public SegmentAnalysisResult{
            SegmentAnalysisResult toOverload;/**< Analytic results for each of the (sub) segments corresponding to driving from an entry point (or initial points inside the field) to an overload start */
            SegmentAnalysisResult overload;/**< Analytic results for each of the (sub) segments corresponding to overloading */
            SegmentAnalysisResult fromOverload;/**< Analytic results for each of the (sub) segments corresponding to driving from an overload finish to an exit point */
        };

        /**
         * @brief Route-segment analytic results corresponding to all in-the-field driving
         */
        struct FieldSegmentsAnalysisResult{
            /**
             * @brief Type of analytic result
             */
            enum ValueType{
                DURATION_TOTAL,/**< Total duration of the segments [s] */
                DURATION_MIN,/**< Minimum duration of a segment [s] */
                DURATION_MAX,/**< Maximum duration of a segment [s] */
                DURATION_AVG,/**< Average segment duration [s] (sum of segment-durations / num segments) */
                DISTANCE_TOTAL,/**< Total distance covered by all segments [m] */
                DISTANCE_MIN,/**< Distance covered by shortest segment [m] */
                DISTANCE_MAX,/**< Distance covered by longest segment [m] */
                DISTANCE_AVG,/**< Average distance covered per segment [m] (sum of segment-distances / num segments) */
                SPEED_MIN,/**< Speed registered in the slowest segment [m/s] */
                SPEED_MAX,/**< Speed registered in the fastest segment [m/s] */
                SPEED_AVG /**< Average speed registered per segment [m/s] (sum of segment-speeds / num segments) */
            };
            double duration_total = -1;/**< Total duration of the segments [s] */
            double duration_min = -1;/**< Minimum duration of a segment [s] */
            double duration_max = -1;/**< Maximum duration of a segment [s] */
            double duration_avg = -1;/**< Average segment duration [s] (sum of segment-durations / num segments) */
            double distance_total = -1;/**< Total distance covered by all segments [m] */
            double distance_min = -1;/**< Distance covered by shortest segment [m] */
            double distance_max = -1;/**< Distance covered by longest segment [m] */
            double distance_avg = -1;/**< Average distance covered per segment [m] (sum of segment-distances / num segments) */
            double speed_min = -1;/**< Speed registered in the slowest segment [m/s] */
            double speed_max = -1;/**< Speed registered in the fastest segment [m/s] */
            double speed_avg = -1;/**< Average speed registered per segment [m/s] (sum of segment-speeds / num segments) */
            double getValue(ValueType type) const;
        };
        std::vector<FieldSegmentAnalysisResult> segments;/**< Analytic results for each of the the segments corresponding to in-the-field driving */
        FieldSegmentsAnalysisResult overall;/**< (overall) Analytic results for all the segments corresponding to in-the-field driving */
    };

    /**
     * @brief Analytic results corresponding to out-of-field driving
     */
    struct OutFieldAnalysisResult{

        /**
         * @brief Route-segment analytic results corresponding to in-the-field driving
         */
        struct OutFieldSegmentAnalysisResult : public SegmentAnalysisResult{
            SegmentAnalysisResult toOverload;/**< Analytic results for each of the (sub) segments corresponding to driving from an exit point to a resource point (silo) */
            SegmentAnalysisResult overload;/**< Analytic results for each of the (sub) segments corresponding to downloading at the resource point */
            SegmentAnalysisResult fromOverload;/**< Analytic results for each of the (sub) segments corresponding to driving from an overload finish to an exit point */
        };

        /**
         * @brief Route-segment analytic results corresponding to all out-of-field driving
         */
        struct OutFieldSegmentsAnalysisResult{
            /**
             * @brief Type of analytic result
             *
             * In the case of uninformed routes, the distances and speeds for out-of-field sub-segments are NAN
             */
            enum ValueType{
                DURATION_TOTAL,/**< Total duration of the segments [s] */
                DURATION_MIN,/**< Minimum duration of a segment [s] */
                DURATION_MAX,/**< Maximum duration of a segment [s] */
                DURATION_AVG,/**< Average segment duration [s] (sum of segment-durations / num segments) */
                DISTANCE_TOTAL,/**< Total distance covered by all segments [m] */
                DISTANCE_MIN,/**< Distance covered by shortest segment [m] */
                DISTANCE_MAX,/**< Distance covered by longest segment [m] */
                DISTANCE_AVG,/**< Average distance covered per segment [m] (sum of segment-distances / num segments) */
                SPEED_MIN,/**< Speed registered in the slowest segment [m/s] */
                SPEED_MAX,/**< Speed registered in the fastest segment [m/s] */
                SPEED_AVG /**< Average speed registered per segment [m/s] (sum of segment-speeds / num segments) */
            };
            double duration_total = -1;/**< Total duration of the segments [s] */
            double duration_min = -1;/**< Minimum duration of a segment [s] */
            double duration_max = -1;/**< Maximum duration of a segment [s] */
            double duration_avg = -1;/**< Average segment duration [s] (sum of segment-durations / num segments) */
            double distance_total = -1;/**< Total distance covered by all segments [m] */
            double distance_min = -1;/**< Distance covered by shortest segment [m] */
            double distance_max = -1;/**< Distance covered by longest segment [m] */
            double distance_avg = -1;/**< Average distance covered per segment [m] (sum of segment-distances / num segments) */
            double speed_min = -1;/**< Speed registered in the slowest segment [m/s] */
            double speed_max = -1;/**< Speed registered in the fastest segment [m/s] */
            double speed_avg = -1;/**< Average speed registered per segment [m/s] (sum of segment-speeds / num segments) */
            double getValue(ValueType type) const;
        };
        std::vector<OutFieldSegmentAnalysisResult> segments;/**< Analytic results for each of the the segments corresponding to out-of-field driving */
        OutFieldSegmentsAnalysisResult overall;/**< (overall) Analytic results for all the segments corresponding to out-of-field driving */

    };

    /**
     * @brief Overall analytic results for two routes being compared
     * The route is divided in segments: initial segment, in-the-field segments, out-of-field segments, and final segment
     *    initial: if the machine is initially outside the field corresponds to the route segment from the inial point until the first entry to the field
     *    final (end): corresponds to the route segment from the last field exit until the last route point
     *    in-the-field: segments corresponding to driving inside the field
     *    out-of-field: segments corresponding to driving outside the field
     * Each 'in-the-field' segment is devided in 3 subsegments which correspond to driving to the overloading start, driving during overloading, and driving from the overloading finish to the exit
     * Each 'out-of-field' segment is devided in 3 subsegments which correspond to driving to the resource point, unloading at the resource point, and driving from the resource point to a field entry
     */
    struct AnalysisResult{
        RouteSegmentationResults routeSegmentation_1;/**< Indexes corresponding to the diferent segments of route 1 */
        RouteSegmentationResults routeSegmentation_2;/**< Indexes corresponding to the diferent segments of route 2 */
        SegmentAnalysisResult initAnalysis_1;/**< (general) Initial-segment analytic results for route 1 (from initial point until first field entry, iif machine is initially outside the field) */
        SegmentAnalysisResult initAnalysis_2;/**< (general) Initial-segment analytic results for route 2 (from initial point until first field entry, iif machine is initially outside the field) */
        SegmentAnalysisResult endAnalysis_1;/**< (general) Final-segment analytic results for route 1 (from last field exit until the last route point) */
        SegmentAnalysisResult endAnalysis_2;/**< (general) Final-segment analytic results for route 2 (from last field exit until the last route point) */
        FieldAnalysisResult fieldAnalysis_1;/**< (general) In-the-field driving analytic results for route 1 */
        FieldAnalysisResult fieldAnalysis_2;/**< (general) In-the-field driving analytic results for route 2 */
        OutFieldAnalysisResult outFieldAnalysis_1;/**< (general) Out-of-field driving analytic results for route 1 */
        OutFieldAnalysisResult outFieldAnalysis_2;/**< (general) Out-of-field driving analytic results for route 2 */
        /*
         * yield?
        */

    };

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit OlvPlanAnalyser(const LogLevel &logLevel = LogLevel::INFO);

    /**
     * @brief Run analytics on the set data
     * @return True on success
     */
    bool runAnalysis() override;

    /**
     * @brief Retrieve the analytics results (after calling @see runAnalysis)
     * @param [out] results Analytics results ( map< subfield id, map< machine id , results > > )
     * @return True on success
     */
    bool getResults(std::map<size_t, std::map<MachineId_t, AnalysisResult >> & results);

    /**
     * @brief Retrieve the analytics results (after calling @see runAnalysis) for a specified subfield
     * @param subfield Subfield id
     * @param [out] results Analytics results ( map< machine id , results > )
     * @return True on success
     */
    bool getResults(size_t subfieldId, std::map<MachineId_t, AnalysisResult > & results);

    /**
     * @brief Retrieve the analytics results (after calling @see runAnalysis) for a specified subfield and a specified machine
     * @param subfield Subfield id
     * @param machineId Machine id
     * @param [out] results Analytics results
     * @return True on success
     */
    bool getResults(size_t subfieldId, const MachineId_t &machineId, AnalysisResult & results);

    /**
     * @brief Retrieve (directly) the analytics results (after calling @see runAnalysis)
     * @return Analytics results ( map< subfield id, map< machine id , results > > )
     */
    const std::map<size_t, std::map<MachineId_t, AnalysisResult >> & getResults();

    /**
     * @brief Retrieve (directly) the analytics results (after calling @see runAnalysis) for a specified subfield
     * @param subfield Subfield id
     * @return Analytics results ( map< machine id , results > )
     */
    const std::map<MachineId_t, AnalysisResult > & getResults(size_t subfieldId);

    /**
     * @brief Retrieve (directly) the analytics results (after calling @see runAnalysis) for a specified subfield and a specified machine
     * @param subfield Subfield id
     * @param machineId Machine id
     * @return Analytics results
     */
    const AnalysisResult& getResults(size_t subfieldId, const MachineId_t &machineId);


    /**
     * @brief Save the analytics results (after calling @see runAnalysis) in a CSV file
     * @param filename File name/path
     * @return True on success
     */
    bool saveResults_CSV(const std::string &filename) const override;
    

protected:
    /**
     * @brief Compute the indexes corresponding to all segments of a route (initial segment, in-the-field segment, headland-harvesting segment, inner-field-harvesting segment (and subsegments), final segment, etc.) for a specified subfield
     * This method asumes that the route was not obtained from a plan (i.e. does not have all the data given by a proper planned route). Used for real-routes.
     * @param subfield Subfield id
     * @param route Route to be analyzed
     * @param [out] segResults Route segmentation results
     * @return True on success
     */
    bool getRouteSegmentsIndexes(const Subfield &subfield, const Route& route, RouteSegmentationResults &segResults) const;

    /**
     * @brief Compute the indexes corresponding to all segments of a route (initial segment, in-the-field segment, headland-harvesting segment, inner-field-harvesting segment (and subsegments), final segment, etc.) for a specified subfield
     * This method asumes that the route was obtained from a plan (i.e. it has all the data given by a proper planned route) and hence the analysis is more accurate
     * @param subfield Subfield id
     * @param route Route to be analyzed
     * @param segResults Route segmentation results
     * @return True on success
     */
    bool getRouteSegmentsIndexes_informed(const Subfield &subfield, const Route& route, RouteSegmentationResults &segResults) const;

    /**
     * @brief Compute the indexes corresponding to the subsegments of a 'in-the-field driving' segment
     * This method asumes that the route was not obtained from a plan (i.e. does not have all the data given by a proper planned route). Used for real-routes.
     * @param route Route to be analyzed
     * @param range Index range of the 'in-the-field driving' segment
     * @return Segmentation results (index ranges of sub-segments)
     */
    std::vector<IndexRange> getFieldSubsegmentIndexes(const Route& route, const IndexRange& range) const;

    /**
     * @brief Compute the indexes corresponding to the subsegments of a 'in-the-field driving' segment
     * This method asumes that the route was obtained from a plan (i.e. it has all the data given by a proper planned route) and hence the analysis is more accurate
     * @param route Route to be analyzed
     * @param range Index range of the 'in-the-field driving' segment
     * @return Segmentation results (index ranges of sub-segments)
     */
    std::vector<IndexRange> getFieldSubsegmentIndexes_informed(const Route& route, const IndexRange& range) const;

    /**
     * @brief Compute the indexes corresponding to the subsegments of a 'out-of-field driving' segment
     * This method asumes that the route was not obtained from a plan (i.e. does not have all the data given by a proper planned route). Used for real-routes.
     * @param route Route to be analyzed
     * @param range Index range of the 'out-of-field driving' segment
     * @return Segmentation results (index ranges of sub-segments)
     */
    std::vector<IndexRange> getOutFieldSubsegmentIndexes(const Route& route, const IndexRange& range) const;

    /**
     * @brief Compute the indexes corresponding to the subsegments of a 'out-of-field driving' segment
     * This method asumes that the route was obtained from a plan (i.e. it has all the data given by a proper planned route) and hence the analysis is more accurate
     * @param route Route to be analyzed
     * @param range Index range of the 'out-of-field driving' segment
     * @return Segmentation results (index ranges of sub-segments)
     */
    std::vector<IndexRange> getOutFieldSubsegmentIndexes_informed(const Route& route, const IndexRange& range) const;

    /**
     * @brief Make an overall analysis of 2 given routes
     * @param route1 Route 1 to be analyzed
     * @param route2 Route 2 to be analyzed
     * @param [out] analysisResult Analysis results
     * @return True on success
     */
    bool analyseSegments(const Route& route1, const Route& route2, AnalysisResult &analysisResult) const;

    /**
     * @brief Make a (general) analysis for a route segment for a given route
     * @param route Route to be analyzed
     * @param indexRange Index range corresponding to the route segment to be analyzed
     * @param [out] analysisResult Analysis results for the given segment
     * @return True on success
     */
    bool analyseSegment(const Route& route, const IndexRange &indexRange, SegmentAnalysisResult &analysisResult) const;

    /**
     * @brief Make an overall analysis to the in-the-field driving segments and subsegments
     * @param route Route to be analyzed
     * @param indexRanges Index ranges corresponding to the in-the-field driving segments and subsegments to be analyzed
     * @param analysisSegments Overall analysis results for the in-the-field driving segments and subsegments
     * @return True on success
     */
    bool analyseFieldSegments(const Route& route,
                              const std::vector< std::pair< IndexRange, std::vector<IndexRange> > > &indexRanges,
                              FieldAnalysisResult &analysisSegments) const;

    /**
     * @brief Make an analysis to an in-the-field driving segment and its subsegments
     * @param route Route to be analyzed
     * @param indexRanges Index ranges corresponding to the in-the-field driving segment (and subsegments) to be analyzed
     * @param [out] analysisSegment Analysis results for the in-the-field driving segment and its subsegments
     * @return True on success
     */
    bool analyseFieldSegment(const Route& route,
                             const std::pair< IndexRange, std::vector<IndexRange> > &indexRanges,
                             FieldAnalysisResult::FieldSegmentAnalysisResult &analysisSegment) const;

    /**
     * @brief Make an overall analysis to the out-of-field driving segments and subsegments
     * @param route Route to be analyzed
     * @param indexRanges Index ranges corresponding to the out-of-field driving segments and subsegments to be analyzed
     * @param [out] analysisSegments Overall analysis results for the out-of-field driving segments and subsegments
     * @return True on success
     */
    bool analyseOutFieldSegments(const Route& route,
                                 const std::vector< std::pair< IndexRange, std::vector<IndexRange> > > &indexRanges,
                                 OutFieldAnalysisResult &analysisSegments) const;

    /**
     * @brief Make an analysis to an out-of-field driving segment and its subsegments
     * @param route Route to be analyzed
     * @param indexRanges Index ranges corresponding to the out-of-field driving segment (and subsegments) to be analyzed
     * @param [out] analysisSegment Analysis results for the out-of-field driving segment and its subsegments
     * @return True on success
     */
    bool analyseOutFieldSegment(const Route& route,
                                const std::pair< IndexRange, std::vector<IndexRange> > &indexRanges,
                                OutFieldAnalysisResult::OutFieldSegmentAnalysisResult &analysisSegment) const;

//    /**
//     * @brief Add a specific analitics result (value) to a CSV output file stream
//     * @param [in/out] of Output file stream
//     * @param results Analisys results containing the value to be added
//     * @param valueType Analitics result (value) to be added
//     */
//    void addSegmentResultsToCSV(std::ofstream &of, const AnalysisResult& results, FieldSegmentAnalysisResult::ValueType valueType) const;

//    /**
//     * @brief Add a specific analitics result (value) to a CSV output file stream corresponding the overall analysis of inner-field-harvesting-segment subsegments
//     * @param [in/out] of Output file stream
//     * @param results Analisys results containing the value to be added
//     * @param valueType Analitics result (value) to be added
//     */
//    void addInfieldSubsegmentResultsToCSV(std::ofstream &of, const AnalysisResult& results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::ValueType valueType) const;

//    /**
//     * @brief Add a specific analitics result (value) to a CSV output file stream corresponding the analysis of a specific inner-field-harvesting-segment subsegment
//     * @param [in/out] of Output file stream
//     * @param results Analisys results containing the value to be added
//     * @param ind Index of the specific subsegment
//     * @param valueType Analitics result (value) to be added
//     */
//    void addInfieldSubsegmentResultsToCSV(std::ofstream &of, const AnalysisResult& results, size_t ind, FieldSegmentAnalysisResult::ValueType valueType) const;


protected:
    std::map<size_t, std::map<MachineId_t, AnalysisResult > > m_analysisResult;/**< Overall analytics results: map<subfieldId, map<machineId, results> > */
};

}

#endif // AROLIB_OLV_PLAN_ANALYSER_H
