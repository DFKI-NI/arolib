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
 
#ifndef AROLIB_HARVESTER_PLAN_ANALYSER_H
#define AROLIB_HARVESTER_PLAN_ANALYSER_H

#include <map>
#include <fstream>

#include "plananalyserbase.hpp"
#include "arolib/misc/loggingcomponent.h"
#include "arolib/types/field.hpp"
#include "arolib/types/machine.hpp"
#include "arolib/types/route.hpp"

namespace arolib {

/**
 * @brief Class to analyze plans of harvesters
 */
class HarvesterPlanAnalyser : public PlanAnalyserBase
{
public:

    /**
     * @brief Indexes resulting from obtaining the diferent segments of a route (initial segment, in-the-field segment, headland-harvesting segment, inner-field-harvesting segment (and subsegments), final segment)
     */
    struct RouteSegmentationResults{
        IndexRange indexRange_init;/**< Index range for the routepoints corresponding to the initial segment (from inial point, until first harvesting route point (informed) or entering the field (not informed)) */
        IndexRange indexRange_end;/**< Index range for the routepoints corresponding to the final segment (from last harvesting route point (informed) or exiting the field (not informed), until the last route point) */
        IndexRange indexRange_field;/**< Index range for the routepoints between the first and last harvesting route points (informed), or corresponding to driving inside the field (not informed) */
        IndexRange indexRange_headland;/**< Index range for the routepoints corresponding to headland harvesting */
        IndexRange indexRange_infield;/**< Index range for the routepoints corresponding to inner-field harvesting */
        std::vector<IndexRange> indexRange_infield__headland;/**< Index ranges for the routepoints corresponding to segments of (inter-track) headland driving during inner-field harvesting */
        std::vector<IndexRange> indexRange_infield__infield;/**< Index ranges for the routepoints corresponding to segments of track-harvesting during inner-field harvesting */
    };

    /**
     * @brief Route-segment analytic results (general for field segments)
     */
    struct FieldSegmentAnalysisResult{

        /**
         * @brief Type of analytic result
         */
        enum ValueType{
            TIME_START,/**< Start time of the segment */
            TIME_END,/**< End time of the segment */
            DURATION,/**< Duration of the segment */
            DISTANCE,/**< Distance covered by the segment */
            SPEED_AVG,/**< Average speed in the segment */
            SPEED_MIN,/**< Minimum speed in the segment */
            SPEED_MAX,/**< Maximum speed in the segment */
            SPEED_HARV_MIN,/**< Minimum speed in the segment whilst harvesting */
            SPEED_HARV_MAX,/**< Maximum speed in the segment whilst harvesting */
            MASS_TOTAL,/**< Total yield mass harvested in the segment */
            MASS_PER_TIME,/**< Rate of harvested yield mass in the segment per second */
            MASS_PER_DISTANCE/**< Rate of harvested yield mass in the segment per meter */
        };
        double timeStart = -1;/**< Start time of the segment [s] */
        double timeEnd = -1;/**< End time of the segment [s] */
        double duration = -1;/**< Duration of the segment [s] */
        double distance = -1;/**< Distance covered by the segment [m] */
        double speed_avg = -1;/**< Average speed in the segment [m/s] */
        double speed_min = -1;/**< Minimum speed in the segment [m/s] */
        double speed_max = -1;/**< Maximum speed in the segment [m/s] */
        double speed_harv_min = -1;/**< Minimum speed in the segment [m/s] whilst harvesting */
        double speed_harv_max = -1;/**< Maximum speed in the segment [m/s] whilst harvesting */
        double yieldMass_total;/**< Total yield mass harvested in the segment [Kg] */
        double yieldMassPerTime_avg;/**< Rate of harvested yield mass in the segment per second [kg/s] */
        double yieldMassPerDistance_avg;/**< Rate of harvested yield mass in the segment per meter [kg/m] */

        /**
         * @brief Get a certain analytics value
         * @param type Type of value to retrieve
         * @return Retrieved value
         */
        double getValue(ValueType type) const;
    };

    /**
     * @brief Analytic results corresponding to inner-field-harvesting
     */
    struct InfieldAnalysisResult : public FieldSegmentAnalysisResult{

        /**
         * @brief Route-segment analytic results corresponding to inner-field-harvesting
         */
        struct InfieldSegmentsAnalysisResult{
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
                SPEED_AVG, /**< Average speed registered per segment [m/s] (sum of segment-speeds / num segments) */
                SPEED_HARV_MIN,/**< Speed registered in the slowest segment [m/s] */
                SPEED_HARV_MAX,/**< Speed registered in the fastest segment [m/s] */
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
            double speed_harv_min = -1;/**< Speed registered in the slowest segment [m/s] */
            double speed_harv_max = -1;/**< Speed registered in the fastest segment [m/s] */
            double getValue(ValueType type) const;
        };
        std::vector<FieldSegmentAnalysisResult> headlandSubSegments;/**< Analytic results for each of the (sub) segments corresponding to (inter-track) headland driving during inner-field harvesting */
        std::vector<FieldSegmentAnalysisResult> infieldSubSegments;/**< Analytic results for each of the (sub) segments corresponding to track-harvesting during inner-field harvesting */
        InfieldSegmentsAnalysisResult headlandSegments;/**< (overall) Analytic results for all the (sub) segments corresponding to (inter-track) headland driving during inner-field harvesting */
        InfieldSegmentsAnalysisResult infieldSegments;/**< (overall) Analytic results for all the (sub) segments corresponding to track-harvesting during inner-field harvesting */

    };

    /**
     * @brief Overall analytic results for two routes being compared
     * The route is divided in 3 main segments: initial, field and final
     *    initial: if the route comes from a proper plan (informed), corresponds to the route segment from the inial point until the first harvesting route point; otherwise (not informed), it corresponds to the route segment from the inial point until entering the field
     *    final (end): if the route comes from a proper plan (informed), corresponds to the route segment from the last harvesting route point until the last route point; otherwise (not informed), it corresponds to the route segment from exiting the field until the last route point
     *    field (in-the-field): corresponds to the route segment in between the initial and final segments
     * The field (in-the-field) segments are devided in several subsegments of two diferent types: headland and infield
     *    headland: corresponds to the connections (through the headland) between tracks that are being harvested
     *    infield: corresponds to the tracks that are being harvested
     */
    struct AnalysisResult{
        FieldSegmentAnalysisResult fieldAnalysis_1;/**< (general) In-the-field-segment analytic results for route 1 */
        FieldSegmentAnalysisResult fieldAnalysis_2;/**< (general) In-the-field-segment analytic results for route 2 */
        RouteSegmentationResults routeSegmentation_1;/**< Indexes corresponding to the diferent segments of route 1 */
        RouteSegmentationResults routeSegmentation_2;/**< Indexes corresponding to the diferent segments of route 2 */
        FieldSegmentAnalysisResult initAnalysis_1;/**< (general) Initial-segment analytic results for route 1 (from initial point, until first harvesting route point (informed) or entering the field (not informed)) */
        FieldSegmentAnalysisResult initAnalysis_2;/**< (general) Initial-segment analytic results for route 2 (from initial point, until first harvesting route point (informed) or entering the field (not informed)) */
        FieldSegmentAnalysisResult endAnalysis_1;/**< (general) Final-segment analytic results for route 1 (from last harvesting route point (informed) or exiting the field (not informed), until the last route point) */
        FieldSegmentAnalysisResult endAnalysis_2;/**< (general) Final-segment analytic results for route 2 (from last harvesting route point (informed) or exiting the field (not informed), until the last route point) */
        FieldSegmentAnalysisResult headlandAnalysis_1;/**< Headland-harvesting-segment analytic results for route 1 */
        FieldSegmentAnalysisResult headlandAnalysis_2;/**< Headland-harvesting-segment analytic results for route 2 */
        InfieldAnalysisResult infieldAnalysis_1;/**< Inner-field-harvesting-segment analytic results for route 1 */
        InfieldAnalysisResult infieldAnalysis_2;/**< Inner-field-harvesting-segment analytic results for route 2 */
        /*
         * yield?
        */

    };

    /**
     * @brief Constructor.
     * @param logLevel Log level
     */
    explicit HarvesterPlanAnalyser(const LogLevel &logLevel = LogLevel::INFO);

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
     * @param [out] segResults Route segmentation results
     * @return True on success
     */
    bool getRouteSegmentsIndexes_informed(const Subfield &subfield, const Route& route, RouteSegmentationResults &segResults) const;

    /**
     * @brief Make an overall analysis of 2 given routes
     * @param route1 Route 1 to be analyzed
     * @param route2 Route 2 to be analyzed
     * @param [out] analysisResult Analysis results
     * @return True on success
     */
    bool analyseSegments(const Route& route1, const Route& route2, AnalysisResult &analysisResult);

    /**
     * @brief Make a (general) analysis for a route segment for a given route
     * @param route Route to be analyzed
     * @param indexRange Index range corresponding to the route segment to be analyzed
     * @param [out] analysisResult Analysis results for the given segment
     * @return True on success
     */
    bool analyseSegment(const Route& route, const IndexRange &indexRange, FieldSegmentAnalysisResult &analysisResult);

    /**
     * @brief Make an overall analysis to the inner-field-harvesting-segment subsegments (i.e. (inter-trac) headland connection subsegments and track-harvesting subsegments), plus an analysis to each of the subsegments separatelly
     * @param route Route to be analyzed
     * @param indexRanges Index ranges corresponding to the inner-field segment subsegments to be analyzed
     * @param [out] analysisSegment Overall analysis results for the inner-field-harvesting-segment subsegments
     * @param [out] analysisSubSegments Analysis results for all subsegments
     * @return True on success
     */
    bool analyseSubSegments(const Route& route,
                            const std::vector<IndexRange> &indexRanges,
                            InfieldAnalysisResult::InfieldSegmentsAnalysisResult &analysisSegment,
                            std::vector<FieldSegmentAnalysisResult> &analysisSubSegments);

    /**
     * @brief Add a specific analitics result (value) to a CSV output file stream
     * @param [in/out] of Output file stream
     * @param results Analisys results containing the value to be added
     * @param valueType Analitics result (value) to be added
     */
    void addSegmentResultsToCSV(std::ofstream &of, const AnalysisResult& results, FieldSegmentAnalysisResult::ValueType valueType) const;

    /**
     * @brief Add a specific analitics result (value) to a CSV output file stream corresponding the overall analysis of inner-field-harvesting-segment subsegments
     * @param [in/out] of Output file stream
     * @param results Analisys results containing the value to be added
     * @param valueType Analitics result (value) to be added
     */
    void addInfieldSubsegmentResultsToCSV(std::ofstream &of, const AnalysisResult& results, InfieldAnalysisResult::InfieldSegmentsAnalysisResult::ValueType valueType) const;

    /**
     * @brief Add a specific analitics result (value) to a CSV output file stream corresponding the analysis of a specific inner-field-harvesting-segment subsegment
     * @param [in/out] of Output file stream
     * @param results Analisys results containing the value to be added
     * @param ind Index of the specific subsegment
     * @param valueType Analitics result (value) to be added
     */
    void addInfieldSubsegmentResultsToCSV(std::ofstream &of, const AnalysisResult& results, size_t ind, FieldSegmentAnalysisResult::ValueType valueType) const;


protected:
    std::map<size_t, std::map<MachineId_t, AnalysisResult > > m_analysisResult;/**< Overall analytics results: map<subfieldId, map<machineId, results> > */
    static const double m_harvestYieldRateThreshold;/**< harvest yield-rate threshold [Kg/m] */
};

}

#endif // AROLIB_HARVESTER_PLAN_ANALYSER_H
