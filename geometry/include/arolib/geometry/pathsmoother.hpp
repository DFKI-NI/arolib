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
 
#ifndef AROLIB_PATH_SMOOTHER_HPP
#define AROLIB_PATH_SMOOTHER_HPP

#include <algorithm>
#include <set>
#include <functional>

#include "curves_helper.hpp"
#include "arolib/types/point.hpp"

namespace arolib{

namespace geometry{

/**
 * @brief Class used to smoothen paths
 */
class PathSmoother{
public:

    /**
     * @brief Strategy to smoothen corners
     */
    enum SmoothenPathCornerStrategy{
        SPCS_DO_NOT_REACH_CORNER_POINT, /**< Don't reach the corner point when smoothing a simple corner */
        SPCS_DO_NOT_REACH_CORNER_POINT_IF_POSSIBLE, /**< If possible, don't reach the corner point when smoothing a simple corner */
        SPCS_REACH_CORNER_POINT_IF_POSSIBLE, /**< Reach the corner point when smoothing a simple corner (not fully implemented) */
        SPCS_REACH_CORNER_POINT /**< If possible, reach the corner point when smoothing a simple corner (not fully implemented) */
    };

    /**
     * @brief Get the SmoothenPathCornerStrategy from its int value
     * @param value Int value
     * @return SmoothenPathCornerStrategy
     */
    static SmoothenPathCornerStrategy intToSmoothePathCornerStrategy(int value);

    /**
     * @brief Constructor
     * @param angRes Angular resolution [deg]
     * @param angThreshold Corners with angles higher than angThreshold won't be smoothen strictly (0°, 180°)
     * @param allowMultiCornerSmoothening Allow checking for smooth sub-paths paths for sub-paths that are not long enough to perform simple (1-corner) smoothing
     * @param allowDubins Allow using dubings in simple smoothing if it is not possible to do it with the default strategy
     * @param checkPiecewiseReverse If true, the input will be segmented into pieces which are smoothen separatelly in forward and reverse direction. the 'best' one is finally selected for each segment/piece
     * @param checkReverseOption If =0, no reverse check will be done to see if smoothening the input in reverse is beter. If <0, complete reverse check is done. If >0, the check will be done with reversing the complete input path iif the unsampled input has <= samples than the given number
     * @param cornerStrategy_sharpCorners Strategy to deal with corners that form an (absolute) angle < 90° (applicable only for simple smoothing)
     * @param cornerStrategy_nonSharpCorners  Strategy to deal with corners that form an (absolute) angle >= 90° (applicable only for simple smoothing)
     */
    explicit PathSmoother(double angRes = 10, double angThreshold = 150,
                          bool allowMultiCornerSmoothening = true,
                          bool allowDubins = true,
                          bool checkPiecewiseReverse = true,
                          int checkReverseOption = -1,
                          SmoothenPathCornerStrategy cornerStrategy_sharpCorners = SPCS_DO_NOT_REACH_CORNER_POINT_IF_POSSIBLE,
                          SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners = SPCS_DO_NOT_REACH_CORNER_POINT);

    /**
     * @brief Destructor
     */
    virtual ~PathSmoother();



    /**
     * @brief Smoothen a path based on a radius
     *
     * The return path is unsampled
     * @param points Points
     * @param radius Turning radius
     * @return Smoothen path (empty on error).
     */
    std::vector<Point> smoothenPath(const std::vector<Point>& points, double radius );

//    /**
//     * @brief Smoothen a path based on a radius
//     *
//     * The path is unsampled
//     * @param points Points
//     * @param radius Turning radius
//     * @param ind_from index of the first point of the segment to be smoothen
//     * @param ind_to index of the last point of the segment to be smoothen
//     * @return Smoothen path (empty on error).
//     */
//    std::vector<Point> smoothenPath(const std::vector<Point>& points, double radius, size_t ind_from, size_t ind_to );

    /**
     * @brief Checks if the segments [p0,p1] and [p1,p2] are long enough to smoothen the corner [p1] using smoothenPath.
     * It also computes the minimum segment distance to perform corner smootzhening, and some of the parameters used for such smoothening
     *
     * @sa smoothenPath
     *
     * @param p0 First point
     * @param p1 Corner point
     * @param p2 Last point
     * @param radius Turning radius
     * @param angRes Angular resolution [deg] for the corners
     * @param angThreshold [deg] Corners with angles higher than angThreshold are considered smooth (in case the the segment distance constraints are not met)
     * @param cornerStrategy_sharpCorners Strategy to deal with corners that forn an (absolute) angle < 90°
     * @param cornerStrategy_nonSharpCorners Strategy to deal with corners that form an (absolute) angle >= 90°
     * @param [out] cutDist Defining p0' and p2' as the points where the (main) corner semi circle will meet the segments [p0,p1] and [p1,p2], cutDist is the distance between p1 and p0' (equal to the distance between p1 and p2')
     * @param [out] minDist Minimum distance needed in the segments to achieve smoothening
     * @param [out] d2centroid Distance from p1 to the center of the (main) circle used for corner smoothening along the bisector of the angle formed by [p0, p1, p2]
     * @param [out] needsExtraSmoothening If true, the connection between the (main) corner semi circle and the segments [p0,p0'] and [p2',p2] need to be further smoothen
     * @return True if the length of segments [p0,p1] and [p1,p2] is long enough for the smoothening (i.e. it is >= minDist); or if the angle is >= angThreshold;
     */
    static bool checkCornerGeometryForPathSmoothening ( const Point& p0,
                                                        const Point& p1,
                                                        const Point& p2,
                                                        double radius,
                                                        double angThreshold,
                                                        SmoothenPathCornerStrategy cornerStrategy_sharpCorners,
                                                        SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners,
                                                        double& cutDist,
                                                        double& minDist,
                                                        double& d2centroid,
                                                        bool& needsExtraSmoothening );


    /**
     * @brief Compute the diference netween two paths
     * @param path1 path1
     * @param path2 path2
     * @return Difference value.
     */
    static double computeDifferenceBetweenPaths(std::vector<Point> path1, std::vector<Point> path2, double radius = 1);


    /**
     * @brief Get the angular resolution
     * @return Angular resolution
     */
    double getAngRes() const;

    /**
     * @brief Set angular resolution
     * @param value Angular resolution
     * @return True if value is valid
     */
    bool setAngRes(double value);

    /**
     * @brief Get the angular threshold
     * @return Angular threshold
     */
    double getAngThreshold() const;

    /**
     * @brief Set angular threshold
     * @param value Angular threshold
     * @return True if value is valid
     */
    bool setAngThreshold(double value);

    /**
     * @brief Is multi-corner smoothing enabled?
     * @return True if enabled
     */
    bool isMultiCornerSmootheningEnabled() const;

    /**
     * @brief Enable/disable multi-corner smoothing?
     * @param value enable?
     */
    void enableMultiCornerSmoothening(bool value);

    /**
     * @brief Is dubins enabled?
     * @return True if enabled
     */
    bool getAllowDubins() const;

    /**
     * @brief Enable/disable dubins for simple corners?
     * @param value enable?
     */
    void setAllowDubins(bool value);

    /**
     * @brief Is CheckPiecewiseReverse enabled?
     * @return True if enabled
     */
    bool getCheckPiecewiseReverse() const;

    /**
     * @brief Enable/disable CheckPiecewiseReverse?
     * @param value enable?
     */
    void setCheckPiecewiseReverse(bool value);

    /**
     * @brief Get CheckReverseOption?
     * @return CheckReverseOption
     */
    int getCheckReverseOption() const;

    /**
     * @brief Is CheckReverseOption enabled?
     * @param value CheckReverseOption
     */
    void setCheckReverseOption(int value);

    /**
     * @brief Get corner strategy for sharp corners
     * @return Corner strategy for sharp corners
     */
    SmoothenPathCornerStrategy getCornerStrategy_sharpCorners() const;

    /**
     * @brief Set corner strategy for sharp corners
     * @param value Corner strategy for sharp corners
     */
    void setCornerStrategy_sharpCorners(const SmoothenPathCornerStrategy &value);

    /**
     * @brief Get corner strategy for non-sharp corners
     * @return Corner strategy for non-sharp corners
     */
    SmoothenPathCornerStrategy getCornerStrategy_nonSharpCorners() const;

    /**
     * @brief Set corner strategy for non-sharp corners
     * @param value Corner strategy for non-sharp corners
     */
    void setCornerStrategy_nonSharpCorners(const SmoothenPathCornerStrategy &value);

protected:

    /**
     * @brief Smoothen a path (internal)
     * @param points_us Input path (unsampled)
     * @param radius Radius
     * @param checkReverse Check smoothing in reverse?
     * @return Smoothen path (empty on error)
     */
    std::vector<Point> smoothenPath_internal(std::vector<Point> points_us, double radius, bool checkReverse );

    /**
     * @brief Smoothen a path (piecewise)
     * @param points_us Input path (unsampled)
     * @param radius Radius
     * @return Smoothen path (empty on error)
     */
    std::vector<Point> smoothenPath_piecewise(std::vector<Point> points_us, double radius);


    /**
     * @brief Smoothen a path recursivelly
     * @param [in/out] points_us Input path (unsampled)
     * @param ind_from Index of the first point from where to start the smoothing
     * @param [in/out] points_out Smoothen path
     * @param radius Radius
     * @param angRes_rad Angular resulution [Rad]
     * @param angThreshold_rad Angular threshold [Rad]
     * @param allowMultiCornerSmoothening Allow multi-corner smoothing?
     * @param cornerStrategy_sharpCorners Ccorner strategy for sharp corner
     * @param cornerStrategy_nonSharpCorners Ccorner strategy for non-sharp corner
     * @return True on success
     */
    bool smoothenPath_recursive(std::vector<Point>& points_us, size_t ind_from,
                                std::vector<Point>& points_out,
                                double radius, double angRes_rad, double angThreshold_rad,
                                bool allowMultiCornerSmoothening,
                                SmoothenPathCornerStrategy cornerStrategy_sharpCorners,
                                SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners );

    /**
     * @brief [deprecated] Smoothen a path recursivelly
     * @param [in/out] points_us Input path (unsampled)
     * @param ind_from Index of the first point from where to start the smoothing
     * @param [in/out] points_out Smoothen path
     * @param radius Radius
     * @param angRes_rad Angular resulution [Rad]
     * @param angThreshold_rad Angular threshold [Rad]
     * @param allowMultiCornerSmoothening Allow multi-corner smoothing?
     * @param cornerStrategy_sharpCorners Ccorner strategy for sharp corner
     * @param cornerStrategy_nonSharpCorners Ccorner strategy for non-sharp corner
     * @return True on success
     */
    bool smoothenPath_recursive_old(std::vector<Point>& points_us, size_t ind_from,
                                std::vector<Point>& points_out,
                                double radius, double angRes_rad, double angThreshold_rad,
                                bool allowMultiCornerSmoothening,
                                SmoothenPathCornerStrategy cornerStrategy_sharpCorners,
                                SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners );

    /**
     * @brief Find the index corresponding to the next segment that is long enough to perform smoothing
     * @param [in/out] points_us Input path (unsampled)
     * @param ind_from Index of the first point from where to start the smoothing
     * @param radius Radius
     * @param angThreshold_rad Angular threshold [Rad]
     * @param cornerStrategy_sharpCorners Ccorner strategy for sharp corner
     * @param cornerStrategy_nonSharpCorners Ccorner strategy for non-sharp corner
     * @param includeFirstSegment Include the first segment in the analysis?
     * @param distMult Distance factor/multiplier to be used in the analysis
     * @return Index
     */
    size_t findIndOfNextLongEnoughSegment(std::vector<Point>& points_us, size_t ind_from, double radius, double angThreshold_rad,
                                          SmoothenPathCornerStrategy cornerStrategy_sharpCorners, SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners,
                                          bool includeFirstSegment, double distMult = 1);

    /**
     * @brief Check if a corner is considered to be already smooth
     * @param p0 First point of the segment
     * @param p1 Second point of the segment (corner)
     * @param p2 Third point of the segment
     * @param radius Radius
     * @param angRes_rad Angular resulution [Rad]
     * @param angThreshold_rad Angular threshold [Rad]
     * @param [out] corner Recomputed corner points on smooth corners
     * @return True if smooth
     */
    bool isCornerSmooth(const Point& p0,
                                      const Point& p1,
                                      const Point& p2,
                                      double radius,
                                      double angRes_rad,
                                      double angThreshold_rad,
                                      std::vector<Point>& corner);

    /**
     * @brief Check if the next segment is valid to make a simple corner smoothing
     * @param [in/out] points_us Input path (unsampled)
     * @param ind_from Index of the first point from where to start checking
     * @param radius Radius
     * @param angThreshold_rad Angular threshold [Rad]
     * @param dist0 Minimum distance needed between the corner and the first point
     * @param cornerStrategy_sharpCorners Ccorner strategy for sharp corner
     * @param cornerStrategy_nonSharpCorners Ccorner strategy for non-sharp corner
     * @return True if valid
     */
    bool checkNextSegments(std::vector<Point>& points_us, size_t ind_from,
                                         double radius, double angThreshold_rad, double dist0,
                                         SmoothenPathCornerStrategy cornerStrategy_sharpCorners, SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners);

    /**
     * @brief Get the smooth-corner points from a (semi-) circle
     * @param [in/out] points_us Input path (unsampled)
     * @param ind_p0 Index of the first point from where to start the connection
     * @param ind_pn Index of the last point
     * @param pStart Point where the semi-circle will start
     * @param pFinish Point where the semi-circle will finish
     * @param centroid Centroid of the circle
     * @param radius Radius
     * @param angRes_rad Angular resolution [Rad]
     * @param angThreshold_rad Angular threshold [Rad]
     * @param negativeAngDir Generate the points following a negative direction?
     * @param checkNextSegs Check next segments?
     * @param cornerStrategy_sharpCorners Ccorner strategy for sharp corner
     * @param cornerStrategy_nonSharpCorners Ccorner strategy for non-sharp corner
     * @return Smooth-corner points (empty on error)
     */
    std::vector<Point> getSmoothCornerFromCircle(std::vector<Point>& points_us, size_t ind_p0, size_t ind_pn,
                                                 Point pStart, Point pFinish, const Point& centroid,
                                                 double radius, double angRes_rad, double angThreshold_rad,
                                                 bool negativeAngDir, bool checkNextSegs,
                                                 SmoothenPathCornerStrategy cornerStrategy_sharpCorners,
                                                 SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners);

    /**
     * @brief Get the smooth-corner points from (semi-) circles
     * @param p0 Start point
     * @param pn End point
     * @param pCutStart Point where the central circle intersects with the first segment
     * @param pCutFinish Point where the central circle intersects with the last segment
     * @param centroid Centroid of the circle
     * @param radius Radius
     * @param angRes_rad Angular resolution [Rad]
     * @param negativeAngDir Generate the points following a negative direction?
     * @param checkNextSegs Check next segments?
     * @return Smooth-corner points (empty on error)
     */
    std::vector<Point> getSmoothCornerFromCircles(Point& p0, Point& pn,
                                                  Point pCutStart, Point pCutFinish, const Point& centroid,
                                                  double radius, double angRes_rad, bool negativeAngDir, bool checkNextSegs);

    /**
     * @brief Get the smooth-corner points using dubins paths
     * @param p0 First point of the segment
     * @param p1 Second point of the segment (corner)
     * @param p2 Third point of the segment
     * @param distFromCorner Distance from corner from where the path will be generated
     * @param radius Radius
     * @param angRes_rad Angular resolution [Rad]
     * @return Smooth-corner points (empty on error)
     */
    std::vector<Point> getSmoothCorner_dubins(Point& p0, Point& p1, Point& p2, double distFromCorner, double radius, double angRes_rad);


    /**
     * @brief Smoothen a path starting with dubins paths
     * @param [in/out] points_us Input path (unsampled)
     * @param ind_from Index of the first point from where to start the smoothing
     * @param [in/out] points_out Smoothen path
     * @param radius Radius
     * @param angRes_rad Angular resulution [Rad]
     * @param angThreshold_rad Angular threshold [Rad]
     * @param cornerStrategy_sharpCorners Ccorner strategy for sharp corner
     * @param cornerStrategy_nonSharpCorners Ccorner strategy for non-sharp corner
     * @return True on success
     */
    bool smoothenPath_dubins(std::vector<Point>& points_us, size_t ind_from,
                                                       std::vector<Point>& points_out,
                                                       double radius, double angRes_rad, double angThreshold_rad,
                                                       SmoothenPathCornerStrategy cornerStrategy_sharpCorners,
                                                       SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners);


    /**
     * @brief Get the smooth path corresponding to a segment with multiple consecutive corners
     * @param [in/out] points_us Input path (unsampled)
     * @param ind_from Index of the first point from where to start the smoothing
     * @param [in/out] points_out Smoothen path
     * @param radius Radius
     * @param angRes_rad Angular resulution [Rad]
     * @param angThreshold_rad Angular threshold [Rad]
     * @param cornerStrategy_sharpCorners Ccorner strategy for sharp corner
     * @param cornerStrategy_nonSharpCorners Ccorner strategy for non-sharp corner
     * @return True on success
     */
    bool getMultiCornerSmoothPath(std::vector<Point>& points_us, size_t ind_from,
                                                std::vector<Point>& points_out,
                                                double radius, double angRes_rad, double angThreshold_rad,
                                                SmoothenPathCornerStrategy cornerStrategy_sharpCorners,
                                                SmoothenPathCornerStrategy cornerStrategy_nonSharpCorners);

    /**
     * @brief Sample a line based on relative distances
     * @param p0 First point of the line
     * @param p1 Second point of the line
     * @param samplesRef Set containing the relative distances [0,1] (relative to the length of the line) where the samples should be generated
     * @return Sampled line
     */
    std::vector<Point> sampleLine(const Point &p0, const Point &p1, const std::multiset<double> &samplesRef);



protected:

    double m_angRes = 10; /**< Angular resolution [deg] for the corners (disregarded when using dubin) */
    double m_angThreshold = 150; /**< Corners with angles higher than angThreshold won't be smoothen strictly (0°, 180°) */
    bool m_allowMultiCornerSmoothening = true; /**< Allow checking for smooth sub-paths paths for sub-paths that are not long enough to perform simple (1-corner) smoothing */
    bool m_allowDubins = true; /**< Allow using dubings in simple smoothing if it is not possible to do it with the default strategy */
    bool m_checkPiecewiseReverse = true;/**< If true, the input will be segmented into pieces which are smoothen separatelly in forward and reverse direction. the 'best' one is finally selected for each segment/piece */
    int m_checkReverseOption = 0; /**< If =0, no reverse check will be done to see if smoothening the input in reverse is beter. If <0, complete reverse check is done. If >0, the check will be done with reversing the complete input path iif the unsampled input has <= samples than the given number */
    SmoothenPathCornerStrategy m_cornerStrategy_sharpCorners = SPCS_DO_NOT_REACH_CORNER_POINT_IF_POSSIBLE; /**< Strategy to deal with corners that form an (absolute) angle < 90° (applicable only for simple smoothing) */
    SmoothenPathCornerStrategy m_cornerStrategy_nonSharpCorners = SPCS_DO_NOT_REACH_CORNER_POINT; /**< Strategy to deal with corners that form an (absolute) angle >= 90° (applicable only for simple smoothing) */

};


}
}
#endif //AROLIB_PATH_SMOOTHER_HPP
