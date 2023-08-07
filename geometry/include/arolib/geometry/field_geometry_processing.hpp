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
 
#ifndef _AROLIB_FIELD_GEOMETRY_PROCESSING_H_
#define _AROLIB_FIELD_GEOMETRY_PROCESSING_H_

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include "arolib/types/field.hpp"
#include "arolib/types/polygon.hpp"
#include "arolib/geometry/geometry_helper.hpp"

namespace arolib{

namespace geometry{

/**
 * @brief Get the shortest connection between 2 points over a given ser of roads
 * @param pStart Start point
 * @param pFinish Finish point
 * @param roads Available roads
 * @param res Sample resolution for the output connection
 * @return Connection points
 */
std::vector<Point> getBestRoadConnection( const Point &pStart, const  Point &pFinish, const std::vector<Linestring> &roads, double res );



/**
 * @brief Strategies to connect inner-field tracks to the headland
 */
enum HeadlandConnectionStrategy{
    MIN_DIST_TO_HEADLAND_SEGMENT, /**< Connect with the line corresponding to the minimum distance between the track-end point and the headland */
    KEEP_TRACK_SEGMENT_DIRECTION /**< Connect keeping the direction of the track end-segment to be connected */
};

/**
 * @brief Get the HeadlandConnectionStrategy from its int value
 * @param value Int value
 * @return HeadlandConnectionStrategy
 */
HeadlandConnectionStrategy intToHeadlandConnectionStrategy(int value);


/**
 * @brief Get the points connecting two segments through the headland
 * @param headland [in/out] Headland points
 * @param value p0_1 First point of the first segment
 * @param value p1_1 Second point of the first segment
 * @param value p0_2 First point of the second segment
 * @param value p1_2 Second point of the second segment
 * @param connection [out] Connection points
 * @param strategy Headland connection strategy
 * @param addConnectionToHeadland Should the connection points be added to the input headland points?
 * @param unsamplingTolerance Unsampling tolerance
 * @return True on success
 */
bool getConnectionToHeadland(std::vector<Point> &headland,
                             const Point &p0_1,
                             const Point &p1_1,
                             const Point &p0_2,
                             const Point &p1_2,
                             std::vector<Point> &connection,
                             const HeadlandConnectionStrategy& strategy,
                             const bool &addConnectionToHeadland,
                             const double unsamplingTolerance = 0.1);


/**
 * @brief Get the point connecting a segment to the headland
 * @param headland [in/out] Headland points
 * @param value p0 First point of the segment
 * @param value p1 Second point of the segment
 * @param connection [out] Connection point in the headland
 * @param strategy Headland connection strategy
 * @param addConnectionToHeadland Should the connection points be added to the input headland points?
 * @return True on success
 */
bool getConnectionToHeadland(std::vector<Point> &headland,
                             const Point &p0,
                             const Point &p1,
                             Point &connection,
                             const HeadlandConnectionStrategy &strategy,
                             const bool &addConnectionToHeadland);

/**
 * @brief Get a subfield estimating its basic geometries from a route
 * @param route_points Route points
 * @param subfield_id Id of the output subfield
 * @return True on success
 */
Subfield getSubieldFromRoutePoints(std::vector<RoutePoint> route_points, int subfield_id = 0);

/**
 * @brief Get a field estimating its basic geometries from its subfields
 * @param subfields Subfields
 * @return True on success
 */
Field getFieldFromSubfields(const std::vector<Subfield> &subfields);

/**
 * @brief Get a list holding the tracks that are adjacent (ordered by index)
 *
 * @param tracks Tracks (sampled)
 * @param withRepeatedEntries If false, No repeated entries
 * @return List
 */
std::map<size_t, std::set<size_t> > getAdjacentTracksList(const std::vector<Track> &tracks, bool withRepeatedEntries = false);

/**
 * @brief Check if two tracks are adjacent based on a list holding the tracks that are adjacent (ordered by index)
 *
 * @sa getAdjacentTracksList
 * @param list List holding adjacency info
 * @param indTrack1 Index of track 1
 * @param indTrack2 Index of track 2
 * @return True if adjacent
 */
bool areTracksAdjacent(const std::map<size_t, std::set<size_t> > &list, size_t indTrack1, size_t indTrack2);

/**
 * @brief Check if two tracks are adjacent
 * @param t1 track 1
 * @param t2 track 2
 * @return True if adjacent
 */
bool areTracksAdjacent(const Track& t1, const Track& t2);

/**
 * @brief Get the indexes of the infield tracks located at the extrema
 * @param sf subfield
 * @return indexes of the infield tracks located at the extrema
 */
std::vector<size_t> getInfieldExtremaTracksIndexes(const Subfield& sf, const std::set<size_t> &excludeTrackIndexes = {});

/**
 * @brief Get the direction of motion from one track to another
 * @param trackFrom trackFrom points
 * @param trackTo trackTo points
 * @return direction
 */
Point getDirectionBetweenTracks(std::vector<Point> trackFrom, std::vector<Point> trackTo);

/**
 * @brief Rotate the field geometries
 * @param [in/out ]field field to be rotated
 * @param angle_rad Rotation angle (Radians)
 * @param pivot Rotation pivot (if invalid, uses the center of geometry of the field boundary)
 * @return True on success
 */
bool rotateField(Field& field, double angle_rad, Point pivot = Point::invalidPoint());

/**
 * @brief Rotate the subfield geometries
 * @param [in/out] sf subfield to be rotated
 * @param angle_rad Rotation angle (Radians)
 * @param pivot Rotation pivot (if invalid, uses the center of geometry of the sf outer boundary)
 * @return True on success
 */
bool rotateSubfield(Subfield& sf, double angle_rad, Point pivot = Point::invalidPoint());

/**
 * @brief Rotate the Headlands geometries
 * @param [in/out] hls Headlands to be rotated
 * @param angle_rad Rotation angle (Radians)
 * @param pivot Rotation pivot
 * @return True on success
 */
bool rotateHeadlands(Headlands& hls, double angle_rad, const Point& pivot);

/**
 * @brief Rotate the Headland geometries
 * @param [in/out] hl Headland to be rotated
 * @param angle_rad Rotation angle (Radians)
 * @param pivot Rotation pivot
 * @return True on success
 */
bool rotateHeadland(CompleteHeadland& hl, double angle_rad, const Point& pivot);

/**
 * @brief Rotate the Headland geometries
 * @param [in/out] hl Headland to be rotated
 * @param angle_rad Rotation angle (Radians)
 * @param pivot Rotation pivot
 * @return True on success
 */
bool rotateHeadland(PartialHeadland& hl, double angle_rad, const Point& pivot);

/**
 * @brief Rotate the track geometries
 * @param [in/out] track Track to be rotated
 * @param angle_rad Rotation angle (Radians)
 * @param pivot Rotation pivot
 * @return True on success
 */
bool rotateTrack(Track& track, double angle_rad, const Point& pivot);

/**
 * @brief Rotate the track geometries
 * @param [in/out] obs Obstacle to be rotated
 * @param angle_rad Rotation angle (Radians)
 * @param pivot Rotation pivot
 * @return True on success
 */
bool rotateObstacle(Obstacle& obs, double angle_rad, const Point& pivot);

}
}


#endif
