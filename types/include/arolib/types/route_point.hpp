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
 
#ifndef _AROLIB_ROUTE_POINT_HPP
#define _AROLIB_ROUTE_POINT_HPP

#include <vector>
#include <set>

#include "point.hpp"
#include "machine.hpp"

namespace arolib
{

  /**
  * @brief Route-point class
  */
  class RoutePoint : public Point
  {
  public:
    /**
   * @brief Route-point types.
   *
   * - HARVESTER ROUTE
   *    - @b DEFAULT: default.
   *    - @b INITIAL_POSITION: Initial location of the machine.
   *    - @b TRACK_START: Harvesting-point refering to a starting-point of a track (for infield and headland tracks).
   *    - @b TRACK_END: Harvesting-point refering to an ending-point of a track (for infield and headland tracks).
   *    - @b DEFAULT: Harvesting-point (excluding TRACK_START and TRACK_END points) for infield and headland harvesting.
   *    - @b HEADLAND:
   *        - Inner-field harvesting: Refers a point in the inter-track (headland) connections
   *        - Headland harvesting: Refers to point in the headland-boundary (normal harvesting points in headland harvesting are not typed HEADLAND, but TRACK_START, DEFAULT and TRACK_END)
   *        - Other:
   *            - Driving point from the initial location to the first harvesting point (excluding field entry and initial-position points).
   *            - Driving point from the last harvesting point to the field exit (excluding field exit points).
   *            - Driving point from the last harvesting point in headland harvesting to the first harvesting point in inner-field harvesting (connection between headland and inner-field routes).
   *    - @b OVERLOADING_START: @a N.A.
   *    - @b OVERLOADING_FINISH: @a N.A.
   *    - @b OVERLOADING: @a N.A.
   *    - @b FIELD_ENTRY: At a field entry point
   *    - @b FIELD_EXIT: At a field exit point
   *    - @b RESOURCE_POINT: At a resource point (silo)
   *    - @b HARVESTING: Harvesting-point (excluding TRACK_START and TRACK_END points) for infield and headland harvesting.
   *    - @b TRANSIT: The machine is in normal transit/driving without any special connotation.
   *
   * - OLV ROUTE
   *    - @b DEFAULT: default.
   *    - @b INITIAL_POSITION: Initial location of the machine
   *    - @b TRACK_START: @a N.A.
   *    - @b TRACK_END: @a N.A.
   *    - @b DEFAULT: NON-overloading driving points inside the field.
   *    - @b HEADLAND: @a N.A.
   *    - @b OVERLOADING_START: Overloading point corresponding to the start of the overload window (first overloading point of the window).
   *    - @b OVERLOADING_FINISH: Overloading point corresponding to the end of the overload window (last overloading point of the window).
   *    - @b OVERLOADING: Overloading point, excluding start/end of overloading, which have specific types (see above).
   *    - @b FIELD_ENTRY: At a field entry point
   *    - @b FIELD_EXIT: At a field exit point
   *    - @b RESOURCE_POINT: At a resource point (silo)
   *    - @b HARVESTING: @a N.A.
   *    - @b TRANSIT: The machine is in normal transit/driving without any special connotation.
   */
    enum RoutePointType
    {
      DEFAULT,            /**< [ALL] default. */
      TRACK_START,        /**< [WorkingTypes] Starting-point of a track (for infield and headland tracks) */
      TRACK_END,          /**< [WorkingTypes] Ending-point of a track (for infield and headland tracks) */
      RESOURCE_POINT,     /**< [ALL] At a resource point / silo */
      FIELD_ENTRY,        /**< [ALL] At a field entry point */
      FIELD_EXIT,         /**< [ALL] At a field exit point */
      OVERLOADING_START,  /**< [OLV] Start of the overload window (first overloading point of the window).*/
      OVERLOADING_FINISH, /**< [OLV] End of the overload window (last overloading point of the window). */
      OVERLOADING,        /**< [OLV] Overloading (except for start/end of overloading, which have specific types). */
      HEADLAND,           /**< [WorkingTypes] See enum description. */
      INITIAL_POSITION,   /**< [ALL] Initial location of the machine */
      HARVESTING,         /**< [HARV] Harvesting */
      TRANSIT,            /**< [ALL] The machine is in transit (driving) */
      SEEDING,            /**< [SEEDER] Seeding */
      SPRAYING,           /**< [SPRAYER] Spraying */
      CULTIVATING,        /**< [CULTIVATOR] Cultivating */
      PLOUGHING,          /**< [PLOUGH] Plough */
      SCANNING = 50,      /**< [SCANNER] Field scanning */
      TRANSIT_OF = 60,    /**< [ALL] The machine is in transit (driving) outside of the field */
    };

    /**
      * @brief Get the RoutePointType (enum) from its int value
      */
    static RoutePointType intToRoutePointType(int value);

    static const std::set<RoutePointType> WorkingRoutePointTypes;/**< Set containing the route point types corresponding to a 'working the field' operation, i.e. to working (primary) machines */

    /**
      * @brief Struct holding the relations of a route point with other machines/routes
      */
    struct MachineRelationInfo
    {
      MachineId_t machine_id;        /**< Id of the machine to which the route point is 'related' */
      int route_id;                  /**< Id of the route to which the route point is 'related' */
      RoutePointType routePointType; /**< Type of the route-point to which the route point is 'related' */
      size_t routePointIndex;        /**< Index of the route-point (w.r.t. its route) to which the route point is 'related' */

      /**
        * @brief operator==
        */
      bool operator==(const MachineRelationInfo &other) const;

      /**
        * @brief operator!=
        */
      bool operator!=(const MachineRelationInfo &other) const;
    };

    /**
      * @brief Struct to get hash value
      */
    struct KeyHash
    {
      std::size_t operator()(const RoutePoint &p) const;
      static std::size_t get(const RoutePoint &p, std::size_t seed = 0);
    };

    /**
      * @brief Default constructor
      */
    explicit RoutePoint() = default;

    /**
      * @brief Constructor with location
      */
    explicit RoutePoint(double _x, double _y, double _z=0) : Point(_x, _y, _z) {}

    /**
      * @brief operator < (used for sorting --> no 'quantitative/qualitative' meaning)
      *
      * The order of checking is: 1) track id; 2) track index; 3) Type; 4) location (point)
      * @param rp Other route point used for the comparison
      * @return True if the (local) route point has lower attributes tha rp (following the order in the method description)
      */
    bool operator<(const RoutePoint &rp) const;

    /**
      * @brief operator ==
      * @param other Other route point used for the comparison
      * @return True if the route points the same attributes
      */
    bool operator==(const RoutePoint &other) const;

    /**
      * @brief operator !=
      * @param other Other route point used for the comparison
      * @return True if the route points have different attributes
      */
    bool operator!=(const RoutePoint &other) const;

    /**
      * @brief Get a reference to the location (point)
      * @return Reference to the location (point)
      */
    inline Point &point()
    {
      return *((Point *)this);
    }

    /**
      * @brief Get a reference to the location (point)
      * @return Reference to the location (point)
      */
    inline const Point &point() const
    {
      return *((Point *)this);
    }

    /**
      * @brief Check if the route point is of any of the given types
      * @param types Types to be compared
      * @return True if the type of the route-point is equal to any of the input types
      */
    inline bool isOfType(const std::set<RoutePointType> &types) const
    {
      return types.find(type) != types.end();
    }

    /**
      * @brief Check if the route point is of any of the working types (excluding track start/end)
      * @return True if the type of the route-point is equal to any of the working types (excluding track start/end)
      */
    inline bool isOfTypeWorking_InTrack(bool includeScanning = false, bool includeDefault = false) const
    {
      return (WorkingRoutePointTypes.find(type) != WorkingRoutePointTypes.end() && type != TRACK_START && type != TRACK_END) || (type == SCANNING && includeScanning) || (type == DEFAULT && includeDefault); //@todo add other working types
    }

    /**
      * @brief Check if the route point is of any of the working types
      * @return True if the type of the route-point is equal to any of the working types
      */
    inline bool isOfTypeWorking(bool includeScanning = false, bool includeDefault = false) const
    {
      return WorkingRoutePointTypes.find(type) != WorkingRoutePointTypes.end() || (type == SCANNING && includeScanning) || (type == DEFAULT && includeDefault); //@todo add other working types
    }

    /**
      * @brief Check if the route point is related to the given working type
      * @return True if the type of the route-point is equal to any of the related working types
      */
    inline bool isOfWorkingType(RoutePointType baseType) const
    {
      return isOfTypeWorking(true) && (type == baseType || type == TRACK_START || type == TRACK_END);
    }

    /**
      * @brief Check if the route point is of field access type
      * @return True if the route point is of field access type
      */
    inline bool isFieldAccess() const
    {
      return type == FIELD_ENTRY || type == FIELD_EXIT;
    }

    /**
      * @brief Get the default route point type based on the machine
      * @param machine Machine
      * @return Default type
      */
    static RoutePointType getDefaultRPType(const Machine &machine);


    /**
      * @brief Get the route point type as a string description
      * @param type RoutePointType
      * @return Type as string
      */
    static std::string RPTypeToString(RoutePointType type);

    /**
      * @brief Get the route point type as a 2-character-string description
      * @param type RoutePointType
      * @return Type as string
      */
    static std::string RPTypeToShortString2c(RoutePointType type);

    /**
      * @brief Get the index of the next route point that has any of the given types
      * @param route_points Route points
      * @param types Types to be compared
      * @param types Start index (inclusive). The comparison will start from this index.
      * @param indn Stop index (inclusive). The comparison will stop from this index. Disregarded if <0;
      * @return Index of the next route point whose type is equal to any of the given types. If no route-point was found, returns route_points.size().
      */
    static size_t getNextIndByType(const std::vector<RoutePoint> &route_points, const std::set<RoutePointType> &types, size_t ind0 = 0, int indn = -1);

    /**
      * @brief Copies the basic working values (bunker mass/volume, worked mass/volume) from another point
      */
    void copyBasicWorkingValuesFrom(const RoutePoint &from);

  public:
    double time_stamp = -1; /**< Time stamp [s] */
    double bunker_mass = 0; /**< Mass [kg] of the yield in the bunker */
    double bunker_volume = 0; /**< Volume [m³] of the yield in the bunker */
    double harvested_mass = 0;/**< (excusive for harvester routes) Yield-mass [kg] harvested in the route until this point (used to keep track of how much the harvesters have harvested) */
    double harvested_volume = 0; /**< (excusive for harvester routes) Yield-volume [m³] harvested in the route until this point (used to keep track of how much the harvesters have harvested) */
    int track_id = -99; /**< Id of the track to which the route-point belongs */
    int track_idx = -1; /**< Index of the track to which the route-point belongs */
    RoutePointType type = DEFAULT; /**< Type of the route point */
    std::vector<MachineRelationInfo> machineRelations; /**< Relation that this route-point has with other machines/routes */
  };


  /**
    * @brief Overload << operator to stream RoutePoint
    */
  inline std::ostream &operator<<(std::ostream &out, const arolib::RoutePoint &point)
  {
    out << point.time_stamp << ": " << point.toString();
    return out;
  }
}

#endif // _AROLIB_ROUTE_POINT_HPP
