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
 
#ifndef _AROLIB_OUTFIELDINFO_H_
#define _AROLIB_OUTFIELDINFO_H_

#include <map>
#include <vector>
#include <iostream>
#include <string>
#include <cmath>

#include "fieldaccesspoint.hpp"
#include "resourcepoint.hpp"

namespace arolib {
/**
  * @brief Class holding information of out-of-field actions (driving, unloading, etc...)
  */
class OutFieldInfo {

public:
    typedef int MachineId_t; /**< Machine id type */

    static const MachineId_t AllMachines; /**< Constant value to refer to ALL machines, as oppose to a specific one (a specific id) */
    static const ResourcePointId_t AllResourcePoints; /**< Constant value to refer to ALL resource/silo points, as oppose to a specific one (a specific id) */
    static const FieldAccessPointId_t AllFieldAccessPoints; /**< Constant value to refer to ALL field access point points, as oppose to a specific one (a specific id) */

    /**
      * @brief Struct holding travel costs
      */
    struct TravelCosts{
        double time; /**< Travel time [s] */
        double time_per_kg; /**< Travel time per kilogram [s/kg] */
        double distance; /**< Travel distance [m] */

        /**
          * @brief Constructor
          */
        explicit TravelCosts();

        /**
          * @brief Constructor with arguments
          * @param _time Travel time [s]
          * @param _time_per_kg Travel time per kilogram [s/kg]
          * @param _distance Travel distance [m]
          */
        explicit TravelCosts(double _time, double _time_per_kg, double _distance);
    };

    /**
      * @brief Struct holding unloading costs (for unloading of yield in resource/silo points)
      */
    struct UnloadingCosts{
        double time; /**< Unloading time [s] */
        double time_per_kg; /**< Unloading time per kilogram [s/kg] */

        /**
          * @brief Constructor
          */
        explicit UnloadingCosts();

        /**
          * @brief Constructor with arguments
          * @param _time Unloading time [s]
          * @param _time_per_kg Unloading time per kilogram [s/kg]
          */
        explicit UnloadingCosts(double _time, double _time_per_kg);
    };

    /**
      * @brief State of the bunker of a machine
      */
    enum MachineBunkerState{
        MACHINE_EMPTY, /**< The bunker is empty */
        MACHINE_LOADED, /**< The bunker is full/loaded */
        ALL_MACHINE_STATES /**< Enum value refering to all states */
    };
    /**
      * @brief Get the MachineBunkerState (enum) from its int value
      */
    static MachineBunkerState intToMachineBunkerState(int value);

    /**
      * @brief Struct holding costs/information for a travel between a specific field access point and a specific resource/silo point, for a specific machine with a specific bunker state
      *
      * Example for non-specific values:
      *     If fieldAccessPointId = 'id_fap', resourcePointId = 'id_rp', machineId = AllMachines, machineBunkerState = ALL_MACHINE_STATES, then the stated travel costs apply for ALL the machines for travels between the field access point with id 'id_fap' and the resource point with id 'id_rp', disregarding their bunker state (e.g. all machines need the same time to reach a certain access point from a silo, doesn't matter how full they are)
      */
    struct TravelData{
        FieldAccessPointId_t fieldAccessPointId; /**< Id of the field access point (if = AllFieldAccessPoints, it applies to all access points: @sa AllFieldAccessPoints) */
        ResourcePointId_t resourcePointId; /**< Id of the resource/silo point (if = AllResourcePoints, it applies to all resource/silo points: @sa AllResourcePoints) */
        MachineId_t machineId = AllMachines; /**< Id of the machine (if = AllMachines, it applies to all machines: @sa AllMachines) */
        MachineBunkerState machineBunkerState = ALL_MACHINE_STATES; /**< State of the bunker of the machine (if = ALL_MACHINE_STATES, it applies to all bunker states.) */
        TravelCosts travelCosts;
    };
    struct TravelData2{
        FieldAccessPointId_t fap_id_from; /**< Id of the field access point (if = AllFieldAccessPoints, it applies to all access points: @sa AllFieldAccessPoints) */
        FieldAccessPointId_t fap_id_to; /**< Id of the resource/silo point (if = AllResourcePoints, it applies to all resource/silo points: @sa AllResourcePoints) */
        MachineId_t machineId = AllMachines; /**< Id of the machine (if = AllMachines, it applies to all machines: @sa AllMachines) */
        MachineBunkerState machineBunkerState = ALL_MACHINE_STATES; /**< State of the bunker of the machine (if = ALL_MACHINE_STATES, it applies to all bunker states.) */
        TravelCosts travelCosts;
    };

    /**
      * @brief Struct holding costs/information for unloading in a specific resource/silo point, for a specific machine
      *
      * Example for non-specific values:
      *     If resourcePointId = AllResourcePoints, machineId = id1, then the stated unloading costs apply for the machine with id 'id1' for ALL resource points (e.g. this machine need the same time to download an any of the silos)
      */
    struct UnloadingData{
        ResourcePointId_t resourcePointId; /**< Id of the resource/silo point (if = AllResourcePoints, it applies to all resource/silo points: @sa AllResourcePoints) */
        MachineId_t machineId; /**< Id of the machine (if = AllMachines, it applies to all machines: @sa AllMachines) */
        UnloadingCosts unloadingCosts;
    };

    /**
      * @brief Struct holding costs/information for a travel between a machine's initial location and a specific field access point, for a specific machine with a specific bunker state
      *
      * Example for non-specific values:
      *     If fieldAccessPointId = 'id_fap', machineId = 'id1', machineBunkerState = ALL_MACHINE_STATES, then the stated arrivaltravel costs apply for the machine with id 'id1' to travel to the field access point with id 'id_fap', disregarding its bunker state (e.g. this machine needs the same time to reach a certain access point from its initial location when it is empty or full)
      */
    struct ArrivalData{
        FieldAccessPointId_t fieldAccessPointId; /**< Id of the field access point (if = AllFieldAccessPoints, it applies to all access points: @sa AllFieldAccessPoints) */
        MachineId_t machineId; /**< Id of the machine (if = AllMachines, it applies to all machines: @sa AllMachines) */
        MachineBunkerState machineBunkerState; /**< State of the bunker of the machine (if = ALL_MACHINE_STATES, it applies to all bunker states.) */
        TravelCosts arrivalCosts;
    };

    typedef std::map< MachineBunkerState, TravelCosts > MapMachineStateTravelCosts_t;
    typedef std::map< MachineId_t , MapMachineStateTravelCosts_t > MapMachineTravelCosts_t;

    typedef std::map< FieldAccessPointId_t ,
                std::map< ResourcePointId_t ,
                    MapMachineTravelCosts_t > > MapAccessPoint2ResourcePoint_t; /**< Map holding the travel costs/info from field access points to resource/silo points (for specific machines and bunker states) */
    typedef std::map< ResourcePointId_t ,
                std::map< FieldAccessPointId_t ,
                    MapMachineTravelCosts_t > > MapResourcePoint2AccessPoint_t; /**< Map holding the travel costs/info from resource/silo points to field access points (for specific machines and bunker states) */
    typedef std::map< ResourcePointId_t ,
                std::map< MachineId_t ,
                    UnloadingCosts > > MapUnloadingCosts_t; /**< Map holding the unloading costs/info from one or more resource/silo points (for specific machines) */
    typedef std::map< FieldAccessPointId_t , MapMachineTravelCosts_t > MapArrivalCosts_t; /**< Map holding the arrival travel costs/info to field access points (for specific machines and bunker states) */
    typedef std::map< FieldAccessPointId_t ,
                std::map< FieldAccessPointId_t ,
                    MapMachineTravelCosts_t > > MapAccessPoint2AccessPoint_t; /**< Map holding the travel costs/info between field access points (for specific machines and bunker states) */


    /**
      * @brief Constructor
      */
    explicit OutFieldInfo();

    /**
      * @brief Remove all data
      */
    void clearAll();

    /**
      * @brief Remove all data refering to costs/info of travels between field access points and resource/silo points
      */
    void clearFAP2RP();

    /**
      * @brief Remove all data refering to costs/info of travels between resource/silo points and field access points
      */
    void clearRP2FAP();

    /**
      * @brief Remove all data refering to costs/info of travels between field access points
      */
    void clearFAP2FAP();

    /**
      * @brief Remove all data refering to unloading costs/info
      */
    void clearUnloadingCosts();

    /**
      * @brief Remove all data refering to arrival-travel costs/info
      */
    void clearArrivalCosts();

    /**
      * @brief Set the map holding the travel costs/info from field access points to resource/silo points (for specific machines and bunker states)
      * @param map Map to be set.
      */
    void setMapAccessPoint2ResourcePoint(const MapAccessPoint2ResourcePoint_t& map){ m_mapAccessPoint2ResourcePoint = map; }

    /**
      * @brief Set the map holding the travel costs/info from resource/silo points to field access points (for specific machines and bunker states)
      * @param map Map to be set.
      */
    void setMapResourcePoint2AccessPoint(const MapResourcePoint2AccessPoint_t& map){ m_mapResourcePoint2AccessPoint = map; }

    /**
      * @brief Set the map holding the travel costs/info between field access points (for specific machines and bunker states)
      * @param map Map to be set.
      */
    void setMapAccessPoint2AccessPoint(const MapAccessPoint2AccessPoint_t& map){ m_mapAccessPoint2AccessPoint = map; }

    /**
      * @brief Set the map holding the unloading costs/info from one or more resource/silo points (for specific machines)
      * @param map Map to be set.
      */
    void setMapUnloadingCosts(const MapUnloadingCosts_t& map){ m_mapUnloadingCosts = map; }

    /**
      * @brief Set the map holding the arrival travel costs/info to field access points (for specific machines and bunker states)
      * @param map Map to be set.
      */
    void setMapArrivalCosts(const MapArrivalCosts_t& map){ m_mapArrivalCosts = map; }


    /**
      * @brief Set the map holding the travel costs/info from field access points to resource/silo points (for specific machines and bunker states) from a vector of travel data (matrix/table-like form)
      * @param map Map to be set.
      */
    void fromMatrix_FAP2RP(const std::vector<TravelData>& matrixData);

    /**
      * @brief Set the map holding the travel costs/info from resource/silo points to field access points (for specific machines and bunker states) from a vector of travel data (matrix/table-like form)
      * @param map Map to be set.
      */
    void fromMatrix_RP2FAP(const std::vector<TravelData>& matrixData);

    /**
      * @brief Set the map holding the travel costs/info between field access points (for specific machines and bunker states) from a vector of travel data (matrix/table-like form)
      * @param map Map to be set.
      */
    void fromMatrix_FAP2FAP(const std::vector<TravelData2>& matrixData, bool bidirecctional = true);

    /**
      * @brief Set the map holding the unloading costs/info from one or more resource/silo points (for specific machines) from a vector of unloading data (matrix/table-like form)
      * @param map Map to be set.
      */
    void fromMatrix_unloadingCosts(const std::vector<UnloadingData>& matrixData);

    /**
      * @brief Set the map holding the arrival travel costs/info to field access points (for specific machines and bunker states) from a vector of arrival-travel data (matrix/table-like form)
      * @param map Map to be set.
      */
    void fromMatrix_arrivalCosts(const std::vector<ArrivalData>& matrixData);

    /**
      * @brief Add new travel data to the (internal) map holding the travel costs/info from field access points to resource/silo points (for specific machines and bunker states)
      * @param map Map to be set.
      */
    void add_FAP2RP(const TravelData& travelData);

    /**
      * @brief Add new travel data to the (internal) map holding the travel costs/info from resource/silo points to field access points (for specific machines and bunker states)
      * @param map Map to be set.
      */
    void add_RP2FAP(const TravelData& travelData);

    /**
      * @brief Add new travel data to the (internal) map holding the travel costs/info between field access points (for specific machines and bunker states)
      * @param map Map to be set.
      */
    void add_FAP2FAP(const TravelData2& travelData, bool bidirecctional = true);

    /**
      * @brief Add new unloading data to the the (internal) map holding the unloading costs/info from one or more resource/silo points (for specific machines)
      * @param map Map to be set.
      */
    void add_unloadingCosts(const UnloadingData& unloadingData);

    /**
      * @brief Add new arrival/travel data to the (internal) map holding the arrival travel costs/info to field access points (for specific machines and bunker states)
      * @param map Map to be set.
      */
    void add_arrivalCosts(const ArrivalData& arrivalData);


    /**
      * @brief Check if the internal maps are empty (i.e. no save travel data, unloading data, arrival data,...)
      * @param True if no data exists (maps are empty).
      */
    bool empty() const;

    /**
      * @brief Check if the internal maps holding the travel data between field access points and resource/silo points are empty
      * @param True if no travel data exists (both maps holding the travel data between field access points and resource/silo points are empty).
      */
    bool empty_travelCosts_FAP_RP() const;

    /**
      * @brief Get how many travel data instances are saved for travels from a specified field access point to a specified resource/silo point for the specified machine (i.e. size of the corresponding map)
      *
      * If fieldAccessPointId = AllFieldAccessPoints, returns the size of the FAP-to-RP travel costs/data map; otherwise...
      * ... if resourcePointId = AllResourcePoints, returns the amount of travel costs/data instances applicable for travels from the specified field access point; otherwise...
      * ... ... if MachineId_t machineId = AllMachines, returns the amount of travel costs/data instances applicable for travels from the specified field access point to the specified resource/silo point; otherwise...
      * ... ... ... returns the amount of travel costs/data instances applicable for the specified machine for travels from the specified field access point to the specified resource/silo point
      * @param fieldAccessPointId Id of the field access point (see method description)
      * @param resourcePointId Id of the resource/silo point (see method description)
      * @param machineId Id of the machine (see method description)
      * @return Amount of travel data instances saved for the given input parameters
      */
    size_t size_FAP2RP(FieldAccessPointId_t fieldAccessPointId = AllFieldAccessPoints,
                       ResourcePointId_t resourcePointId = AllResourcePoints,
                       MachineId_t machineId = AllMachines) const;

    /**
      * @brief Get how many travel data instances are saved for travels from a specified resource/silo point to a specified field access point for the specified machine (i.e. size of the corresponding map)
      *
      * If resourcePointId = AllResourcePoints, returns the size of the RP-to-FAP travel costs/data map; otherwise...@n
      * ... if fieldAccessPointId = AllFieldAccessPoints, returns the amount of travel costs/data instances applicable for travels from the specified resource/silo point; otherwise...@n
      * ... ... if MachineId_t machineId = AllMachines, returns the amount of travel costs/data instances applicable for travels from the specified field access point to the specified resource/silo point; otherwise...@n
      * ... ... ... returns the amount of travel costs/data instances applicable for the specified machine for travels from the specified resource/silo point to the specified field access point
      * @param resourcePointId Id of the resource/silo point (see method description)
      * @param fieldAccessPointId Id of the field access point (see method description)
      * @param machineId Id of the machine (see method description)
      * @return Amount of travel data instances saved for the given input parameters
      */
    size_t size_RP2FAP(ResourcePointId_t resourcePointId = AllResourcePoints,
                       FieldAccessPointId_t fieldAccessPointId = AllFieldAccessPoints,
                       MachineId_t machineId = AllMachines) const;

    /**
      * @brief Get how many travel data instances are saved for travels from a specified field access point to a specified field access point for the specified machine (i.e. size of the corresponding map)
      *
      * If fap_id_from = AllFieldAccessPoints, returns the size of the FAP-to-FAP travel costs/data map; otherwise...@n
      * ... if fap_id_to = AllFieldAccessPoints, returns the amount of travel costs/data instances applicable for travels from the specified access point 'from'; otherwise...@n
      * ... ... if MachineId_t machineId = AllMachines, returns the amount of travel costs/data instances applicable for travels from the specified field access point to the specified resource/silo point; otherwise...@n
      * ... ... ... returns the amount of travel costs/data instances applicable for the specified machine for travels from the specified field access points
      * @param fap_id_from Id of the 'from' access point (see method description)
      * @param fap_id_to Id of the 'to' access point (see method description)
      * @param machineId Id of the machine (see method description)
      * @return Amount of travel data instances saved for the given input parameters
      */
    size_t size_FAP2FAP(FieldAccessPointId_t fap_id_from = AllFieldAccessPoints,
                        FieldAccessPointId_t fap_id_to = AllFieldAccessPoints,
                        MachineId_t machineId = AllMachines) const;

    /**
      * @brief Get how many unloading data instances are saved a specified resource/silo point for the specified machine (i.e. size of the corresponding map)
      *
      * If resourcePointId = AllResourcePoints, returns the size of the unloading costs/data map; otherwise...
      * ... if MachineId_t machineId = AllMachines, returns the amount of unloading costs/data instances applicable for the specified resource/silo point; otherwise...
      * ... ... returns the amount of unloading costs/data instances applicable for the specified machine for the specified resource/silo point
      * @param resourcePointId Id of the resource/silo point (see method description)
      * @param machineId Id of the machine (see method description)
      * @return Amount of unloading data instances saved for the given input parameters
      */
    size_t size_unloadingCosts(ResourcePointId_t resourcePointId = AllResourcePoints,
                               MachineId_t machineId = AllMachines) const;

    /**
      * @brief Get how many arrival-travel data instances are saved for travels from the initial location of the specified machine to a specified field access point (i.e. size of the corresponding map)
      *
      * If fieldAccessPointId = AllFieldAccessPoints, returns the size of the arrival-travel costs/data map; otherwise...
      * ... if MachineId_t machineId = AllMachines, returns the amount of arrival travel costs/data instances applicable for travels from initial machine locations to the specified field access point; otherwise...
      * ... ... returns the amount of arrival-travel costs/data instances applicable for the specified machine for travels from its initial location to the specified field access point
      * @param fieldAccessPointId Id of the field access point (see method description)
      * @param machineId Id of the machine (see method description)
      * @return Amount of arrival-travel data instances saved for the given input parameters
      */
    size_t size_arrivalCosts(FieldAccessPointId_t fieldAccessPointId = AllFieldAccessPoints,
                               MachineId_t machineId = AllMachines) const;

    /**
      * @brief Check if there exists travel data for travel from any the given field access points to a specified resource point AND from the specified resource/silo point to any the given field access points (no necessarily the same access point)
      *
      * This method is used to know if it is possible to get to the resource/silo point from any of the given field access points and back to the field to any of the access points (using the saved travel data)
      * @param resourcePointId Id of the resource/silo point.
      * @param accessPoints Field access points
      * @return True if there exists travel data for bidirectional travel between the specified resource point and the given field access points
      */
    bool resourcePointBidirectionalConnectionExists(ResourcePointId_t resourcePointId,
                                                    const std::vector<FieldAccessPoint> &accessPoints = {}) const;

    /**
      * @brief Get travel costs/data for travels from a specified field access point to a specified resource/silo point for the specified machine and bunker state
      *
      * It first searches for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified fieldAccessPointId, resourcePointId, machineId and machineBunkerState (specific to this constellation), but there exists an entry for the specified fieldAccessPointId and resourcePointId, and for 'AllMachines' and 'ALL_MACHINE_STATES' it returns the data that applies for all machines and all bunker states for those field access and resource points
      * @param fieldAccessPointId Id of the field access point
      * @param resourcePointId Id of the resource/silo point
      * @param machineId Id of the machine
      * @param machineBunkerState Machine bunker state
      * @param [out] travelCosts Travel costs specific to the given inputs
      * @return True on success (i.e. if travel data exists for the specified inputs)
      */
    bool getTravelCost_FAP2RP(FieldAccessPointId_t fieldAccessPointId,
                              ResourcePointId_t resourcePointId,
                              MachineId_t machineId,
                              MachineBunkerState machineBunkerState,
                              TravelCosts& travelCosts) const;

    /**
      * @brief Get travel costs/data for travels from a specified field access point to a specified resource/silo point for the specified machine
      *
      * It first searches for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified fieldAccessPointId, resourcePointId, machineId and machineBunkerState (specific to this constellation), but there exists an entry for the specified fieldAccessPointId and resourcePointId, and for 'AllMachines' and 'ALL_MACHINE_STATES' it returns the data that applies for all machines and all bunker states for those field access and resource points
      * @param fieldAccessPointId Id of the field access point
      * @param resourcePointId Id of the resource/silo point
      * @param machineId Id of the machine
      * @param [out] travelCosts Travel costs specific to the given inputs
      * @return True on success (i.e. if travel data exists for the specified inputs)
      */
    bool getTravelCost_FAP2RP(FieldAccessPointId_t fieldAccessPointId,
                              ResourcePointId_t resourcePointId,
                              MachineId_t machineId,
                              MapMachineStateTravelCosts_t& travelCosts) const;

    /**
      * @brief Get travel costs/data (machine-based) map for travels from a specified field access point to a specified resource/silo point
      *
      * It first searches for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified fieldAccessPointId and resourcePointId (specific to this constellation), but there exists an entry for 'AllFieldAccessPoints' and for the specified resourcePointId, it returns the data that applies for travels from all field access points to the specified resource point
      * @param fieldAccessPointId Id of the field access point
      * @param resourcePointId Id of the resource/silo point
      * @param [out] travelCostsMap Travel costs map specific to the given inputs (the map hold the travel costs for specific machines and bunker states)
      * @return True on success (i.e. if travel data exists for the specified inputs)
      */
    bool getTravelCost_FAP2RP(FieldAccessPointId_t fieldAccessPointId,
                              ResourcePointId_t resourcePointId,
                              MapMachineTravelCosts_t& travelCostsMap) const;

    /**
      * @brief Get travel costs/data for travels from a specified resource/silo point to a specified field access point for the specified machine and bunker state
      *
      * It first searches for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified resourcePointId, fieldAccessPointId, machineId and machineBunkerState (specific to this constellation), but there exists an entry for the specified resourcePointId and fieldAccessPointId, and for 'AllMachines' and 'ALL_MACHINE_STATES' it returns the data that applies for all machines and all bunker states for those resource and field access points
      * @param resourcePointId Id of the resource/silo point
      * @param fieldAccessPointId Id of the field access point
      * @param machineId Id of the machine
      * @param machineBunkerState Machine bunker state
      * @param [out] travelCosts Travel costs specific to the given inputs
      * @return True on success (i.e. if travel data exists for the specified inputs)
      */
    bool getTravelCost_RP2FAP(ResourcePointId_t resourcePointId,
                              FieldAccessPointId_t fieldAccessPointId,
                              MachineId_t machineId,
                              MachineBunkerState machineBunkerState,
                              TravelCosts& travelCosts) const;

    /**
      * @brief Get travel costs/data for travels from a specified resource/silo point to a specified field access point for the specified machine
      *
      * It first searches for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified resourcePointId, fieldAccessPointId, machineId and machineBunkerState (specific to this constellation), but there exists an entry for the specified resourcePointId and fieldAccessPointId, and for 'AllMachines' and 'ALL_MACHINE_STATES' it returns the data that applies for all machines and all bunker states for those resource and field access points
      * @param resourcePointId Id of the resource/silo point
      * @param fieldAccessPointId Id of the field access point
      * @param machineId Id of the machine
      * @param [out] travelCosts Travel costs specific to the given inputs
      * @return True on success (i.e. if travel data exists for the specified inputs)
      */
    bool getTravelCost_RP2FAP(ResourcePointId_t resourcePointId,
                              FieldAccessPointId_t fieldAccessPointId,
                              MachineId_t machineId,
                              MapMachineStateTravelCosts_t& travelCosts) const;

    /**
      * @brief Get travel costs/data (machine-based) map for travels from a specified resource/silo point to a specified field access point
      *
      * It first searches for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified resourcePointId and fieldAccessPointId (specific to this constellation), but there exists an entry for the specified resourcePointId and for 'AllFieldAccessPoints', it returns the data that applies for travels from the specified resource point to all field access points
      * @param resourcePointId Id of the resource/silo point
      * @param fieldAccessPointId Id of the field access point
      * @param [out] travelCostsMap Travel costs map specific to the given inputs (the map hold the travel costs for specific machines and bunker states)
      * @return True on success (i.e. if travel data exists for the specified inputs)
      */
    bool getTravelCost_RP2FAP(ResourcePointId_t resourcePointId,
                              FieldAccessPointId_t fieldAccessPointId,
                              MapMachineTravelCosts_t& travelCostsMap) const;

    /**
      * @brief Get travel costs/data for travels from a specified field access point to a specified field access point for the specified machine and bunker state
      *
      * It first searches for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified fap_id_from, fap_id_to, machineId and machineBunkerState (specific to this constellation), but there exists an entry for the specified fap_id_from and fap_id_to, and for 'AllMachines' and 'ALL_MACHINE_STATES' it returns the data that applies for all machines and all bunker states for those field access points
      * @param fap_id_from Id of the 'from' field access point
      * @param fap_id_to Id of the 'to' field access point
      * @param machineId Id of the machine
      * @param machineBunkerState Machine bunker state
      * @param [out] travelCosts Travel costs specific to the given inputs
      * @return True on success (i.e. if travel data exists for the specified inputs)
      */
    bool getTravelCost_FAP2FAP(FieldAccessPointId_t fap_id_from,
                               FieldAccessPointId_t fap_id_to,
                               MachineId_t machineId,
                               MachineBunkerState machineBunkerState,
                               TravelCosts& travelCosts) const;

    /**
      * @brief Get travel costs/data for travels from a specified field access point to a specified field access point for the specified machine
      *
      * It first searches for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified fap_id_from, fap_id_to, machineId and machineBunkerState (specific to this constellation), but there exists an entry for the specified fap_id_from and fap_id_to, and for 'AllMachines' and 'ALL_MACHINE_STATES' it returns the data that applies for all machines and all bunker states for those field access points
      * @param fap_id_from Id of the 'from' field access point
      * @param fap_id_to Id of the 'to' field access point
      * @param machineId Id of the machine
      * @param [out] travelCosts Travel costs specific to the given inputs
      * @return True on success (i.e. if travel data exists for the specified inputs)
      */
    bool getTravelCost_FAP2FAP(FieldAccessPointId_t fap_id_from,
                               FieldAccessPointId_t fap_id_to,
                               MachineId_t machineId,
                               MapMachineStateTravelCosts_t& travelCosts) const;

    /**
      * @brief Get travel costs/data (machine-based) map for travels from a specified field access point to a specified field access point
      *
      * It first searches for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified fap_id_from, fap_id_to, machineId and machineBunkerState (specific to this constellation), but there exists an entry for the specified fap_id_from and fap_id_to, and for 'AllMachines' and 'ALL_MACHINE_STATES' it returns the data that applies for all machines and all bunker states for those field access points
      * @param fap_id_from Id of the 'from' field access point
      * @param fap_id_to Id of the 'to' field access point
      * @param [out] travelCostsMap Travel costs map specific to the given inputs (the map hold the travel costs for specific machines and bunker states)
      * @return True on success (i.e. if travel data exists for the specified inputs)
      */
    bool getTravelCost_FAP2FAP(ResourcePointId_t fap_id_from,
                               FieldAccessPointId_t fap_id_to,
                               MapMachineTravelCosts_t& travelCostsMap) const;

    /**
      * @brief Get unloading costs/data for a specified resource/silo point for the specified machine
      *
      * It first searches for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified resourcePointId and machineId (specific to this constellation), but there exists an entry for the specified resourcePointId and 'AllMachines', it returns the data that applies for all machines for that resource point
      * @param resourcePointId Id of the resource/silo point
      * @param machineId Id of the machine
      * @param [out] unloadingCosts Unloading costs specific to the given inputs
      * @return True on success (i.e. if unloading data exists for the specified inputs)
      */
    bool getUnloadingCosts(ResourcePointId_t resourcePointId,
                           MachineId_t machineId,
                           UnloadingCosts& unloadingCosts) const;

    /**
      * @brief Get unloading costs/data (machine-based) map for a specified resource/silo point
      *
      * It first searches for data specific to the given reource point. If it doesn't exist, it searches data for 'AllResourcePoints'
      * @param resourcePointId Id of the resource/silo point
      * @param [out] unloadingCostsMap Unloading costs map, holding the data for each machine
      * @return True on success (i.e. if unloading data exists for the specified resource/silo point)
      */
    bool getUnloadingCosts(const ResourcePointId_t & resourcePointId,
                           std::map< MachineId_t , UnloadingCosts >& unloadingCostsMap) const;

    /**
      * @brief Get arrival-travel costs/data for travels from the initial location of a specified machine to a specified field access point, for the specified machine bunker state
      *
      * It first searches for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified fieldAccessPointId, machineId and machineBunkerState (specific to this constellation), but there exists an entry for the specified fieldAccessPointId and machineId, and for 'ALL_MACHINE_STATES' it returns the data that applies for all machines bunker states for that field access points and that machine
      * @param fieldAccessPointId Id of the field access point
      * @param machineId Id of the machine
      * @param machineBunkerState Machine bunker state
      * @param [out] arrivalCosts Arrival-travel costs specific to the given inputs
      * @return True on success (i.e. if arrival-travel data exists for the specified inputs)
      */
    bool getArrivalCost(FieldAccessPointId_t fieldAccessPointId,
                        MachineId_t machineId,
                        MachineBunkerState machineBunkerState,
                        TravelCosts& arrivalCosts) const;

    /**
      * @brief Get arrival-travel costs/data for travels from the initial location of a specified machine to a specified field access point
      *
      * It first searches for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified fieldAccessPointId, machineId and machineBunkerState (specific to this constellation), but there exists an entry for the specified fieldAccessPointId and machineId, and for 'ALL_MACHINE_STATES' it returns the data that applies for all machines bunker states for that field access points and that machine
      * @param fieldAccessPointId Id of the field access point
      * @param machineId Id of the machine
      * @param [out] arrivalCosts Arrival-travel costs for all bunker states
      * @return True on success (i.e. if arrival-travel data exists for the specified inputs)
      */
    bool getArrivalCost(FieldAccessPointId_t fieldAccessPointId,
                        MachineId_t machineId,
                        MapMachineStateTravelCosts_t& arrivalCosts) const;

    /**
      * @brief Get arrival-travel costs/data (machine-based) map for travels from the initial location of a specified machine to a specified field access point, for the specified machine bunker state
      *
      * It first searches for data specific to the given field access point. If it doesn't exist, it searches data for 'AllFieldAccessPoints'
      * @param fieldAccessPointId Id of the field access point
      * @param [out] arrivalCosts Arrival-travel costs specific to the given field access point, holding the data for each machine
      * @return True on success (i.e. if arrival-travel data exists for the specified inputs)
      */
    bool getArrivalCost(FieldAccessPointId_t fieldAccessPointId,
                        MapMachineTravelCosts_t& arrivalCostsMap) const;

    /**
      * @brief Read the travel costs/data for a specified machine and bunker state from a given costs' map
      *
      * It first searches for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified machineId and machineBunkerState (specific to this constellation), but there exists an entry for the specified machineId and for 'ALL_MACHINE_STATES', it returns the data that applies to all machines bunker states
      * @param travelCostsMap Travel costs' map to read from
      * @param machineId Id of the machine
      * @param machineBunkerState Machine bunker state
      * @param [out] travelCosts Travel costs specific to the given inputs
      * @return True on success (i.e. if travel data exists for the specified inputs)
      */
    static bool getTravelCost(const MapMachineTravelCosts_t &travelCostsMap,
                              MachineId_t machineId,
                              MachineBunkerState machineBunkerState,
                              TravelCosts& travelCosts);

    /**
      * @brief Read the travel costs/data for a specified machine from a given costs' map
      *
      * It first searches for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified machineId and machineBunkerState (specific to this constellation), but there exists an entry for the specified machineId and for 'ALL_MACHINE_STATES', it returns the data that applies to all machines bunker states
      * @param travelCostsMap Travel costs' map to read from
      * @param machineId Id of the machine
      * @param [out] travelCosts Travel costs specific to the given inputs
      * @return True on success (i.e. if travel data exists for the specified inputs)
      */
    static bool getTravelCost(const MapMachineTravelCosts_t &travelCostsMap,
                              MachineId_t machineId,
                              MapMachineStateTravelCosts_t& travelCosts);

    /**
      * @brief Read the travel costs/data for a specified bunker state from a given costs' map
      *
      * It first searches for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified machineId and machineBunkerState (specific to this constellation), but there exists an entry for the specified machineId and for 'ALL_MACHINE_STATES', it returns the data that applies to all machines bunker states
      * @param travelCostsMap Travel costs' map to read from
      * @param machineBunkerState Machine bunker state
      * @param [out] travelCosts Travel costs specific to the given inputs
      * @return True on success (i.e. if travel data exists for the specified inputs)
      */
    static bool getTravelCost(const MapMachineStateTravelCosts_t &travelCostsMap,
                              MachineBunkerState machineBunkerState,
                              TravelCosts& travelCosts);

    /**
      * @brief Read the unloading costs/data for a specified machine from a given costs' map
      *
      * It first searches for data specific to the specified machine, and if no data is found, searches data for 'AllMachines':
      * @param unloadingCostsMap Unloading costs' map to read from
      * @param machineId Id of the machine
      * @param [out] unloadingCosts Unloading costs specific to the given inputs
      * @return True on success (i.e. if unloading data exists for the specified inputs)
      */
    static bool getUnloadingCosts(const std::map< MachineId_t , UnloadingCosts >& unloadingCostsMap,
                                  MachineId_t machineId,
                                  UnloadingCosts& unloadingCosts);

    /**
      * @brief Get unloading costs/data (machine-based) map for a specified resource/silo point
      *
      * It first searches for data specific to the given reource point. If it doesn't exist, it searches data for 'AllResourcePoints'
      * @param unloadingCostsMap_all Unloading costs' map to read from
      * @param resourcePointId Id of the resource/silo point
      * @param [out] unloadingCostsMap_rp Unloading costs map, holding the data for each machine
      * @return True on success (i.e. if unloading data exists for the specified resource/silo point)
      */
    static bool getUnloadingCosts(const MapUnloadingCosts_t &unloadingCostsMap_all,
                                  const ResourcePointId_t & resourcePointId,
                                  std::map< MachineId_t , UnloadingCosts >& unloadingCostsMap_rp);

    /**
      * @brief Read the arrival-travel costs/data for a specified machine and bunker state from a given costs' map
      *
      * It first search for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified machineId and machineBunkerState (specific to this constellation), but there exists an entry for the specified machineId and for 'ALL_MACHINE_STATES', it returns the data that applies to all machines bunker states
      * @param arrivalCostsMap Arrival-travel costs' map to read from
      * @param machineId Id of the machine
      * @param machineBunkerState Machine bunker state
      * @param [out] arrivalCosts Arrival-travel costs specific to the given inputs
      * @return True on success (i.e. if arrival-travel data exists for the specified inputs)
      */
    static bool getArrivalCost(const MapMachineTravelCosts_t &arrivalCostsMap,
                               MachineId_t machineId,
                               MachineBunkerState machineBunkerState,
                               TravelCosts& arrivalCosts);

    /**
      * @brief Read the arrival-travel costs/data for a specified machine and bunker state from a given costs' map
      *
      * It first search for data specific for each one of the input parameters, and if no data is found, searches for general data:
      *     e.g. if there isn't data for the specified machineId and machineBunkerState (specific to this constellation), but there exists an entry for the specified machineId and for 'ALL_MACHINE_STATES', it returns the data that applies to all machines bunker states
      * @param arrivalCostsMap Arrival-travel costs' map to read from
      * @param machineId Id of the machine
      * @param [out] arrivalCosts Arrival-travel costs specific to the given inputs
      * @return True on success (i.e. if arrival-travel data exists for the specified inputs)
      */
    static bool getArrivalCost(const MapMachineTravelCosts_t &arrivalCostsMap,
                               MachineId_t machineId,
                               MapMachineStateTravelCosts_t& arrivalCosts);

    /**
      * @brief Add default unloading costs for a given resource/silo point (i.e. costs that apply for all machines)
      * @param resourcePointId Id of the resource/silo point (if = AllResourcePoints, it will apply for all resource/silo points)
      * @param unloadingCosts Default unloading costs to be set
      * @param overwrite If there exist unloading costs for the given resource point, they will be replaced iif overwrite = true
      * @return True on success
      */
    bool addDefaultUnloadingCosts(const ResourcePointId_t & resourcePointId,
                                  const UnloadingCosts& unloadingCosts,
                                  bool overwrite = false);


    /**
      * @brief Add default arrival-travel costs for a given field access point (i.e. costs that apply for all machines and all bunker states)
      * @param fieldAccessPointId Id of the field access point (if = AllFieldAccessPoints, it will apply for all field access points)
      * @param arrivalCosts Default arrival-travel costs to be set
      * @param overwrite If there exist arrival costs for the given access point, they will be replaced iif overwrite = true
      * @return True on success
      */
    bool addDefaultArrivalCosts(const FieldAccessPointId_t & fieldAccessPointId,
                                const TravelCosts& arrivalCosts,
                                bool overwrite = false);


    /**
      * @brief Get all travel costs from field access points to resource/silo points as a vector
      * @return All travel costs from field access points to resource/silo points
      */
    std::vector<TravelData> getAllAccessPoint2ResourcePointData() const;

    /**
      * @brief Get all travel costs from resource/silo points to field access points as a vector
      * @return All travel costs from resource/silo points to field access points
      */
    std::vector<TravelData> getAllResourcePoint2AccessPointData() const;

    /**
      * @brief Get all travel costs vetween field access points as a vector
      * @return All travel costs between field access points
      */
    std::vector<TravelData2> getAllAccessPoint2AccessPointData() const;

    /**
      * @brief Get all unloading costs as a vector
      * @return All unloading costs
      */
    std::vector<UnloadingData> getAllUnloadingData() const;

    /**
      * @brief Get all arrival-travel costs from initial machine locations to field access points
      * @return All arrival-travel costs from initial machine locations to field access points
      */
    std::vector<ArrivalData> getAllArrivalData() const;

    /**
      * @brief Get a reference to the map holding the travel costs/info from field access points to resource/silo points (for specific machines and bunker states)
      * @return Reference to the map holding the travel costs/info from field access points to resource/silo points (for specific machines and bunker states)
      */
    const MapAccessPoint2ResourcePoint_t& mapAccessPoint2ResourcePoint() const{ return m_mapAccessPoint2ResourcePoint; }

    /**
      * @brief Get a reference to the map holding the travel costs/info from field access points to resource/silo points (for specific machines and bunker states)
      * @return Reference to the map holding the travel costs/info from field access points to resource/silo points (for specific machines and bunker states)
      */
    MapAccessPoint2ResourcePoint_t& mapAccessPoint2ResourcePoint() { return m_mapAccessPoint2ResourcePoint; }

    /**
      * @brief Get a reference to the map holding the travel costs/info from resource/silo points to field access points (for specific machines and bunker states)
      * @return Reference to the map holding the travel costs/info from resource/silo points to field access points (for specific machines and bunker states)
      */
    const MapResourcePoint2AccessPoint_t& mapResourcePoint2AccessPoint() const{ return m_mapResourcePoint2AccessPoint; }

    /**
      * @brief Get a reference to the map holding the travel costs/info from resource/silo points to field access points (for specific machines and bunker states)
      * @return Reference to the map holding the travel costs/info from resource/silo points to field access points (for specific machines and bunker states)
      */
    MapResourcePoint2AccessPoint_t& mapResourcePoint2AccessPoint() { return m_mapResourcePoint2AccessPoint; }

    /**
      * @brief Get a reference to the map holding the travel costs/info between field access points (for specific machines and bunker states)
      * @return Reference to the map holding the travel costs/info betweenfield access points (for specific machines and bunker states)
      */
    const MapResourcePoint2AccessPoint_t& mapAccessPoint2AccessPoint() const{ return m_mapAccessPoint2AccessPoint; }

    /**
      * @brief Get a reference to the map holding the travel costs/info between field access points (for specific machines and bunker states)
      * @return Reference to the map holding the travel costs/info between field access points (for specific machines and bunker states)
      */
    MapResourcePoint2AccessPoint_t& mapAccessPoint2AccessPoint() { return m_mapAccessPoint2AccessPoint; }

    /**
      * @brief Get a reference to the map holding the unloading costs/info from one or more resource/silo points (for specific machines)
      * @return Reference to the map holding the unloading costs/info from one or more resource/silo points (for specific machines)
      */
    const MapUnloadingCosts_t& mapUnloadingCosts() const{ return m_mapUnloadingCosts; }

    /**
      * @brief Get a reference to the map holding the unloading costs/info from one or more resource/silo points (for specific machines)
      * @return Reference to the map holding the unloading costs/info from one or more resource/silo points (for specific machines)
      */
    MapUnloadingCosts_t& mapUnloadingCosts() { return m_mapUnloadingCosts; }

    /**
      * @brief Get a reference to the map holding the arrival travel costs/info to field access points (for specific machines and bunker states)
      * @return Reference to the map holding the arrival travel costs/info to field access points (for specific machines and bunker states)
      */
    const MapArrivalCosts_t& mapArrivalCosts() const{ return m_mapArrivalCosts; }

    /**
      * @brief Get a reference to the map holding the arrival travel costs/info to field access points (for specific machines and bunker states)
      * @return Reference to the map holding the arrival travel costs/info to field access points (for specific machines and bunker states)
      */
    MapArrivalCosts_t& mapArrivalCosts() { return m_mapArrivalCosts; }

protected:
    MapAccessPoint2ResourcePoint_t m_mapAccessPoint2ResourcePoint; /**< Map holding the travel costs/info from field access points to resource/silo points (for specific machines and bunker states) */
    MapResourcePoint2AccessPoint_t m_mapResourcePoint2AccessPoint; /**< Map holding the travel costs/info from resource/silo points to field access points (for specific machines and bunker states) */
    MapAccessPoint2AccessPoint_t m_mapAccessPoint2AccessPoint; /**< Map holding the travel costs/info between field access points (for specific machines and bunker states) */
    MapUnloadingCosts_t m_mapUnloadingCosts; /**< Map holding the unloading costs/info from one or more resource/silo points (for specific machines) */
    MapArrivalCosts_t m_mapArrivalCosts; /**< Map holding the arrival travel costs/info to field access points (for specific machines and bunker states) */

};

}


#endif //_AROLIB_OUTFIELDINFO_H_
