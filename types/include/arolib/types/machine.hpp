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
 
#ifndef _AROLIB_MACHINE_H_
#define _AROLIB_MACHINE_H_

#include <limits>
#include <vector>
#include <set>
#include <map>

namespace arolib {

typedef int MachineId_t;

/**
  * @brief Machine class
  */
class Machine{
public:

    static const MachineId_t AllMachineIds; /**< Used to refer to all machine ids. Must NOT be assigned to any machine */

    static const unsigned char WorkingSide_RIGHT; /**< The machine can 'work' (e.g. download) to the right */
    static const unsigned char WorkingSide_LEFT; /**< The machine can 'work' (e.g. download) to the left */
    static const unsigned char WorkingSide_BACK; /**< The machine can 'work' (e.g. download) to the back */
    static const unsigned char WorkingSide_FRONT; /**< The machine can 'work' (e.g. download) to the front */
    static const unsigned char MaxUnloadSides; /**< Hold the maximum value that the unload_sides can have */
    static const unsigned char DefaultDownloadSides; /**< Default downloading sides for harvesters */

    /**
      * @brief Machine type
      */
    enum MachineType{
        HARVESTER = 0, /**< Harvesting machine */
        OLV, /**< Overload/transport vehicle */
        SOWER, /**< Seeding machine */
        SPRAYER, /**< Spraying machine */
        CULTIVATOR, /**< Cultivating machine */
        PLOUGH, /**< Plough machine */
        SCANNER = 20, /**< Field scanning machine */
        UNDEFINED_TYPE /**< Undefined (not supported at the moment) */
    };
    /**
      * @brief Get the MachineType (enum) from its int value
      */
    static MachineType intToMachineType(int value);
    // static inline std::string MachineTypeToString(MachineType type)
    // {
    //   switch(type)
    //   {
    //     case MachineType::HARVESTER : return "Harvester";
    //     case MachineType::OLV : return "OLV";
    //     case MachineType::SOWER : return "Sower";
    //     case MachineType::SPRAYER : return "Sprayer";
    //     case MachineType::CULTIVATOR : return "Cultivator";
    //     case MachineType::PLOUGH : return "Plough";
    //     case MachineType::SCANNER : return "Scanner";
    //     case MachineType::UNDEFINED_TYPE : return "Undefined";
    //     default: return "Unknown";
    //   }
    // }

    static const std::set<MachineType> AllMachineTypes; /**< Set containing all machine types */
    static const std::set<MachineType> WorkingMachineTypes; /**< Set containing all working (primary) machine types */
    static const std::set<MachineType> NonWorkingMachineTypes; /**< Set containing all non-working machine types */


    /**
      * @brief Assignemet of the machine
      */
    enum MachineAssignment{
        HEADLAND_INFIELD = 0, /**< The machine is assigned to work in harvesting of both headland and inner-field */
        HEADLAND, /**< The machine is assigned to work ONLY in headland harvesting */
        INFIELD /**< The machine is assigned to work ONLY in inner-field harvesting */
    };
    /**
      * @brief Get the MachineAssignment (enum) from its int value
      */
    static MachineAssignment intToMachineAssignment(int value);


    /**
      * @brief Constructor
      */
    explicit Machine() = default;

    /**
      * @brief operator < based on the machine id
      * TODO: Describe what this does
      */
    bool operator <(const Machine &m ) const;

    /**
      * @brief Get the machine properties as a string
      */
    std::string toString() const;


    /**
      * @brief Compute the machine speed based on the current bunker mass, the machine's bunker (mass) capacity, and the machine's speed limits
      * @param currentMass Current mass in the bunker [kg]
      * @return currentMass Computed speed [m/s]
      */
    double calcSpeed(double currentMass) const;

    /**
      * @brief Compute the pseudo- working radius of the machine
      * @return pseudo- working radius of the machine [m]
      */
    double workingRadius() const;

    /**
      * @brief Estimate the turning radius of the vehicle
      * @return Estimated turning radius of the machine [m]
      */
    double getTurningRadius() const;

    /**
      * @brief Check if the machine is a 'working field' machine (e.g. harvester, seeder, sprayer)
      * @return includeScanner Include SCANNER as working type
      * @return True if the machine is a 'working field' machine
      */
    bool isOfWorkingType(bool includeScanner = false) const;

    /**
      * @brief Check if the machine is a 'working field' machine (e.g. harvester, seeder, sprayer)
      * @return machinetype Machine type
      * @return includeScanner Include SCANNER as working type
      * @return True if the machine is a 'working field' machine
      */
    static bool isOfWorkingType(MachineType machinetype, bool includeScanner = false);

    /**
      * @brief Return the machine type as a string description
      * @param type Machine type
      * @return Machine type as string
      */
    static std::string machineTypeToString(MachineType type);

    /**
      * @brief Return the machine type as a 3-characters-string description
      * @param type Machine type
      * @return Machine type as string
      */
    static std::string machineTypeToShortString3c(MachineType type);

    /**
      * @brief Get the set of machine working (primary) types
      * @param includeScanner Include SCANNER as working type
      * @return Machine type as string
      */
    static std::set<MachineType> getWorkingTypes(bool includeScanner = false);

    /**
      * @brief Get the set of machine non-working (non-primary) types
      * @param includeScanner Include SCANNER as working type
      * @return Machine type as string
      */
    static std::set<MachineType> getNonWorkingTypes(bool includeScanner = true);

    /**
      * @brief Convert a vector of machines into a map with machine_id as key
      * @param machines Vector of machines
      * @return Machine's map
      */
    static std::map<MachineId_t, Machine> toMachineIdMap(const std::vector<Machine>& machines);

public:

    MachineId_t id = -99999; /**< Machine's unique id */

    int id_intern = -99999; /**< Machine's unique id in the database (for SoilAssist, almost deprecated). */
    std::string manufacturer = "_UNDEF_"; /**< Machine's manufacturer */
    std::string model = "_UNDEF_"; /**< Machine's model */
    double width = -99999; /**< Machine's width [m] */
    double length = -99999; /**< Machine's length [m] */
    double height = -99999; /**< Machine's height [m] */
    double weight = -99999; /**< Machine's weight [kg] (with empty bunker) */
    double bunker_mass = -99999; /**< Machine's bunker mass/weight capacity [kg] */
    double bunker_volume = -99999; /**< Machine's bunker volume capacity [m³] @todo not being used yet */
    double working_width = -99999; /**< Machine's working width [m] (normally the one taken for computations) */
    double max_speed_empty = -99999; /**< Machine's speed limit when the bunker is empty [m/s] */
    double max_speed_full = -99999; /**< Machine's speed limit when the bunker is full [m/s] */
    double def_working_speed = -99999; /**< Machine's default working speed [m/s] */
    int num_axis = -1; /**< number of (wheel) axis */
    double turning_radius = -99999; /**< Turning radius [m] */
    double axis_distance = -99999; /**< Distance between the machine's axis [m] */
    double gauge = -99999; /**< Machine's gauge */
    double engine_power = -99999; /**< Machine's engine power */
    int knifes_count = -99999; /**< Machine's knifes count (harvesters) */
    char unload_sides = DefaultDownloadSides; /**< To what side can the machine download (harvesters) */

    MachineType machinetype = UNDEFINED_TYPE; /**< Machine's type */
    MachineAssignment machineassignment = MachineAssignment::HEADLAND_INFIELD; /**< Machine's assignement (headland and/or inner-field harvesting) */

};


/**
  * @brief operator<< to add (print) the most important properties of a machine to an output stream
  * @param ostr Output stream
  * @param m Machine to be added/printed
  * @return Updated output stream
  */
inline std::ostream& operator<< (std::ostream &ostr, const Machine& m) {
    ostr << m.toString();
    return ostr;
}

}

#endif //_AROLIB_MACHINE_H_
