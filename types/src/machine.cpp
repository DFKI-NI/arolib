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
 
#include "arolib/types/machine.hpp"
#include <cmath>
#include "arolib/types/units.hpp"

namespace arolib {
const MachineId_t Machine::AllMachineIds = std::numeric_limits<MachineId_t>::max();
const std::set<Machine::MachineType> Machine::AllMachineTypes = { Machine::HARVESTER,
                                                                  Machine::OLV,
                                                                  Machine::SOWER,
                                                                  Machine::SPRAYER,
                                                                  Machine::CULTIVATOR,
                                                                  Machine::PLOUGH,
                                                                  Machine::SCANNER,
                                                                  Machine::UNDEFINED_TYPE };
const std::set<Machine::MachineType> Machine::WorkingMachineTypes = { Machine::HARVESTER,
                                                                      Machine::SOWER,
                                                                      Machine::SPRAYER,
                                                                      Machine::CULTIVATOR,
                                                                      Machine::PLOUGH };
const std::set<Machine::MachineType> Machine::NonWorkingMachineTypes = { Machine::OLV,
                                                                         Machine::SCANNER,
                                                                         Machine::UNDEFINED_TYPE };

const unsigned char Machine::WorkingSide_RIGHT = 1;
const unsigned char Machine::WorkingSide_LEFT = 2;
const unsigned char Machine::WorkingSide_BACK = 4;
const unsigned char Machine::WorkingSide_FRONT = 8;
const unsigned char Machine::MaxUnloadSides = 0x0F;
const unsigned char Machine::DefaultDownloadSides = WorkingSide_RIGHT | WorkingSide_LEFT | WorkingSide_BACK;

Machine::MachineType Machine::intToMachineType(int value){
    if(value == MachineType::HARVESTER)
        return MachineType::HARVESTER;
    else if(value == MachineType::OLV)
        return MachineType::OLV;
    else if(value == MachineType::SOWER)
        return MachineType::SOWER;
    else if(value == MachineType::SPRAYER)
        return MachineType::SPRAYER;
    else if(value == MachineType::CULTIVATOR)
        return MachineType::CULTIVATOR;
    else if(value == MachineType::PLOUGH)
        return MachineType::PLOUGH;
    else if(value == MachineType::SCANNER)
        return MachineType::SCANNER;
    else if(value == MachineType::UNDEFINED_TYPE)
        return MachineType::UNDEFINED_TYPE;

    throw std::invalid_argument( "The given value does not correspond to any Machine::MachineType" );
}

Machine::MachineAssignment Machine::intToMachineAssignment(int value){
    if(value == MachineAssignment::HEADLAND_INFIELD)
        return MachineAssignment::HEADLAND_INFIELD;
    else if(value == MachineAssignment::HEADLAND)
        return MachineAssignment::HEADLAND;
    else if(value == MachineAssignment::INFIELD)
        return MachineAssignment::INFIELD;
    throw std::invalid_argument( "The given value does not correspond to any Machine::MachineAssignment" );
}

bool Machine::operator <(const Machine &m) const
{
    return id < m.id;
}

std::string Machine::toString() const {
    return "Machine (" + manufacturer + ": " + model
            + "; id: " + std::to_string(id)
            + "; type: " + machineTypeToString(machinetype)
            + "; speed: " + std::to_string(def_working_speed) + " m/s (max " + std::to_string(max_speed_empty) + " m/s , " + std::to_string(max_speed_full) + " m/s)"
            + ", bunker mass: " + std::to_string(bunker_mass) + ")";
}

double Machine::calcSpeed(double currentMass) const{
    if(bunker_mass == 0)
        return max_speed_empty;
    return max_speed_empty + std::min( 1.0, std::max( 0.0, currentMass/bunker_mass) )
            * (max_speed_full - max_speed_empty);
}

double Machine::workingRadius() const{
    double r = 0;
    r = std::max(r, working_width);
    r = std::max(r, width);
    r = std::max(r, length);
    if(r < 1e-3)
        return -1;
    return r * 0.5;
}

double Machine::getTurningRadius() const
{
    if(turning_radius > -1e-9)
        return std::max(0.0, turning_radius);
    double d;
    if(axis_distance > 1e-9)
        d = axis_distance;
    else if(length > 1e-9){
        const double deltaAxisDist = 0.5;//estimeted location of the front/back axis from the machine front/back
        d = length > 2*deltaAxisDist ? length - 2*deltaAxisDist : length;
    }
    else
        return workingRadius();

    const double steeringAng = deg2rad(55);//max steering angle in rad

    double b = d * sin(M_PI_2 - steeringAng) / sin(steeringAng);
    double w = 0.5 * std::max( 0.0, width > 0 ? width : working_width );
    return sqrt( (b+w)*(b+w) + 0.25*d*d );

}

bool Machine::isOfWorkingType(bool includeScanner) const
{
    return isOfWorkingType(machinetype, includeScanner);
}

bool Machine::isOfWorkingType(Machine::MachineType machinetype, bool includeScanner)
{
    return machinetype != OLV
            && machinetype != UNDEFINED_TYPE
            && ( includeScanner || machinetype != SCANNER ); //@TODO add new working types
}

bool Machine::isOfWorkingTypeForMaterialOutput() const
{
    return isOfWorkingTypeForMaterialOutput(machinetype);
}

bool Machine::isOfWorkingTypeForMaterialOutput(MachineType machinetype)
{
    return machinetype == HARVESTER;
}

bool Machine::isOfWorkingTypeForMaterialInput() const
{
    return isOfWorkingTypeForMaterialInput(machinetype);
}

bool Machine::isOfWorkingTypeForMaterialInput(MachineType machinetype)
{
    return machinetype == SOWER ||
            machinetype == SPRAYER;
}

bool Machine::isOfWorkingTypeForNoMaterialFlow(bool includeScanner) const
{
    return isOfWorkingTypeForNoMaterialFlow(machinetype);
}

bool Machine::isOfWorkingTypeForNoMaterialFlow(MachineType machinetype, bool includeScanner)
{
    return machinetype == CULTIVATOR ||
            machinetype == PLOUGH;
}

std::string Machine::machineTypeToString(Machine::MachineType type)
{
    if(type == Machine::HARVESTER)
        return "HARVESTER";
    if(type == Machine::OLV)
        return "OLV";
    if(type == Machine::SOWER)
        return "SEEDER";
    if(type == Machine::SPRAYER)
        return "SPRAYER";
    if(type == Machine::CULTIVATOR)
        return "CULTIVATOR";
    if(type == Machine::PLOUGH)
        return "PLOUGH";
    if(type == Machine::SCANNER)
        return "SCANNER";
    return "---";

}

std::string Machine::machineTypeToShortString3c(Machine::MachineType type)
{
    if(type == Machine::HARVESTER)
        return "HRV";
    if(type == Machine::OLV)
        return "OLV";
    if(type == Machine::SOWER)
        return "SWR";
    if(type == Machine::SPRAYER)
        return "SPY";
    if(type == Machine::CULTIVATOR)
        return "CUL";
    if(type == Machine::PLOUGH)
        return "PLG";
    if(type == Machine::SCANNER)
        return "SCN";
    return "---";
}

std::set<Machine::MachineType> Machine::getWorkingTypes(bool includeScanner)
{
    auto ret = WorkingMachineTypes;
    if(includeScanner)
        ret.insert(MachineType::SCANNER);
    return ret;
}

std::set<Machine::MachineType> Machine::getNonWorkingTypes(bool includeScanner)
{
   auto ret = NonWorkingMachineTypes;
    if(!includeScanner)
        ret.erase(MachineType::SCANNER);
    return ret;
}

std::map<MachineId_t, Machine> Machine::toMachineIdMap(const std::vector<Machine> &machines)
{
    std::map<MachineId_t, Machine> ret;
    for( auto &m : machines)
        ret[m.id] = m;
    return ret;
}


}
