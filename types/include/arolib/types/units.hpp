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
 
#ifndef _AROLIB_UNITS_H_
#define _AROLIB_UNITS_H_

#include <string>
#include <stdexcept>
#include <cmath>

namespace arolib {

/**
  * @brief Supported units
  */
enum Unit {
    UNIT_CUSTOM,
    UNIT_SECOND,
    UNIT_HOUR,
    UNIT_METERS,
    UNIT_KM,
    UNIT_METER2,
    UNIT_HECTARE,
    UNIT_METER3,
    UNIT_LITER,
    UNIT_METERS_PER_SECOND,
    UNIT_KM_PER_HOUR,
    UNIT_KG,
    UNIT_TONS,
    UNIT_KG_PER_METER2,
    UNIT_TONS_PER_HECTARE,
    UNIT_KG_PER_SECOND,
    UNIT_METER3_PER_METER2,
    UNIT_METER3_PER_HECTARE,
    UNIT_LITERS_PER_METER2,
    UNIT_LITERS_PER_HECTARE,
    UNIT_DEGREE,
    UNIT_RADIAN,
    UNIT_COUNT
};

/**
  * @brief Get the unique string (description) from a unit
  * @return String (description)
  */
std::string unitToString(Unit u);

/**
  * @brief Get the unit corresponding to a string (description)
  * @return Unit
  */
Unit stringToUnit(std::string value);

/**
  * @brief Get the unit corresponding to an int
  * @return Unit
  */
Unit intToUnit(int value);

/**
  * @brief Check if two units correspond to the same type (e.g distance, speed, mass/area, ...)
  * @param u1 Unit 1
  * @param u1 Unit 2
  * @return True if the units correspond to the same type
  */
bool areSameUnitTypes(Unit u1, Unit u2);

/**
  * @brief Check if a unit is of type 'time'
  * @return True if unit is of the type.
  */
bool isTimeUnit(Unit u);

/**
  * @brief Check if a unit is of type 'distance'
  * @return True if unit is of the type.
  */
bool isDistanceUnit(Unit u);

/**
  * @brief Check if a unit is of type 'area'
  * @return True if unit is of the type.
  */
bool isAreaUnit(Unit u);

/**
  * @brief Check if a unit is of type 'volume'
  * @return True if unit is of the type.
  */
bool isVolumeUnit(Unit u);

/**
  * @brief Check if a unit is of type 'speed'
  * @return True if unit is of the type.
  */
bool isSpeedUnit(Unit u);

/**
  * @brief Check if a unit is of type 'mass'
  * @return True if unit is of the type.
  */
bool isMassUnit(Unit u);

/**
  * @brief Check if a unit is of type 'mass/area'
  * @return True if unit is of the type.
  */
bool isMassPerAreaUnit(Unit u);

/**
  * @brief Check if a unit is of type 'mass/time'
  * @return True if unit is of the type.
  */
bool isMassPerTimeUnit(Unit u);

/**
  * @brief Check if a unit is of type 'volume/area'
  * @return True if unit is of the type.
  */
bool isVolumePerAreaUnit(Unit u);

/**
  * @brief Check if a unit is of type 'angle'
  * @return True if unit is of the type.
  */
bool isAngleUnit(Unit u);

/**
  * @brief Check if a unit is of type 'count'
  * @return True if unit is of the type.
  */
bool isCountUnit(Unit u);

/**
  * @brief Convert a value from one unit to another (
  *
  * The units must be of the same type
  * @param val Value to convert
  * @param from Units of the input value
  * @param to Target units
  * @return Converted value.
  */
double convertUnits(double val, Unit from, Unit to);

/**
  * @brief Convert a value from one unit to another (
  *
  * The units must be of the same type
  * @param val_in Value to convert (input value)
  * @param val_out [out] Converted value
  * @param from Units of the input value
  * @param to Target units
  * @return True on success.
  */
bool convertUnits(double val_in, double& val_out, Unit from, Unit to);

/**
  * @brief Convert m² to hectares
  * @param Input value
  * @return Converted value.
  */
inline double sqrmeters2hectares(const double& value) {return value * 0.0001;}

/**
  * @brief Convert hectares to m²
  * @param Input value
  * @return Converted value.
  */
inline double hectares2sqrmeters(const double& value) {return value * 10000.0;}

/**
  * @brief Convert tonnes to Kg
  * @param Input value
  * @return Converted value.
  */
inline double tonnes2Kg(const double& value) {return value * 1000;}

/**
  * @brief Convert Kg to tonnes
  * @param Input value
  * @return Converted value.
  */
inline double Kg2tonnes(const double& value) {return value * 0.001;}

/**
  * @brief Convert tonnes/hectare to Kg/m²
  * @param Input value
  * @return Converted value.
  */
inline double t_ha2Kg_sqrm(const double& value) {return ( tonnes2Kg(value) * sqrmeters2hectares(1.0) );}

/**
  * @brief Convert Kg/m² to tonnes/hectare
  * @param Input value
  * @return Converted value.
  */
inline double Kg_sqrm2t_ha(const double& value) {return ( Kg2tonnes(value) * hectares2sqrmeters(1.0) );}

/**
  * @brief Convert m³ to liters
  * @param Input value
  * @return Converted value.
  */
inline double cubicmeters2liters(const double& value) {return value * 1000;}

/**
  * @brief Convert liters to m³
  * @param Input value
  * @return Converted value.
  */
inline double liters2cubicmeters(const double& value) {return value * 0.001;}

/**
  * @brief Convert m/s to km/h
  * @param Input value
  * @return Converted value.
  */
inline double meters_s2Km_hour(const double& value) {return value * 3.6;}

/**
  * @brief Convert km/h to m/s
  * @param Input value
  * @return Converted value.
  */
inline double Km_hour2meters_s(const double& value) {return value / 3.6;}

/**
  * @brief Convert degrees to radians
  * @param Input value
  * @return Converted value.
  */
inline double deg2rad(const double& value) {return value * M_PI / 180.0;}

/**
  * @brief Convert radians to degrees
  * @param Input value
  * @return Converted value.
  */
inline double rad2deg(const double& value) {return value * M_1_PI * 180.0;}

}

#endif //_AROLIB_UNITS_H_
