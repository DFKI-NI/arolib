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
 
#include "arolib/types/units.hpp"

namespace arolib {

std::string unitToString(Unit u)
{
    if(u == UNIT_CUSTOM)
        return "UNIT_CUSTOM";
    if(u == UNIT_SECOND)
        return "UNIT_SECOND";
    if(u == UNIT_HOUR)
        return "UNIT_HOUR";
    if(u == UNIT_METERS)
        return "UNIT_METERS";
    if(u == UNIT_KM)
        return "UNIT_KM";
    if(u == UNIT_METER2)
        return "UNIT_METER2";
    if(u == UNIT_HECTARE)
        return "UNIT_HECTARE";
    if(u == UNIT_METER3)
        return "UNIT_METER3";
    if(u == UNIT_LITER)
        return "UNIT_LITER";
    if(u == UNIT_METERS_PER_SECOND)
        return "UNIT_METERS_PER_SECOND";
    if(u == UNIT_KM_PER_HOUR)
        return "UNIT_KM_PER_HOUR";
    if(u == UNIT_KG)
        return "UNIT_KG";
    if(u == UNIT_TONS)
        return "UNIT_TONS";
    if(u == UNIT_KG_PER_METER2)
        return "UNIT_KG_PER_METER2";
    if(u == UNIT_TONS_PER_HECTARE)
        return "UNIT_TONS_PER_HECTARE";
    if(u == UNIT_KG_PER_SECOND)
        return "UNIT_KG_PER_SECOND";
    if(u == UNIT_METER3_PER_METER2)
        return "UNIT_METER3_PER_METER2";
    if(u == UNIT_METER3_PER_HECTARE)
        return "UNIT_METER3_PER_HECTARE";
    if(u == UNIT_LITERS_PER_METER2)
        return "UNIT_LITERS_PER_METER2";
    if(u == UNIT_LITERS_PER_HECTARE)
        return "UNIT_LITERS_PER_HECTARE";
    if(u == UNIT_DEGREE)
        return "UNIT_DEGREE";
    if(u == UNIT_RADIAN)
        return "UNIT_RADIAN";
    if(u == UNIT_COUNT)
        return "UNIT_COUNT";
    return "UNIT_CUSTOM";
}

Unit stringToUnit(std::string value)
{
    if(value == "UNIT_CUSTOM")
        return UNIT_CUSTOM;
    if(value == "UNIT_SECOND")
        return UNIT_SECOND;
    if(value == "UNIT_HOUR")
        return UNIT_HOUR;
    if(value == "UNIT_METERS")
        return UNIT_METERS;
    if(value == "UNIT_KM")
        return UNIT_KM;
    if(value == "UNIT_METER2")
        return UNIT_METER2;
    if(value == "UNIT_HECTARE")
        return UNIT_HECTARE;
    if(value == "UNIT_METER3")
        return UNIT_METER3;
    if(value == "UNIT_LITER")
        return UNIT_LITER;
    if(value == "UNIT_METERS_PER_SECOND")
        return UNIT_METERS_PER_SECOND;
    if(value == "UNIT_KM_PER_HOUR")
        return UNIT_KM_PER_HOUR;
    if(value == "UNIT_KG")
        return UNIT_KG;
    if(value == "UNIT_TONS")
        return UNIT_TONS;
    if(value == "UNIT_KG_PER_METER2")
        return UNIT_KG_PER_METER2;
    if(value == "UNIT_TONS_PER_HECTARE")
        return UNIT_TONS_PER_HECTARE;
    if(value == "UNIT_KG_PER_SECOND")
        return UNIT_KG_PER_SECOND;
    if(value == "UNIT_METER3_PER_METER2")
        return UNIT_METER3_PER_METER2;
    if(value == "UNIT_METER3_PER_HECTARE")
        return UNIT_METER3_PER_HECTARE;
    if(value == "UNIT_LITERS_PER_METER2")
        return UNIT_LITERS_PER_METER2;
    if(value == "UNIT_LITERS_PER_HECTARE")
        return UNIT_LITERS_PER_HECTARE;
    if(value == "UNIT_DEGREE")
        return UNIT_DEGREE;
    if(value == "UNIT_RADIAN")
        return UNIT_RADIAN;
    if(value == "UNIT_COUNT")
        return UNIT_COUNT;

    throw std::invalid_argument( "The given value does not correspond to any Unit" );
}

Unit intToUnit(int value)
{
    if(value == UNIT_CUSTOM)
        return UNIT_CUSTOM;
    if(value == UNIT_SECOND)
        return UNIT_SECOND;
    if(value == UNIT_HOUR)
        return UNIT_HOUR;
    if(value == UNIT_METERS)
        return UNIT_METERS;
    if(value == UNIT_KM)
        return UNIT_KM;
    if(value == UNIT_METER2)
        return UNIT_METER2;
    if(value == UNIT_HECTARE)
        return UNIT_HECTARE;
    if(value == UNIT_METER3)
        return UNIT_METER3;
    if(value == UNIT_LITER)
        return UNIT_LITER;
    if(value == UNIT_METERS_PER_SECOND)
        return UNIT_METERS_PER_SECOND;
    if(value == UNIT_KM_PER_HOUR)
        return UNIT_KM_PER_HOUR;
    if(value == UNIT_KG)
        return UNIT_KG;
    if(value == UNIT_TONS)
        return UNIT_TONS;
    if(value == UNIT_KG_PER_METER2)
        return UNIT_KG_PER_METER2;
    if(value == UNIT_TONS_PER_HECTARE)
        return UNIT_TONS_PER_HECTARE;
    if(value == UNIT_KG_PER_SECOND)
        return UNIT_KG_PER_SECOND;
    if(value == UNIT_METER3_PER_METER2)
        return UNIT_METER3_PER_METER2;
    if(value == UNIT_METER3_PER_HECTARE)
        return UNIT_METER3_PER_HECTARE;
    if(value == UNIT_LITERS_PER_METER2)
        return UNIT_LITERS_PER_METER2;
    if(value == UNIT_LITERS_PER_HECTARE)
        return UNIT_LITERS_PER_HECTARE;
    if(value == UNIT_DEGREE)
        return UNIT_DEGREE;
    if(value == UNIT_RADIAN)
        return UNIT_RADIAN;
    if(value == UNIT_COUNT)
        return UNIT_COUNT;

    throw std::invalid_argument( "The given value does not correspond to any Unit" );
}

bool areSameUnitTypes(Unit u1, Unit u2)
{
    return  ( u1 == u2 ) ||
            ( isTimeUnit(u1) && isTimeUnit(u2) ) ||
            ( isDistanceUnit(u1) && isDistanceUnit(u2) ) ||
            ( isAreaUnit(u1) && isAreaUnit(u2) ) ||
            ( isVolumeUnit(u1) && isVolumeUnit(u2) ) ||
            ( isSpeedUnit(u1) && isSpeedUnit(u2) ) ||
            ( isMassUnit(u1) && isMassUnit(u2) ) ||
            ( isMassPerAreaUnit(u1) && isMassPerAreaUnit(u2) ) ||
            ( isMassPerTimeUnit(u1) && isMassPerTimeUnit(u2) ) ||
            ( isVolumePerAreaUnit(u1) && isVolumePerAreaUnit(u2) ) ||
            ( isAngleUnit(u1) && isAngleUnit(u2) ) ||
            ( isCountUnit(u1) && isCountUnit(u2) );
}

bool isTimeUnit(Unit u)
{
    return u == UNIT_SECOND || u == UNIT_HOUR;
}

bool isDistanceUnit(Unit u)
{
    return u == UNIT_METERS || u == UNIT_KM;
}

bool isAreaUnit(Unit u)
{
    return u == UNIT_METER2 || u == UNIT_HECTARE;
}

bool isVolumeUnit(Unit u)
{
    return u == UNIT_METER3 || u == UNIT_LITER;

}

bool isSpeedUnit(Unit u)
{
    return u == UNIT_METERS_PER_SECOND || u == UNIT_KM_PER_HOUR;
}

bool isMassUnit(Unit u)
{
    return u == UNIT_KG || u == UNIT_TONS;
}

bool isMassPerAreaUnit(Unit u)
{
    return u == UNIT_KG_PER_METER2 || u == UNIT_TONS_PER_HECTARE;
}

bool isMassPerTimeUnit(Unit u)
{
    return u == UNIT_KG_PER_SECOND;
}

bool isVolumePerAreaUnit(Unit u)
{
    return u == UNIT_METER3_PER_METER2 || u == UNIT_METER3_PER_HECTARE ||
            UNIT_LITERS_PER_METER2 || u == UNIT_LITERS_PER_HECTARE;
}

bool isAngleUnit(Unit u)
{
    return u == UNIT_DEGREE || u == UNIT_RADIAN;
}

bool isCountUnit(Unit u)
{
    return u == UNIT_COUNT;
}

double convertUnits(double val, Unit from, Unit to)
{
    if(!areSameUnitTypes(from, to))
        throw std::invalid_argument( std::string(__FUNCTION__) + ": the given units are not of the same type" );

    if(from == to)
        return val;

    //---time
    if(from == UNIT_SECOND && to == UNIT_HOUR)
        return val / 3600;
    if(from == UNIT_HOUR && to == UNIT_SECOND)
        return val * 3600;

    //---distance
    if(from == UNIT_METERS && to == UNIT_KM)
        return val * 1e-3;
    if(from == UNIT_KM && to == UNIT_METERS)
        return val * 1000;

    //---area
    if(from == UNIT_METER2 && to == UNIT_HECTARE)
        return sqrmeters2hectares(val);
    if(from == UNIT_HECTARE && to == UNIT_METER2)
        return hectares2sqrmeters(val);

    //---volume
    if(from == UNIT_METER3 && to == UNIT_LITER)
        return cubicmeters2liters(val);
    if(from == UNIT_LITER && to == UNIT_METER3)
        return liters2cubicmeters(val);

    //---speed
    if(from == UNIT_METERS_PER_SECOND && to == UNIT_KM_PER_HOUR)
        return meters_s2Km_hour(val);
    if(from == UNIT_KM_PER_HOUR && to == UNIT_METERS_PER_SECOND)
        return Km_hour2meters_s(val);

    //---mass
    if(from == UNIT_KG && to == UNIT_TONS)
        return Kg2tonnes(val);
    if(from == UNIT_TONS && to == UNIT_KG)
        return tonnes2Kg(val);

    //---mass/area
    if(from == UNIT_KG_PER_METER2 && to == UNIT_TONS_PER_HECTARE)
        return Kg_sqrm2t_ha(val);
    if(from == UNIT_TONS_PER_HECTARE && to == UNIT_KG_PER_METER2)
        return t_ha2Kg_sqrm(val);

    //---volume/area
    if(from == UNIT_METER3_PER_METER2 && to == UNIT_METER3_PER_HECTARE)
        return val * hectares2sqrmeters(1.0);
    if(from == UNIT_METER3_PER_METER2 && to == UNIT_LITERS_PER_METER2)
        return cubicmeters2liters(val);
    if(from == UNIT_METER3_PER_METER2 && to == UNIT_LITERS_PER_HECTARE)
        return cubicmeters2liters(val) * hectares2sqrmeters(1.0);

    if(from == UNIT_METER3_PER_HECTARE && to == UNIT_METER3_PER_METER2)
        return val * sqrmeters2hectares(1.0);
    if(from == UNIT_METER3_PER_HECTARE && to == UNIT_LITERS_PER_METER2)
        return cubicmeters2liters(val) * sqrmeters2hectares(1.0);
    if(from == UNIT_METER3_PER_HECTARE && to == UNIT_LITERS_PER_HECTARE)
        return cubicmeters2liters(val);

    if(from == UNIT_LITERS_PER_METER2 && to == UNIT_METER3_PER_METER2)
        return liters2cubicmeters(val);
    if(from == UNIT_LITERS_PER_METER2 && to == UNIT_METER3_PER_HECTARE)
        return liters2cubicmeters(val) * sqrmeters2hectares(1.0);
    if(from == UNIT_LITERS_PER_METER2 && to == UNIT_LITERS_PER_HECTARE)
        return val * hectares2sqrmeters(1.0);

    if(from == UNIT_LITERS_PER_HECTARE && to == UNIT_METER3_PER_METER2)
        return liters2cubicmeters(val) * sqrmeters2hectares(1.0);
    if(from == UNIT_LITERS_PER_HECTARE && to == UNIT_METER3_PER_HECTARE)
        return liters2cubicmeters(val);
    if(from == UNIT_LITERS_PER_HECTARE && to == UNIT_LITERS_PER_METER2)
        return val * sqrmeters2hectares(1.0);


    //---angle
    if(from == UNIT_DEGREE && to == UNIT_RADIAN)
        return deg2rad(val);
    if(from == UNIT_RADIAN && to == UNIT_DEGREE)
        return rad2deg(val);

    throw std::invalid_argument( std::string(__FUNCTION__) + ": conversion not supported" );
}

bool convertUnits(double val_in, double &val_out, Unit from, Unit to)
{
    try{
        val_out = convertUnits(val_in, from, to);
        return true;
    }
    catch(...){
        return false;
    }
}

}
