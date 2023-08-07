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
 
#ifndef _AROLIB_OBSTACLE_H_
#define _AROLIB_OBSTACLE_H_

#include "polygon.hpp"

namespace arolib {

/**
  * @brief Obstacle class
  */
class Obstacle {
public:

    /**
      * @brief Obstacle types
      */
    enum ObstacleType{
        OBS_OTHER,
        OBS_TREES,
        OBS_WATER,
        OBS_ROCKS,
        OBS_POLE,
        OBS_WINDMILL
    };

    /**
      * @brief Conver int to onstacle type
      */
    static ObstacleType intToObstacleType(int value){
        if(value == ObstacleType::OBS_OTHER)
            return ObstacleType::OBS_OTHER;
        else if(value == ObstacleType::OBS_TREES)
            return ObstacleType::OBS_TREES;
        else if(value == ObstacleType::OBS_WATER)
            return ObstacleType::OBS_WATER;
        else if(value == ObstacleType::OBS_ROCKS)
            return ObstacleType::OBS_ROCKS;
        else if(value == ObstacleType::OBS_POLE)
            return ObstacleType::OBS_POLE;
        else if(value == ObstacleType::OBS_WINDMILL)
            return ObstacleType::OBS_WINDMILL;

        throw std::invalid_argument( "The given value does not correspond to any Obstacle::ObstacleType" );
    }

    ObstacleType type = ObstacleType::OBS_OTHER; /**< Type */
    std::string type_description; /**< Type description */
    Polygon boundary; /**< Perimeter points */
};

inline bool operator==(const Obstacle& lhs, const Obstacle& rhs) {
  return (lhs.type == rhs.type) && (lhs.boundary == rhs.boundary);
}

}

#endif
