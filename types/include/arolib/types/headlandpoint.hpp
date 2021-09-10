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
 
#ifndef _AROLIB_HEADLANDPOINT_H_
#define _AROLIB_HEADLANDPOINT_H_

#include <vector>
#include "point.hpp"

namespace arolib {

/**
  * @brief Type of point belonging to the headland middle track (subfield planned headland)
  */
class HeadlandPoint : public Point {
public:
    /**
      * @brief Constructor
      */
    explicit HeadlandPoint() : Point(0,0), track_id(-1) {}

    /**
      * @brief Constructor
      * @param p Location
      * @param trackid Track id
      */
    explicit HeadlandPoint(const Point &p, const int &trackid = -1) : Point(p), track_id(trackid) {}


public:
    int track_id = -1; /**< Id of the track related to the headland point (if <0, no track is related) */
};

/**
  * @brief operator<< to add (print) a headland point to a output stream
  * @param ostr Output stream
  * @param hp Headland point to be added/printed
  * @return Updated output stream
  */
inline std::ostream& operator<< (std::ostream &ostr, const HeadlandPoint& hp) {
  ostr << "(track:" << hp.track_id  << ", point: " << hp.point() << ")";
  return ostr;
}

}

#endif
