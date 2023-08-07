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

#include "arolib/geometry/boost_geometries_wrapper.hpp"

namespace arolib{

namespace geometry{

boost_polygon_t toBoostPolygon(const PolygonWithHoles &poly){
    return toBoostPolygon(poly.outer, poly.holes);
}

boost_polygon_t toBoostPolygon(const Polygon &outer, const std::vector<Polygon> &inners){
    boost_polygon_t boostPoly;
    std::string description = "POLYGON(";
    if(!outer.points.empty()){
        description += "(";
        for(size_t i = 0 ; i < outer.points.size() ; ++i){
            description += double2string(outer.points.at(i).x) + " " + double2string(outer.points.at(i).y);
            if(i != outer.points.size()-1)
                description += ",";
        }
        description += ")";
    }
    for(size_t i = 0 ; i < inners.size() ; ++i){
        description += "(";
        for(size_t j = 0 ; j < inners.at(i).points.size() ; ++j){
            description += double2string(inners.at(i).points.at(j).x) + " " + double2string(inners.at(i).points.at(j).y);
            if(j != inners.at(i).points.size()-1)
                description += ",";
        }
        description += ")";
    }
    description += ")";

    boost::geometry::read_wkt(description, boostPoly);
    return boostPoly;
}

PolygonWithHoles fromBoostPolygon(const boost_polygon_t &boostPoly)
{
   PolygonWithHoles ret;
   fromBoostPolygon(boostPoly, ret.outer, &ret.holes);
   return ret;
}

void fromBoostPolygon(const boost_polygon_t &boostPoly, Polygon &outter, std::vector<Polygon> *inners){
    outter.points.resize( boostPoly.outer().size() );
    for(size_t i = 0 ; i < boostPoly.outer().size() ; ++i){
        outter.points.at(i).x = boostPoly.outer().at(i).x();
        outter.points.at(i).y = boostPoly.outer().at(i).y();
    }
    if(inners){
        inners->clear();
        for(size_t i = 0 ; i < boostPoly.inners().size() ; ++i){
            inners->push_back(Polygon());
            inners->back().points.resize(boostPoly.inners().at(i).size());
            for(size_t j = 0 ; j < boostPoly.inners().at(i).size() ; ++j){
                inners->back().points.at(j).x = boostPoly.inners().at(i).at(j).x();
                inners->back().points.at(j).y = boostPoly.inners().at(i).at(j).y();
            }
        }
    }
}

}

}
