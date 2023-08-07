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
 
#include "arolib/planning/path_search/graph_building_info.hpp"

namespace arolib{
namespace DirectedGraph{

GraphBuildingInfoManager::EdgeInfo::EdgeInfo(const std::string _added_desc, EdgeType _edge_type, const vertex_t _vt_from, const vertex_t _vt_to, const Point _p_from, const Point _p_to, RoutePoint::RoutePointType _vt_from_type, RoutePoint::RoutePointType _vt_to_type, bool _bidirectional)
    : added_desc(_added_desc)
    , edge_type(_edge_type)
    , vt_from(_vt_from)
    , vt_to(_vt_to)
    , p_from(_p_from)
    , p_to(_p_to)
    , vt_from_type(_vt_from_type)
    , vt_to_type(_vt_to_type)
    , bidirectional(_bidirectional)
{
}

GraphBuildingInfoManager::GraphBuildingInfoManager(LogLevel logLevel)
    : LoggingComponent(logLevel, __FUNCTION__)
{

}

void GraphBuildingInfoManager::addEdgeInfo(const GraphBuildingInfoManager::EdgeInfo &info)
{
    m_edgesInfo.emplace_back(info);
}

bool GraphBuildingInfoManager::saveEdgesInfo(const std::string &filename)
{
    if(filename.empty()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid filename" );
        return false;
    }

    auto dir = io::getPathToFile(filename);
    if(!dir.empty())
        io::create_directory(dir);
    std::ofstream file(filename);
    if(!file.is_open()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Unable to open file " + filename );
        return false;
    }

    const std::string tab = ";";
    file << "added_desc" << tab
            << "edge_type" << tab
            << "vertex_from" << tab
            << "vertex_to" << tab
            << "p_from" << tab << tab
            << "p_to" << tab << tab
            << "vt_from_type" << tab
            << "vt_to_type" << tab
            << "bidirectional" << std::endl;

    for(auto &info : m_edgesInfo){
        Point p0Geo = info.p_from, p1Geo = info.p_to;
        arolib::CoordTransformer::GetInstance().convert_to_geodetic( info.p_from , p0Geo );
        arolib::CoordTransformer::GetInstance().convert_to_geodetic( info.p_to , p1Geo );
        file << std::setprecision(12)
             << info.added_desc << tab
             << info.edge_type << tab
             << info.vt_from << tab
             << info.vt_to << tab
             << p0Geo.x << tab << p0Geo.y << tab
             << p1Geo.x << tab << p1Geo.y << tab
             << info.vt_from_type << tab
             << info.vt_to_type << tab
             << info.bidirectional << tab << std::endl;
    }

    file.close();
    return true;
}

bool GraphBuildingInfoManager::loadEdgesInfo(const std::string &filename)
{
    m_edgesInfo.clear();

    try{

        std::ifstream file(filename);
        if(!file.is_open()){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error reading graph from " + filename);
            return false;
        }

        const size_t expected = 9;
        const std::string tab = ";";
        size_t indAdded = 0;
        size_t indEdgeType = 1;
        size_t indVertexIn = 2;
        size_t indVertexOut = 3;
        size_t indP0 = 4;
        size_t indP1 = 6;
        size_t indP0type = 7;
        size_t indP1type = 8;
        size_t indBidirectional = 8;
        std::string line;

        std::getline(file, line);

        std::vector<std::string> headers;
        boost::split(headers, line, boost::is_any_of(tab));
        for(size_t i = 0 ; i < headers.size() ; ++i){
            if(headers.at(i) == "added_desc")
                indAdded = i;
            else if(headers.at(i) == "edge_type")
                indEdgeType = i;
            else if(headers.at(i) == "vertex_from")
                indVertexIn = i;
            else if(headers.at(i) == "vertex_to")
                indVertexOut = i;
            else if(headers.at(i) == "p_from")
                indP0 = i;
            else if(headers.at(i) == "p_to")
                indP1 = i;
            else if(headers.at(i) == "vt_from_type")
                indP0type = i;
            else if(headers.at(i) == "vt_to_type")
                indP1type = i;
            else if(headers.at(i) == "bidirectional")
                indBidirectional = i;
        }

        while(std::getline(file, line)){
            std::vector<std::string> info;
            boost::split(info, line, boost::is_any_of(tab));
            if(info.size() < expected){
                logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error reading graph from " + filename + ": not enough data in line");
                return false;
            }

            EdgeInfo ei;
            Point pGeo;
            ei.added_desc = info.at(indAdded);
            ei.edge_type = intToEdgeType( std::stoi( info.at(indEdgeType) ) );
            ei.vt_from = std::stoi( info.at(indVertexIn) );
            ei.vt_to = std::stoi( info.at(indVertexOut) );
            pGeo.x = string2double( info.at(indP0) );
            pGeo.y = string2double( info.at(indP0+1) );
            CoordTransformer::GetInstance().convert_to_cartesian(pGeo, ei.p_from);
            pGeo.x = string2double( info.at(indP1) );
            pGeo.y = string2double( info.at(indP1+1) );
            CoordTransformer::GetInstance().convert_to_cartesian(pGeo, ei.p_to);
            ei.vt_from_type = RoutePoint::intToRoutePointType( std::stoi( info.at(indP0type) ) );
            ei.vt_to_type = RoutePoint::intToRoutePointType( std::stoi( info.at(indP1type) ) );
            ei.bidirectional = std::stoi( info.at(indBidirectional) );

            m_edgesInfo.push_back(ei);
        }

        return true;

    }
    catch(std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error reading graph from " + filename  + " - Exception cought: " + e.what());
        return false;
    }
}

void GraphBuildingInfoManager::clear()
{
    m_edgesInfo.clear();
}


}

}
