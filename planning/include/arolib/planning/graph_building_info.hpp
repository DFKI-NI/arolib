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
 
#ifndef _AROLIB_GRAPH_BUILDING_INFO_H
#define _AROLIB_GRAPH_BUILDING_INFO_H

#include <iostream>
#include <fstream>
#include <tuple>
#include <ctime>
#include <functional>
#include <unordered_set>

#include <arolib/planning/directedgraph.hpp>
#include <arolib/misc/loggingcomponent.h>
#include <arolib/misc/filesystem_helper.h>
#include <arolib/types/coordtransformer.hpp>


namespace arolib{
namespace DirectedGraph{

/**
 * @brief Class used to store information about how a graph was built (for debug)
 */
class GraphBuildingInfoManager : public LoggingComponent
{
public:

    /**
     * @brief Information about and added edge
     */
    struct EdgeInfo
    {
        std::string added_desc; /**< Description on when the edge was added */
        EdgeType edge_type; /**< Edge type */
        vertex_t vt_from; /**< Source vertex */
        vertex_t vt_to; /**< Target vertex */
        Point p_from; /**< Location of the source vertex */
        Point p_to; /**< Location of the target vertex */
        RoutePoint::RoutePointType vt_from_type; /**< Type of the source vertex */
        RoutePoint::RoutePointType vt_to_type; /**< Type of the target vertex */
        bool bidirectional; /**< Was the connection done bidirectionally? */

        /**
         * @brief Default constructor
         */
        EdgeInfo() = default;

        /**
         * @brief Constructor
         * @param _added_desc Description on when the edge was added
         * @param _edge_type Edge type
         * @param _vt_from Source vertex
         * @param _vt_to Target vertex
         * @param _p_from Location of the source vertex
         * @param _p_to Location of the target vertex
         * @param _vt_from_type Type of the source vertex
         * @param _vt_to_type Type of the target vertex
         * @param _bidirectional Was the connection done bidirectionally?
         */
        EdgeInfo(const std::string _added_desc, EdgeType _edge_type,
                 const vertex_t _vt_from, const vertex_t _vt_to,
                 const Point _p_from, const Point _p_to,
                 RoutePoint::RoutePointType _vt_from_type,
                 RoutePoint::RoutePointType _vt_to_type,
                 bool _bidirectional);
    };

    /**
     * @brief Constructor
     * @param logLevel Log level
     */
    explicit GraphBuildingInfoManager(LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Get the list of edges-info
     * @return List of edges-info
     */
    inline const std::vector<EdgeInfo>& edgesInfo() const { return m_edgesInfo; }

    /**
     * @brief Add an edges-info
     * @param info Edge info
     */
    void addEdgeInfo(const EdgeInfo& info);

    /**
     * @brief Save the list of edges-info in a file
     * @param filename File name/path
     * @return True on success
     */
    bool saveEdgesInfo(const std::string& filename);

    /**
     * @brief Read the list of edges-info from a file
     * @param filename File name/path
     * @return True on success
     */
    bool loadEdgesInfo(const std::string& filename);

    /**
     * @brief Cleare current data
     */
    void clear();

public:
    std::vector<EdgeInfo> m_edgesInfo; /**< List of edges-info */
};

}

}
#endif //_AROLIB_GRAPH_BUILDING_INFO_H
