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
 
#include "arolib/processing/overrideMapProcessor.hpp"

namespace arolib{

OverrideMapProcessor::OverrideMapProcessor(LogLevel logLevel)
    : LoggingComponent(logLevel, __FUNCTION__)
{

}

AroResp OverrideMapProcessor::createMaps_simple(const Polygon &boundary,
                                                const std::vector<Route> &routes,
                                                const std::vector<Machine> &machines,
                                                double resolution,
                                                std::shared_ptr<ArolibGrid_t> overrideCountMap,
                                                std::shared_ptr<ArolibGrid_t> overrideMassMap,
                                                std::shared_ptr<ArolibGrid_t> overrideMassPerTimeMap)
{
    if(!arolib::geometry::isPolygonValid(boundary))
        return AroResp(1, "Invalid boundary");
    if(machines.empty())
        return AroResp(1, "No machines given");
    if(routes.empty())
        return AroResp(1, "No routes given");
    if(!overrideCountMap && !overrideMassMap && !overrideMassPerTimeMap)
        return AroResp(1, "No maps to generate");

    std::map<MachineId_t, double> machineWidths;
    std::map<MachineId_t, Machine> machinesMap;
    double min_width = std::numeric_limits<double>::max();
    for(auto & m : machines){
        double width;
        if(m.width > 0)
            width = m.width;
        else if(m.working_width > 0)
            width = m.working_width;
        else
            return AroResp(1, "Invalid width of machine with id " + std::to_string(m.id));
        machineWidths[m.id] = width;
        machinesMap[m.id] = m;
        min_width = std::min(min_width, width);
    }

    if(resolution <= 1e-9)
       resolution = min_width;

    for(auto& route : routes){
        if( machineWidths.find(route.machine_id) == machineWidths.end() )
            return AroResp(1, "Machine with id " + std::to_string(route.machine_id) + " of route " + std::to_string(route.route_id) + " not given.");
    }

    AroResp resp = initMaps(boundary, resolution, overrideCountMap, overrideMassMap, overrideMassPerTimeMap);
    if(resp.isError())
        return resp;

    std::shared_ptr<ArolibGrid_t> refMap;//all maps have the same geometry. use this as reference
    if(overrideCountMap)
        refMap = overrideCountMap;
    else if(overrideMassMap)
        refMap = overrideMassMap;
    else if(overrideMassPerTimeMap)
        refMap = overrideMassPerTimeMap;

    for(auto& route : routes){
        auto &machine = machinesMap[route.machine_id];

        for(size_t i = 0 ; i+1 < route.route_points.size() ; ++i){
            auto& rp0 = route.route_points.at(i);
            auto& rp1 = route.route_points.at(i+1);

            if(rp0.type == RoutePoint::FIELD_EXIT || rp1.type == RoutePoint::FIELD_ENTRY)
                continue;

            int indRef = i;
            unsigned int prevX = refMap->getSizeX()+1;
            unsigned int prevY = refMap->getSizeY()+1;

            auto rp0_tmp = rp0;
            bool keepSampling = true;

            double samplingRes = 0.25 * resolution;

            while(keepSampling){
                RoutePoint rp1_tmp;
                double dist = arolib::geometry::calc_dist(rp0_tmp, rp1);
                if(dist <= samplingRes){
                    rp1_tmp = rp1;
                    keepSampling = false;
                }
                else{
                    double deltaTime = (rp1.time_stamp - rp0_tmp.time_stamp) * samplingRes / dist;
                    rp1_tmp = route.calcPoint2(rp0_tmp.time_stamp + deltaTime, indRef);
                }

                Point center = arolib::geometry::getCentroid(rp0_tmp, rp1_tmp);
                double edgeMass = machine.weight + 0.5 * (rp1_tmp.bunker_mass + rp0_tmp.bunker_mass);
                double edgeDuration = rp1_tmp.time_stamp - rp0_tmp.time_stamp;

                rp0_tmp = rp1_tmp;

                unsigned int x, y;
                if( !refMap->point2index(center, x, y) )
                    continue;

                bool errorTmp;
                if(overrideCountMap){
                    if(x != prevX || y != prevY){
                        double prevVal = 0;
                        if(overrideCountMap->hasValue(x, y, &errorTmp) && !errorTmp)
                            prevVal = overrideCountMap->getValue(x, y, &errorTmp);
                        overrideCountMap->setValue(x, y, prevVal+1);
                    }
                }
                if(overrideMassMap){
                    if(x != prevX || y != prevY){
                        double prevVal = 0;
                        if(overrideMassMap->hasValue(x, y, &errorTmp) && !errorTmp)
                            prevVal = overrideMassMap->getValue(x, y, &errorTmp);
                        overrideMassMap->setValue(x, y, prevVal+edgeMass);
                    }
                }
                if(overrideMassPerTimeMap && edgeDuration > 0){
                    double prevVal = 0;
                    if(overrideMassPerTimeMap->hasValue(x, y, &errorTmp) && !errorTmp)
                        prevVal = overrideMassPerTimeMap->getValue(x, y, &errorTmp);
                    overrideMassPerTimeMap->setValue(x, y, prevVal + edgeMass * edgeDuration);
                }

                prevX = x;
                prevY = y;
            }



        }

    }


    return resp;
}

AroResp OverrideMapProcessor::createMaps(const Polygon &boundary,
                                         const std::vector<Route> &routes,
                                         const std::vector<Machine> &machines,
                                         double resolution,
                                         bool bePrecise,
                                         std::shared_ptr<ArolibGrid_t> overrideCountMap,
                                         std::shared_ptr<ArolibGrid_t> overrideMassMap,
                                         std::shared_ptr<ArolibGrid_t> overrideMassPerTimeMap)
{
    if(!arolib::geometry::isPolygonValid(boundary))
        return AroResp(1, "Invalid boundary");
    if(machines.empty())
        return AroResp(1, "No machines given");
    if(routes.empty())
        return AroResp(1, "No routes given");
    if(!overrideCountMap && !overrideMassMap && !overrideMassPerTimeMap)
        return AroResp(1, "No maps to generate");

    std::map<MachineId_t, double> machineWidths;
    std::map<MachineId_t, Machine> machinesMap;
    double min_width = std::numeric_limits<double>::max();
    for(auto & m : machines){
        double width;
        if(m.width > 0)
            width = m.width;
        else if(m.working_width > 0)
            width = m.working_width;
        else
            return AroResp(1, "Invalid width of machine with id " + std::to_string(m.id));
        machineWidths[m.id] = width;
        machinesMap[m.id] = m;
        min_width = std::min(min_width, width);
    }

    if(resolution <= 1e-9)
       resolution = min_width * 0.5;

    for(auto& route : routes){
        if( machineWidths.find(route.machine_id) == machineWidths.end() )
            return AroResp(1, "Machine with id " + std::to_string(route.machine_id) + " of route " + std::to_string(route.route_id) + " not given.");
    }

    AroResp resp = initMaps(boundary, resolution, overrideCountMap, overrideMassMap, overrideMassPerTimeMap);
    if(resp.isError())
        return resp;

    std::shared_ptr<ArolibGrid_t> refMap;//all maps have the same geometry. use this as reference
    if(overrideCountMap)
        refMap = overrideCountMap;
    else if(overrideMassMap)
        refMap = overrideMassMap;
    else if(overrideMassPerTimeMap)
        refMap = overrideMassPerTimeMap;

    for(auto& route : routes){
        double width = machineWidths[route.machine_id];
        auto &machine = machinesMap[route.machine_id];

        std::unordered_map< int, std::unordered_set<int> > prevCells;
        for(size_t i = 0 ; i+1 < route.route_points.size() ; ++i){
            auto& rp0 = route.route_points.at(i);
            auto& rp1 = route.route_points.at(i+1);
            double dist = arolib::geometry::calc_dist(rp0, rp1);

            if(rp0.type == RoutePoint::FIELD_EXIT || rp1.type == RoutePoint::FIELD_ENTRY){
                prevCells.clear();
                continue;
            }

            std::vector<gridmap::GridmapLayout::GridCellOverlap> cellsInfo;
/*            if(dist < resolution){
                Point center = arolib::geometry::getCentroid(rp0, rp1);
                unsigned int x, y;
                if( !refMap->point2index(center, x, y) )
                    continue;
                cellsInfo.emplace_back( gridmap::GridmapLayout::GridCellOverlap(x,y,dist / resolution) );
            }
            else*/{
                if(bePrecise)
                    cellsInfo = refMap->getCellsOverlapUnderLine(rp0, rp1, width);
                else{
                    size_t cellsCount = 0;
                    auto indexMap = refMap->getCellsUnderLine(rp0, rp1, width);
                    int min_y, max_y;
                    for (unsigned int x = indexMap.minX() ; x <= indexMap.maxX() ; ++x){
                        auto yRanges = indexMap.getColumn(x);
                        int lastY = -1;
                        for(const auto& yRange : yRanges){
                            min_y = yRange.first;
                            max_y = yRange.second;
                            if(lastY == min_y)
                                ++min_y;
                            lastY = max_y;
                            for (int y = min_y ; y <= max_y ; ++y){
                                cellsInfo.emplace_back( gridmap::GridmapLayout::GridCellOverlap(x,y,1) );//set a value of 1 to all
                                ++cellsCount;
                            }
                        }
                    }
                }
            }

            if(!bePrecise){//avoid counting over cells from the previous segment
                auto prevCells2 = prevCells;

                //update cells of current segment and remove the cells that were computed in the previous segment
                prevCells.clear();
                for(size_t j = 0 ; j < cellsInfo.size() ; ++j){
                    prevCells[cellsInfo.at(j).x].insert(cellsInfo.at(j).y);
                    auto it_x = prevCells2.find(cellsInfo.at(j).x);
                    if(it_x != prevCells2.end() && it_x->second.find(cellsInfo.at(j).y) != it_x->second.end() ){
                        cellsInfo.erase( cellsInfo.begin()+j );
                        --j;
                        continue;
                    }
                }

            }


            double edgeMass = machine.weight + 0.5 * (rp1.bunker_mass + rp0.bunker_mass);
            double deltaTime = std::fabs( rp1.time_stamp - rp0.time_stamp );

            if(overrideCountMap)
                overrideCountMap->addCellsValue(1, cellsInfo);
            if(overrideMassMap)
                overrideMassMap->addCellsValue(edgeMass, cellsInfo);
            if(overrideMassPerTimeMap && deltaTime > 0){
                double deltaTimePerCell = deltaTime;
                if(dist > resolution)
                    deltaTimePerCell *= resolution/dist;//estimated duration per cell
                for(auto& cell : cellsInfo)
                    overrideMassPerTimeMap->addValue( cell.x, cell.y, cell.overlap * (edgeMass/deltaTime) );
            }

        }

    }


    return resp;
}

AroResp OverrideMapProcessor::initMaps(const Polygon &boundary,
                                       double resolution,
                                       std::shared_ptr<ArolibGrid_t> overrideCountMap,
                                       std::shared_ptr<ArolibGrid_t> overrideMassMap,
                                       std::shared_ptr<ArolibGrid_t> overrideMassPerTimeMap)
{
    double minX, maxX, minY, maxY;
    arolib::geometry::getPolygonLimits(boundary, minX, maxX, minY, maxY);

    ArolibGrid_t baseGrid;
    if(!baseGrid.createGrid(minX, maxX, minY, maxY, resolution))
        return AroResp(1, "Error creating base grid");

    if(overrideCountMap){
        *overrideCountMap =  baseGrid;
        overrideCountMap->setUnits(Unit::UNIT_COUNT);
    }

    if(overrideMassMap){
        *overrideMassMap =  baseGrid;
        overrideMassMap->setUnits(Unit::UNIT_KG);
    }

    if(overrideMassPerTimeMap){
        *overrideMassPerTimeMap =  baseGrid;
        overrideMassPerTimeMap->setUnits(Unit::UNIT_CUSTOM);
    }

    return AroResp::ok();
}


}
