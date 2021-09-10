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
 
#include "arolib/cartography/sharedgridsmanager.hpp"


namespace arolib {
namespace gridmap{

std::set<int> SharedGridsManager::m_ids;

SharedGridsManager::SharedGridsManager(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{
    do{
        m_id = gen_random_int(0, 1000);
    }while( m_ids.find(m_id) != m_ids.end() );

    m_ids.insert(m_id);
    m_prefix = "sgm_" + std::to_string(m_id) + "__";
}

SharedGridsManager::~SharedGridsManager()
{
    removeGridsFromCIM();
    m_ids.erase(m_id);
}

bool SharedGridsManager::setCellsInfoManager(std::shared_ptr<gridmap::GridCellsInfoManager> cim, bool removeGrids)
{
    if(cim == m_cellsInfoManager)
        return true;

    if(removeGrids)
        removeGridsFromCIM();
    m_cellsInfoManager = cim;

    bool ok = true;

    if(m_cellsInfoManager){
        for(auto& grid_it : m_grids){
            std::string name_cim = m_prefix + grid_it.first;
            if( m_cellsInfoManager->isGridRegistered(name_cim) ){
                logger().printWarning(__FUNCTION__, "A grid with name " + name_cim + " is already registered in the GridCellsInfoManager. It will be replaced!");
                //ok = false;
            }
            if(grid_it.second->isAllocated()){
                if( !m_cellsInfoManager->registerGrid( name_cim, *(grid_it.second), true) ){
                    logger().printError(__FUNCTION__, "Error adding current grid " + name_cim + " to CellsInfoManager");
                    ok = false;
                }
            }
        }
    }
    return ok;
}

bool SharedGridsManager::hasGrid(const std::string &name)
{
    return m_grids.find( name ) != m_grids.end();
}

bool SharedGridsManager::addGrid(const std::string &name, ConstGridPtr grid, bool replaceIfExists){
    if(!grid /*|| !grid->isAllocated()*/){
        logger().printError(__FUNCTION__, "Invalid grid");
        return false;
    }

    auto it = m_grids.find( name );
    if( it != m_grids.end() ){
        if(!replaceIfExists)
            return true;
        it->second = grid;
    }
    else
        it = m_grids.insert( std::make_pair(name, grid) ).first;

    if(m_cellsInfoManager && grid->isAllocated()){//if not allocated it will be registered later (if it get later allocated)
        if( !m_cellsInfoManager->registerGrid( m_prefix + name, *grid, replaceIfExists) )
            logger().printError(__FUNCTION__, "Error adding grid to CellsInfoManager");
    }
    return true;
}

bool SharedGridsManager::removeGrid(const std::string &name)
{
    auto it = m_grids.find( name );
    if( it == m_grids.end() ){
        logger().printWarning(__FUNCTION__, "Grid not found");
        return true;
    }

    if(m_cellsInfoManager){
        if( m_cellsInfoManager->isGridRegistered(m_prefix + name) && !m_cellsInfoManager->removeGrid(m_prefix + name) )
            logger().printError(__FUNCTION__, "Error removing grid from CellsInfoManager");
    }

    m_grids.erase(it);

    return true;

}

SharedGridsManager::ConstGridPtr SharedGridsManager::getGrid(const std::string &name) const
{
    auto it = m_grids.find( name );
    if( it == m_grids.end() ){
        //logger().printWarning(__FUNCTION__, "Grid not found");
        return nullptr;
    }

    return it->second;

}

SharedGridsManager::GridPtr SharedGridsManager::getGridCopy(const std::string &name) const
{
    auto grid = getGrid(name);
    if(!grid)
        return nullptr;

    return std::make_shared<ArolibGrid_t>(*grid);
}

bool SharedGridsManager::clearAll()
{
    removeGridsFromCIM();
    m_grids.clear();
    return true;
}

bool SharedGridsManager::getCellsInfoUnderLine(const std::string &gridName, const Point &p0, const Point &p1, double width, PreciseCalculationOption precise, std::vector<gridmap::GridmapLayout::GridCellOverlap>& cellsInfo)
{
    cellsInfo.clear();

    auto it = m_grids.find( gridName );
    if( it == m_grids.end() ){
        logger().printWarning(__FUNCTION__, "Grid not found");
        return true;
    }

    auto grid = it->second;

    if(!grid->isAllocated()){
        logger().printWarning(__FUNCTION__, "Grid not allocated");
        return true;
    }

    width = std::fabs(width);    
    if(width < 1e-9)
        width = 0;


    GridCellsInfoManager::Edge edge(p0, p1, width, precise != PreciseCalculationOption::NOT_PRECISE);

    //check first if the info is in the CIM
    if(m_cellsInfoManager){
        bool gridRegistered = m_cellsInfoManager->isGridRegistered(m_prefix + gridName);
        if( !gridRegistered )//it might have not been registered if it was not allocated when addded
            gridRegistered = m_cellsInfoManager->registerGrid(m_prefix + gridName, *grid, true);

        if(gridRegistered){
            if(!m_cellsInfoManager->checkGridGeometry(m_prefix + gridName, *grid)) //grid geometry changed
                m_cellsInfoManager->registerGrid(m_prefix + gridName, *grid, true); //replace

            else if(precise == PreciseCalculationOption::PRECISE_ONLY_IF_AVAILABLE){
                if(!m_cellsInfoManager->hasCellsInfo(m_prefix + gridName, edge))
                    edge.precise = false;
            }

            if( m_cellsInfoManager->computeAndUpdateCellsInfo(m_prefix + gridName, edge, *grid, true, &cellsInfo) )
                return true;
        }
        else
            logger().printWarning(__FUNCTION__, "Error with grid registratio in GridCellsInfoManager");
    }

    edge.precise = (precise == PreciseCalculationOption::PRECISE);
    return GridCellsInfoManager::computeListCellsUnderLine(*grid, edge, cellsInfo);
}

void SharedGridsManager::removeGridsFromCIM()
{
    if(m_cellsInfoManager){
        for( auto& it : m_grids ){
            if( m_cellsInfoManager->isGridRegistered(m_prefix + it.first) && !m_cellsInfoManager->removeGrid(m_prefix + it.first) )
                logger().printError(__FUNCTION__, "Error removing grid " + it.first + " from CellsInfoManager");
        }
    }
}

}

}
