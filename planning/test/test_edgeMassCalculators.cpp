/*
 * Copyright 2021-2025 DFKI GmbH
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
 
#include <boost/test/unit_test.hpp>

#include <memory>

#include "arolib/planning/edge_calculators/edgeMassCalculator.hpp"
#include "arolib/io/io_xml.hpp"


using namespace arolib;


BOOST_AUTO_TEST_SUITE(test_edgeMassCalculators)
BOOST_AUTO_TEST_CASE(test_edgeMassCalculators_def)
{
    const double Kg_sqrm = 10;

    auto getEMC_def = [&Kg_sqrm]()->std::shared_ptr<IEdgeMassCalculator>{
        std::shared_ptr<EdgeMassCalculatorDef> emc = std::make_shared<EdgeMassCalculatorDef>();
        emc->setParameters( Kg_sqrm2t_ha( Kg_sqrm ) );
        return emc;
    };

    std::cout << "Running test edgeMassCalculators (def)..." << std::endl;

    std::shared_ptr<IEdgeMassCalculator> emc_def = getEMC_def();
    std::shared_ptr<IEdgeMassCalculator> emc_def_x2 = getEMC_def();
    std::shared_ptr<IEdgeMassCalculator> emc_def_bound = getEMC_def();

    std::shared_ptr<ArolibGrid_t> factorMap_x2 = std::make_shared<ArolibGrid_t>();
    float factor_x2 = 2;
    factorMap_x2->createGrid(-1.0, 11.0, -1.0, 11.0, 0.1, &factor_x2);
//    factorMap_x2->saveGridAsGeoTiff("/tmp/factorMap_x2.tif" );


    float val_boundOut = 0;
    const float epsBoundary = 0; //1e-6;
    std::shared_ptr<ArolibGrid_t> factorMap_bound = std::make_shared<ArolibGrid_t>();
    factorMap_bound->createGrid(-1.0, 11.0, -1.0, 6.0, 0.1/*, &val_boundOut*/);
    Polygon boundary = geometry::createRectangleFromLine( Point(-0.5+epsBoundary, 2.5), Point(10.5-epsBoundary, 2.5), 6-2*epsBoundary );
    factorMap_bound->updatePolygonProportionally(boundary, 1.0);
    factorMap_bound->saveGridAsGeoTiff("/tmp/factorMap_bound.tif" );

    std::shared_ptr<gridmap::GridCellsInfoManager> cim = std::make_shared<gridmap::GridCellsInfoManager>();
//    cim->registerGrid("factorMap_x2", *factorMap_x2, true);

    emc_def->setGridCellsInfoManager(cim);
    emc_def_x2->setGridCellsInfoManager(cim);
    emc_def_bound->setGridCellsInfoManager(cim);

    emc_def_x2->setFactorMap(factorMap_x2);
    emc_def_bound->setFactorMap(factorMap_bound);

    double width = 1;
    for(size_t i = 0 ; i < 10 ; ++i){
        for(size_t j = 0 ; j <= 5 ; ++j){
            Point p0 = Point(i, j);
            Point p1 = Point(i+1, j);
            double area = geometry::calc_dist(p0, p1) * width;
            double mass_def = emc_def->calcMass(p0, p1, width);
            double mass_def_x2 = emc_def_x2->calcMass(p0, p1, width);
            double mass_def_bound = emc_def_bound->calcMass(p0, p1, width);
            std::cout << "def   :: [" << p0.toString() << " , " << p1.toString() << "] x " << width << " ::  mass = " << mass_def << " : mass_x2 = " << mass_def_x2 << " : mass_bound = " << mass_def_bound << std::endl << std::endl;

            double mass = Kg_sqrm*area;
            BOOST_CHECK_CLOSE(mass_def, mass, 0.001);
            BOOST_CHECK_CLOSE(mass_def_x2, 2*mass, 0.001);
            BOOST_CHECK_CLOSE(mass_def_bound, mass, 0.1);

//            ArolibGrid_t factorMap_x2_edge;
//            factorMap_x2_edge.copyFrom(*factorMap_x2, true);
//            std::vector<GridCellInfo> cellsInfo;
//            cim->getCellsInfo("factorMap_x2", GridCellsInfoManager::Edge(p0, p1, width, true), cellsInfo);
//            for(auto& info : cellsInfo){
//                factorMap_x2_edge.setValue(info.x, info.y, info.value);
//            }
//            //factorMap_x2_edge.saveValuesInCSV("/tmp/factorMap_x2_edge_" + std::to_string(i) + "_" + std::to_string(j) + ".csv");
//            //factorMap_x2_edge.saveGridAsGeoTiff("/tmp/factorMap_x2_edge_" + std::to_string(i) + "_" + std::to_string(j) + ".tif" );

        }
        for(size_t j = 8 ; j <= 10 ; ++j){
            Point p0 = Point(i, j);
            Point p1 = Point(i+1, j);
            double mass_def_bound = emc_def_bound->calcMass(p0, p1, width);
            std::cout << "def   :: [" << p0.toString() << " , " << p1.toString() << "] x " << width << " ::  mass_bound = " << mass_def_bound << std::endl << std::endl;

            BOOST_CHECK_SMALL(mass_def_bound, 0.001);
        }
    }
    std::cout << "Finished test edgeMassCalculators (def)" << std::endl;
}

BOOST_AUTO_TEST_CASE(test_edgeMassCalculators_MassGrid)
{
    const double Kg_sqrm = 100;

    auto getEMC_grid = [&Kg_sqrm]()->std::shared_ptr<IEdgeMassCalculator>{
        std::shared_ptr<EMC_MassGrid> emc = std::make_shared<EMC_MassGrid>();
        std::shared_ptr<ArolibGrid_t> massmap = std::make_shared<ArolibGrid_t>();
        float massProp = Kg_sqrm2t_ha( Kg_sqrm );
        massmap->createGrid(-1.0, 11.0, -1.0, 11.0, 0.1, &massProp);
//        massmap->saveGridAsGeoTiff("/tmp/massmap.tif" );
        emc->setParameters( Kg_sqrm2t_ha( 10*Kg_sqrm ) );
        emc->setMassMap(massmap);
        return emc;
    };

    std::cout << "Running test edgeMassCalculators (MassGrid)..." << std::endl;

    std::shared_ptr<IEdgeMassCalculator> emc_grid = getEMC_grid();
    std::shared_ptr<IEdgeMassCalculator> emc_grid_x2 = getEMC_grid();
    std::shared_ptr<IEdgeMassCalculator> emc_grid_bound = getEMC_grid();

    std::shared_ptr<ArolibGrid_t> factorMap_x2 = std::make_shared<ArolibGrid_t>();
    float factor_x2 = 2;
    factorMap_x2->createGrid(-1.0, 11.0, -1.0, 11.0, 0.1, &factor_x2);
//    factorMap_x2->saveGridAsGeoTiff("/tmp/factorMap_x2.tif" );


    float val_boundOut = 0;
    const float epsBoundary = 0; //1e-6;
    std::shared_ptr<ArolibGrid_t> factorMap_bound = std::make_shared<ArolibGrid_t>();
    factorMap_bound->createGrid(-1.0, 11.0, -1.0, 6.0, 0.1/*, &val_boundOut*/);
    Polygon boundary = geometry::createRectangleFromLine( Point(-0.5+epsBoundary, 2.5), Point(10.5-epsBoundary, 2.5), 6-2*epsBoundary );
    factorMap_bound->updatePolygonProportionally(boundary, 1.0);
//    factorMap_bound->saveGridAsGeoTiff("/tmp/factorMap_bound.tif" );

    std::shared_ptr<gridmap::GridCellsInfoManager> cim = std::make_shared<gridmap::GridCellsInfoManager>();
//    cim->registerGrid("factorMap_x2", *factorMap_x2, true);

    emc_grid->setGridCellsInfoManager(cim);
    emc_grid_x2->setGridCellsInfoManager(cim);
    emc_grid_bound->setGridCellsInfoManager(cim);

    emc_grid_x2->setFactorMap(factorMap_x2);
    emc_grid_bound->setFactorMap(factorMap_bound);

    double width = 1;
    for(size_t i = 0 ; i < 10 ; ++i){
        for(size_t j = 0 ; j <= 5 ; ++j){
            Point p0 = Point(i, j);
            Point p1 = Point(i+1, j);
            double area = geometry::calc_dist(p0, p1) * width;
            double mass_grid = emc_grid->calcMass(p0, p1, width);
            double mass_grid_x2 = emc_grid_x2->calcMass(p0, p1, width);
            double mass_grid_bound = emc_grid_bound->calcMass(p0, p1, width);

            std::cout << "grid  :: [" << p0.toString() << " , " << p1.toString() << "] x " << width << " ::  mass = " << mass_grid << " : mass_x2 = " << mass_grid_x2 << " : mass_bound = " << mass_grid_bound << std::endl << std::endl;


            double mass = Kg_sqrm*area;
            BOOST_CHECK_CLOSE(mass_grid, mass, 0.001);
            BOOST_CHECK_CLOSE(mass_grid_x2, 2*mass, 0.001);
            BOOST_CHECK_CLOSE(mass_grid_bound, mass, 0.1);
        }
        for(size_t j = 8 ; j <= 10 ; ++j){
            Point p0 = Point(i, j);
            Point p1 = Point(i+1, j);
            double mass_def_bound = emc_grid_bound->calcMass(p0, p1, width);
            std::cout << "grid   :: [" << p0.toString() << " , " << p1.toString() << "] x " << width << " ::  mass_bound = " << mass_def_bound << std::endl << std::endl;

            BOOST_CHECK_SMALL(mass_def_bound, 0.001);
        }
    }
    std::cout << "Finished test edgeMassCalculators (MassGrid)" << std::endl;
}

BOOST_AUTO_TEST_SUITE_END()
