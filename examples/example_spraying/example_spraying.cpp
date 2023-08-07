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
 
/*
=== AroLib Example - Spraying ===

A simple example which executes the whole planning process of AroLib for a
spraying process with one capacitaded sprayer switching working windows (in the inner-field) either only at track-ends or anywhere in the field
*/


#include "arolib/components/machinedb.h"
#include "arolib/components/fieldgeometryprocessor.h"
#include "arolib/components/baseroutesplanner.h"
#include "arolib/components/baseroutesprocessor.h"
#include "arolib/components/fieldprocessplanner.h"
#include "arolib/components/graphprocessor.h"
#include "arolib/planning/edge_calculators/edgeCostCalculatorSoil.hpp"
#include <fstream>

using namespace arolib;

const LogLevel gLogLevel = LogLevel::INFO;

/*
 * Holds the data used during the planning process.
 * */
struct WorkSpace{
    Field field; //field
    std::vector<Machine> workingGroup; //machines participating in the operation
    OutFieldInfo outFieldInfo; //Information related to operations done outside of the field (incl. travel)
    std::map<MachineId_t, MachineDynamicInfo> machineInitialStates; //initial states of the machines
    std::shared_ptr< gridmap::GridCellsInfoManager > cellsInfoManager = std::make_shared<gridmap::GridCellsInfoManager>(gLogLevel);//cells manager used by several components to make computations in gridmaps
    std::shared_ptr<IEdgeSpeedCalculator> speedCalculator = std::make_shared<EdgeWorkingSpeedCalculatorDef>(gLogLevel);//calculator used to compute the machine working speed
    std::shared_ptr<IEdgeSpeedCalculator> speedCalculatorTransit = std::make_shared<EdgeTransitSpeedCalculatorDef>(gLogLevel);//calculator used to compute the machine speed during transit
    std::shared_ptr<IEdgeMassCalculator> massCalculator = nullptr;//calculator used to compute the amount of mass to be extracted from the field
    std::shared_ptr<IEdgeCostCalculator> costCalculator = nullptr;//calculator used to compute the edge costs during path planning
    std::shared_ptr< ArolibGrid_t > massMap = nullptr;//gridmap holding the (bio)mass distribution in the field (t/ha)
    std::shared_ptr< ArolibGrid_t > massFactorMap = nullptr;//gridmap used by the mass calculator to factor the amount of mass in a specific region
    std::shared_ptr< ArolibGrid_t > soilMap = nullptr;//gridmap holding the soil-cost values in the field
    DirectedGraph::Graph graph;//graph used in path planning
    std::vector<Route> baseRoutes;//base-routes
    std::vector<Route> plannedRoutes;//final planned routes
};


bool initTestField( WorkSpace & workSpace );
bool initGridmaps( WorkSpace & workSpace );
bool initMassCalculator( WorkSpace & workSpace );
bool initCostCalculator( WorkSpace & workSpace, size_t operationType );
bool initWorkingGroup( WorkSpace & workSpace );
bool initMachineStates( WorkSpace & workSpace );
bool initOutFieldInfo( WorkSpace & workSpace );
bool processFieldGeometries( WorkSpace & workSpace, size_t headlandType );
bool planBaseRoutes( WorkSpace & workSpace );
bool preProcessBaseRoutes( WorkSpace & workSpace );
bool generateGraph( WorkSpace & workSpace );
bool planOperation( WorkSpace & workSpace, size_t operationType );
bool savePlan( WorkSpace & workSpace, size_t operationType, size_t headlandType );


int main()
{
    WorkSpace workSpace;

    // Get the test field
    if( !initTestField( workSpace ) )
        return 10;

    // Initialize the biomass and soil-cost gridmaps
    if( !initGridmaps( workSpace ) )
        return 20;

    // Initialize the mass calculator
    if( !initMassCalculator( workSpace ) )
        return 30;

    for(size_t operationType = 0 ; operationType < 2 ; ++operationType){

        // Initialize the cost calculator based on the operation type
        if( !initCostCalculator( workSpace, operationType ) )
            return 40 + operationType;

        // Get the machines' working group based
        if( !initWorkingGroup( workSpace) )
            return 50 + operationType;

        for(size_t headlandType = 0 ; headlandType < 1 /*2*/ ; ++headlandType){
            std::cout << std::endl << "----- OPERATION TYPE " << operationType << "  :  HEADLAND TYPE " << headlandType << " -----" << std::endl << std::endl;

            std::cout << std::endl << "-- Working group --" << std::endl;
            for(size_t i = 0 ; i < workSpace.workingGroup.size() ; ++i){
                const auto& machine = workSpace.workingGroup.at(i);
                std::cout << "   Machine # " << ( i+1 ) << std::endl
                          << "      id = " << machine.id << std::endl
                          << "      type = " << Machine::machineTypeToShortString3c(machine.machinetype) << std::endl
                          << "      manufacturer = " << machine.manufacturer << std::endl
                          << "      model = " << machine.model << std::endl
                          << "      width [m] = " << machine.width << std::endl
                          << "      working width [m] = " << machine.working_width << std::endl
                          << "      mass [kg] = " << machine.weight << std::endl;
            }
            std::cout << std::endl << "-- Working group --" << std::endl << std::endl;

            // Initialize the initial states of the machines
            if( !initMachineStates( workSpace ) )
                return 60 + operationType + 2*headlandType;

            // Initialize the out-of-field information based on the working group, access points, resource points, etc.
            if( !initOutFieldInfo( workSpace ) )
                return 70 + operationType + 2*headlandType;

            // Generate the field geometries
            if( !processFieldGeometries( workSpace, headlandType ) )
                return 80 + operationType + 2*headlandType;

            // Generate the headland base routes
            if( !planBaseRoutes( workSpace ) )
                return 90 + operationType + 2*headlandType;

            // Connect the headland and inner-field base routes
            if( !preProcessBaseRoutes( workSpace ) )
                return 110 + operationType + 2*headlandType;

            //generate the graph
            if( !generateGraph( workSpace ) )
                return 120 + operationType + 2*headlandType;

            //plan the routes for all machines
            if( !planOperation( workSpace, operationType ) )
                return 130 + operationType + 2*headlandType;

            //save plan (processed field geometries, routes and some planning parameters)
            savePlan( workSpace, operationType, headlandType );


            std::cout << std::endl << "----- FINISHED PLANNING OPERATION TYPE " << operationType << "  :  HEADLAND TYPE " << headlandType << " -----" << std::endl << std::endl;

        }

    }


    return 0;
}

bool initTestField( WorkSpace & workSpace ){
    auto& field = workSpace.field;
    const std::vector<Point> points_wgs = { Point( 9.93593, 52.00872 ),
                                            Point( 9.93931, 52.00996 ),
                                            Point( 9.94040, 52.00811 ),
                                            Point( 9.93665, 52.00726 ) }; //boundary points in (lon, lat)

    field.clear();

    //get the field boundary
    if( !CoordTransformer::GetInstance().convert_to_cartesian(points_wgs, field.outer_boundary.points) ){
        std::cerr << "-- Error converting boundary points from WGS to UTM --" << std::endl;
        return false;
    }
    geometry::closePolygon( field.outer_boundary );

    //create a subfield with the same field boundary
    field.subfields.emplace_back( Subfield() );
    auto& subfield = field.subfields.front();
    subfield.boundary_outer = field.outer_boundary;

    //add a inner-field track reference line
    subfield.reference_lines.emplace_back( Linestring() );
    subfield.reference_lines.back().points = { field.outer_boundary.points.at(1), field.outer_boundary.points.at(2) };

    //add field access points correspoinding to 2 boundary corners
    FieldAccessPoint accessPoint;
    accessPoint.id = 0;
    accessPoint.point() = field.outer_boundary.points.back();
    subfield.access_points.push_back(accessPoint);
    accessPoint.id = 1;
    accessPoint.point() = r_at( field.outer_boundary.points, 1 );
    subfield.access_points.push_back(accessPoint);

    //add resource points near the access points
    for(auto& ap : subfield.access_points){
        ResourcePoint resourcePoint;
        resourcePoint.id = subfield.resource_points.size();
        resourcePoint.point() = ap.point() + Point(-10, 0);
        subfield.resource_points.push_back( resourcePoint );
    }


    field.id = 0;
    field.name = "Test field";

    return true;
}

/*
 * This function creates test biomass, biomass-factor, and soil-cost gridmaps based on the field boundary;
 * Biomass gridmap: this gridmap holds the values of mass [t/h] in the field
 * Soil-cost gridmap: this gridmap holds the values of soil-cost [0.0, 1.0] in the field.
 * */
bool initGridmaps( WorkSpace & workSpace ){

    auto & massMap = workSpace.massMap;
    auto & massFactorMap = workSpace.massFactorMap;
    auto & soilMap = workSpace.soilMap;
    auto & boundary = workSpace.field.outer_boundary;
    const double cellsize = 2.0; //meters

    //get the center of geometry of the boundary polygon
    Point center;
    if( !geometry::getCentroid(boundary, center) ){
        std::cerr << "-- Error getting the center of geometry of the boundary --" << std::endl;
        return false;
    }

    //create a biomass gridmap based on the field boundary (cells inside the field will have a value = 0; the rest will have no value)
    massMap = std::make_shared<ArolibGrid_t>(gLogLevel);
    if( !massMap->convertPolygonToGrid(boundary.points, cellsize, 0.0) ){
        std::cerr << "-- Error creatring gridmap from boundary --" << std::endl;
        return false;
    }

    //copy the mass gridmap to the soilMap and factorMap
    massFactorMap = std::make_shared<ArolibGrid_t>( *massMap );
    soilMap = std::make_shared<ArolibGrid_t>( *massMap );

    //set the cell values of the biomass gridmap
    for( size_t x = 0 ; x < massMap->getSizeX() ; x++){
        for(size_t y = 0 ; y < massMap->getSizeY() ; y++){
            if( !massMap->hasValue(x, y) )//outside the field
                continue;

            Point cellCenter;
            massMap->getCellCenter(x, y, cellCenter);
            if(cellCenter.x < center.x)
                massMap->setValue(x, y, 30.0);
            else
                massMap->setValue(x, y, 50.0);
        }
    }

    //set a circular region in the soil-cost gridmap with a value of 0.8;
    Polygon circle = geometry::create_circle(center, 40.0, 18);
    if( !soilMap->updatePolygonProportionally(circle, 0.8, true) ){
        std::cerr << "-- Error creatring circular region in the soil-cost gridmap --" << std::endl;
        return false;
    }

    //set the units of the gridmaps
    massMap->setUnits(UNIT_TONS_PER_HECTARE);
    soilMap->setUnits(UNIT_CUSTOM);

    return true;
}


/*
 * This function creates and initializes the edge mass calculator to be used in the planning process. In this case, of type EMC_MassGrid
 * */
bool initMassCalculator( WorkSpace & workSpace ){
    std::shared_ptr<EMC_MassGrid> massCalculator = std::make_shared< EMC_MassGrid >(gLogLevel);
    massCalculator->setGridCellsInfoManager( workSpace.cellsInfoManager );
    massCalculator->setMassMap( workSpace.massMap );
    massCalculator->setParameters( 40.0 );
    massCalculator->setMapsComputationPrecision( gridmap::SharedGridsManager::PRECISE_ONLY_IF_AVAILABLE );

    workSpace.massCalculator = massCalculator;
    return true;
}

/*
 * This function creates and initializes the edge cost calculator to be used in the planning process.
 * If operationType == 0 -> Time optimization
 * If operationType != 0 -> Soil protection
 * */
bool initCostCalculator( WorkSpace & workSpace, size_t operationType ){
    IEdgeCostCalculator::GeneralParameters generalParams;
    if(operationType == 0){
        generalParams.crossCostMult = 25;
        generalParams.boundaryCrossCostMult = 50;
        workSpace.costCalculator = std::make_shared< ECC_timeOptimization >(gLogLevel);
    }
    else{
        generalParams.crossCostMult = 2.5;
        generalParams.boundaryCrossCostMult = 10;

        ECC_soilOptimization1::CostCoefficients costCoeffs;
        costCoeffs.K_prevWeights = 0.5;
        costCoeffs.K_bias = 0.2;
        costCoeffs.K_soil = 3;
        costCoeffs.Kpow_soil = 2;
        costCoeffs.K_time = 0;
        costCoeffs.K_prevWeights = 0.5;

        std::shared_ptr<ECC_soilOptimization1> costCalculator = std::make_shared< ECC_soilOptimization1 >(gLogLevel);
        costCalculator->setGridCellsInfoManager( workSpace.cellsInfoManager );
        costCalculator->setCostCoefficients( costCoeffs );
        costCalculator->setSoilCostMap( workSpace.soilMap );
        costCalculator->setBoundary( workSpace.field.outer_boundary );

        workSpace.costCalculator = costCalculator;
    }

    workSpace.costCalculator->setGeneralParameters( generalParams );
    return true;
}

/*
 * This function updates the mass-factor gridmap used by the mass calculator.
 * The cells inside the boundary will have a hactor value of 1.0, whereas the cells outside will have bo value
 * */
bool updateMassFactorMap( WorkSpace & workSpace, const Polygon & boundary ){

    if( boundary.points.empty() ){//remove the factor map
        //set all values to 1.0
        if( !workSpace.massFactorMap->setAllValues( 1.0 ) ){
            std::cerr << "-- Error setting all factor map values to 1.0 --" << std::endl;
            return false;
        }
    }
    else{
        //set all values to 'no value'
        if( !workSpace.massFactorMap->setAllValues( nullptr ) ){
            std::cerr << "-- Error setting all factor map values to 'no value' --" << std::endl;
            return false;
        }

        //set all values inside the polygon to 1.0
        if( !workSpace.massFactorMap->updatePolygonProportionally(boundary, 1.0, false) ){
            std::cerr << "-- Error setting the factor map values inside the boundary --" << std::endl;
            return false;
        }
    }
    return true;
}

/*
 * This function initializes the working group of machines with:
 * - One capacitated straying machine
 * */
bool initWorkingGroup( WorkSpace & workSpace ){

    workSpace.workingGroup.clear();

    Machine m;
    m.id = 0;
    m.machinetype = Machine::SPRAYER;
    m.width = 3; // m
    m.length = 8; // m
    m.working_width = 6; // m
    m.weight = 27000; // kg
    m.bunker_mass = 16000; // kg
    m.turning_radius = 7; // m
    m.max_speed_empty = 4; // m/s
    m.max_speed_full = 3.5; // m/s
    m.def_working_speed = 2.5; // m/s
    m.num_axis = 2; //
    m.manufacturer = "Test Company"; //
    m.model = "Capacitated Sprayer"; //


//    Machine m;
//    m.id = 0;
//    m.machinetype = Machine::SPRAYER;
//    m.width = 2.75; // m
//    m.length = 8; // m
//    m.working_width = 2.75; // m
//    m.weight = 21160; // kg
//    m.bunker_mass = 10000; // kg
//    m.turning_radius = 5.7; // m
//    m.max_speed_empty = 2.5; // m/s
//    m.max_speed_full = 2.5; // m/s
//    m.def_working_speed = 2.5; // m/s
//    m.num_axis = 2;
//    m.manufacturer = "Undef"; //
//    m.model = "Manure Tanker";

    workSpace.workingGroup.push_back(m);

    return true;
}

/*
 * This function initializes the current states of the machines in the working group
 * */
bool initMachineStates( WorkSpace & workSpace ){
    auto& states = workSpace.machineInitialStates;
    FieldAccessPoint refAccessPoint;

    try{
        refAccessPoint = workSpace.field.subfields.front().access_points.front();
    }
    catch(...){
        std::cerr << "-- Error getting first access point of the subfield --" << std::endl;
        return false;

    }

    states.clear();

    for(size_t indMachine = 0 ; indMachine < workSpace.workingGroup.size() ; ++indMachine){
        const auto& machine = workSpace.workingGroup.at(indMachine);
        MachineDynamicInfo state;
        state.position = refAccessPoint.point() + Point(-10.0 - indMachine*10, 10.0 + indMachine*10); //random(-ish) location
        state.theta = -M_PI_2;

        //set the bunker to half full
        state.bunkerMass = machine.bunker_mass * 0.5;
        state.bunkerVolume = machine.bunker_volume * 0.5;

        states[machine.id] = state;
    }

    return true;
}

/*
 * This function populates the out-of-field information based on the working group of machines and the access points and resource points of the field
 * */
bool initOutFieldInfo( WorkSpace & workSpace ){

    auto &outFieldInfo = workSpace.outFieldInfo;
    const auto & resourcePoints = workSpace.field.subfields.front().resource_points;
    const auto & accessPoints = workSpace.field.subfields.front().access_points;
    const double defaultSpeed = 2;
    const double unloadingRate = 100; //kg/s

    outFieldInfo.clearAll();

    //add travel information between resource points and access points
    for( auto& resourcePoint : resourcePoints ){
        for( auto& accessPoint : accessPoints ){
            OutFieldInfo::TravelData travelData;
            travelData.fieldAccessPointId = accessPoint.id;
            travelData.resourcePointId = resourcePoint.id;
            travelData.machineId = OutFieldInfo::AllMachines; //apply these travel costs for all machines
            travelData.machineBunkerState = OutFieldInfo::ALL_MACHINE_STATES; //apply these travel costs for all machine states
            travelData.travelCosts.distance = geometry::calc_manhattan_dist(resourcePoint, accessPoint);
            travelData.travelCosts.time = travelData.travelCosts.distance / defaultSpeed;

            outFieldInfo.add_FAP2RP(travelData);
            outFieldInfo.add_RP2FAP(travelData);

            //for machine with ID = 0, add a specific travel cost
            travelData.machineId = 0;
            travelData.travelCosts.time *= 1.5;

            outFieldInfo.add_FAP2RP(travelData);
            outFieldInfo.add_RP2FAP(travelData);

        }
    }

    //add travel information between access points
    for( auto& accessPoint1 : resourcePoints ){
        for( auto& accessPoint2 : accessPoints ){
            if(accessPoint1.id == accessPoint2.id)//do not connect the access point with itself
                continue;

            OutFieldInfo::TravelData2 travelData;
            travelData.fap_id_from = accessPoint1.id;
            travelData.fap_id_to = accessPoint2.id;
            travelData.machineId = OutFieldInfo::AllMachines; //apply these travel costs for all machines
            travelData.machineBunkerState = OutFieldInfo::ALL_MACHINE_STATES; //apply these travel costs for all machine states
            travelData.travelCosts.distance = geometry::calc_manhattan_dist(accessPoint1, accessPoint2);
            travelData.travelCosts.time = travelData.travelCosts.distance / defaultSpeed;

            outFieldInfo.add_FAP2FAP(travelData);

            std::swap(travelData.fap_id_from, travelData.fap_id_to);

            outFieldInfo.add_FAP2FAP(travelData);
        }
    }

    //add arrival travel information (i.e. between current location of the machines and the access points)
    for( auto& accessPoint : resourcePoints ){
        for(auto& it_machine : workSpace.machineInitialStates){
            const auto& machineId = it_machine.first;
            const auto& machineState = it_machine.second;
            OutFieldInfo::ArrivalData arrivalData;
            arrivalData.machineId = machineId;
            arrivalData.machineBunkerState = OutFieldInfo::ALL_MACHINE_STATES; //apply this for all machine states
            arrivalData.fieldAccessPointId = accessPoint.id;
            arrivalData.arrivalCosts.distance = geometry::calc_manhattan_dist(accessPoint, machineState.position);
            arrivalData.arrivalCosts.time = arrivalData.arrivalCosts.distance / defaultSpeed;

            outFieldInfo.add_arrivalCosts(arrivalData);
        }
    }


    //add unloading costs to all resource points and machines
    for(auto& machine : workSpace.workingGroup){
        OutFieldInfo::UnloadingData unloadingData;
        unloadingData.machineId = machine.id;
        unloadingData.resourcePointId = OutFieldInfo::AllResourcePoints; // apply these costs to all resource points
        unloadingData.unloadingCosts.time = machine.bunker_mass / unloadingRate; //set different unloading times depending on the machine capacity
        outFieldInfo.add_unloadingCosts(unloadingData);
    }


    return true;
}

/*
 * This function generates the field geometries (headland boundaries, headland track, and inner-field boundary, inner-field tracks)
 * */
bool processFieldGeometries( WorkSpace & workSpace, size_t headlandType ){

    std::cout << std::endl << "-- Processing field geometries..... --" << std::endl;

    double workingWidth = -1;
    for(auto& machine : workSpace.workingGroup){
        if( Machine::isOfWorkingType( machine.machinetype ) )
            workingWidth = std::max(workingWidth, machine.working_width);
    }

    FieldGeometryProcessor::HeadlandParameters paramsHL;
    paramsHL.headlandWidth = 20;//minimum headland width (the real width will be a multiple of the trackWidth/workingWidth)
    paramsHL.trackWidth = workingWidth;
    paramsHL.numTracks = 0;//will be computed automatically ased on headlandWidth and trackWidth
    paramsHL.sampleResolution = 5;
    paramsHL.headlandConnectionTrackWidth = -1; //for partial/side headlands

    FieldGeometryProcessor::InfieldParameters paramsIF;
    paramsIF.sampleResolution = 5;
    paramsIF.checkForRemainingTracks = true;
    paramsIF.onlyUntilBoundaryIntersection = false;
    paramsIF.shiftingStrategy = geometry::TracksGenerator::TRANSLATE_TRACKS;
    paramsIF.direction = Point(0, 0);//obtain the translation direction automatically
    paramsIF.trackSamplingStrategy = geometry::TracksGenerator::DEFAULT_SAMPLING;
    paramsIF.trackDistance = workingWidth;

    FieldGeometryProcessor fgp(gLogLevel);
    if(headlandType == 0){
        auto aroResp = fgp.processSubfieldWithSurroundingHeadland(workSpace.field.subfields.front(),
                                                                  paramsHL,
                                                                  paramsIF,
                                                                  0,
                                                                  nullptr);
        if(aroResp.isError()){
            std::cerr << "-- Error generating geometries for field with complete/surrounding headland: " << aroResp.msg << " --" << std::endl;
            return false;
        }
    }
    else{
        auto aroResp = fgp.processSubfieldWithSideHeadlands(workSpace.field.subfields.front(),
                                                            paramsHL,
                                                            paramsIF,
                                                            0);
        if(aroResp.isError()){
            std::cerr << "-- Error generating geometries for field with partial/side headlands: " << aroResp.msg << " --" << std::endl;
            return false;
        }
    }

    std::cout << std::endl << "-- Finished generating field geometries --" << std::endl;
    return true;
}


/*
 * This function generates the base-routes of the sprayer for the headland
 * */
bool planBaseRoutes( WorkSpace & workSpace ){

    std::cout << std::endl << "-- Planning base-routes for the sprayer..... --" << std::endl;

    BaseRoutesPlanner::PlannerParameters plannerParameters;
    plannerParameters.workHeadlandFirst = true;
    plannerParameters.workedAreaTransitRestriction = HeadlandBaseRoutesPlanner::WorkedAreaTransitRestriction::NO_RESTRICTION;
    plannerParameters.startHeadlandFromOutermostTrack = true;
    plannerParameters.finishHeadlandWithOutermostTrack = false; //for partial/side headlands
    plannerParameters.headlandClockwise = true; //for complete/surrounding headland
    plannerParameters.restrictToBoundary = true; //the mass calculation in the headland will check the intersection with the field outer boundary
    plannerParameters.monitorPlannedAreasInHeadland = false; //do not monitor which areas have been planned to be harvested in previously planned tracks
    plannerParameters.headlandSpeedMultiplier = 0.8;//the speed of the sprayer when working the headland will be scaled by 0.8


    BaseRoutesPlanner brp(gLogLevel);
    auto aroResp = brp.plan(workSpace.field.subfields.front(),
                            workSpace.workingGroup,
                            plannerParameters,
                            workSpace.massCalculator,
                            workSpace.speedCalculator,
                            workSpace.speedCalculator,
                            workSpace.speedCalculatorTransit,
                            workSpace.baseRoutes,
                            nullptr,//the massFactorMap was already applied in the edgeMassCalculator
                            &workSpace.machineInitialStates,
                            nullptr,
                            &workSpace.outFieldInfo,
                            nullptr);
    if(aroResp.isError()){
        std::cerr << "-- Error generating the base routes: " << aroResp.msg << " --" << std::endl;
        return false;
    }

    std::cout << std::endl << "-- Finished planning base-routes for the sprayer --" << std::endl;
    return true;
}


/*
 * This function connects the base-routes of the headland and inner-field and adjust the route point properties accordingly
 * */
bool preProcessBaseRoutes( WorkSpace & workSpace ){

    std::cout << std::endl << "-- Reversing base-routes --" << std::endl;
    //reverse base routes to spray the inner-field before the headland
    for(size_t i = 0 ; i < workSpace.baseRoutes.size(); ++i)
        workSpace.baseRoutes[i] = BaseRoutesProcessor::reverseRoute( workSpace.baseRoutes[i] );

    std::cout << std::endl << "-- Finished reversing base-routes --" << std::endl;

    return true;
}

/*
 * This function generates the graph to be used in path planning
 * */
bool generateGraph( WorkSpace & workSpace ){

    std::cout << std::endl << "-- Creating the graph..... --" << std::endl;

    GraphProcessor::Settings gpSettings;
    gpSettings.incVisitPeriods = false;
    gpSettings.workingWidth = -1;
    for(auto& machine : workSpace.workingGroup){
        if( Machine::isOfWorkingType( machine.machinetype ) )
            gpSettings.workingWidth = std::max(gpSettings.workingWidth, machine.working_width);
    }
    gpSettings.workingWidthHL = gpSettings.workingWidth;

    GraphProcessor graphProcessor(gLogLevel);
    auto aroResp = graphProcessor.createGraph(workSpace.field.subfields.front(),
                                              workSpace.baseRoutes,
                                              workSpace.workingGroup,
                                              gpSettings,
                                              workSpace.outFieldInfo,
                                              workSpace.machineInitialStates,
                                              workSpace.graph);
    if(aroResp.isError()){
        std::cerr << "-- Error generating the graph: " << aroResp.msg << " --" << std::endl;
        return false;
    }

    std::cout << std::endl << "-- Finished creating the graph --" << std::endl;

    return true;

}

/*
 * This function generates the operation routes for all machines
 * */
bool planOperation(WorkSpace & workSpace , size_t operationType){

    std::cout << std::endl << "-- Planning operation routes..... --" << std::endl;

    FieldProcessPlanner::PlannerParameters plannerParameters;
    plannerParameters.threadsOption = MultiOLVPlanner::MULTIPLE_THREADS; //plan using multiple threads (one per permutation)
    plannerParameters.clearanceTime = 10; // time [s] a machine has to wait for a vertex to be free
    plannerParameters.collisionAvoidanceOption = Astar::COLLISION_AVOIDANCE__OVERALL; //with collition avoidance
    plannerParameters.finishAtResourcePoint = true; //transport vehicles will finish at the unloading facility
    plannerParameters.includeCostOfOverload = true;
    plannerParameters.includeWaitInCost = true;
    plannerParameters.maxPlanningTime = plannerParameters.max_planning_time = 0; //do not limit the planning time
    plannerParameters.max_waiting_time = 1800;// do not wait more that x seconds for a vertex to be free
//    plannerParameters.olvOrderStrategy = MultiOLVPlanner::CHECK_ALL_PERMUTATIONS; //check all permutations (options) to order the transport vehicles (starting with the estimated best order)
//    plannerParameters.numFixedInitalOlvsInOrder = 0; //check permutations starting from the first overload window (i.e. do not assign by force any of the trnsport vehicles to any of the initial working windows)

    if(operationType == 0)
        plannerParameters.switchOnlyAtTrackEnd = true; //switch between working windows (in the inner-field) only at track-ends
    else
        plannerParameters.switchOnlyAtTrackEnd = false; //switch between working windows anywhere in the field when the capacity limits are reached

    PlanGeneralInfo planInfo;
    FieldProcessPlanner planner(gLogLevel);
    auto aroResp = planner.planSubfield( workSpace.graph,
                                         workSpace.field.subfields.front(),
                                         workSpace.baseRoutes,
                                         workSpace.plannedRoutes,
                                         workSpace.workingGroup,
                                         workSpace.outFieldInfo,
                                         workSpace.machineInitialStates,
                                         plannerParameters,
                                         ArolibGrid_t(),
                                         ArolibGrid_t(),
                                         workSpace.costCalculator,
                                         &planInfo);
    if(aroResp.isError()){
        std::cerr << "-- Error planning the operation routes: " << aroResp.msg << " --" << std::endl;
        return false;
    }

    std::cout << std::endl << "-- Finished planning operation routes --" << std::endl;

    return true;

}

/*
 * This function saves the processed field and planned routes in '/tmp' (if possible)
 * */
bool savePlan( WorkSpace & workSpace, size_t operationType, size_t headlandType ){
    const std::string baseDir = "/tmp";
    const std::string outDir = baseDir + "/example_spraying/operationType_" + std::to_string(operationType) + "__headlandType_" + std::to_string(headlandType) + "/";

    std::cout << std::endl << "-- Saving field and plan in " << outDir << "'..... --" << std::endl;

    if (!boost::filesystem::exists(baseDir.c_str())){
        std::cerr << "-- Unable to save plan: base directory '" << baseDir << "' does not exist --" << std::endl;
        return false;
    }

    if (!io::create_directory(outDir)){
        std::cerr << "-- Unable to save plan: error creating output directory '" << outDir << "' --" << std::endl;
        return false;
    }

    if (!io::writeFieldKML(outDir + "processedField.kml", workSpace.field)){
        std::cerr << "-- Unable to save plan: error saving field in output directory '" << outDir << "' --" << std::endl;
        return false;
    }

    std::cout << std::endl << "-- Field (KML) saved in " << outDir << "' --" << std::endl;

    std::map<std::string, ArolibGrid_t* > gridmaps;
    gridmaps["biomass"] = workSpace.massMap.get();
    gridmaps["soilcost"] = workSpace.soilMap.get();
    if( !io::writePlanXML( outDir + "plan.xml",
                           workSpace.field,
                           workSpace.workingGroup,
                           {workSpace.plannedRoutes},
                           gridmaps ) ){
        std::cerr << "-- Unable to save plan: error saving plan in output directory '" << outDir << "' --" << std::endl;
        return false;
    }

    std::cout << std::endl << "-- Field and plan saved in " << outDir << "' --" << std::endl;

    return true;

}
