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
 
/*
=== AroLib Example - Harvesting ===

A simple example which executes the whole planning process of AroLib
Harvesting process with one capacitaded harvester OR one non-capacitated harvester and 2 transport vehicles
*/


#include "arolib/components/machinedb.h"
#include "arolib/components/headlandplanner.h"
#include "arolib/components/baseroutesinfieldplanner.h"
#include "arolib/components/tracksgenerator.h"
#include "arolib/components/baseroutesprocessor.h"
#include "arolib/components/fieldprocessplanner.h"
#include "arolib/components/graphprocessor.h"
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
    std::shared_ptr<IEdgeSpeedCalculator> speedCalculator = std::make_shared<EdgeSpeedCalculatorDef>(gLogLevel);//calculator used to compute the harvester spped while harvesting
    std::shared_ptr<IEdgeMassCalculator> massCalculator = nullptr;//calculator used to compute the amount of mass to be extracted from the field
    std::shared_ptr<IEdgeCostCalculator> costCalculator = nullptr;//calculator used to compute the edge costs during path planning
    std::shared_ptr< ArolibGrid_t > massMap = nullptr;//gridmap holding the (bio)mass distribution in the field (t/ha)
    std::shared_ptr< ArolibGrid_t > massFactorMap = nullptr;//gridmap used by the mass calculator to factor the amount of mass in a specific region
    std::shared_ptr< ArolibGrid_t > soilMap = nullptr;//gridmap holding the soil-cost values in the field
    DirectedGraph::Graph graph;//graph used in path planning
    std::vector<HeadlandRoute> baseRoutes_headland;//base-routes for the headland
    std::vector<Route> baseRoutes_innerField;//base-routes for the inner-field
    std::vector<Route> baseRoutes;//base-routes
    std::vector<Route> plannedRoutes;//final planned routes
};


bool initTestField( WorkSpace & workSpace );
bool initGridmaps( WorkSpace & workSpace );
bool initMassCalculator( WorkSpace & workSpace );
bool initCostCalculator( WorkSpace & workSpace, size_t operationType );
bool updateMassFactorMap( WorkSpace & workSpace, const Polygon & boundary );
bool initWorkingGroup_cHarv( WorkSpace & workSpace );
bool initWorkingGroup_harvTV( WorkSpace & workSpace );
bool initMachineStates( WorkSpace & workSpace );
bool initOutFieldInfo( WorkSpace & workSpace );
bool planHeadland( WorkSpace & workSpace );
bool generateInnerFieldTracks( WorkSpace & workSpace );
bool planInnerFieldBaseRoutes( WorkSpace & workSpace );
bool preProcessBaseRoutes( WorkSpace & workSpace );
bool generateGraph( WorkSpace & workSpace );
bool planOperation( WorkSpace & workSpace );
bool savePlan( WorkSpace & workSpace, size_t operationType );


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

        std::cout << std::endl << "----- PLANNING OPERATION TYPE " << operationType << " -----" << std::endl << std::endl;

        // Initialize the cost calculator based on the operation type
        if( !initCostCalculator( workSpace, operationType ) )
            return 40 + operationType;

        // Get the machines' working group based on the operation type
        if(operationType == 0){ //capacitated harvester
            if( !initWorkingGroup_cHarv( workSpace) )
                return 50 + operationType;
        }
        else{ //non-capacitated harvester + transport vehicles
            if( !initWorkingGroup_harvTV( workSpace ) )
                return 50 + operationType;
        }

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
            return 60 + operationType;

        // Initialize the out-of-field information based on the working group, access points, resource points, etc.
        if( !initOutFieldInfo( workSpace ) )
            return 70 + operationType;

        // Generate the headland geometries and headland base-routes
        if( !planHeadland( workSpace ) )
            return 80 + operationType;

        // Generate the inner-field tracks
        if( !generateInnerFieldTracks( workSpace ) )
            return 90 + operationType;

        // Generate the inner-field base routes
        if( !planInnerFieldBaseRoutes( workSpace ) )
            return 100 + operationType;

        // Connect the headland and inner-field base routes
        if( !preProcessBaseRoutes( workSpace ) )
            return 110 + operationType;

        //generate the graph
        if( !generateGraph( workSpace ) )
            return 120 + operationType;

        //plan the routes for all machines
        if( !planOperation( workSpace ) )
            return 130 + operationType;

        //save plan (processed field geometries, routes and some planning parameters)
        savePlan( workSpace, operationType );


        std::cout << std::endl << "----- FINISHED PLANNING OPERATION TYPE " << operationType << " -----" << std::endl << std::endl;

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
 * This function initializes the working group of machines with:.
 * - One capacitated harvester
 * */
bool initWorkingGroup_cHarv( WorkSpace & workSpace ){

    workSpace.workingGroup.clear();

    Machine harv;
    harv.id = 0;
    harv.machinetype = Machine::HARVESTER;
    harv.width = 5; // m
    harv.length = 15; // m
    harv.working_width = 5; // m
    harv.weight = 35000; // kg
    harv.bunker_mass = 30000; // kg
    harv.turning_radius = 7; // m
    harv.max_speed_empty = 5; // m/s
    harv.max_speed_full = 4; // m/s
    harv.def_working_speed = 2.5; // m/s
    harv.num_axis = 3; //
    harv.manufacturer = "Test Company"; //
    harv.model = "Capacitated Harvester"; //

    workSpace.workingGroup.push_back(harv);


    return true;
}

/*
 * This function initializes the working group of machines with:
 * - One harvester without capacity
 * - Two transport (overload) vehicles
 * */
bool initWorkingGroup_harvTV( WorkSpace & workSpace ){

    workSpace.workingGroup.clear();

    Machine harv;
    harv.id = 10;
    harv.machinetype = Machine::HARVESTER;
    harv.width = 3; // m
    harv.length = 7; // m
    harv.working_width = 6; // m
    harv.weight = 19000; // kg
    harv.bunker_mass = 0; // kg
    harv.turning_radius = 6; // m
    harv.max_speed_empty = 5; // m/s
    harv.max_speed_full = 4; // m/s
    harv.def_working_speed = 2.5; // m/s
    harv.num_axis = 2; //
    harv.manufacturer = "Test Company"; //
    harv.model = "Non-capacitated Harvester"; //

    workSpace.workingGroup.push_back(harv);

    Machine olv;
    olv.machinetype = Machine::OLV;
    olv.width = 3; // m
    olv.length = 7; // m
    olv.working_width = 3; // m
    olv.weight = 21000; // kg
    olv.bunker_mass = 10000; // kg
    olv.turning_radius = 6; // m
    olv.max_speed_empty = 5; // m/s
    olv.max_speed_full = 4; // m/s
    olv.def_working_speed = 2.5; // m/s
    olv.num_axis = 2; //
    olv.manufacturer = "Test Company"; //
    olv.model = "Overload Vehicle"; //

    for(int i = 11 ; i < 13 ; ++i){
        olv.id = i;
        workSpace.workingGroup.push_back(olv);
    }
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
    bool olvAdded = false;

    for(size_t indMachine = 0 ; indMachine < workSpace.workingGroup.size() ; ++indMachine){
        const auto& machine = workSpace.workingGroup.at(indMachine);
        MachineDynamicInfo state;
        state.position = refAccessPoint.point() + Point(-10.0 - indMachine*10, 10.0 + indMachine*10); //random(-ish) location
        state.theta = -M_PI_2;
        if(machine.machinetype != Machine::OLV || olvAdded)
            state.bunkerMass = state.bunkerVolume = 0;
        else{//set the bunker to half full for one of the OLVs
            state.bunkerMass = machine.bunker_mass * 0.5;
            state.bunkerVolume = machine.bunker_volume * 0.5;
            olvAdded = true;
        }
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
 * This function generates the headland geometries (headland track, and inner-field boundary) and the base-routes of the harvester
 * */
bool planHeadland( WorkSpace & workSpace ){

    std::cout << std::endl << "-- Planning the headland..... --" << std::endl;

    //update the factor map with all values in 1.0
    if(!updateMassFactorMap(workSpace, Polygon())){
        std::cerr << "-- Error updating mass factor map --" << std::endl;
        return false;
    }

    HeadlandPlanner::PlannerParameters plannerParameters;
    plannerParameters.clockwise = true;
    plannerParameters.avgMassPerArea = 40;
    plannerParameters.headlandWidth = 20;//minimum headland width (the real width will be a multiple of the harvester's working width)
    plannerParameters.sampleResolution = 5;

    HeadlandPlanner headlandPlanner(gLogLevel);
    auto aroResp = headlandPlanner.planHeadlandSubfield(workSpace.field.subfields.front(),
                                                        workSpace.workingGroup,
                                                        workSpace.outFieldInfo,
                                                        plannerParameters,
                                                        ArolibGrid_t(),
                                                        workSpace.massCalculator,
                                                        workSpace.speedCalculator,
                                                        workSpace.baseRoutes_headland);
    if(aroResp.isError()){
        std::cerr << "-- Error planning the headland: " << aroResp.msg << " --" << std::endl;
        return false;
    }

    std::cout << std::endl << "-- Finished planning the headland --" << std::endl;
    return true;
}

/*
 * This function generates the tracks of the inner-field
 * */
bool generateInnerFieldTracks( WorkSpace & workSpace ){

    std::cout << std::endl << "-- Generating the inner-field tracks..... --" << std::endl;

    TracksGenerator::TracksGeneratorParameters TGParams;
    TGParams.sampleResolution = 5;
    TGParams.checkForRemainingTracks = true;
    TGParams.onlyUntilBoundaryIntersection = false;
    TGParams.shiftingStrategy = TracksGenerator::TRANSLATE_TRACKS;
    TGParams.direction = Point(0, 0);//obtain the translation direction automatically
    TGParams.trackSamplingStrategy = TracksGenerator::DEFAULT_SAMPLING;
    TGParams.trackDistance = -1;
    for(auto& machine : workSpace.workingGroup){
        if( Machine::isOfWorkingType( machine.machinetype ) )
            TGParams.trackDistance = std::max(TGParams.trackDistance, machine.working_width);
    }

    TracksGenerator tracksGenerator(gLogLevel);
    auto aroResp = tracksGenerator.generateTracks(workSpace.field.subfields.front(),
                                                  TGParams);
    if(aroResp.isError()){
        std::cerr << "-- Error generating the inner-field tracks: " << aroResp.msg << " --" << std::endl;
        return false;
    }

    std::cout << std::endl << "-- Finished generating the inner-field tracks --" << std::endl;
    return true;
}

/*
 * This function generates the base-routes of the harvester for the inner-field
 * */
bool planInnerFieldBaseRoutes( WorkSpace & workSpace ){

    std::cout << std::endl << "-- Planning inner-field base-routes for the harvester..... --" << std::endl;

    //set the reference pose (used by the planner to select where to start planning the base routes in the innner field) to the last pose of the headland base_route.
    std::vector<Pose2D> refPose;
    if(!workSpace.baseRoutes_headland.empty() && workSpace.baseRoutes_headland.front().route_points.size() > 1){
        refPose.push_back( Pose2D(workSpace.baseRoutes_headland.front().route_points.back(),
                                  arolib::geometry::get_angle( r_at(workSpace.baseRoutes_headland.front().route_points, 1),
                                                               workSpace.baseRoutes_headland.front().route_points.back() ) ) );
    }

    //update the factor map to remove already harvested areas in the headland
    if(!updateMassFactorMap(workSpace, workSpace.field.subfields.front().boundary_inner)){
        std::cerr << "-- Error updating mass factor map with inner boundary --" << std::endl;
        return false;
    }

    BaseRoutesInfieldPlanner::PlannerParameters plannerParameters;
    plannerParameters.avgMassPerArea = 40;
    plannerParameters.sequenceStrategy = SimpleTrackSequencer::MEANDER;//simple sequencing

    BaseRoutesInfieldPlanner infieldPlanner(gLogLevel);
    auto aroResp = infieldPlanner.plan(workSpace.field.subfields.front(),
                                       workSpace.workingGroup,
                                       plannerParameters,
                                       ArolibGrid_t(),
                                       workSpace.machineInitialStates,
                                       *workSpace.massCalculator,
                                       *workSpace.speedCalculator,
                                       workSpace.baseRoutes_innerField,
                                       refPose);
    if(aroResp.isError()){
        std::cerr << "-- Error generating the inner-field base routes: " << aroResp.msg << " --" << std::endl;
        return false;
    }

    std::cout << std::endl << "-- Finished planning inner-field base-routes for the harvester --" << std::endl;
    return true;
}

/*
 * This function connects the base-routes of the headland and inner-field and adjust the route point properties accordingly
 * */
bool preProcessBaseRoutes( WorkSpace & workSpace ){

    std::cout << std::endl << "-- Connecting base-routes..... --" << std::endl;

    BaseRoutesProcessor::Settings brpSettings;

    BaseRoutesProcessor brp(gLogLevel);
    auto aroResp = brp.processRoutes(workSpace.field.subfields.front(),
                                     workSpace.baseRoutes_headland,
                                     workSpace.baseRoutes_innerField,
                                     workSpace.workingGroup,
                                     brpSettings,
                                     workSpace.baseRoutes);
    if(aroResp.isError()){
        std::cerr << "-- Error connecting the base routes: " << aroResp.msg << " --" << std::endl;
        return false;
    }

    std::cout << std::endl << "-- Finished connecting base-routes --" << std::endl;

    return true;
}

/*
 * This function generates the graph to be used in path planning
 * */
bool generateGraph( WorkSpace & workSpace ){

    std::cout << std::endl << "-- Creating the graph..... --" << std::endl;

    GraphProcessor::Settings gpSettings;
    gpSettings.incVisitPeriods = true;
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
bool planOperation( WorkSpace & workSpace ){

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
    plannerParameters.olvOrderStrategy = MultiOLVPlanner::CHECK_ALL_PERMUTATIONS; //check all permutations (options) to order the transport vehicles (starting with the estimated best order)
    plannerParameters.numFixedInitalOlvsInOrder = 0; //check permutations starting from the first overload window (i.e. do not assign by force any of the trnsport vehicles to any of the initial working windows)


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
bool savePlan( WorkSpace & workSpace, size_t operationType ){
    const std::string baseDir = "/tmp";
    const std::string outDir = baseDir + "/example_harvesting/operationType_" + std::to_string(operationType) + "/";

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
