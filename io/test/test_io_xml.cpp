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
#include <boost/filesystem.hpp>

#include "arolib/io/io_xml.hpp"
#include "arolib/geometry/field_geometry_processing.hpp"

#include "test_io_common.cpp"

using namespace arolib;
using namespace arolib::io;

namespace {

boost::filesystem::path getOutputDir(){
    auto out_dir = boost::filesystem::temp_directory_path() / "arolib" / "test" / "test_io_xml";
    boost::filesystem::create_directories(out_dir);
    return out_dir;
}

std::map<std::string, std::map<std::string, std::string> > getParametersAsStringMap()
{
    std::map<std::string, std::map<std::string, std::string> > map;

    auto addParams = [&map](std::string name, size_t num_params)
    {
        auto& subMap = map[name];
        for(size_t i = 0 ; i < num_params ; ++i){
            if( i%2 == 0)
                subMap[ "param_" + std::to_string(i) ] = "val " + std::to_string(i);
            else
                subMap[ "param_" + std::to_string(i) ] = std::to_string(i) + ".000";
        }
    };

    for(size_t i = 0 ; i < 6 ; ++i)
        addParams( "parameter_set_" + std::to_string(i) , i );


    return map;

}

//arolib::Field getTestField(const arolib::Point& trans = arolib::Point(0,0)){

//    arolib::Point pRef = arolib::Point(564300, 5762400) + trans;
//    const double dx = 200;
//    const double dy = 100;
//    const double hl_width = 20;
//    arolib::Polygon polyTmp;

//    arolib::Field field;
//    field.id = 100;
//    field.name = "TestField";

//    field.outer_boundary = arolib::geometry::createRectangleFromLine(pRef, pRef + arolib::Point(2*dx, 0), dy);

//    for(size_t s = 0 ; s < 2 ; ++s){
//        field.subfields.emplace_back(arolib::Subfield());
//        auto& sf = field.subfields.back();
//        sf.id = 1000 + s;
//        sf.boundary_outer = arolib::geometry::createRectangleFromLine(pRef + arolib::Point(s*dx, 0), pRef + arolib::Point(dx*(1+s), 0), dy);
//        arolib::geometry::offsetPolygon(sf.boundary_outer, sf.boundary_inner, hl_width, false, 0);

//        sf.headlands.complete.headlandWidth = hl_width;
//        sf.headlands.complete.boundaries = std::make_pair(sf.boundary_outer, sf.boundary_inner);
//        for(int i = 5 ; i < hl_width ; i += 5){
//            arolib::geometry::offsetPolygon(sf.boundary_outer, polyTmp, i, false, 0);
//            sf.headlands.complete.tracks.emplace_back(arolib::Track());
//            sf.headlands.complete.tracks.back().id = 1000*s + i;
//            sf.headlands.complete.tracks.back().points = polyTmp.points;
//        }
//        sf.reference_lines.emplace_back(arolib::Linestring({pRef, pRef + arolib::Point(dx*s, 50)}));

//        arolib::geometry::TracksGenerator tg;
//        arolib::geometry::TracksGenerator::TracksGeneratorParameters tgp;
//        tgp.trackDistance = tgp.sampleResolution = 10;
//        tg.generateTracks(sf, tgp, 0);

//        for(size_t i = 0 ; i+1 < sf.boundary_outer.points.size() ; ++i)
//            sf.access_points.emplace_back(arolib::FieldAccessPoint(sf.boundary_outer.points.at(i), i));

//        arolib::Point center;
//        arolib::geometry::getCentroid(sf.boundary_outer, center);
//        sf.obstacles.emplace_back( arolib::geometry::create_circle(center, 5, 10, 0.0, -2*M_PI) );
//        sf.obstacles.emplace_back( arolib::geometry::create_circle(center + Point(5, 0), 2, 10, 0.0, -2*M_PI) );

//        auto updateResourcePoint = [&sf, &s](){
//            auto& rp = sf.resource_points.back();
//            rp.geometry = arolib::geometry::create_circle(rp, 1, 5);
//            rp.massCapacity = 10 * rp.id;
//            rp.massCapacity = 100 * rp.id;
//        };

//        sf.resource_points.emplace_back( arolib::ResourcePoint(field.external_roads.back().points.front(), 10*s + 1 ) );
//        updateResourcePoint();
//        sf.resource_points.emplace_back( arolib::ResourcePoint( arolib::geometry::getPointAtHalfLength(field.external_roads.back().points).first, 10*s + 2 ) );
//        updateResourcePoint();
//    }

//    arolib::geometry::offsetPolygon(field.outer_boundary, polyTmp, 10, true, 0);
//    field.external_roads.emplace_back(polyTmp.points);
//    arolib::geometry::offsetPolygon(field.outer_boundary, polyTmp, 20, true, 0);
//    field.external_roads.emplace_back(polyTmp.points);


//    return field;
//}

std::vector<Machine> getMachines(){
    std::vector<Machine> ret(4);
    for(size_t i = 0 ; i < ret.size() ; ++i){
        auto& m = ret.at(i);
        auto n = std::to_string(i);

        try{
            m.machinetype = Machine::intToMachineType(i);
        }
        catch(...){
            m.machinetype = Machine::UNDEFINED_TYPE;
        }

        m.id = i;
        m.manufacturer = "Man " + n;
        m.model = "Mod " + n;
        m.bunker_mass = 1000 * i;
        m.bunker_volume = -1000 * i;
        m.def_working_speed = 10 * i;
        m.max_speed_empty = 100 * i;
        m.max_speed_full = 1 * i;
        m.turning_radius = i;
        m.weight = m.length = m.width = 10 * i;
        m.unloading_speed_mass = m.unloading_speed_volume = -10 * i;

    }
    return ret;
}

std::vector<Route> getRoutes( const std::vector<Machine>& machines ){
    const Point refPoint(564300, 5762400);

    std::vector<Route> routes;
    routes.reserve( machines.size() );

    for(auto& m : machines){
        routes.push_back( Route() );
        auto& r = routes.back();

        r.machine_id = m.id;
        r.baseDateTime = "timestamp-" + std::to_string(m.id);
        r.route_points.resize(20 + routes.size());

        for(size_t i = 0 ; i < r.route_points.size() ; ++i){
            auto& rp = r.route_points.at(i);
            try{
                rp.type = RoutePoint::intToRoutePointType(i);
            }
            catch(...){
                rp.type = RoutePoint::DEFAULT;
            }
            rp.point() = refPoint + Point(i, i);
            rp.time_stamp = 100 * m.id + i;
            rp.bunker_mass = 10 * i;
            rp.bunker_volume = -10 * i;
            rp.worked_mass = 100 * i;
            rp.worked_volume = -100 * i;
            rp.track_id = 1000 * m.id + i;
        }
    }
    return routes;
}

bool areEqual(std::istream &s1, std::istream &s2)
{
    char c1, c2;

    int p1 = s1.tellg();
    int p2 = s2.tellg();

    s1.seekg(0, s1.end);
    s2.seekg(0, s2.end);

    int l1 = s1.tellg();
    int l2 = s2.tellg();

    s1.seekg(0, s1.beg);
    s2.seekg(0, s2.beg);

    bool ok = true;

    if(l1 != l2)
        ok = false;
    else if(l1==0)
        ok = (l1==l2);
    else{
        while(ok){
            bool ok1 = false, ok2 = false;
            if(s1.get(c1))
                ok1 = true;
            if(s2.get(c2))
                ok2 = true;
            ok = ok1==ok2;
            if(!ok || !ok1)
                break;
            ok = (c1==c2);
        }
    }

    s1.seekg(p1, s1.beg);
    s2.seekg(p2, s2.beg);

    return ok;
}

bool areFilesEqual(const std::string &filename1, const std::string &filename2)
{
//    try{
//        std::ifstream f1(filename1);
//        if(!f1.is_open())
//            return false;
//        std::ifstream f2(filename2);
//        if(!f2.is_open()){
//            f1.close();
//            return false;
//        }
//        auto ok = areEqual(f1, f2);
//        f1.close();
//        f2.close();
//        return ok;
//    }
//    catch(...){
//        return false;
//    }

    //--
    std::string cmd = "cmp " + filename1 + " " + filename2;
    int r = system(cmd.c_str());
    return r == 0;
}

ArolibGrid_t getGridmap(double cellsize, size_t _size, double start, float val0 ){
    ArolibGrid_t g;
    g.createGrid(start, start + cellsize * _size, start, start + cellsize * _size, cellsize, 0.0);
    for(size_t x = 0 ; x < g.getSizeX() ; ++x){
        for(size_t y = 0 ; y < g.getSizeY() ; ++y){
            if( (x+y)%2 == 0 )
                g.setNoValue(x, y);
            else
                g.setValue(x, y, val0 * (x + y) );
        }
    }
    return g;
}

std::map<MachineId_t, MachineDynamicInfo> getMachineStates(const Subfield& sf, const std::vector<Machine>& machines){
    std::map<MachineId_t, MachineDynamicInfo> mdi;
    size_t fapInd = 0;
    for(auto& m : machines){
        auto& info = mdi[m.id];
        info.bunkerMass = 0.5 * m.bunker_mass;
        info.bunkerVolume = 0.5 * m.bunker_volume;
        if(fapInd >= sf.access_points.size())
            fapInd = 0;
        info.position = sf.access_points.at(fapInd);
        ++fapInd;
    }
    return mdi;
}

OutFieldInfo getOutFieldInfo(const Subfield& sf, const std::map<MachineId_t, MachineDynamicInfo>& mdi){
    OutFieldInfo ofi;
    for(auto& fap : sf.access_points){
        for(auto& fap2 : sf.access_points){
            OutFieldInfo::TravelData2 td;
            td.fap_id_from = fap.id;
            td.fap_id_to = fap2.id;
            td.machineId = OutFieldInfo::AllMachines;
            td.machineBunkerState = OutFieldInfo::ALL_MACHINE_STATES;
            td.travelCosts.distance = geometry::calc_dist(fap, fap2);
            td.travelCosts.time = 100*td.travelCosts.distance;
            ofi.add_FAP2FAP(td, true);
        }
        for(auto& rp : sf.resource_points){
            OutFieldInfo::TravelData td;
            td.fieldAccessPointId = fap.id;
            td.resourcePointId = rp.id;
            td.machineId = OutFieldInfo::AllMachines;
            td.machineBunkerState = OutFieldInfo::ALL_MACHINE_STATES;
            td.travelCosts.distance = geometry::calc_dist(fap, rp);
            td.travelCosts.time = 100*td.travelCosts.distance;
            ofi.add_FAP2RP(td);
            ofi.add_RP2FAP(td);
        }
        for(auto& it_m : mdi){
            OutFieldInfo::ArrivalData td;
            td.machineId = it_m.first;
            td.fieldAccessPointId = fap.id;
            td.machineBunkerState = OutFieldInfo::ALL_MACHINE_STATES;
            td.arrivalCosts.distance = geometry::calc_dist(fap, it_m.second.position);
            td.arrivalCosts.time = 100*td.arrivalCosts.distance;
            ofi.add_arrivalCosts(td);
        }
    }

    for(auto& rp : sf.resource_points){
        for(auto& it_m : mdi){
            OutFieldInfo::UnloadingData ud;
            ud.resourcePointId = rp.id;
            ud.machineId = OutFieldInfo::AllMachines;
            ud.unloadingCosts.time = 1000 + rp.id;
            ofi.add_unloadingCosts(ud);
        }
    }

    return ofi;
}

std::map<ResourcePointId_t, ResourcePointState> getResourcePointStates(const Subfield& sf){
    std::map<ResourcePointId_t, ResourcePointState> states;
    for(auto& rp : sf.resource_points){
        auto& state = states[rp.id];
        state.capacityMass = 0.5 * rp.massCapacity;
        state.capacityVolume = 0.5 * rp.volumeCapacity;
        state.enabled = (rp.id%2 == 0);
        state.timestamp = 10 * rp.id;
    }
    return states;
}

}

BOOST_AUTO_TEST_SUITE(test_io_xml)
BOOST_AUTO_TEST_CASE(test_io_xml_1)
{
    std::cout << "Running test io_xml..." << std::endl;

    bool xmlok = true;
    std::vector<std::string> parentTags = {"Tag_1","Tag_2","Tag_3"};
    auto configParameters = getParametersAsStringMap();
    auto field = getTestField();
    auto machines = getMachines();
    auto routes = getRoutes(machines);

    auto field2 = field;
    geometry::rotateField(field, M_PI_2);

    io::AroXMLOutDocument xmldoc;
    std::string extraVal = "extra value";

    auto filename_multivalues = (getOutputDir() / "xmltest_multivalues").string();

    std::cout << "Creating file xmltest_multivalues..." << std::endl;

    BOOST_REQUIRE( xmldoc.openFile( filename_multivalues + ".xml" ) );
    BOOST_REQUIRE( xmldoc.openDocument() );
    for(auto &tag : parentTags)
        BOOST_REQUIRE( xmldoc.openTag(tag) );

    xmlok = true;
    xmlok &= xmldoc.add( std::make_pair( &machines, std::string("DaWorkingGroup") ),
                         std::make_pair( &routes , std::string("DaRoutes") ),
                         std::make_pair( &extraVal, std::string("DaExtra") ),
                         std::make_pair( std::string("Extra value 2"), std::string("DaOdaExtra") ),
                         std::make_pair( "Extra value 2.2", std::string("DaOdaExtra") ),
                         std::make_pair( &field, std::string("DaVirginField") ),
                         std::make_pair( &configParameters, std::string("DaConfig") ) );
    xmlok &= xmldoc.add("Extra value 3", "DaOdaOdaExtra");
    xmlok &= xmldoc.add(field2, XMLOutDocument::UseDefaultTag);

    BOOST_REQUIRE( xmldoc.closeFile() );

    BOOST_REQUIRE( xmlok );

    std::cout << "File xmltest_multivalues created." << std::endl;


    std::cout << "Reading file xmltest_multivalues..." << std::endl;

    io::AroXMLInDocument xmldoc2;
    BOOST_REQUIRE( xmldoc2.openFile( filename_multivalues + ".xml" ) );
    BOOST_REQUIRE( xmldoc2.openDocument() );

    std::vector<Route> IN_routes;
    std::string IN_extraVal, IN_extraVal2, IN_extraVal3;
    std::vector<std::string> IN_extraValues;
    Field IN_field0, IN_field;
    std::vector<Machine> IN_machines;
    OutFieldInfo IN_outfiedInfo;
    std::map<MachineId_t, MachineDynamicInfo> IN_dynamicMachinesInfo;
    std::map<ResourcePointId_t, ResourcePointState> IN_resourcePointStates;
    auto IN_configParameters = configParameters; IN_configParameters.clear();
    size_t multCount = 0;

    xmlok = true;
    xmlok &= xmldoc2.read( IN_extraVal, "DaExtra", parentTags );
    xmlok &= xmldoc2.read( IN_extraVal2, "DaOdaExtra", parentTags );
    xmlok &= xmldoc2.read( IN_extraVal3, "DaOdaOdaExtra", parentTags );
    xmlok &= xmldoc2.read( IN_field0, "DaVirginField", parentTags );
    xmlok &= xmldoc2.read( IN_field, XMLInDocument::UseDefaultTag, parentTags );
    xmlok &= xmldoc2.read( IN_configParameters, "DaConfig", parentTags );

    multCount = 0;
    auto parentTags_machines = parentTags;
    parentTags_machines.push_back("DaWorkingGroup");
    xmlok &= xmldoc2.getMultiBranchHandlers(XMLInDocument::getTag<Machine>(),
                                           [&](const XMLInDocument::ReadHandler& rh) {
                                               IN_machines.emplace_back( Machine() );
                                               if( !xmldoc2.read(rh, IN_machines.back() ) ){
                                                   std::cout << "Machines: Error reading 'machine' of element #" << IN_machines.size() << std::endl;
                                                   IN_machines.pop_back();
                                                   return false;
                                               }
                                               std::cout << "Machine # " << IN_machines.size() << " added" << std::endl;
                                               return true;
                                           },
                                           parentTags_machines );

    auto parentTags_routes = parentTags;
    parentTags_routes.push_back("DaRoutes");
//    multCount = 0;
//    xmlok &= xmldoc2.getMultiBranchHandlers(XMLInDocument::getTag<Route>(),
//                                           [&](const XMLInDocument::ReadHandler& rh) {
//                                               ++multCount;
//                                               IN_routes.push_back(Route());
//                                               if( !xmldoc2.read(rh, IN_routes.back() ) ){
//                                                   std::cout << "Route: Error reading 'routes' of element #" << multCount << std::endl;
//                                                   IN_routes.pop_back();
//                                                   return true;
//                                               }
//                                               return true;
//                                           },
//                                           parentTags_routes);
    xmlok &= xmldoc2.readMultiple(IN_routes, XMLInDocument::UseDefaultTag, parentTags_routes, true);

//    multCount = 0;
//    xmlok &= xmldoc2.getMultiBranchHandlers("DaOdaExtra",
//                                           [&](const XMLInDocument::ReadHandler& rh) {
//                                               ++multCount;
//                                               IN_extraValues.push_back(std::string());
//                                               if( !xmldoc2.read(rh, IN_extraValues.back() ) ){
//                                                   std::cout << "DaOdaExtra: Error reading 'routes' of element #" << multCount << std::endl;
//                                                   IN_extraValues.pop_back();
//                                                   return true;
//                                               }
//                                               return true;
//                                           },
//                                           parentTags);
    xmlok &= xmldoc2.readMultiple(IN_extraValues, "DaOdaExtra", parentTags, true);

    BOOST_REQUIRE( xmldoc2.closeFile() );

    BOOST_REQUIRE( xmlok );

    std::cout << "File xmltest_multivalues read." << std::endl;


    std::cout << "Creating file xmltest_multivalues (read)..." << std::endl;

    BOOST_REQUIRE( xmldoc.openFile( filename_multivalues + "_read.xml" ) );
    BOOST_REQUIRE( xmldoc.openDocument() );
    for(auto &tag : parentTags)
        BOOST_REQUIRE( xmldoc.openTag(tag) );

    xmlok = true;
    xmlok &= xmldoc.add( std::make_pair( &IN_machines, std::string("DaWorkingGroup") ),
                         std::make_pair( &IN_routes, std::string("DaRoutes") ),
                         std::make_pair( &IN_extraVal, std::string("DaExtra") ),
                         std::make_pair( &IN_extraValues, std::string("DaOdaExtra") ),
                         std::make_pair( &IN_field0, std::string("DaVirginField") ),
                         std::make_pair( &IN_configParameters, std::string("DaConfig") ) );
    xmlok &= xmldoc.add(IN_extraVal3, "DaOdaOdaExtra");
    xmlok &= xmldoc.add(IN_field, io::XMLOutDocument::UseDefaultTag);

    BOOST_REQUIRE( xmldoc.closeFile() );

    BOOST_REQUIRE( xmlok );

    std::cout << "File xmltest_multivalues (read) created." << std::endl;

    std::cout << "Comparing original and read xmltest_multivalues files..." << std::endl;

    BOOST_CHECK( areFilesEqual(filename_multivalues + ".xml", filename_multivalues + "_read.xml") );

    xmlok = true;

    auto filename_plan = (getOutputDir() / "xmltest_plan").string();
    std::vector< std::vector<Route> > xmlplan;
    xmlplan.push_back( routes );
    std::map<int, std::vector<Route> > IN_xmlplan;

    std::cout << "Writing/Reading plan 1..." << std::endl;
    BOOST_REQUIRE( AroXMLOutDocument::savePlan(filename_plan + "1.xml", xmlplan) );
    BOOST_REQUIRE( AroXMLInDocument::readPlan(filename_plan + "1.xml", IN_xmlplan) );
    BOOST_REQUIRE( AroXMLOutDocument::savePlan(filename_plan + "1_read.xml", IN_xmlplan) );

    std::cout << "Comparing plan 1 files..." << std::endl;
    BOOST_CHECK( areFilesEqual(filename_plan + "1.xml", filename_plan + "1_read.xml") );

    IN_xmlplan.clear();
    std::map<std::string, ArolibGrid_t> IN_gridmaps;

    std::cout << "Writing/Reading plan 2..." << std::endl;
    BOOST_REQUIRE( AroXMLOutDocument::savePlan(filename_plan + "2.xml", field2, machines, xmlplan) );
    BOOST_REQUIRE( AroXMLInDocument::readPlan(filename_plan + "2.xml", IN_field, IN_machines, IN_xmlplan, IN_gridmaps) );
    BOOST_REQUIRE( AroXMLOutDocument::savePlan(filename_plan + "2_read.xml", IN_field, IN_machines, IN_xmlplan) );

    std::cout << "Comparing plan 2 files..." << std::endl;
    BOOST_CHECK( areFilesEqual(filename_plan + "2.xml", filename_plan + "2_read.xml") );

    std::map<std::string, const ArolibGrid_t*> gridmaps;
    auto gm1 = getGridmap(1, 10, 0, 1);
    auto gm2 = getGridmap(2, 20, 20, 2);
    auto gm3 = getGridmap(2, 30, 300, 3);
    gridmaps.insert( std::make_pair("GM1", &gm1) );
    gridmaps.insert( std::make_pair("GM2", &gm2) );
    gridmaps.insert( std::make_pair("GM3", &gm3) );

    auto filename_planParams = (getOutputDir() / "xmltest_planParameters").string();

    auto dynamicMachinesInfo = getMachineStates(field.subfields.front(), machines);
    auto outfiedInfo = getOutFieldInfo(field.subfields.front(), dynamicMachinesInfo);
    auto resourcePointStates = getResourcePointStates(field.subfields.front());

    std::cout << "Writing/Reading plan parameters..." << std::endl;
    BOOST_REQUIRE( io::AroXMLOutDocument::savePlanParameters(filename_planParams + ".xml",
                                                             machines,
                                                             configParameters,
                                                             outfiedInfo,
                                                             dynamicMachinesInfo,
                                                             resourcePointStates,
                                                             gridmaps) );
    BOOST_REQUIRE( io::AroXMLInDocument::readPlanParameters(filename_planParams + ".xml",
                                                            IN_machines,
                                                            IN_configParameters,
                                                            IN_outfiedInfo,
                                                            IN_dynamicMachinesInfo,
                                                            IN_resourcePointStates,
                                                            IN_gridmaps) );

    gridmaps.clear();
    for(auto& it : IN_gridmaps)
        gridmaps[it.first] = &it.second;
    BOOST_REQUIRE( io::AroXMLOutDocument::savePlanParameters(filename_planParams + "_read.xml",
                                                             IN_machines,
                                                             IN_configParameters,
                                                             IN_outfiedInfo,
                                                             IN_dynamicMachinesInfo,
                                                             IN_resourcePointStates,
                                                             gridmaps) );

    std::cout << "Comparing original and read plan parameters..." << std::endl;
    BOOST_CHECK( areFilesEqual(filename_planParams + ".xml", filename_planParams + "_read.xml") );

    std::vector<Field> IN_fields;

    auto filename_fields = (getOutputDir() / "xmltest_fields").string();

    std::cout << "Writing/Reading fields..." << std::endl;
    BOOST_REQUIRE( AroXMLOutDocument::saveFields(filename_fields + ".xml", {field, field2}) );
    BOOST_REQUIRE( AroXMLInDocument::readFields(filename_fields + ".xml", IN_fields) );
    BOOST_REQUIRE( AroXMLOutDocument::saveFields(filename_fields + "_read.xml", IN_fields) );

    std::cout << "Comparing original and read fields..." << std::endl;
    BOOST_CHECK( areFilesEqual(filename_fields + ".xml", filename_fields + "_read.xml") );

    std::cout << "Finished test io_xml" << std::endl;
}
BOOST_AUTO_TEST_SUITE_END()
