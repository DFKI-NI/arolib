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

#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/tracksgenerator.h"
#include "arolib/io/io_xml.hpp"
#include "arolib/io/io_kml.hpp"
#include "arolib/io/io_hdf5.hpp"

#include "test_io_common.cpp"

//namespace  {

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
//        sf.obstacles.emplace_back( arolib::geometry::create_circle(center + arolib::Point(5, 0), 2, 10, 0.0, -2*M_PI) );

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

//}

BOOST_AUTO_TEST_SUITE(test_io_field)
BOOST_AUTO_TEST_CASE(test_io_field_single)
{
    auto field = getTestField();

    auto out_dir = boost::filesystem::temp_directory_path() / "arolib" / "test" / "test_io_field_single";
    boost::filesystem::create_directories(out_dir);

    for(int i = 0 ; i < 3 ; ++i){

        std::string filename;
        bool okRead, okWrite;
        arolib::Field fieldIn;
        if(i == 0){
            filename = "field.xml";
            okWrite = arolib::io::writeFieldXML( (out_dir / filename).string(), field );
            if(okWrite)
                okRead = arolib::io::readFieldXML( (out_dir / filename).string(), fieldIn );
        }
        else if(i == 1){
            filename = "field.kml";
            okWrite = arolib::io::writeFieldKML( (out_dir / filename).string(), field );
            if(okWrite)
                okRead = arolib::io::readFieldKML( (out_dir / filename).string(), fieldIn );

            //debug!
            okWrite = arolib::io::writeFieldKML( (out_dir / "fieldIn.kml").string(), fieldIn );
        }
        else{
            filename = "field.h5";
            okWrite = arolib::io::write_field_hdf5( (out_dir / filename).string(), field.name, field );
            if(okWrite)
                okRead = arolib::io::read_field_hdf5( (out_dir / filename).string(), field.name, fieldIn );
        }

        std::cout << "--- Writing and reading file " << filename << " ---" << std::endl;
        BOOST_TEST(okWrite);
        if(!okWrite)
            continue;

        BOOST_TEST(okRead);
        if(!okRead)
            continue;

        BOOST_CHECK_EQUAL(field.name, fieldIn.name);
        BOOST_CHECK_EQUAL(field.id, fieldIn.id);
        BOOST_CHECK_EQUAL(field.external_roads.size(), fieldIn.external_roads.size());
        if(field.external_roads.size() == fieldIn.external_roads.size()){
            for(size_t j = 0 ; j < field.external_roads.size() ; ++j)
                BOOST_TEST( field.external_roads.at(j) == fieldIn.external_roads.at(j) );
        }

        BOOST_CHECK_EQUAL(field.subfields.size(), fieldIn.subfields.size());
        if(field.subfields.size() != fieldIn.subfields.size())
            continue;

        for(size_t j = 0 ; j < field.subfields.size() ; ++j){
            auto& sf = field.subfields.at(j);
            auto& sfIn = fieldIn.subfields.at(j);

            BOOST_CHECK_EQUAL(sf.id, sfIn.id);
            BOOST_TEST( sf.boundary_outer == sfIn.boundary_outer );
            BOOST_TEST( sf.boundary_inner == sfIn.boundary_inner );

            BOOST_CHECK_EQUAL( sf.reference_lines.size(), sfIn.reference_lines.size() );
            if(sf.reference_lines.size() == sfIn.reference_lines.size()){
                for(size_t k = 0 ; k < sf.reference_lines.size() ; ++k)
                    BOOST_TEST( sf.reference_lines.at(k) == sfIn.reference_lines.at(k) );

            }

            BOOST_CHECK_EQUAL( sf.access_points.size(), sfIn.access_points.size() );
            if(sf.access_points.size() == sfIn.access_points.size()){
                for(size_t k = 0 ; k < sf.access_points.size() ; ++k)
                    BOOST_TEST( sf.access_points.at(k) == sfIn.access_points.at(k) );

            }

            BOOST_CHECK_EQUAL( sf.resource_points.size(), sfIn.resource_points.size() );
            if(sf.resource_points.size() == sfIn.resource_points.size()){
                for(size_t k = 0 ; k < sf.resource_points.size() ; ++k)
                    BOOST_TEST( sf.resource_points.at(k) == sfIn.resource_points.at(k) );

            }

            BOOST_CHECK_EQUAL( sf.obstacles.size(), sfIn.obstacles.size() );
            if(sf.obstacles.size() == sfIn.obstacles.size()){
                for(size_t k = 0 ; k < sf.obstacles.size() ; ++k)
                    BOOST_TEST( sf.obstacles.at(k) == sfIn.obstacles.at(k) );

            }

            BOOST_CHECK_EQUAL( sf.headlands.partial.size(), sfIn.headlands.partial.size() );
            BOOST_CHECK_CLOSE( sf.headlands.complete.headlandWidth, sfIn.headlands.complete.headlandWidth, 0.001 );

            BOOST_CHECK_EQUAL( sf.headlands.complete.tracks.size(), sfIn.headlands.complete.tracks.size() );
            if(sf.headlands.complete.tracks.size() == sfIn.headlands.complete.tracks.size()){
                for(size_t k = 0 ; k < sf.headlands.complete.tracks.size() ; ++k)
                    BOOST_TEST( sf.headlands.complete.tracks.at(k) == sfIn.headlands.complete.tracks.at(k) );

            }


            BOOST_CHECK_EQUAL( sf.tracks.size(), sfIn.tracks.size() );
            if(sf.tracks.size() == sfIn.tracks.size()){
                for(size_t k = 0 ; k < sf.tracks.size() ; ++k)
                    BOOST_TEST( sf.tracks.at(k) == sfIn.tracks.at(k) );

            }

        }

    }

}
BOOST_AUTO_TEST_SUITE_END()
