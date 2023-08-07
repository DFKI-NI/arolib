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
 
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <chrono>
#include <string>

#include "arolib/geometry/tracksgenerator.h"
#include "arolib/io/io_kml.hpp"

BOOST_AUTO_TEST_SUITE(test_tracksGenerator)

boost::filesystem::path getOutputDir()
{
    auto out_dir = boost::filesystem::temp_directory_path() / "arolib" / "test" / "test_tracksGenerator";
    boost::filesystem::create_directories(out_dir);
    return out_dir;
}

arolib::Field getTestField(bool convex = true)
{

    arolib::Point geoRef(9.93661676071641, 52.007229170305955), utmRef;
    arolib::CoordTransformer::GetInstance().convert_to_cartesian(geoRef, utmRef);

    arolib::Field f;
    f.subfields.emplace_back(arolib::Subfield());
    auto& sf = f.subfields.front();
    sf.reference_lines.emplace_back(arolib::Linestring());
    auto& boundary = sf.boundary_outer;
    auto& refLine = sf.reference_lines.front().points;

    boundary.points.clear();
    refLine.clear();

    boundary.points.push_back(utmRef);
    boundary.points.push_back(boundary.points.back() + arolib::Point(100, 20));
    boundary.points.push_back(boundary.points.back() + arolib::Point(0, -150));
    boundary.points.push_back(boundary.points.back() + arolib::Point(-90, 30));
    boundary.points.push_back(boundary.points.back() + arolib::Point(30, 40)); //inner spike -> i = 4
    boundary.points.push_back(boundary.points.back() + arolib::Point(-40, 20));
    boundary.points.push_back(boundary.points.front());

    if (convex)
    {
        boundary.points.erase(boundary.points.begin() + 4);
    }

    sf.boundary_inner = boundary;

    refLine.push_back(utmRef);
    refLine.push_back(utmRef + arolib::Point(0, -10));

    return f;
}

void test_generateTracks()
{

    for(size_t convex = 0 ; convex < 2 ; ++convex){
        arolib::Field field = getTestField(convex);
        std::vector<double> trackWidths = {1, 2, 3};
        std::vector<size_t> trackWidthsIndexes;

        arolib::geometry::TracksGenerator tg(arolib::LogLevel::ERROR);
        arolib::geometry::TracksGenerator::TracksGeneratorParameters tg_params;
        tg_params.checkForRemainingTracks = false;
        tg_params.onlyUntilBoundaryIntersection = false;
        tg_params.sampleResolution = 1;
        tg_params.shiftingStrategy = convex == 0 ? arolib::geometry::TracksGenerator::TRANSLATE_TRACKS : arolib::geometry::TracksGenerator::OFFSET_TRACKS;
        tg_params.trackAreaThreshold = 0.2;
        tg_params.trackDistance = 5;//should be disregarded
        tg_params.trackOrderStrategy = convex == 0 ? arolib::geometry::TracksGenerator::LONGEST_TRACK_FIRST : arolib::geometry::TracksGenerator::EXTRA_TRACK_LAST;
        tg_params.trackSamplingStrategy = convex == 0 ? arolib::geometry::TracksGenerator::START_AT_TRACK_START : arolib::geometry::TracksGenerator::MIN_DIST_BETWEEN_TRACKS;
        tg_params.useRefLineAsCentralLine = convex;

        auto& boundary = field.subfields.front().boundary_outer;
        auto& refLine = field.subfields.front().reference_lines.front().points;
        auto& tracks = field.subfields.front().tracks;

        auto aroResp = tg.generateTracks(boundary,
                                         refLine,
                                         trackWidths,
                                         tg_params,
                                         tracks,
                                         &trackWidthsIndexes);


        BOOST_TEST_INFO("Generating tracks for polygon " << convex << "...");
        BOOST_CHECK(aroResp.isOK());
        BOOST_TEST_INFO("Checking tracks and distance-indexes sizes...");
        BOOST_CHECK(tracks.size() == trackWidthsIndexes.size());
        if(tracks.size() == trackWidthsIndexes.size()){
            for(size_t i = 1 ; i + 1 < tracks.size() ; ++i){
                BOOST_TEST_INFO("Checking width of track " << i << "...");
                BOOST_CHECK(trackWidthsIndexes.at(i) < trackWidths.size());
                if( trackWidthsIndexes.at(i) < trackWidths.size() ){
                    BOOST_CHECK_CLOSE( trackWidths.at( trackWidthsIndexes.at(i) ), tracks.at(i).width, 0.1 );
                }
                BOOST_TEST_INFO("Checking points and boundary of track " << i << "...");
                BOOST_CHECK(tracks.at(i).points.size() > 1);
                BOOST_CHECK( arolib::geometry::isPolygonValid(tracks.at(i).boundary) != arolib::geometry::PolygonValidity::INVALID_POLYGON );
            }
        }

        std::string filename = "test_generateTracks_" + std::to_string(convex) + ".kml";
        arolib::io::writeFieldKML( ( getOutputDir() / filename.c_str() ).string(),
                                  field );
    }

    return;
}

BOOST_AUTO_TEST_CASE(test1)
{
    test_generateTracks();
}
BOOST_AUTO_TEST_SUITE_END()
