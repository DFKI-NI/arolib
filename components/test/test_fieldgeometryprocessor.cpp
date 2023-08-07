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
#include <iostream>

#include "arolib/components/fieldgeometryprocessor.h"

BOOST_AUTO_TEST_SUITE(test_fieldgeometryprocessor)

arolib::Subfield getTestSubfield()
{
    arolib::Subfield sf;
    sf. boundary_outer.points.push_back( arolib::Point( 564236.4, 5762421.3 ) );
    sf. boundary_outer.points.push_back( arolib::Point( 564384.8, 5762512 ) );
    sf. boundary_outer.points.push_back( arolib::Point( 564502.1, 5762482.9 ) );
    sf. boundary_outer.points.push_back( arolib::Point( 564549.5, 5762355.5 ) );
    sf. boundary_outer.points.push_back( arolib::Point( 564289.3, 5762256.8 ) );
    sf. boundary_outer.points.push_back(sf. boundary_outer.points.front());

    sf.reference_lines.push_back(arolib::Linestring());
    sf.reference_lines.front().points.push_back( arolib::Point( 564258.482601, 5762413.08968 ) );
    sf.reference_lines.front().points.push_back( arolib::Point( 564303.143312, 5762279.82116 ) );

    return sf;
}

void test_completeHL_()
{
    auto sf0 = getTestSubfield();

    arolib::FieldGeometryProcessor fgp;

    double trackWidth = 6;

    arolib::FieldGeometryProcessor::HeadlandParameters hl_params;
    hl_params.headlandWidth = 25;
    hl_params.trackWidth = trackWidth;
    hl_params.numTracks = 0;
    hl_params.sampleResolution = 10;

    arolib::FieldGeometryProcessor::InfieldParameters if_params;
    if_params.sampleResolution = 10;
    if_params.trackDistance = trackWidth;

    std::cout << std::endl << "Testing generation of complete headland with track width..." << std::endl << std::endl;

    auto sf = sf0;
    auto aroResp = fgp.processSubfieldWithSurroundingHeadland(sf,
                                                              hl_params,
                                                              if_params);

    BOOST_CHECK(!aroResp.isError());
    BOOST_CHECK(sf.boundary_inner.points.size() > 3);
    BOOST_CHECK(sf.headlands.complete.boundaries.first.points.size() > 3);
    BOOST_CHECK(sf.headlands.complete.boundaries.second.points.size() > 3);
    BOOST_CHECK(sf.headlands.complete.tracks.size() == std::ceil(hl_params.headlandWidth/trackWidth) );
    for(auto& track : sf.headlands.complete.tracks){
        BOOST_CHECK(track.points.size() > 3);
        BOOST_CHECK_CLOSE(track.width, trackWidth, 0.01);
    }

    BOOST_CHECK(sf.tracks.size() > 10);
    for(auto& track : sf.tracks){
        BOOST_CHECK(track.points.size() > 1);
        BOOST_CHECK_CLOSE(track.width, trackWidth, 0.01);
    }


    std::cout << std::endl << "Testing generation of complete headland with number of tracks..." << std::endl << std::endl;

    hl_params.trackWidth = -1;
    hl_params.numTracks = 5;

    sf = sf0;
    fgp.processSubfieldWithSurroundingHeadland(sf,
                                               hl_params,
                                               if_params);

    BOOST_CHECK(sf.boundary_inner.points.size() > 3);
    BOOST_CHECK(sf.headlands.complete.boundaries.first.points.size() > 3);
    BOOST_CHECK(sf.headlands.complete.boundaries.second.points.size() > 3);
    BOOST_CHECK(sf.headlands.complete.tracks.size() == hl_params.numTracks);
    for(auto& track : sf.headlands.complete.tracks){
        BOOST_CHECK(track.points.size() > 3);
        BOOST_CHECK_CLOSE(track.width, hl_params.headlandWidth/hl_params.numTracks, 0.01);
    }

    BOOST_CHECK(sf.tracks.size() > 10);
    for(auto& track : sf.tracks){
        BOOST_CHECK(track.points.size() > 1);
        BOOST_CHECK_CLOSE(track.width, trackWidth, 0.01);
    }


    std::cout << std::endl << "Testing generation of complete headland without tracks..." << std::endl << std::endl;

    hl_params.trackWidth = -1;
    hl_params.numTracks = 0;

    sf = sf0;
    fgp.processSubfieldWithSurroundingHeadland(sf,
                                               hl_params,
                                               if_params);

    BOOST_CHECK(sf.boundary_inner.points.size() > 3);
    BOOST_CHECK(sf.headlands.complete.boundaries.first.points.size() > 3);
    BOOST_CHECK(sf.headlands.complete.boundaries.second.points.size() > 3);
    BOOST_CHECK(sf.headlands.complete.tracks.empty());
    BOOST_CHECK(sf.headlands.complete.middle_track.points.size() > 3);

}

void test_partialHLs_(bool withConnectingHLs)
{
    auto sf0 = getTestSubfield();

    arolib::FieldGeometryProcessor fgp;

    double trackWidth = 6;

    arolib::FieldGeometryProcessor::HeadlandParameters hl_params;
    hl_params.headlandWidth = 25;
    hl_params.trackWidth = trackWidth;
    hl_params.numTracks = 0;
    hl_params.sampleResolution = 10;
    hl_params.headlandConnectionTrackWidth = withConnectingHLs ? -1 : 0;

    arolib::FieldGeometryProcessor::InfieldParameters if_params;
    if_params.sampleResolution = 10;
    if_params.trackDistance = trackWidth;

    std::cout << std::endl << "Testing generation of partial headlands " << (withConnectingHLs ? "with" : "without") << " connecting headlands with track width..." << std::endl << std::endl;

    auto sf = sf0;
    size_t countConnectingHLs = 0;
    double trackWithConnHL = hl_params.headlandConnectionTrackWidth < 0 ? trackWidth : hl_params.headlandConnectionTrackWidth;
    auto aroResp = fgp.processSubfieldWithSideHeadlands(sf,
                                                        hl_params,
                                                        if_params);

    BOOST_CHECK(!aroResp.isError());
    BOOST_CHECK(sf.boundary_inner.points.size() > 3);
    BOOST_CHECK(sf.headlands.partial.size() == (withConnectingHLs ? 4 : 2) );
    for(auto& hl : sf.headlands.partial){
        BOOST_CHECK(hl.boundary.points.size() > 3);
        double expectedTrackWidth = trackWidth;
        if(hl.isConnectingHeadland()){
            BOOST_CHECK(hl.tracks.size() == 1);
            expectedTrackWidth = trackWithConnHL;
            ++countConnectingHLs;
        }
        else
            BOOST_CHECK(hl.tracks.size() == 5);

        for(auto& track : hl.tracks){
            BOOST_CHECK(track.points.size() > 1);
            BOOST_CHECK_CLOSE(track.width, expectedTrackWidth, 0.01);
        }
    }
    if(withConnectingHLs){
        BOOST_CHECK(countConnectingHLs == 2);
        BOOST_CHECK(arolib::PartialHeadland::getHeadlandConnectionSequences(sf.headlands.partial, 0, 0).size() > 1);
    }
    else{
        BOOST_CHECK(countConnectingHLs == 0);
        BOOST_CHECK(arolib::PartialHeadland::getHeadlandConnectionSequences(sf.headlands.partial, 0, 0).size() == 1);
    }

    BOOST_CHECK(sf.tracks.size() > 10);
    for(auto& track : sf.tracks){
        BOOST_CHECK(track.points.size() > 1);
        BOOST_CHECK_CLOSE(track.width, trackWidth, 0.01);
    }


    std::cout << std::endl << "Testing generation of partial headlands " << (withConnectingHLs ? "with" : "without") << " connecting headlands with number of tracks..." << std::endl << std::endl;

    hl_params.trackWidth = -1;
    hl_params.numTracks = 5;

    sf = sf0;
    countConnectingHLs = 0;
    trackWithConnHL = hl_params.headlandConnectionTrackWidth < 0 ? hl_params.headlandWidth/hl_params.numTracks : hl_params.headlandConnectionTrackWidth;
    aroResp = fgp.processSubfieldWithSideHeadlands(sf,
                                                   hl_params,
                                                   if_params);

    BOOST_CHECK(!aroResp.isError());
    BOOST_CHECK(sf.boundary_inner.points.size() > 3);
    BOOST_CHECK(sf.headlands.partial.size() == (withConnectingHLs ? 4 : 2) );
    for(auto& hl : sf.headlands.partial){
        BOOST_CHECK(hl.boundary.points.size() > 3);
        double expectedTrackWidth = hl_params.headlandWidth/hl_params.numTracks;
        if(hl.isConnectingHeadland()){
            BOOST_CHECK(hl.tracks.size() == 1);
            expectedTrackWidth = trackWithConnHL;
            ++countConnectingHLs;
        }
        else
            BOOST_CHECK(hl.tracks.size() == hl_params.numTracks);

        for(auto& track : hl.tracks){
            BOOST_CHECK(track.points.size() > 1);
            BOOST_CHECK_CLOSE(track.width, expectedTrackWidth, 0.01);
        }
    }
    if(withConnectingHLs){
        BOOST_CHECK(countConnectingHLs == 2);
        BOOST_CHECK(arolib::PartialHeadland::getHeadlandConnectionSequences(sf.headlands.partial, 0, 0).size() > 1);
    }
    else{
        BOOST_CHECK(countConnectingHLs == 0);
        BOOST_CHECK(arolib::PartialHeadland::getHeadlandConnectionSequences(sf.headlands.partial, 0, 0).size() == 1);
    }

    BOOST_CHECK(sf.tracks.size() > 10);
    for(auto& track : sf.tracks){
        BOOST_CHECK(track.points.size() > 1);
        BOOST_CHECK_CLOSE(track.width, trackWidth, 0.01);
    }


    std::cout << std::endl << "Testing generation of partial headlands " << (withConnectingHLs ? "with" : "without") << " connecting headlands without tracks..." << std::endl << std::endl;

    hl_params.trackWidth = -1;
    hl_params.numTracks = 0;

    if(withConnectingHLs)
        hl_params.headlandConnectionTrackWidth = 6;//the main headlands will have no tracks, but the connecting headlands will have 1 track

    sf = sf0;
    countConnectingHLs = 0;
    aroResp = fgp.processSubfieldWithSideHeadlands(sf,
                                                   hl_params,
                                                   if_params);

    BOOST_CHECK(!aroResp.isError());
    BOOST_CHECK(sf.boundary_inner.points.size() > 3);
    BOOST_CHECK(sf.headlands.partial.size() == (withConnectingHLs ? 4 : 2) );
    for(auto& hl : sf.headlands.partial){
        BOOST_CHECK(hl.boundary.points.size() > 3);
        if(hl.isConnectingHeadland()){
            BOOST_CHECK(hl.tracks.size() == 1);
            ++countConnectingHLs;
        }
        else
            BOOST_CHECK(hl.tracks.empty());

        for(auto& track : hl.tracks){
            BOOST_CHECK(track.points.size() > 1);
            BOOST_CHECK_CLOSE(track.width, hl_params.headlandConnectionTrackWidth, 0.01);
        }
    }
    if(withConnectingHLs){
        BOOST_CHECK(countConnectingHLs == 2);
        BOOST_CHECK(arolib::PartialHeadland::getHeadlandConnectionSequences(sf.headlands.partial, 0, 0).size() > 1);
    }
    else{
        BOOST_CHECK(countConnectingHLs == 0);
        BOOST_CHECK(arolib::PartialHeadland::getHeadlandConnectionSequences(sf.headlands.partial, 0, 0).size() == 1);
    }

    BOOST_CHECK(sf.tracks.size() > 10);
    for(auto& track : sf.tracks){
        BOOST_CHECK(track.points.size() > 1);
        BOOST_CHECK_CLOSE(track.width, trackWidth, 0.01);
    }

}

BOOST_AUTO_TEST_CASE(test_completeHL){ test_completeHL_(); }
BOOST_AUTO_TEST_CASE(test_partialHLs_withConnectingHeadlands){ test_partialHLs_(true); }
BOOST_AUTO_TEST_CASE(test_partialHLs_withoutConnectingHeadlands){ test_partialHLs_(false); }

BOOST_AUTO_TEST_SUITE_END()
