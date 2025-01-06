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

#include "arolib/types/route.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/geometry/tracksgenerator.h"
#include "arolib/planning/track_connectors/infieldtracksconnectordef.hpp"
#include "arolib/planning/track_sequencing/tracksequencerclosestnext.hpp"
#include "arolib/planning/track_sequencing/simulated_annealing/tracksequencersimannealing.hpp"
#include "arolib/planning/track_sequencing/simulated_annealing/stategenerator_swaptracks.hpp"
#include "arolib/io/io_xml.hpp"


using namespace arolib;

Subfield getTestSubfield(double workingWidth){
    Subfield sf;
    const Point ptRef(564300, 5762428);

    sf.boundary_outer.points = {
        ptRef,
        ptRef + Point(0, 50 * workingWidth),
        ptRef + Point(20 * workingWidth, 60 * workingWidth),
        ptRef + Point(50 * workingWidth, 50 * workingWidth),
        ptRef + Point(50 * workingWidth, 0),
        ptRef
    };

    for(int i = 1 ; i < 5 ; ++i){
        Polygon polyTmp;
        geometry::offsetPolygon(sf.boundary_outer, polyTmp, workingWidth * i, false);
        sf.headlands.complete.tracks.emplace_back(Track());
        sf.headlands.complete.tracks.back().points = std::move(polyTmp.points);
    }
    geometry::offsetPolygon(sf.boundary_outer, sf.boundary_inner, workingWidth * 5, false);

    geometry::TracksGenerator tg;
    geometry::TracksGenerator::TracksGeneratorParameters tgParams;
    tgParams.trackDistance = workingWidth;
    tg.generateTracks(sf.boundary_inner,
                      {ptRef, ptRef + Point(0, 50 * workingWidth)},
                      tgParams,
                      sf.tracks);

    return sf;
}

bool checkTrackIndexesInRange(const Subfield& sf, const std::vector<ITrackSequencer::TrackInfo>& sequences){
    for(auto& ti : sequences){
        if(ti.trackIndex >= sf.tracks.size())
            return false;
    }
    return true;
}

bool checkTracksVisits(const Subfield& sf, const std::vector<ITrackSequencer::TrackInfo>& sequences){
    std::set<size_t> visited;
    bool ok = true;
    for(size_t i = 0 ; i < sequences.size() ; ++i){
        if(visited.find(sequences.at(i).trackIndex) != visited.end()){
            std::cerr << "ERROR: track ind " << sequences.at(i).trackIndex << " is repeated" << std::endl;
            ok = false;
            continue;
        }
        visited.insert(sequences.at(i).trackIndex);
    }
    if(visited.size() != sf.tracks.size()){
        std::cerr << "ERROR: Not all tracks were visited" << std::endl;
        ok = false;
    }
    return ok;
}

Pose2D getEntryExitPose (const Subfield& subfield, size_t trackIndex, bool trackInPointsDirection, bool entry){
    if(entry)
        return Pose2D ( trackInPointsDirection ? subfield.tracks.at(trackIndex).points.front() : subfield.tracks.at(trackIndex).points.back() ,
                        trackInPointsDirection ? subfield.tracks.at(trackIndex).points.at(1) : r_at(subfield.tracks.at(trackIndex).points, 1) );
    auto& p = !trackInPointsDirection ? subfield.tracks.at(trackIndex).points.front() : subfield.tracks.at(trackIndex).points.back();
    return Pose2D ( p,
                    geometry::get_angle( !trackInPointsDirection ? subfield.tracks.at(trackIndex).points.at(1) : r_at(subfield.tracks.at(trackIndex).points, 1), p ) );
}

double getConnectionLength(const Subfield& sf, const std::vector<ITrackSequencer::TrackInfo>& sequences, const Machine& machine, ITrackSequencer& sequencer, IInfieldTracksConnector& connector){
    double length = 0;
    bool ok = true;

    for(size_t i = 0 ; i+1 < sequences.size() ; ++i){
        auto& ti = sequences.at(i);
        auto& tiNext = sequences.at(i+1);

        auto poseStart = getEntryExitPose( sf, ti.trackIndex, ti.trackPointsDirection == ITrackSequencer::TrackPointsDirection::FORWARD, false );
        auto poseEnd = getEntryExitPose( sf, tiNext.trackIndex, tiNext.trackPointsDirection == ITrackSequencer::TrackPointsDirection::FORWARD, true );

        auto path = sequencer.getPathsMapManager()->getPathFromMap(poseStart, poseEnd, machine.turning_radius, true);

        if(path.empty()){
            std::cerr << "Error retrieving path from sequencer: i =" << i << std::endl;
            //return std::nan("1");
            ok = false;
            continue;
        }

//        path = connector.getConnection( sf, machine,
//                                        poseStart, poseEnd,
//                                        machine.turning_radius,
//                                        std::make_pair(0.0, 0.0) );

        length += geometry::getGeometryLength(path);
    }
    return ok ? length : std::nan("1");
}

void savePath(const std::string& posfix, const Subfield& sf, const std::vector<ITrackSequencer::TrackInfo>& sequences, const Machine& machine, ITrackSequencer& sequencer){

    std::vector<std::vector<Route>> routes(1);
    routes.back().emplace_back(Route());
    auto& route = routes.back().back();

    auto addPath = [&](const std::vector<Point> & _pts, int track_id, bool fwd, bool excludeEnds){
        auto pts = geometry::sample_geometry(_pts, machine.turning_radius);
        for(size_t j = excludeEnds ; j+excludeEnds < pts.size(); ++j){
            const auto& p = fwd ? pts.at(j) : r_at(pts, j);
            route.route_points.emplace_back(RoutePoint(p));
            auto& rp = route.route_points.back();
            if(route.route_points.size() == 1)
                rp.time_stamp = 0;
            else{
                auto& rpPrev = r_at(route.route_points, 1);
                auto d = geometry::calc_dist(rp, rpPrev);
                rp.time_stamp = rpPrev.time_stamp + d / machine.def_working_speed;
            }
            rp.track_id = track_id;
            rp.type = track_id < 0 ? RoutePoint::TRANSIT : RoutePoint::DEFAULT;
        }

    };

    for(size_t i = 0 ; i+1 < sequences.size() ; ++i){
        auto& ti = sequences.at(i);
        auto& tiNext = sequences.at(i+1);

        auto poseStart = getEntryExitPose( sf, ti.trackIndex, ti.trackPointsDirection == ITrackSequencer::TrackPointsDirection::FORWARD, false );
        auto poseEnd = getEntryExitPose( sf, tiNext.trackIndex, tiNext.trackPointsDirection == ITrackSequencer::TrackPointsDirection::FORWARD, true );

        auto path = sequencer.getPathsMapManager()->getPathFromMap(poseStart, poseEnd, machine.turning_radius, true);

        if(path.empty()){
            std::cerr << "Error retrieving path from sequencer: i =" << i << std::endl;
            return;
        }

        addPath(sf.tracks.at(ti.trackIndex).points, sf.tracks.at(ti.trackIndex).id, ti.trackPointsDirection == ITrackSequencer::FORWARD, false);
        addPath(path, -1, true, true);
    }
    addPath(sf.tracks.at(sequences.back().trackIndex).points, sf.tracks.at(sequences.back().trackIndex).id, sequences.back().trackPointsDirection == ITrackSequencer::FORWARD, false);

//    Field f;
//    f.outer_boundary = sf.boundary_outer;
//    f.subfields.emplace_back(sf);
//    std::vector<Machine> machines{machine};
//    io::writePlanXML("/tmp/test_tracksequencers_" + posfix + ".xml",
//                     f, machines, routes);

    PointVecVec ptsOut{sf.boundary_inner.points, sf.boundary_outer.points, PointVec()};
    auto& pts = ptsOut.back();
    pts.reserve(route.route_points.size());
    for(auto& p : route.route_points)
        pts.emplace_back(p);
    std::cout << std::endl << "*** test_tracksequencers_" << posfix << " *** " << std::endl;
    std::cout << Point::toStringCSV(ptsOut) << std::endl;
}

BOOST_AUTO_TEST_SUITE(test_tracksequencers)
BOOST_AUTO_TEST_CASE(test_tracksequencersimannealing_1)
{
    std::vector<Machine> machines(1, Machine());
    auto& m = machines.back();
    m.id = 1;
    m.working_width = 1;
    m.turning_radius = 2 * m.working_width;
    m.def_working_speed = 3;
    auto sf = getTestSubfield(m.working_width);

    std::shared_ptr<InfieldTracksConnectorDef> connector = std::make_shared<InfieldTracksConnectorDef>();
    std::shared_ptr<TrackSequencerSimAnnealing::TracksConnectorDef> connector_sa = std::make_shared<TrackSequencerSimAnnealing::TracksConnectorDef>(connector);
    std::map<MachineId_t, std::vector<ITrackSequencer::TrackInfo>> seqs_cn, seqs_sa;
    ITrackSequencer::TrackSequencerSettings settings;
    settings.limitStartToExtremaTracks = false;
    settings.useMachineTurningRad = true;

    std::cout << "Creating sequence with TrackSequencerClosestNext" << std::endl;

    std::unique_ptr<TrackSequencerClosestNext> sequencer_cn = std::make_unique<TrackSequencerClosestNext>();
    sequencer_cn->setInfieldTrackConnector(connector);
    sequencer_cn->setSaveAllComputedPaths(true);

    TrackSequencerClosestNext sequencer_cn2;
    sequencer_cn->setInfieldTrackConnector(connector);
    sequencer_cn->setSaveAllComputedPaths(true);

    AroResp aroResp = sequencer_cn->computeSequences(sf, machines, settings, seqs_cn);
    BOOST_REQUIRE(!aroResp.isError());// , "Error generating sequences with TrackSequencerClosestNext");

    auto it_m = seqs_cn.find(m.id);
    BOOST_REQUIRE(it_m != seqs_cn.end());// , "Error generating sequences with TrackSequencerClosestNext: no sequences for the machine");
    auto& machineSeqs_cn = it_m->second;
    BOOST_REQUIRE(machineSeqs_cn.size() == sf.tracks.size());// , "Error generating sequences with TrackSequencerClosestNext: sequences size != tracks size");
    BOOST_REQUIRE(checkTrackIndexesInRange(sf, machineSeqs_cn));// , "Error generating sequences with TrackSequencerClosestNext: track indexes not in tracks range");
    BOOST_CHECK( checkTracksVisits(sf, machineSeqs_cn) );// , "Error generating sequences with TrackSequencerClosestNext: invalid or incomplete track visits");

    auto cost_cn = getConnectionLength(sf, machineSeqs_cn, m, *sequencer_cn, *connector);
    BOOST_CHECK( !std::isnan(cost_cn) );
    BOOST_CHECK( cost_cn > 1e-9 );

    std::cout << "TrackSequencerClosestNext: Energy (cost) = " << cost_cn << std::endl;

//    //debug!
//    savePath("TrackSequencerClosestNext", sf, machineSeqs_cn, m, *sequencer_cn);


    std::cout << "Creating sequence with TrackSequencerSimAnnealing" << std::endl;

    TrackSequencerSimAnnealing sequencer_sa(std::move(sequencer_cn), std::make_shared<StateGeneratorSingleSwap>(), connector_sa);

    sequencer_sa.setSaveAllComputedPaths(true);
    aroResp = sequencer_sa.computeSequences(sf, machines, settings, seqs_sa);
    BOOST_REQUIRE(!aroResp.isError());// , "Error generating sequences with TrackSequencerSimAnnealing");

    it_m = seqs_sa.find(m.id);
    BOOST_REQUIRE(it_m != seqs_sa.end());// , "Error generating sequences with TrackSequencerSimAnnealing: no sequences for the machine");
    auto& machineSeqs_sa = it_m->second;
    BOOST_REQUIRE(machineSeqs_sa.size() == sf.tracks.size());// , "Error generating sequences with TrackSequencerSimAnnealing: sequences size != tracks size");
    BOOST_REQUIRE(checkTrackIndexesInRange(sf, machineSeqs_sa));// , "Error generating sequences with TrackSequencerSimAnnealing: track indexes not in tracks range");
    BOOST_CHECK( checkTracksVisits(sf, machineSeqs_sa) );// , "Error generating sequences with TrackSequencerSimAnnealing: invalid or incomplete track visits");

    auto cost_sa = getConnectionLength(sf, machineSeqs_sa, m, sequencer_sa, *connector);

    BOOST_CHECK( !std::isnan(cost_sa) );
    BOOST_CHECK( cost_sa > 1e-9 );

    std::cout << "TrackSequencerSimAnnealing: Energy (cost) = " << cost_sa << std::endl;

    BOOST_CHECK( cost_sa < cost_cn + 1e-9 );

    std::cout << "Test finished" << std::endl;

//    //debug!
//    savePath("TrackSequencerSimAnnealing", sf, machineSeqs_sa, m, sequencer_sa);
}

BOOST_AUTO_TEST_SUITE_END()
