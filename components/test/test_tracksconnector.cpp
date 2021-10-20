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
 
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <chrono>

#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/io/io_kml.hpp"
#include "arolib/io/io_xml.hpp"
#include "arolib/components/headlandplanner.h"
#include "arolib/components/tracksgenerator.h"
#include "arolib/planning/infieldtracksconnector.hpp"

BOOST_AUTO_TEST_SUITE(test_tracksconnector)
boost::filesystem::path getOutputDir()
{
    auto out_dir = boost::filesystem::temp_directory_path() / "arolib" / "test" / "test_tracksconnector";
    boost::filesystem::create_directories(out_dir);
    return out_dir;
}

arolib::Field getTestField(bool convex = true)
{

    arolib::Point geoRef(9.93661676071641, 52.007229170305955), utmRef;
    arolib::CoordTransformer::GetInstance().convert_to_cartesian(geoRef, utmRef);

    arolib::Field f;
    f.outer_boundary.points.push_back(utmRef);
    f.outer_boundary.points.push_back(f.outer_boundary.points.back() + arolib::Point(100, 20));
    f.outer_boundary.points.push_back(f.outer_boundary.points.back() + arolib::Point(0, -150));
    f.outer_boundary.points.push_back(f.outer_boundary.points.back() + arolib::Point(-90, 30));
    f.outer_boundary.points.push_back(f.outer_boundary.points.back() + arolib::Point(30, 40)); //inner spike -> i = 4
    f.outer_boundary.points.push_back(f.outer_boundary.points.back() + arolib::Point(-40, 20));
    f.outer_boundary.points.push_back(f.outer_boundary.points.front());

    if (convex)
    {
        f.outer_boundary.points.erase(f.outer_boundary.points.begin() + 4);
    }

    f.subfields.push_back(arolib::Subfield());
    auto &sf = f.subfields.front();
    sf.boundary_outer = f.outer_boundary;

    sf.reference_lines.push_back(arolib::Linestring());
    sf.reference_lines.front().points.push_back(utmRef);
    sf.reference_lines.front().points.push_back(utmRef + arolib::Point(0, -10));

    std::vector<arolib::Point> seg = {f.outer_boundary.points.front(), f.outer_boundary.points.at(1)};
    sf.access_points.push_back(arolib::FieldAccessPoint());
    sf.access_points.back().point() = arolib::geometry::getPointAtRelativeDist(seg, 0.3).first;

    seg = {f.outer_boundary.points.back(), r_at(f.outer_boundary.points, 1)};
    sf.access_points.push_back(arolib::FieldAccessPoint());
    sf.access_points.back().point() = arolib::geometry::getPointAtRelativeDist(seg, 0.1).first;

    return f;
}

void processSubieldGeometries_completeHL(arolib::Subfield &sf, double minTrackDist, double hl_width)
{
    arolib::HeadlandPlanner hp;
    arolib::HeadlandPlanner::PlannerParameters hp_params;
    hp_params.headlandWidth = hl_width;
    hp_params.numTracks = hl_width;
    std::vector<arolib::HeadlandRoute> routes;
    hp.planHeadlandSubfield(sf,
                            {},
                            arolib::OutFieldInfo(),
                            hp_params,
                            arolib::ArolibGrid_t(),
                            std::make_shared<arolib::EMC_MassGrid>(),
                            std::make_shared<arolib::EdgeSpeedCalculatorDef>(),
                            routes);

    arolib::TracksGenerator tg;
    arolib::TracksGenerator::TracksGeneratorParameters tg_params;
    tg_params.sampleResolution = 5;
    tg_params.trackDistance = minTrackDist;
    tg.generateTracks(sf,
                      tg_params);

    if (sf.tracks.front().points.front().x > sf.tracks.back().points.front().x)
        std::reverse(sf.tracks.begin(), sf.tracks.end());

    size_t prevIdx = 5;
    int count_delete = 1;
    int dCount = 1;
    std::vector<size_t> indDelete;
    bool stop = false;
    while (!stop)
    {
        for (int i = 0; i < count_delete; ++i)
        {
            auto ind = prevIdx + i;
            if (ind + 2 > sf.tracks.size())
            {
                stop = true;
                break;
            }
            indDelete.push_back(ind);
        }
        prevIdx += count_delete + 1;
        if (count_delete > 4)
            dCount = -1;
        else if (count_delete < 1)
            dCount = -1;
        count_delete += dCount;
    }

    std::reverse(indDelete.begin(), indDelete.end());
    for (auto &i : indDelete)
        sf.tracks.erase(sf.tracks.begin() + i);

    for (size_t i = 0; i < sf.tracks.size(); ++i)
    {
        sf.tracks.at(i).id = i;
        std::reverse(sf.tracks.at(i).points.begin(), sf.tracks.at(i).points.end());
    }
}

arolib::Route createRoute(const std::vector<arolib::Point> &path, int m_id, int r_id)
{
    arolib::Route r;
    r.machine_id = m_id;
    r.route_id = r_id;
    r.route_points.resize(path.size());
    for (size_t i = 0; i < path.size(); ++i)
    {
        auto &rp = r.route_points.at(i);
        rp.time_stamp = i;
        rp.point() = path.at(i);
    }
    return r;
}

void test_connectTracks_withCompleteHL()
{
    double turningRad = 2;
    double extraDist = 1;

    auto field = getTestField(false);
    field.name = "field_withCompleteHL";
    auto &sf = field.subfields.front();
    processSubieldGeometries_completeHL(sf, turningRad, 8);

    arolib::io::writeFieldKML((getOutputDir() / "test_field_completeHL.kml").string(), field);

    arolib::Logger logger(arolib::LogLevel::DEBUG, __FUNCTION__);
    std::shared_ptr<arolib::IInfieldTracksConnector> tc = std::make_shared<arolib::InfieldTracksConnectorDef>(&logger);
    std::vector<arolib::Machine> machines;
    std::vector<arolib::Route> routes;
    for (int type = 0; type < 4; type++)
    {
        machines.push_back(arolib::Machine());
        machines.back().id = type;
        std::vector<arolib::Point> path = sf.tracks.front().points;
        arolib::Headlands hls = sf.headlands;
        arolib::Polygon limitBoundary = sf.boundary_outer;
        if (type > 0) //without HL tracks
            hls.complete.tracks.clear();
        if (type > 1) //without HL tracks and middle track
            hls.complete.middle_track.points.clear();
        if (type > 2) //without limit boundary
            limitBoundary.points.clear();
        for (size_t i = 1; i < sf.tracks.size(); ++i)
        {
            auto &track = sf.tracks.at(i).points;
            arolib::Pose2D pose_start(path.back());
            pose_start.angle = arolib::geometry::get_angle(r_at(path, 1), path.back());

            auto connection = tc->getConnection(machines.back(),
                                                pose_start,
                                                track,
                                                true,
                                                turningRad,
                                                extraDist,
                                                limitBoundary,
                                                sf.boundary_inner,
                                                hls,
                                                0);

            BOOST_TEST_INFO("Trying to connect with track " << i << " - type " << type);
            BOOST_CHECK(!connection.empty());
            if (connection.empty())
                break;
            path.insert(path.end(), connection.begin(), connection.end());
            if (arolib::geometry::calc_dist(path.back(), track.front()) > arolib::geometry::calc_dist(path.back(), track.back()))
                std::reverse(track.begin(), track.end());
            path.insert(path.end(), track.begin(), track.end());
        }

        if (!path.empty())
            routes.push_back(createRoute(path, machines.back().id, machines.back().id));
    }

    arolib::io::writePlanXML((getOutputDir() / "test_plan_completeHL.xml").string(),
                             field,
                             machines,
                             {routes});
    return;
}

void test_connectTracks_withoutHeadland()
{
    // TODO ???
    return;
}

void test_connectTracks2AccessPoint_withCompleteHL()
{
    double turningRad = 2;
    double extraDist = 1;

    auto field = getTestField(false);
    field.name = "field_withCompleteHL_fap";
    auto &sf = field.subfields.front();
    processSubieldGeometries_completeHL(sf, turningRad, 8);

    std::vector<arolib::Pose2D> fapPoses(sf.access_points.size());
    fapPoses.front().point() = sf.access_points.front();
    fapPoses.front().angle = M_PI_2;
    fapPoses.back().point() = sf.access_points.back();
    fapPoses.back().angle = M_PI;

    arolib::io::writeFieldKML((getOutputDir() / "test_field_completeHL_fap.kml").string(), field);

    arolib::Logger logger(arolib::LogLevel::DEBUG, __FUNCTION__);
    std::shared_ptr<arolib::IInfieldTracksConnector> tc = std::make_shared<arolib::InfieldTracksConnectorDef>(&logger);
    std::vector<arolib::Machine> machines;
    std::vector<arolib::Route> routes;
    for (int type = 0; type < 4; type++)
    {
        arolib::Headlands hls = sf.headlands;
        arolib::Polygon limitBoundary = sf.boundary_outer;
        if (type > 0) //without HL tracks
            hls.complete.tracks.clear();
        if (type > 1) //without HL tracks and middle track
            hls.complete.middle_track.points.clear();
        if (type > 2) //without limit boundary
            limitBoundary.points.clear();
        for (size_t i = 0; i < sf.tracks.size(); ++i)
        {
            auto &track = sf.tracks.at(i).points;
            machines.push_back(arolib::Machine());
            machines.back().id = machines.size() - 1;
            std::vector<arolib::Point> path;

            for (auto &fapPose : fapPoses)
            {
                arolib::Pose2D pose_start(track.front());
                pose_start.angle = arolib::geometry::get_angle(track.at(1), track.front());
                arolib::Pose2D pose_end = fapPose;

                auto connection = tc->getConnection(machines.back(),
                                                    pose_start,
                                                    pose_end,
                                                    turningRad,
                                                    extraDist,
                                                    limitBoundary,
                                                    sf.boundary_inner,
                                                    hls,
                                                    0,
                                                    -1);

                BOOST_TEST_CONTEXT("Trying to connect with exit - track start " << i << " - type " << type);
                BOOST_CHECK(!connection.empty());

                path.insert(path.end(), connection.begin(), connection.end());
                path.push_back(arolib::geometry::getPointAtDist(fapPose, 10));

                // entry to track start
                std::swap(pose_start, pose_end);
                pose_start.angle -= M_PI;
                pose_end.angle -= M_PI;
                arolib::geometry::correct_angle(pose_start.angle);
                arolib::geometry::correct_angle(pose_end.angle);

                connection = tc->getConnection(machines.back(),
                                               pose_start,
                                               pose_end,
                                               turningRad,
                                               extraDist,
                                               limitBoundary,
                                               sf.boundary_inner,
                                               hls,
                                               0,
                                               -1);

                BOOST_TEST_CONTEXT("Trying to connect with entry - track start " << i << " - type " << type);
                BOOST_CHECK(!connection.empty());

                path.insert(path.end(), connection.begin(), connection.end());
                path.insert(path.end(), track.begin(), track.end());

                // track end to exit
                pose_start.point() = track.back();
                pose_start.angle = arolib::geometry::get_angle(r_at(track, 1), track.back());
                pose_end = fapPose;

                connection = tc->getConnection(machines.back(),
                                               pose_start,
                                               pose_end,
                                               turningRad,
                                               extraDist,
                                               limitBoundary,
                                               sf.boundary_inner,
                                               hls,
                                               0,
                                               -1);

                BOOST_TEST_CONTEXT("Trying to connect with exit - track end " << i << " - type " << type);
                BOOST_CHECK(!connection.empty());

                path.insert(path.end(), connection.begin(), connection.end());
                path.push_back(arolib::geometry::getPointAtDist(fapPose, 10));

                // entry to track end
                std::swap(pose_start, pose_end);
                pose_start.angle -= M_PI;
                pose_end.angle -= M_PI;
                arolib::geometry::correct_angle(pose_start.angle);
                arolib::geometry::correct_angle(pose_end.angle);

                connection = tc->getConnection(machines.back(),
                                               pose_start,
                                               pose_end,
                                               turningRad,
                                               extraDist,
                                               limitBoundary,
                                               sf.boundary_inner,
                                               hls,
                                               0,
                                               -1);

                BOOST_TEST_CONTEXT("Trying to connect with entry - track end " << i << " - type " << type);
                BOOST_CHECK(!connection.empty());

                path.insert(path.end(), connection.begin(), connection.end());
                path.insert(path.end(), track.rbegin(), track.rend());
            }

            if (!path.empty())
                routes.push_back(createRoute(path, machines.back().id, machines.back().id));
        }
    }

    arolib::io::writePlanXML((getOutputDir() / "test_plan_completeHL_fap.xml").string(),
                             field,
                             machines,
                             {routes});
}

BOOST_AUTO_TEST_CASE(test1)
{
    test_connectTracks_withCompleteHL();
    test_connectTracks_withoutHeadland();
    test_connectTracks2AccessPoint_withCompleteHL();
}
BOOST_AUTO_TEST_SUITE_END()
