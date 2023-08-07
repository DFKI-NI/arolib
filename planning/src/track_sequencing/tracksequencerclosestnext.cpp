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
 
#include "arolib/planning/track_sequencing/tracksequencerclosestnext.hpp"

#include <stdexcept>
#include <algorithm>

namespace arolib{

TrackSequencerClosestNext::TrackSequencerClosestNext(LogLevel logLevel) :
    ITrackSequencer(__FUNCTION__, logLevel)
{
//    m_tracksConnector = std::make_shared<InfieldTracksConnectorDef>();
//    m_tracksConnector->logger().setParent(loggerPtr());
}

AroResp TrackSequencerClosestNext::computeSequences(const Subfield &subfield,
                                                    const std::vector<Machine> &machines,
                                                    const TrackSequencerSettings& settings,
                                                    std::map<MachineId_t, std::vector<TrackInfo> > &sequences,
                                                    const Pose2D* initRefPose,
                                                    const std::set<size_t> &excludeTrackIndexes)
{

    sequences.clear();

    if(machines.empty())
        return AroResp(1, "No machines given");

    if(subfield.tracks.empty())
        return AroResp(1, "No tracks given");

    for(auto& track : subfield.tracks){
        if(track.points.size() < 2)
            return AroResp(1, "One or more tracks have less than 2 points");
    }

    std::vector<size_t> trackInds;
    for(size_t i = 0 ; i < subfield.tracks.size() ; ++i){
        if(excludeTrackIndexes.find(i) != excludeTrackIndexes.end()
                || subfield.tracks.at(i).points.size() < 2)
            continue;
        trackInds.push_back(i);
    }
    if(trackInds.empty())
        return AroResp(1, "No tracks left after excludeding given tracks");


    std::vector<size_t> extremaTrackInds;
    if(settings.limitStartToExtremaTracks)
        extremaTrackInds = geometry::getInfieldExtremaTracksIndexes(subfield, excludeTrackIndexes);

    auto& trackIndsStart = ( extremaTrackInds.empty() ? trackInds : extremaTrackInds );


    auto nextTrackIndexes = getFirstTracksIndexes(subfield, initRefPose, trackIndsStart, machines.size());  // holds indexes corresponding to trackInds, not the the track index in the tracks vectors
    if(nextTrackIndexes.empty())
        return AroResp(1, "Error obtaining the initial track(s)");


    if(&trackIndsStart != &trackInds){//search the indexes in trackInds corresponding to nextTrackIndexes which correspond to trackIndsStart
        for(size_t i = 0 ; i < nextTrackIndexes.size() ; ++i){
            auto& indInfo = nextTrackIndexes.at(i);
            int ind = -1;
            for(size_t j = 0 ; j < trackInds.size() ; ++j){
                if(trackInds.at(j) == extremaTrackInds.at(indInfo.first)){
                    ind = j;
                    break;
                }
            }
            if(ind < 0)
                return AroResp(1, "Error obtaining track indexes from extrema track indexes");
            indInfo = std::make_pair( ind, indInfo.second );
        }
    }


    std::set<size_t> assignedTrackIndexes; // corresponding to trackInds, not the the track index in the tracks vectors
    std::map<MachineId_t, Polygon> limitBoundaries;
    for(size_t i = 0 ; i < nextTrackIndexes.size() ; ++i){
        auto& indInfo = nextTrackIndexes.at(i);
        auto machine = machines.at(i);
        sequences[machine.id].push_back( ITrackSequencer::TrackInfo( trackInds.at( indInfo.first ), indInfo.second ) );
        assignedTrackIndexes.insert( indInfo.first );
        if(m_tracksConnector)
            limitBoundaries[machine.id] = m_tracksConnector->getExtendedLimitBoundary(subfield, std::max(0.0, machine.getTurningRadius()) );
        else
            limitBoundaries[machine.id] = getDefTracksConnector()->getExtendedLimitBoundary(subfield, std::max(0.0, machine.getTurningRadius()) );
    }

    Polygon connBoundary;
    if(m_useConnOverBoundaryAsReference){
        if(subfield.boundary_inner.points.size() > 3)
            connBoundary = subfield.boundary_inner;
        else if(subfield.boundary_outer.points.size() > 3)
            connBoundary = subfield.boundary_outer;
    }

    if(!connBoundary.points.empty()){
        for(auto& ind : trackInds){
            auto& track = subfield.tracks.at(ind);
            geometry::addSampleToGeometryClosestToPoint(connBoundary.points, track.points.front(), 1);
            geometry::addSampleToGeometryClosestToPoint(connBoundary.points, track.points.back(), 1);
        }
    }

    int indMachine = 0;
    while(assignedTrackIndexes.size() < trackInds.size()){
        auto machine = machines.at(indMachine);
        size_t indPrevTrack = nextTrackIndexes.at(indMachine).first;
        bool prevTrackInReverse = nextTrackIndexes.at(indMachine).second == ITrackSequencer::REVERSE;

        std::pair<int, bool> nextTrackInfo = getNextTrack(subfield,
                                                          trackInds,
                                                          assignedTrackIndexes,
                                                          machine,
                                                          indPrevTrack,
                                                          prevTrackInReverse,
                                                          sequences[machine.id],
                                                          limitBoundaries[machine.id],
                                                          connBoundary,
                                                          settings.useMachineTurningRad);


        if(nextTrackInfo.first < 0){
            logger().printDebug(__FUNCTION__, "IF tracks sequences so far...");
            for(auto& it : sequences){
                logger().printDebug("", "\t Machine id : " + std::to_string(it.first) + ":");
                for(TrackInfo& info : it.second)
                    logger().printDebug("", "\t\t" + std::to_string(info.trackIndex) + "("  + std::to_string(info.trackPointsDirection) + ")");
            }
            logger().printDebug(__FUNCTION__, "IF tracks sequences so far (end)");

            return AroResp(1, "Error obtaining connection to next track from track with index " + std::to_string( trackInds.at(indPrevTrack) ));
        }

        nextTrackIndexes.at(indMachine) = std::make_pair( (size_t)nextTrackInfo.first, nextTrackInfo.second ? ITrackSequencer::REVERSE : ITrackSequencer::FORWARD );
        assignedTrackIndexes.insert( nextTrackInfo.first );
        sequences[machine.id].push_back( ITrackSequencer::TrackInfo( trackInds.at( nextTrackIndexes.at(indMachine).first ), nextTrackIndexes.at(indMachine).second ) );

        indMachine = get_index_from_cyclic_container(nextTrackIndexes, indMachine+1);
    }

    {
        logger().printDebug(__FUNCTION__, "Resulting IF tracks sequences ...");
        for(auto& it : sequences){
            logger().printDebug("", "\t Machine id : " + std::to_string(it.first) + ":");
            for(TrackInfo& info : it.second)
                logger().printDebug("", "\t\t" + std::to_string(info.trackIndex) + "("  + std::to_string(info.trackPointsDirection) + ")");
        }
        logger().printDebug(__FUNCTION__, "Resulting IF tracks sequences (end)");
    }

    return AroResp::ok();

}

void TrackSequencerClosestNext::setUseConnOverBoundaryAsReference(bool useConnOverBoundaryAsReference){
    m_useConnOverBoundaryAsReference = useConnOverBoundaryAsReference;
}

std::pair<int, bool> TrackSequencerClosestNext::getNextTrack(const Subfield &subfield,
                                                             const std::vector<size_t>& trackInds,
                                                             const std::set<size_t>& assignedTrackIndexes,
                                                             const Machine& machine,
                                                             size_t indPrevTrack,
                                                             bool prevTrackInReverse,
                                                             const std::vector<TrackInfo>& sequence,
                                                             const Polygon& limitBoundary,
                                                             const Polygon& connBoundary,
                                                             bool useMachineTurningRad){

    auto getEpsForLengthComparison = [](double minLength, double currentLength) -> double{
        return 0.02 * std::min( minLength, currentLength );
    };

    auto checkForConnPathSimilarity = [](const PointVec& connPath, const PointVec& path1, const PointVec& path2, double path1Length, double connLength)->double{

        if(path2.empty())
            return path1Length;
        double path2Length = geometry::getGeometryLength(path2);
        if(connLength < path2Length)
            return path1Length;

        auto pt1_1 = geometry::getPointAtRelativeDist(path1, 0.25).first;
        auto pt2_1 = geometry::getPointAtRelativeDist(path1, 0.5).first;
        auto pt3_1 = geometry::getPointAtRelativeDist(path1, -0.25).first;

        auto pt1_2 = geometry::getPointAtRelativeDist(path2, 0.25).first;
        auto pt2_2 = geometry::getPointAtRelativeDist(path2, 0.5).first;
        auto pt3_2 = geometry::getPointAtRelativeDist(path2, -0.25).first;

        double dist1 = geometry::calc_dist_to_linestring(connPath, pt1_1, false) +
                       geometry::calc_dist_to_linestring(connPath, pt2_1, false) +
                       geometry::calc_dist_to_linestring(connPath, pt3_1, false);
        double dist2 = geometry::calc_dist_to_linestring(connPath, pt1_2, false) +
                       geometry::calc_dist_to_linestring(connPath, pt2_2, false) +
                       geometry::calc_dist_to_linestring(connPath, pt3_2, false);
        if(dist1 > dist2)
            return path2Length;
        return path1Length;
    };

    double minDist = std::numeric_limits<double>::max();
    int indMin = -1;
    bool nextInReverse;

    size_t indFwd = indPrevTrack;
    size_t indBwd = indPrevTrack;
    bool checkFwd = true;
    for(size_t i = 0 ; i+1 < trackInds.size() ; ++i){
        size_t indNext;

        //it is likelly that the best options is either in the next or previous tracks in the list -> alternate checking direction
        if(checkFwd){
            if( indFwd+1 >= trackInds.size() ){//keep checking only backwards
                --i;
                checkFwd = !checkFwd;
                continue;
            }
            ++indFwd;
            indNext = indFwd;
        }
        else{
            if( indBwd == 0 ){//keep checking only forward
                --i;
                checkFwd = !checkFwd;
                continue;
            }
            --indBwd;
            indNext = indBwd;
        }

        checkFwd = !checkFwd;
        if( assignedTrackIndexes.find(indNext) != assignedTrackIndexes.end() )
            continue;

        std::pair<double, bool> connInfo = std::make_pair(-1, false);
        double dist0TC = -1, distnTC = -1;

        const auto& trackFrom = subfield.tracks.at( trackInds.at(indPrevTrack) );
        const auto& trackNext = subfield.tracks.at( trackInds.at(indNext) );

        PointVec connPath0, connPathn;

        if(connBoundary.points.empty())
            connInfo = computeConnectionDistances(m_tracksConnector,
                                                  subfield,
                                                  machine,
                                                  trackFrom,
                                                  trackNext,
                                                  prevTrackInReverse,
                                                  minDist,
                                                  limitBoundary,
                                                  useMachineTurningRad,
                                                  false,
                                                  true, true,
                                                  connPath0, connPathn,
                                                  dist0TC, distnTC);
        else {//use a combination of the default tracks connector and connection over boundary
            auto& trackFromPts = trackFrom.points;
            auto& trackNextPts = trackNext.points;
            Point p0 = prevTrackInReverse ? trackFromPts.front() : trackFromPts.back();


            double dist0 = geometry::calc_dist(p0, trackNextPts.front());
            double distn = geometry::calc_dist(p0, trackNextPts.back());

            if(indMin >= 0){
                double eps = getEpsForLengthComparison(minDist, dist0);
                if( dist0 > minDist + eps )
                    dist0 = -1;
                eps = getEpsForLengthComparison(minDist, dist0);
                if( distn > minDist + eps )
                    distn = -1;
            }
            if( dist0 < -1e-9 && distn < -1e-9 )
                continue;

            PointVec path0, pathn;

            if(dist0 > -1e-9){
                path0 = geometry::getShortestGeometryPart(connBoundary.points, p0, trackNextPts.front());
                if(path0.empty())
                    dist0 = -1;
                else
                    dist0 = geometry::getGeometryLength(path0);

            }
            if(distn > -1e-9){
                pathn = geometry::getShortestGeometryPart(connBoundary.points, p0, trackNextPts.back());
                if(pathn.empty())
                    distn = -1;
                else
                    distn = geometry::getGeometryLength(pathn);
            }

            if(path0.empty() && pathn.empty())
                continue;

            if(indMin >= 0){
                double eps = getEpsForLengthComparison(minDist, dist0);
                if( dist0 > minDist + eps )
                    dist0 = -1;
                eps = getEpsForLengthComparison(minDist, dist0);
                if( distn > minDist + eps )
                    distn = -1;
            }
            if( dist0 < -1e-9 && distn < -1e-9 )
                continue;

            computeConnectionDistances(m_tracksConnector,
                                       subfield,
                                       machine,
                                       trackFrom,
                                       trackNext,
                                       prevTrackInReverse,
                                       -1,//we want to check both sides independent of the current minDist
                                       limitBoundary,
                                       useMachineTurningRad,
                                       true,
                                       dist0 > -1e-9, distn > -1e-9,
                                       connPath0, connPathn,
                                       dist0TC, distnTC);

            if(dist0TC < -1e-9)
                dist0 = -1;
            if(distnTC < -1e-9)
                distn = -1;
            if(dist0 < -1e-9)
                dist0TC = -1;
            if(distn < -1e-9)
                distnTC = -1;


            if( dist0 < -1e-9 && distn < -1e-9 ) //the default connector failed
                continue;

            //check if the connection was really done over the shortest paths
            if( dist0 > -1e-9 && dist0TC > 0 && connPath0.size() > 1 ){
                auto path0_2 = geometry::getLongestGeometryPart(connBoundary.points, p0, trackNextPts.front());
                dist0 = checkForConnPathSimilarity(connPath0, path0, path0_2, dist0, dist0TC);

            }
            if( distn > -1e-9 && distnTC > 0 && connPathn.size() > 1 ){
                auto pathn_2 = geometry::getLongestGeometryPart(connBoundary.points, p0, trackNextPts.back());
                distn = checkForConnPathSimilarity(connPathn, pathn, pathn_2, distn, distnTC);
            }

            if(dist0TC < -1e-9)
                connInfo = std::make_pair( distn, true );
            else if(distnTC < -1e-9)
                connInfo = std::make_pair( dist0, false );
            else{
                if( dist0TC < distnTC )
                    connInfo = std::make_pair( dist0, false );
                else if( dist0TC > distnTC )
                    connInfo = std::make_pair( distn, true );
                else if(dist0 > distn)
                    connInfo = std::make_pair( distn, true );
                else
                    connInfo = std::make_pair( dist0, false );
            }
        }
        if(connInfo.first < -1e-9)
            continue;

        double eps = getEpsForLengthComparison( minDist, connInfo.first );//used for when the distances are too similar
        if(std::fabs(minDist - connInfo.first) <= eps && indMin >= 0){//if the distances are too similar, check previously visited track and visit the one that is in the same direction to the previous visited areas
            if(sequence.size() > 1){
                //check the "directions" between the previous visited tracks and potential next tracks

                Point directionPrev = getSequenceAverageDirection(subfield, sequence);

                auto& trackFrom = subfield.tracks.at( trackInds.at(indPrevTrack) ).points;
                auto& nextTrack1 = subfield.tracks.at( indMin ).points;
                auto& nextTrack2 = subfield.tracks.at( indNext ).points;

                Point direction1 = geometry::getDirectionBetweenTracks(trackFrom, nextTrack1);
                Point direction2 = geometry::getDirectionBetweenTracks(trackFrom, nextTrack2);

                double angle1 = geometry::get_angle(directionPrev, Point(0,0), direction1, true);
                double angle2 = geometry::get_angle(directionPrev, Point(0,0), direction2, true);

                double angDiff = angle2 - angle1;

                if(std::fabs(angDiff) > 1 && std::fabs(angle1) < std::fabs(angle2))
                    minDist = std::numeric_limits<double>::max();
            }
        }
        if(minDist > connInfo.first ){
            minDist = connInfo.first;
            nextInReverse = connInfo.second;
            indMin = indNext;
        }

    }
    return std::make_pair(indMin, nextInReverse);
}


std::pair<int, bool> TrackSequencerClosestNext::getNextTrack2(const Subfield &subfield,
                                                             const std::vector<size_t>& trackInds,
                                                             const std::set<size_t>& assignedTrackIndexes,
                                                             const Machine& machine,
                                                             size_t indPrevTrack,
                                                             bool prevTrackInReverse,
                                                             const std::vector<TrackInfo>& sequence,
                                                             const Polygon& limitBoundary,
                                                             const Polygon& connBoundary,
                                                             bool useMachineTurningRad){

    auto getEpsForLengthComparison = [](double minLength, double currentLength) -> double{
        return 0.02 * std::min( minLength, currentLength );
    };

    auto checkForConnPathSimilarity = [](const PointVec& connPath, const PointVec& path1, const PointVec& path2, double path1Length, double connLength)->double{

        if(path2.empty())
            return path1Length;
        double path2Length = geometry::getGeometryLength(path2);
        if(connLength < path2Length)
            return path1Length;

        auto pt1_1 = geometry::getPointAtRelativeDist(path1, 0.25).first;
        auto pt2_1 = geometry::getPointAtRelativeDist(path1, 0.5).first;
        auto pt3_1 = geometry::getPointAtRelativeDist(path1, -0.25).first;

        auto pt1_2 = geometry::getPointAtRelativeDist(path2, 0.25).first;
        auto pt2_2 = geometry::getPointAtRelativeDist(path2, 0.5).first;
        auto pt3_2 = geometry::getPointAtRelativeDist(path2, -0.25).first;

        double dist1 = geometry::calc_dist_to_linestring(connPath, pt1_1, false) +
                       geometry::calc_dist_to_linestring(connPath, pt2_1, false) +
                       geometry::calc_dist_to_linestring(connPath, pt3_1, false);
        double dist2 = geometry::calc_dist_to_linestring(connPath, pt1_2, false) +
                       geometry::calc_dist_to_linestring(connPath, pt2_2, false) +
                       geometry::calc_dist_to_linestring(connPath, pt3_2, false);
        if(dist1 > dist2)
            return path2Length;
        return path1Length;
    };

    double minDist = std::numeric_limits<double>::max();
    int indMin = -1;
    bool nextInReverse;

    unsigned int numThreads = trackInds.size()-assignedTrackIndexes.size();
    numThreads = std::max( (unsigned int)1, std::min( std::thread::hardware_concurrency(), numThreads ) );
    numThreads = std::min( (unsigned int)2, numThreads );//using many threads makes it slower :/

    std::vector< std::vector<size_t> > threadInds(numThreads);
    size_t indThreadInds = 0;

    size_t indFwd = indPrevTrack;
    size_t indBwd = indPrevTrack;
    bool checkFwd = true;
    for(size_t i = 0 ; i+1 < trackInds.size() ; ++i){
        size_t indNext;

        //it is likelly that the best options is either in the next or previous tracks in the list -> alternate checking direction
        if(checkFwd){
            if( indFwd+1 >= trackInds.size() ){//keep checking only backwards
                --i;
                checkFwd = !checkFwd;
                continue;
            }
            ++indFwd;
            indNext = indFwd;
        }
        else{
            if( indBwd == 0 ){//keep checking only forward
                --i;
                checkFwd = !checkFwd;
                continue;
            }
            --indBwd;
            indNext = indBwd;
        }

        checkFwd = !checkFwd;
        if( assignedTrackIndexes.find(indNext) != assignedTrackIndexes.end() )
            continue;

        threadInds.at(indThreadInds).push_back(indNext);
        indThreadInds = get_index_from_cyclic_container(threadInds, indThreadInds+1);
    }



    std::vector< std::future< void > > futures_search (numThreads);
    std::mutex mutex_best;

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Started searching for next track in " + std::to_string(numThreads) + " threads... " );
    for(size_t kk = 0 ; kk < futures_search.size() ; ++kk){
        auto& fu = futures_search.at(kk);
        auto& inds = threadInds.at(kk);

        fu = std::async( std::launch::async,
                         [kk,
                          this,
                          &mutex_best,
                          &inds,
                          &subfield,
                          &trackInds,
                          &machine,
                          indPrevTrack,
                          prevTrackInReverse,
                          &sequence,
                          &limitBoundary,
                          &connBoundary,
                          useMachineTurningRad,
                          &minDist,
                          &indMin,
                          &nextInReverse,
                          getEpsForLengthComparison,
                          checkForConnPathSimilarity](){

            for(auto& indNext : inds){

                std::pair<double, bool> connInfo = std::make_pair(-1, false);
                double dist0TC = -1, distnTC = -1;

                const auto& trackFrom = subfield.tracks.at( trackInds.at(indPrevTrack) );
                const auto& trackNext = subfield.tracks.at( trackInds.at(indNext) );

                PointVec connPath0, connPathn;

                double minDistCopy;
                int indMinCopy;

                auto safeCopyMinData = [&mutex_best, &minDist, &indMin, &minDistCopy, &indMinCopy](){
                    std::lock_guard<std::mutex> guard(mutex_best);
                    minDistCopy = minDist;
                    indMinCopy = indMin;
                };

                safeCopyMinData();

                if(connBoundary.points.empty())
                    connInfo = computeConnectionDistances(m_tracksConnector,
                                                          subfield,
                                                          machine,
                                                          trackFrom,
                                                          trackNext,
                                                          prevTrackInReverse,
                                                          minDistCopy,
                                                          limitBoundary,
                                                          useMachineTurningRad,
                                                          false,
                                                          true, true,
                                                          connPath0, connPathn,
                                                          dist0TC, distnTC);
                else {//use a combination of the default tracks connector and connection over boundary
                    auto& trackFromPts = trackFrom.points;
                    auto& trackNextPts = trackNext.points;
                    Point p0 = prevTrackInReverse ? trackFromPts.front() : trackFromPts.back();


                    double dist0 = geometry::calc_dist(p0, trackNextPts.front());
                    double distn = geometry::calc_dist(p0, trackNextPts.back());

                    safeCopyMinData();

                    if(indMinCopy >= 0){
                        double eps = getEpsForLengthComparison(minDistCopy, dist0);
                        if( dist0 > minDistCopy + eps )
                            dist0 = -1;
                        eps = getEpsForLengthComparison(minDistCopy, dist0);
                        if( distn > minDistCopy + eps )
                            distn = -1;
                    }
                    if( dist0 < -1e-9 && distn < -1e-9 ){
                        continue;
                    }

                    PointVec path0, pathn;

                    if(dist0 > -1e-9){
                        path0 = geometry::getShortestGeometryPart(connBoundary.points, p0, trackNextPts.front());
                        if(path0.empty())
                            dist0 = -1;
                        else
                            dist0 = geometry::getGeometryLength(path0);

                    }
                    if(distn > -1e-9){
                        pathn = geometry::getShortestGeometryPart(connBoundary.points, p0, trackNextPts.back());
                        if(pathn.empty())
                            distn = -1;
                        else
                            distn = geometry::getGeometryLength(pathn);
                    }

                    if(path0.empty() && pathn.empty()){
                        continue;
                    }

                    safeCopyMinData();

                    if(indMinCopy >= 0){
                        double eps = getEpsForLengthComparison(minDistCopy, dist0);
                        if( dist0 > minDistCopy + eps )
                            dist0 = -1;
                        eps = getEpsForLengthComparison(minDistCopy, dist0);
                        if( distn > minDistCopy + eps )
                            distn = -1;
                    }
                    if( dist0 < -1e-9 && distn < -1e-9 ){
                        continue;
                    }

                    computeConnectionDistances(m_tracksConnector,
                                               subfield,
                                               machine,
                                               trackFrom,
                                               trackNext,
                                               prevTrackInReverse,
                                               -1,//we want to check both sides independent of the current minDist
                                               limitBoundary,
                                               useMachineTurningRad,
                                               true,
                                               dist0 > -1e-9, distn > -1e-9,
                                               connPath0, connPathn,
                                               dist0TC, distnTC);

                    if(dist0TC < -1e-9)
                        dist0 = -1;
                    if(distnTC < -1e-9)
                        distn = -1;
                    if(dist0 < -1e-9)
                        dist0TC = -1;
                    if(distn < -1e-9)
                        distnTC = -1;


                    if( dist0 < -1e-9 && distn < -1e-9 ){ //the default connector failed
                        continue;
                    }

                    //check if the connection was really done over the shortest paths
                    if( dist0 > -1e-9 && dist0TC > 0 && connPath0.size() > 1 ){
                        auto path0_2 = geometry::getLongestGeometryPart(connBoundary.points, p0, trackNextPts.front());
                        dist0 = checkForConnPathSimilarity(connPath0, path0, path0_2, dist0, dist0TC);

                    }
                    if( distn > -1e-9 && distnTC > 0 && connPathn.size() > 1 ){
                        auto pathn_2 = geometry::getLongestGeometryPart(connBoundary.points, p0, trackNextPts.back());
                        distn = checkForConnPathSimilarity(connPathn, pathn, pathn_2, distn, distnTC);
                    }

                    if(dist0TC < -1e-9)
                        connInfo = std::make_pair( distn, true );
                    else if(distnTC < -1e-9)
                        connInfo = std::make_pair( dist0, false );
                    else{
                        if( dist0TC < distnTC )
                            connInfo = std::make_pair( dist0, false );
                        else if( dist0TC > distnTC )
                            connInfo = std::make_pair( distn, true );
                        else if(dist0 > distn)
                            connInfo = std::make_pair( distn, true );
                        else
                            connInfo = std::make_pair( dist0, false );
                    }
                }
                if(connInfo.first < -1e-9){
                    continue;
                }


                {
                    std::lock_guard<std::mutex> guard(mutex_best);
                    double eps = getEpsForLengthComparison( minDist, connInfo.first );//used for when the distances are too similar
                    if(std::fabs(minDist - connInfo.first) <= eps && indMin >= 0){//if the distances are too similar, check previously visited track and visit the one that is in the same direction to the previous visited areas
                        if(sequence.size() > 1){
                            //check the "directions" between the previous visited tracks and potential next tracks

                            Point directionPrev = getSequenceAverageDirection(subfield, sequence);

                            auto& trackFrom = subfield.tracks.at( trackInds.at(indPrevTrack) ).points;
                            auto& nextTrack1 = subfield.tracks.at( indMin ).points;
                            auto& nextTrack2 = subfield.tracks.at( indNext ).points;

                            Point direction1 = geometry::getDirectionBetweenTracks(trackFrom, nextTrack1);
                            Point direction2 = geometry::getDirectionBetweenTracks(trackFrom, nextTrack2);

                            double angle1 = geometry::get_angle(directionPrev, Point(0,0), direction1, true);
                            double angle2 = geometry::get_angle(directionPrev, Point(0,0), direction2, true);

                            double angDiff = angle2 - angle1;

                            if(std::fabs(angDiff) > 1 && std::fabs(angle1) < std::fabs(angle2))
                                minDist = std::numeric_limits<double>::max();
                        }
                    }
                    if(minDist > connInfo.first ){
                        minDist = connInfo.first;
                        nextInReverse = connInfo.second;
                        indMin = indNext;
                    }
                }
            }
        });
    }

    for(auto& fu : futures_search)
        fu.wait();

    return std::make_pair(indMin, nextInReverse);
}

std::pair<double, bool> TrackSequencerClosestNext::computeConnectionDistances(std::shared_ptr<IInfieldTracksConnector> connector,
                                                                              const Subfield &subfield,
                                                                              const Machine &machine,
                                                                              const Track& trackFrom,
                                                                              const Track& trackNext,
                                                                              bool trackFromInReverse,
                                                                              double maxDist,
                                                                              const Polygon &limitBoundary,
                                                                              bool withTurningRad,
                                                                              bool checkSidesIndependently,
                                                                              bool computeForTrackStart,
                                                                              bool computeForTrackEnd,
                                                                              PointVec& path0,
                                                                              PointVec& pathn,
                                                                              double & lengthToNext0,
                                                                              double & lengthToNextn)
{
    lengthToNext0 = lengthToNextn = -1;
    path0.clear();
    pathn.clear();

    if(!connector){
        //use default connector
        connector = getDefTracksConnector();
    }

    auto& trackFromPts = trackFrom.points;
    auto& trackNextPts = trackNext.points;
    Pose2D poseFrom;
    if(trackFromInReverse){
        poseFrom.point() = trackFromPts.front();
        poseFrom.angle = geometry::get_angle( trackFromPts.at(1), trackFromPts.front() );
    }
    else{
        poseFrom.point() = trackFromPts.back();
        poseFrom.angle = geometry::get_angle( r_at(trackFromPts, 1), trackFromPts.back() );
    }

    double turningRad = withTurningRad ? -1 : 0;

    if(computeForTrackStart){
        Pose2D poseNext0( trackNextPts.front(), geometry::get_angle( trackNextPts.front(), trackNextPts.at(1) ) );
        path0 = getPathFromMap(poseFrom, poseNext0, turningRad, true);
        if(path0.empty()){
            path0 = connector->getConnection( machine,
                                              poseFrom,
                                              poseNext0,
                                              turningRad,
                                              std::make_pair(0.0, 0.0),
                                              limitBoundary,
                                              subfield.boundary_inner,
                                              subfield.headlands,
                                              -1,
                                              maxDist );

            if( !path0.empty() ){
                addPathToMap(poseFrom, poseNext0, turningRad, path0);
            }
        }

        if( !path0.empty() ){
            lengthToNext0 = geometry::getGeometryLength(path0);
            if(!checkSidesIndependently)
                maxDist = std::min(maxDist, lengthToNext0);
        }
    }

    if(computeForTrackEnd){
        Pose2D poseNextn( trackNextPts.back(), geometry::get_angle( trackNextPts.back(), r_at(trackNextPts, 1) ) );
        pathn = getPathFromMap(m_pathsMap, poseFrom, poseNextn, turningRad, true);
        if(pathn.empty()){
            pathn = connector->getConnection( machine,
                                              poseFrom,
                                              poseNextn,
                                              turningRad,
                                              std::make_pair(0.0, 0.0),
                                              limitBoundary,
                                              subfield.boundary_inner,
                                              subfield.headlands,
                                              -1,
                                              maxDist );

            if( !pathn.empty() ){
                addPathToMap(poseFrom, poseNextn, turningRad, pathn);
            }
        }
        if( !pathn.empty() ){
            lengthToNextn = geometry::getGeometryLength(pathn);
        }
    }

    if(path0.empty() && pathn.empty())
        return std::make_pair(-1, false);

    if(path0.empty())
        return std::make_pair(lengthToNextn, true);

    if(pathn.empty())
        return std::make_pair(lengthToNext0, false);

    return std::make_pair(std::min(lengthToNext0, lengthToNextn), lengthToNext0 > lengthToNextn);
}


std::vector<std::pair<size_t, ITrackSequencer::TrackPointsDirection>> TrackSequencerClosestNext::getFirstTracksIndexes(const Subfield &subfield,
                                                                                                                       const Point *initRefPoint,
                                                                                                                       const std::vector<size_t>& trackInds,
                                                                                                                       size_t numMachines)
{
    auto getDefault = [&](void)->std::vector<std::pair<size_t, ITrackSequencer::TrackPointsDirection>>{
        std::vector<std::pair<size_t, ITrackSequencer::TrackPointsDirection>> ret;
        size_t countMachines = 0;
        for(size_t i = 0 ; countMachines < numMachines && i < trackInds.size() ; i++, countMachines++)
            ret.push_back( std::make_pair(i, ITrackSequencer::UNDEF) );
        return ret;
    };

    if(numMachines == 0)
        return {};
    if(!initRefPoint)
        return getDefault();
    if(!initRefPoint->isValid())
        return getDefault();

    std::multimap<double, size_t> trackDistances;
    for(size_t i = 0 ; i < trackInds.size() ; ++i){
        double dist = std::min( geometry::calc_dist(subfield.tracks.at( trackInds.at(i) ).points.front(), *initRefPoint),
                                geometry::calc_dist(subfield.tracks.at( trackInds.at(i) ).points.back(), *initRefPoint) );
        trackDistances.insert( std::make_pair( dist, i ) ) ;
    }

    size_t countMachines = 0;
    std::vector<std::pair<size_t, ITrackSequencer::TrackPointsDirection>> ret;
    for(auto& it : trackDistances){
        if(geometry::calc_dist(subfield.tracks.at( trackInds.at(it.second) ).points.front(), *initRefPoint) > geometry::calc_dist(subfield.tracks.at( trackInds.at(it.second) ).points.back(), *initRefPoint))
            ret.push_back( std::make_pair(it.second, ITrackSequencer::REVERSE) );
        else
            ret.push_back( std::make_pair(it.second, ITrackSequencer::FORWARD) );
        ++countMachines;
        if(countMachines >= numMachines)
            break;
    }
    return ret;
}

Point TrackSequencerClosestNext::getSequenceAverageDirection(const Subfield &subfield, const std::vector<TrackInfo> &sequence){
    Point directionPrev(0, 0);
    for(size_t j = 0 ; j+1 < sequence.size() ; ++j){
        auto& infoFrom = sequence.at(j);
        auto& infoTo = sequence.at(j+1);
        auto& trackFrom = subfield.tracks.at( infoFrom.trackIndex ).points;
        auto& trackTo = subfield.tracks.at( infoTo.trackIndex ).points;

        Point directionTmp = geometry::getDirectionBetweenTracks(trackFrom, trackTo);

        directionPrev = directionPrev + directionTmp;
        geometry::setVectorLength(directionPrev, 1);
    }
    return directionPrev;
}

std::shared_ptr<IInfieldTracksConnector> TrackSequencerClosestNext::getDefTracksConnector() const{
    return std::make_shared<InfieldTracksConnectorDef>(loggerPtr());
}

} // namespace arolib
