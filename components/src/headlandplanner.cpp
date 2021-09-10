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
 
#include "arolib/components/headlandplanner.h"
namespace arolib {


const double HeadlandPlanner::m_thresholdIsWorked = 0.5;

HeadlandPlanner::TracksSamplingStrategy HeadlandPlanner::intToTracksSamplingStrategy(int value)
{
    if(value == TracksSamplingStrategy::SIMPLE_TRACK_SAMPLING)
        return TracksSamplingStrategy::SIMPLE_TRACK_SAMPLING;
    else if(value == TracksSamplingStrategy::SAMPLE_TRACKS_PERPENDICULARILY)
        return TracksSamplingStrategy::SAMPLE_TRACKS_PERPENDICULARILY;
    else if(value == TracksSamplingStrategy::SAMPLE_TRACKS_WITH_CLOSEST_SAMPLES)
        return TracksSamplingStrategy::SAMPLE_TRACKS_WITH_CLOSEST_SAMPLES;

    throw std::invalid_argument( "The given value does not correspond to any HeadlandPlanner::TracksSamplingStrategy" );
}


bool HeadlandPlanner::PlannerParameters::parseFromStringMap(HeadlandPlanner::PlannerParameters &params, const std::map<std::string, std::string> &map, bool strict)
{
    HeadlandPlanner::PlannerParameters tmp;

    try{

        if( !FieldGeneralParameters::parseFromStringMap(tmp, map, strict) )
            return false;
        if( !GridComputationSettings::parseFromStringMap(tmp, map, strict) )
            return false;

        int tracksSamplingStrategy = HeadlandPlanner::SAMPLE_TRACKS_PERPENDICULARILY;
        std::map<std::string, double*> dMap = { {"headlandWidth" , &tmp.headlandWidth},
                                                {"widthMultiplier" , &tmp.widthMultiplier},
                                                {"speedMultiplier" , &tmp.speedMultiplier},
                                                {"trackDistance" , &tmp.trackDistance},
                                                {"sampleResolution" , &tmp.sampleResolution} };
        std::map<std::string, size_t*> uiMap = { {"numTracks" , &tmp.numTracks} };
        std::map<std::string, bool*> bMap = { {"clockwise" , &tmp.clockwise},
                                              {"sortMachinesByWidth" , &tmp.sortMachinesByWidth} };
        std::map<std::string, int*> enumMap = { {"tracksSamplingStrategy" , &tracksSamplingStrategy}};

        if( !setValuesFromStringMap( map, dMap, strict)
                || !setValuesFromStringMap( map, uiMap, strict)
                || !setValuesFromStringMap( map, bMap, strict)
                || !setValuesFromStringMap( map, enumMap, strict) )
            return false;

        tmp.tracksSamplingStrategy = HeadlandPlanner::intToTracksSamplingStrategy( tracksSamplingStrategy );

    } catch(...){ return false; }

    params = tmp;

    return true;
}

std::map<std::string, std::string> HeadlandPlanner::PlannerParameters::parseToStringMap(const HeadlandPlanner::PlannerParameters &params)
{
    std::map<std::string, std::string> ret, subMap;
    subMap = FieldGeneralParameters::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );
    subMap = GridComputationSettings::parseToStringMap(params);
    ret.insert( subMap.begin(), subMap.end() );

    ret["headlandWidth"] = double2string( params.headlandWidth );
    ret["widthMultiplier"] = double2string( params.widthMultiplier );
    ret["speedMultiplier"] = double2string( params.speedMultiplier );
    ret["trackDistance"] = double2string( params.trackDistance );
    ret["sampleResolution"] = double2string( params.sampleResolution );
    ret["numTracks"] = std::to_string( params.numTracks );
    ret["clockwise"] = std::to_string( params.clockwise );
    ret["sortMachinesByWidth"] = std::to_string( params.sortMachinesByWidth );
    ret["tracksSamplingStrategy"] = std::to_string( params.tracksSamplingStrategy );

    return ret;
}

HeadlandPlanner::HeadlandPlanner(const LogLevel &logLevel):
    LoggingComponent(logLevel, __FUNCTION__)
{

}


AroResp HeadlandPlanner::planHeadlandSubfield(Subfield &subfield,
                                              const std::vector<Machine> &workinggroup,
                                              const OutFieldInfo &outFieldInfo,
                                              const PlannerParameters &plannerParameters,
                                              std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                              std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                              std::vector<HeadlandRoute> &routes,
                                              const std::vector<Point> &initRefPoint)
{
    if(!edgeMassCalculator){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Invalid edgeMassCalculator.");
        return AroResp(1, "Invalid edgeMassCalculator.");
    }
    if(!edgeSpeedCalculator){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Invalid edgeSpeedCalculator.");
        return AroResp(1, "Invalid edgeSpeedCalculator.");
    }



    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Planning headland for a single subfield");

    routes.clear();

    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction
    LoggingComponent::setTemporalLoggersParent(lh, *this, *edgeMassCalculator, *edgeSpeedCalculator);

    try {
        std::vector<Machine> machines = getAssignedHarvesters(workinggroup);
        if (machines.empty()) {
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "No machines assigned to harvest the headland.");
            //return AroResp(1, "No machines assigned to harvest the headland.");
        }

        if(plannerParameters.sortMachinesByWidth){
            std::sort( machines.begin(),
                       machines.end(),
                       [](const Machine & a, const Machine & b) -> bool { return a.working_width > b.working_width; }
            );
        }

        if(!reorderSubfieldPoints(subfield, nullptr)){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Problem reordering outer boundary points of subfield");
            return AroResp(1, "Problem reordering outer boundary points of subfield");
        }

        Point startingPoint;

        if(initRefPoint.empty())
            startingPoint = getStartingPoint(subfield,
                                             workinggroup,
                                             outFieldInfo);
        else
            startingPoint = initRefPoint.front();


        if(plannerParameters.headlandWidth > 0){//process the field using plannerParameters.headlandWidth and the working width of the assigned machines (if existent)
            if(! processSubfield( subfield,
                                  plannerParameters.headlandWidth,
                                  machines,
                                  edgeMassCalculator,
                                  edgeSpeedCalculator,
                                  routes,
                                  startingPoint,
                                  plannerParameters ) ){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error processing subfield.");
                return AroResp(1, "Error processing subfield");
            }
        }
        else{//process the field using plannerParameters.widthMultiplier and the working width of the assigned machines
            if (plannerParameters.widthMultiplier <= 0) {
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid headland_width and width_mult.");
                return AroResp(1, "Invalid headland_width and width_mult.");
            }
            if (machines.empty()) {
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No assigned machines to calculate the headland width using width_mult.");
                return AroResp(1, "No assigned machines to calculate the headland width using width_mult.");
            }
            if(! processSubfield( subfield,
                                  machines,
                                  edgeMassCalculator,
                                  edgeSpeedCalculator,
                                  routes,
                                  startingPoint,
                                  plannerParameters ) ){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error processing subfield.");
                return AroResp(1, "Error processing subfield");
            }
        }

        return AroResp(0, "OK");
    }
    catch (std::exception &e) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what());
        return AroResp(1, e.what());
    }
}

AroResp HeadlandPlanner::planHeadlandSubfield(Subfield &subfield,
                                                    const std::vector<Machine> &workinggroup,
                                                    const OutFieldInfo &outFieldInfo,
                                                    const HeadlandPlanner::PlannerParameters &plannerParameters,
                                                    const ArolibGrid_t &remainingAreaMap,
                                                    std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                                    std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                                    std::vector<HeadlandRoute> &routes)
{    
    if(!edgeMassCalculator){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Invalid edgeMassCalculator.");
        return AroResp(1, "Invalid edgeMassCalculator.");
    }
    if(!edgeSpeedCalculator){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Invalid edgeSpeedCalculator.");
        return AroResp(1, "Invalid edgeSpeedCalculator.");
    }

    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction
    LoggingComponent::setTemporalLoggersParent(lh, *this, *edgeMassCalculator, *edgeSpeedCalculator, remainingAreaMap);

    AroResp compResp = planHeadlandSubfield(subfield,
                                            workinggroup,
                                            outFieldInfo,
                                            plannerParameters,
                                            edgeMassCalculator,
                                            edgeSpeedCalculator,
                                            routes,
                                            {});
    if(compResp.errorID > 0)
        return compResp;

    Point newInitRefPoint;
    if( replanningNeeded(routes,
                         workinggroup,
                         remainingAreaMap,
                         plannerParameters,
                         newInitRefPoint) ){
        subfield.boundary_inner.points.clear();
        subfield.headlands.complete = CompleteHeadland();
        routes.clear();

        compResp = planHeadlandSubfield(subfield,
                                        workinggroup,
                                        outFieldInfo,
                                        plannerParameters,
                                        edgeMassCalculator,
                                        edgeSpeedCalculator,
                                        routes,
                                        {newInitRefPoint});
    }

    if(compResp.isError())
        return compResp;


    adjustRoutes(routes, workinggroup, remainingAreaMap, plannerParameters);

    return compResp;
}

AroResp HeadlandPlanner::planHeadlandField(Field &field,
                                           const std::vector<Machine> &workinggroup,
                                           const OutFieldInfo &outFieldInfo,
                                           const PlannerParameters& plannerParameters,
                                           std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                           std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                           std::map<int, std::vector<HeadlandRoute> > &routes,
                                           const std::vector<Point> &initRefPoint)
{

    if(!edgeMassCalculator){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Invalid edgeMassCalculator.");
        return AroResp(1, "Invalid edgeMassCalculator.");
    }
    if(!edgeSpeedCalculator){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Invalid edgeSpeedCalculator.");
        return AroResp(1, "Invalid edgeSpeedCalculator.");
    }

    //m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Planning headland for a field");

    routes.clear();

    if(field.subfields.size() == 0){
        Subfield sf;
        sf.boundary_outer = field.outer_boundary;
        field.subfields.push_back(sf);
    }

    std::vector<Machine> machines = getAssignedHarvesters(workinggroup);
    if (machines.empty()) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No machines assigned to harvest the headland.");
        //return AroResp(1, "No machines assigned to harvest the headland.");
    }

    if(plannerParameters.sortMachinesByWidth){
        std::sort( machines.begin(),
                   machines.end(),
                   [](const Machine & a, const Machine & b) -> bool { return a.working_width > b.working_width; }
        );
    }

    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction
    LoggingComponent::setTemporalLoggersParent(lh, *this, *edgeMassCalculator, *edgeSpeedCalculator);

    try {

        for (size_t i = 0; i < field.subfields.size(); ++i) {
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Planning subfield " + std::to_string(i));

            std::vector< HeadlandRoute > sfRoutes;

            Subfield &subfield = field.subfields.at(i);

            arolib::geometry::unsample_polygon(subfield.boundary_outer);
            arolib::geometry::openPolygon(subfield.boundary_outer);
            if(subfield.boundary_outer.points.size() < 3) {
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Boundry of subfield " + std::to_string(i) + " doesn't have enough points");
                return AroResp(1, "Boundry of subfield " + std::to_string(i) + " doesn't have enough points");
            }
            arolib::geometry::closePolygon(subfield.boundary_outer);

            Point startingPoint;

            if (i == 0 || routes[i-1].empty() ){//get the initial reference point for the first subfield (or subsequen subfields if no routes were calculated)
                if(initRefPoint.empty())
                    startingPoint = getStartingPoint(subfield,
                                                     workinggroup,
                                                     outFieldInfo);
                else
                    startingPoint = initRefPoint.front();

//                if(!reorderSubfieldPoints(subfield, nullptr)){
//                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Problem reordering outer boundary points of subfield " + std::to_string(i));
//                    return AroResp(1, "Problem reordering outer boundary points of subfield " + std::to_string(i));
//                }
            }
            else{//the initial reference point for the subfield is obtained based on the routes of the previous subfield
                Point lastRoutePoint = routes[i-1].back().route_points.back().point();
                if(!reorderSubfieldPoints(subfield, &lastRoutePoint)){
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Problem reordering outer boundary points of subfield " + std::to_string(i));
                    return AroResp(1, "Problem reordering outer boundary points of subfield " + std::to_string(i));
                }
                startingPoint = subfield.boundary_outer.points.front();
            }

            if(plannerParameters.headlandWidth > 0){//process the field using plannerParameters.headlandWidth and the working width of the assigned machines (if existent)
                if(! processSubfield( subfield,
                                      plannerParameters.headlandWidth,
                                      machines,
                                      edgeMassCalculator,
                                      edgeSpeedCalculator,
                                      sfRoutes,
                                      startingPoint,
                                      plannerParameters ) ){
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error processing subfield " + std::to_string(i));
                    return AroResp(1, "Error processing subfield " + std::to_string(i));
                }
            }
            else{//process the field using plannerParameters.widthMultiplier and the working width of the assigned machines
                if (plannerParameters.widthMultiplier <= 0) {
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid headland_width and width_multiplier");
                    return AroResp(1, "Invalid headland_width and width_multiplier");
                }
                if (machines.empty()) {
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No assigned machines to calculate the headland width using width_mult.");
                    return AroResp(1, "No assigned machines to calculate the headland width using width_mult.");
                }
                if(! processSubfield(subfield,
                                     machines,
                                     edgeMassCalculator,
                                     edgeSpeedCalculator,
                                     sfRoutes,
                                     startingPoint,
                                     plannerParameters ) ){
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error processing subfield " + std::to_string(i));
                    return AroResp(1, "Error processing subfield " + std::to_string(i));
                }
            }

            //routes[subfield.id] = sfRoutes;
            routes[i] = sfRoutes;
        }
        return AroResp(0, "OK");

    }
    catch (std::exception & e) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what());
        return AroResp(1, e.what());
    }

}

AroResp HeadlandPlanner::planHeadlandField(Field &field,
                                           const std::vector<Machine> &workinggroup,
                                           const OutFieldInfo &outFieldInfo,
                                           const HeadlandPlanner::PlannerParameters &plannerParameters,
                                           const ArolibGrid_t &remainingAreaMap,
                                           std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                           std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                           std::map<int, std::vector<HeadlandRoute> > &routes)
{
    if(!edgeMassCalculator){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Invalid edgeMassCalculator.");
        return AroResp(1, "Invalid edgeMassCalculator.");
    }
    if(!edgeSpeedCalculator){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Invalid edgeSpeedCalculator.");
        return AroResp(1, "Invalid edgeSpeedCalculator.");
    }

    AroResp compResp = planHeadlandField(field,
                                         workinggroup,
                                         outFieldInfo,
                                         plannerParameters,
                                         edgeMassCalculator,
                                         edgeSpeedCalculator,
                                         routes,
                                         {});
    if(compResp.errorID > 0 || routes.empty())
        return compResp;

    LoggingComponent::LoggersHandler lh(true);//will be reset on destruction
    LoggingComponent::setTemporalLoggersParent(lh, *this, *edgeMassCalculator, *edgeSpeedCalculator);

    for(auto &routes_it : routes){
        if(routes_it.first >= (int)field.subfields.size())
            continue;
        std::vector<HeadlandRoute> &routes = routes_it.second;
        Subfield& subfield = field.subfields.at(routes_it.first);
        Point newInitRefPoint;
        if( replanningNeeded(routes,
                             workinggroup,
                             remainingAreaMap,
                             plannerParameters,
                             newInitRefPoint) ){

            subfield.boundary_inner.points.clear();
            subfield.headlands.complete = CompleteHeadland();
            routes.clear();

            compResp = planHeadlandSubfield(subfield,
                                            workinggroup,
                                            outFieldInfo,
                                            plannerParameters,
                                            edgeMassCalculator,
                                            edgeSpeedCalculator,
                                            routes,
                                            {newInitRefPoint});
            if(compResp.errorID > 0)
                return compResp;

            adjustRoutes(routes, workinggroup, remainingAreaMap, plannerParameters);
        }
    }
    return compResp;

}

AroResp HeadlandPlanner::planHeadland(PlanningWorkspace &pw,
                                      const HeadlandPlanner::PlannerParameters &plannerParameters,
                                      std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                      std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                      int subfieldIdx,
                                      const std::vector<Point>& initRefPoint)
{
    if(!edgeMassCalculator){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Invalid edgeMassCalculator.");
        return AroResp(1, "Invalid edgeMassCalculator.");
    }
    if(!edgeSpeedCalculator){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Invalid edgeSpeedCalculator.");
        return AroResp(1, "Invalid edgeSpeedCalculator.");
    }

    auto& field = getField(pw);
    auto& maps = getMaps(pw);
    if(subfieldIdx >= (int)field.subfields.size()){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid subfield index " + std::to_string(subfieldIdx) + " >= " + std::to_string(field.subfields.size()) );
        return AroResp(1, "Invalid subfield index.");
    }
    if(subfieldIdx < 0){
        if( maps.at(PlanningWorkspace::GridType::REMAINING_AREA).isAllocated() ){
            auto aroResp = processField(pw, plannerParameters, edgeMassCalculator, edgeSpeedCalculator);
            if(!aroResp.isError())
                adjustRoutes(pw, subfieldIdx, plannerParameters);
            return aroResp;
        }
        return processField(pw, plannerParameters, edgeMassCalculator, edgeSpeedCalculator, initRefPoint);
    }
    else{

        m_calcGridValueOption = CALC_PW;
        m_planningWorkspace = &pw;

        AroResp aroResp;

        if( maps.at(PlanningWorkspace::GridType::REMAINING_AREA).isAllocated() ){
            aroResp = processSubfield_pw(subfieldIdx,
                                         plannerParameters,
                                         edgeMassCalculator,
                                         edgeSpeedCalculator);

            if(!aroResp.isError())
                adjustRoutes(pw, subfieldIdx, plannerParameters);
        }
        else
            aroResp = processSubfield_pw(subfieldIdx,
                                         plannerParameters,
                                         edgeMassCalculator,
                                         edgeSpeedCalculator,
                                         initRefPoint);

        m_planningWorkspace = nullptr;
        m_calcGridValueOption = CALC_DIRECT;
        return aroResp;
    }

}

std::vector<Machine> HeadlandPlanner::getAssignedHarvesters(const std::vector<Machine> &machines) const
{
    std::vector<Machine> ret;
    for(auto &m : machines){
        if( m.isOfWorkingType(true)
                && m.machineassignment != Machine::INFIELD )
            ret.push_back(m);
    }
    return ret;
}

double HeadlandPlanner::getHeadlandWidth(const HeadlandPlanner::PlannerParameters &plannerParameters,
                                         const std::vector<Machine> &machines)
{

    if(plannerParameters.headlandWidth > 0)
        return plannerParameters.headlandWidth;

    if (plannerParameters.widthMultiplier <= 0) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid headland_width and width_multiplier");
        return -1;
    }
    double headland_width = 0;
    for (size_t i = 0 ; i < machines.size() ; ++i)
        headland_width += machines.at(i).working_width;
    headland_width *= plannerParameters.widthMultiplier;


    if (headland_width < 1e-6){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid headland width calculated from the machines working widths and the widthMultiplier");
        return -1;
    }

    return headland_width;
}

bool HeadlandPlanner::reorderSubfieldPoints(Subfield &sf, const Point *lastRoutePoint){
    if(sf.boundary_outer.points.size() < 3)
        return false;

    arolib::geometry::openPolygon(sf.boundary_outer);
    size_t indSP = 0;
    double distSP = std::numeric_limits<double>::max();

    //Get the point in the outer boundary that is closest to a field access point
    for(size_t i = 0 ; i < sf.access_points.size() ; ++i){
        size_t ind = arolib::geometry::getPointIndInMinDist(sf.boundary_outer.points, sf.access_points.at(i));
        double dist = arolib::geometry::calc_dist(sf.boundary_outer.points.at(ind), sf.access_points.at(i));
        if(distSP > dist){
            indSP = ind;
            distSP = dist;
        }

    }

    //Get the point in the outer boundary that is closest to the given lastRoutePoint (iif the distance is shorter than the one for the closest field access point)
    if(lastRoutePoint){
        size_t ind = arolib::geometry::getPointIndInMinDist(sf.boundary_outer.points, *lastRoutePoint);
        double dist = arolib::geometry::calc_dist(sf.boundary_outer.points.at(ind), *lastRoutePoint);
        if(distSP > dist){
            indSP = ind;
            distSP = dist;
        }
    }

    //reorder the outer boundary points so that the first point is the previously calculated closest point
    if(indSP != 0){
        std::vector<Point> pointsTmp;
        pointsTmp.insert( pointsTmp.end(), sf.boundary_outer.points.begin(), sf.boundary_outer.points.begin()+indSP );
        sf.boundary_outer.points.erase( sf.boundary_outer.points.begin(), sf.boundary_outer.points.begin()+indSP );
        sf.boundary_outer.points.insert( sf.boundary_outer.points.end(), pointsTmp.begin(), pointsTmp.end() );
    }
    arolib::geometry::closePolygon(sf.boundary_outer);

    return true;

}

Point HeadlandPlanner::getStartingPoint(const Subfield &sf,
                                        const std::vector<Machine> &workinggroup,
                                        const OutFieldInfo &outFieldInfo)
{
    Point startingPoint = sf.boundary_outer.points.front();

    double minGlobalArrivalTime = std::numeric_limits<double>::max();
    bool found = false;

    //obtain the field access point to which all the assigned harvesters would arrive first
    for(auto &fap : sf.access_points){
        double maxLocalArrivalTime = -1;
        if(outFieldInfo.size_arrivalCosts( fap.id ) == 0)
            continue;

        //get the maximum arrival time (for the assigned harvesters) to the field access point
        for(auto &m : workinggroup){
            if(m.isOfWorkingType(true)
                    || m.machineassignment == Machine::INFIELD)
                continue;
            OutFieldInfo::TravelCosts arrivalCosts;
            if(outFieldInfo.getArrivalCost(fap.id, m.id, OutFieldInfo::MACHINE_EMPTY, arrivalCosts))
                maxLocalArrivalTime = std::max(maxLocalArrivalTime, arrivalCosts.time);
        }

        //update the overall minimum arrival time for all harvesters and the corresponding field access point
        if(maxLocalArrivalTime >= -0.001 &&
                minGlobalArrivalTime > maxLocalArrivalTime){
            minGlobalArrivalTime = maxLocalArrivalTime;
            startingPoint = fap.point();
            found = true;
        }
    }

    if(!found){
        //obtain the field access point closest to the first boundary point
        for(auto &fap : sf.access_points){
            double dist = arolib::geometry::calc_dist(sf.boundary_outer.points.front(), fap);
            //update the overall minimum arrival time for all harvesters and the corresponding field access point
            if(minGlobalArrivalTime > dist){
                minGlobalArrivalTime = dist;
                startingPoint = fap.point();
            }
        }
    }

    return startingPoint;

}

AroResp HeadlandPlanner::processField(PlanningWorkspace &pw,
                                            const HeadlandPlanner::PlannerParameters &plannerParameters, std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator, std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                            const std::vector<Point> &initRefPoint)
{

    //m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Planning headland for a field");

    auto &field = getField(pw);
    auto &baseRoutes_headland = getBaseRoutes_headland(pw);
    auto &machines_all = getMachines(pw);
    auto &outFieldInfo = getOutFieldInfo(pw);

    baseRoutes_headland.clear();

    if(field.subfields.size() == 0){
        Subfield sf;
        sf.boundary_outer = field.outer_boundary;
        field.subfields.push_back(sf);
    }

    std::vector<Machine> machines = getAssignedHarvesters(machines_all);
    if (machines.empty()) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "No machines assigned to harvest the headland.");
        //return AroResp(1, "No machines assigned to harvest the headland.");
    }

    if(plannerParameters.sortMachinesByWidth){
        std::sort( machines.begin(),
                   machines.end(),
                   [](const Machine & a, const Machine & b) -> bool { return a.working_width > b.working_width; }
        );
    }

    try {

        for (size_t i = 0; i < field.subfields.size(); ++i) {
            m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Planning subfield " + std::to_string(i));

            Subfield &subfield = field.subfields.at(i);

            arolib::geometry::unsample_polygon(subfield.boundary_outer);
            arolib::geometry::openPolygon(subfield.boundary_outer);
            if(subfield.boundary_outer.points.size() < 3) {
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Boundry of subfield " + std::to_string(i) + " doesn't have enough points");
                return AroResp(1, "Boundry of subfield " + std::to_string(i) + " doesn't have enough points");
            }
            arolib::geometry::closePolygon(subfield.boundary_outer);

            Point startingPoint;

            if (i == 0 || baseRoutes_headland[i-1].empty() ){//get the initial reference point for the first subfield (or subsequen subfields if no routes were calculated)

                if(initRefPoint.empty())
                    startingPoint = getStartingPoint(subfield,
                                                     machines_all,
                                                     outFieldInfo);
                else
                    startingPoint = initRefPoint.front();

//                if(!reorderSubfieldPoints(subfield, nullptr)){
//                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Problem reordering outer boundary points of subfield " + std::to_string(i));
//                    return AroResp(1, "Problem reordering outer boundary points of subfield " + std::to_string(i));
//                }
            }
            else{
                Point lastRoutePoint = baseRoutes_headland[i-1].back().route_points.back().point();
                if(!reorderSubfieldPoints(subfield, &lastRoutePoint)){
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Problem reordering outer boundary points of subfield " + std::to_string(i));
                    return AroResp(1, "Problem reordering outer boundary points of subfield " + std::to_string(i));
                }
                startingPoint = subfield.boundary_outer.points.front();
            }


            m_calcGridValueOption = CALC_PW;
            m_planningWorkspace = &pw;

            AroResp compResp = processSubfield_pw( i,
                                                   plannerParameters,
                                                   edgeMassCalculator,
                                                   edgeSpeedCalculator,
                                                   {startingPoint} );

            m_planningWorkspace = nullptr;
            m_calcGridValueOption = CALC_DIRECT;

            if(compResp.errorID > 0 || baseRoutes_headland.empty()){
                m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error processing subfield " + std::to_string(i) + ": " + compResp.msg);
                return AroResp(1, "Error processing subfield " + std::to_string(i) + ": " + compResp.msg);
            }
        }

    } catch (std::exception &e) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what());
        return AroResp(1, e.what());
    }

    return AroResp(0, "OK");

}

AroResp HeadlandPlanner::processField(PlanningWorkspace &pw,
                                      const HeadlandPlanner::PlannerParameters &plannerParameters,
                                      std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                      std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator)
{
    AroResp compResp;
    bool needsInitialProcessing = true;

    auto& field = getField(pw);
    auto& machines = getMachines(pw);
    auto& maps = getMaps(pw);
    auto &baseRoutes_headland = getBaseRoutes_headland(pw);

    if( maps.at( PlanningWorkspace::GridType::REMAINING_AREA ).isAllocated() ){
        needsInitialProcessing = false;
        for(size_t i = 0 ; i < field.subfields.size() ; ++i){
            auto it_routes = baseRoutes_headland.find(i);
            if(it_routes == baseRoutes_headland.end() || it_routes->second.empty()){
                needsInitialProcessing = true;
                break;
            }
        }
    }

    if(needsInitialProcessing){
        baseRoutes_headland.clear();
        compResp = processField(pw,
                                plannerParameters,
                                edgeMassCalculator,
                                edgeSpeedCalculator,
                                {});
        if(compResp.errorID > 0 || baseRoutes_headland.empty())
            return compResp;
    }


    m_calcGridValueOption = CALC_PW;
    m_planningWorkspace = &pw;

    m_planningWorkspace = nullptr;
    m_calcGridValueOption = CALC_DIRECT;

    for(auto &routes_it : baseRoutes_headland){
        if(routes_it.first >= (int)field.subfields.size())
            continue;
        std::vector<HeadlandRoute> &routes = routes_it.second;
        Subfield& subfield = field.subfields.at(routes_it.first);
        Point newInitRefPoint;
        if( replanningNeeded(routes,
                             machines,
                             maps.at( PlanningWorkspace::GridType::REMAINING_AREA ),
                             plannerParameters,
                             newInitRefPoint) ){

            subfield.boundary_inner.points.clear();
            subfield.headlands.complete = CompleteHeadland();
            routes.clear();//@TODO will the new route points correspond to the previos edge points?

            compResp = processSubfield_pw( routes_it.first,
                                           plannerParameters,
                                           edgeMassCalculator,
                                           edgeSpeedCalculator,
                                           {newInitRefPoint} );
            if(compResp.errorID > 0)
                break;
        }
    }

    m_planningWorkspace = nullptr;
    m_calcGridValueOption = CALC_DIRECT;

    return compResp;

}

AroResp HeadlandPlanner::processSubfield_pw(size_t subfieldIdx,
                                            const PlannerParameters &plannerParameters,
                                            std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                            std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                            const std::vector<Point> &initRefPoint)
{

    auto& pw = *m_planningWorkspace;
    auto& field = getField(pw);
    auto& machines_all = getMachines(pw);
    auto& outFieldInfo = getOutFieldInfo(pw);
    auto& baseRoutes_headland = getBaseRoutes_headland(pw);

    m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Planning headland for a single subfield");

    if(subfieldIdx >= field.subfields.size()){
        m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Invalid subfield index.");
        return AroResp(1, "Invalid subfield index.");
    }
    Subfield& subfield = field.subfields.at(subfieldIdx);

    std::vector<HeadlandRoute>& routes = baseRoutes_headland[subfieldIdx];

    routes.clear();

    try {
        std::vector<Machine> machines = getAssignedHarvesters(machines_all);
        if (machines.empty()) {
            m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "No machines assigned to harvest the headland.");
            //return AroResp(1, "No machines assigned to harvest the headland.");
        }

        if(plannerParameters.sortMachinesByWidth){
            std::sort( machines.begin(),
                       machines.end(),
                       [](const Machine & a, const Machine & b) -> bool { return a.working_width > b.working_width; }
            );
        }

        if(!reorderSubfieldPoints(subfield, nullptr)){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Problem reordering outer boundary points of subfield");
            return AroResp(1, "Problem reordering outer boundary points of subfield");
        }

        Point startingPoint;

        if(initRefPoint.empty())
            startingPoint = getStartingPoint(subfield,
                                             machines_all,
                                             outFieldInfo);
        else
            startingPoint = initRefPoint.front();

        double headlandWidth = getHeadlandWidth(plannerParameters, machines);
        if(headlandWidth < 0)
            return AroResp(1, "Invalid headland_width and width_multiplier");

        AroResp compResp = processSubfield_pw(subfieldIdx,
                                              plannerParameters,
                                              headlandWidth,
                                              machines,
                                              edgeMassCalculator,
                                              edgeSpeedCalculator,
                                              startingPoint);
        if(compResp.errorID > 0)
            return compResp;


    } catch (std::exception &e) {
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, std::string("Exception cought: ") + e.what());
        return AroResp(1, e.what());
    }
    return AroResp(0, "OK");

}

AroResp HeadlandPlanner::processSubfield_pw(size_t subfieldIdx,
                                            const HeadlandPlanner::PlannerParameters &plannerParameters,
                                            std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                            std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator)
{
    auto& pw = *m_planningWorkspace;
    auto& field = getField(pw);
    auto& machines_all = getMachines(pw);
    auto& baseRoutes_headland = getBaseRoutes_headland(pw);
    auto& maps = getMaps(pw);

    AroResp compResp;
    bool needsInitialProcessing = true;

    if( maps.at( PlanningWorkspace::GridType::REMAINING_AREA ).isAllocated() ){
        auto it_routes = baseRoutes_headland.find(subfieldIdx);
        if(it_routes != baseRoutes_headland.end() && !it_routes->second.empty())
            needsInitialProcessing = false;
    }

    if(needsInitialProcessing){
        baseRoutes_headland[subfieldIdx].clear();
        AroResp compResp = processSubfield_pw(subfieldIdx,
                                              plannerParameters,
                                              edgeMassCalculator,
                                              edgeSpeedCalculator,
                                              {});
        if(compResp.errorID > 0 || baseRoutes_headland.empty())
            return compResp;
    }

    Subfield &subfield = field.subfields.at(subfieldIdx);
    std::vector<HeadlandRoute> &routes = baseRoutes_headland[subfieldIdx];

    Point newInitRefPoint;
    if( replanningNeeded(baseRoutes_headland.at(subfieldIdx),
                         machines_all,
                         maps.at(PlanningWorkspace::GridType::REMAINING_AREA),
                         plannerParameters,
                         newInitRefPoint) ){

        subfield.boundary_inner.points.clear();
        subfield.headlands.complete = CompleteHeadland();
        routes.clear();//@TODO will the new route points correspond to the previos edge points?

        return compResp = processSubfield_pw(subfieldIdx,
                                             plannerParameters,
                                             edgeMassCalculator,
                                             edgeSpeedCalculator,
                                             {newInitRefPoint});
    }
    return compResp;

}

AroResp HeadlandPlanner::processSubfield_pw(size_t subfieldIdx,
                                            const HeadlandPlanner::PlannerParameters &plannerParameters,
                                            double headlandWidth,
                                            const std::vector<Machine> &machines,
                                            std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                            std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                            Point startingPoint)
{

    auto& pw = *m_planningWorkspace;
    auto& field = getField(pw);
    auto& baseRoutes_headland = getBaseRoutes_headland(pw);
    auto& maps = getMaps(pw);

    Subfield &subfield = field.subfields.at(subfieldIdx);
    std::vector< HeadlandRoute > &routes = baseRoutes_headland[subfieldIdx];

    bool ret = processSubfield(subfield,
                               headlandWidth,
                               machines,
                               edgeMassCalculator,
                               edgeSpeedCalculator,
                               routes,
                               startingPoint,
                               plannerParameters);

    if(!ret){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error processing subfield.");
        return AroResp(1, "Error processing subfield");
    }

    return AroResp(0, "OK");
}

bool HeadlandPlanner::processSubfield(Subfield &subfield,
                                      double headlandWidth,
                                      const std::vector<Machine> &machines,
                                      std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                      std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                      std::vector<HeadlandRoute> &routes,
                                      Point startingPoint,
                                      const PlannerParameters &settings) {


    subfield.headlands.complete.tracks.clear();

    if(subfield.boundary_outer.points.size() < 3)
        return false;

    std::vector< std::tuple<Polygon, size_t, int> > tracks; //<tracks, machine_ind, trackId>


    arolib::geometry::openPolygon(subfield.boundary_outer);

    //get the index of the point in the outer boundary that is closest to the initial reference point and reorder the boundary points if needed
    size_t indSP = arolib::geometry::getPointIndInMinDist(subfield.boundary_outer.points, startingPoint);
    if(indSP != 0){
        std::vector<Point> pointsTmp;
        pointsTmp.insert( pointsTmp.end(), subfield.boundary_outer.points.begin(), subfield.boundary_outer.points.begin()+indSP );
        subfield.boundary_outer.points.erase( subfield.boundary_outer.points.begin(), subfield.boundary_outer.points.begin()+indSP );
        subfield.boundary_outer.points.insert( subfield.boundary_outer.points.end(), pointsTmp.begin(), pointsTmp.end() );
    }
    arolib::geometry::closePolygon(subfield.boundary_outer);

    bool offsetRoutesFromOuterBoundary = true;

    double totalWidth = headlandWidth;

    double limitExpandHeadlandWidth = 0.95;//multiplier (95%) used later to see if an extra track is needed

    bool tracksOK = machines.empty();//if no machines are asigned, no routes will be generated, but it is acceptable

    std::vector<Machine> machinesTmp;
    if(!machines.empty())
        machinesTmp = machines;
    else if (headlandWidth > 1e-5){
        if (settings.numTracks > 0){
            machinesTmp.emplace_back(Machine());
            machinesTmp.back().working_width = headlandWidth / settings.numTracks;
        }
        else if(settings.trackDistance > 0){
            machinesTmp.emplace_back(Machine());
            machinesTmp.back().working_width = settings.trackDistance;
        }
    }

    if(!machinesTmp.empty()){
        tracksOK = true;

        bool lastTrack = false;
        double remainingWidth;
        totalWidth = 0;
        int trackId = Track::DeltaHLTrackId;//start at 'DeltaHLTrackId' to avoid problems with inner field tracks
        unsigned int machine_ind = 0;
        const Machine* machine;
        Polygon outer_tmp = subfield.boundary_outer, inner_tmp;

        while (totalWidth < headlandWidth){//we must ensure that the final headland width is >= to the given headlandWidth
            machine = &machinesTmp.at(machine_ind);//get next machine in queue

            tracks.emplace_back( std::make_tuple( Polygon() , machine_ind , trackId ) );
            auto& headlandTrack_tmp = std::get<0>(tracks.back());

            if(offsetRoutesFromOuterBoundary){//the new track is generated using the outer boundary polygone and the corresponding offset distance (sum of current and previous machine working widths)
                if ( !offsetBoundary(subfield.boundary_outer,
                                     headlandTrack_tmp,
                                     totalWidth + machine->working_width * 0.5,
                                     false) ){
                    tracksOK = false;

                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting the boundary of track " + std::to_string(trackId) + " to obtain the headland route track. Routes are incomplete!");

                    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "outer boundary - track");
                    for (size_t j = 0; j < subfield.boundary_outer.points.size(); ++j)
                        m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "   " + subfield.boundary_outer.points.at(j).toString(30) );
                    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "headlandTrack_tmp");
                    for (size_t j = 0; j < headlandTrack_tmp.points.size(); ++j)
                        m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "   " + headlandTrack_tmp.points.at(j).toString(30) );
                    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "dist = " + std::to_string(totalWidth + machine->working_width * 0.5) + "\n");

                    totalWidth = headlandWidth;
                    tracks.pop_back();
                    break;
                }

            }
            else{//the new track is generated using the previously generated track and the corresponding offset distance (current machine working width)
                if ( !offsetBoundary(outer_tmp,
                                     headlandTrack_tmp,
                                     machine->working_width * 0.5,
                                     false) ){
                    tracksOK = false;
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting the boundary of track " + std::to_string(trackId) + " to obtain the headland route track. Routes are incomplete!");

                    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "outer boundary (tmp) - track");
                    for (size_t j = 0; j < outer_tmp.points.size(); ++j)
                        m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "   " + outer_tmp.points.at(j).toString(30) );
                    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "headlandTrack_tmp");
                    for (size_t j = 0; j < headlandTrack_tmp.points.size(); ++j)
                        m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "   " + headlandTrack_tmp.points.at(j).toString(30) );
                    m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "dist = " + std::to_string(machine->working_width * 0.5) + "\n");

                    totalWidth = headlandWidth;
                    tracks.pop_back();
                    break;
                }

                if(!lastTrack){//calculate the reference boundary to be used to generate the next track
                    if ( !offsetBoundary(outer_tmp,
                                         inner_tmp,
                                         machine->working_width,
                                         false) ){
                        tracksOK = false;
                        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting the boundary of track " + std::to_string(trackId) + " to obtain the headland route track (limit). Routes are incomplete!");

                        m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "outer boundary (tmp) - track");
                        for (size_t j = 0; j < outer_tmp.points.size(); ++j)
                            m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "   " + outer_tmp.points.at(j).toString(30) );
                        m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "inner_tmp");
                        for (size_t j = 0; j < headlandTrack_tmp.points.size(); ++j)
                            m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "   " + inner_tmp.points.at(j).toString(30) );
                        m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "dist = " + std::to_string(machine->working_width) + "\n");

                        totalWidth = headlandWidth;
                        tracks.pop_back();
                        break;
                    }
                }

            }

            arolib::geometry::unsample_polygon(headlandTrack_tmp, 0.1);//the sampling is done later

            trackId++;
            outer_tmp = inner_tmp;
            totalWidth += machine->working_width;
            remainingWidth = headlandWidth - totalWidth;

            m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, std::string("Adding track...\n") +
                                                                         "     trackId : " + std::to_string(trackId-1) + "\n" +
                                                                         "     machine_ind : " + std::to_string(machine_ind) + "\n" +
                                                                         "     headland_width : " + std::to_string(headlandWidth) + "\n" +
                                                                         "     totalWidth : " + std::to_string(totalWidth) + "\n" +
                                                                         "     remainingWidth : " + std::to_string(remainingWidth) + "\n" );

            machine_ind = (machine_ind+1) % machinesTmp.size();//update the index for the next machine in queue

            //check if the next track to be generated is the last track, based on the machine's working width and remaining headland width
            if( machinesTmp.at(machine_ind).working_width <= remainingWidth ){
                m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, std::string("Switching to next machine...\n") +
                         "     remainingWidth : " + std::to_string(remainingWidth) + "\n" +
                         "     next machine working_width : " + std::to_string(machinesTmp.at(machine_ind).working_width) + "\n" );
                continue;
            }

            lastTrack = true;

            if( totalWidth >= headlandWidth ){
                m_logger.printOut(LogLevel::INFO, __FUNCTION__, "Finished generating tracks.\n");
                break;
            }

            //search for the best machine to make the final headland track, based on the machines' working widths and remaining headland width
            bool foundMachine = false;

            //first check the machines that aren't used at the moment
            for( size_t i = machine_ind ; i < machinesTmp.size() ; ++i){
                machine = &machinesTmp.at(i);
                if( machine->working_width >= remainingWidth &&
                        (machine->working_width - remainingWidth) < limitExpandHeadlandWidth*machine->working_width){
                    machine_ind = i;
                    for( size_t j = i+1 ; j < machinesTmp.size() ; ++j){
                        const Machine *machine_2 = &machinesTmp.at(j);
                        if( machine_2->working_width >= remainingWidth &&
                                machine_2->working_width < machine->working_width){
                            machine = machine_2;
                            machine_ind = j;
                        }
                    }
                    foundMachine = true;
                    break;
                }
            }

            if(foundMachine){
                m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Adding last headland route with machine " + std::to_string(machine_ind) + " (next machine)...\n");
                continue;
            }

            //check the rest of the machines
            for( size_t i = 0 ; i < machine_ind ; ++i){
                machine = &machinesTmp.at(i);
                if( machine->working_width >= remainingWidth &&
                        (machine->working_width - remainingWidth) < limitExpandHeadlandWidth*machine->working_width){
                    machine_ind = i;
                    for( size_t j = i+1 ; j < machine_ind ; ++j){
                        const Machine *machine_2 = &machinesTmp.at(j);
                        if( machine_2->working_width >= remainingWidth &&
                                machine_2->working_width < machine->working_width){
                            machine = machine_2;
                            machine_ind = j;
                        }
                    }
                    foundMachine = true;
                    break;
                }
            }

            if(!foundMachine){
                m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "No machine found for last headland route.\n");
                break;
            }

            m_logger.printOut(LogLevel::DEBUG, __FUNCTION__, "Adding last headland route with machine " + std::to_string(machine_ind) + " (best-fitting machine)...\n");
        }
    }

    //obtain the inner boundary from the outer boundary using the resulting headland width
    if(!offsetBoundary(subfield.boundary_outer,
                       subfield.boundary_inner,
                       totalWidth,
                       false) ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting the outer boundary to obtain the inner boundary");
        return false;
    }

    //obtain the planned headland (linestring between the outer and the inner boundaries) from the outer boundary using the resulting headland width / 2
    if(!offsetBoundary(subfield.boundary_outer,
                       subfield.headlands.complete.middle_track,
                       totalWidth*0.5,
                       false) ){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error offsetting the outer boundary to obtain the headland");
        return false;
    }

    //generate the sampled tracks and harvester headland routes
    generateTracksAndRoutes(subfield,
                            machines,
                            tracks,
                            edgeMassCalculator,
                            edgeSpeedCalculator,
                            startingPoint,
                            totalWidth,//has to be this one and not the one from the parameters
                            settings,
                            routes);

    subfield.headlands.complete.headlandWidth = totalWidth;
    subfield.headlands.complete.boundaries.first = subfield.boundary_outer;
    subfield.headlands.complete.boundaries.second = subfield.boundary_inner;

    return tracksOK;

}

bool HeadlandPlanner::processSubfield(Subfield &subfield,
                                      const std::vector<Machine> &machines,
                                      std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                      std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                      std::vector<HeadlandRoute> &routes,
                                      Point startingPoint,
                                      const PlannerParameters &settings) {

    double headland_width = 0;
    for (size_t i = 0 ; i < machines.size() ; ++i)
        headland_width += machines.at(i).working_width;
    headland_width *= settings.widthMultiplier;

    if (headland_width < 1e-6){
        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid headland width calculated from the machines working widths and the widthMultiplier");
        return false;
    }

    return processSubfield(subfield,
                           headland_width,
                           machines,
                           edgeMassCalculator,
                           edgeSpeedCalculator,
                           routes,
                           startingPoint,
                           settings);
}

void HeadlandPlanner::adjustRoutes(std::vector<HeadlandRoute> &routes,
                                   const std::vector<Machine> &machines,
                                   const ArolibGrid_t &remainingAreaMap,
                                   const PlannerParameters &plannerParameters)
{
    if(!remainingAreaMap.isAllocated())
        return;

    std::map<MachineId_t, double> widths;//map containing the machine working widths (for easy access)
    for(auto &m : machines)
        widths[m.id] = m.working_width;
    std::vector< size_t > firstGoodIndexes(routes.size(), 0);//vector containing the route point index of the first segment to be worked
    size_t indFirstGood = -1;
    double deltaTime = -1;

    //search the route point index of the first segment to be worked (i.e. has biomass and is not worked) for each route
    for(size_t i = 0 ; i < routes.size() ; ++i){
        auto &route = routes.at(i);
        auto it_width = widths.find(route.machine_id);
        if(it_width == widths.end())
            continue;

        //get the first index in the route that is not worked (excluding inter-track (headland) connections)
        for(size_t j = 0 ; j+1 < route.route_points.size() ; ++j){
            auto& p0 = route.route_points.at(j);
            auto& p1 = route.route_points.at(j+1);
            if(p0.type != RoutePoint::TRACK_START && !p0.isOfTypeWorking_InTrack(true))
                continue;

            if( isWorked(p0, p1, it_width->second, remainingAreaMap, plannerParameters.bePreciseWithRemainingAreaMap) )
                continue;
            firstGoodIndexes.at(i) = j;
            break;
        }
    }

    //if the first good index for all routes is 0, nothing has been worked
    bool continueChecking = false;
    for(auto & fgi : firstGoodIndexes){
        if(fgi > 0){
            continueChecking = true;
            break;
        }
    }
    if(!continueChecking)
        return;

    //obtain the timestamp of the first good index in all routes
    for(size_t i = 0 ; i < firstGoodIndexes.size() ; ++i){
        auto &route = routes.at(i);
        auto& rp = route.route_points.at(firstGoodIndexes.at(i));
        if(deltaTime < 0 || deltaTime > rp.time_stamp ){
            deltaTime = rp.time_stamp;
            indFirstGood = i;
        }
    }

    //adjust the timestamps with the obtained deltaTime, setting the timestamps of the points before that deltatime to -1 (i.e. these point swill be read as 'worked' by the following planners)
    if(deltaTime > 0){
        for(size_t i = 0 ; i < routes.size() ; ++i){
            auto &route = routes.at(i);
            for(size_t j = 0 ; j < route.route_points.size() ; ++j){
                auto& rp = route.route_points.at(j);
                rp.time_stamp -= deltaTime;
                if(rp.time_stamp < -1e-5)
                    rp.time_stamp = -1;
            }
        }
    }

}

void HeadlandPlanner::adjustRoutes(PlanningWorkspace &pw,
                                   size_t subfieldIdx,
                                   const PlannerParameters &plannerParameters)
{
    if(subfieldIdx >= pw.getField().subfields.size())
        return;


    auto& machines = getMachines(pw);
    auto& routes = getBaseRoutes_headland(pw);
    auto& remainingAreaMap = getMaps(pw).at(PlanningWorkspace::GridType::REMAINING_AREA);

    if(subfieldIdx >= 0){
        auto it_sf = routes.find(subfieldIdx);
        if(it_sf == routes.end())
            return;
        adjustRoutes(it_sf->second, machines, remainingAreaMap, plannerParameters);
        return;
    }

    for(auto& it_sf : routes)
        adjustRoutes(it_sf.second, machines, remainingAreaMap, plannerParameters);
}

bool HeadlandPlanner::offsetBoundary(const Polygon &boundary_in,
                                     Polygon &boundary_out,
                                     double offset,
                                     bool inflated)
{
    return arolib::geometry::offsetPolygon(boundary_in, boundary_out, offset, inflated);
}

void HeadlandPlanner::generateTracksAndRoutes(Subfield &subfield,
                                              const std::vector<Machine> &machines,
                                              std::vector<std::tuple<Polygon, size_t, int> >& tracks,
                                              std::shared_ptr<IEdgeMassCalculator> edgeMassCalculator,
                                              std::shared_ptr<IEdgeSpeedCalculator> edgeSpeedCalculator,
                                              const Point& startingPoint,
                                              double headlandWidth,
                                              const PlannerParameters &settings,
                                              std::vector<HeadlandRoute> &routes)
{
    routes.clear();
    Point lastHeadlandTrackPoint = startingPoint;

    std::vector< HeadlandRouteAssembler > routeAss;
    for( auto& m :  machines ){
        routeAss.emplace_back( RoutePoint::getDefaultRPType(m), m_logger.logLevel() );
        routeAss.back().logger().setParent(&m_logger);
        routeAss.back().setWorkingWidth(m.working_width);
        routeAss.back().setIDs( m.id, routeAss.size()+1000-1 );
    }

    bool reverse = (arolib::geometry::isPolygonClockwise(subfield.boundary_outer) != settings.clockwise);


    double speed_mult = settings.speedMultiplier;

    if(speed_mult <= 0)
        speed_mult = 0.5; //@TODO check this value!

    double res = settings.sampleResolution;
    if(res < 1e-3){//set the resoulution as the shortest machine length
        res = std::numeric_limits<double>::max();
        for(auto& m : machines)
            res = std::min(res, m.length);
    }
    if(res < 1e-3 || res > std::numeric_limits<double>::max()-1){//no valid machine length found --> set the resoulution as the shortest machine size
        res = std::numeric_limits<double>::max();
        for(auto& m : machines)
            res = std::min(res, m.workingRadius());
    }
    if(res < 1e-3 || res > std::numeric_limits<double>::max()-1){//no valid machine length found --> set default
        res = 5;
        //res = 10;
    }

    if(settings.tracksSamplingStrategy == TracksSamplingStrategy::SAMPLE_TRACKS_PERPENDICULARILY)
        sampleTracksPerpendicularly(tracks, res, headlandWidth);
    else if(settings.tracksSamplingStrategy == TracksSamplingStrategy::SAMPLE_TRACKS_WITH_CLOSEST_SAMPLES)
        sampleTracksWithClosestPoints(tracks, res);


    for(auto & track : tracks){
        Polygon &poly = std::get<0>(track);
        int trackId = std::get<2>(track);

        if(settings.tracksSamplingStrategy == TracksSamplingStrategy::SIMPLE_TRACK_SAMPLING)
            poly.points = arolib::geometry::sample_geometry( poly.points, res, 0.15*res);

        //reorder the track points so that the first point is the one closest to the initial point (1st track) or the track-end of the previous (outwards) track
        arolib::geometry::openPolygon(poly);
        size_t indFirstHLP = arolib::geometry::getPointIndInMinDist(poly.points, lastHeadlandTrackPoint);
        if(indFirstHLP != 0){
            std::vector<Point> pointsTmp;
            pointsTmp.insert( pointsTmp.end(), poly.points.begin(), poly.points.begin()+indFirstHLP );
            poly.points.erase( poly.points.begin(), poly.points.begin()+indFirstHLP );
            poly.points.insert( poly.points.end(), pointsTmp.begin(), pointsTmp.end() );
        }
        arolib::geometry::closePolygon(poly);
        lastHeadlandTrackPoint = poly.points.back();

        if(!machines.empty()){
            auto machine_ind = std::get<1>(track);
            routeAss.at(machine_ind).addTrack(poly.points,
                                              trackId,
                                              reverse,
                                              machines.at( machine_ind ),
                                              *edgeMassCalculator,
                                              *edgeSpeedCalculator);

        }

        //add track to the subfield
        subfield.headlands.complete.tracks.push_back(Track());
        subfield.headlands.complete.tracks.back().id = trackId;
        subfield.headlands.complete.tracks.back().type = Track::MAIN_HL;
        subfield.headlands.complete.tracks.back().points = poly.points;
    }

    for( size_t i = 0 ; i < routeAss.size() ; ++i){
        routes.push_back(routeAss.at(i).getRoute());
        routes.back().subfield_id = subfield.id;
    }

}

void HeadlandPlanner::addReferenceSamplesToTracks(std::vector<std::tuple<Polygon, size_t, int> > &tracks, double resolution)
{
    if(tracks.size() < 2)
        return;

    double multAngle = 1;
    if(!arolib::geometry::isPolygonClockwise(std::get<0>( tracks.back() )))
        multAngle = -1;

    for(int i = tracks.size()-2 ; i >= 0 ; --i){
        Polygon &poly = std::get<0>( tracks.at(i) );
        Polygon &polyRef = std::get<0>( tracks.at(i+1) );

        std::vector< std::pair<Point, Point> > perpLines;
        for(size_t ii = 0 ; ii+1 < polyRef.points.size() ; ++ii){
            auto& p0 = polyRef.points.at(ii);
            auto& p1 = polyRef.points.at(ii+1);
            Point pp = arolib::geometry::rotate(p0, p1, M_PI_2 * multAngle);
            perpLines.push_back( std::make_pair(p0, pp) );
        }

        int jRef = -1;
        size_t refIndex = 0;
        for(size_t j = 0 ; j+1 < poly.points.size() ; ++j){

            auto& p0 = poly.points.at(j);
            auto& p1 = poly.points.at(j+1);
            bool addPerpLine = j != jRef;
            if(refIndex < perpLines.size()){
                auto& p0_ref = perpLines.at(refIndex).first;
                auto& p1_ref = perpLines.at(refIndex).second;
                Point sample;

                if( arolib::geometry::get_intersection( p0_ref, p1_ref, p0, p1, sample, true )
                        && std::fabs( arolib::geometry::get_angle(p0_ref, p1_ref, p0_ref, sample) ) < M_PI_2 ){
                    if( arolib::geometry::calc_dist(sample, p0) > 0.1*resolution &&
                            arolib::geometry::calc_dist(sample, p1) > 0.1*resolution ){
                        poly.points.insert( poly.points.begin()+j+1, sample );
                        jRef = j+1;
                        ++refIndex;
                    }
                    else{
                        jRef = j--;
                        perpLines.erase( perpLines.begin()+refIndex );
                    }
                    addPerpLine = false;
                }
            }
        }
    }
}

void HeadlandPlanner::addReferenceSamplesToTracks_old(std::vector<std::tuple<Polygon, size_t, int> > &tracks, double resolution)
{
    if(tracks.size() < 2)
        return;

    Polygon &polyRef = std::get<0>( tracks.back() );
    double multAngle = 1;
    if(!arolib::geometry::isPolygonClockwise(polyRef))
        multAngle = -1;

    std::vector< std::vector< std::pair<Point, Point> > > perpLines(tracks.size());
    for(size_t i = 0 ; i+1 < polyRef.points.size() ; ++i){
        auto& p0 = polyRef.points.at(i);
        auto& p1 = polyRef.points.at(i+1);
        Point pp = arolib::geometry::rotate(p0, p1, M_PI_2 * multAngle);
        perpLines.back().push_back( std::make_pair(p0, pp) );
    }

    for(int i = tracks.size()-2 ; i >= 0 ; --i){
        auto& track = tracks.at(i);
        Polygon &poly = std::get<0>(track);

        for(int ii = tracks.size()-1 ; ii > i ; --ii){

            int jRef = -1;
            size_t refIndex = 0;
            for(size_t j = 0 ; j+1 < poly.points.size() ; ++j){

                auto& p0 = poly.points.at(j);
                auto& p1 = poly.points.at(j+1);
                bool addPerpLine = j != jRef;
                if(refIndex < perpLines.at(ii).size()){
                    auto& p0_ref = perpLines.at(ii).at(refIndex).first;
                    auto& p1_ref = perpLines.at(ii).at(refIndex).second;
                    Point sample;

                    if( arolib::geometry::get_intersection( p0_ref, p1_ref, p0, p1, sample, true )
                            && std::fabs( arolib::geometry::get_angle(p0_ref, p1_ref, p0_ref, sample) ) < M_PI_2 ){
                        if( arolib::geometry::calc_dist(sample, p0) > 0.1*resolution &&
                                arolib::geometry::calc_dist(sample, p1) > 0.1*resolution ){
                            poly.points.insert( poly.points.begin()+j+1, sample );
                            jRef = j+1;
                            ++refIndex;
                        }
                        else{
                            jRef = j--;
                            perpLines.at(ii).erase( perpLines.at(ii).begin()+refIndex );
                        }
                        addPerpLine = false;
                    }
                }
                if(addPerpLine && i != 0){
                    Point pp = arolib::geometry::rotate(p0, p1, M_PI_2 * multAngle);
                    perpLines.at(i).push_back( std::make_pair(p0, pp) );
                }
            }

        }

    }
}

void HeadlandPlanner::sampleTracksPerpendicularly(std::vector<std::tuple<Polygon, size_t, int> > &tracks, double resolution, double headlandWidth)
{
    if(tracks.empty())
        return;

    const double minDeltaSample = 0.15*resolution;
    const double minDeltaPerpSample = 0.1*resolution;

    if(tracks.size() == 1){//sample normally
        Polygon &poly = std::get<0>( tracks.front() );
        poly.points = arolib::geometry::sample_geometry(poly.points, resolution, minDeltaSample);
        return;
    }

    double multAngle = 1;//multiplier used to obtain the rotation angle (for the perpendicular vectors) depending on the polygon direction
    if(!arolib::geometry::isPolygonClockwise(std::get<0>( tracks.back() )))
        multAngle = -1;

    for(int i = tracks.size()-2 ; i >= 0 ; --i){
        Polygon &poly = std::get<0>( tracks.at(i) );
        Polygon &polyRef = std::get<0>( tracks.at(i+1) );

        polyRef.points = arolib::geometry::sample_geometry(polyRef.points, resolution, minDeltaSample);
        arolib::geometry::closePolygon(polyRef);
        std::vector< std::pair<Point, Point> > perpLines;
        std::vector< Point > refPoints;

        if(polyRef.points.size() > 3){//get the perpendicular points iif
            {
                auto& p0 = polyRef.points.front();
                auto& p1 = polyRef.points.at(1);
                auto& p2 = *( polyRef.points.end()-2 );

                //get the angle between the 2 (consecutive) segments to which p0 belongs (p0 being the connection between the 2 segments)
                double ang1 = ( multAngle>0 ?
                                    arolib::geometry::get_angle(p0, p1, p0, p2, true) :
                                    arolib::geometry::get_angle(p0, p2, p0, p1, true) );
                if( ang1 < 0 || ang1 >= 180){
                    Point pp = arolib::geometry::rotate(p0, p1, M_PI_2 * multAngle);
                    perpLines.push_back( std::make_pair(p0, pp) );
                }
                else if(ang1 > 175)
                    refPoints.emplace_back(p0);
            }
            for(size_t ii = 1 ; ii+1 < polyRef.points.size() ; ++ii){
                auto& p0 = polyRef.points.at(ii);
                auto& p1 = polyRef.points.at(ii+1);
                auto& p2 = polyRef.points.at(ii-1);

                //get the angle between the 2 (consecutive) segments to which p0 belongs (p0 being the connection between the 2 segments)
                double ang1 = ( multAngle>0 ?
                                    arolib::geometry::get_angle(p0, p1, p0, p2, true) :
                                    arolib::geometry::get_angle(p0, p2, p0, p1, true) );
                if( ang1 > 0 && ang1 < 180){
                    if(ang1 > 175)
                        refPoints.emplace_back(p0);
                    continue;
                }
                if( std::fabs( arolib::geometry::get_angle(p0, p1, p0, p2, true) ) < 170){
                    Point pp0 = arolib::geometry::rotate(p0, p2, M_PI_2 * -multAngle);
                    perpLines.push_back( std::make_pair(p0, pp0) );
                }
                Point pp = arolib::geometry::rotate(p0, p1, M_PI_2 * multAngle);
                perpLines.push_back( std::make_pair(p0, pp) );
            }
            {//add last ref perp point
                auto& p0 = polyRef.points.front();
                auto& p1 = polyRef.points.at(1);
                auto& p2 = *( polyRef.points.end()-2 );
                double ang1 = ( multAngle>0 ?
                                    arolib::geometry::get_angle(p0, p1, p0, p2, true) :
                                    arolib::geometry::get_angle(p0, p2, p0, p1, true) );
                if( ang1 < 0 || ang1 >= 180-1e-3){
                    if( std::fabs( arolib::geometry::get_angle(p0, p1, p0, p2, true) ) < 170){
                        Point pp0 = arolib::geometry::rotate(p0, p2, M_PI_2 * -multAngle);
                        perpLines.push_back( std::make_pair(p0, pp0) );
                    }
                }
            }
        }

        size_t refIndex = 0;
        for(int j = -1 ; j+1 < poly.points.size() ; ++j){

            Point p0;
            auto& p1 = poly.points.at(j+1);

            if(j < 0)
                p0 = *( poly.points.end()-2 );
            else
                p0 = poly.points.at(j);

            if(refIndex < perpLines.size()){
                auto& p0_ref = perpLines.at(refIndex).first;
                auto& p1_ref = perpLines.at(refIndex).second;
                Point sample;

                if( arolib::geometry::get_intersection( p0_ref, p1_ref, p0, p1, sample, true )
                        && std::fabs( arolib::geometry::get_angle(p0_ref, p1_ref, p0_ref, sample) ) < M_PI_2
                        && arolib::geometry::calc_dist(p0_ref, sample) < 0.5*headlandWidth ){
                    if( arolib::geometry::calc_dist(sample, p0) > minDeltaPerpSample &&
                            arolib::geometry::calc_dist(sample, p1) > minDeltaPerpSample ){

                        Point p3;
                        if(j+2 < poly.points.size() )
                            p3 = poly.points.at(j+2);
                        else
                            p3 = poly.points.at(1);


                        if( arolib::geometry::calc_dist(sample, p1) < resolution
                                && arolib::geometry::is_line(std::vector<Point>{sample, p1, p3}, 0.1) ){
                            if (j == -1){
                                poly.points.front() = sample;
                                poly.points.back() = sample;
                            }
                            else
                                poly.points.at(j+1) = sample;
                            --j;
                        }
                        else{
                            if(j < 0){
                                poly.points.insert( poly.points.end()-1, sample );
                                --j;
                            }
                            else
                                poly.points.insert( poly.points.begin()+j+1, sample );
                        }

                    }
                    else
                        --j;
                    ++refIndex;
                }
            }
        }

        for(auto& p :  refPoints)
            arolib::geometry::addSampleToGeometryClosestToPoint(poly.points, p, 1, minDeltaPerpSample);
    }


    Polygon &poly0 = std::get<0>( tracks.front() );
    poly0.points = arolib::geometry::sample_geometry(poly0.points, resolution, minDeltaSample);
}

void HeadlandPlanner::sampleTracksWithClosestPoints(std::vector<std::tuple<Polygon, size_t, int> > &tracks, double resolution)
{
    if(tracks.empty())
        return;

    const double minDeltaSample = 0.15*resolution;
    const double minDeltaPerpSample = 0.1*resolution;

    if(tracks.size() == 1){//sample normally
        Polygon &poly = std::get<0>( tracks.front() );
        poly.points = arolib::geometry::sample_geometry(poly.points, resolution, minDeltaSample);
        return;
    }

    double multAngle = 1;
    if(!arolib::geometry::isPolygonClockwise(std::get<0>( tracks.back() )))
        multAngle = -1;

    for(int i = tracks.size()-2 ; i >= 0 ; --i){
        Polygon &poly = std::get<0>( tracks.at(i) );
        Polygon &polyRef = std::get<0>( tracks.at(i+1) );

        polyRef.points = arolib::geometry::sample_geometry(polyRef.points, resolution, minDeltaSample);
        arolib::geometry::closePolygon(polyRef);

        for(size_t i = 0 ; i+1 < polyRef.points.size() ; ++i)
            arolib::geometry::addSampleToGeometryClosestToPoint(poly.points, polyRef.points.at(i), 0, minDeltaPerpSample);
    }


    Polygon &poly0 = std::get<0>( tracks.front() );
    poly0.points = arolib::geometry::sample_geometry(poly0.points, resolution, minDeltaSample);

}

bool HeadlandPlanner::replanningNeeded(std::vector<HeadlandRoute> &routes,
                                       const std::vector<Machine> &workingGroup,
                                       const ArolibGrid_t &remainingAreaMap,
                                       const PlannerParameters &settings,
                                       Point &newInitRefPoint)
{
    if (!remainingAreaMap.isAllocated() || routes.empty())
        return false;

    //check if the headland is completelly harvested
    bool somethingToHarvest = false;
    for(auto &route : routes){
        for(size_t i = 0 ; i+1 < route.route_points.size() ; ++i){
            auto &rp0 = route.route_points.at(i);
            auto &rp1 = route.route_points.at(i+1);

            double workingWidth = 0;
            for(auto &m : workingGroup){
                if(m.id == route.machine_id){
                    workingWidth = m.working_width;
                    break;
                }
            }

            if( !isWorked(rp0, rp1, workingWidth, remainingAreaMap, settings.bePreciseWithRemainingAreaMap) ){
                somethingToHarvest = true;
                break;
            }
        }
        if(somethingToHarvest)
            break;
    }
    if(!somethingToHarvest){
        routes.clear();
        return false;
    }


    HeadlandRoute &harvRoute = routes.front();//workaround: check only the first route
    if(harvRoute.route_points.size() < 2 )
        return false;


    double workingWidth = 0;
    for(auto &m : workingGroup){
        if(m.id == harvRoute.machine_id){
            workingWidth = m.working_width;
            break;
        }
    }

    int indNotHarvToHarv = -1;
    int indHarvToNotHarv = -1;
    bool somethingHarvested = false;
    bool harv_0_1;
    bool harv_1_2;

    std::vector<RoutePoint> &routePoints = harvRoute.route_points;
    for(size_t i = 0 ; i+2 < routePoints.size() ; ++i){
        auto &rp0 = routePoints.at(i);
        auto &rp1 = routePoints.at(i+1);
        auto &rp2 = routePoints.at(i+2);

        if(i == 0)
            harv_0_1 = isWorked(rp0, rp1, workingWidth, remainingAreaMap);
        else
            harv_0_1 = harv_1_2;
        harv_1_2 = isWorked(rp1, rp2, workingWidth, remainingAreaMap);

        somethingHarvested |= (harv_0_1 || harv_1_2);

        if(indNotHarvToHarv < 0 && !harv_0_1 && harv_1_2)
            indNotHarvToHarv = i+1;
        if(indHarvToNotHarv < 0 && harv_0_1 && !harv_1_2)
            indHarvToNotHarv = i+1;
    }

    if(!somethingHarvested)
        return false;
    if(indHarvToNotHarv < 0){//all harvested
        newInitRefPoint = routePoints.back().point();
        return true;//this route is all harvested, but some of the other ones not
    }

    if(indNotHarvToHarv < 0)
        newInitRefPoint = routePoints.front().point();
//    else if(indNotHarvToHarv+1 < routePoints.size()){//should we really use indNotHarvToHarv+1 (the next point)?
//        if( routePoints.at(indNotHarvToHarv).type == RoutePoint::TRACK_END ){//the desired point is actually the one after the previous track_start (next point is in the next track)
//            for(int i = indNotHarvToHarv-1 ; i >= 0 ; --i){
//                if( routePoints.at(i).type == RoutePoint::TRACK_START )
//                    indNotHarvToHarv = i;
//            }

//        }
//        newInitRefPoint = routePoints.at(indNotHarvToHarv+1).point;
//    }
    else if(indNotHarvToHarv < routePoints.size())
        newInitRefPoint = routePoints.at(indNotHarvToHarv).point();
    else
        newInitRefPoint = routePoints.back().point();

    return true;
}

bool HeadlandPlanner::isWorked(const Point &p0,
                               const Point &p1,
                               double width,
                               const ArolibGrid_t &remainingAreaMap,
                               bool bePrecise){
    if(!remainingAreaMap.isAllocated())
        return false;

    bePrecise &= width > 1e-5;

    bool errorTmp;
    double value;

    if(bePrecise){

        if(m_calcGridValueOption == CALC_PW){
            const auto &map = getMaps(*m_planningWorkspace).at( PlanningWorkspace::GridType::REMAINING_AREA );
            PlanningWorkspace::Edge edge( p0,
                                          p1,
                                          width);

            auto it_edge = getEdgesInfield(*m_planningWorkspace).find(edge);
            auto &ep = getEdgesInfield(*m_planningWorkspace)[edge];

            if(it_edge == getEdgesInfield(*m_planningWorkspace).end() || !ep.gridCellsInfoIsSet.at(PlanningWorkspace::GridType::REMAINING_AREA))
                ep.updateGridCells( PlanningWorkspace::GridType::REMAINING_AREA, map, edge, true );

            value = map.getCellsComputedValue( ep.gridCellsInfo.at(PlanningWorkspace::GridType::REMAINING_AREA),
                                               ArolibGrid_t::AVERAGE_TOTAL,
                                               arolib::geometry::calc_dist(edge.p0, edge.p1) * edge.width,
                                               false,
                                               &errorTmp );
        }
        else
            value = remainingAreaMap.getLineComputedValue(p0,
                                                           p1,
                                                           width,
                                                           true,
                                                           ArolibGrid_t::AVERAGE_TOTAL,
                                                           &errorTmp);
        return (errorTmp || value < m_thresholdIsWorked);
    }

    arolib::Point p0_1;
    p0_1.x = 0.5*( p0.x + p1.x );
    p0_1.y = 0.5*( p0.y + p1.y );

    value = remainingAreaMap.getValue(p0_1, &errorTmp);
    return (!errorTmp && value < m_thresholdIsWorked);

}

}
