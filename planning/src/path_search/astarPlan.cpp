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
 
#include "arolib/planning/path_search/astarPlan.hpp"


namespace arolib{

void AstarPlan::adjustAccessPoints(bool lastAccessIsExit)
{
    for(auto it_rp = route_points_.rbegin() ; it_rp != route_points_.rend() ; it_rp++){
        if(it_rp->isFieldAccess()){
            if(lastAccessIsExit)
                it_rp->type = RoutePoint::FIELD_EXIT;
            else
                it_rp->type = RoutePoint::FIELD_ENTRY;
            lastAccessIsExit = !lastAccessIsExit;
        }
    }
}

void AstarPlan::insertIntoGraph(DirectedGraph::Graph &graph) const {
    for (int i = 0; i < edge_overruns_.size(); ++i){
        const DirectedGraph::EdgeOverrun& eo = edge_overruns_.at(i);
        graph.addOverrun(source(eo.first,graph), target(eo.first,graph), eo.second);
    }

    for (auto vp : visit_periods)
        graph[vp.first].visitPeriods.insert( std::make_pair(vp.second.time_in, vp.second) );
}

AstarPlan &AstarPlan::add(const AstarPlan &other, bool removeFirstRoutePoint)
{
    isOK &= other.isOK;
    plan_cost_ += other.plan_cost_;
    plan_cost_total += other.plan_cost_total;
    waitTime += other.waitTime;
    numCrossings += other.numCrossings;
    numCrossings_HL += other.numCrossings_HL;
    if(!other.route_points_.empty())
        route_points_.insert(route_points_.end(), other.route_points_.begin()+removeFirstRoutePoint, other.route_points_.end());
    edge_overruns_.insert(edge_overruns_.end(), other.edge_overruns_.begin(), other.edge_overruns_.end());
    visit_periods.insert(visit_periods.end(), other.visit_periods.begin(), other.visit_periods.end());
    visited_vertices.insert(visited_vertices.end(), other.visited_vertices.begin(), other.visited_vertices.end());
    visited_edges.insert(visited_edges.end(), other.visited_edges.begin(), other.visited_edges.end());

    return *this;
}

}
