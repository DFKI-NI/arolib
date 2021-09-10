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
 
#include "arolib/io/arotagsmanager.hpp"

namespace arolib {
namespace io {

const std::string AroTagsManager::UseDefaultTag = "<>";
const std::map<std::type_index, std::string> AroTagsManager::m_tags = { { std::type_index(typeid(Field)),"field" },
                                                                        { std::type_index(typeid(Subfield)), "subfield" },
                                                                        { std::type_index(typeid(Track)), "track" },
                                                                        { std::type_index(typeid(Track::TrackType)), "type" },
                                                                        { std::type_index(typeid(Headlands)), "headlands" },
                                                                        { std::type_index(typeid(CompleteHeadland)), "complete_headland" },
                                                                        { std::type_index(typeid(Obstacle)), "obstacle" },
                                                                        { std::type_index(typeid(Obstacle::ObstacleType)), "type" },
                                                                        { std::type_index(typeid(Point)), "coordinates" },
                                                                        { std::type_index(typeid(RoutePoint)), "route_point" },
                                                                        { std::type_index(typeid(RoutePoint::RoutePointType)), "type" },
                                                                        { std::type_index(typeid(HeadlandPoint)), "headland_point" },
                                                                        { std::type_index(typeid(ResourcePoint)), "resource_point" },
                                                                        { std::type_index(typeid(FieldAccessPoint)), "access_point" },
                                                                        { std::type_index(typeid(FieldAccessPoint::AccessPointType)), "accessType" },
                                                                        { std::type_index(typeid(Machine)), "machine" },
                                                                        { std::type_index(typeid(Machine::MachineType)), "machinetype" },
                                                                        { std::type_index(typeid(Machine::MachineAssignment)), "machineassignment" },
                                                                        { std::type_index(typeid(Route)), "route" },
                                                                        { std::type_index(typeid(HeadlandRoute)), "route" },
                                                                        { std::type_index(typeid(Linestring)), "linestring" },
                                                                        { std::type_index(typeid(Polygon)), "polygon" },
                                                                        { std::type_index(typeid(OutFieldInfo)), "outFieldInfo" },
                                                                        { std::type_index(typeid(OutFieldInfo::MapArrivalCosts_t)), "mapArrivalCosts" },
                                                                        { std::type_index(typeid(OutFieldInfo::MapAccessPoint2ResourcePoint_t)), "mapAccessPoint2ResourcePoint" },
                                                                        { std::type_index(typeid(OutFieldInfo::MapResourcePoint2AccessPoint_t)), "mapResourcePoint2AccessPoint" },
                                                                        { std::type_index(typeid(OutFieldInfo::MapUnloadingCosts_t)), "mapUnloadingCosts" },
                                                                        { std::type_index(typeid(OutFieldInfo::UnloadingCosts)), "unloading_costs" },
                                                                        { std::type_index(typeid(OutFieldInfo::ArrivalData)), "arrival_data" },
                                                                        { std::type_index(typeid(OutFieldInfo::TravelCosts)), "travel_costs" },
                                                                        { std::type_index(typeid(OutFieldInfo::MachineBunkerState)), "bunker_state" },
                                                                        { std::type_index(typeid(std::map<MachineId_t, MachineDynamicInfo>)), "machinesDynamicInfo" },
                                                                        { std::type_index(typeid(MachineDynamicInfo)), "dynamic_info" },
                                                                        { std::type_index(typeid(ArolibGrid_t)), "grid" },
                                                                        { std::type_index(typeid(DirectedGraph::Graph)), "graph" },
                                                                        { std::type_index(typeid(DirectedGraph::vertex_pair)), "vertex" },
                                                                        { std::type_index(typeid(DirectedGraph::edge_pair)), "edge" },
                                                                        { std::type_index(typeid(DirectedGraph::vertex_property)), "prop" },
                                                                        { std::type_index(typeid(DirectedGraph::edge_t)), "edge_id" },
                                                                        { std::type_index(typeid(DirectedGraph::edge_property)), "prop" },
                                                                        { std::type_index(typeid(DirectedGraph::EdgeType)), "type" },
                                                                        { std::type_index(typeid(DirectedGraph::VisitPeriod)), "visit_period" },
                                                                        { std::type_index(typeid(DirectedGraph::overroll_property)), "overrun" },
                                                                        { std::type_index(typeid(std::vector<Field>)),"fields" },
                                                                        { std::type_index(typeid(std::vector<Subfield>)),"subfields" } ,
                                                                        { std::type_index(typeid(std::vector<Obstacle>)),"obstacles" } ,
                                                                        { std::type_index(typeid(std::vector<Track>)),"tracks" } ,
                                                                        { std::type_index(typeid(std::vector<Point>)),"coordinates" },
                                                                        { std::type_index(typeid(std::vector<RoutePoint>)),"route_points" },
                                                                        { std::type_index(typeid(std::vector<HeadlandPoint>)),"headland_points" },
                                                                        { std::type_index(typeid(std::vector<ResourcePoint>)),"resource_points" },
                                                                        { std::type_index(typeid(std::vector<FieldAccessPoint>)),"access_points" },
                                                                        { std::type_index(typeid(std::vector<Machine>)),"machines" },
                                                                        { std::type_index(typeid(std::vector<Route>)),"routes" },
                                                                        { std::type_index(typeid(std::vector<HeadlandRoute>)),"headland_routes" } ,
                                                                        { std::type_index(typeid(std::set<ResourcePoint::ResourceType>)),"resource_types" },
                                                                        { std::type_index(typeid(std::vector<DirectedGraph::overroll_property>)),"overruns" }};






}
}//end namespace arolib

