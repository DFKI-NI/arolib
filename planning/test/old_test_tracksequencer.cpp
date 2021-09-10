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
 
#include "machine.hpp"
#include "io_txt.hpp"
#include "io_xml.hpp"
#include "field.hpp"
#include "field_geometry_processing.hpp"
#include "tracksequencer.hpp"

int main() {
  std::vector<arolib::Machine> machines;
  arolib::readMachineXML("../arolib/test/machines.xml", machines);
  arolib::Field f;
  std::string field_directory = "../arolib/test/testfields/field_30-02"; //change directory for testfield here!!!!
  arolib::readFieldTXT((field_directory + "/field_out_gps").c_str(), f);
  arolib::readReferenceLineTXT((field_directory + "/reference_Line_multiPoints.txt").c_str(), f.subfields[0]);

  arolib::create_innerBoundary(f.subfields[0].boundary_outer, f.subfields[0].boundary_inner, 26.5, 200.0);
  arolib::Polygon help_headland;
  arolib::create_innerBoundary(f.subfields[0].boundary_outer, help_headland, 10.0, 200.0);
  f.subfields.at(0).planed_headland.points = arolib::geometry::sample_geometry(help_headland.points, 10.0);
  f.subfields.at(0).tracks = arolib::shiftReferenceLine(f.subfields[0], 10.0);

  arolib::TrackSequencer track_sequencer;
  track_sequencer.setTracksPerMachine(4);
  track_sequencer.setField(f.subfields.at(0));
  std::cout << f.subfields.at(0).tracks.size() << std::endl;

  track_sequencer.setStrategy(arolib::TrackSequencer::INNER_TO_OUTER);
  //track_sequencer.setStrategy(arolib::TrackSequencer::MEANDER);

  for (int i = 0; i < machines.size(); ++i) {
      std::cout << "Machine " << i << ": ID = " << machines.at(i).id << std::endl;
      track_sequencer.addMachine(machines.at(i));
  }
  track_sequencer.computeSequence();
  std::vector<int> sequence0 = track_sequencer.getSequence(machines.at(0).id);
  std::vector<int> sequence1 = track_sequencer.getSequence(machines.at(1).id);

  std::cout << "Track 0:" << std::endl;
  for (size_t i=0; i < sequence0.size(); i++)
    std::cout << sequence0.at(i) << " ";
  std::cout << std::endl;
  std::cout << "Track 1:" << std::endl;
  for (size_t i=0; i < sequence1.size(); i++)
    std::cout << sequence1.at(i) << " ";
  std::cout << std::endl;
}
