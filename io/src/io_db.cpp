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
 
#include <io_db.hpp>

#include <sstream>
#include <iterator>
#include <stdlib.h>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

/// mysql connection
#define SERVER "localhost"
#define USER "root"
#define PASSWORD ""
#define DATABASE "machines_db"


namespace arolib{

std::vector<Machine> getDataDB(MYSQL *connector, boost::property_tree::ptree pt){
    using boost::property_tree::ptree;

    std::vector<Machine> machine_setup;
    BOOST_FOREACH( ptree::value_type &u, pt){
        if(u.first == "machine" || u.first == "implement"){
            Machine m;
            /// get name of the machine of implement
            std::string name = u.second.get<std::string>("name");
            /// pointer for return values
            MYSQL_RES *result;
            /// variable for rows
            MYSQL_ROW row;
            /// variable for field
            MYSQL_FIELD *field;
            /// send SQL query to database
            std::string query = "SELECT Machines.*, ProducerMachines.name AS producer_name,"
                    "MachineType.name AS machine_type_name, MachineType.type AS machine_type, Kinematics.kinematicsType,"
                    "Kinematics.maxSpeed, Kinematics.minSpeed, Kinematics.maxCurvature, Kinematics.maxCurvatureChange,"
                    "Kinematics.wheelBase, Kinematics.frontAxleLength, Kinematics.rareAxleLength FROM Machines"
                    " INNER JOIN ProducerMachines ON (ProducerMachines.id = Machines.producerID)"
                    " INNER JOIN Kinematics ON (Kinematics.id = Machines.kinematicsID)"
                    " INNER JOIN MachineType ON (MachineType.id = Machines.machineTypeID) WHERE Machines.name = '" + name + "'";
            mysql_query(connector, query.c_str());

            result = mysql_store_result(connector);

            /// get fields number
            unsigned int num_fields = mysql_num_fields(result);
            char *field_name[num_fields];
            row = mysql_fetch_row(result);
            for(unsigned int i = 0; (field = mysql_fetch_field(result)); i++){
                field_name[i] = field->name;
                unsigned long *lengths;
                lengths = mysql_fetch_lengths(result);

                if(strcmp("id", field_name[i]) == 0){
                    m.id = atoi(row[i]);
                }
                else if(strcmp("producer_name", field_name[i]) == 0){
                    m.producer_name = row[i];
                }
                else if(strcmp("name", field_name[i]) == 0){
                    m.machine_name = row[i];
                }
                else if(strcmp("machine_type_name", field_name[i]) == 0){
                    m.machine_type_name = row[i];
                }
                else if(strcmp("machine_type", field_name[i]) == 0){
                    m.machine_type = row[i];
                }
                else if(strcmp("width", field_name[i]) == 0){
                    if(lengths[i] == 0){
                        m.width = std::numeric_limits<double>::quiet_NaN();
                    }
                    else{
                        m.width = atof(row[i]);
                    }
                }
                else if(strcmp("length", field_name[i]) == 0){
                    if(lengths[i] == 0){
                        m.length = std::numeric_limits<double>::quiet_NaN();
                    }
                    else{
                        m.length = atof(row[i]);
                    }
                }
                else if(strcmp("height", field_name[i]) == 0){
                    if(lengths[i] == 0){
                        m.height = std::numeric_limits<double>::quiet_NaN();
                    }
                    else{
                        m.height = atof(row[i]);
                    }
                }
                else if(strcmp("weight", field_name[i]) == 0){
                    if(lengths[i] == 0){
                        m.weight = std::numeric_limits<double>::quiet_NaN();
                    }
                    else{
                        m.weight = atof(row[i]);
                    }
                }
                else if(strcmp("bunkerVolume", field_name[i]) == 0){
                    if(lengths[i] == 0){
                        m.bunker_volume = std::numeric_limits<double>::quiet_NaN();
                    }
                    else{
                        m.bunker_volume = atof(row[i]);
                    }
                }
                else if(strcmp("footprint", field_name[i]) == 0){
                    if(!lengths[i] == 0){
                        std::string fpp = row[i];
                        std::istringstream iss(fpp);
                        std::vector<std::string> tokens;
                        copy(std::istream_iterator<std::string>(iss),
                                 std::istream_iterator<std::string>(),
                                 std::back_inserter<std::vector<std::string> >(tokens));
                        Polygon poly;
                        for(int i = 0; i < tokens.size()-1; i+=2){
                                Point p(atof(tokens[i].c_str()), atof(tokens[i+1].c_str()));
                                poly.points.push_back(p);
                        }
                        m.footprint.push_back(poly);
                    }
                }
                else if(strcmp("kinematicsType", field_name[i]) == 0){
                    m.kinematic_type = row[i];
                }
                else if(strcmp("maxSpeed", field_name[i]) == 0){
                    if(lengths[i] == 0){
                        m.max_speed = std::numeric_limits<double>::quiet_NaN();
                    }
                    else{
                        m.max_speed = atof(row[i]);
                    }
                }
                else if(strcmp("minSpeed", field_name[i]) == 0){
                    if(lengths[i] == 0){
                        m.min_speed = std::numeric_limits<double>::quiet_NaN();
                    }
                    else{
                        m.min_speed = atof(row[i]);
                    }
                }
                else if(strcmp("maxCurvature", field_name[i]) == 0){
                    if(lengths[i] == 0){
                        m.max_curvature = std::numeric_limits<double>::quiet_NaN();
                    }
                    else{
                        m.max_curvature = atof(row[i]);
                    }
                }
                else if(strcmp("maxCurvatureChange", field_name[i]) == 0){
                    if(lengths[i] == 0){
                        m.max_curvature_change = std::numeric_limits<double>::quiet_NaN();
                    }
                    else{
                        m.max_curvature_change = atof(row[i]);
                    }
                }
                //else if(strcmp("wheelBase", field_name[i]) == 0){
                //    if(lengths[i] == 0){
                //        m.wheel_base = std::numeric_limits<double>::quiet_NaN();
                //    }
                //    else{
                //        m.wheel_base = atof(row[i]);
                //    }
                //}
                //else if(strcmp("frontAxleLength", field_name[i]) == 0){
                //    if(lengths[i] == 0){
                //        m.front_axle_length = std::numeric_limits<double>::quiet_NaN();
                //    }
                //    else{
                //        m.front_axle_length = atof(row[i]);
                //    }
                //}
                //else if(strcmp("rareAxleLength", field_name[i]) == 0){
                //    if(lengths[i] == 0){
                //        m.rare_axle_length = std::numeric_limits<double>::quiet_NaN();
                //    }
                //    else{
                //        m.rare_axle_length = atof(row[i]);
                //    }
                //}
                else if(strcmp("mountingPositions", field_name[i]) == 0){
                    if(!lengths[i] == 0){
                        std::string mp = row[i];
                        std::istringstream iss(mp);
                        std::vector<std::string> tokens;
                        copy(std::istream_iterator<std::string>(iss),
                                 std::istream_iterator<std::string>(),
                                 std::back_inserter<std::vector<std::string> >(tokens));
                        for(int i = 0; i < tokens.size()-1; i+=5){
                            arolib::Machine::mounting_position mounting_p;
                            mounting_p.x = atof(tokens[i].c_str());
                            mounting_p.y = atof(tokens[i+1].c_str());
                            mounting_p.z = atof(tokens[i+2].c_str());
                            mounting_p.theta = atof(tokens[i+3].c_str());
                            mounting_p.phi = atof(tokens[i+4].c_str());
                            m.mounting_positions.push_back(mounting_p);
                        }
                    }
                }
                else if(strcmp("referencePointX", field_name[i]) == 0){
                    if(lengths[i] == 0){
                        m.reference_point_x = std::numeric_limits<double>::quiet_NaN();
                    }
                    else{
                        m.reference_point_x = atof(row[i]);
                    }
                }
                else if(strcmp("referencePointY", field_name[i]) == 0){
                    if(lengths[i] == 0){
                        m.reference_point_y = std::numeric_limits<double>::quiet_NaN();
                    }
                    else{
                        m.reference_point_y = atof(row[i]);
                    }
                }
                else if(strcmp("referencePointZ", field_name[i]) == 0){
                    if(lengths[i] == 0){
                        m.reference_point_z = std::numeric_limits<double>::quiet_NaN();
                    }
                    else{
                        m.reference_point_z = atof(row[i]);
                    }
                }
                else if(strcmp("referencePointTheta", field_name[i]) == 0){
                    if(lengths[i] == 0){
                        m.reference_point_theta = std::numeric_limits<double>::quiet_NaN();
                    }
                    else{
                        m.reference_point_theta = atof(row[i]);
                    }
                }
                else if(strcmp("referencePointPhi", field_name[i]) == 0){
                    if(lengths[i] == 0){
                        m.reference_point_phi = std::numeric_limits<double>::quiet_NaN();
                    }
                    else{
                        m.reference_point_phi = atof(row[i]);
                    }
                }
            }

            /// get wheels and sensors
            ptree w = u.second;
            BOOST_FOREACH( ptree::value_type &v, w){
                if(v.first == "wheel"){
                    Machine::wheel tire;
                    /// get name of the wheel
                    std::string name = v.second.get<std::string>("name");
                    /// send SQL query to database
                    query = "SELECT Wheels.*, ProducerWheels.name AS wheel_producer_name"
                            " FROM Wheels INNER JOIN ProducerWheels ON (ProducerWheels.id = Wheels.producerID)"
                            " WHERE Wheels.name = '" + name + "'";
                    mysql_query(connector, query.c_str());

                    result = mysql_store_result(connector);
                    /// get fields number
                    num_fields = mysql_num_fields(result);
                    field_name[num_fields];
                    row = mysql_fetch_row(result);
                    for(unsigned int i = 0; (field = mysql_fetch_field(result)); i++){
                        field_name[i] = field->name;
                        unsigned long *lengths;
                        lengths = mysql_fetch_lengths(result);

                        if(strcmp("id", field_name[i]) == 0){
                            tire.id = atoi(row[i]);
                        }
                        else if(strcmp("name", field_name[i]) == 0){
                            tire.name = row[i];
                        }
                        else if(strcmp("wheel_producer_name", field_name[i]) == 0){
                            tire.producer_name = row[i];
                        }
                        else if(strcmp("type", field_name[i]) == 0){
                            tire.wheel_type = row[i];
                        }
                        else if(strcmp("radius", field_name[i]) == 0){
                            if(lengths[i] == 0){
                                tire.radius = std::numeric_limits<double>::quiet_NaN();
                            }
                            else{
                                tire.radius = atof(row[i]);
                            }
                        }
                        else if(strcmp("width", field_name[i]) == 0){
                            if(lengths[i] == 0){
                                tire.width = std::numeric_limits<double>::quiet_NaN();
                            }
                            else{
                                tire.width = atof(row[i]);
                            }
                        }
                    }

                    std::string position = v.second.get<std::string>("position");
                    std::istringstream iss(position);
                    std::vector<std::string> tokens;
                    copy(std::istream_iterator<std::string>(iss),
                         std::istream_iterator<std::string>(),
                         std::back_inserter<std::vector<std::string> >(tokens));
                    for(int i = 0; i < tokens.size()-1; i+=5){
                        tire.mp.x = atof(tokens[i].c_str());
                        tire.mp.y = atof(tokens[i+1].c_str());
                        tire.mp.z = atof(tokens[i+2].c_str());
                        tire.mp.theta = atof(tokens[i+3].c_str());
                        tire.mp.phi = atof(tokens[i+4].c_str());
                    }
                    m.wheels.push_back(tire);
                }
                /// sensors
                else if(v.first == "sensor"){
                    Machine::sensor s;
                    /// get name of the sensor
                    std::string name = v.second.get<std::string>("name");
                    /// send SQL query to database
                    query = "SELECT Sensors.*, ProducerSensors.name AS sensor_producer_name"
                            " FROM Sensors INNER JOIN ProducerSensors ON (ProducerSensors.id = Sensors.producerID)"
                            " WHERE Sensors.name = '" + name + "'";
                    mysql_query(connector, query.c_str());

                    result = mysql_store_result(connector);
                    /// get fields number
                    num_fields = mysql_num_fields(result);
                    field_name[num_fields];
                    row = mysql_fetch_row(result);
                    for(unsigned int i = 0; (field = mysql_fetch_field(result)); i++){
                        field_name[i] = field->name;
                        unsigned long *lengths;
                        lengths = mysql_fetch_lengths(result);

                        if(strcmp("id", field_name[i]) == 0){
                            s.id = atoi(row[i]);
                        }
                        else if(strcmp("sensor_producer_name", field_name[i]) == 0){
                            s.producer_name = row[i];
                        }
                        else if(strcmp("name", field_name[i]) == 0){
                            s.name = row[i];
                        }
                        else if(strcmp("type", field_name[i]) == 0){
                            s.sensor_type = row[i];
                        }
                    }

                    std::string position = v.second.get<std::string>("position");
                    std::istringstream iss(position);
                    std::vector<std::string> tokens;
                    copy(std::istream_iterator<std::string>(iss),
                         std::istream_iterator<std::string>(),
                            std::back_inserter<std::vector<std::string> > (tokens));
                    for(int i = 0; i < tokens.size()-1; i+=5){
                        s.mp.x = atof(tokens[i].c_str());
                        s.mp.y = atof(tokens[i+1].c_str());
                        s.mp.z = atof(tokens[i+2].c_str());
                        s.mp.theta = atof(tokens[i+3].c_str());
                        s.mp.phi = atof(tokens[i+4].c_str());
                    }
                    m.sensors.push_back(s);
                }
            }

            machine_setup.push_back(m);
        }

    }
    return machine_setup;
}


bool readMachineSetups(const std::string &filename, std::vector<std::vector<Machine> > &machine_setups){

    /// connect to database
    /// pointer to mysql instance
     MYSQL *connect;
     /// init instance
     connect = mysql_init(NULL);
     /// init ok?
     if(!connect){
         std::cerr << "mysql initialization failed" << std::endl;
         return false;
     }
     /// connect to database
     connect = mysql_real_connect(connect, SERVER, USER, PASSWORD, DATABASE, 0, NULL, 0);
     if(!connect){
         std::cerr << "mysql connection failed" << std::endl;
         return false;
     }

    std::ifstream in(filename.c_str());

    /// check if file open is ok
    if(!in.is_open()){
        std::cerr << "ERROR: Cannot open file " << filename << std::endl;
        return false;
    }

    using boost::property_tree::ptree;
    ptree pt;
    read_xml(in, pt);


    /// traverse ptree
    BOOST_FOREACH( ptree::value_type &v, pt.get_child("setup")){
        if(v.first == "machine_setup"){
            ptree ms = v.second;
            std::vector<Machine> machine_setup = getDataDB(connect, ms);
            machine_setups.push_back(machine_setup);
        }
    }
    std::cout << "machine_setups: " << machine_setups.size() << std::endl;
    return true;
}





}
