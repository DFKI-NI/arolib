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
 
#include "arolib/io/io_hdf5.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/io/io_common.hpp"
#include <highfive/H5Easy.hpp>

#include <iostream>
#include <functional>

namespace arolib {
namespace io {

// Helper functions (only needed here)
namespace
{

    template<typename ... Args>
    std::string string_format( const std::string& format, Args ... args );


    void toLineString(boost::multi_array<double,2>& array_in, size_t& id, arolib::Linestring& linestring_out);
    void toPointVec(boost::multi_array<double,2>& array_in, std::vector<arolib::Point>& points_out);
    void toPointVec(boost::multi_array<double,2>& array_in, Polygon &poly_out);
    boost::multi_array<double,2> readMatrix(HighFive::File& file, std::string path);
    arolib::Linestring toLineStringNew(boost::multi_array<double,2>& array_in, size_t& id);

    arolib::Subfield readSubField(HighFive::File &file, HighFive::Group& group);
    std::vector<arolib::Track> readTracks(HighFive::File &file, HighFive::Group& group);

    void writePointsAsMatrix(const std::vector<arolib::Point>& points, std::string path, HighFive::File &file, H5Easy::DumpOptions &opt);
    void writeTracks(const std::vector<arolib::Track>& tracks, std::string path, HighFive::File &file, H5Easy::DumpOptions &opt);

    std::vector<std::string> listNodes(std::string file_path, std::string node_path);

    arolib::Subfield readSubField(HighFive::File &file, HighFive::Group& subfield_group)
    {
        Subfield subfield;
        // -----------------------------
        // Access Points
        // -----------------------------
        size_t found = subfield_group.getPath().find_last_of("/\\");
        std::string group_path = subfield_group.getPath().substr(0,found);
        std::string group_name = subfield_group.getPath().substr(found+1);
        subfield.id = std::stoi(group_name);
        std::string current_node_str = "access_points";
        if(subfield_group.exist(current_node_str))
        {
            auto group = subfield_group.getGroup(current_node_str);
            std::vector<std::string> child_list = group.listObjectNames();
            for(auto&& child : child_list)
            {
                auto point = H5Easy::load<boost::multi_array<double, 2>>(file,group.getPath() + "/" + child  + "/position");
                int type = H5Easy::load<int>(file,group.getPath() + "/" + child  + "/type");
                // convert to arolib objects
                size_t child_id = std::stoi(child);
                subfield.access_points.emplace_back(arolib::Point(point[0][0],point[0][1],point[0][2]), child_id,
                                                    FieldAccessPoint::intToAccessPointType(type));
            }
        }
        // -----------------------------
        // Boundary Inner
        // -----------------------------
        current_node_str = "boundary_inner";
        if(subfield_group.exist(current_node_str))
        {
            auto points = readMatrix(file,subfield_group.getPath() + "/" + current_node_str);
            toPointVec(points,subfield.boundary_inner);
        }
        // -----------------------------
        // Boundary Outer
        // -----------------------------
        current_node_str = "boundary_outer";
        if(subfield_group.exist(current_node_str))
        {
            auto points = readMatrix(file,subfield_group.getPath() + "/" + current_node_str);
            toPointVec(points,subfield.boundary_outer);
        }

        // -----------------------------
        // Headlands
        // -----------------------------
        current_node_str = "headlands";
        if(subfield_group.exist(current_node_str))
        {
            auto group = subfield_group.getGroup(current_node_str);
            // -----------------------------
            // Headlands/Complete
            // -----------------------------
            std::string current_node_str2 = "complete";
            if(group.exist(current_node_str2))
            {
                auto sub_group = group.getGroup(current_node_str2);
                // -----------------------------
                // Headlands/Complete
                // -----------------------------
                subfield.headlands.complete.headlandWidth = H5Easy::load<double>(file,sub_group.getPath() + "/width");
                auto middle_track_points = readMatrix(file,sub_group.getPath() + "/middle_track");
                toPointVec(middle_track_points,subfield.headlands.complete.middle_track.points);
                // -----------------------------
                // Headlands/Complete/Tracks
                // -----------------------------
                subfield.headlands.complete.tracks = readTracks(file,sub_group);


                auto boundary_points = readMatrix(file,sub_group.getPath() + "/boundary_out");
                toPointVec(boundary_points,subfield.headlands.complete.boundaries.first);
                boundary_points = readMatrix(file,sub_group.getPath() + "/boundary_in");
                toPointVec(boundary_points,subfield.headlands.complete.boundaries.second);
            }

        }
        // -----------------------------
        // Obstacles
        // -----------------------------
        current_node_str = "obstacles";
        if(subfield_group.exist(current_node_str))
        {
            auto group = subfield_group.getGroup(current_node_str);
            std::vector<std::string> child_list = group.listObjectNames();
            for(auto&& child : child_list)
            {
                size_t child_id = std::stoi(child);
                Obstacle obs;
                obs.type = Obstacle::intToObstacleType(H5Easy::load<int>(file,group.getPath() + "/" + child  + "/type"));
                obs.type_description = H5Easy::load<std::string>(file,group.getPath() + "/" + child  + "/description");
                auto points = readMatrix(file,group.getPath() + "/" + child  + "/boundary");
                toPointVec(points,obs.boundary);
                subfield.obstacles.push_back(obs);
            }
        }
        // -----------------------------
        // Tracks
        // -----------------------------
        subfield.tracks = readTracks(file,subfield_group);


        // -----------------------------
        // resource_points
        // -----------------------------
        current_node_str = "resource_points";
        if(subfield_group.exist(current_node_str))
        {
            auto group = subfield_group.getGroup(current_node_str);
            std::vector<std::string> child_list = group.listObjectNames();
            for(auto&& child : child_list)
            {
                size_t child_id = std::stoi(child);

                ResourcePoint rp;
                rp.id = child_id;
                auto point = H5Easy::load<boost::multi_array<double, 2>>(file,group.getPath() + "/" + child  + "/position");
                rp.x = point[0][0];
                rp.y = point[0][1];
                rp.z = point[0][2];
                auto types = H5Easy::load<std::vector<int>>(file,group.getPath() + "/" + child  + "/types");
                for(auto type : types)
                {
                    rp.resourceTypes.insert(ResourcePoint::intToResourceType(type));
                }
                auto points = readMatrix(file,group.getPath() + "/" + child  + "/geometry");
                toPointVec(points,rp.geometry.points);

                rp.defaultUnloadingTime = H5Easy::load<double>(file,group.getPath() + "/" + child  + "/defaultUnloadingTime");
                rp.defaultUnloadingTimePerKg = H5Easy::load<double>(file,group.getPath() + "/" + child  + "/defaultUnloadingTimePerKg");
                subfield.resource_points.push_back(rp);
            }
        }



        // -----------------------------
        // reference_lines
        // -----------------------------
        current_node_str = "reference_lines";
        if(subfield_group.exist(current_node_str))
        {
            auto group = subfield_group.getGroup(current_node_str);
            std::vector<std::string> child_list = group.listObjectNames();
            for(auto&& child : child_list)
            {
                size_t child_id = std::stoi(child);
                auto points = readMatrix(file,group.getPath() + "/" + child  + "/coordinates");
                subfield.reference_lines.push_back(toLineStringNew(points,child_id));
            }
        }
        // -----------------------------
        // working_direction
        // -----------------------------
        subfield.working_direction = H5Easy::load<double>(file,subfield_group.getPath() + "/working_direction");
        return subfield;
    }
    std::vector<std::string> listNodes(std::string file_path, std::string node_path)
    {
        std::vector<std::string> names;
        try
        {
            HighFive::File hdf5_file(file_path, HighFive::File::ReadOnly);
            if(hdf5_file.exist(node_path))
            {
                auto g =  hdf5_file.getGroup(node_path);
                names = g.listObjectNames();
            }
        }
        catch (HighFive::Exception &e) {
            std::cerr << "HDF Exception" << e.what() << std::endl;
            return std::vector<std::string>();
        }
        return names;
    }
    void writeTracks(const std::vector<arolib::Track>& tracks, std::string path, HighFive::File &file, H5Easy::DumpOptions &opt)
    {
        for(size_t i = 0 ; i < tracks.size(); i++)
        {
            auto group_path = path + "/" + string_format("tracks/%05u",tracks[i].id);
            H5Easy::dump(file, group_path + "/type",(int)tracks[i].type,opt);
            H5Easy::dump(file, group_path + "/width",tracks[i].width,opt);
            writePointsAsMatrix(tracks[i].points,group_path + "/points",file,opt);
            writePointsAsMatrix(tracks[i].boundary.points,group_path + "/boundary",file,opt);
        }
    }

    std::vector<arolib::Track> readTracks(HighFive::File &file, HighFive::Group& track_group)
    {
        std::vector<arolib::Track> tracks;
        // -----------------------------
        // Track
        // -----------------------------
        std::string current_node_str = "tracks";
        if(track_group.exist(current_node_str))
        {
            auto group = track_group.getGroup(current_node_str);
            std::vector<std::string> child_list = group.listObjectNames();
            for(auto&& child : child_list)
            {
                arolib::Track track;
                auto points = readMatrix(file,group.getPath() + "/" + child  + "/points");
                auto boundary_points = readMatrix(file,group.getPath() + "/" + child + "/boundary_out");
                track.type = Track::intToTrackType(H5Easy::load<int>(file,group.getPath() + "/" + child + "/type"));
                track.width = H5Easy::load<double>(file,group.getPath() + "/" + child + "/width");
                // convert to arolib objects
                size_t child_id = std::stoi(child);
                track.id = child_id;
                toPointVec(points,track.points);
                toPointVec(boundary_points,track.boundary);
                tracks.push_back(track);

            }
        }
        return tracks;
    }



    template<typename ... Args>
    std::string string_format( const std::string& format, Args ... args )
    {
        int size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
        if( size <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
        std::unique_ptr<char[]> buf( new char[ size ] );
        snprintf( buf.get(), size, format.c_str(), args ... );
        return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
    }


    void toPointVec(boost::multi_array<double,2>& array_in, std::vector<arolib::Point>& points_out)
    {
        auto num_points = array_in.shape()[0];
        auto dim        = array_in.shape()[1];
        // points should be 3d
        if(dim != 3) return;
        // make sure linestring is empty
        points_out.clear();
        for(size_t i = 0 ; i < num_points ; i++)
        {
            points_out.emplace_back(array_in[i][0],array_in[i][1],array_in[i][2]);
        }
    }

    void toPointVec(boost::multi_array<double,2>& array_in, arolib::Polygon& poly_out)
    {
        auto num_points = array_in.shape()[0];
        auto dim        = array_in.shape()[1];
        // points should be 3d
        if(dim != 3) return;
        // make sure linestring is empty
        poly_out.points.clear();
        for(size_t i = 0 ; i < num_points ; i++)
        {
            poly_out.points.emplace_back(array_in[i][0],array_in[i][1],array_in[i][2]);
        }
        arolib::geometry::correct_polygon(poly_out);
    }


    void toLineString(boost::multi_array<double,2>& array_in, size_t& id, arolib::Linestring& linestring_out)
    {
        linestring_out.id = id;
        toPointVec(array_in,linestring_out.points);
    }

    arolib::Linestring toLineStringNew(boost::multi_array<double,2>& array_in, size_t& id)
    {
        arolib::Linestring ls;
        toLineString(array_in,id,ls);
        return ls;
    }

    boost::multi_array<double,2> readMatrix(HighFive::File& file, std::string path)
    {
        boost::multi_array<double,2> mat;
        if(file.exist(path))
        {
            auto nmat = H5Easy::load<boost::multi_array<double, 2>>(file,path);
            return nmat;
        }
        return mat;
    }

    void writePointsAsMatrix(const std::vector<arolib::Point>& points, std::string path, HighFive::File &file, H5Easy::DumpOptions &opt)
    {
        if(points.empty()) return;
        size_t num_points = points.size();
        size_t dim = 3;
        boost::multi_array<double, 2> matrix(boost::extents[num_points][dim]);
        for(size_t i = 0 ; i < points.size() ; i++)
        {
            matrix[i][0] = points[i].x;
            matrix[i][1] = points[i].y;
            matrix[i][2] = points[i].z;
        }

        if (file.exist(path))
        {
            file.unlink(path);
        }
        size_t found = path.find_last_of("/\\");
        std::string group_path = path.substr(0,found);
        std::string dataset_name = path.substr(found+1);
        if(!file.exist(group_path))
        {
            HighFive::Group g = file.createGroup(group_path);
            auto dataset = g.createDataSet<double>(dataset_name, HighFive::DataSpace::From(matrix));
            dataset.write(matrix);

        }
        else
        {
            HighFive::Group g  = file.getGroup(group_path);
            auto dataset = g.createDataSet<double>(dataset_name, HighFive::DataSpace::From(matrix));
            dataset.write(matrix);


        }
    }

    bool areValuesEqual(double a, double b){
        if(std::isnan(a) && std::isnan(b))
            return true;
        return a == b;
    }
}

bool read_field_hdf5(const std::string& file_path, const std::string& field_name, arolib::Field &f)
{
    try
    {
        HighFive::File hdf5_file(file_path, HighFive::File::ReadOnly);

        // throws exception if field not found and function returns false
        auto field_group = hdf5_file.getGroup("/field_geometries/" + field_name);

        f.id = H5Easy::load<int>(hdf5_file,field_group.getPath() + "/id");

        // -----------------------------
        // External roads
        // -----------------------------
        std::string current_node_str = "external_roads";
        if(field_group.exist(current_node_str))
        {
            auto group = field_group.getGroup(current_node_str);
            std::vector<std::string> child_list = group.listObjectNames();
            for(auto&& child : child_list)
            {
                auto points = readMatrix(hdf5_file,group.getPath() + "/" + child + "/coordinates");
                // convert to arolib objects
                size_t child_id = std::stoi(child);
                f.external_roads.push_back(toLineStringNew(points,child_id));
            }
        }

        // -----------------------------
        // Boundary
        // -----------------------------
        current_node_str = "boundary_outer";
        if(field_group.exist(current_node_str))
        {
            auto points = readMatrix(hdf5_file,field_group.getPath() + "/" + current_node_str);
            toPointVec(points,f.outer_boundary);
        }

        // -----------------------------
        // Subfields
        // -----------------------------
        current_node_str = "subfields";
        if(field_group.exist(current_node_str))
        {
            // Process each subfield
            auto group = field_group.getGroup(current_node_str);
            std::vector<std::string> child_list = group.listObjectNames();
            f.subfields.clear();
            for(auto&& child : child_list)
            {
                // convert to arolib objects
                auto subgroup = group.getGroup(child);
                Subfield s = readSubField(hdf5_file,subgroup);
                f.subfields.push_back(s);
            }
        }

        f.filename = file_path;
        f.name = field_name;

        return true;

    }
    catch(HighFive::Exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }


}


bool write_field_hdf5(const std::string& file_path, const std::string& field_name, const arolib::Field &f)
{
     try {
         HighFive::File file(file_path, HighFive::File::OpenOrCreate);

         if(!file.exist("/field_geometries"))
         {
             file.createGroup("field_geometries");
         }

         auto field_group = file.getGroup("field_geometries");

         if (field_group.exist(field_name))
         {
             field_group.unlink(field_name);
             std::cout << field_name << " already exists" << std::endl;
             file.flush();
         }
         H5Easy::DumpOptions dump_ops(H5Easy::Compression(), H5Easy::DumpMode::Overwrite);

         std::string base_path = "field_geometries/" + field_name;

         auto write_points = std::bind(&writePointsAsMatrix,std::placeholders::_1,std::placeholders::_2, file, dump_ops);

         H5Easy::dump(file, base_path + "/id",f.id,dump_ops);

         // -----------------------------
         // external_roads
         // -----------------------------
         for(size_t i = 0 ; i < f.external_roads.size(); i++)
         {
            auto&& road = f.external_roads[i];
            std::string write_path = base_path + "/" + string_format("external_roads/%05u/coordinates",road.id);
            write_points(road.points, write_path);
         }

         // -----------------------------
         // boundary_outer
         // -----------------------------
         write_points(f.outer_boundary.points, base_path + "/boundary_outer");

         // -----------------------------
         // subfields
         // -----------------------------
         for(size_t i = 0 ; i < f.subfields.size(); i++)
         {
             auto&& subfield = f.subfields[i];
             // -----------------------------
             // subfields/access_points
             // -----------------------------
             for(size_t j = 0 ; j < subfield.access_points.size(); j++)
             {
                 auto && access_point = subfield.access_points[j];
                 auto group_path = base_path + "/" + string_format("subfields/%05u/access_points/%05u",subfield.id,access_point.id);
                 std::vector<arolib::Point> ptvec;
                 ptvec.push_back(access_point.point());
                 write_points(ptvec, group_path + "/position" );
                 H5Easy::dump(file, group_path + "/type",(int)access_point.accessType,dump_ops);
             }
             // -----------------------------
             // subfields/boundary_outer
             // -----------------------------
             write_points(subfield.boundary_outer.points, base_path + "/" + string_format("subfields/%05u/boundary_outer",subfield.id));

             // -----------------------------
             // subfields/boundary_innter
             // -----------------------------
             write_points(subfield.boundary_inner.points, base_path + "/" + string_format("subfields/%05u/boundary_inner",subfield.id));

             // -----------------------------
             // subfields/headlands/complete
             // -----------------------------
             {
                 auto group_path = base_path + "/" + string_format("subfields/%05u/headlands/complete",subfield.id);
                 H5Easy::dump(file, group_path + "/width",subfield.headlands.complete.headlandWidth,dump_ops);
                 write_points(subfield.headlands.complete.middle_track.points,group_path + "/middle_track");
                 writeTracks(subfield.headlands.complete.tracks,group_path,file,dump_ops);
                 write_points(subfield.headlands.complete.boundaries.first.points,group_path + "/boundary_out");
                 write_points(subfield.headlands.complete.boundaries.second.points,group_path + "/boundary_in");
             }

             // -----------------------------
             // subfields/obstacles
             // -----------------------------
             for(size_t j = 0; j < subfield.obstacles.size(); j++)
             {
                 auto&& obs = subfield.obstacles[j];
                 auto group_path = base_path + "/" + string_format("subfields/%05u/obstacles/%05u",subfield.id, j);
                 H5Easy::dump(file, group_path + "/type",(int)obs.type,dump_ops);
                 H5Easy::dump(file, group_path + "/description",obs.type_description,dump_ops);
                 write_points(obs.boundary.points,group_path + "/boundary");
             }

             // -----------------------------
             // subfields/tracks
             // -----------------------------
             writeTracks(subfield.tracks,base_path + "/" + string_format("subfields/%05u",subfield.id),file,dump_ops);

             // -----------------------------
             // subfields/resource_points
             // -----------------------------
             for(size_t j = 0; j < subfield.resource_points.size(); j++)
             {
                 auto&& rp = subfield.resource_points[j];
                 auto group_path = base_path + "/" + string_format("subfields/%05u/resource_points/%05u",subfield.id, rp.id);
                 std::vector<arolib::Point> ptvec;
                 ptvec.push_back(rp.point());
                 write_points(ptvec, group_path + "/position" );
                 std::vector<int> resource_types;
                 for(auto&& r : rp.resourceTypes)
                 {
                     resource_types.push_back((int)r);
                 }
                 H5Easy::dump(file, group_path + "/types",resource_types,dump_ops);
                 write_points(rp.geometry.points,group_path + "/geometry");
                 H5Easy::dump(file, group_path + "/defaultUnloadingTime",rp.defaultUnloadingTime,dump_ops);
                 H5Easy::dump(file, group_path + "/defaultUnloadingTimePerKg",rp.defaultUnloadingTimePerKg,dump_ops);
             }

             // -----------------------------
             // subfields/access_points
             // -----------------------------
             for(size_t j = 0; j < subfield.access_points.size(); j++)
             {
                 auto&& rp = subfield.access_points[j];
                 auto group_path = base_path + "/" + string_format("subfields/%05u/access_points/%05u",subfield.id, rp.id);
                 std::vector<arolib::Point> ptvec;
                 ptvec.push_back(rp.point());
                 write_points(ptvec, group_path + "/position" );
                 H5Easy::dump(file, group_path + "/type",(int)rp.accessType,dump_ops);
             }

             // -----------------------------
             // subfields/reference_lines
             // -----------------------------
             for(size_t j = 0; j < subfield.reference_lines.size(); j++)
             {
                 auto&& line = subfield.reference_lines[j];
                 auto group_path = base_path + "/" + string_format("subfields/%05u/reference_lines/%05u",subfield.id, line.id);
                 write_points(line.points, group_path + "/coordinates" );
             }
             // -----------------------------
             // subfields/working_direction
             // -----------------------------
             H5Easy::dump(file, base_path + "/" + string_format("subfields/%05u/working_direction",subfield.id),subfield.working_direction,dump_ops);
         }
         return true;

     }
     catch(HighFive::Exception& e)
     {
         std::cerr << e.what() << '\n';
         throw e;
         return false;
     }
}

bool delete_field_hdf5(const std::string& file_path, const std::string& field_name)
{
    try
    {
        HighFive::File hdf5_file(file_path, HighFive::File::ReadWrite);
        std::string field_path = "/field_geometries/" + field_name;
        if(hdf5_file.exist(field_path))
        {
            auto group = hdf5_file.getGroup("field_geometries");
            group.unlink(field_name);
            hdf5_file.flush();
            return true;
        }
        else
        {
            return false;
        }
    }
    catch (HighFive::Exception &e) {
        std::cerr << "HDF Exception" << e.what() << std::endl;
        return false;
    }
}
bool read_grid_hdf5(const std::string& file_path, const std::string& grid_type, const std::string& grid_name, std::vector<arolib::ArolibGrid_t> &grids)
{
    std::cout << "READING GRID " << std::endl;
    try {
        grids.clear();
        HighFive::File file(file_path, HighFive::File::ReadOnly);
        std::cout << "OPEN H5 " << std::endl;

        std::string path = "/maps/" + grid_type + "/" + grid_name + "/map";
        if(!file.exist(path))
            return false;

        double min_x = H5Easy::loadAttribute<double>(file, path, "min_x");
        double min_y = H5Easy::loadAttribute<double>(file, path , "min_y");
        double cell_size = H5Easy::loadAttribute<double>(file, path, "cell_size");
        double no_value = H5Easy::loadAttribute<float>(file, path, "none_value");

        auto h5shape = H5Easy::getShape(file,path);
        if(h5shape.size()==2)
        {
            auto grid_array = H5Easy::load<boost::multi_array<double, 2>>(file,path);
            size_t size_x = grid_array.shape()[0];
            size_t size_y = grid_array.shape()[1];

            arolib::gridmap::GridmapLayout layout(min_x, min_y, size_x, size_y, cell_size);
            if(!layout.isValid())
                return false;

            grids.push_back( arolib::ArolibGrid_t(layout) );
            auto& grid = grids.back();

            for (size_t x = 0; x < size_x; x++) {
                for (size_t y = 0; y < size_y; y++) {
                    auto val_in = grid_array[x][y];
                    if( !areValuesEqual( val_in, no_value ) )
                    {
                        grid.setValue(x,y,val_in);
                    }
                }
            }
        }
        else if(h5shape.size()==3)
        {
            auto grid_array = H5Easy::load<boost::multi_array<double, 3>>(file,path);
            size_t size_x = grid_array.shape()[0];
            size_t size_y = grid_array.shape()[1];
            size_t max_dim = grid_array.shape()[2];

            arolib::gridmap::GridmapLayout layout(min_x, min_y, size_x, size_y, cell_size);
            if(!layout.isValid())
                return false;

            for(size_t dim = 0 ;  dim < max_dim ; dim++)
            {
                grids.push_back( arolib::ArolibGrid_t(layout) );
                auto& grid = grids.back();
                for (size_t x = 0; x < size_x; x++) {
                    for (size_t y = 0; y < size_y; y++) {
                        auto val_in = grid_array[x][y][dim];
                        if( !areValuesEqual( val_in, no_value ))
                        {
                            grid.setValue(x,y,val_in);
                        }
                    }
                }
            }
        }
        else
        {
            return false;
        }


        return true;
    }
    catch (HighFive::Exception &e) {
        std::cerr << "HDF Exception" << e.what() << std::endl;
        return false;
    }
}


bool read_grid_hdf5(const std::string& file_path, const std::string& grid_type, const std::string& grid_name, arolib::ArolibGrid_t &g)
{
    std::cout << "READING GRID " << std::endl;
     try {
         HighFive::File file(file_path, HighFive::File::ReadOnly);
         std::cout << "OPEN H5 " << std::endl;

         std::string path = "/maps/" + grid_type + "/" + grid_name + "/map";
         if(!file.exist(path)) return false;

         auto grid_array = H5Easy::load<boost::multi_array<double, 2>>(file,path);
         size_t size_x = grid_array.shape()[0];
         size_t size_y = grid_array.shape()[1];
         double min_x = H5Easy::loadAttribute<double>(file, path, "min_x");
         double min_y = H5Easy::loadAttribute<double>(file, path , "min_y");
         double cell_size = H5Easy::loadAttribute<double>(file, path, "cell_size");
         double no_value = H5Easy::loadAttribute<float>(file, path, "none_value");

         arolib::gridmap::GridmapLayout layout(min_x, min_y, size_x, size_y, cell_size);
         if(!layout.isValid())
             return false;

         g.createGrid(layout);

         for (size_t x = 0; x < size_x; x++) {
             for (size_t y = 0; y < size_y; y++) {
                 auto val_in = grid_array[x][y];
                 if( !areValuesEqual( val_in, no_value ))
                 {
                     g.setValue(x,y,val_in);
                 }
             }
         }

         return true;
     }
     catch (HighFive::Exception &e) {
         std::cerr << "HDF Exception" << e.what() << std::endl;
         return false;
     }
}


bool write_grid_hdf5(const std::string& file_path, const std::string& grid_type, const std::string& grid_name, const std::vector<const arolib::ArolibGrid_t*> &grid)
{
    try {
        if(grid.empty()) return false;

        HighFive::File file(file_path, HighFive::File::OpenOrCreate);

        if(!file.exist("/maps"))
        {
            file.createGroup("maps");
        }

        auto map_group = file.getGroup("maps");

        if (!map_group.exist(grid_type))
        {
            map_group.createGroup(grid_type);
        }

        auto map_group2 =  map_group.getGroup(grid_type);
        if(map_group2.exist(grid_name))
        {
            map_group2.unlink(grid_name);
        }


        H5Easy::DumpOptions dump_ops(H5Easy::Compression(), H5Easy::DumpMode::Overwrite);


        auto g = map_group2.createGroup(grid_name);



        size_t max_x = grid[0]->getSizeX();
        size_t max_y = grid[0]->getSizeY();
        size_t dim = grid.size();
        double no_value = std::nan("1");

        boost::multi_array<double, 3> grid_array(boost::extents[max_x][max_y][dim]);
        bool errorTmp;

        for(size_t current_dim = 0 ; current_dim < dim ; current_dim++)
        {
            for (size_t x = 0; x < max_x; x++) {
                for (size_t y = 0; y < max_y; y++) {
                    if( grid[current_dim]->hasValue(x, y, &errorTmp) && !errorTmp )
                        grid_array[x][y][current_dim] = grid[current_dim]->getValue(x, y, &errorTmp);
                    else
                        grid_array[x][y][current_dim] = no_value;
                }
            }
        }



        auto dataset = g.createDataSet<double>("map", HighFive::DataSpace::From(grid_array));
        dataset.write(grid_array);
        dataset.createAttribute("min_x",grid[0]->getMinPointX());
        dataset.createAttribute("min_y",grid[0]->getMinPointY());
        dataset.createAttribute("cell_size",grid[0]->getCellsize());
        dataset.createAttribute("none_value", no_value);

        return true;


    }
    catch(HighFive::Exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}

bool write_grid_hdf5(const std::string& file_path, const std::string& grid_type, const std::string& grid_name, const arolib::ArolibGrid_t &grid)
{
    try {
        HighFive::File file(file_path, HighFive::File::OpenOrCreate);

        if(!file.exist("/maps"))
        {
            file.createGroup("maps");
        }

        auto map_group = file.getGroup("maps");

        if (!map_group.exist(grid_type))
        {
            map_group.createGroup(grid_type);
        }

        auto map_group2 =  map_group.getGroup(grid_type);
        if(map_group2.exist(grid_name))
        {
            map_group2.unlink(grid_name);
        }


        H5Easy::DumpOptions dump_ops(H5Easy::Compression(), H5Easy::DumpMode::Overwrite);


        auto g = map_group2.createGroup(grid_name);


        double no_value = std::nan("1");
        size_t max_x = grid.getSizeX();
        size_t max_y = grid.getSizeY();
        boost::multi_array<double, 2> grid_array(boost::extents[max_x][max_y]);
        bool errorTmp;
        for (size_t x = 0; x < max_x; x++) {
            for (size_t y = 0; y < max_y; y++) {
                if(grid.hasValue(x, y, &errorTmp) && !errorTmp)
                    grid_array[x][y] = grid.getValue(x, y, &errorTmp);
                else
                    grid_array[x][y] = no_value;
            }
        }

        auto dataset = g.createDataSet<double>("map", HighFive::DataSpace::From(grid_array));
        dataset.write(grid_array);
        dataset.createAttribute("min_x",grid.getMinPointX());
        dataset.createAttribute("min_y",grid.getMinPointY());
        dataset.createAttribute("cell_size",grid.getCellsize());
        dataset.createAttribute("none_value",no_value);

        return true;


    }
    catch(HighFive::Exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}


bool delete_grid_hdf5(const std::string& file_path, const std::string& grid_type, const std::string& grid_name)
{
    try
    {
        HighFive::File hdf5_file(file_path, HighFive::File::ReadWrite);
        std::string field_path = "/maps/" + grid_type + "/" + grid_name;
        if(hdf5_file.exist(field_path))
        {
            hdf5_file.unlink(field_path);
            hdf5_file.flush();
            return true;
        }
        else
        {
            return false;
        }
    }
    catch (HighFive::Exception &e) {
        std::cerr << "HDF Exception" << e.what() << std::endl;
        return false;
    }
}

int get_grid_layer_count_hdf5(const std::string& file_path, const std::string& grid_type, const std::string& grid_name)
{
    try
    {
        HighFive::File hdf5_file(file_path, HighFive::File::ReadOnly);
        std::string field_path = "/maps/" + grid_type + "/" + grid_name + "/map";
        if(hdf5_file.exist(field_path))
        {
            auto shape = H5Easy::getShape(hdf5_file, field_path);
            if(shape.size() > 3 || shape.size() < 2) return -1;
            else if(shape.size()==2) return 1;
            else
            {
                return shape[2];
            }
        }
        else
        {
            return -1;
        }
    }
    catch (HighFive::Exception &e) {
        std::cerr << "HDF Exception" << e.what() << std::endl;
        return -1;
    }
}

std::vector<std::string> get_field_names_hdf5(const std::string& file_path)
{
    return listNodes(file_path, "/field_geometries");
}


std::vector<std::string> get_grid_types_hdf5(const std::string& file_path)
{
    return listNodes(file_path, "/maps");
}

std::vector<std::string> get_grid_names_hdf5(const std::string& file_path, const std::string& grid_type)
{
    return listNodes(file_path, "/maps/" + grid_type);
}


std::map< std::string, std::vector<HDF5GridInfo> > get_grid_list_hdf5(const std::string& file_path)
{
    std::map< std::string, std::vector<HDF5GridInfo> > grids_info;
    auto grid_types = get_grid_types_hdf5(file_path);
    for(auto&& grid_type : grid_types)
    {
        std::vector<HDF5GridInfo> infos = get_grid_list_hdf5(file_path,grid_type);
        grids_info[grid_type] = infos;
    }
    return grids_info;
}


std::vector<HDF5GridInfo> get_grid_list_hdf5(const std::string& file_path, const std::string& grid_type)
{
    std::vector<HDF5GridInfo> infos;
    auto grid_list =  get_grid_names_hdf5(file_path, grid_type);
    for(auto && grid_name : grid_list)
    {
        HDF5GridInfo info;
        info.id = grid_name;
        info.layers = get_grid_layer_count_hdf5(file_path, grid_type,  grid_name);
        infos.push_back(info);
    }
    return infos;
}

}
}
