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
 
#include "arolib/io/io_kml.hpp"

namespace arolib {
namespace io {


namespace{//to avoid linking problems


    std::vector<ResourcePoint> read_resource_points(std::string filename, std::string fieldname) {
        std::vector<ResourcePoint> points;

        int index = filename.find(".kml");
        if (index != std::string::npos) {
            filename.replace(index, 4, "_" + fieldname + ".rp");
        }
        std::cout << "   * trying to read " << filename << std::endl;

        std::ifstream in(filename);
        Point p_wgs, p_utm;
        while (in.good()) {
            in >> p_wgs.x >> p_wgs.y;

            //if (in.good()) {
            p_utm = p_wgs;
            arolib::CoordTransformer::GetInstance().convert_to_cartesian(p_wgs, p_utm);

            points.push_back( ResourcePoint(p_utm, points.size()) );
            //}
        }
        in.close();
        in.clear();
        return points;
    }


    /**
    * @brief stringToPointList parses the given string for coordinates and fills
    *        the given points vector.
    * @param coordinates the coordinates string
    * @param pts the point vector filled in this function
    */
    void stringToPointList(std::string coordinates, std::vector<Point>& pts){
        if(coordinates.empty())
            return;
        boost::trim(coordinates);
        std::vector<std::string> strs;
        boost::split(strs, coordinates, boost::is_any_of(" "));
        //        std::cout << "split: " << coordinates << std::endl;

        for(unsigned int i = 0; i < strs.size(); i++){
            std::vector<std::string> xyz;
            //            std::cout << "   split-2: " << strs.at(i) << std::endl;
            boost::split(xyz, strs.at(i), boost::is_any_of(","));

            // sometimes, the points come with 2 coordinates, sometimes 3, .... whatever. It's a bad world.
            Point pt_geo;
            if (xyz.size() == 2)
                pt_geo = Point(string2double(xyz[0]), string2double(xyz[1]), 0);
            else if (xyz.size() == 3)
                pt_geo = Point(string2double(xyz[0]), string2double(xyz[1]), string2double(xyz[2]));
            else {
                std::cout << "ERROR: Cannot parse/split the point " << strs.at(i) << std::endl;
                continue;
            }

            Point pt = pt_geo;
            arolib::CoordTransformer::GetInstance().convert_to_cartesian(pt_geo, pt);
            pts.push_back(pt);
        }
    }


    /**
    * @brief stringToPointList parses the given string for coordinates and fills
    *        the given points vector.
    * @param coordinates the coordinates string
    * @param pts the point vector filled in this function
    */
    void stringToPointList(std::string coordinates, std::vector<ResourcePoint>& pts){
        std::vector<Point> points;
        stringToPointList(coordinates, points);
        pts.resize( points.size() );
        for(size_t i = 0 ; i < points.size(); ++i)
            pts[i] = ResourcePoint( points[i] );
    }


}//end namespace


bool readFieldKML(const std::string &filename, Field &field, Point::ProjectionType coordinatesType_out) {
    return AroKMLInDocument::readField(filename, field, coordinatesType_out);
}

bool readFieldsKML(const std::string &filename, std::vector<Field> &fields, Point::ProjectionType coordinatesType_out)
{
    return AroKMLInDocument::readFields(filename, fields, coordinatesType_out);
}

bool writeFieldKML(const std::string& filename, const arolib::Field& field, Point::ProjectionType coordinatesType_in) {
//        std::ofstream out(filename.c_str());
//        if (!out.good()) return false;

//        out<<std::setprecision(12);
//        out<<"<?xml version='1.0' encoding='UTF-8'?>\n";
//        out<<"<kml xmlns='http://www.opengis.net/kml/2.2'>\n";
//        out<<"\t<Document>\n";
//        out<<"\t\t<name>"<< field.name <<"</name>\n";
//        out<<"\t\t<description><![CDATA[]]></description>\n";
//        for(unsigned int i = 0; i < field.subfields.size(); ++i) {
//            addSubfield(field.subfields.at(i), i, out);
//        }
//        addStyle(out,"outer_boundary_style","ffCCA90B",2);
//        addStyle(out,"inner_boundary_style","ff4A1BA6",1);
//        addStyle(out,"reference_line_style","ffA3EEE6",1);
//        addStyle(out,"resource_point_style","00FFFF0B",2);
//        addStyle(out,"access_point_style","FF00FF0B",2);
//        addStyle(out,"track_style","ff969CEE",1);
//        addStyle(out,"obstacle_style","ff000000",1);
//        addStyle(out,"headlandtrack_style","ff000000",1);
//        out<<"\t</Document>\n";
//        out<<"</kml>";
//        return true;

    return AroKMLOutDocument::saveField(filename, field, coordinatesType_in);
}

bool writeFieldsKML(const std::string& filename, const std::vector<arolib::Field>& fields, Point::ProjectionType coordinatesType_in)
{
    return AroKMLOutDocument::saveFields(filename, fields, coordinatesType_in);
}

bool read_field_kml_best_guess(const std::string& filename, std::vector<arolib::Field>& fields) {
    try{
        using boost::property_tree::ptree;
        boost::optional< ptree& > poly;
        boost::optional< ptree& > linestring;
        std::vector<Polygon> polys;
        std::vector<Linestring> lines;
        std::vector<std::string> names;
        ptree pt;
        bool is_inside;
        std::string name;
        read_xml(filename, pt);

        std::cout << "   Read file " << filename << std::endl;

        // first get all polygons and linestrings from the kml file
        BOOST_FOREACH(boost::property_tree::ptree::value_type & v,pt.get_child("kml.Document")) {
            poly = v.second.get_child_optional( "Polygon" );

            linestring = v.second.get_child_optional( "LineString" );
            if(poly){
                Polygon p;
                stringToPointList(poly.get().get<std::string>("coordinates"),p.points);
                polys.push_back(p);
                name = v.second.get<std::string>("name");
                names.push_back(name);
                continue;
            }
            if(linestring) {
                Linestring l;
                stringToPointList(linestring.get().get<std::string>("coordinates"),l.points);
                lines.push_back(l);
                continue;
            }
            if(v.first == "Folder") {
                BOOST_FOREACH(boost::property_tree::ptree::value_type & node,v.second) {
                    poly = node.second.get_child_optional( "Polygon" );
                    linestring = node.second.get_child_optional( "LineString" );
                    if(poly){
                        Polygon p;
                        stringToPointList(poly.get().get<std::string>("outerBoundaryIs.LinearRing.coordinates"),p.points);
                        polys.push_back(p);
                        name = node.second.get<std::string>("name");
                        std::cout << "   Processing " << name << std::endl;
                        names.push_back(name);
                        continue;
                    }
                    if(linestring) {
                        Linestring l;
                        stringToPointList(linestring.get().get<std::string>("coordinates"),l.points);
                        lines.push_back(l);
                        continue;
                    }
                }
            }
        }
        // if a polygon is not inside another polygon. Create a Field and make the
        // polygon its outer border
        for(unsigned int i = 0; i < polys.size(); ++i) {
            is_inside = false;
            for(unsigned int j = 0; j < polys.size(); ++j) {
                if(i==j) continue;
                if(arolib::geometry::in_polygon(polys.at(i),polys.at(j))){
                    is_inside = true;
                }
            }
            if(!is_inside) {
                Field field;
                field.id = fields.size();
                field.name = names.at(i);
                field.filename = filename;
                field.outer_boundary = polys.at(i);
                geometry::correct_polygon(field.outer_boundary);

                Subfield s;
                s.id = 0;
                s.boundary_outer = polys.at(i);
#ifdef DEBUG
                std::cout<<std::endl<<"nw field "<< field.name.c_str()<<std::endl;
                for (int j = 0; j < s.boundary_outer.points.size(); ++j) {
                    std::cout<<std::setprecision(12)<<s.boundary_outer.points.at(j)<<std::endl;
                }
#endif
                geometry::correct_polygon(s.boundary_outer);
#ifdef DEBUG
                std::cout<<"after correction"<<std::endl;
                for (int j = 0; j < s.boundary_outer.points.size(); ++j) {
                    std::cout<<std::setprecision(12)<<s.boundary_outer.points.at(j)<<std::endl;
                }
#endif

                // dealing with ressource points for this field:
                std::vector<ResourcePoint> resource_points = read_resource_points(filename, field.name);
                if (resource_points.size() == 0) std::cout << "     no ressource points found" << std::endl;
                else std::cout << "     ressource points:" << std::endl;
                for (size_t i=0; i < resource_points.size(); i++){
                    resource_points.at(i).id = i; //@TODO: temporary workaround. should be read from the file as well, together with the other resource point parameters
                    std::cout << "       " << resource_points[i] << std::endl;
                }
                std::cout << std::endl;

                s.resource_points = resource_points;
                field.subfields.push_back(s);
                fields.push_back(field);
            } else {
            }
        }
        // if a Polygon is inside a Field add it as obstacle to the field
        // if a Linestring is inside a Field add it as reference line to the field
        for(unsigned int i = 0; i < fields.size();++i) {
            for(unsigned int j =0; j < lines.size();++j) {
                //if(arolib::geometry::in_polygon(lines.at(j),fields.at(i).outer_boundary))
                    fields.at(i).subfields.at(0).reference_lines.push_back(lines.at(j));

            }
            for(unsigned int j = 0; j < polys.size(); ++j) {
                if(arolib::geometry::in_polygon(polys.at(j),fields.at(i).outer_boundary)){
                    Obstacle obs;
                    obs.boundary = polys.at(j);
                    obs.type = Obstacle::OBS_OTHER;
                    fields.at(i).subfields.at(0).obstacles.push_back(obs);
                }
            }
        }
        return !fields.empty();
    }
    catch(...){
        return false;
    }
}



bool read_field_kml_best_guess__ed(const std::string &filename, std::vector<Field> &fields)
{
    try{
        std::string refLineNameTrailer = "_RL";
        using boost::property_tree::ptree;
        boost::optional< ptree& > poly;
        boost::optional< ptree& > linestring;
        std::vector<Polygon> polys;
        std::map< std::string, std::vector<Linestring> > lines;
        std::vector<std::string> names;
        ptree pt;
        bool is_inside;
        std::string name;
        read_xml(filename, pt);

        std::cout << "   Read file " << filename << std::endl;

        // first get all polygons and linestrings from the kml file
        BOOST_FOREACH(boost::property_tree::ptree::value_type & v,pt.get_child("kml.Document")) {
            poly = v.second.get_child_optional( "Polygon" );

            linestring = v.second.get_child_optional( "LineString" );
            if(poly){
                Polygon p;
                stringToPointList(poly.get().get<std::string>("coordinates"),p.points);
                polys.push_back(p);
                name = v.second.get<std::string>("name");
                names.push_back(name);
                continue;
            }
            if(linestring) {
                Linestring l;
                stringToPointList(linestring.get().get<std::string>("coordinates"),l.points);
                name = v.second.get<std::string>("name");
                if(name.size() < refLineNameTrailer.size())
                    name = "";
                else
                    name = name.substr( 0, name.size()-refLineNameTrailer.size() );
                lines[name].push_back(l);
                continue;
            }
            if(v.first == "Folder") {
                BOOST_FOREACH(boost::property_tree::ptree::value_type & node,v.second) {
                    poly = node.second.get_child_optional( "Polygon" );
                    linestring = node.second.get_child_optional( "LineString" );
                    if(poly){
                        Polygon p;
                        stringToPointList(poly.get().get<std::string>("outerBoundaryIs.LinearRing.coordinates"),p.points);
                        polys.push_back(p);
                        name = node.second.get<std::string>("name");
                        std::cout << "   Processing " << name << std::endl;
                        names.push_back(name);
                        continue;
                    }
                    if(linestring) {
                        Linestring l;
                        stringToPointList(linestring.get().get<std::string>("coordinates"),l.points);
                        name = node.second.get<std::string>("name");
                        if(name.size() < refLineNameTrailer.size())
                            name = "";
                        else
                            name = name.substr( 0, name.size()-refLineNameTrailer.size() );
                        lines[name].push_back(l);
                        continue;
                    }
                }
            }
        }
        // if a polygon is not inside another polygon. Create a Field and make the
        // polygon its outer border
        for(unsigned int i = 0; i < polys.size(); ++i) {
            is_inside = false;
            for(unsigned int j = 0; j < polys.size(); ++j) {
                if(i==j) continue;
                if(arolib::geometry::in_polygon(polys.at(i),polys.at(j))){
                    is_inside = true;
                }
            }
            if(!is_inside) {
                Field field;
                field.id = fields.size();
                field.name = names.at(i);
                field.filename = filename;
                field.outer_boundary = polys.at(i);
                geometry::correct_polygon(field.outer_boundary);

                Subfield s;
                s.id = 0;
                s.boundary_outer = polys.at(i);

                // dealing with ressource points for this field:
                std::vector<ResourcePoint> resource_points = read_resource_points(filename, field.name);
                if (resource_points.size() == 0) std::cout << "     no ressource points found" << std::endl;
                else std::cout << "     ressource points:" << std::endl;
                for (size_t i=0; i < resource_points.size(); i++)
                    std::cout << "       " << resource_points[i] << std::endl;
                std::cout << std::endl;

                s.resource_points = resource_points;
                field.subfields.push_back(s);
                fields.push_back(field);
            } else {
            }
        }
        // if a Polygon is inside a Field add it as obstacle to the field
        // if a Linestring is inside a Field add it as reference line to the field
        for(auto & field : fields) {
            auto it = lines.find( field.name);
            if(it == lines.end())
                continue;

            for(auto & sf : field.subfields)
                sf.reference_lines = it->second;

            for(unsigned int j = 0; j < polys.size(); ++j) {
                if(arolib::geometry::in_polygon(polys.at(j),field.outer_boundary)){
                    Obstacle obs;
                    obs.boundary = polys.at(j);
                    obs.type = Obstacle::OBS_OTHER;
                    field.subfields.at(0).obstacles.push_back(obs);
                }
            }
        }
        return !fields.empty();
    }
    catch(...){
        return false;
    }

}


}

}//end namespace arolib
