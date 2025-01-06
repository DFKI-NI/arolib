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

#include <fstream>
#include <set>

#include <boost/test/unit_test.hpp>

#include "arolib/misc/filesystem_helper.h"

using namespace arolib::io;

BOOST_AUTO_TEST_SUITE(test_filesystem)


BOOST_AUTO_TEST_CASE(test_int)
{
    const auto sep = get_path_separator();

    auto p1 = create_path_2( true, true, "a", "b", "c" );
    auto p2 = create_path_2( true, false, "a", "b", "c" );
    auto p3 = create_path_2( false, true, "a", "b", "c" );
    auto p4 = create_path_2( false, false, "a", "b", "c" );
    auto p5 = create_path( "a", "b", "c" );

    BOOST_CHECK_EQUAL(p1, sep + std::string("a") + sep + "b" + sep + "c" + sep );
    BOOST_CHECK_EQUAL(p2, sep + std::string("a") + sep + "b" + sep + "c"       );
    BOOST_CHECK_EQUAL(p3,       std::string("a") + sep + "b" + sep + "c" + sep );
    BOOST_CHECK_EQUAL(p4,       std::string("a") + sep + "b" + sep + "c"       );
    BOOST_CHECK_EQUAL(p4, p5);

    auto p11 = create_path_2( true, true, "a" );
    auto p12 = create_path_2( true, false, "a" );
    auto p13 = create_path_2( false, true, "a" );
    auto p14 = create_path_2( false, false, "a" );
    auto p15 = create_path( "a" );

    BOOST_CHECK_EQUAL(p11, sep + std::string("a") + sep );
    BOOST_CHECK_EQUAL(p12, sep + std::string("a")       );
    BOOST_CHECK_EQUAL(p13,       std::string("a") + sep );
    BOOST_CHECK_EQUAL(p14,       std::string("a")       );
    BOOST_CHECK_EQUAL(p14, p15);

    std::string filepath = create_path( p1, "d.cpp" );
    BOOST_CHECK_EQUAL(filepath, p1 + "d.cpp");
    BOOST_CHECK( file_has_extension(filepath, "cpp") );
    BOOST_CHECK( file_has_extension(filepath, "CPP", false) );
    BOOST_CHECK( !file_has_extension(filepath, "CPP", true) );
    BOOST_CHECK( !file_has_extension(filepath, "hpp") );
    BOOST_CHECK( !file_has_extension(p1, "cpp") );


    auto filename = get_filename(filepath, false);
    auto filename2 = get_filename(filepath, true);
    BOOST_CHECK_EQUAL(filename, "d.cpp");
    BOOST_CHECK_EQUAL(filename2, "d");

    auto dir = get_path_to_file( filepath );
    append_separator_to_path(dir);
    BOOST_CHECK_EQUAL(dir, p1);

    append_separator_to_path(p2);
    BOOST_CHECK_EQUAL(p1, p2);
    append_separator_to_path(p2);
    BOOST_CHECK_EQUAL(p1, p2);

    prepend_separator_to_path(p3);
    BOOST_CHECK_EQUAL(p1, p3);
    prepend_separator_to_path(p3);
    BOOST_CHECK_EQUAL(p1, p3);

    auto tmpdir = create_path( get_temp_dir(), "arolib", "test", "test_misc", "test_filesystem" );
    BOOST_REQUIRE( create_directory(tmpdir, true) );
    BOOST_CHECK( dir_exists(tmpdir) );
    BOOST_CHECK( file_or_dir_exists(tmpdir) );
    append_separator_to_path(tmpdir);
    BOOST_CHECK( dir_exists(tmpdir) );
    BOOST_CHECK( file_or_dir_exists(tmpdir) );

    std::string txt_out = "abc";
    filename = create_path( tmpdir, "test.txt");
    std::ofstream file_out( filename );
    BOOST_REQUIRE( file_out.is_open() );
    file_out << txt_out;
    file_out.close();
    BOOST_REQUIRE( file_or_dir_exists(filename) );

    std::string txt_in;
    std::ifstream file_in( filename );
    BOOST_REQUIRE( file_in.is_open() );
    file_in >> txt_in;
    file_in.close();
    BOOST_CHECK_EQUAL(txt_in, txt_out);


    BOOST_REQUIRE( create_directory(tmpdir, false) );
    BOOST_CHECK( file_or_dir_exists(filename) );
    BOOST_REQUIRE( create_directory(tmpdir, true) );
    BOOST_CHECK( !file_or_dir_exists(filename) );

    std::vector<std::string> dirs, paths, dirs_1, dirs_1_comp, dirs_12, dirs_12_comp;
    for(int i = 1 ; i < 4 ; ++i){
        dirs_1.emplace_back( "d" + std::to_string(i) );
        dirs_1_comp.emplace_back( create_path( tmpdir, dirs_1.back() ) );
        dirs_12.emplace_back( dirs_1.back() );
        for(int j = 1 ; j < 3 ; ++j){
            dirs_12.emplace_back( create_path( dirs_1.back(), "sd" + std::to_string(j) ) );
            dirs.emplace_back( dirs_12.back() ) ;
            auto dir = create_path(tmpdir, dirs.back());
            BOOST_REQUIRE( create_directory(dir, false) );
            for(int k = 1 ; k < 3 ; ++k){
                paths.emplace_back( create_path(dir, "f" + std::to_string(k) + (k == 1 ? ".cpp" : ".hpp") ) );
                std::ofstream file_out( paths.back() );
                file_out << paths.back();
                file_out.close();
            }
        }
    }

    {
        auto dirs_1_in = get_directories(tmpdir, false);
        auto dirs_1_comp_in = get_directories(tmpdir, true);
        auto dirs_in = get_directories_recursive(tmpdir, false, false);
        auto dirs_comp_in = get_directories_recursive(tmpdir, false, true);
        auto dirs_12_in = get_directories_recursive(tmpdir, true, false);
        auto dirs_12_comp_in = get_directories_recursive(tmpdir, true, true);

        std::vector< std::pair<std::vector<std::string>*, std::set<std::string>> > out_in;
        out_in.emplace_back( std::make_pair(&dirs_1,  std::set(dirs_1_in.begin(), dirs_1_in.end())) );
        out_in.emplace_back( std::make_pair(&dirs_1,  std::set(dirs_1_comp_in.begin(), dirs_1_comp_in.end())) );
        out_in.emplace_back( std::make_pair(&dirs,    std::set(dirs_in.begin(), dirs_in.end())) );
        out_in.emplace_back( std::make_pair(&dirs,    std::set(dirs_comp_in.begin(), dirs_comp_in.end())) );
        out_in.emplace_back( std::make_pair(&dirs_12, std::set(dirs_12_in.begin(), dirs_12_in.end())) );
        out_in.emplace_back( std::make_pair(&dirs_12, std::set(dirs_12_comp_in.begin(), dirs_12_comp_in.end())) );

        for(size_t i = 0 ; i < out_in.size() ; ++i){
            auto& _out = *out_in[i].first;
            auto& _in = out_in[i].second;
            BOOST_REQUIRE_EQUAL( _out.size(), _in.size() );
            for(auto& d : _out){{
                    auto p = ( i%2 == 0 ? d : create_path(tmpdir, d) );
                    BOOST_CHECK( _in.find( p ) != _in.end() );
                }
            }
        }
    }

    {
        auto paths_in = get_filenames_recursive(tmpdir, "", true, true);
        auto paths_in_2 = get_filenames_recursive(tmpdir, "", false, true);
        auto filenames_in = get_filenames_recursive(tmpdir, "", false, false);
        std::set paths_in_set(paths_in.begin(), paths_in.end());
        std::set paths_in_2_set(paths_in_2.begin(), paths_in_2.end());
        std::multiset filenames_in_set(filenames_in.begin(), filenames_in.end());
        BOOST_REQUIRE_EQUAL( paths.size(), paths_in_set.size() );
        BOOST_REQUIRE_EQUAL( paths.size(), paths_in_2_set.size() );
        BOOST_REQUIRE_EQUAL( paths.size(), filenames_in_set.size() );
        for(auto& p : paths){
            auto path_2 = p;
            remove_dir_from_filename( path_2, tmpdir );
            auto filename = get_filename( p, false );
            BOOST_CHECK( paths_in_set.find( p ) != paths_in_set.end() );
            BOOST_CHECK( paths_in_2_set.find( path_2 ) != paths_in_2_set.end() );
            auto it = filenames_in_set.find( filename );
            BOOST_REQUIRE( it != filenames_in_set.end() );
            filenames_in_set.erase(it);
        }

        BOOST_CHECK_EQUAL( 2 * get_filenames_recursive(tmpdir, ".cpp", false, false).size(), paths_in.size() );
    }

}


BOOST_AUTO_TEST_SUITE_END()
