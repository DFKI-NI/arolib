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

#include "arolib/misc/filesystem_helper.h"
#include <cstdarg>
#include <dirent.h>


namespace arolib{
namespace io {

std::string correct_path_separators(const std::string &path)
{
    return boost::filesystem::path(path).make_preferred().string();
}


std::string create_path(const std::string & element){
    return correct_path_separators(element);
}

std::string create_path_2(bool startWithSep, bool endWithSep, const std::string & element)
{
//    boost::filesystem::path path( startWithSep ? "" : element );
//    if(startWithSep)
//        path /= element;
//    if(endWithSep)
//        path /= "";
//    return path.make_preferred().string();

    auto path = create_path(element);
    if(startWithSep)
        prepend_separator_to_path(path);
    if(endWithSep)
        append_separator_to_path(path);
    return path;
}

bool create_directory(std::string dir, bool clearIfExistent)
{
    if(dir.empty())
        return false;

    const auto sep = get_path_separator();
    dir = correct_path_separators(dir);
    while (dir.back() == sep){
        dir.pop_back();
        if(dir.empty())
            return false;
    }

    if (boost::filesystem::exists(dir.c_str())){
        if(!boost::filesystem::is_directory(dir.c_str()))
            return false;
        if(!clearIfExistent)
            return true;
        if(!boost::filesystem::remove_all(dir.c_str()))
            return false;
    }
    return boost::filesystem::create_directories(dir.c_str());
}

bool file_or_dir_exists(const std::string & path)
{
    return boost::filesystem::exists(path);
}

bool dir_exists(const std::string &path)
{
    return boost::filesystem::is_directory(path);
}

std::vector<std::string> get_filenames_recursive(std::string basedir, std::string endPattern, bool includeBaseDir, bool includeSubDir)
{
    std::vector<std::string> filelist;
    append_separator_to_path(basedir);

    struct dirent *pDirent;
    DIR *pDir;
    pDir = opendir(basedir.c_str());
    if (pDir == NULL)
        return filelist;

    includeSubDir |= includeBaseDir;

    bool addSubdirs = !includeBaseDir && includeSubDir;
    const auto sep = get_path_separator();

    while ((pDirent = readdir(pDir)) != NULL) {
        std::string filename = pDirent->d_name;
        if (pDirent->d_type == DT_DIR) {
            if (filename == "." || filename == "..")
                continue;
            auto filelist2 = get_filenames_recursive(basedir + filename, endPattern, includeBaseDir, includeSubDir);
            if(addSubdirs){
                for(auto &f : filelist2)
                    f = filename + sep + f;
            }
            filelist.insert(filelist.end(), filelist2.begin(), filelist2.end());
        }
        else {
            if(includeBaseDir)
                filename = basedir + filename;
            if(endPattern.empty())
                filelist.emplace_back(filename);
            else{
                if(filename.rfind(endPattern) == filename.size() - endPattern.size())
                    filelist.push_back(filename);
            }
        }
    }
    closedir (pDir);
    std::sort(filelist.begin(), filelist.end());
    return filelist;
}

std::vector<std::string> get_directories(std::string basedir, bool includeBaseDir)
{
    std::vector<std::string> dirlist;
    append_separator_to_path(basedir);

    struct dirent *pDirent;
    DIR *pDir;
    pDir = opendir(basedir.c_str());
    if (pDir == NULL)
        return dirlist;

    while ((pDirent = readdir(pDir)) != NULL) {
        if (pDirent->d_type != DT_DIR)
            continue;
        std::string name = pDirent->d_name;
        if (name == "." || name == "..")
            continue;
        dirlist.push_back( (includeBaseDir ? basedir : "") + name);
    }
    closedir (pDir);
    std::sort(dirlist.begin(), dirlist.end());
    return dirlist;

}

std::vector<std::string> get_directories_recursive(std::string basedir, bool includeIntermediateDirs, bool includeBaseDir)
{
    std::vector<std::string> dirlist;
    append_separator_to_path(basedir);

    struct dirent *pDirent;
    DIR *pDir;
    pDir = opendir(basedir.c_str());
    if (pDir == NULL)
        return dirlist;

    while ((pDirent = readdir(pDir)) != NULL) {
        if (pDirent->d_type != DT_DIR)
            continue;
        std::string name = pDirent->d_name;
        if (name == "." || name == "..")
            continue;
        auto subdirs = get_directories_recursive(basedir+name, includeIntermediateDirs, includeBaseDir);
        if(includeIntermediateDirs || subdirs.empty()){
            dirlist.push_back( (includeBaseDir ? basedir : "") + name);
            if(subdirs.empty())
                continue;
        }
        if(includeBaseDir){
            dirlist.insert(dirlist.end(), subdirs.begin(), subdirs.end());
            continue;
        }

        dirlist.reserve( dirlist.size() + subdirs.size() );
        for(auto& sd : subdirs)
            dirlist.emplace_back( create_path(name, sd) );
    }
    closedir (pDir);
    std::sort(dirlist.begin(), dirlist.end());
    return dirlist;

}

void append_separator_to_path(std::string &path)
{
//    path =  std::move( ( boost::filesystem::path(path) / "" ).make_preferred().string() );
    if(path.empty() || path.back() != get_path_separator())
        path.push_back( get_path_separator() );
}

void prepend_separator_to_path(std::string &path)
{
//    path = std::move( ( boost::filesystem::path("") / path ).make_preferred().string() );
    if(path.empty() || path.front() != get_path_separator())
        path.insert( path.begin(), get_path_separator() );
}

void remove_dir_from_filename(std::string &filename, std::string dir)
{
    if(dir.empty())
        return;

    append_separator_to_path(dir);

    if(filename.size() < dir.size())
        return;

    if(filename.size() == dir.size()){
        if(filename == dir)
            filename.clear();
        return;
    }

    if(filename.find(dir) == 0){
        filename = filename.substr(dir.size());
    }
}

std::string get_filename(const std::string &filepath, bool removePotentialExtension)
{
    std::string filename = filepath;
    if(filepath.empty())
        return filename;
    auto index = filepath.find_last_of( get_path_separator() );
    if (index != std::string::npos)
        filename = filepath.substr( index+1 );

    if(removePotentialExtension){
        index = filename.find_last_of('.');
        if (index != std::string::npos && index != 0)
            filename = filename.substr( 0, index);
    }

    return filename;
}

bool file_has_extension(std::string filename, std::string ext, bool caseSensitive)
{
    if(ext.empty()){//no extension
        auto tmp = get_filename(filename, false);
        return tmp.find(".") == std::string::npos;
    }
    if(ext.front() != '.')
        ext = "." + ext;

    if(filename.size() <= ext.size())
        return false;

    if(!caseSensitive){
        std::transform( filename.begin(), filename.end(), filename.begin(),
                        [](unsigned char c){ return std::tolower(c); } );
        std::transform( ext.begin(), ext.end(), ext.begin(),
                        [](unsigned char c){ return std::tolower(c); } );
    }

    return filename.rfind(ext) == filename.size() - ext.size();
}


std::string get_path_to_file(const std::string& filename)
{
    if(filename.empty())
        return filename;
    auto index = filename.find_last_of( get_path_separator() );
    if (index == std::string::npos)
        return filename;

    return filename.substr( 0, index+1 );
}

}
}
