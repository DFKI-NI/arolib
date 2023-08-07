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
 
#include "arolib/misc/filesystem_helper.h"

namespace arolib{
namespace io {

bool create_directory(std::string dir, bool clearIfExistent)
{
    if(dir.empty())
        return false;

    while (dir.back() == '/'){
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

bool file_or_dir_exists(std::string path)
{
    return boost::filesystem::exists(path);
}

std::vector<std::string> get_filenames_recursive(std::string basedir, std::string endPattern, bool includeBaseDir, bool includeSubDir)
{
    std::vector<std::string> filelist;
    append_slash_to_dir(basedir);

    struct dirent *pDirent;
    DIR *pDir;
    pDir = opendir(basedir.c_str());
    if (pDir == NULL)
        return filelist;

    includeSubDir |= includeBaseDir;

    bool addSubdirs = !includeBaseDir && includeSubDir;

    while ((pDirent = readdir(pDir)) != NULL) {
        std::string filename = pDirent->d_name;
        if (pDirent->d_type == DT_DIR) {
            if (filename == "." || filename == "..")
                continue;
            auto filelist2 = get_filenames_recursive(basedir + filename, endPattern, includeBaseDir, includeSubDir);
            if(addSubdirs){
                for(auto &f : filelist2)
                    f = filename + "/" + f;
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

void remove_dir_from_filename(std::string &filename, std::string dir)
{
    if(dir.empty())
        return;

    append_slash_to_dir(dir);

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

void append_slash_to_dir(std::string &dir)
{
    if(!dir.empty() && dir.back() != '/')
        dir.push_back('/');
}

std::string get_filename(const std::string &filepath, bool removePotentialExtension)
{
    std::string filename = filepath;
    if(filepath.empty())
        return filename;
    auto index = filepath.find_last_of('/');
    if (index != std::string::npos)
        filename = filepath.substr( index+1 );

    if(removePotentialExtension){
        index = filename.find_last_of('.');
        if (index != std::string::npos && index != 0)
            filename = filename.substr( 0, index);
    }

    return filename;
}

bool fileHasExtension(std::string filename, std::string ext, bool caseSensitive)
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


std::string getPathToFile(const std::string& filename)
{
    if(filename.empty())
        return filename;
    auto index = filename.find_last_of('/');
    if (index == std::string::npos)
        return filename;

    return filename.substr( 0, index+1 );
}

}
}
