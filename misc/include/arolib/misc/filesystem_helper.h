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
 
#ifndef AROLIB_FILESYSTEM_HELPER_H
#define AROLIB_FILESYSTEM_HELPER_H

#include <string>
#include <vector>
#include <boost/filesystem.hpp>

namespace arolib {
namespace io {

/**
 * @brief Get the preferred OS path separator
 * @return OS path separator
 */
constexpr char get_path_separator(){ return boost::filesystem::path::preferred_separator; }

/**
 * @brief Appends a final separator to a directory (if needed)
 * @param dir Directory path
 */
void append_separator_to_path(std::string &path);

/**
 * @brief Appends an initial separator to a directory (if needed)
 * @param dir Directory path
 */
void prepend_separator_to_path(std::string &path);

/**
 * @brief Corrects the path separators
 * @param path Path
 * @return Corrected path
 */
std::string correct_path_separators(const std::string& path);

/**
 * @brief Create a boost path
 * @param element element
 * @return Path
 */
inline boost::filesystem::path create_boost_path(const std::string & element)
{
    return boost::filesystem::path(element);
}

/**
 * @brief Create a boost path
 * @param element first element
 * @param elements other elements
 * @return Path
 */
template<typename... Ts >
boost::filesystem::path create_boost_path(const std::string & element, const Ts& ... elements)
{
    boost::filesystem::path path(element);
    path /= create_boost_path(elements...);
    return path;
}

/**
 * @brief Create a path
 * @param element first element
 * @return Path
 */
std::string create_path(const std::string& element);

/**
 * @brief Create a path
 * @param element first element
 * @param elements other elements
 * @return Path
 */
template<typename... Ts >
std::string create_path(const std::string & element, const Ts& ... elements)
{
    boost::filesystem::path path(element);
    path /= create_boost_path(elements...);
    return path.make_preferred().string();

//    return ( !element.empty() && element.back() == get_path_separator() ) ?
//                element + create_path(elements...) :
//                ( boost::filesystem::path(element) /= create_path(elements...) ).make_preferred().string();
}

/**
 * @brief Create a path
 * @param startWithSep Add an initial separator
 * @param endWithSep Add an final separator
 * @param element first element
 * @return Path
 */
std::string create_path_2(bool startWithSep, bool endWithSep, const std::string& element);

/**
 * @brief Create a path
 * @param startWithSep Add an initial separator
 * @param endWithSep Add an final separator
 * @param element first element
 * @param elements other elements
 * @return Path
 */
template<typename... Ts >
std::string create_path_2(bool startWithSep, bool endWithSep, const std::string & element, const Ts& ... elements)
{
//    boost::filesystem::path path( startWithSep ? "" : create_boost_path(element, elements...) );
//    if(startWithSep)
//        path /= create_boost_path(element, elements...);
//    if(endWithSep)
//        path /= "";
//    return path.make_preferred().string();

    auto path = create_path(element, elements...);
    if(startWithSep)
        prepend_separator_to_path(path);
    if(endWithSep)
        append_separator_to_path(path);
    return path;
}

/**
 * @brief Create a directory
 * @param dir Directory
 * @param clearIfExistent If dir exist it removes it contents
 * @return True on success
 */
bool create_directory(std::string dir, bool clearIfExistent = false);

/**
 * @brief Check if a file or directory exists
 * @param path File / directory path
 * @return True if the file or directory exists
 */
bool file_or_dir_exists(const std::string &path);

/**
 * @brief Check if a directory exists
 * @param path Directory path
 * @return True if the directory exists
 */
bool dir_exists(const std::string &path);

/**
 * @brief Get a list of filenames for files in a directory (with optional end-pattern filter)
 * @param basedir Base directory
 * @param endPattern End-pattern to filter (disregarded if empty)
 * @param includeBaseDir If true, the path of the base directory will be included in the filenames
 * @param includeSubDir If true, the path of the sub-directories will be included in the filenames
 * @return List of filenames
 */
std::vector<std::string> get_filenames_recursive(std::string basedir, std::string endPattern = "", bool includeBaseDir = true, bool includeSubDir = true);

/**
 * @brief Get a list of (sub) directories in a directory
 * @param basedir Base directory
 * @param includeBaseDir If true, the path of the base directory will be included in the output
 * @return List of (sub) directories
 */
std::vector<std::string> get_directories(std::string basedir, bool includeBaseDir = true);

/**
 * @brief Get a list of all subdirectories directories in a directory
 * @param basedir Base directory
 * @param includeIntermediateDirs If true, paths to (sub) directories which have subdirectories will also be included
 * @param includeBaseDir If true, the path of the base directory will be included in the output
 * @return List of (sub) directories
 */
std::vector<std::string> get_directories_recursive(std::string basedir, bool includeIntermediateDirs, bool includeBaseDir = true);

/**
 * @brief Get the filename from a path
 * @param filepath File path
 * @param removePotentialExtension If true, it will remove potential file extensions
 * @return Filename
 */
std::string get_filename(const std::string &filepath, bool removePotentialExtension);

/**
 * @brief Removes a given (base) directory/directories from a filename
 * @param [in/out] filename Filename
 * @param dir Base directory path to be removed
 */
void remove_dir_from_filename(std::string &filename, std::string dir);

/**
 * @brief Check if a filename has a given file extension
 * @param filename Filename
 * @param ext File extension
 * @param caseSensitive Case-sensitive
 * @return True if the filename has the given file extension
 */
bool file_has_extension(std::string filename, std::string ext, bool caseSensitive = false);

/**
 * @brief Get the directory path of a filename
 * @param filename Filename
 * @return Directory path
 */
std::string get_path_to_file(const std::string &filename);

/**
 * @brief Get the temp directory path
 * @return Temp directory path
 */
inline std::string get_temp_dir() { return boost::filesystem::temp_directory_path().string(); }

}
}

#endif // AROLIB_FILESYSTEM_HELPER_H
