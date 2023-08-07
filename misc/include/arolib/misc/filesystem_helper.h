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
 
#ifndef AROLIB_FILESYSTEM_HELPER_H
#define AROLIB_FILESYSTEM_HELPER_H

#include <boost/filesystem.hpp>
#include <dirent.h>

namespace arolib {
namespace io {

/**
 * @brief Create a directory
 * @param dir Directory
 * @param clearIfExistent If dir exist it removes it contents
 * @return True on success
 */
bool create_directory(std::string dir, bool clearIfExistent = false);

/**
 * @brief Check if a file or directory exists
 * @param filename filename
 * @return True if file or directoryexists
 */
bool file_or_dir_exists(std::string path);

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
 * @brief Get the filename from a path
 * @param filepath File path
 * @param removePotentialExtension If true, it will remove potential file extensions
 * @return Filename
 */
std::string get_filename(const std::string &filepath, bool removePotentialExtension);

/**
 * @brief Appends a final slash to a directory (if needed)
 * @param dir Directory path
 */
void append_slash_to_dir(std::string &dir);

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
bool fileHasExtension(std::string filename, std::string ext, bool caseSensitive = false);

/**
 * @brief Get the directory path of a filename
 * @param filename Filename
 * @return Directory path
 */
std::string getPathToFile(const std::string &filename);

}
}

#endif // AROLIB_FILESYSTEM_HELPER_H
