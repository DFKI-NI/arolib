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
 
#ifndef AROLIB_IO_ARODOCUMENT_HPP
#define AROLIB_IO_ARODOCUMENT_HPP

#include <ostream>
#include <fstream>
#include "arolib/misc/loggingcomponent.h"
#include "arolib/types/point.hpp"

namespace arolib {

/**
 * I/O namespace.
 */
namespace io {

/**
 * @brief Base Arolib document
 */
class AroDocument : public LoggingComponent{
public:
    /**
     * @brief Constructor.
     *
     * @param logLevel Log level
     */
    explicit AroDocument(LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Destructor.
     */
    virtual ~AroDocument();

    /**
     * @brief Open file.
     *
     * @param filename Filename
     * @return True on success
     */
    virtual bool openFile(const std::string& filename) = 0;

    /**
     * @brief Close file.
     *
     * @return True on success
     */
    virtual bool closeFile() = 0;

    /**
     * @brief Open document.
     *
     * @return True on success
     */
    virtual bool openDocument();

    /**
     * @brief Close document.
     *
     * @return True on success
     */
    virtual bool closeDocument();

protected:

    /**
     * @brief Open document (implemented by derived classes).
     *
     * @return True on success
     */
    virtual bool openDoc() = 0;

    /**
     * @brief Close document (implemented by derived classes).
     *
     * @return True on success
     */
    virtual bool closeDoc() = 0;

protected:
    bool m_isOpen = false; /**< Is the file open? */
    bool m_isDocOpen = false; /**< Is the document open? */

    static const std::string m_docTag; /**< (default) Tag for the base "Document" level */
    static const std::string m_coordTypeTag; /**< (default) Tag for coordinates */
};


/**
 * @brief Base Arolib output document
 */
class AroOutDocument : public AroDocument{

public:
    /**
     * @brief Constructor.
     *
     * @param logLevel Log level
     */
    explicit AroOutDocument(LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Destructor.
     */
    virtual ~AroOutDocument();


    /**
     * @brief Open file.
     *
     * @param filename Filename
     * @return True on success
     */
    virtual bool openFile(const std::string& filename);

    /**
     * @brief Close file.
     *
     * @return True on success
     */
    virtual bool closeFile();

    /**
     * @brief Set the output stream.
     *
     * @return True on success
     */
    virtual bool setOutputStream(std::ostream* os);

    /**
     * @brief Unset/reset the output stream.
     *
     * @return True on success
     */
    virtual bool unsetOutputStream();

    /**
     * @brief Get the output stream.
     *
     * @return Output stream
     */
    const std::ostream *outputStream() const;

    /**
      * @brief Set the coordinates (projection) type for the points in the document
      * @param in Cooridinate (projection) type of the source
      * @param out Cooridinate (projection) type of the target
      * @return True on success
      */
    virtual bool setCoordinatesTypes(Point::ProjectionType in, Point::ProjectionType out);

protected:
    /**
     * @brief Check if the document is ready to write
     *
     * @return True if ready
     */
    virtual bool isReadyToWrite() const;

    /**
     * @brief Open document (Default AroDocument derived class implementation).
     *
     * @return True on success
     */
    virtual bool openDoc(){return isReadyToWrite();}

    /**
     * @brief Close document (Default AroDocument derived class implementation).
     *
     * @return True on success
     */
    virtual bool closeDoc(){return isReadyToWrite();}

protected:
    std::ostream* m_os = nullptr; /**< Output stream */
    Point::ProjectionType m_coordinatesType_in = Point::UTM; /**< Input coordinates projection type */
    Point::ProjectionType m_coordinatesType_out = Point::WGS; /**< Input coordinates projection type */
};


/**
 * @brief Base Arolib input document
 */
class AroInDocument : public AroDocument{

public:
    /**
     * @brief Constructor.
     *
     * @param logLevel Log level
     */
    explicit AroInDocument(LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Destructor.
     */
    virtual ~AroInDocument();

    /**
     * @brief Open file.
     *
     * @param filename Filename
     * @return True on success
     */
    virtual bool openFile(const std::string& filename);

    /**
     * @brief Close file.
     *
     * @return True on success
     */
    virtual bool closeFile();

    /**
     * @brief Set the input stream.
     *
     * @return True on success
     */
    virtual bool setInputStream(std::istream* is);

    /**
     * @brief Unset/reset the input stream.
     *
     * @return True on success
     */
    virtual bool unsetInputStream();

    /**
     * @brief Get the input stream.
     *
     * @return Input stream
     */
    const std::istream *inputStream() const;

    /**
      * @brief Set the coordinates (projection) type for the resulting read points
      * @param out Cooridinate (projection) type of the target
      */
    virtual bool setCoordinatesType(Point::ProjectionType out);

protected:
    /**
     * @brief Check if the document is ready to read
     *
     * @return True if ready
     */
    virtual bool isReadyToRead() const;

    /**
     * @brief Open document (Default AroDocument derived class implementation).
     *
     * @return True on success
     */
    virtual bool openDoc(){return isReadyToRead();}

    /**
     * @brief Close document (Default AroDocument derived class implementation).
     *
     * @return True on success
     */
    virtual bool closeDoc(){return isReadyToRead();}

protected:
    std::istream* m_is = nullptr; /**< Input stream */
    Point::ProjectionType m_coordinatesType_in = Point::WGS; /**< Input coordinates projection type */
    Point::ProjectionType m_coordinatesType_out = Point::UTM; /**< Input coordinates projection type */
};

}
}//end namespace arolib


#endif //AROLIB_IO_ARODOCUMENT_HPP

