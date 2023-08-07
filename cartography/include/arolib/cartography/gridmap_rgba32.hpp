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
 
#ifndef AROLIB_CARTOGRAPHY_GRIDMAP_RGBA32_HPP
#define AROLIB_CARTOGRAPHY_GRIDMAP_RGBA32_HPP

#include "gridmap.hpp"

namespace arolib{
namespace gridmap{

/**
 * @brief Holds the cell data for RGBA 32bit-pixels
 */
struct RGBA32CellData{
    unsigned char r = 0; /**< Red value. */
    unsigned char g = 0; /**< Green value. */
    unsigned char b = 0; /**< Blue value. */
    unsigned char a = 0; /**< Alpha (opacity) value. */

    /**
     * @brief Default constructor
     */
    RGBA32CellData() = default;

    /**
     * @brief Constructor
     * @param _r Red value
     * @param _g Green value
     * @param _b Blue value
     * @param _a Alpha (opacity) value
     */
    RGBA32CellData(unsigned char _r, unsigned char _g, unsigned char _b, unsigned char _a = 0xFF);

    /**
     * @brief Constructor parsing from unsigned int 32bits
     * @param val Value in uint32 format
     */
    RGBA32CellData(const uint32_t& val);

    /**
     * @brief Constructor from png::rgba_pixel
     * @param pixel png::rgba_pixel
     */
    RGBA32CellData(const png::rgba_pixel& pixel);

    /**
     * @brief operator== (check if the cell data / pixels are equal)
     * @param other Other cell
     * @return True if equal
     */
    bool operator==(const RGBA32CellData& other) const;

    /**
     * @brief Parse from unsigned int 32bits
     * @param val Value in uint32 format
     */
    void fromUInt32(const uint32_t& val);

    /**
     * @brief Parse to unsigned int 32bits
     * @return Value in uint32 format
     */
    uint32_t toUInt32() const;

    /**Sets the rgba value from its normalized values [0, 1]
     * @param _r Red normalized value [0, 1]
     * @param _g Green normalized value [0, 1]
     * @param _b Bluenormalized  value [0, 1]
     * @param _a Alpha (opacity) normalized value [0, 1]
     */
    void fromNorm(float _r, float _g, float _b, float _a = 1);


    /**Get the rgba normalized values [0, 1]
     * @param _r [out] Red normalized value [0, 1]
     * @param _g [out] Green normalized value [0, 1]
     * @param _b [out] Bluenormalized  value [0, 1]
     * @param _a [out] Alpha (opacity) normalized value [0, 1]
     */
    void toNorm(float &_r, float &_g, float &_b, float &_a);

};

/**
 * @brief Gridmap with RGBA 32bit-pixel cell values
 */
class RGBA32Gridmap : public Gridmap<RGBA32CellData>{
public:

    /**
    * Constructor.
    *
    * Already allocates the grid with the given size if the layout is valid.
    * @param lo Gridmap geometric layout.
    * @param logLevel Log level
    */
   explicit RGBA32Gridmap(const GridmapLayout &lo, LogLevel logLevel = LogLevel::INFO);

   /**
    * Constructor.
    * The grid is not allocated yet (it doesn't exist).
    * @param logLevel Log level
    */
   explicit RGBA32Gridmap(LogLevel logLevel = LogLevel::INFO);

   /**
    * Copy constructor.
     * @param other Other gridmap.
    */
   RGBA32Gridmap(const RGBA32Gridmap& other);

   /**
    * Destructor.
    */
   virtual ~RGBA32Gridmap();


   /**
   * Copy assignment
   * @param other Other grid.
   */
   virtual RGBA32Gridmap& operator=(const RGBA32Gridmap& other);

   /**
    * Saves the grid as a PNG file (plus meta-file).
    * @param filename Name of the output file.
    * @return True on success
    */
   bool saveGridAsPNG32(const std::string& _filename) const;


   /**
   * Gets the grid as a PNG ostringstream.
   * @param image_out ostringstream containing the png data.
   * @return True on success
   */
   bool saveGridAsPNG32(std::ostringstream &image_out) const;

   /**
    * Gets the grid as a PNG ostream.
    * @param image_out output stream containing the png data.
    * @return True on success
   */
   bool saveGridAsPNG32(std::ostream &image_out) const;


   /**
    * Gets the grid values in CSV format.
    * @param image_out output stream containing the png data.
    * @param sep Separator between values
    * @param rgbaSep Separator between RGBA values
    * @return True on success
   */
   bool saveValuesCsv(std::ostream &out, const std::string& sep = ";", const std::string& rgbaSep = ".") const;

   /**
    * Loads the grid from a PNG file.
    * @param filename Name of the file to be read.
    * @return True on success
    */
   bool readGridFromPNG32(const std::string& filename);

   /**
   * Sets the grid as from a PNG input stream.
   * @param image_in stream containing the png data.
   * @param cellsize Cellsize (resolution).
   * @param minX X-location of the lower-left corner.
   * @param minY Y-location of the lower-left corner.
   * @return True on success
   */
   bool readGridFromPNG32(std::istream& image_in,
                          double cellsize,
                          double minX,
                          double minY);

   /**
   * Sets the grid as from a PNG input stream.
   * @param image_in stream containing the png data.
   * @param minX X-location of the lower-left corner.
   * @param minY Y-location of the lower-left corner.
   * @param maxX X-location of the upper-right corner.
   * @param maxY Y-location of the upper-right corner.
   * @return True on success
   */
   bool readGridFromPNG32(std::istream& image_in,
                          double minX,
                          double minY,
                          double maxX,
                          double maxY);

   /**
   * Sets the grid from a PNG string.
   * @param image_in c_string containing the png data.
   * @param cellsize Cellsize (resolution).
   * @param minX X-location of the lower-left corner.
   * @param minY Y-location of the lower-left corner.
   * @return True on success
   */
   bool readGridFromPNG32(const std::string& image_in,
                          double cellsize,
                          double minX,
                          double minY);

   /**
   * Sets the grid from a PNG string.
   * @param image_in c_string containing the png data.
   * @param minX X-location of the lower-left corner.
   * @param minY Y-location of the lower-left corner.
   * @param maxX X-location of the upper-right corner.
   * @param maxY Y-location of the upper-right corner.
   * @return True on success
   */
   bool readGridFromPNG32(const std::string& image_in,
                          double minX,
                          double minY,
                          double maxX,
                          double maxY);

   /**
   * Gets the grid as a PNG ostringstream.
   * @param image_in c_string containing the png data.
   * @param size size of the image_in c_string containing the png data.
   * @param cellsize Cellsize (resolution).
   * @param minX X-location of the lower-left corner.
   * @param minY Y-location of the lower-left corner.
   * @return True on success
   */
   bool readGridFromPNG32(const char* image_in,
                          unsigned int size,
                          double cellsize,
                          double minX,
                          double minY);

   /**
   * Gets the grid as a PNG ostringstream.
   * @param image_in c_string containing the png data.
   * @param size size of the image_in c_string containing the png data.
   * @param minX X-location of the lower-left corner.
   * @param minY Y-location of the lower-left corner.
   * @param maxX X-location of the upper-right corner.
   * @param maxY Y-location of the upper-right corner.
   * @return True on success
   */
   bool readGridFromPNG32(const char* image_in,
                          unsigned int size,
                          double minX,
                          double minY,
                          double maxX,
                          double maxY);

private:
};

}
}

#endif // AROLIB_CARTOGRAPHY_GRIDMAP_RGBA32_HPP
