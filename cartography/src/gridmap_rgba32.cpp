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
 
#include "arolib/cartography/gridmap_rgba32.hpp"

namespace arolib{
namespace gridmap{


RGBA32CellData::RGBA32CellData(unsigned char _r, unsigned char _g, unsigned char _b, unsigned char _a)
    : r(_r), g(_g), b(_b), a(_a)
{

}

RGBA32CellData::RGBA32CellData(const uint32_t &val)
{
    fromUInt32(val);
}

RGBA32CellData::RGBA32CellData(const png::rgba_pixel &pixel)
    : RGBA32CellData(pixel.red, pixel.green, pixel.blue, pixel.alpha)
{

}

bool RGBA32CellData::operator==(const RGBA32CellData &other) const
{
    return  r == other.r &&
            g == other.g &&
            b == other.b &&
            a == other.a;
}

void RGBA32CellData::fromUInt32(const uint32_t &val)
{
    r = ( (val & 0xFF000000) >> 24 );
    g = ( (val & 0x00FF0000) >> 16 );
    b = ( (val & 0x0000FF00) >> 8 );
    a = (val & 0x000000FF);
}

uint32_t RGBA32CellData::toUInt32() const
{
    uint32_t _r = r;
    uint32_t _g = g;
    uint32_t _b = b;
    uint32_t _a = a;
    uint32_t ret = ( _r << 24 )
            | ( _g << 16 )
            | ( _b << 8 )
            | ( _a );
    return    ( _r << 24 )
            | ( _g << 16 )
            | ( _b << 8 )
            | ( _a );

}

void RGBA32CellData::fromNorm(float _r, float _g, float _b, float _a)
{
    r = std::min(1.0f,  std::max(0.0f, 255 * _r) );
    g = std::min(1.0f,  std::max(0.0f, 255 * _g) );
    b = std::min(1.0f,  std::max(0.0f, 255 * _b) );
    r = std::min(1.0f,  std::max(0.0f, 255 * _a) );

}

void RGBA32CellData::toNorm(float &_r, float &_g, float &_b, float &_a)
{
    _r = (float)255 / r;
    _g = (float)255 / g;
    _b = (float)255 / b;
    _a = (float)255 / a;
}

RGBA32Gridmap::RGBA32Gridmap(const GridmapLayout &lo, LogLevel logLevel)
    : Gridmap<RGBA32CellData>(lo, logLevel)
{
    logger().setBaseName(__FUNCTION__);
}

RGBA32Gridmap::RGBA32Gridmap(LogLevel logLevel)
    : RGBA32Gridmap( GridmapLayout(), logLevel)
{

}

RGBA32Gridmap::RGBA32Gridmap(const RGBA32Gridmap &other)
    : Gridmap<RGBA32CellData>(other)
{
    logger().setBaseName(__FUNCTION__);
}

RGBA32Gridmap::~RGBA32Gridmap()
{

}

RGBA32Gridmap &RGBA32Gridmap::operator=(const RGBA32Gridmap &other)
{
    Gridmap<RGBA32CellData>::operator =(other);
    return *this;
}


bool RGBA32Gridmap::saveGridAsPNG32(const std::string &_filename) const  {
    if ( !checkInternalParameters(true,__FUNCTION__) )
        return false;

    std::string filename = _filename;
    std::string metadata = _filename;
    size_t index = filename.find(".png");
    if (index != std::string::npos) {
        metadata.replace(index, 4, ".meta");
    }
    else{
        filename += ".png";
        metadata += ".meta";
    }
    std::ofstream outMeta(metadata.c_str());
    if(!outMeta.is_open()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating/opening file for metadata.");
        return false;
    }
    outMeta << std::setprecision(12) << m_layout.getCellsize()
            << " " << m_layout.getMinPointX() << " " << m_layout.getMinPointY() << std::endl;
    outMeta.close();

    std::ofstream image_out(filename);
    if(!image_out.is_open()){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating/opening image file.");
        return false;
    }

    bool ok = saveGridAsPNG32(image_out);
    image_out.close();

    if(ok)
        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Grid saved successfully.");

    return ok;
}

bool RGBA32Gridmap::saveGridAsPNG32(std::ostringstream &image_out) const  {
    try{
        image_out.str(std::string());
        std::ostream& image_out_ref = image_out;
        return saveGridAsPNG32(image_out_ref) && image_out.good();
    }
    catch (std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error converting grid to png: " + std::string( e.what() ) );
        return false;
    }
}

bool RGBA32Gridmap::saveGridAsPNG32(std::ostream &image_out) const{
    try{
        if ( !checkInternalParameters(true,"getGridAsPNG") )
            return false;

        png::image< png::rgba_pixel > image(m_layout.getSizeX(), m_layout.getSizeY());

        for (unsigned int x=0; x < m_layout.getSizeX(); x++) {
            for (unsigned int y=0; y < m_layout.getSizeY(); y++) {
                RGBA32CellData rgba;
                png::rgba_pixel pixel;
                rgba.a = 0;
                if( isSet(x, y) )
                    rgba = get(x, y);

                pixel.red = rgba.r;
                pixel.green = rgba.g;
                pixel.blue = rgba.b;
                pixel.alpha = rgba.a;
                image[m_layout.getSizeY()-y-1][x] = pixel;
            }
        }

        image.write_stream(image_out);

        return true;
    }
    catch (std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error converting grid to png: " + std::string( e.what() ) );
        return false;
    }
}

bool RGBA32Gridmap::saveValuesCsv(std::ostream &out, const std::string &sep, const std::string &rgbaSep) const
{
    if ( !checkInternalParameters(true,"getGridAsPNG") )
        return false;

    for (unsigned int y=0; y < m_layout.getSizeY(); y++){
        for (unsigned int x=0; x < m_layout.getSizeX(); x++) {
            RGBA32CellData rgba;
            rgba.a = 0;
            if( isSet(x, y) ){
                rgba = get(x, y);
                out << rgba.r << rgbaSep << rgba.g << rgbaSep << rgba.b << rgbaSep << rgba.a;
            }
            out << sep;
        }
        out << std::endl;
    }
    return true;
}

bool RGBA32Gridmap::readGridFromPNG32(const std::string &filename) {

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Reading grid from " + filename);
    std::string metadata = filename;
    size_t index = metadata.find(".png");
    if (index != std::string::npos) {
        metadata.replace(index, 4, ".meta");
    }
    std::ifstream inMeta(metadata.c_str());
    if (!inMeta.is_open()) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Cannot read from file " + metadata);
        return false;
    }

    logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Meta information read from " + metadata);
    double cellsize,dataMinX,dataMinY;
    inMeta >> cellsize >> dataMinX >> dataMinY;
    inMeta.close();

    if (cellsize <= 0 ) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error in cellsize input parameter");
        return false;
    }

    std::ifstream image_in(filename.c_str());
    if (!image_in.is_open()) {
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Cannot read from file " + filename);
        return false;
    }

    return readGridFromPNG32(image_in,
                             cellsize,
                             dataMinX,
                             dataMinY);
}

bool RGBA32Gridmap::readGridFromPNG32(std::istream &image_in, double cellsize, double minX, double minY) {
    try{
        destroy();

        png::image< png::rgba_pixel > image;
        image.read_stream( image_in );

        image_in.seekg(0, image_in.beg);

        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Generating grid with size: " + std::to_string(image.get_width())
                          + " x " + std::to_string(image.get_height()));

        int width = image.get_width();
        int height = image.get_height();
        if(width <= 0 || height <= 0){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid image size");
            return false;
        };

        if( !m_layout.init(minX, minY, width, height, cellsize) ){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error setting geometric layout");
            return false;
        }

        if ( !allocate() ){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error allocating new grid");
            return false;
        };

        m_units = UNIT_CUSTOM;

        for (unsigned int x=0; x < m_layout.getSizeX(); x++) {
            for (unsigned int y=0; y < m_layout.getSizeY(); y++) {
                png::rgba_pixel pixelVal = image[m_layout.getSizeY()-y-1][x];
                if(pixelVal.alpha != 0)
                    set(x, y, RGBA32CellData(pixelVal));
            }
        }

        logger().printOut(LogLevel::DEBUG, __FUNCTION__, "Grid generated successfully.");

        return true;
    }
    catch (std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating grid from png: " + std::string( e.what() ) );
        return false;
    }
}

bool RGBA32Gridmap::readGridFromPNG32(std::istream &image_in, double minX, double minY, double maxX, double maxY) {
    try{
        png::image< png::rgba_pixel > image;
        image.read_stream( image_in );
        image_in.seekg(0, image_in.beg);

        int width = image.get_width();
        int height = image.get_height();
        if(width <= 0 || height <= 0){
            logger().printOut(LogLevel::ERROR, __FUNCTION__, "Invalid image size");
            return false;
        };

        double cellsize = 0.5 * ( (maxX - minX) / width + (maxY - minY) / height );

        return readGridFromPNG32(image_in, cellsize, minX, minY);
    }
    catch (std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating grid from png: " + std::string( e.what() ) );
        return false;
    }
}

bool RGBA32Gridmap::readGridFromPNG32(const std::string &image_in, double cellsize, double minX, double minY) {
    try{
        if ( image_in.empty() )
            return false;

        std::istringstream stream;
        stream.str( image_in );
        return readGridFromPNG32(stream,
                                 cellsize,
                                 minX,
                                 minY);
    }
    catch (std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating grid from png: " + std::string( e.what() ) );
        return false;
    }
}

bool RGBA32Gridmap::readGridFromPNG32(const std::string &image_in, double minX, double minY, double maxX, double maxY) {
    try{
        if ( image_in.empty() )
            return false;

        std::istringstream stream;
        stream.str( image_in );
        return readGridFromPNG32(stream,
                                 minX,
                                 minY,
                                 maxX,
                                 maxY);
    }
    catch (std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating grid from png: " + std::string( e.what() ) );
        return false;
    }
}

bool RGBA32Gridmap::readGridFromPNG32(const char *image_in, unsigned int size, double cellsize, double minX, double minY) {

    try{
        if ( !image_in || size == 0 )
            return false;

        //we could do the same as the setGridfromPNG (string,...) to avoid creating a copy with all the data, but for some reason the istringstream.read(image_in,size) is not working

        return readGridFromPNG32( std::string(image_in, size),
                                  cellsize,
                                  minX,
                                  minY );
    }
    catch (std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating grid from png: " + std::string( e.what() ) );
        return false;
    }
}

bool RGBA32Gridmap::readGridFromPNG32(const char *image_in, unsigned int size, double minX, double minY, double maxX, double maxY) {

    try{
        if ( !image_in || size == 0 )
            return false;

        //we could do the same as the setGridfromPNG (string,...) to avoid creating a copy with all the data, but for some reason the istringstream.read(image_in,size) is not working

        return readGridFromPNG32( std::string(image_in, size),
                                  minX,
                                  minY,
                                  maxX,
                                  maxY );
    }
    catch (std::exception &e){
        logger().printOut(LogLevel::ERROR, __FUNCTION__, "Error creating grid from png: " + std::string( e.what() ) );
        return false;
    }
}




}
}
