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
 
#ifndef ARO_COLOR_HELPER_HPP
#define ARO_COLOR_HELPER_HPP

#include <type_traits>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <vector>
#include <map>

namespace arolib {


/**
 * @brief Type of color-mapping
 */
enum Range2RGBColorType{
    COLOR_GRAY,
    COLOR_HEAT,
    COLOR_GREEN2RED,
    COLOR_RED2GREEN
};

/**
 * @brief Get a Range2RGBColorType from its int value
 *
 * @param value Values
 */
Range2RGBColorType intToRange2RGBColorType(int value);

/**
 * @brief RGB pixel
 */
template<typename T>
struct RGBPixel{
    static_assert(std::is_arithmetic<T>::value, "Pixel type must be numeric (template argument)");
    static const T minPixelVal = std::numeric_limits<T>::lowest();

    T r = minPixelVal; /**< Red value */
    T g = minPixelVal; /**< Green value */
    T b = minPixelVal; /**< Blue value */

    /**
     * @brief Default constructor
     */
    RGBPixel() = default;

    /**
     * @brief Constructor
     *
     * @param _r Red value
     * @param _g Green value
     * @param _b Blue value
     */
    RGBPixel(T _r, T _g, T _b) : r(_r), g(_g), b(_b){}

    /**
     * @brief Get a black pixel
     */
    constexpr static RGBPixel<T> black(){
        return RGBPixel<T>(std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest());
    }

    /**
     * @brief Get a white pixel
     */
    constexpr static RGBPixel<T> white(){
        return RGBPixel<T>(std::numeric_limits<T>::max(), std::numeric_limits<T>::max(), std::numeric_limits<T>::max());
    }

    /**
     * @brief Get a red pixel
     */
    constexpr static RGBPixel<T> red(){
        return RGBPixel<T>(std::numeric_limits<T>::max(), std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest());
    }

    /**
     * @brief Get a green pixel
     */
    constexpr static RGBPixel<T> green(){
        return RGBPixel<T>(std::numeric_limits<T>::lowest(), std::numeric_limits<T>::max(), std::numeric_limits<T>::lowest());
    }

    /**
     * @brief Get a blue pixel
     */
    constexpr static RGBPixel<T> blue(){
        return RGBPixel<T>(std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest(), std::numeric_limits<T>::max());
    }
};


/**
 * @brief Get a gray value for a given value and range (grayscale)
 *
 * @param value Value
 * @param min Minimum value of the range
 * @param max Maximum value of the range
 * @return Gray value between T:min and T:max
 */
template<typename T>
T getGrayValue(double value, double min, double max){
    static_assert(std::is_arithmetic<T>::value, "Pixel type must be numeric (template argument)");
    static const T minPixelVal = std::numeric_limits<T>::lowest();
    static const T rangePixelVal = std::numeric_limits<T>::max() - std::numeric_limits<T>::lowest();
    T ret = std::numeric_limits<T>::max();
    if(min == max)
        return ret;
    if(min > max)
        std::swap(min, max);
    value = std::min( max, std::max( min, value) );

    value -= min;
    value *= (double)rangePixelVal / (max - min);
    ret = value + minPixelVal;
    return ret;
}

/**
 * @brief Get a value for a given gray value and range (grayscale)
 *
 * Inverse conversion from getGrayValue
 * @sa getGrayValue
 *
 * @param value Gray value between min T:min and T:max
 * @param min Minimum value of the range
 * @param max Maximum value of the range
 * @return Value between min min and max
 */
template<typename T>
double getValueFromGray(T value, double min, double max){
    static_assert(std::is_arithmetic<T>::value, "Pixel type must be numeric (template argument)");
    static const T minPixelVal = std::numeric_limits<T>::lowest();
    static const T rangePixelVal = std::numeric_limits<T>::max() - std::numeric_limits<T>::lowest();
    double ret = min;
    if(min == max)
        return ret;
//    if(min > max)
//        std::swap(min, max);
    ret = ((long double)value - minPixelVal) / rangePixelVal;
    ret *= (max-min);
    ret += min;
    ret = std::min( max, std::max( min, ret) );
    return ret;
}

/**
 * @brief Get a RGB pixel for a given value, range and color mapping type
 *
 * @param value Value
 * @param min Minimum value of the range
 * @param max Maximum value of the range
 * @param colorType Color mapping type
 * @return RGB pixel (with color values between T:min and T:max)
 */
template<typename T>
RGBPixel<T> getRGBValue(double value, double min, double max, Range2RGBColorType colorType){

    if(colorType == COLOR_GRAY){
        std::map<float, RGBPixel<T>> colormap = { {0.0, RGBPixel<T>::black()},
                                                  {1.0, RGBPixel<T>::white()} };

        return getRGBValue<T>(value, min, max, colormap);
    }
    if(colorType == COLOR_HEAT){

        std::map<float, RGBPixel<T>> colormap = { {0.0, RGBPixel<T>::blue()},
                                                  {0.5, RGBPixel<T>::green()},
                                                  {1.0, RGBPixel<T>::red()} };

        return getRGBValue<T>(value, min, max, colormap);
    }
    else{

        std::map<float, RGBPixel<T>> colormap = { {0.0, RGBPixel<T>::green()},
                                                  {1.0, RGBPixel<T>::red()} };
        auto ret = getRGBValue<T>(value, min, max, colormap);
        if(colorType == COLOR_RED2GREEN)
            std::swap(ret.r, ret.g);
        return ret;
    }
}


/**
 * @brief Get a value for a RGB Pixe, range, and color mapping type
 *
 * Inverse conversion from getRGBValue
 * @sa getRGBValue
 *
 * @param value RGB pixel (with color values between T:min and T:max)
 * @param min Minimum value of the range
 * @param max Maximum value of the range
 * @param colorType Color mapping type used to generate the pixel
 * @return Value between min min and max
 */
template<typename T>
double getValueFromRGB(const RGBPixel<T>& value, double min, double max, Range2RGBColorType colorType){
    static const T minPixelVal = std::numeric_limits<T>::lowest();
    static const T rangePixelVal = std::numeric_limits<T>::max() - std::numeric_limits<T>::lowest();
    double ret = max;
    if(min == max)
        return ret;
//    if(min > max)
//        std::swap(min, max);

    if(colorType == COLOR_GRAY){
        return getValueFromGray<T>(value.r, min, max);
    }
    if(colorType == COLOR_HEAT){
        double ratio = 0.5;
        long double b = (long double)value.b - minPixelVal;
        long double r = (long double)value.r - minPixelVal;
        if( std::fabs(b) > 1e-6 )
            ratio = 0.5 * ( 1 - b / rangePixelVal );
        else if ( std::fabs(r) > 1e-6 )
            ratio = 1 - 0.5 * ( 1 - r / rangePixelVal );
        ret = ratio * (max-min) + min;
    }
    else{
        long double r = (long double)value.r - minPixelVal;
        double ratio = r / rangePixelVal;
        if(colorType == COLOR_RED2GREEN)
            ratio = 1 - ratio;
        ret = ratio * (max-min) + min;
    }

    ret = std::min( max, std::max( min, ret) );

    return ret;
}


/**
 * @brief Get a RGB pixel for a given value, range, and custom colormap thresholds
 *
 * @param value Value
 * @param min Minimum value of the range
 * @param max Maximum value of the range
 * @param colormapThresholds Colormap thresholds
 * @return RGB pixel (with color values between T:min and T:max)
 */
template<typename T>
RGBPixel<T> getRGBValue(double value, double min, double max, const std::map< float, RGBPixel<T> >& colormapThresholds){
    if(colormapThresholds.empty())
        return RGBPixel<T>::black();
    if(colormapThresholds.size() == 1)
        return colormapThresholds.begin()->second;

    if(min == max)
        return value <= min ? colormapThresholds.begin()->second : colormapThresholds.rbegin()->second;


    if(min > max)
        std::swap(min, max);

    double relVal = (value - min) / (max - min);

    if( value <= min || relVal <= colormapThresholds.begin()->first )
        return colormapThresholds.begin()->second;
    if( value >= max || relVal >= colormapThresholds.rbegin()->first )
        return colormapThresholds.rbegin()->second;

    RGBPixel<T> ret;

    auto it_high = colormapThresholds.upper_bound(relVal);
    if(it_high == colormapThresholds.end())
        return colormapThresholds.rbegin()->second;
    if(it_high == colormapThresholds.begin())
        return colormapThresholds.begin()->second;
    auto it_low = std::prev(it_high);

    float step = it_high->first - it_low->first;
    double dR = it_low->second.r;
    double dG = it_low->second.g;
    double dB = it_low->second.b;

    if( std::is_integral<T>::value ){
        dR += 0.5;
        dG += 0.5;
        dB += 0.5;
    }

    double ratio = ( relVal - it_low->first ) / ( step );

    ret.r = dR + ratio * (it_high->second.r - it_low->second.r);
    ret.g = dG + ratio * (it_high->second.g - it_low->second.g);
    ret.b = dB + ratio * (it_high->second.b - it_low->second.b);

    return ret;
}


}

#endif // ARO_COLOR_HELPER_HPP
