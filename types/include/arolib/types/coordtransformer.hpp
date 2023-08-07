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
 
/**
  @file coordtransformer.hpp

  @brief declaration of conversion functions from geodetic (WGS84)
  coordinates to a local metric cartesian coordinate system using projection
  and translation.
*/

#ifndef AROLIB_COORDTRANSFORMER_H_
#define AROLIB_COORDTRANSFORMER_H_

#include <mutex>
#include <boost/math/constants/constants.hpp>


#ifdef BOOST_VERSION
    #if BOOST_VERSION / 100000 > 1
        #define AROLIB_COORDTRANSFORMER___BOOST_OK
    #elif BOOST_VERSION / 100 % 1000 > 70
        #define AROLIB_COORDTRANSFORMER___BOOST_OK
    #endif
#endif

#ifdef AROLIB_COORDTRANSFORMER___BOOST_OK
    #include <boost/geometry/geometry.hpp>
    #include <boost/geometry/core/cs.hpp>
    #include <boost/geometry/srs/epsg.hpp>
    #include <boost/geometry/srs/projection.hpp>
#endif


#include "arolib/misc/loggingcomponent.h"
#include "point.hpp"

#include <gdal/ogr_spatialref.h>
#include <gdal/ogr_geometry.h>

namespace arolib{

/**
 * @brief Singleton class with conversion functions from geodetic (WGS84) coordinates to a local metric cartesian coordinate system using projection
 */
class CoordTransformer : LoggingComponent{
public:

    /**
     * @brief Get the singleton instance
     */
    static CoordTransformer& GetInstance();

    /**
     * @brief Converts orientation from bearing (north = 0°, clockwise) to theta (x-axis = 0°, counter-clockwise)
     * @param bearing Geodetic orientation (bearing)
     * @param [out] theta Computed cartesian angle theta
     */
    void convert_to_cartesian(const double bearing, double &theta);

    /**
     * @brief Converts orientation from theta (x-axis = 0°, counter-clockwise) to bearing (north = 0°, clockwise)
     * @param theta Cartesian angle theta
     * @param [out] bearing Computed geodetic orientation (bearing)
     */
    void convert_to_geodetic(const double theta, double &bearing);

    /**
     * @brief Conversion of a geodetic point to a cartesian point.
     *
     * The conversion consists of a projection from WGS84 to a metric cartesian system (transversal mercator projection).
     * See @externalpage http://spatialreference.org/ref/epsg/32632/
     * @param from Geodetic point that will be converted (x:lon, y:lat, z:alt)
     * @param [out] to Computed cartesian point
     * @return True on success
     */
    bool convert_to_cartesian(const arolib::Point &from, arolib::Point &to);

    /**
     * @brief Conversion of a geodetic points to a cartesian points.
     * @param from Geodetic points that will be converted (x:lon, y:lat, z:alt)
     * @param [out] to Computed cartesian points
     * @return True on success
     */
    template< typename T,
              typename = typename std::enable_if< std::is_base_of<arolib::Point, T>::value, void >::type >
    bool convert_to_cartesian(const std::vector<T> &from, std::vector<T> &to){
        bool ok = true;
        to.resize(from.size());
        for(size_t i = 0 ; i < from.size() ; ++i)
            ok &= convert_to_cartesian(from.at(i), to.at(i));
        return ok;
    }

    /**
     * @brief Conversion of a cartesian point to a geodetic point
     *
     * The conversion consists of a projection from the metric cartesian system to WGS84 (inverse transversal mercator projection).
     * See @externalpage http://spatialreference.org/ref/epsg/32632/
     * @param from Cartesian point that will be converted
     * @param [out] to Computed geodetic point (x:lon, y:lat, z:alt)
     * @return True on success
     */
    bool convert_to_geodetic(const arolib::Point &from, arolib::Point &to, int utm_zone = -1, std::string designator = "Z");

    /**
     * @brief Conversion of a cartesian points to a geodetic points
     * @param from Cartesian points that will be converted
     * @param [out] to Computed geodetic points (x:lon, y:lat, z:alt)
     * @return True on success
     */
    template< typename T,
              typename = typename std::enable_if< std::is_base_of<arolib::Point, T>::value, void >::type >
    bool convert_to_geodetic(const std::vector<T> &from, std::vector<T> &to, int utm_zone = -1, std::string designator = "Z"){
        bool ok = true;
        to.resize(from.size());
        for(size_t i = 0 ; i < from.size() ; ++i)
            ok &= convert_to_geodetic(from.at(i), to.at(i), utm_zone, designator);
        return ok;
    }

    /**
     * @brief Get the current UTM zome
     * @return Current UTM zome
     */
    inline int getUTMZone() {return m_utm_zone;}

    /**
     * @brief Get the current UTM designator
     * @return Current UTM designator
     */
    inline std::string getUTMDesignator() {return m_utm_designator;}

  private:

    /**
     * @brief Constructor
     */
    CoordTransformer();

    /**
     * @brief Copy constructor
     */
    CoordTransformer(const CoordTransformer&);

    /**
     * @brief Copy assignment
     */
    CoordTransformer& operator=(const CoordTransformer&);


    /**
     * @brief Conversion of a geodetic point to a cartesian point  or viceversa using gdal library
     * @param from Source point that will be converted, either Geodetic point (x:lon, y:lat, z:alt) (wgs2utm == true) or cartesian (wgs2utm == false)
     * @param [out] to Computed point, either cartetian (wgs2utm == true) or Geodetic point (x:lon, y:lat, z:alt) (wgs2utm == false)
     * @param wgs2utm Convert from WGS to UTM or viceversa?
     * @return True on success
     */
    bool convert_with_gdal_2(const arolib::Point &from, arolib::Point &to, bool wgs2utm);


#ifdef AROLIB_COORDTRANSFORMER___BOOST_OK
    /**
     * @brief Conversion of a geodetic point to a cartesian point  or viceversa using boost geometry library
     * @param from Source point that will be converted, either Geodetic point (x:lon, y:lat, z:alt) (wgs2utm == true) or cartesian (wgs2utm == false)
     * @param [out] to Computed point, either cartetian (wgs2utm == true) or Geodetic point (x:lon, y:lat, z:alt) (wgs2utm == false)
     * @param wgs2utm Convert from WGS to UTM or viceversa?
     * @return True on success
     */
    bool convert_with_boost(const arolib::Point &from, arolib::Point &to, bool wgs2utm);
#endif

    /**
     * @brief Conversion of a geodetic point to a cartesian point using gdal library
     * @param from Geodetic point that will be converted (x:lon, y:lat, z:alt)
     * @param [out] to Computed cartesian point
     * @return True on success
     */
    bool convert_to_cartesian__gdal(const arolib::Point &from, arolib::Point &to);

    /**
     * @brief (not implemented) Conversion of a geodetic point to a cartesian point using WGS84toCartesian library
     * @param from Geodetic point that will be converted (x:lon, y:lat, z:alt)
     * @param [out] to Computed cartesian point
     * @return True on success
     */
    bool convert_to_cartesian__WGS84toCartesian(const arolib::Point &from, arolib::Point &to);

    /**
     * @brief (not implemented) Conversion of a geodetic point to a cartesian point using geographiclib library
     * @param from Geodetic point that will be converted (x:lon, y:lat, z:alt)
     * @param [out] to Computed cartesian point
     * @return True on success
     */
    bool convert_to_cartesian__geographiclib(const arolib::Point &from, arolib::Point &to);

    /**
     * @brief (not implemented) Conversion of a geodetic point to a cartesian point using proj4 library
     * @param from Geodetic point that will be converted (x:lon, y:lat, z:alt)
     * @param [out] to Computed cartesian point
     * @return True on success
     */
    bool convert_to_cartesian__proj4(const arolib::Point &from, arolib::Point &to);

    /**
     * @brief Conversion of a cartesian point to a geodetic point using gdal library
     * @param from Cartesian point that will be converted
     * @param [out] to Computed geodetic point (x:lon, y:lat, z:alt)
     * @return True on success
     */
    bool convert_to_geodetic__gdal(const arolib::Point &from, arolib::Point &to);

    /**
     * @brief (not implemented) Conversion of a cartesian point to a geodetic point geographiclib library
     * @param from Cartesian point that will be converted
     * @param [out] to Computed geodetic point (x:lon, y:lat, z:alt)
     * @return True on success
     */
    bool convert_to_geodetic__geographiclib(const arolib::Point &from, arolib::Point &to);

    /**
     * @brief (not implemented) Conversion of a cartesian point to a geodetic point using proj4 library
     * @param from Cartesian point that will be converted
     * @param [out] to Computed geodetic point (x:lon, y:lat, z:alt)
     * @return True on success
     */
    bool convert_to_geodetic__proj4(const arolib::Point &from, arolib::Point &to);

    static bool m_isUtmZoneSet; /**< Was the UTM zone set already? */
    static int m_utm_zone; /**< Current UTM zone */
    static std::string m_utm_designator; /**< Current UTM designator */
    static std::mutex m_mutex; /**< Mutex */
};

}

#endif  // AROLIB_COORDTRANSFORMER_H_
