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
 
#include "arolib/types/coordtransformer.hpp"

namespace arolib {

    int CoordTransformer::m_utm_zone = 32;
    std::string CoordTransformer::m_utm_designator = "U";
    bool CoordTransformer::m_isUtmZoneSet = false;
    std::mutex CoordTransformer::m_mutex;

/**
 * @brief Computes the UTM zone and designator
 * @param lon Logitude
 * @param lat Latitude
 * @param [out] zone Computed UTM zone
 * @param [out] designator Computed UTM designator
 * @return True on success
 *
 * @see https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
 */
bool computeUTMZone(double lon, double lat, int& zone, std::string& designator) {
  std::string letters = "CDEFGHJKLMNPQRSTUVWXX";
  if( -80 <= lat && lat <= 84 )
    designator = letters.substr((lat + 80) / 8, 1);
  else{
      designator = "Z"; //This is here as an error flag to show that the latitude is outside the UTM limits
      return false;
  }
  zone = floor((lon + 180.0) / 6.0) + 1; // longitudinal zone
  if( lat >= 56.0 && lat < 64.0 && lon >= 3.0 && lon < 12.0 )
          zone = 32;
  // Special zones for Svalbard
  if( lat >= 72.0 && lat < 84.0 ) {
    if  ( lon >= 0.0  && lon <  9.0 )
          zone = 31;
    else if( lon >= 9.0  && lon < 21.0 )
          zone = 33;
    else if(lon >= 21.0 && lon < 33.0 )
          zone = 35;
    else if(lon >= 33.0 && lon < 42.0 )
          zone = 37;
  }
  return true;
}

CoordTransformer &CoordTransformer::GetInstance() {
  static CoordTransformer instance;
  return instance;
}

void CoordTransformer::convert_to_cartesian(const double bearing, double &theta) {
    std::lock_guard<std::mutex> guard(m_mutex);
    theta = boost::math::constants::pi<double>()/2.0 -
        (bearing/180.0*boost::math::constants::pi<double>());
    if (theta < 0)
      theta += 2*boost::math::constants::pi<double>();
}

void CoordTransformer::convert_to_geodetic(const double theta, double &bearing) {
    std::lock_guard<std::mutex> guard(m_mutex);
    bearing = 180.0/boost::math::constants::pi<double>() *
            (boost::math::constants::pi<double>()/2.0 - theta);
    if (bearing < 0)
      bearing += 360;
}

bool CoordTransformer::convert_to_cartesian(const Point &from, Point &to) {
    std::lock_guard<std::mutex> guard(m_mutex);
    int zone = m_utm_zone;
    std::string designator = m_utm_designator;

    if(!m_isUtmZoneSet){//allow utm zone changes only once
        if( !computeUTMZone(from.x, from.y,zone,designator) ){
            logger().printOut(LogLevel::WARNING, __FUNCTION__, 10, "Error computing UTM zone for point ", from.toString(10));
            return false;
        }
        //m_isUtmZoneSet = true;// do the initialization just once (@TODO or again in case the utm_zone changes?)
        if(zone != m_utm_zone) {
            logger().printOut(LogLevel::WARNING, __FUNCTION__, -1, "Change of UTM zone:  ", m_utm_zone, m_utm_designator,
                              " --> ", zone, designator,
                              " :: ", from.toString(10));
            m_utm_zone = zone;
            m_utm_designator = designator;
        }
    }

//Note: at the moment of testing, gdal v3.x had significant performance issues
//@todo; should the defailt be boost if available?
#if GDAL_VERSION_MAJOR<3
    return convert_with_gdal_2(from, to, true);
#else
    #ifdef AROLIB_COORDTRANSFORMER___BOOST_OK
        return convert_with_boost(from, to, true);
    #else
        return convert_with_gdal_2(from, to, true);
    #endif
#endif
}

bool CoordTransformer::convert_to_geodetic(const Point &from, Point &to, int zone, std::string designator) {

    std::lock_guard<std::mutex> guard(m_mutex);
    if(zone > 0  && zone != m_utm_zone) {
        m_utm_zone = zone;
        m_utm_designator = designator;
    }

    //@warning
    //if using m_utm_zone and m_utm_designator, it is needed that the zone and designator are up-to-date and correspond to the correct area (e.g. by calling convert_to_cartesian on a point in the area)
    //it can happen that a geometry/field/etc was saved in UTM and never calls convert_to_cartesian when reading it, hence when converting to WGS it might use an incorrect zone (e.g. if the last geometry/field/... tha was read from WGS bolongs to another zone)

//Note: at the moment of testing, gdal v3.x had significant performance issues
//@todo; should the defailt be boost if available?
#if GDAL_VERSION_MAJOR<3
    return convert_with_gdal_2(from, to, false);
#else
    #ifdef AROLIB_COORDTRANSFORMER___BOOST_OK
        return convert_with_boost(from, to, false);
    #else
        return convert_with_gdal_2(from, to, false);
    #endif
#endif
}

bool CoordTransformer::convert_with_gdal_2(const Point &from, Point &to, bool wgs2utm)
{
    OGRSpatialReference    oUTM, *poLongLat;
    OGRCoordinateTransformation *poTransform = nullptr;
    OGRPoint* geom = nullptr;
    try{
        std::string utmDesc = "UTM " + std::to_string(m_utm_zone) + " / WGS84";
        oUTM.SetProjCS(utmDesc.c_str());
        oUTM.SetWellKnownGeogCS( "WGS84" );
        oUTM.SetUTM( m_utm_zone, 1 );

        poLongLat = oUTM.CloneGeogCS();

#ifdef GDAL_VERSION_MAJOR
#if GDAL_VERSION_MAJOR>2
        poLongLat->SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
#endif
#endif

        if(wgs2utm)
            poTransform = OGRCreateCoordinateTransformation(poLongLat, &oUTM);
        else
            poTransform = OGRCreateCoordinateTransformation(&oUTM, poLongLat);

        if (!poTransform){
            std::cerr << "error: (convert_with_gdal): poTransform = null" << std::endl;
            return false;
        }

        geom = dynamic_cast<OGRPoint*>( OGRGeometryFactory::createGeometry(wkbPoint) );
        geom->setX(from.x);
        geom->setY(from.y);
        geom->setZ(from.z);

        OGRErr gdalError = geom->transform(poTransform);
        if (gdalError != OGRERR_NONE ){
            std::cerr << "error: (convert_with_gdal): gdalError = " << gdalError << std::endl;
            delete poTransform;
            delete geom;
            return false;
        }

        to = Point( geom->getX(), geom->getY(), geom->getZ() );

        delete poTransform;
        delete geom;
        delete poLongLat;
        return true;  // exit successfully
    }
    catch(...){
        if(poTransform)
            delete poTransform;
        if(geom)
            delete geom;
        if(poLongLat)
            delete poLongLat;
        return false;
    }

}

#ifdef AROLIB_COORDTRANSFORMER___BOOST_OK
bool CoordTransformer::convert_with_boost(const arolib::Point &from, arolib::Point &to, bool wgs2utm){
    using boost_point_wgs_t = boost::geometry::model::d2::point_xy<double, boost::geometry::cs::geographic<boost::geometry::degree> >;
    using boost_point_utm_t = boost::geometry::model::d2::point_xy<double>;

    static int zone = 32;
    static boost::geometry::srs::projection<> proj = boost::geometry::srs::proj4("+proj=utm +ellps=WGS84 +units=m +no_defs +zone=" + std::to_string(zone));
    
    if(zone != m_utm_zone){
        zone = m_utm_zone;
        proj = boost::geometry::srs::proj4("+proj=utm +ellps=WGS84 +units=m +no_defs +zone=" + std::to_string(m_utm_zone));
    }

    if(wgs2utm){
        boost_point_wgs_t bFrom( from.x, from.y );
        boost_point_utm_t bTo;
        if( !proj.forward(bFrom, bTo) ){
            std::cerr << "error converting WGS to UTM (convert_with_boost)" << std::endl;
            return false;
        }
        to.x = boost::geometry::get<0>(bTo);
        to.y = boost::geometry::get<1>(bTo);
    }
    else{
        boost_point_utm_t bFrom( from.x, from.y );
        boost_point_wgs_t bTo;
        if( !proj.inverse(bFrom, bTo) ){
            std::cerr << "error converting WGS to UTM (convert_with_boost)" << std::endl;
            return false;
        }
        to.x = boost::geometry::get<0>(bTo);
        to.y = boost::geometry::get<1>(bTo);
    }
    
    return true;

}
#endif

bool CoordTransformer::convert_to_cartesian__gdal(const Point &from, Point &to)
{
    OGRCoordinateTransformation *raw = nullptr;
    OGRPoint* geom = nullptr;
    try{
        OGRSpatialReference frameFrom, frameTo;

        frameFrom.SetWellKnownGeogCS("WGS84");
        frameTo.SetWellKnownGeogCS("WGS84");
        frameTo.SetUTM(m_utm_zone, true);

        raw = OGRCreateCoordinateTransformation(&frameFrom, &frameTo);

        if (!raw){
            std::cerr << "error: (convert_to_cartesian): poTransform = null" << std::endl;
            return false;
        }

        geom = dynamic_cast<OGRPoint*>( OGRGeometryFactory::createGeometry(wkbPoint) );
        geom->setX(from.x);
        geom->setY(from.y);
        geom->setZ(from.z);

        OGRErr gdalError = geom->transform(raw);
        if (gdalError != OGRERR_NONE ){
            std::cerr << "error: (convertToCartesian): gdalError = " << gdalError << std::endl;
            delete raw;
            delete geom;
            return false;
        }

        to = Point( geom->getX(), geom->getY(), geom->getZ() );

        delete raw;
        delete geom;
        return true;  // exit successfully
    }
    catch(...){
        if(raw)
            delete raw;
        if(geom)
            delete geom;
        return false;
    }

}


bool CoordTransformer::convert_to_geodetic__gdal(const Point &from, Point &to)
{
    OGRCoordinateTransformation *raw = nullptr;
    OGRPoint* geom = nullptr;
    try{
        OGRSpatialReference frameFrom, frameTo;
        OGRErr gdalError;

        gdalError = frameTo.SetWellKnownGeogCS("WGS84");
        gdalError = frameFrom.SetWellKnownGeogCS("WGS84");
        gdalError = frameFrom.SetUTM(m_utm_zone, true);
        //gdalError = frameTo.SetUTM(m_utm_zone, true);

        raw = OGRCreateCoordinateTransformation(&frameFrom, &frameTo);

        if (!raw){
            std::cerr << "error: (convert_to_geodetic): poTransform = null" << std::endl;
            return false;
        }

        geom = dynamic_cast<OGRPoint*>( OGRGeometryFactory::createGeometry(wkbPoint) );
        geom->setX(from.x);
        geom->setY(from.y);
        geom->setZ(from.z);

        gdalError = geom->transform(raw);
        if (gdalError != OGRERR_NONE ){
            logger().printOut(LogLevel::WARNING, __FUNCTION__, 10, "gdalError =  ", gdalError);
            delete raw;
            delete geom;
            return false;
        }

        to = Point( geom->getX(), geom->getY(), geom->getZ() );

        delete raw;
        delete geom;
        return true;  // exit successfully
    }
    catch(...){
        if(raw)
            delete raw;
        if(geom)
            delete geom;
        return false;
    }

}

CoordTransformer::CoordTransformer():
    LoggingComponent(LogLevel::INFO, __FUNCTION__) {
  // Initialize projection parameters
  m_utm_zone = 32;
  m_utm_designator = "U";
  m_isUtmZoneSet = false;
}

}
