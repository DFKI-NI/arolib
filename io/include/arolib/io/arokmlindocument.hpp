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
 
#ifndef AROLIB_IO_AROKMLINDOCUMENT_HPP
#define AROLIB_IO_AROKMLINDOCUMENT_HPP

#include <ostream>
#include <fstream>
#include <sstream>
#include <functional>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/optional/optional.hpp>
#include <boost/algorithm/string.hpp>

#include "xmlindocument.hpp"
#include "kmltags.hpp"
#include "io_common.hpp"
#include "arolib/types/coordtransformer.hpp"
#include "arolib/geometry/geometry_helper.hpp"
#include "arolib/misc/base64Utility.hpp"
#include "arolib/misc/tuple_helper.h"

namespace arolib {
namespace io {

/**
 * @brief Arolib KML input document for Arolib types
 */
class AroKMLInDocument : public XMLInDocument{

public:

    /**
     * @brief KML-tag structure
     */
    struct KmlTag{
        std::string kml_tag; /**< Tag */
        std::string name; /**< Name */

        /**
         * @brief Constructor.
         *
         * @param _kml_tag Tag
         * @param _name Name
         */
        KmlTag(const std::string& _kml_tag, const std::string& _name = UseDefaultTag);

        /**
         * @brief Type-based constructor.
         *
         * Create with default tag and name depending on the type
         * @param t Object of type T
         * @param _name Name
         */
        template<typename T,
                 typename = typename std::enable_if< !std::is_same<T, std::string>::value >::type>
        KmlTag(const T& t, const std::string& _name = UseDefaultTag):
            kml_tag( getKMLBaseTag(t) ),
            name( getTag<T>(_name) )
        {
        }
    };

    /**
     * @brief Constructor.
     *
     * @param logLevel Log level
     */
    explicit AroKMLInDocument(LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Destructor.
     */
    virtual ~AroKMLInDocument();

    /**
     * @brief Read object/value from base ReadHandler.
     *
     * Note: needed because overloading is not managed in inheritance, but we want to use the available (standard) reads from XMLInDocument
     * @param base Base handler
     * @param [out] value Object/value read from the base handler
     * @return True on success
     */
    template< typename T,
              typename = typename std::enable_if< XMLInDocument::has_read_method<T>::value, void >::type >
    bool read( const ReadHandler & base, T& value ){
        return XMLInDocument::read(base, value);
    }


    /**
     * @brief Read point from base ReadHandler.
     * @param base Base handler
     * @param [out] pt Point read from the base handler
     * @param description Description holding (meta) information
     * @return True on success
     */
    bool read( const ReadHandler & base, Point &pt, std::string* description = nullptr );

    /**
     * @brief Read points from base ReadHandler.
     * @param base Base handler
     * @param [out] pts Points read from the base handler
     * @param description Description holding (meta) information
     * @return True on success
     */
    bool read( const ReadHandler & base, std::vector<Point> &pts, std::string* description = nullptr );

    /**
     * @brief Read linestring from base ReadHandler.
     * @param base Base handler
     * @param [out] ls Linestring read from the base handler
     * @param description Description holding (meta) information
     * @return True on success
     */
    bool read( const ReadHandler & base, Linestring &ls, std::string* description = nullptr );

    /**
     * @brief Read polygon from base ReadHandler.
     * @param base Base handler
     * @param [out] poly Polygon read from the base handler
     * @param description Description holding (meta) information
     * @return True on success
     */
    bool read( const ReadHandler & base, Polygon &poly, std::string* description = nullptr );

    /**
     * @brief Read field-access point from base ReadHandler.
     * @param base Base handler
     * @param [out] pt Point read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, FieldAccessPoint &pt );

    /**
     * @brief Read field-access points from base ReadHandler.
     * @param base Base handler
     * @param [out] pts Points read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, std::vector<FieldAccessPoint> &pts );

    /**
     * @brief Read resource point from base ReadHandler.
     * @param base Base handler
     * @param [out] pt Point read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, ResourcePoint &pt );

    /**
     * @brief Read resource points from base ReadHandler.
     * @param base Base handler
     * @param [out] pts Points read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, std::vector<ResourcePoint> &pts );

    /**
     * @brief Read route point from base ReadHandler.
     * @param base Base handler
     * @param [out] pt Point read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, RoutePoint &pt );

    /**
     * @brief Read route points from base ReadHandler.
     * @param base Base handler
     * @param [out] pts Points read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, std::vector<RoutePoint> &pts );

    /**
     * @brief Read headland point from base ReadHandler.
     * @param base Base handler
     * @param [out] pt Point read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, HeadlandPoint &pt );

    /**
     * @brief Read headland points from base ReadHandler.
     * @param base Base handler
     * @param [out] pts Points read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, std::vector<HeadlandPoint> &pts );

    /**
     * @brief Read track from base ReadHandler.
     * @param base Base handler
     * @param [out] track Track read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, Track &track );

    /**
     * @brief Read tracks from base ReadHandler.
     * @param base Base handler
     * @param [out] tracks Tracks read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, std::vector<Track> &tracks );

    /**
     * @brief Read headlands from base ReadHandler.
     * @param base Base handler
     * @param [out] headlands Headlands read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, Headlands &headlands );

    /**
     * @brief Read complete headland from base ReadHandler.
     * @param base Base handler
     * @param [out] hl Complete headland read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, CompleteHeadland &hl );

    /**
     * @brief Read obstacle from base ReadHandler.
     * @param base Base handler
     * @param [out] obs Obstacle read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, Obstacle &obs );

    /**
     * @brief Read obstacles from base ReadHandler.
     * @param base Base handler
     * @param [out] obstacles Obstacles read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, std::vector<Obstacle> &obstacles );

    /**
     * @brief Read subfield from base ReadHandler.
     * @param base Base handler
     * @param [out] sf Subfield read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, Subfield &sf );

    /**
     * @brief Read subfields from base ReadHandler.
     * @param base Base handler
     * @param [out] sfs Subfields read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, std::vector<Subfield> &sfs );

    /**
     * @brief Read field from base ReadHandler.
     * @param base Base handler
     * @param [out] field Field read from the base handler
     * @return True on success
     */
    bool read( const ReadHandler & base, Field& field );


    /**
     * @brief Read an object/value with a given KLM-tag (and optional KML parent tags) from the base handler (root) of the document.
     * @param [out] value Object/value read
     * @param kmlTag Expected kmlTag for the value
     * @param parentTags KML parent tags to reach the location of the value/kmlTag
     * @return True on success
     */
    template<typename T,
             typename = typename std::enable_if< !std::is_same<T, ReadHandler>::value
                                                 && !std::is_same<T, boost::property_tree::ptree>::value >::type>
    bool read(T& value, const KmlTag& kmlTag, const std::vector<std::string>& parentTags = {}){

        bool wasOpen = m_isDocOpen;
        if(!wasOpen){
            if(!openDocument()){
                if(isReadyToRead())
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Cannot open document");
                return false;
            }
        }

        bool resp = read(value, m_base, kmlTag, parentTags);
        if(!wasOpen)
            closeDocument();
        return resp;
    }


    /**
     * @brief Read an object/value with a given KLM-tag (and optional KML parent tags) from a given base handler.
     * @param [out] value Object/value read
     * @param base Base handler
     * @param kmlTag Expected kmlTag for the value
     * @param parentTags KML parent tags to reach the location of the value/kmlTag
     * @return True on success
     */
    template<typename T,
             typename = typename std::enable_if< !std::is_same<T, ReadHandler>::value
                                                 && !std::is_same<T, boost::property_tree::ptree>::value >::type>
    bool read(T& value, const ReadHandler & base, const KmlTag& kmlTag, const std::vector<std::string>& parentTags = {}){

        if(!m_isDocOpen){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
            return false;
        }

        if(kmlTag.kml_tag.empty()){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid kml tag");
            return false;
        }

        ReadHandler branch;
        if(!getBranch(base, parentTags, branch))
            return false;

        auto kmlTagEd = kmlTag;
        kmlTagEd.name = getTag<T>(kmlTag.name);

        if(!getKmlBranch(branch, kmlTagEd, branch))
            return false;

        bool resp = read(branch, value);

        return resp;
    }


    /**
     * @brief Read multiple objects/values with a given KLM-tag (and optional KML parent tags) from the base handler (root) of the document.
     * @param [out] values Objects/values read
     * @param kmlTag Expected kmlTag for the value
     * @param parentTags KML parent tags to reach the location of the value/kmlTag
     * @param strict If true, all values under the given tag must be read without errors
     * @return True on success
     */
    template<typename T,
             typename = typename std::enable_if< !std::is_same<T, ReadHandler>::value
                                                 && !std::is_same<T, boost::property_tree::ptree>::value >::type>
    bool readMultiple( std::vector<T>& values, const KmlTag& kmlTag, const std::vector<std::string>& parentTags = {}, bool strict = false){

        values.clear();
        bool wasOpen = m_isDocOpen;
        if(!wasOpen){
            if(!openDocument()){
                if(isReadyToRead())
                    m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Cannot open document");
                return false;
            }
        }

        if(kmlTag.kml_tag.empty()){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid kml tag");
            return false;
        }

        ReadHandler branch;
        if(!getBranchHandler(parentTags, branch))
            return false;
        size_t count = 0;

        std::string tag = getTag<T>(kmlTag.name);

        BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, RHTree(branch)){
            if(v.first == kmlTag.kml_tag){
                if(!tag.empty()){
                    std::string name;
                    if(!getValue(v.second, KMLTag_name, name) || name != tag)
                        continue;
                }

                ++count;
                values.push_back( T() );
                if(!read( createRH(v.second) , values.back())){
                    values.pop_back();
                    if(strict){
                        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error reading element # " + std::to_string(count));
                        return false;
                    }
                    m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error reading element # " + std::to_string(count));
                }
            }
        }

        if(!wasOpen)
            closeDocument();
        return true;
    }


    /**
     * @brief Read multiple objects/values with a given KLM-tag from a given base handler.
     * @param base Base handler
     * @param [out] values Objects/values read
     * @param kmlTag Expected kmlTag for the value
     * @param strict If true, all values under the given tag must be read without errors
     * @return True on success
     */
    template<typename T,
             typename = typename std::enable_if< !std::is_same<T, ReadHandler>::value
                                                 && !std::is_same<T, boost::property_tree::ptree>::value >::type>
    bool readMultiple( const ReadHandler & base, std::vector<T>& values, const KmlTag& kmlTag, bool strict = false){

        values.clear();

        if(!m_isDocOpen){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Document is not open");
            return false;
        }

        if(kmlTag.kml_tag.empty()){
            m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Invalid kml tag");
            return false;
        }

        std::string tag = getTag<T>(kmlTag.name);
        size_t count = 0;
        BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, RHTree(base)){
            if(v.first == kmlTag.kml_tag){
                if(!tag.empty()){
                    std::string name;
                    if(!getValue(v.second, KMLTag_name, name) || name != tag)
                        continue;
                }

                ++count;
                values.push_back( T() );
                if(!read( createRH(v.second) , values.back())){
                    values.pop_back();
                    if(strict){
                        m_logger.printOut(LogLevel::ERROR, __FUNCTION__, "Error reading element # " + std::to_string(count));
                        return false;
                    }
                    m_logger.printOut(LogLevel::WARNING, __FUNCTION__, "Error reading element # " + std::to_string(count));
                }
            }
        }
        return true;
    }

    /**
     * @brief Read a field from a (Arolib-formatted) KML file.
     * @param filename Filename
     * @param [out] field Read Field
     * @param coordinatesType_out Projection type for the target coordinates
     * @param parentTags KML parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readField(const std::string& filename,
                          Field& field,
                          Point::ProjectionType coordinatesType_out = Point::UTM,
                          const std::vector<std::string>& parentTags = {},
                          LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Read multiple fields from a (Arolib-formatted) KML file.
     * @param filename Filename
     * @param [out] fields Read Fields
     * @param coordinatesType_out Projection type for the target coordinates
     * @param parentTags KML parent tags to reach the location of the data to be read
     * @param logLevel Log level
     * @return True on success
     */
    static bool readFields(const std::string& filename,
                           std::vector<Field>& fields,
                           Point::ProjectionType coordinatesType_out = Point::UTM,
                           const std::vector<std::string>& parentTags = {},
                           LogLevel logLevel = LogLevel::INFO);

    /**
     * @brief Get (general) strict flag.
     * @return (general) strict flag
     */
    bool strict() const;

    /**
     * @brief Set (general) strict flag.
     *
     * If set to false, some of the object properties will be set to the default values if not present in the KML file
     * @param strict (general) strict flag
     */
    void setStrict(bool strict);

protected:

    /**
     * @brief Open document (AroDocument derived class implementation).
     *
     * @return True on success
     */
    virtual bool openDoc() override;


    /**
     * @brief Parse a string holding coordinates to the respective point.
     *
     * @param coordinates Coordinates as string
     * @param [out] pt Output point
     * @return True on success
     */
    bool stringToPoint(std::string coordinates, Point& pt);

    /**
     * @brief Parse a string holding multiple coordinates to the respective points.
     *
     * @param coordinates Coordinates as string
     * @param [out] pts Output points
     * @return True on success
     */
    bool stringToPointList(std::string coordinates, std::vector<Point>& pts);


    /**
     * @brief Check if there is a value tagged with a given name in the given tree-value pair
     *
     * @param v Tree
     * @param name Name
     * @return True if value exists
     */
    template <typename T>
    static bool hasValue(const boost::property_tree::ptree::value_type & v, const std::string& name){
        return (v.second.get_optional<T>(name));
    }


    /**
     * @brief Check if there is a value tagged with a given name in the given tree
     *
     * @param v Tree
     * @param name Name
     * @return True if value exists
     */
    template <typename T>
    static bool hasValue(const boost::property_tree::ptree & tree, const std::string& name){
        return (tree.get_optional<T>(name));
    }


    /**
     * @brief Read a value tagged with a given name in the given tree
     *
     * @param v Tree
     * @param name Name
     * @param [out] value Value read
     * @return True on success
     */
    template <typename T>
    static bool getValue(const boost::property_tree::ptree& tree, const std::string& name, T& value){
        try{
            auto ovalue = tree.get_optional<T>(name);
            if(!ovalue)
                return false;
            value = *ovalue;
            return true;
        }
        catch(...){
            return false;
        }
    }

    /**
     * @brief Read a value in the given tree
     *
     * @param v Tree
     * @param [out] value Value read
     * @return True on success
     */
    template< typename T >
    static bool getValue( const boost::property_tree::ptree& tree, T& value ){
        auto ovalue = tree.get_value_optional<T>();
        if(!ovalue)
            return false;
        value = *ovalue;
        return true;
    }


    /**
     * @brief Get a branch with a given KML tag from a given (base) tree
     *
     * @param tree Tree
     * @param kmlTag KLM tag
     * @param [out] branch Output branch
     * @return True on success
     */
    bool getKmlBranch(const boost::property_tree::ptree& tree, const KmlTag& kmlTag, boost::property_tree::ptree& branch);

    /**
     * @brief Get a branch with a given sequence of KML tags from a given (base) tree
     *
     * @param tree Tree
     * @param kmlTags KLM tags sequence
     * @param [out] branch Output branch
     * @return True on success
     */
    bool getKmlBranch(const boost::property_tree::ptree& tree, const std::vector< KmlTag >& kmlTags, boost::property_tree::ptree& branch);

    /**
     * @brief Get a branch with a given KML tag from the (base) tree of the document
     *
     * @param kmlTag KLM tag
     * @param [out] branch Output branch
     * @return True on success
     */
    bool getKmlBranch(const KmlTag& kmlTag, boost::property_tree::ptree& branch);

    /**
     * @brief Get a branch with a given sequence of KML tags from the (base) tree of the document
     *
     * @param kmlTags KLM tags sequence
     * @param [out] branch Output branch
     * @return True on success
     */
    bool getKmlBranch(const std::vector< KmlTag >& kmlTags, boost::property_tree::ptree& branch);

    /**
     * @brief Get a branch handler with a given KML tag from a given (base) handler
     *
     * @param base (base) Handler
     * @param kmlTag KLM tag
     * @param [out] branch Output branch handler
     * @return True on success
     */
    bool getKmlBranch(const ReadHandler & base, const KmlTag& kmlTag, ReadHandler &branch);


    /**
     * @brief Get a branch handler with a given sequence of KML tags from a given (base) handler
     *
     * @param base (base) Handler
     * @param kmlTags KLM tags sequence
     * @param [out] branch Output branch handler
     * @return True on success
     */
    bool getKmlBranch(const ReadHandler & base, const std::vector< KmlTag >& kmlTags, ReadHandler &branch);

    /**
     * @brief Get a branch handler with a given KML tag from the (base) tree of the document
     *
     * @param kmlTag KLM tag
     * @param [out] branch Output branch handler
     * @return True on success
     */
    bool getKmlBranch(const KmlTag& kmlTag, ReadHandler &branch);


    /**
     * @brief Get a branch handler with a given sequence of KML tags from the (base) tree of the document
     *
     * @param kmlTags KLM tags sequence
     * @param [out] branch Output branch handler
     * @return True on success
     */
    bool getKmlBranch(const std::vector< KmlTag >& kmlTags, ReadHandler &branch);



    /**
     * @brief Read field properties from description
     *
     * @param [out] field Field read
     * @param description Description holding the properties
     * @param strict If true, all the properties must be read from the description for success.
     * @return True on success
     */
    static bool readFromDescription(Field& field, const std::string& description, bool strict);

    /**
     * @brief Read subfield properties from description
     *
     * @param [out] sf Subfield read
     * @param description Description holding the properties
     * @param strict If true, all the properties must be read from the description for success.
     * @return True on success
     */
    static bool readFromDescription(Subfield& sf, const std::string& description, bool strict);

    /**
     * @brief Read linestring properties from description
     *
     * @param [out] ls Linestring read
     * @param description Description holding the properties
     * @param strict If true, all the properties must be read from the description for success.
     * @return True on success
     */
    static bool readFromDescription(Linestring& ls, const std::string& description, bool strict);

    /**
     * @brief Read field-access point properties from description
     *
     * @param [out] pt Field-access point read
     * @param description Description holding the properties
     * @param strict If true, all the properties must be read from the description for success.
     * @return True on success
     */
    static bool readFromDescription(FieldAccessPoint& pt, const std::string& description, bool strict);

    /**
     * @brief Read resource point properties from description
     *
     * @param [out] pt Resource point read
     * @param description Description holding the properties
     * @param strict If true, all the properties must be read from the description for success.
     * @return True on success
     */
    static bool readFromDescription(ResourcePoint& pt, const std::string& description, bool strict);

    /**
     * @brief Read headland point properties from description
     *
     * @param [out] pt Headland point read
     * @param description Description holding the properties
     * @param strict If true, all the properties must be read from the description for success.
     * @return True on success
     */
    static bool readFromDescription(HeadlandPoint& pt, const std::string& description, bool strict);

    /**
     * @brief Read route point properties from description
     *
     * @param [out] pt Route point read
     * @param description Description holding the properties
     * @param strict If true, all the properties must be read from the description for success.
     * @return True on success
     */
    static bool readFromDescription(RoutePoint& pt, const std::string& description, bool strict);

    /**
     * @brief Read track properties from description
     *
     * @param [out] track Track read
     * @param description Description holding the properties
     * @param strict If true, all the properties must be read from the description for success.
     * @return True on success
     */
    static bool readFromDescription(Track& track, const std::string& description, bool strict);

    /**
     * @brief Read obstacle properties from description
     *
     * @param [out] obs Obstacle read
     * @param description Description holding the properties
     * @param strict If true, all the properties must be read from the description for success.
     * @return True on success
     */
    static bool readFromDescription(Obstacle& obs, const std::string& description, bool strict);

    /**
     * @brief Read complete headland properties from description
     *
     * @param [out] hl Complete headland read
     * @param description Description holding the properties
     * @param strict If true, all the properties must be read from the description for success.
     * @return True on success
     */
    static bool readFromDescription(CompleteHeadland& hl, const std::string& description, bool strict);


    /**
     * @brief Read object properties from the description located in a given (base) handler (under the default tag for the 'description' value)
     *
     * @param base (base) Handler
     * @param [out] t object read
     * @return True on success
     */
    template<typename T>
    bool readFromDescription(const ReadHandler & base, T& t){
        std::string desc;
        if(!getValue(RHTree(base), KMLTag_description, desc) && m_strict)
            return false;

        return readFromDescription(t, desc, m_strict);
    }

    /**
     * @brief Split the description into its items (key/value pairs)
     *
     * @param description Complete description
     * @param [out] t object read
     * @return Description items (key/value pairs)
     */
    static std::vector< std::pair<std::string, std::string> > getDescriptionItems(const std::string &description);

protected:
    bool m_strict = true;  /**< If set to false, some of the object properties will be set to the default values if not present in the KML file. Otherwise, the read methods will fail */
};

}
}//end namespace arolib


#endif //AROLIB_IO_AROKMLINDOCUMENT_HPP

