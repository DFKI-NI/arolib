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
 
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <random>
#include <string>
#include <cmath>

#include "arolib/cartography/gridmap_numeric.hpp"
#include "arolib/types/point.hpp"
#include "arolib/types/polygon.hpp"

using namespace arolib;

BOOST_AUTO_TEST_SUITE(test_gridmap_numeric)
typedef gridmap::NumericGridmap<float> FloatGrid_t;

boost::filesystem::path getOutputDir(){
    auto out_dir = boost::filesystem::temp_directory_path() / "arolib" / "test" / "test_gridmap_numeric";
    boost::filesystem::create_directories(out_dir);
    return out_dir;
}

FloatGrid_t get_grid(double minX, double maxX, double minY, double maxY, double stepsize = 1.0, float cellval = 0.0)
{
    FloatGrid_t grid;
    grid.createGrid(minX, maxX, minY, maxY, stepsize, cellval);
    return grid;
}

void randomize_grid(FloatGrid_t &grid)
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<> dist(0.0, 1.0);

    for (double x = grid.getMinPointX(); x < grid.getMaxPointX(); ++x)
    {
        for (double y = grid.getMinPointY(); y < grid.getMaxPointY(); ++y)
        {
            arolib::Point p(x, y);
            grid.setValue(p, dist(rng));
        }
    }
}

bool get_grid_sum(FloatGrid_t &grid, float &result)
{
    Polygon boundary_poly = grid.getGridPolygon();
    bool error = false;
    result = grid.getPolygonComputedValue(boundary_poly, arolib::gridmap::NumericGridmap<float>::ComputedValueType::SUM, true, &error);
    return error;
}

void saveGridHD(const FloatGrid_t& grid, const Polygon& poly, float poly_val, float edges_val, const std::string& file_name, unsigned int scale = 50){
    auto grid_HD = grid;
    grid_HD.scaleResolution(scale);
    grid_HD.setPolygon(poly, poly_val);
    for(size_t x = 0 ; x < grid.getSizeX() ; ++x){
        for(size_t y = 0 ; y < grid.getSizeY() ; ++y){
            Polygon cellPoly;
            grid.getCellPolygon(x, y, cellPoly);
            for(size_t i = 0 ; i+1 < cellPoly.points.size() ; ++i)
                grid_HD.setLine(cellPoly.points.at(i), cellPoly.points.at(i+1), 0.0, edges_val);
        }
    }
    grid_HD.saveGridAsGeoTiff( (getOutputDir() / file_name).string() + ".tif");
}

BOOST_AUTO_TEST_CASE(save_and_load_test)
{
    FloatGrid_t grid = get_grid(-100, 100, -100, 100);
    randomize_grid(grid);

    std::string png_filename = ( getOutputDir() / "grid.png" ).string();
    BOOST_REQUIRE(grid.saveGridAsPNG(png_filename));
    FloatGrid_t grid_png;
    BOOST_REQUIRE(grid_png.readGridFromPNG(png_filename));
    BOOST_CHECK(grid.equalGeometry(grid_png));
    BOOST_CHECK(grid.equals(grid_png, [](const float& val1, const float& val2)->bool{
        return std::fabs(val1-val2) < 1.001/255;
    }));

    BOOST_CHECK(grid_png.setPointLimits_min(-100, -100, 1.0));

    std::string tif_filename = ( getOutputDir() / "grid.tif" ).string();
    BOOST_REQUIRE(grid.saveGridAsGeoTiff(tif_filename));
    FloatGrid_t grid_tif;
    BOOST_REQUIRE(grid_tif.readGridFromGeoTiff(tif_filename));
    BOOST_CHECK(grid.operator==(grid_tif));
}

BOOST_AUTO_TEST_CASE(test_getValue)
{
    FloatGrid_t grid = get_grid(-100, 100, -100, 100);
    randomize_grid(grid);

    BOOST_REQUIRE(grid.setLayoutTolerance(0.1));

    std::map<Point, float> points_in_grid{
        {Point(0, 0), -0.5},
        {Point(-100, -100), -0.7},
        {Point(100, 100), 0.3},
        {Point(-100.1, 0), 0.3},
        {Point(0, 100.1), 0.3}};

    std::map<Point, float> points_outside_grid{
        {Point(100.11, 0), -0.5},
        {Point(-100, -100.1000001), -0.7},
        {Point(200, 100), 0.3},
        {Point(-9999, 0), 0.3},
        {Point(110, 99), 0.3}};

    for (const auto &pair : points_in_grid)
        BOOST_CHECK(grid.setValue(pair.first, pair.second));

    for (const auto &pair : points_outside_grid)
        BOOST_CHECK(!grid.setValue(pair.first, pair.second));

    for (const auto &pair : points_in_grid)
    {
        BOOST_CHECK(grid.checkPointInRange(pair.first));
        bool error(false);
        BOOST_CHECK_SMALL(grid.getValue(pair.first, &error) - pair.second, 1e-6f);
        BOOST_CHECK(!error);
    }

    for (const auto &pair : points_outside_grid)
    {
        BOOST_CHECK(!grid.checkPointInRange(pair.first));
        bool error(false);
        grid.getValue(pair.first, &error);
        BOOST_CHECK(error);
    }
}

BOOST_AUTO_TEST_CASE(test_polygon)
{
    FloatGrid_t grid;
    float cellval = 0.0;
    grid.createGrid(-10, 10, -10, 10, 1.0, &cellval);
    float poly_area;
    BOOST_CHECK(!get_grid_sum(grid, poly_area));
    BOOST_CHECK_SMALL(poly_area - 0.0, 1e-6);

    std::vector<Point> polygon_points = {
        Point(0, 0),
        Point(8, 4),
        Point(-5.5, 4),
        Point(-5.5, -4),
        Point(8, -4),
        Point(0, 0)};

    // Tiles on the boundary:   8+4     = 12
    // Tiles in boundary:       40+24   = 64
    
    Polygon p1;
    p1.points = polygon_points;
    
    bool error = false;
    BOOST_CHECK(grid.updatePolygonProportionally(p1, 1.0));
    
    // Check values on boundary
    BOOST_CHECK_CLOSE(grid.getValue(Point(-1, 3), &error), 1.0, 1);
    BOOST_CHECK_CLOSE(grid.getValue(Point(0.5, 0.5), &error), 0.75, 1);
    BOOST_CHECK_CLOSE(grid.getValue(Point(-5.5, 3.5), &error), 0.5, 1);
    BOOST_CHECK_CLOSE(grid.getValue(Point(-5.5, -3.5), &error), 0.5, 1);
    BOOST_CHECK_CLOSE(grid.getValue(Point(7.5, -3.5), &error), 0.25, 1);
    BOOST_CHECK_CLOSE(grid.getValue(Point(-1, -3), &error), 1.0, 1);

    BOOST_CHECK(!get_grid_sum(grid, poly_area));
    BOOST_CHECK_CLOSE(poly_area, 76, 1);

//    saveGridHD(grid, p1, 2, 0, "grid_HD_updatePoly", 50);
}

BOOST_AUTO_TEST_CASE(test_common_functions)
{
    const double minX(0), maxX(100), minY(-300), maxY(100);
    const float val = 0.5;
    FloatGrid_t grid;
    for (const auto &cellsize : std::vector<float>{0.3, 1.2})
    {
        grid.createGrid(minX, maxX, minY, maxY, cellsize, &val);

        bool error = false;
        BOOST_CHECK_SMALL(grid.getCellsize() - cellsize, 1e-6);
        BOOST_CHECK_SMALL(grid.getCellArea() - cellsize * cellsize, 1e-6);

        double realmaxX = std::ceil((maxX - minX) / cellsize) * cellsize + minX;
        double realmaxY = std::ceil((maxY - minY) / cellsize) * cellsize + minY;
        BOOST_CHECK_EQUAL(grid.getLowerLeftCorner(), Point(minX, minY));
        BOOST_CHECK_EQUAL(grid.getUpperRightCorner(), Point(realmaxX, realmaxY));
        BOOST_CHECK_EQUAL(grid.getGridArea(&error), (realmaxX - minX) * (realmaxY - minY));
        BOOST_CHECK(!error);
    }
}

BOOST_AUTO_TEST_CASE(test_setGeometry)
{
    double line_width = 1.0;
    Point p1(-10,-9);
    Point p2(10, 1);
    Polygon rect = geometry::createRectangleFromLine(p1, p2, line_width);

    auto grid1 = get_grid(-10, 10, -10, 10, 1.0, 0.0);
    BOOST_CHECK(grid1.setLine(p1,p2,line_width, 1.0));

    auto grid2 = get_grid(-10, 10, -10, 10, 1.0, 0.0);
    BOOST_CHECK(grid2.setLine(p2,p1,line_width, 1.0));

    auto grid3 = get_grid(-10, 10, -10, 10, 1.0, 0.0);
    BOOST_CHECK(grid3.setPolygon(rect, 1.0));

    BOOST_CHECK(grid1.operator==(grid2));
    BOOST_CHECK(grid1.operator==(grid3));
    bool error = false;
    BOOST_CHECK_SMALL(grid1.getValue(Point(-9.5, -8.5), &error) - 1.0, 1e-6);
    BOOST_CHECK_SMALL(grid1.getValue(Point(-9.5, -6.5), &error) - 0.0, 1e-6);
    BOOST_CHECK_SMALL(grid1.getValue(Point(0.5, -1.5), &error) - 0.0, 1e-6);
    BOOST_CHECK_SMALL(grid1.getValue(Point(0.5, -3.5), &error) - 1.0, 1e-6);
    BOOST_CHECK_SMALL(grid1.getValue(Point(9.5, 2.5), &error) - 0.0, 1e-6);
    BOOST_CHECK_SMALL(grid1.getValue(Point(9.5, 1.5), &error) - 1.0, 1e-6);

//    saveGridHD(grid1, rect, 2, 0, "grid1_HD_setLine", 50);
//    saveGridHD(grid2, rect, 2, 0, "grid2_HD_setLine", 50);
//    saveGridHD(grid3, rect, 2, 0, "grid3_HD_setPoly", 50);
}

BOOST_AUTO_TEST_CASE(test_updateGeometryProportionally)
{
    double line_width = 1.0; 
    Point p1(-10,-9);
    Point p2(10, 1);
    Polygon rect = geometry::createRectangleFromLine(p1, p2, line_width);

    auto grid1 = get_grid(-10, 10, -10, 10, 1.0, 0.0);
    BOOST_CHECK(grid1.updateLineProportionally(p1,p2,line_width, 1.0));

    auto grid2 = get_grid(-10, 10, -10, 10, 1.0, 0.0);
    BOOST_CHECK(grid2.updateLineProportionally(p2,p1,line_width, 1.0));

    auto grid3 = get_grid(-10, 10, -10, 10, 1.0, 0.0);
    BOOST_CHECK(grid3.updatePolygonProportionally(rect, 1.0));

    BOOST_CHECK(grid1.operator==(grid2));
    BOOST_CHECK(grid1.equals(grid3, [](const float& val1, const float& val2)->bool{
        return std::fabs(val1-val2) < 1e-3;
    }));
    bool error = false;
    BOOST_CHECK_CLOSE(grid1.getValue(Point(-9.5, -8.5), &error), 0.8, 1); // 1% difference allowed
    BOOST_CHECK_SMALL(grid1.getValue(Point(-9.5, -6.5), &error), 1e-6f);
    BOOST_CHECK_SMALL(grid1.getValue(Point(0.5, -1.5), &error), 1e-6f);
    BOOST_CHECK_CLOSE(grid1.getValue(Point(-0.5, -3.5), &error), 0.31, 1);

//    saveGridHD(grid1, rect, 2, 0, "grid1_HD_updateLine", 50);
//    saveGridHD(grid2, rect, 2, 0, "grid2_HD_updateLine", 50);
//    saveGridHD(grid3, rect, 2, 0, "grid3_HD_updatePoly", 50);
}
BOOST_AUTO_TEST_SUITE_END()
