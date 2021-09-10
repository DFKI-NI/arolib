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
 
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <random>
#include <string>
#include <cmath>
#include <functional>

#include "arolib/cartography/gridmap.hpp"
#include "arolib/cartography/gridmap_numeric.hpp"
#include "arolib/types/point.hpp"
#include "arolib/types/polygon.hpp"

using namespace arolib;

boost::filesystem::path getOutputDir()
{
    auto out_dir = boost::filesystem::temp_directory_path() / "arolib" / "test" / "test_gridmap";
    boost::filesystem::create_directories(out_dir);
    return out_dir;
}

BOOST_AUTO_TEST_SUITE(test_gridmap)
template <typename T>
gridmap::Gridmap<T> get_grid(double minX, double maxX, double minY, double maxY, T cellval, double stepsize = 1.0)
{
    gridmap::Gridmap<T> grid;
    grid.createGrid(minX, maxX, minY, maxY, stepsize, cellval);
    return grid;
}

template <typename T>
void randomize_grid(gridmap::Gridmap<T> &grid, std::function<T()> random_sampling_function)
{
    for (double x = grid.getMinPointX(); x < grid.getMaxPointX(); ++x)
    {
        for (double y = grid.getMinPointY(); y < grid.getMaxPointY(); ++y)
        {
            arolib::Point p(x, y);
            grid.setValue(p, random_sampling_function());
        }
    }
}

std::random_device g_dev;
std::mt19937 g_rng(g_dev());

template <typename T>
T get_random_int(T min = 0, T max = 100)
{
    std::uniform_int_distribution<T> int_dist(min, max);
    return int_dist(g_rng);
}

std::string get_random_string(int n_permutations = 10)
{
    return std::string(get_random_int<int>(0, n_permutations), '*');
}

template <typename T>
T get_random_float(T min = 0.0, T max = 1.0)
{
    std::uniform_real_distribution<T> float_dist(min, max);
    return float_dist(g_rng);
}

void randomize_grid(gridmap::Gridmap<std::string> &grid)
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<> int_dist(0, 100);
    std::vector<std::string> string_vals;
    string_vals.push_back("0");
    for (int i = 1; i <= 100; i++)
        string_vals.push_back(string_vals[i - 1] + std::to_string(i));

    randomize_grid<std::string>(grid, [&]() { return string_vals[int_dist(rng)]; });
}

void randomize_grid(gridmap::Gridmap<float> &grid)
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<> float_dist(0.0, 1.0);

    randomize_grid<float>(grid, [&]() { return float_dist(rng); });
}

template <typename T>
bool compare_floats(T a, T b)
{
    T tolerance = 1e-6f;
    auto result = (std::fabs(a - b) <= tolerance);
    if (!result)
        std::cout << "Failed comparison: " << a << " != " << b << std::endl;
    return result;
}

template <typename T>
bool compare_default(T a, T b)
{
    bool result = (a == b);
    if (!result)
        std::cout << "Failed comparison: " << a << " != " << b << std::endl;
    return result;
}

template <typename T>
void test_get_value(T default_val, std::function<T()> random_sampling_function, std::function<void(T a, T b)> comparison_function)
{
    gridmap::Gridmap<T> grid_png, grid_tif;
    auto grid = get_grid<T>(-100, 100, -100, 100, default_val);
    randomize_grid(grid, random_sampling_function);

    BOOST_REQUIRE(grid.setLayoutTolerance(0.1));

    std::map<Point, T> points_in_grid{
        {Point(0, 0), random_sampling_function()},
        {Point(-100, -100), random_sampling_function()},
        {Point(100, 100), random_sampling_function()},
        {Point(-100.1, 0), random_sampling_function()},
        {Point(0, 100.1), random_sampling_function()}};

    std::map<Point, T> points_outside_grid{
        {Point(100.11, 0), random_sampling_function()},
        {Point(-100, -100.1000001), random_sampling_function()},
        {Point(200, 100), random_sampling_function()},
        {Point(-9999, 0), random_sampling_function()},
        {Point(110, 99), random_sampling_function()}};

    for (const auto &pair : points_in_grid)
        BOOST_CHECK(grid.setValue(pair.first, pair.second));

    for (const auto &pair : points_outside_grid)
        BOOST_CHECK(!grid.setValue(pair.first, pair.second));

    for (const auto &pair : points_in_grid)
    {
        BOOST_CHECK(grid.checkPointInRange(pair.first));
        bool error(false);
        auto a = grid.getValue(pair.first, &error);
        comparison_function(a, pair.second);
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

BOOST_AUTO_TEST_CASE(test_getValue)
{
    test_get_value<float>(
        0.5, []() { return get_random_float<float>(); }, compare_floats<float>);
    test_get_value<double>(
        0.5, []() { return get_random_float<float>(); }, compare_floats<double>);
    test_get_value<int>(
        0, []() { return get_random_int<int>(); }, compare_default<int>);
    test_get_value<std::string>(
        "☯", []() { return get_random_string(); }, compare_default<std::string>);
}

template <typename T>
void saveGridHD(const gridmap::Gridmap<T> &grid, const Polygon &poly, T default_val, const std::string &file_name, unsigned int scale = 50)
{
    gridmap::NumericGridmap<float> grid_HD(grid.getLayout());

    for (size_t x = 0; x < grid.getSizeX(); ++x)
    {
        for (size_t y = 0; y < grid.getSizeY(); ++y)
        {
            if (grid.hasValue(x, y) && grid.getValue(x, y) != default_val)
                grid_HD.setValue(x, y, 1.0);
            else
                grid_HD.setValue(x, y, 0.0);
        }
    }

    grid_HD.scaleResolution(scale);
    grid_HD.setPolygon(poly, 2.0);
    for (size_t x = 0; x < grid.getSizeX(); ++x)
    {
        for (size_t y = 0; y < grid.getSizeY(); ++y)
        {
            Polygon cellPoly;
            grid.getCellPolygon(x, y, cellPoly);
            for (size_t i = 0; i + 1 < cellPoly.points.size(); ++i)
                grid_HD.setLine(cellPoly.points.at(i), cellPoly.points.at(i + 1), 0.0, 0.0);
        }
    }
    grid_HD.saveGridAsGeoTiff((getOutputDir() / file_name).string() + ".tif");
}

template <typename T>
void test_polygon_fun(std::function<bool(T a, T b)> comparison_function, T default_val = 0, T new_val = 1)
{
    const std::vector<Point> polygon_points = {
        Point(0, 0),
        Point(8, 4),
        Point(-5.5, 4),
        Point(-5.5, -4),
        Point(8, -4),
        Point(0, 0)};

    Polygon p1;
    p1.points = polygon_points;
    bool error = false;

    // === without overlap threshold ===
    std::cout << "Check with overlapThreshold = -1" << std::endl;
    {
        gridmap::Gridmap<T> grid;
        grid.createGrid(-10, 10, -10, 10, 1.0, &default_val);
        // Tiles completely in boundary:    20*2 + 12*2 = 64
        // Tiles partially in boudnary:     4*2 + 8*2   = 24
        // 64 + 24 = 88
        float overlapThreshold = -1;
        BOOST_CHECK(grid.setPolygon(p1, new_val, overlapThreshold));

        //saveGridHD(grid, p1, default_val, std::string("grid_HD_setPoly__") + typeid(T).name(), 50);

        // Check values inside
        BOOST_CHECK(comparison_function(grid.getValue(Point(0.5, 1), &error), new_val));
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(-2, 2), &error), new_val));
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(-2, -2), &error), new_val));
        BOOST_CHECK(!error);

        // Check values on boundary
        BOOST_CHECK(comparison_function(grid.getValue(Point(0.5, 0.5), &error), new_val));
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(-5.5, 3.5), &error), new_val));
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(-5.5, -3.5), &error), new_val));
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(7.5, -3.5), &error), new_val));
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(7.5, 3.5), &error), new_val));
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(3.5, 1.5), &error), new_val));
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(2.5, -0.5), &error), default_val));
        BOOST_CHECK(!error);
    }

    // === with overlap threshold ===
    std::cout << "Check with overlapThreshold = 0.5" << std::endl;
    {
        gridmap::Gridmap<T> grid;
        grid.createGrid(-10, 10, -10, 10, 1.0, &default_val);
        // Tiles completely in boundary:    20*2 + 12*2 = 64
        // Tiles partially in boudnary:     4*2 + 8*2   = 24
        // 64 + 24 = 88
        float overlapThreshold = 0.5;
        BOOST_CHECK(grid.setPolygon(p1, new_val, overlapThreshold));

        //saveGridHD(grid, p1, default_val, std::string("grid_HD_setPoly__") + typeid(T).name(), 50);

        // Check values inside
        BOOST_CHECK(comparison_function(grid.getValue(Point(0.5, 1), &error), new_val));
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(-2, 2), &error), new_val));
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(-2, -2), &error), new_val));
        BOOST_CHECK(!error);

        // Check values on boundary
        BOOST_CHECK(comparison_function(grid.getValue(Point(0.5, 0.5), &error), new_val));
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(-5.5, 3.5), &error), new_val)); // exactly half cell
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(-5.5, -3.5), &error), new_val)); // exactly half cell
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(7.5, -3.5), &error), default_val));
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(7.5, 3.5), &error), default_val));
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(3.5, 1.5), &error), default_val));
        BOOST_CHECK(!error);
        BOOST_CHECK(comparison_function(grid.getValue(Point(2.5, -0.5), &error), default_val));
        BOOST_CHECK(!error);
    }
}

BOOST_AUTO_TEST_CASE(test_polygon)
{
    std::cout << "Run test_polygon_fun<float> ..." << std::endl;
    test_polygon_fun<float>(compare_floats<float>);
    std::cout << "Run test_polygon_fun<double> ..." << std::endl;
    test_polygon_fun<double>(compare_floats<double>);
    std::cout << "Run test_polygon_fun<int> ..." << std::endl;
    test_polygon_fun<int>(compare_default<int>);
    std::cout << "Run test_polygon_fun<std::string> ..." << std::endl;
    std::string initial_string("☯"), new_string("🖥🎮");
    test_polygon_fun<std::string>(compare_default<std::string>, initial_string, new_string);
}

template <typename T>
void test_common_functions_function(T val)
{
    const double minX(0), maxX(100), minY(-300), maxY(100);
    gridmap::Gridmap<T> grid;
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

BOOST_AUTO_TEST_CASE(test_common_functions)
{
    test_common_functions_function<float>(0.33);
    test_common_functions_function<std::string>("☯");
}

template <typename T>
void test_line_fun(const std::function<bool(T a, T b)> comparison_function, const T default_val = 0, const T new_val = 1)
{
    const double line_width = 2.0;
    const Point p1(-10, -9);
    const Point p2(10, 1);
    const Polygon rect = geometry::createRectangleFromLine(p1, p2, line_width);
    bool error = false;

    // === without overlap threshold ===
    std::cout << "Check with overlapThreshold = -1" << std::endl;
    {
        gridmap::Gridmap<T> grid, grid2, grid3;
        grid.createGrid(-10, 10, -10, 10, 1.0, &default_val);
        grid2 = grid;
        grid3 = grid;

        float overlapThreshold = -1;
        BOOST_CHECK(grid.setLine(p1, p2, line_width, new_val, overlapThreshold));
        BOOST_CHECK(grid2.setLine(p2, p1, line_width, new_val, overlapThreshold));
        BOOST_CHECK(grid3.setPolygon(rect, new_val, overlapThreshold));

        BOOST_CHECK(grid.operator==(grid2));
        BOOST_CHECK(grid.operator==(grid3));

        // Check values inside
        BOOST_CHECK(comparison_function(grid.getValue(Point(-9.5, -6.5), &error), default_val));
        BOOST_CHECK(!error); // empty cell
        BOOST_CHECK(comparison_function(grid.getValue(Point(0.5, -1.5), &error), default_val));
        BOOST_CHECK(!error); // empty cell
        BOOST_CHECK(comparison_function(grid.getValue(Point(9.5, 2.5), &error), default_val));
        BOOST_CHECK(!error); // empty cell

        BOOST_CHECK(comparison_function(grid.getValue(Point(-9.5, -8.5), &error), new_val));
        BOOST_CHECK(!error); // full cell
        BOOST_CHECK(comparison_function(grid.getValue(Point(0.5, -3.5), &error), new_val));
        BOOST_CHECK(!error); // full cell

        BOOST_CHECK(comparison_function(grid.getValue(Point(7.5, 0.5), &error), new_val));
        BOOST_CHECK(!error); // partial cell > 0.5
        BOOST_CHECK(comparison_function(grid.getValue(Point(10.5, 0.5), &error), new_val));
        BOOST_CHECK(!error); // partial cell > 0.5 outside bounds
        BOOST_CHECK(comparison_function(grid.getValue(Point(0.5, -2.5), &error), new_val));
        BOOST_CHECK(!error); // partial cell < 0.5
        BOOST_CHECK(comparison_function(grid.getValue(Point(-2.5, -6.5), &error), new_val));
        BOOST_CHECK(!error); // partial cell < 0.5
    }

    // === with overlap threshold ===
    std::cout << "Check with overlapThreshold = 0.5" << std::endl;
    {
        gridmap::Gridmap<T> grid, grid2, grid3;
        grid.createGrid(-10, 10, -10, 10, 1.0, &default_val);
        grid2 = grid;
        grid3 = grid;

        const float overlapThreshold = 0.5;
        BOOST_CHECK(grid.setLine(p1, p2, line_width, new_val, overlapThreshold));
        BOOST_CHECK(grid2.setLine(p2, p1, line_width, new_val, overlapThreshold));
        BOOST_CHECK(grid3.setPolygon(rect, new_val, overlapThreshold));

        BOOST_CHECK(grid.operator==(grid2));
        BOOST_CHECK(grid.operator==(grid3));

        // Check values inside
        BOOST_CHECK(comparison_function(grid.getValue(Point(-9.5, -6.5), &error), default_val));
        BOOST_CHECK(!error); // empty cell
        BOOST_CHECK(comparison_function(grid.getValue(Point(0.5, -1.5), &error), default_val));
        BOOST_CHECK(!error); // empty cell
        BOOST_CHECK(comparison_function(grid.getValue(Point(9.5, 2.5), &error), default_val));
        BOOST_CHECK(!error); // empty cell

        BOOST_CHECK(comparison_function(grid.getValue(Point(-9.5, -8.5), &error), new_val));
        BOOST_CHECK(!error); // full cell
        BOOST_CHECK(comparison_function(grid.getValue(Point(0.5, -3.5), &error), new_val));
        BOOST_CHECK(!error); // full cell

        BOOST_CHECK(comparison_function(grid.getValue(Point(7.5, 0.5), &error), new_val));
        BOOST_CHECK(!error); // partial cell > 0.5
        BOOST_CHECK(comparison_function(grid.getValue(Point(10.5, 0.5), &error), new_val));
        BOOST_CHECK(!error); // partial cell > 0.5 outside bounds
        BOOST_CHECK(comparison_function(grid.getValue(Point(0.5, -2.5), &error), default_val));
        BOOST_CHECK(!error); // partial cell < 0.5
        BOOST_CHECK(comparison_function(grid.getValue(Point(-2.5, -6.5), &error), default_val));
        BOOST_CHECK(!error); // partial cell < 0.5
    }
}

BOOST_AUTO_TEST_CASE(test_line)
{
    std::cout << "Run test_line_fun<float> ..." << std::endl;
    test_line_fun<float>(compare_floats<float>);
    std::cout << "Run test_line_fun<double> ..." << std::endl;
    test_line_fun<double>(compare_floats<double>);
    std::cout << "Run test_line_fun<int> ..." << std::endl;
    test_line_fun<int>(compare_default<int>);
    std::cout << "Run test_line_fun<std::string> ..." << std::endl;
    std::string initial_string("☯"), new_string("🖥🎮");
    test_line_fun<std::string>(compare_default<std::string>, initial_string, new_string);
}

BOOST_AUTO_TEST_SUITE_END()
