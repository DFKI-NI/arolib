# arolib

[[_TOC_]]

`arolib` is a c++ library for field geometry processing and (global) route planning of primary machines (e.g. harvesters) and service units (e.g. overload/transport vehicles) participating in arable farming operations such as harvesting.



<img src="documentation/images/test_field_1.jpg" alt="Processed test field" width="850" />

<img src="documentation/images/test_field_1__routes_TO.jpg" alt="Planned routes" width="850" />





## Overview



### Modules

`arolib` currently consists of the following modules:

- `types`: defines the basic types
- `components`: higher level components used for the overall planning process
- `geometry`: includes classes and functions for geometry processing
- `planning`: Includes the components used for route planning
- `cartography`: includes the definitions and functionality related to gridmaps
- `io`: input/output components to read and write arolib types in files.
- `misc`: common classes and functions
- `processing`: post-planning processing functions like route smoothing



### Main types

- `Point`: Point/coordinates
- `Polygon`: Simple polygon 
- `Linestring`: Line-string of points
- `Pose2D`: A 2D Cartesian pose containing a position and an angle (with respect to the positive x axis)
- `Machine`: Models a machine; contains properties of the machine such as model, type, dimensions, etc.
- `MachineDynamicInfo`: Dynamic properties (state) of a single machine instance, including location and bunker mass
- `Field`: Represents a field consisting of one or more subfields. A valid field will have a boundary and at least one valid subfield (currently, only one subfield is supported for planning)
- `Subfield`: Represents a part of a `Field`; contains properties such as boundaries, track reference lines, access points, resource points, tracks, etc. A valid subfield has a outer boundary and at least one reference line for (inner field) tracks.
- `Headlands`: The headland(s) of one subfield; each subfield has either a `CompleteHeadland` which surrounds the whole subfield or a number of `PartialHeadland` located at some sides.
- `Track`: defines a track (lane)
- `FieldAccessPoint`: Field entry/exit point
- `ResourcePoint`: Models a process resource point. Currently, only unloading facilities are supported (e.g. silos, clamps). 
- `OutFieldInfo`: Contains information of activities/transit done outside of the field (e.g. transit between field access points and resource points located outside of the field)
- `RoutePoint`: A point in a route containing spatio-temporal and operation-specific information.
- `Route`: Route for a specific machine.
- `DirectedGraph::Graph`: Directed graph used for path planning.
- `GridMap<T>`: models a gridmap with cell values of type T. Currently, the planners normally use `ArolibGrid_t := GridMap<float>` 
- `IEdgeCostCalculator`: Interface (abstract class) used by the planners to obtain edge costs. 
- `IEdgeSpeedCalculator`: Interface (abstract class) used by the planners to compute the speed of a machine driving between two points. 
- `IEdgeMassCalculator`: Interface (abstract class) used by the planners to obtain the amount of mass under a rectangle. 
- `IInfieldTracksConnector`: Interface (abstract class) used by the planners to obtain the path to connect a given pose  with an inner-field track taking into account the inner-field boundaries. 
- `ITrackSequencer`: Interface (abstract class) used to select the sequence in which the inner-field tracks will be visited by the primary machines for field coverage.
- `Logger`: Logging class. 
- `Unit`: Represents a unit (e.g  m^2, m/s, etc) supported in the library.



### Main terms

- Headland (HL): area of the subfield used by the machines for turning. Can be an area surrounding the main area of the subfield, or could be located only at some sides of the subfield.

- Inner-field (IF): main area of the subfield (the subfield excluding the headland)

- Working group: set of machines participating in the operation

- Primary machine: types of machines who perform the main work on the field (e.g., harvesters, sprayers, etc.)

- Service unit: types of machines that assist the primary machines (e.g., overload/transport vehicles)

- Capacitated machine: machine that contains a bunker to store material.

  



### Documentation

To generate the documentation for the C++ code, run `doxygen` in the base directory:
```
doxygen Doxyfile
```
The documentation will be generated in the `/docs` subdirectory. You can start browsing from `/docs/html/index.html`.



## Set up



### Requirements

1. Install dependencies

   * On **Linux** the dependencies can simply be installed by running the script '`scripts/install_deps.sh`'. 



### Variables

The following variables are used by the package scripts:

- `AROLIB_AROLIB_ROOT`: directory where the source code is located. If not set, many scripts will set the default relative to the script's directory (hence the location of the scripts is important)
- `AROLIB_AROLIB_BUILD_PATH` : directory where the built files will be located. Default: `$AROLIB_AROLIB_ROOT/build`
- `AROLIB_AROLIB_INSTALL_PATH`: directory where the installation files will be located. 



### Building & Installation

Open a new terminal. Before building,  set the aforementioned variables if/as desired (`AROLIB_AROLIB_ROOT`, `AROLIB_AROLIB_BUILD_PATH` , `AROLIB_AROLIB_INSTALL_PATH`) :

Currently there are 3 different ways to build/install arolib:

1. Using the available script `scripts/arolib_make`

   ```plaintext
    cd $AROLIB_AROLIB_ROOT
    ./scripts/arolib_make
   ```

   

   Notes:

   - If `AROLIB_AROLIB_ROOT` is not set, it will be set relatively based on the absolute path of the script (hence the location of the script is important)

   - If `AROLIB_AROLIB_BUILD_PATH` is not set, it will be set to the default path `$AROLIB_AROLIB_ROOT/build`

   - If `AROLIB_AROLIB_INSTALL_PATH` is not set, it will be set to the default path `$HOME/arolib/install/arolib`

   - Run the script `./scripts/arolib_make_debug` to build/install the debug version.

     

2. Using `make` (`Makefile`)

   * Run `make build` to build arolib. The headers will be copied to `AROLIB_AROLIB_INSTALL_PATH` and the shared libraries will be copied to `AROLIB_AROLIB_BUILD_PATH`.

   * Run `make install` to install the arolib library to the system default locations (on Linux): The headers will be copied to `/usr/local/include/arolib` and the libs will be copied to `/usr/local/lib`.

   ```plaintext
    cd $AROLIB_AROLIB_ROOT
    make build
    make install
   ```

   

   Notes:

   - If `AROLIB_AROLIB_ROOT` is not set, it will be set relatively based on the absolute path of the `Makefile` (hence the location of the `Makefile` is important)

   - If `AROLIB_AROLIB_BUILD_PATH` is not set, it will be set to the default path `$AROLIB_AROLIB_ROOT/build`

   - If `AROLIB_AROLIB_INSTALL_PATH` is not set, it will be set to the default path `$AROLIB_AROLIB_ROOT/install`

     

3. Docker alternative using `make` (`Makefile`)

   Alternatively, arolib can be built inside a Docker container.

   ```plaintext
    cd $AROLIB_AROLIB_ROOT
    sudo make docker/build_image/focal
    sudo -E make docker/build_arolib/focal
   ```

   

   Notes:

   * The option `-E` for `sudo` passes on the session's environment variables

     

### Testing

There are currently 3 `make` targets to run tests:
- `make test/units` will run the unit tests
- `make test/integration` will run the integration tests
- `make test` will run all tests



### Using the library

If the installation was not done in on of the system's default locations, the installation sub paths must be added to the system environmental variables. For this, the script `scripts/env_install_dir.sh` is available. This script receives one argument: the path where arolib was installed (without the final '/'). For example:

```
export AROLIB_AROLIB_INSTALL_PATH=$HOME/arolib/install/arolib
source scripts/env_install_dir.sh $AROLIB_AROLIB_INSTALL_PATH
```

*  Add '`source <arolib_arolib_src_directory>/scripts/env_install_dir.sh <arolib_install_path>`'' to your '`~/.bashrc`' if desired

The script `scripts/env.h` can also be used for this purpose. This script will set a default value for `AROLIB_AROLIB_INSTALL_PATH` (if not set before), and add the respective paths to the system environmental variables.

```
# export AROLIB_AROLIB_INSTALL_PATH=<arolib_install_path> ##optional
source $AROLIB_AROLIB_ROOT/scripts/env.sh
```

*  Add '`source <arolib_arolib_src_directory>/scripts/env.sh `'' to your '`~/.bashrc`' if desired (`AROLIB_AROLIB_INSTALL_PATH` must be set before)




## Usage



### Planning with arolib



#### General input parameters

In order to plan with the `arolib` components, some parameters must be set by the user. This parameters are used by one or more of the planning components. All geometries must be given in the Cartesian coordinate system (geodetic coordinates must be transformed/projected accordingly, e.g. by using the available `CoordTransformer`)

* **Field**: 

  A (virgin) field must be set, containing the outer boundary and at least one subfield (currently, only one subfield is supported). The subfields must contain the outer boundary, at least one reference linestring for the inner-field tracks, access points, and resource points. The field can be initialized directly by the user, or read using the i/o methods (xml, kml, hdf5) offered by `arolib` (the files must follow the `arolib` data structure).

* **Working group**: 

  The group of machines that will participate in the operation must be given. At the moment, two types of output-material-flow operations are suported:

  * One capacitated primary machine which works the field and transports the material to the unloading points.
  * One non-capacitated primary machine that works the field assited by one or more transport vehicles (OLVs)

* **Out-of-field information**: 

  Information of the operations done outside of the field must be given. This includes:

  * Travel distance and durations between access points and (external) resource points (general or specific for a machine)
  * Travel distance and durations between (external) resource points and access points (general or specific for a machine)
  * Travel distance and durations between access points (general or specific for a machine)
  * Travel distance and durations between the current location of each machine and each access point (if the machine is outside the field)
  * Default unloading durations for the resource points  (general or specific for a machine)

* **Machine's current states**: 
  * The currents state (location, bunker mass) of the participating machines

* **Edge calculators**:  

  Several planning components use edge calculators to compute some values related to a segment driven by a machine. Some calculators are available in `arolib`, however, the user can define specific calculators following the respective interfaces.

  * **Working-speed calculator:** This calculator must inherit from `IEdgeSpeedCalculator`. It is used by some planners to compute the speed of the primary machines while working the field based on the machine and the (2-point) segment to be driven.
  * **Transit-speed calculator:** This calculator must inherit from `IEdgeSpeedCalculator`. It is used by some planners to compute the transit speed of machines inside the field.
  * **Mass calculator:** This calculator must inherit from `IEdgeMassCalculator`. It is used by some planners to compute the amount of mass under a rectangle given by a (2-point) segment and a width.
  * **Cost calculator:** This calculator must inherit from `IEdgeCostCalculator`. It is used by the A* path planners to compute the edge cost and heuristics. It can be defined based on the desired optimization criterion for the operation.

* **Grid-maps**:  

  Some planners or edge calculators might need some gridmaps to operate. For Instance, a mass calculator might need a field biomass gridmap to obtain the amount of mass in a certain region; or a cost-calculator might need a cost-map.

* **Inner-field track connectors** [optional]:

  Used by some planning components to compute the path to connect a given pose with an inner-field track taking into account the inner-field boundaries (e.g., to connect the track-ends of two different tracks). Some connectors are available in `arolib`, however, the user can define specific connectors following the `IInfieldTracksConnector` interface.

* **Inner-field track sequencer** [optional]:

  Used to select the sequence in which the inner-field tracks will be visited by the primary machines for field coverage. Some sequencers are available in `arolib`, however, the user can define specific sequencers following the `ITrackSequencer` interface. 

  

#### Planning steps

The following steps must be followed to plan the process in a virgin field based on the given input parameters. The main components used in the process are located in [components/](components/).

1. **Generate the field geometric representation:** 

   The first step is to generate the geometries of the subfield, namely the boundaries and tracks of the headland- and inner-field- regions. For this, the `FieldGeometryProcessor` component ([here](components/include/arolib/components/fieldgeometryprocessor.h)) is used. The component will generate the geometries based on the given `FieldGeometryProcessor::HeadlandParameters` and `FieldGeometryProcessor::InfieldParameters`, and the selected inner-field tracks' reference line. The component offers the option to generate the field geometries with a complete/surrounding headland (`processSubfieldWithSurroundingHeadland`) or with side/partial headlands (`processSubfieldWithSideHeadlands`).

2. **Build the initial field graph:**

   Based on the field geometries (boundaries, tracks, access points, resource points), out-of-field information and current states of the machines, the `GraphProcessor` component ([here](components/include/arolib/components/graphprocessor.h)) is used to build the graph that will be used by the path planners.

3. **Generate the base-route**

   Based on the generated geometries, the base-route for the primary machine is computed using the `BaseRoutesPlanner` component ([here](components/include/arolib/components/baseroutesplanner.h)). The base-route is the route that the primary machine would follow to cover the field without taking into account capacity constraints and overloading/unloading activities. This route is generated based on the given `PlannerParameters::PlannerParameters` , edge mass calculator, and edge speed calculators,  among other parameters. If not set, the planner will use the default inner-field track sequencer and inner-field tracks' connector to generate the routes.

4. **Update the graph with the base-route information**

   The `GraphProcessor` component ([here](components/include/arolib/components/graphprocessor.h)) is used to update vertex properties based on the computed base-routes, including the (initial) working timestamps.

5. **Plan the routes for the process**

   Finally, the routes for all machines are generated using the `FieldProcessPlanner` component ([here](components/include/arolib/components/fieldprocessplanner.h)), based on the given `FieldProcessPlanner::PlannerParameters`, the generated graph, the base-route, the given edge-cost calculator, the machine current states, and the types of the machines participating in the operation (i.e., the operation type).

6. **[Optional] Smoothen the routes**

   The routes can be smoothen based on the machines' turning radius using the `RouteSmoother` ([here](processing/include/arolib/processing/route_smoother.hpp)) provided in [processing/](processing/).

7. **[Optional]  Save data**

   The processed field, generated routes, planning parameters, grid-maps, and other objects/parameters can be saved using the methods provided in [io/](io/): [io_xml](io/include/arolib/io/io_xml.hpp), [io_kml](io/include/arolib/io/io_kml.hpp) (fields only), and [io_hdf5](io/include/arolib/io/io_hdf5.hpp) (fields only).

8. **[Optional]  Simulation**

   The `GPSSimulator` component ([here](components/include/arolib/components/gpssimulator.h)) can be used to simulate the computed routes.



### Examples

In [`examples/example_harvesting`](examples/example_harvesting) you can find a simple example demonstrating the usage of `AroLib` for the supported operations and headland types.



## Used libraries and other software

| Library                                                      | License                                                      |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| [boost](https://www.boost.org/)                              | [Boost Software License](https://www.boost.org/users/license.html) |
| [GDAL](https://gdal.org/index.html)                          | [MIT/X style license](https://gdal.org/license.html)         |
| [HDF5](https://hdfgroup.org/solutions/hdf5/)                 | [BSD-style](https://support.hdfgroup.org/ftp/HDF5/releases/COPYING) |
| [HighFive](https://github.com/BlueBrain/HighFive)            | [Boost Software License 1.0](https://github.com/BlueBrain/HighFive/blob/master/LICENSE) |
| [crypto++](https://www.cryptopp.com/)                        | [Boost Software License 1.0](https://www.cryptopp.com/License.txt) |
| [Dubins-Curves](https://github.com/AndrewWalker/Dubins-Curves) | [MIT](https://github.com/AndrewWalker/Dubins-Curves/blob/master/LICENSE.txt) |
| [Bezier](https://github.com/oysteinmyrmo/bezier)             | [MIT](https://github.com/oysteinmyrmo/bezier/blob/master/LICENSE.txt) |
| [Clipper](http://www.angusj.com/delphi/clipper.php)          | [Boost Software License 1.0](https://www.boost.org/LICENSE_1_0.txt) |
| [Spline](https://github.com/brainexcerpts/Spline)            | [MIT](https://github.com/brainexcerpts/Spline/blob/master/LICENSE) |
| [cmake](https://cmake.org)                                   | [BSD 3-clause License](https://cmake.org/licensing/)         |



## Funding information

`arolib` was developed in part within the following funded projects:

| Project                                                      | Sponsor                                                      | Funding code(s)                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ | -------------------------------- |
| [prospective.HARVEST](https://www.prospectiveharvest.de/de/startseite.html) | German Federal Ministry of Food and Agriculture ([BMEL](https://www.bmel.de/EN/Home/home_node.html)) | 2815700915                       |
| [SOILAssist](https://www.soilassist.de/en/)                  | German Federal Ministry of Education and Research ([BMBF](https://www.bmbf.de/bmbf/en/home/home_node.html)) | 031A563B / 031B0684B / 031B1065B |

2023: The DFKI Niedersachsen (DFKI NI) is sponsored by the Ministry of Science and Culture of Lower Saxony and the VolkswagenStiftung.

