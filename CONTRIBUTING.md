# Contributing

[[_TOC_]]

## Tests
This sections explains how to add unit and integration tests to `arolib`.

### Unit tests
A unit test a small part of the whole system (as isolated from the rest of the system as possible).
It makes sure that the individual parts serve their exact purpose.

The unit tests use the `Boost.Test` framework
Each module has a subdirectory `module/test`, which contains all the unit tests.
The `test` directories contain one `main.cpp` and a number of other `.cpp` files.
The `main.cpp` is compiled to an executable and the other `.cpp` files are included in the `main.cpp`.
Each `.cpp` file in the `test` directory (except `main.cpp`) defines one *test suite* and tests one part of the whole module.

To add a new test (in this example we want to test a class called `NEW` so the test will be called `test_NEW`):

1. Copy one of the existing `.cpp` files (e.g. `module/test/test_mock.cpp`) and give it a fitting name (here `module/test/test_NEW.cpp`)

2. In `module/test/test_NEW.cpp`: Enter the name of the new unit test suite 

    E.g. replace `BOOST_AUTO_TEST_SUITE(test_mock)` with `BOOST_AUTO_TEST_SUITE(test_NEW)`

3. Include the new `.cpp` file in the `module/test/main.cpp`

    E.g. add `#include "test_NEW.cpp` at the top

4. Add the new test suite to the `test_suits` list in `module/test/CMakeLists.txt`
    
    E.g. Replace the line `set(test_suits test_mock test_whatever)` with `set(test_suits test_mock test_whatever test_NEW)`

5. Add the test cases in `module/test/test_NEW.cpp` (see the `Boost.Test` documentation for that)

### Integration tests
Integration tests test several units together.
They make sure that the units can work together and can achieve their functional requirements.

The integration tests can currently be found in `/examples`.
Each example (= integration test) is compiled to one executable and executed in the test stage.
Test is passed when the executable runs without error.

To add a new integration test:
1. Copy one of the existing directories (e.g. `/examples/example1` -> `/examples/example_NEW`)
2. Replace all occurences of the name of the old example in `/examples/example_NEW` with the new name of the example
3. Integrate the new example into the project structure by adding `add_subdirectory(example_NEW)` into `/examples/CMakeLists.txt`
