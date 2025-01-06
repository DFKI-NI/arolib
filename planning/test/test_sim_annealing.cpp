/*
 * Copyright 2021-2025 DFKI GmbH
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

#include "arolib/planning/track_sequencing/simulated_annealing/simulated_annealing.hpp"

BOOST_AUTO_TEST_SUITE(test_sim_annealing)
BOOST_AUTO_TEST_CASE(test_sim_annealing_1)
{
    using namespace arolib;

    SimulatedAnnealing<float> sa;

    auto tf = [](size_t k){
        return std::exp( -0.1 * k );
    };

    class StateGen : public SimulatedAnnealing<float>::IStateGenerator{
    public:

        static double energyFunction(float x){
            return std::sin(x) * std::exp(5-0.2*x);
        }

        SimulatedAnnealing<float>::ResultState getState(const float& , double) override{
            float newState = gen_random_double(0, 30);
            return std::make_unique< std::pair<float, double> >( std::make_pair( newState, energyFunction(newState) ) );
        }
    };


    float start = 6;
    double startEnergy = StateGen::energyFunction(start);

    StateGen sg;

    for(int _case = 0 ; _case < 2 ; ++_case){
        bool minimizeEnergy = _case == 0;
        std::cout << "Running test: " << (minimizeEnergy ? "minimizing" : "maximazing") << " energy" << std::endl;

        SimulatedAnnealing<float>::BreakFunction breakFunction = [minimizeEnergy](const float&, double y) -> bool{
            if(minimizeEnergy)
                return y < 55;
            return y > 110;
        };

        std::vector< std::pair<float, double> > history;
        auto registerVisitedState = [&history](const float& state, double energy, bool bestSoFar, bool accepted, bool /*partOfSubIteration*/){
            if(bestSoFar && accepted)
                history.emplace_back( std::make_pair(state, energy) );
        };

        auto result = sa.compute(start, startEnergy, 100, sg, tf, minimizeEnergy, 2, &breakFunction, -1, registerVisitedState );

        BOOST_CHECK(result != nullptr); // "Sim. annealing failed!");
        if(!result)
            continue;

        bool resultIsBetterOrEqual = ( minimizeEnergy ? startEnergy > result->second + 1e-9 : startEnergy < result->second - 1e-9 );
        BOOST_CHECK( resultIsBetterOrEqual ); // "Resulting energy is worse than starting energy");

        double calcEnergy = StateGen::energyFunction(result->first);
        BOOST_CHECK( std::fabs(result->second - calcEnergy) < 1e-9 ); // < 1e-9, "Result energy does not concurr with energy function");

        bool resultIsDifferent = ( std::fabs(startEnergy - result->second) > 1e-3 );
        if(resultIsDifferent){
            BOOST_CHECK(!history.empty() ); // "No history obtained");
            if(history.empty())
                continue;

            BOOST_CHECK( std::fabs(result->first - history.back().first) < 1e-9 ); // "Last history state differs from result state");
            BOOST_CHECK( std::fabs(result->second - history.back().second) < 1e-9 ); //  "Last history energy differs from result energy");

            for(auto& state_pair : history){
                double calcEnergy = StateGen::energyFunction(state_pair.first);
                BOOST_CHECK( std::fabs(state_pair.second - calcEnergy) < 1e-9 ); //  "History energy does not concurr with energy function");

            }
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
